/* ssh_bridge.c
 *
 * SSH server that bridges an SSH terminal session to a USB CDC serial
 * device.
 *
 * Author:  Hiroshi Nakajima
 * 
 * Version 0.1.0 (2026-04-15 initial release)
 * 
 * Credentials (compile-time defaults):
 *   username : any
 *   password : "esp32"   (change SSH_BRIDGE_PASSWORD as needed)
 *
 * Architecture:
 *   Listener task  →  accept TCP connections
 *   Session task   →  one at a time; handles wolfSSH protocol + data
 *
 *   CDC → SSH :  ssh_bridge_cdc_rx() feeds a ring buffer;
 *                session task drains it via wolfSSH_ChannelIdSend().
 *   SSH → CDC :  session task reads via wolfSSH_ChannelIdRead();
 *                calls the cdc_tx_cb provided at init.
 */

#include "ssh_bridge.h"

/* wolfSSL / wolfCrypt */
#define WOLFSSL_USER_SETTINGS
#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/sha256.h>
#include <wolfssl/wolfcrypt/wc_port.h>

/* wolfSSH */
#include <wolfssh/ssh.h>
#include <wolfssh/certs_test.h>  /* embedded RSA host key buffer */
#include <wolfssh/error.h>
#include <wolfssh/internal.h>    /* WOLFSSH_CHANNEL.eofTxd - see update command's
                                * premature-EOF-echo workaround below */

/* ESP-IDF */
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/idf_additions.h>

/* POSIX sockets */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>  /* malloc / free / realloc for wolfSSL_SetAllocators */

/* ── Configuration ───────────────────────────────────────────────────────── */

#define TAG                 "SSH_BRIDGE"

#include "version_gen.h"  /* SSH_BRIDGE_VERSION, generated from Cargo.toml by build.rs */

#ifndef SSH_BRIDGE_VERSION
#define SSH_BRIDGE_VERSION  "0.0.0"
#endif

#ifndef SSH_BRIDGE_PASSWORD
#define SSH_BRIDGE_PASSWORD "esp32"
#endif

/* Rust-side logging (goes through syslog); declared early so the device-slot
 * helpers below (used from ssh_bridge_cdc_rx / session task) can call it. */
extern void ssh_log_info(const char *msg);
extern void ssh_log_warn(const char *msg);
/* One-shot diagnostic line, retrievable via GET /api/diag_log (NVS-backed,
 * rolling last 10) — temporary aid while there's no serial console access. */
extern void ssh_record_diag(const char *msg);

/* Ring buffer size for CDC → SSH direction.
 * Must be large enough to absorb bursts while the SSH channel window
 * is temporarily full.  USB CDC data arrives at USB bus speed (not
 * baud-rate limited like UART), so commands like `dmesg` via g_serial
 * can dump tens of KB in a single burst.  128 KB in PSRAM is safe. */
#define CDC_RING_SZ  (128 * 1024)

/* wolfSSH session task stack size.
 * wolfSSH_accept() runs RSA key exchange which consumes deep stack;
 * non-blocking wolfSSH_worker() may also recurse.  48 KB is safe. */
#define SSH_SESSION_STACK  (48 * 1024)

/* wolfSSH listener task stack size */
#define SSH_LISTEN_STACK   (6 * 1024)

/* Maximum simultaneous SSH sessions.  Each console session is bound to its
 * own serial device (own ring buffer), so distinct devices can be bridged
 * concurrently; only claiming the *same* device twice is rejected.
 * Non-console commands (reboot/update/usage) are short-lived and also count
 * against this limit while they run. */
#define MAX_CONCURRENT_SESSIONS  6

/* Shell channel ID (first channel opened is always 0) */
#define SHELL_CHANNEL_ID   0

/* ── Per-device → SSH ring buffers ───────────────────────────────────────────
 * Each bridged device (currently only the GPIB controller) can be bridged to up to
 * MAX_SESSIONS_PER_DEVICE simultaneous SSH console sessions.  Each session
 * gets its own ring buffer (allocated lazily from PSRAM when it connects,
 * freed when it disconnects); incoming CDC/UART data is fanned out to every
 * session's ring buffer so all connected clients see the same serial stream.
 * SSH client → device writes from any session go straight to the shared
 * write callback (no fan-out needed there — it's just one physical wire).
 * Device IDs must match the Rust-side DEVICE_* constants. */

#define SSH_DEV_NONE  0
#define SSH_DEV_GPIB  1
#define NUM_DEVICE_SLOTS  2   /* index 0 (SSH_DEV_NONE) unused */

/* Max concurrent SSH console sessions sharing a single serial device. */
#define MAX_SESSIONS_PER_DEVICE  4

typedef struct {
    uint8_t          *ring;       /* NULL when this session slot is free */
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile WOLFSSH  *owner_ssh; /* session bound to this ring (stale probe) */
} ring_slot_t;

typedef struct {
    ring_slot_t rings[MAX_SESSIONS_PER_DEVICE];
} device_slot_t;

static device_slot_t    s_devices[NUM_DEVICE_SLOTS];
static SemaphoreHandle_t s_devtable_mutex = NULL;

/* Map a device name (as accepted by ssh_select_device) to its slot index. */
static int device_id_from_name(const char *name)
{
    if (!name) return SSH_DEV_NONE;
    /* "gpib" = interactive (local echo), "gpibraw" = no echo (scripts) */
    if (strcmp(name, "gpib") == 0 || strcmp(name, "gpibraw") == 0) return SSH_DEV_GPIB;
    return SSH_DEV_NONE;
}

/* Probe whether the wolfSSH session owning a slot has gone away without
 * cleaning up (e.g. task crash).  Returns 1 if the underlying socket looks
 * dead. */
static int ssh_handle_is_dead(WOLFSSH *ssh)
{
    int fd = wolfSSH_get_fd(ssh);
    int probe = fcntl(fd, F_GETFL);
    if (probe < 0 && errno == EBADF) return 1;
    char peek;
    int r = recv(fd, &peek, 1, MSG_PEEK | MSG_DONTWAIT);
    return (r == 0) || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK);
}

/* Count how many sessions currently share device `idx`. */
static int devtable_count(int idx)
{
    if (idx <= SSH_DEV_NONE || idx >= NUM_DEVICE_SLOTS) return 0;
    int n = 0;
    xSemaphoreTake(s_devtable_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_SESSIONS_PER_DEVICE; i++) {
        if (s_devices[idx].rings[i].ring != NULL) n++;
    }
    xSemaphoreGive(s_devtable_mutex);
    return n;
}

/* Claim a free ring-buffer slot for `name`, binding it to `ssh`.  Several
 * sessions may share the same device concurrently (up to
 * MAX_SESSIONS_PER_DEVICE).  Returns NULL if the device name is unknown, the
 * device is already at capacity, or PSRAM allocation fails. */
static ring_slot_t *devtable_acquire(const char *name, WOLFSSH *ssh)
{
    int idx = device_id_from_name(name);
    if (idx == SSH_DEV_NONE) return NULL;

    xSemaphoreTake(s_devtable_mutex, portMAX_DELAY);
    device_slot_t *dev = &s_devices[idx];
    ring_slot_t   *free_slot = NULL;
    for (int i = 0; i < MAX_SESSIONS_PER_DEVICE; i++) {
        ring_slot_t *r = &dev->rings[i];
        if (r->ring != NULL && ssh_handle_is_dead((WOLFSSH *)r->owner_ssh)) {
            ESP_LOGW(TAG, "Stale session on device %d slot %d — force-clearing", idx, i);
            ssh_log_warn("Stale session on device — force-clearing");
            heap_caps_free(r->ring);
            r->ring      = NULL;
            r->owner_ssh = NULL;
        }
        if (r->ring == NULL && free_slot == NULL) {
            free_slot = r;
        }
    }
    if (!free_slot) {
        xSemaphoreGive(s_devtable_mutex);
        return NULL;   /* device already has MAX_SESSIONS_PER_DEVICE sessions */
    }
    uint8_t *ring = (uint8_t *)heap_caps_malloc(CDC_RING_SZ, MALLOC_CAP_SPIRAM);
    if (!ring) {
        xSemaphoreGive(s_devtable_mutex);
        ESP_LOGE(TAG, "PSRAM ring buffer alloc failed for device %d", idx);
        return NULL;
    }
    free_slot->ring      = ring;
    free_slot->head      = 0;
    free_slot->tail      = 0;
    free_slot->owner_ssh = ssh;
    xSemaphoreGive(s_devtable_mutex);
    return free_slot;
}

/* Release a session's ring-buffer slot. */
static void devtable_release(ring_slot_t *slot)
{
    if (!slot) return;
    xSemaphoreTake(s_devtable_mutex, portMAX_DELAY);
    if (slot->ring) {
        heap_caps_free(slot->ring);
        slot->ring = NULL;
    }
    slot->owner_ssh = NULL;
    xSemaphoreGive(s_devtable_mutex);
}

/* Internal: write into one ring buffer.  Caller must hold s_devtable_mutex. */
static void ring_write_locked(ring_slot_t *slot, const uint8_t *data, size_t len)
{
    if (!slot->ring) return;
    for (size_t i = 0; i < len; i++) {
        uint32_t next = (slot->head + 1) % CDC_RING_SZ;
        if (next != slot->tail) {
            slot->ring[slot->head] = data[i];
            slot->head = next;
        }
    }
}

/* Fan out received CDC/UART data to every session currently bridging this
 * device, so all connected SSH clients see the same serial stream. */
static void device_broadcast_write(int device_id, const uint8_t *data, size_t len)
{
    if (device_id <= SSH_DEV_NONE || device_id >= NUM_DEVICE_SLOTS) return;
    device_slot_t *dev = &s_devices[device_id];
    xSemaphoreTake(s_devtable_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_SESSIONS_PER_DEVICE; i++) {
        ring_write_locked(&dev->rings[i], data, len);
    }
    xSemaphoreGive(s_devtable_mutex);
}

/* Drain up to max_len bytes from one session's own ring buffer. */
static size_t ring_read(ring_slot_t *slot, uint8_t *buf, size_t max_len)
{
    if (!slot || !slot->ring) return 0;
    xSemaphoreTake(s_devtable_mutex, portMAX_DELAY);
    size_t n = 0;
    while (slot->tail != slot->head && n < max_len) {
        buf[n++] = slot->ring[slot->tail];
        slot->tail = (slot->tail + 1) % CDC_RING_SZ;
    }
    xSemaphoreGive(s_devtable_mutex);
    return n;
}

/* ── Shared state ────────────────────────────────────────────────────────── */

static uint16_t         s_port     = 22;
static const char      *s_password = NULL;  /* set by ssh_bridge_init */

/* Session tracking; protected by s_ssh_mutex.
 * s_session_count – total active sessions (console + reboot/update/usage).
 * Per-device console ownership is tracked separately in s_devices[]. */
static volatile int      s_session_count = 0;
static SemaphoreHandle_t s_ssh_mutex     = NULL;

/* wolfSSH context (shared across sessions) */
static WOLFSSH_CTX *s_ctx = NULL;

/* ── wolfSSH authentication callback ─────────────────────────────────────── */

static int ws_user_auth(byte authType,
                        WS_UserAuthData *authData,
                        void *ctx)
{
    (void)ctx;

    if (authType != WOLFSSH_USERAUTH_PASSWORD)
        return WOLFSSH_USERAUTH_FAILURE;

    const char *pw  = s_password ? s_password : SSH_BRIDGE_PASSWORD;
    word32      pwz = (word32)strlen(pw);

    if (authData->sf.password.passwordSz == pwz &&
        memcmp(authData->sf.password.password, pw, pwz) == 0) {
        ESP_LOGI(TAG, "SSH auth OK for user '%.*s'",
                 (int)authData->usernameSz, authData->username);
        return WOLFSSH_USERAUTH_SUCCESS;
    }

    ESP_LOGW(TAG, "SSH auth FAILED for user '%.*s'",
             (int)authData->usernameSz, authData->username);
    return WOLFSSH_USERAUTH_FAILURE;
}

/* ── Rust-side logging/device callbacks (goes through syslog) ────────────── */
extern void ssh_log_hex(const char *dir, const uint8_t *data, size_t len);
extern const char *ssh_device_error(void);
extern ssh_cdc_tx_cb_t ssh_select_device(const char *name);
extern void ssh_release_device(const char *name);

/* ── SSH session handler task ────────────────────────────────────────────── */

/* ── Rust-side GPIO / PWM callbacks ──────────────────────────────────────── */

/* Pump the (non-blocking) socket briefly so any response text queued via
 * wolfSSH_ChannelIdSend() (command result/usage/error, etc.) actually
 * reaches the client instead of sitting in wolfSSH's internal output
 * buffer when the session is torn down. Safe to call even if there is
 * nothing pending (returns quickly once wolfSSH_worker() has nothing left
 * to flush). */
static int ssh_flush_pending_output(WOLFSSH *ssh, int client_fd)
{
    int flags = fcntl(client_fd, F_GETFL, 0);
    fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
    int i;
    for (i = 0; i < 30; i++) {
        word32 lastChannel = 0;
        int rc = wolfSSH_worker(ssh, &lastChannel);
        if (rc != WS_SUCCESS && rc != WS_WANT_READ &&
            rc != WS_WANT_WRITE && rc != WS_CHAN_RXD) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return i;
}

/* Reliably send `len` bytes of a one-shot command reply on the shell
 * channel. wolfSSH_ChannelIdSend() only performs a single non-blocking
 * attempt and may return a partial count, 0, or WS_WANT_WRITE if the
 * channel window/socket is momentarily unable to accept more data — the
 * caller is responsible for retrying the remainder. Command handlers that
 * reply right before `goto cleanup`/reboot (i.e. outside of an event loop
 * that already retries pending sends, like the interactive console bridge)
 * used a single fire-and-forget call, which could silently drop the reply
 * (e.g. the OTA "update" command's result message after a large firmware
 * transfer). This helper pumps wolfSSH_worker() between attempts so
 * ACK/WINDOW_ADJUST packets get processed, then retries until the whole
 * message is sent or the retry budget is exhausted. */
static int ssh_send_reliable(WOLFSSH *ssh, const uint8_t *data, size_t len)
{
    size_t sent = 0;
    for (int attempt = 0; attempt < 200 && sent < len; attempt++) {
        int w = wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                       (byte *)(data + sent), (word32)(len - sent));
        if (w > 0) {
            sent += (size_t)w;
            continue;
        }
        if (w == WS_WANT_WRITE || w == 0) {
            word32 lastChannel = 0;
            wolfSSH_worker(ssh, &lastChannel);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        ESP_LOGW(TAG, "ssh_send_reliable: ChannelIdSend err %d (sent %u/%u)",
                 w, (unsigned)sent, (unsigned)len);
        return -1;
    }
    if (sent < len) {
        ESP_LOGW(TAG, "ssh_send_reliable: timed out, sent %u/%u bytes",
                 (unsigned)sent, (unsigned)len);
        return -1;
    }
    return 0;
}

/* ssh_record_diag() does an NVS write, which — like esp_ota_*() — touches
 * flash and requires cache access that a PSRAM-stack task cannot safely
 * provide (see ota_worker_task's comment for the general constraint). The
 * SSH session task's stack lives in PSRAM, so it must never call
 * ssh_record_diag() directly; route it through a short-lived internal-RAM
 * task instead, mirroring the ota_worker_task pattern.
 *
 * Stack sized to match OTA_WORKER_STACK (8KB): the Rust-side NVS path
 * (String/format! allocations + esp-idf-sys FFI call depth) needs more
 * than a minimal few KB — an earlier 3KB attempt caused stack-overflow
 * panics on every call. */
#define DIAG_WORKER_STACK (8 * 1024)

typedef struct {
    char              msg[96];
    SemaphoreHandle_t done;
} diag_worker_ctx_t;

static void diag_worker_task(void *arg)
{
    diag_worker_ctx_t *ctx = (diag_worker_ctx_t *)arg;
    ssh_record_diag(ctx->msg);
    xSemaphoreGive(ctx->done);
    /* Do not self-delete: a task created with xTaskCreateWithCaps() must be
     * torn down via vTaskDeleteWithCaps() from another task. */
    vTaskSuspend(NULL);
}

static void ssh_record_diag_safe(const char *msg)
{
    diag_worker_ctx_t ctx = {0};
    strncpy(ctx.msg, msg, sizeof(ctx.msg) - 1);
    ctx.done = xSemaphoreCreateBinary();
    if (!ctx.done) {
        return;
    }
    TaskHandle_t th = NULL;
    BaseType_t ret = xTaskCreateWithCaps(diag_worker_task, "diag_worker",
                                          DIAG_WORKER_STACK, &ctx, 5, &th,
                                          MALLOC_CAP_INTERNAL);
    if (ret == pdPASS) {
        xSemaphoreTake(ctx.done, pdMS_TO_TICKS(2000));
        vTaskDeleteWithCaps(th);
    }
    vSemaphoreDelete(ctx.done);
}

/* Gracefully close the SSH session (exit-status + channel EOF/close, then
 * flush and shut down the TCP connection) before rebooting the device, so
 * the client sees a clean session/connection close instead of the TCP
 * connection just vanishing when the reset happens. Call this right before
 * esp_restart() from any command that reboots the device. */
static void ssh_close_before_reboot(WOLFSSH *ssh, int client_fd)
{
    int exit_ret = wolfSSH_stream_exit(ssh, 0);

    /* Pump the (non-blocking) socket briefly so the exit-status/EOF/close
     * packets queued above actually reach the client instead of sitting in
     * wolfSSH's internal output buffer. Note: once the channel is closed,
     * wolfSSH_worker() tends to return a non-"keep going" status almost
     * immediately, so this loop alone cannot be relied on to pace the
     * shutdown — see the explicit delay below. */
    int flush_iters = ssh_flush_pending_output(ssh, client_fd);
    {
        char diag[96];
        snprintf(diag, sizeof(diag),
                 "close_before_reboot: exit_ret=%d flush_iters=%d",
                 exit_ret, flush_iters);
        ssh_record_diag_safe(diag);
    }

    /* Give the client's SSH stack a real grace period to receive, decrypt,
     * and *display* the final reply plus the exit-status/EOF/close
     * messages before we rip down the TCP connection. Without this, the
     * FIN can reach the client in (or immediately after) the same burst as
     * the last channel data, and some SSH clients will tear the session
     * down without flushing the last bit of output to the terminal. */
    vTaskDelay(pdMS_TO_TICKS(500));

    wolfSSH_free(ssh);
    shutdown(client_fd, SHUT_RDWR);
    close(client_fd);

    /* Give the client's SSH/TCP stack time to observe the channel close and
     * FIN before the device actually resets. */
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/* ── OTA update helper task ───────────────────────────────────────────────
 * esp_ota_begin()/esp_ota_write()/esp_ota_end() briefly disable both CPU
 * caches while touching flash, which requires the *calling task's own
 * stack* to live in internal RAM — ssh_session_task's stack is deliberately
 * allocated in PSRAM (see MALLOC_CAP_SPIRAM below), so calling esp_ota_*
 * directly from it crashes with
 * "assert failed: ... esp_task_stack_is_sane_cache_disabled()".
 * The actual flash writes are therefore off-loaded to this small dedicated
 * task (internal-RAM stack), fed one chunk at a time through a semaphore
 * handoff from ssh_session_task. The chunk data itself is also bounce-
 * copied into an internal static buffer before esp_ota_write(), since the
 * source buffer must likewise be reachable while caches are disabled. */
#define OTA_WORKER_STACK  (8 * 1024)

typedef struct {
    SemaphoreHandle_t       ready;      /* producer -> worker: chunk (or EOF) ready */
    SemaphoreHandle_t       done;       /* worker -> producer: chunk processed */
    const uint8_t          *data;       /* chunk pointer (may be in PSRAM); ignored if eof */
    size_t                  len;
    int                     eof;        /* 1 = no more data, finalise the update */
    esp_err_t               err;        /* result of the last operation */
    const esp_partition_t  *partition;  /* set once esp_ota_get_next_update_partition() succeeds */
} ota_worker_ctx_t;

static void ota_worker_task(void *arg)
{
    ota_worker_ctx_t *ctx = (ota_worker_ctx_t *)arg;
    /* Internal-RAM bounce buffer (matches the SSH read chunk size below). */
    static uint8_t s_ota_bounce[1024];
    esp_ota_handle_t handle = 0;
    esp_err_t err;
    int failed = 0;

    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    ctx->partition = part;
    if (!part) {
        ctx->err = ESP_ERR_NOT_FOUND;
        xSemaphoreGive(ctx->done);
        vTaskSuspend(NULL);
        return; /* not reached; task torn down by the producer */
    }

    err = esp_ota_begin(part, OTA_SIZE_UNKNOWN, &handle);
    ctx->err = err;
    xSemaphoreGive(ctx->done);
    if (err != ESP_OK) {
        vTaskSuspend(NULL);
        return;
    }

    for (;;) {
        xSemaphoreTake(ctx->ready, portMAX_DELAY);
        if (ctx->eof) {
            break;
        }
        size_t len = ctx->len;
        if (len > sizeof(s_ota_bounce)) len = sizeof(s_ota_bounce); /* safety clamp */
        memcpy(s_ota_bounce, ctx->data, len);
        if (!failed) {
            err = esp_ota_write(handle, s_ota_bounce, len);
            if (err != ESP_OK) failed = 1;
        }
        ctx->err = failed ? err : ESP_OK;
        xSemaphoreGive(ctx->done);
    }

    if (!failed) {
        err = esp_ota_end(handle);
        if (err == ESP_OK) {
            err = esp_ota_set_boot_partition(part);
        }
    } else {
        esp_ota_abort(handle);
        err = ctx->err; /* preserve the original write failure */
    }
    ctx->err = err;
    xSemaphoreGive(ctx->done);

    /* Do not self-delete: a task created with xTaskCreateWithCaps() must be
     * torn down via vTaskDeleteWithCaps() from another task. */
    vTaskSuspend(NULL);
}

static void ssh_session_task(void *arg)
{
    int             client_fd      = (int)(intptr_t)arg;
    char            device_name[32] = {0};     /* filled only for console sessions */
    ssh_cdc_tx_cb_t device_write_cb = NULL;
    ring_slot_t    *devslot         = NULL;     /* this session's ring buffer, if console */

    /* Build wolfSSH session */
    ESP_LOGI(TAG, "heap: total=%u internal=%u psram=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    WOLFSSH *ssh = wolfSSH_new(s_ctx);
    if (!ssh) {
        ESP_LOGE(TAG, "wolfSSH_new failed — internal free=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        xSemaphoreTake(s_ssh_mutex, portMAX_DELAY);
        s_session_count--;
        xSemaphoreGive(s_ssh_mutex);
        close(client_fd);
        vTaskDelete(NULL);
        return;
    }
    wolfSSH_set_fd(ssh, client_fd);

    ESP_LOGI(TAG, "heap after wolfSSH_new: total=%u internal=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    /* SSH handshake (blocking — wolfSSH_accept does not handle EAGAIN) */
    int rc = wolfSSH_accept(ssh);
    if (rc != WS_SUCCESS) {
        ESP_LOGW(TAG, "wolfSSH_accept error: rc=%d err=%d  internal_free=%u",
                 rc, wolfSSH_get_error(ssh),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        goto cleanup;
    }

    /* ── Parse exec command ──────────────────────────────────────────────────
     * Expected forms:
     *   ssh -tt <host> gpib                   – GPIB controller (Prologix-compatible,
     *                                           interactive: local echo)
     *   ssh     <host> gpib raw               – same, without echo (scripts / socat)
     *   ssh -tt <host> console gpib           – alias of "gpib"
     *   ssh <host> reboot                     – Reboot the ESP32-S3
     *   ssh <host> update  < firmware.bin      – OTA firmware update
     *                                            (e.g. cat app.bin | ssh host update)
     * ─────────────────────────────────────────────────────────────────────── */
    char cmd[16] = {0};
    char a1[32]  = {0};
    char a2[32]  = {0};
    {
        const char *exec_cmd = wolfSSH_GetSessionCommand(ssh);
        if (exec_cmd && exec_cmd[0] != '\0') {
            sscanf(exec_cmd, "%15s %31s %31s", cmd, a1, a2);
        }
        ESP_LOGI(TAG, "SSH exec: cmd='%s' a1='%s' a2='%s'", cmd, a1, a2);
    }

    /* ── Dispatch ─────────────────────────────────────────────────────────── */

    if (strcmp(cmd, "console") == 0 || strcmp(cmd, "gpib") == 0) {
        /* ssh -tt <host> gpib [raw]   /   ssh -tt <host> console gpib */
        if (strcmp(cmd, "gpib") == 0) {
            snprintf(device_name, sizeof(device_name), "%s",
                     strcmp(a1, "raw") == 0 ? "gpibraw" : "gpib");
        } else {
            snprintf(device_name, sizeof(device_name), "%s", a1[0] ? a1 : "gpib");
        }

        /* Claim the device.  Several SSH sessions may bridge the same
         * device concurrently (each gets its own ring buffer fed the same
         * data), up to MAX_SESSIONS_PER_DEVICE. */
        device_write_cb = ssh_select_device(device_name);
        if (!device_write_cb) {
            const char *reason = ssh_device_error();
            char err[256];
            snprintf(err, sizeof(err),
                     "Error: %s\r\n"
                     "Usage: ssh -tt admin@host gpib [raw]\r\n",
                     reason);
            wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                   (const byte *)err, (word32)strlen(err));
            goto cleanup;
        }

        /* Allocate/claim a ring buffer slot for this session's CDC/UART → SSH data. */
        devslot = devtable_acquire(device_name, ssh);
        if (!devslot) {
            char err[128];
            snprintf(err, sizeof(err),
                     "Error: too many sessions already connected to this device (max %d).\r\n",
                     MAX_SESSIONS_PER_DEVICE);
            wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                   (const byte *)err, (word32)strlen(err));
            ssh_release_device(device_name);
            device_write_cb = NULL;
            goto cleanup;
        }

        /* Welcome banner (notes other sessions already sharing this device) */
        {
            int shared = devtable_count(device_id_from_name(device_name)) - 1;
            char banner[320];
            if (shared > 0) {
                snprintf(banner, sizeof(banner),
                         "\r\nConnected to ESP32-S3 GPIB-Server v" SSH_BRIDGE_VERSION " [%s].\r\n"
                         "Note: %d other session(s) are also connected to this device — input/output is shared.\r\n"
                         "Type ++help for commands. Disconnect: Enter then ~.\r\n\r\n",
                         device_name, shared);
            } else {
                snprintf(banner, sizeof(banner),
                         "\r\nConnected to ESP32-S3 GPIB-Server v" SSH_BRIDGE_VERSION " [%s].\r\n"
                         "Type ++help for commands. Disconnect: Enter then ~.\r\n\r\n",
                         device_name);
            }
            wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                   (const byte *)banner, (word32)strlen(banner));
        }

        /* Discard any serial data that accumulated before or during connect,
         * so stale output does not flood the terminal on login. */
        {
            uint8_t tmp[256];
            size_t flushed = 0;
            size_t r;
            while ((r = ring_read(devslot, tmp, sizeof(tmp))) > 0) flushed += r;
            // char msg[64];
            // snprintf(msg, sizeof(msg), "[DBG] ring flushed %d bytes on connect", (int)flushed);
            // ssh_log_info(msg);
        }

        /* ── Data bridge loop ──────────────────────────────────────────────────────────
         * select() with 10 ms timeout: runs wolfSSH_worker() only when SSH
         * socket has data, otherwise only drains the serial→SSH ring buffer.
         *
         * Pending-send mechanism: wolfSSH_ChannelIdSend() may return a partial
         * byte count or WS_WANT_WRITE when the SSH channel window is full.
         * We keep the unsent remainder in `pend_buf` and retry on the next
         * iteration *before* reading more from the ring buffer.
         *
         * input_received: serial→SSH forwarding is suppressed until the SSH
         * client sends at least one byte (prevents stale data appearing on
         * connect before the user has interacted with the terminal).
         * ──────────────────────────────────────────────────────────────────── */
        {
            /* Set socket non-blocking so wolfSSH_worker() returns
             * WS_WANT_READ immediately instead of blocking the loop. */
            int flags = fcntl(client_fd, F_GETFL, 0);
            fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);

            uint8_t buf[512];
            word32  lastChannel = 0;
            int     input_received = 0; /* 0 = suppress serial→SSH until first SSH TX */

            /* Pending-send state: data that was ring_read()'d but could not
             * yet be pushed through wolfSSH_ChannelIdSend(). */
            uint8_t pend_buf[512];
            size_t  pend_off = 0;   /* offset of first unsent byte */
            size_t  pend_len = 0;   /* total valid bytes in pend_buf */

            while (1) {
                fd_set rfds, wfds;
                struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
                FD_ZERO(&rfds);
                FD_ZERO(&wfds);
                FD_SET(client_fd, &rfds);
                /* When data is waiting to be sent, also poll for write
                 * readiness so wolfSSH_worker can flush to TCP. */
                if (pend_len > 0)
                    FD_SET(client_fd, &wfds);
                int sel = select(client_fd + 1, &rfds, &wfds, NULL, &tv);
                if (sel < 0) {
                    ESP_LOGI(TAG, "select() error: errno=%d", errno);
                    break;
                }

                /* Always pump the SSH protocol so outgoing data gets
                 * flushed to TCP — not just when the socket is readable.
                 * wolfSSH_worker() handles non-blocking correctly and
                 * returns WS_WANT_READ when there is nothing to receive. */
                rc = wolfSSH_worker(ssh, &lastChannel);
                int wolf_err = wolfSSH_get_error(ssh);

                if (rc == WS_CHAN_RXD && lastChannel == SHELL_CHANNEL_ID) {
                    /* SSH client → device TX.
                     * Drain wolfSSH's internal channel buffer completely:
                     * a single large paste may be buffered internally after
                     * one wolfSSH_worker() call, but wolfSSH_ChannelIdRead()
                     * returns at most sizeof(buf) bytes per call.  Without
                     * this loop the remainder sits in the wolfSSH buffer until
                     * the next WS_CHAN_RXD, so the device never receives the
                     * full paste and the echo never comes back. */
                    int n;
                    while ((n = wolfSSH_ChannelIdRead(ssh, SHELL_CHANNEL_ID,
                                                       buf, sizeof(buf))) > 0) {
                        if (device_write_cb) {
                            device_write_cb(buf, (size_t)n);
                            if (!input_received) {
                                /* Discard any serial data queued before first input */
                                uint8_t discard[256];
                                size_t  dr;
                                while ((dr = ring_read(devslot, discard, sizeof(discard))) > 0)
                                    (void)dr;
                                input_received = 1; /* unlock serial→SSH forwarding */
                            }
                        }
                    }
                    if (n < 0 && n != WS_WANT_READ) {
                        ESP_LOGI(TAG, "ChannelIdRead err %d", n);
                        break;
                    }
                } else if (rc == WS_SUCCESS   ||
                           rc == WS_WANT_READ  ||
                           rc == WS_WANT_WRITE ||
                           rc == WS_CHAN_RXD    ||
                           wolf_err == WS_WANT_READ  ||
                           wolf_err == WS_WANT_WRITE) {
                    /* Normal transient states (non-blocking socket) */
                } else {
                    // { char tmp[96];
                    //   snprintf(tmp, sizeof(tmp), "[DBG] loop break rc=%d err=%d", rc, wolf_err);
                    //   ssh_log_info(tmp); }
                    ESP_LOGI(TAG, "wolfSSH_worker ended rc=%d err=%d",
                             rc, wolf_err);
                    break;
                }

                /* Device RX → SSH client (with retry for partial sends) */

                /* First, flush any pending unsent data from previous iteration */
                while (pend_off < pend_len) {
                    int w = wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                                   pend_buf + pend_off,
                                                   (word32)(pend_len - pend_off));
                    if (w > 0) {
                        pend_off += (size_t)w;
                    } else if (w == WS_WANT_WRITE || w == 0) {
                        break;  /* channel window full — retry next iteration */
                    } else {
                        ESP_LOGI(TAG, "ChannelIdSend err %d", w);
                        goto cleanup;
                    }
                }
                if (pend_off >= pend_len) {
                    pend_off = 0;
                    pend_len = 0;
                }

                /* If nothing pending, read fresh data from the ring buffer.
                 * Discard silently until the SSH client has sent its first
                 * keystroke so the terminal is not flooded on connect. */
                if (pend_len == 0) {
                    size_t serial_n = ring_read(devslot, buf, sizeof(buf));
                    // if (serial_n > 0) {
                    //     char tmp[96];
                    //     snprintf(tmp, sizeof(tmp), "[DBG] ring_read %d bytes input_received=%d [0x%02x]",
                    //              (int)serial_n, input_received, buf[0]);
                    //     ssh_log_info(tmp);
                    // }
                    if (serial_n > 0 && !input_received) {
                        // char tmp[64];
                        // snprintf(tmp, sizeof(tmp), "[DBG] DISCARDED %d bytes (no input yet)", (int)serial_n);
                        // ssh_log_info(tmp);
                        serial_n = 0;
                    }
                    if (serial_n > 0) {
                        int w = wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                                       buf, (word32)serial_n);
                        if (w > 0 && (size_t)w < serial_n) {
                            /* Partial write — save remainder */
                            pend_len = serial_n;
                            pend_off = (size_t)w;
                            memcpy(pend_buf, buf, serial_n);
                        } else if (w == WS_WANT_WRITE || w == 0) {
                            /* Channel window full — save entire chunk */
                            pend_len = serial_n;
                            pend_off = 0;
                            memcpy(pend_buf, buf, serial_n);
                        } else if (w < 0 && w != WS_WANT_WRITE) {
                            ESP_LOGI(TAG, "ChannelIdSend err %d", w);
                            break;
                        }
                    }
                }
            }
        }

    } else if (strcmp(cmd, "reboot") == 0) {
        /* ssh <host> reboot  — reboot the ESP32-S3 */
        const char *resp = "Rebooting...\r\n";
        wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                               (const byte *)resp, (word32)strlen(resp));
        /* Close the SSH session cleanly before resetting. */
        ssh_close_before_reboot(ssh, client_fd);
        esp_restart();
        /* not reached */
        goto cleanup;

    } else if (strcmp(cmd, "update") == 0) {
        /* ssh <host> update  < firmware.bin  — OTA firmware update.
         * Client pipes the app-only image (NOT the merged flash image) into
         * the exec channel's stdin, e.g.:
         *   cat app_<board>.bin | ssh admin@host update
         * Data is streamed straight into the inactive OTA partition; once
         * the client-side pipe reaches EOF the image is finalised, set as
         * the next boot partition, and the device reboots.  On any error
         * the write is aborted and the currently running firmware is left
         * untouched.
         *
         * The actual esp_ota_*() calls run on a dedicated internal-RAM
         * task (ota_worker_task) — see its comment for why: this session
         * task's own stack lives in PSRAM, which flash writes can't
         * tolerate. */
        char resp[128];

        ota_worker_ctx_t ota_ctx = {0};
        ota_ctx.ready = xSemaphoreCreateBinary();
        ota_ctx.done  = xSemaphoreCreateBinary();
        if (!ota_ctx.ready || !ota_ctx.done) {
            const char *err = "Error: failed to allocate OTA sync objects\r\n";
            ssh_send_reliable(ssh, (const uint8_t *)err, strlen(err));
            if (ota_ctx.ready) vSemaphoreDelete(ota_ctx.ready);
            if (ota_ctx.done)  vSemaphoreDelete(ota_ctx.done);
            goto cleanup;
        }

        TaskHandle_t ota_task_handle = NULL;
        BaseType_t task_ret = xTaskCreateWithCaps(
            ota_worker_task, "ota_worker", OTA_WORKER_STACK,
            &ota_ctx, 5, &ota_task_handle, MALLOC_CAP_INTERNAL
        );
        if (task_ret != pdPASS) {
            const char *err = "Error: failed to start OTA worker task\r\n";
            ssh_send_reliable(ssh, (const uint8_t *)err, strlen(err));
            vSemaphoreDelete(ota_ctx.ready);
            vSemaphoreDelete(ota_ctx.done);
            goto cleanup;
        }

        /* Wait for esp_ota_get_next_update_partition()/esp_ota_begin() to
         * complete inside the worker before streaming any data. */
        xSemaphoreTake(ota_ctx.done, portMAX_DELAY);
        if (ota_ctx.err != ESP_OK) {
            snprintf(resp, sizeof(resp),
                     "Error: OTA initialisation failed (%d)\r\n", (int)ota_ctx.err);
            ssh_send_reliable(ssh, (const uint8_t *)resp, strlen(resp));
            vTaskDeleteWithCaps(ota_task_handle);
            vSemaphoreDelete(ota_ctx.ready);
            vSemaphoreDelete(ota_ctx.done);
            goto cleanup;
        }

        ESP_LOGI(TAG, "OTA update starting -> partition '%s' at 0x%x",
                 ota_ctx.partition->label, (unsigned)ota_ctx.partition->address);

        {
            int flags = fcntl(client_fd, F_GETFL, 0);
            fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);

            uint8_t buf[1024];
            word32  lastChannel = 0;
            size_t  total = 0;
            int     ota_write_failed = 0;
            WOLFSSH_CHANNEL *chan =
                wolfSSH_ChannelFind(ssh, SHELL_CHANNEL_ID, WS_CHANNEL_ID_SELF);

            /* Workaround for wolfSSH's DoChannelEof() (internal.c): as soon as
             * it parses the client's own upload-completion CHANNEL_EOF, it
             * unconditionally echoes CHANNEL_EOF straight back to the client
             * (unless channel->eofTxd is already set), *inside* the very
             * wolfSSH_worker() call below that delivers the last upload
             * chunk - i.e. long before we've finished the OTA write and sent
             * our "OTA update OK" reply. OpenSSH clients react to that
             * premature EOF by shutting down their local receive/output side
             * immediately (confirmed via strace: the client's output fd is
             * close()'d ~600ms *before* our reply is even read off the
             * socket), so the real reply that arrives afterward is silently
             * discarded. Force eofTxd=1 up front to suppress that auto-echo
             * for the duration of the upload; it's reset to 0 right after
             * this loop so the real, deferred EOF is sent normally as part
             * of the close sequence (wolfSSH_stream_exit(), called from
             * ssh_close_before_reboot()/cleanup below) *after* our reply. */
            if (chan) chan->eofTxd = 1;

            while (1) {
                fd_set rfds;
                struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
                FD_ZERO(&rfds);
                FD_SET(client_fd, &rfds);
                int sel = select(client_fd + 1, &rfds, NULL, NULL, &tv);
                if (sel < 0) {
                    ESP_LOGI(TAG, "update: select() error errno=%d", errno);
                    break;
                }

                rc = wolfSSH_worker(ssh, &lastChannel);
                int wolf_err = wolfSSH_get_error(ssh);

                if (rc == WS_CHAN_RXD && lastChannel == SHELL_CHANNEL_ID) {
                    int n;
                    while ((n = wolfSSH_ChannelIdRead(ssh, SHELL_CHANNEL_ID,
                                                       buf, sizeof(buf))) > 0) {
                        if (!ota_write_failed) {
                            ota_ctx.data = buf;
                            ota_ctx.len  = (size_t)n;
                            ota_ctx.eof  = 0;
                            xSemaphoreGive(ota_ctx.ready);
                            xSemaphoreTake(ota_ctx.done, portMAX_DELAY);
                            if (ota_ctx.err != ESP_OK) {
                                ESP_LOGW(TAG, "esp_ota_write failed: %d", (int)ota_ctx.err);
                                ota_write_failed = 1;
                            } else {
                                total += (size_t)n;
                            }
                        }
                    }
                    if (n < 0 && n != WS_WANT_READ) {
                        ESP_LOGI(TAG, "update: ChannelIdRead err %d", n);
                        break;
                    }
                } else if (rc == WS_SUCCESS   ||
                           rc == WS_WANT_READ  ||
                           rc == WS_WANT_WRITE ||
                           rc == WS_CHAN_RXD    ||
                           wolf_err == WS_WANT_READ  ||
                           wolf_err == WS_WANT_WRITE) {
                    /* transient, non-blocking socket */
                } else {
                    ESP_LOGI(TAG, "update: wolfSSH_worker ended rc=%d err=%d",
                             rc, wolf_err);
                    break;
                }

                if (chan && wolfSSH_ChannelGetEof(chan)) {
                    ESP_LOGI(TAG, "update: client EOF, total=%u bytes", (unsigned)total);
                    break;
                }
            }

            /* Re-arm eofTxd so whichever close path runs next (the success
             * path's ssh_close_before_reboot(), or the shared `cleanup:`
             * label on error) sends the real, now-deferred CHANNEL_EOF via
             * wolfSSH_stream_exit() at the correct time - after our reply. */
            if (chan) chan->eofTxd = 0;

            /* Signal EOF so the worker finalises (esp_ota_end +
             * esp_ota_set_boot_partition) or aborts, per ota_write_failed. */
            ota_ctx.eof = 1;
            xSemaphoreGive(ota_ctx.ready);
            xSemaphoreTake(ota_ctx.done, portMAX_DELAY);

            const esp_partition_t *update_partition = ota_ctx.partition;
            esp_err_t final_err = ota_ctx.err;

            vTaskDeleteWithCaps(ota_task_handle);
            vSemaphoreDelete(ota_ctx.ready);
            vSemaphoreDelete(ota_ctx.done);

            if (ota_write_failed || total == 0) {
                snprintf(resp, sizeof(resp),
                         total == 0 ? "Error: no data received, OTA aborted\r\n"
                                    : "Error: OTA write failed, aborted\r\n");
                wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                       (const byte *)resp, (word32)strlen(resp));
                goto cleanup;
            }

            if (final_err != ESP_OK) {
                snprintf(resp, sizeof(resp),
                         "Error: OTA finalise failed (%d)\r\n", (int)final_err);
                wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                       (const byte *)resp, (word32)strlen(resp));
                goto cleanup;
            }

            snprintf(resp, sizeof(resp),
                     "OTA update OK: %u bytes written to '%s'. Rebooting...\r\n",
                     (unsigned)total, update_partition->label);
            {
                int send_ret = wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                                       (const byte *)resp, (word32)strlen(resp));
                char diag[96];
                snprintf(diag, sizeof(diag),
                         "update: send ret=%d len=%d wolf_err=%d",
                         send_ret, (int)strlen(resp), wolfSSH_get_error(ssh));
                ssh_record_diag_safe(diag);
            }
            ESP_LOGI(TAG, "OTA update OK, %u bytes -> '%s', rebooting",
                     (unsigned)total, update_partition->label);
            /* Flush the message onto the wire and give the client a grace
             * period to actually receive+display it *before* we start
             * sending the exit-status/EOF/channel-close packets. Without
             * this, the close-related packets tend to reach the client in
             * (or immediately after) the same read() as the data, and
             * OpenSSH processing the session teardown can end up not
             * displaying data that hasn't been flushed to the terminal
             * yet. (See ssh_close_before_reboot(), which already delays
             * *its own* socket teardown but sends the close packets
             * immediately — this extra pre-delay targets the client-side
             * race specifically, which only seems to bite after a large
             * preceding data transfer like this OTA write.) */
            int flush_iters_pre = ssh_flush_pending_output(ssh, client_fd);
            vTaskDelay(pdMS_TO_TICKS(300));
            {
                char diag[96];
                snprintf(diag, sizeof(diag),
                         "update: pre-close flush_iters=%d", flush_iters_pre);
                ssh_record_diag_safe(diag);
            }
            /* Close the SSH session cleanly before resetting. */
            ssh_close_before_reboot(ssh, client_fd);
            esp_restart();
            /* not reached */
        }
        goto cleanup;

    } else {
        /* No command or unknown command → show usage */
        const char *usage =
            "\r\nESP32-S3 GPIB-Server v" SSH_BRIDGE_VERSION "\r\n"
            "Usage:\r\n"
            "  ssh -tt admin@host gpib                -- GPIB controller (Prologix-compatible, echo)\r\n"
            "  ssh    admin@host gpib raw            -- GPIB controller without echo (scripts)\r\n"
            "  ssh    admin@host reboot              -- Reboot the ESP32-S3\r\n"
            "  ssh    admin@host update < firmware.bin -- OTA firmware update\r\n"
            "\r\n";
        wolfSSH_ChannelIdSend(ssh, SHELL_CHANNEL_ID,
                               (const byte *)usage, (word32)strlen(usage));
        goto cleanup;
    }

cleanup:
    /* Send the exit-status/eow/eof/channel-close SSH protocol messages
     * (same as ssh_close_before_reboot(), minus the reboot delay) before
     * flushing and tearing down the raw socket. Without this, the client
     * never sees a clean channel close and may treat the abrupt TCP
     * teardown as an unexpected disconnect ("Connection closed by remote
     * host") — discarding any already-buffered channel data (the command
     * reply) instead of flushing it to the terminal. */
    wolfSSH_stream_exit(ssh, 0);

    /* Flush any response text queued above via wolfSSH_ChannelIdSend()
     * (command result/usage/error message) so it actually reaches the
     * client before the session/socket is torn down. Without this, replies
     * for commands that just `goto cleanup` right after sending a message
     * (update-error/usage, etc.) could still be sitting
     * unsent in wolfSSH's internal buffer and silently get dropped. */
    ssh_flush_pending_output(ssh, client_fd);
    ESP_LOGI(TAG, "SSH session closed [%s]", device_name[0] ? device_name : "-");
    if (devslot) {
        devtable_release(devslot);
    }
    if (device_write_cb) {
        ssh_release_device(device_name);
    }
    xSemaphoreTake(s_ssh_mutex, portMAX_DELAY);
    wolfSSH_free(ssh);
    s_session_count--;
    xSemaphoreGive(s_ssh_mutex);

    shutdown(client_fd, SHUT_RDWR);
    close(client_fd);
    vTaskDelete(NULL);
}

/* ── SSH listener task ───────────────────────────────────────────────────── */

static void ssh_listen_task(void *arg)
{
    (void)arg;

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    /* Allow address reuse so we can restart quickly */
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(s_port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_fd, MAX_CONCURRENT_SESSIONS) < 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "SSH server listening on port %u", s_port);
    {
        char tmp[64];
        snprintf(tmp, sizeof(tmp), "SSH server listening on port %u", s_port);
        ssh_log_info(tmp);
    }

    while (1) {
        struct sockaddr_in peer_addr;
        socklen_t peer_len = sizeof(peer_addr);
        int client_fd = accept(listen_fd,
                               (struct sockaddr *)&peer_addr, &peer_len);
        if (client_fd < 0) {
            ESP_LOGW(TAG, "accept() failed: %d", errno);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Incoming SSH connection from %s", inet_ntoa(peer_addr.sin_addr));
        {
            char tmp[80];
            snprintf(tmp, sizeof(tmp), "Incoming SSH from %s", inet_ntoa(peer_addr.sin_addr));
            ssh_log_info(tmp);
        }

        /* Check overall session limit.  Per-device staleness (a console task
         * that crashed without releasing its device slot) is detected lazily
         * in devtable_acquire() when the device is next claimed. */
        xSemaphoreTake(s_ssh_mutex, portMAX_DELAY);
        int full = (s_session_count >= MAX_CONCURRENT_SESSIONS);
        xSemaphoreGive(s_ssh_mutex);

        if (full) {
            ESP_LOGW(TAG, "Max sessions (%d) reached — rejecting", MAX_CONCURRENT_SESSIONS);
            ssh_log_warn("Rejecting: max concurrent sessions reached");
            close(client_fd);
            continue;
        }

        /* Log free heap before creating 48KB session task */
        {
            char tmp[80];
            snprintf(tmp, sizeof(tmp), "Free heap before session task: %u",
                     (unsigned)esp_get_free_heap_size());
            ssh_log_info(tmp);
        }

        /* Spawn a session handler task (stack in PSRAM — internal SRAM is
         * too scarce when the USB host driver is active). */
        BaseType_t ret = xTaskCreateWithCaps(
            ssh_session_task,
            "ssh_session",
            SSH_SESSION_STACK,
            (void *)(intptr_t)client_fd,
            5,
            NULL,
            MALLOC_CAP_SPIRAM
        );
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "xTaskCreate(ssh_session) failed");
            ssh_log_warn("xTaskCreate(ssh_session) FAILED — not enough memory?");
            close(client_fd);
        } else {
            xSemaphoreTake(s_ssh_mutex, portMAX_DELAY);
            s_session_count++;
            xSemaphoreGive(s_ssh_mutex);
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void ssh_bridge_cdc_rx(uint8_t device_id, const uint8_t *data, size_t len)
{
    device_broadcast_write((int)device_id, data, len);
}

int ssh_bridge_init(uint16_t port,
                    const char *username, const char *password)
{
    s_port     = port;
    s_password = password;  /* pointer must remain valid for program lifetime */

    ESP_LOGI(TAG, "SSH bridge: user='%s'", username ? username : "(any)");
    /* Create synchronisation primitives */
    s_devtable_mutex = xSemaphoreCreateMutex();
    s_ssh_mutex      = xSemaphoreCreateMutex();
    if (!s_devtable_mutex || !s_ssh_mutex) {
        ESP_LOGE(TAG, "Mutex creation failed");
        return -1;
    }

    /* Per-device ring buffers are allocated lazily from PSRAM when a console
     * session claims that device (see devtable_acquire()). */

    /* Route wolfSSL/wolfCrypt heap allocations through stdlib malloc so that
     * ESP-IDF's SPIRAM_USE_MALLOC fallback is active.  By default wolfSSL
     * uses pvPortMalloc() (FreeRTOS internal-SRAM only) when FREERTOS is
     * defined, which starves the SSH handshake on systems with limited
     * internal SRAM. */
    wolfSSL_SetAllocators(malloc, free, realloc);

    /* Initialise wolfSSH */
    if (wolfSSH_Init() != WS_SUCCESS) {
        ESP_LOGE(TAG, "wolfSSH_Init() failed");
        return -2;
    }

    /* Create wolfSSH server context */
    s_ctx = wolfSSH_CTX_new(WOLFSSH_ENDPOINT_SERVER, NULL);
    if (!s_ctx) {
        ESP_LOGE(TAG, "wolfSSH_CTX_new() failed");
        return -3;
    }

    wolfSSH_SetUserAuth(s_ctx, ws_user_auth);
    wolfSSH_CTX_SetBanner(s_ctx, "ESP32-S3 GPIB-Server v" SSH_BRIDGE_VERSION "\r\n");

    /* Load embedded ECC-256 host key (much less heap than RSA 2048 during handshake) */
    int rc = wolfSSH_CTX_UsePrivateKey_buffer(
        s_ctx,
        (const byte *)ecc_key_der_256_ssh,
        (word32)sizeof_ecc_key_der_256_ssh,
        WOLFSSH_FORMAT_ASN1
    );
    if (rc != WS_SUCCESS) {
        ESP_LOGE(TAG, "UsePrivateKey_buffer(ECC-256) failed: %d", rc);
        wolfSSH_CTX_free(s_ctx);
        s_ctx = NULL;
        return -4;
    }

    ESP_LOGI(TAG, "wolfSSH context ready (ECC-256 host key loaded)");

    /* Start listener task (stack in PSRAM) */
    BaseType_t ret = xTaskCreateWithCaps(
        ssh_listen_task,
        "ssh_listen",
        SSH_LISTEN_STACK,
        NULL,
        4,
        NULL,
        MALLOC_CAP_SPIRAM
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(ssh_listen) failed");
        return -5;
    }

    return 0;  /* success */
}
