//! Glue between the GPIB interpreter and its front ends:
//!   * SSH sessions (components/ssh_bridge, C / wolfSSH)
//!   * the browser terminal (WebSocket /ws/serial)
//!
//! Replaces the former usb_host.rs (USB CDC host + UART bridge), which is not
//! present on the GPIB-Server board.

use esp_idf_svc::http::server::ws::EspHttpWsDetachedSender;
use embedded_svc::ws::FrameType;
use log::{info, warn};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Mutex;
use std::thread;
use std::time::Duration;

/// Device slot ID of the GPIB controller (must match SSH_DEV_GPIB in ssh_bridge.c).
const DEVICE_GPIB: u8 = 1;

/// Number of SSH console sessions currently attached to the GPIB device.
static ACTIVE_SSH: AtomicU8 = AtomicU8::new(0);
static SSH_BRIDGE_STARTED: AtomicBool = AtomicBool::new(false);

/// Last device selection error reason (NUL-terminated, static storage).
static LAST_SELECT_ERR: Mutex<&'static [u8]> = Mutex::new(b"Unknown device\0");

// ─── SSH bridge FFI ──────────────────────────────────────────────────────────

extern "C" {
    fn ssh_bridge_init(port: u16,
                       username: *const core::ffi::c_char,
                       password: *const core::ffi::c_char) -> i32;
    fn ssh_bridge_cdc_rx(device_id: u8, data: *const u8, len: usize);
}

#[no_mangle]
pub extern "C" fn ssh_log_info(msg: *const core::ffi::c_char) {
    let s = unsafe { std::ffi::CStr::from_ptr(msg) }.to_str().unwrap_or("?");
    log::info!("{}", s);
}

#[no_mangle]
pub extern "C" fn ssh_log_warn(msg: *const core::ffi::c_char) {
    let s = unsafe { std::ffi::CStr::from_ptr(msg) }.to_str().unwrap_or("?");
    log::warn!("{}", s);
}

/// Persist a short diagnostic line to NVS (see boot_log::record_diag).
#[no_mangle]
pub extern "C" fn ssh_record_diag(msg: *const core::ffi::c_char) {
    let s = unsafe { std::ffi::CStr::from_ptr(msg) }.to_str().unwrap_or("?");
    crate::boot_log::record_diag(s);
}

/// Hex dump hook called from the C bridge loop (disabled).
#[no_mangle]
pub extern "C" fn ssh_log_hex(dir: *const core::ffi::c_char, data: *const u8, len: usize) {
    let _ = (dir, data, len);
}

/// SSH client → GPIB interpreter.
extern "C" fn gpib_write_cb(data: *const u8, len: usize) {
    if data.is_null() || len == 0 {
        return;
    }
    let slice = unsafe { core::slice::from_raw_parts(data, len) };
    crate::gpib::check_ctrl_c(slice); // must not wait for the (busy) GPIB thread
    crate::gpib::input(slice);
}

/// Called by ssh_bridge.c when a session runs `gpib` / `console gpib`.
/// "gpib" = interactive (echo + auto read), "gpibraw" = Prologix-style raw.
#[no_mangle]
pub extern "C" fn ssh_select_device(
    name: *const core::ffi::c_char,
) -> Option<extern "C" fn(*const u8, usize)> {
    let name_str = unsafe { std::ffi::CStr::from_ptr(name) }.to_str().unwrap_or("");
    match name_str {
        "gpib" | "gpibraw" => {
            ACTIVE_SSH.fetch_add(1, Ordering::AcqRel);
            crate::gpib::set_interactive(name_str == "gpib");
            info!("[SSH] device selected: {}", name_str);
            Some(gpib_write_cb)
        }
        other => {
            warn!("[SSH] unknown device: '{}'", other);
            *LAST_SELECT_ERR.lock().unwrap() = b"Unknown device. Available: gpib\0";
            None
        }
    }
}

#[no_mangle]
pub extern "C" fn ssh_release_device(name: *const core::ffi::c_char) {
    let name_str = unsafe { std::ffi::CStr::from_ptr(name) }.to_str().unwrap_or("");
    if name_str == "gpib" || name_str == "gpibraw" {
        let _ = ACTIVE_SSH.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |v| Some(v.saturating_sub(1)));
    }
    info!("[SSH] device released: {}", name_str);
}

#[no_mangle]
pub extern "C" fn ssh_device_error() -> *const u8 {
    LAST_SELECT_ERR.lock().unwrap().as_ptr()
}

/// Start the SSH server (port 22).
pub fn start_ssh(username: &'static str, password: &'static str) {
    if SSH_BRIDGE_STARTED.swap(true, Ordering::SeqCst) {
        return;
    }
    info!("[SSH] Initialising SSH bridge...");
    let user_cstr = std::ffi::CString::new(username)
        .unwrap_or_else(|_| std::ffi::CString::new("admin").unwrap());
    let pass_cstr = std::ffi::CString::new(password)
        .unwrap_or_else(|_| std::ffi::CString::new("esp32").unwrap());
    unsafe {
        let rc = ssh_bridge_init(22, user_cstr.as_ptr(), pass_cstr.as_ptr());
        info!("[SSH] SSH bridge init returned rc={}", rc);
        if rc == 0 {
            // Basic services are up — confirm this OTA image so the bootloader
            // does not roll back (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE).
            esp_idf_sys::esp_ota_mark_app_valid_cancel_rollback();
        }
    }
    // The C side keeps the raw pointers; keep the memory alive forever.
    std::mem::forget(user_cstr);
    std::mem::forget(pass_cstr);
}

/// Number of SSH sessions attached to the GPIB controller.
pub fn active_ssh_sessions() -> u8 {
    ACTIVE_SSH.load(Ordering::Relaxed)
}

// ─── Output fan-out (GPIB interpreter → SSH + WebSocket) ──────────────────────

/// Output sink installed into the GPIB interpreter.
pub fn gpib_output(data: &[u8]) {
    if data.is_empty() {
        return;
    }
    if ACTIVE_SSH.load(Ordering::Acquire) > 0 {
        unsafe { ssh_bridge_cdc_rx(DEVICE_GPIB, data.as_ptr(), data.len()); }
    }
    ws_enqueue(data);
}

// ─── WebSocket terminal (browser ⇄ GPIB interpreter) ──────────────────────────

/// Ring buffer capacity for interpreter → browser data.
const WS_RING_CAP: usize = 65536;

static WS_SENDERS: Mutex<Vec<(i32, EspHttpWsDetachedSender)>> = Mutex::new(Vec::new());
static WS_RING: Mutex<VecDeque<u8>> = Mutex::new(VecDeque::new());
static WS_TX: Mutex<VecDeque<u8>> = Mutex::new(VecDeque::new());
static WS_FDS: Mutex<Vec<i32>> = Mutex::new(Vec::new());

/// Register a browser terminal session.
pub fn ws_register_sender(fd: i32, sender: EspHttpWsDetachedSender) {
    WS_SENDERS.lock().unwrap().push((fd, sender));
    if let Ok(mut fds) = WS_FDS.lock() {
        if !fds.contains(&fd) {
            fds.push(fd);
        }
    }
    crate::gpib::set_interactive(true);
    info!("[WS] registered GPIB terminal fd={}", fd);
}

/// Remove a browser terminal session.
pub fn ws_remove_sender(fd: i32) {
    WS_SENDERS.lock().unwrap().retain(|(f, _)| *f != fd);
    if let Ok(mut fds) = WS_FDS.lock() {
        fds.retain(|f| *f != fd);
    }
}

/// True when `fd` is a registered GPIB terminal session.
/// Uses WS_FDS (not WS_SENDERS) so the httpd thread never waits on the
/// sender thread, which may hold WS_SENDERS while blocked in send().
pub fn ws_is_registered(fd: i32) -> bool {
    WS_FDS.lock().map(|f| f.contains(&fd)).unwrap_or(false)
}

/// Browser → GPIB interpreter (non-blocking; drained by the sender thread).
pub fn ws_input(data: &[u8]) {
    crate::gpib::check_ctrl_c(data);
    if let Ok(mut q) = WS_TX.try_lock() {
        for &b in data {
            if q.len() >= WS_RING_CAP {
                q.pop_front();
            }
            q.push_back(b);
        }
    }
}

fn ws_enqueue(data: &[u8]) {
    if let Ok(mut ring) = WS_RING.try_lock() {
        for &b in data {
            if ring.len() >= WS_RING_CAP {
                ring.pop_front();
            }
            ring.push_back(b);
        }
    }
}

// ─── Spectrum view (browser /spectrum ⇄ gpib.rs scope loop) ───────────────────

static SPEC_SENDERS: Mutex<Vec<(i32, EspHttpWsDetachedSender)>> = Mutex::new(Vec::new());
static SPEC_FDS: Mutex<Vec<i32>> = Mutex::new(Vec::new());
/// Pending JSON text messages (meta / error / state), oldest first.
static SPEC_TEXT: Mutex<VecDeque<String>> = Mutex::new(VecDeque::new());
/// Last "meta" message, replayed to newly connected viewers.
static SPEC_LAST_META: Mutex<Option<String>> = Mutex::new(None);
/// Latest trace frame per trace ('A', 'B'); older frames are dropped if the
/// browser is slow. Each frame starts with its tag byte.
/// Lower-screen meta of a split display (WINDOW Zoom / F/T / T/T).
static SPEC_LAST_META2: Mutex<Option<String>> = Mutex::new(None);
/// Latest frame per tag: 'A' trace A, 'B' trace B, 'C' lower screen (split).
static SPEC_FRAME: Mutex<[Option<Vec<u8>>; 3]> = Mutex::new([None, None, None]);

/// Sinks installed into the GPIB scope loop.
pub const SCOPE_SINKS: crate::gpib::ScopeSinks = crate::gpib::ScopeSinks {
    text: spectrum_text,
    frame: spectrum_frame,
    inst: inst_text,
};

fn spectrum_text(msg: &str) {
    if msg.contains(r#""type":"meta""#) {
        *SPEC_LAST_META.lock().unwrap() = Some(msg.to_string());
        if msg.contains(r#""split":false"#) {
            *SPEC_LAST_META2.lock().unwrap() = None;
        }
    } else if msg.contains(r#""type":"meta2""#) {
        *SPEC_LAST_META2.lock().unwrap() = Some(msg.to_string());
    }
    let mut q = SPEC_TEXT.lock().unwrap();
    if q.len() >= 32 {
        q.pop_front();
    }
    q.push_back(msg.to_string());
}

/// Report an error to the spectrum viewers.
pub fn spec_error(msg: &str) {
    let m = msg.replace('\\', "\\\\").replace('"', "\\\"");
    spectrum_text(&format!(r#"{{"type":"error","msg":"{}"}}"#, m));
}

fn spectrum_frame(data: &[u8]) {
    let slot = match data.first() { Some(&b'B') => 1, Some(&b'C') => 2, _ => 0 };
    SPEC_FRAME.lock().unwrap()[slot] = Some(data.to_vec());
}

/// Register a spectrum viewer and send it the current state.
pub fn spec_register_sender(fd: i32, sender: EspHttpWsDetachedSender) {
    let mut fds = SPEC_FDS.lock().unwrap();
    if fds.contains(&fd) {
        return;
    }
    fds.push(fd);
    drop(fds);
    SPEC_SENDERS.lock().unwrap().push((fd, sender));
    // Replay meta + running state (broadcast; harmless for existing viewers).
    // Copy first: spectrum_text() locks SPEC_LAST_META itself, and the guard
    // of an `if let` scrutinee would still be held inside the block (deadlock).
    let last = SPEC_LAST_META.lock().unwrap().clone();
    if let Some(m) = last {
        spectrum_text(&m);
    }
    let last2 = SPEC_LAST_META2.lock().unwrap().clone();
    if let Some(m) = last2 {
        spectrum_text(&m);
    }
    spectrum_text(&format!(r#"{{"type":"state","running":{}}}"#, crate::gpib::scope_running()));
    info!("[WS] registered spectrum viewer fd={}", fd);
}

/// Remove a spectrum viewer; stops the capture when the last one leaves.
pub fn spec_remove_sender(fd: i32) {
    SPEC_SENDERS.lock().unwrap().retain(|(f, _)| *f != fd);
    let mut fds = SPEC_FDS.lock().unwrap();
    let had = fds.contains(&fd);
    fds.retain(|f| *f != fd);
    if had && fds.is_empty() && crate::gpib::scope_running() {
        info!("[WS] last spectrum viewer left — stopping capture");
        crate::gpib::scope_stop();
    }
}

pub fn spec_is_registered(fd: i32) -> bool {
    SPEC_FDS.lock().map(|f| f.contains(&fd)).unwrap_or(false)
}

type SenderList = Mutex<Vec<(i32, EspHttpWsDetachedSender)>>;

/// Send `data` to every sender in `list` WITHOUT holding the list lock while
/// sending: `send()` waits for the httpd task, and httpd handlers lock the same
/// list (register/remove), so holding it would deadlock the web server.
/// Senders removed meanwhile (fd no longer in `fds`) or failed are dropped;
/// senders registered meanwhile are kept.
fn broadcast(list: &SenderList, fds: &Mutex<Vec<i32>>, frame: FrameType, data: &[u8]) {
    let mut taken = core::mem::take(&mut *list.lock().unwrap());
    if taken.is_empty() {
        return;
    }
    let mut dead: Vec<i32> = Vec::new();
    for (fd, sender) in taken.iter_mut() {
        if sender.is_closed() {
            dead.push(*fd);
        } else if let Err(e) = sender.send(frame, data) {
            warn!("[WS] send error fd={}: {:?}", fd, e);
            dead.push(*fd);
        }
    }
    let live: Vec<i32> = fds.lock().unwrap().clone();
    taken.retain(|(fd, _)| !dead.contains(fd) && live.contains(fd));
    let mut cur = list.lock().unwrap();
    taken.append(&mut cur);
    *cur = taken;
}

fn spec_broadcast(frame: FrameType, data: &[u8]) {
    broadcast(&SPEC_SENDERS, &SPEC_FDS, frame, data);
}

fn spec_pump() {
    let texts: Vec<String> = SPEC_TEXT.lock().map(|mut q| q.drain(..).collect()).unwrap_or_default();
    for t in texts {
        spec_broadcast(FrameType::Text(false), t.as_bytes());
    }
    let frames: Vec<Vec<u8>> = SPEC_FRAME.lock()
        .map(|mut f| f.iter_mut().filter_map(|x| x.take()).collect())
        .unwrap_or_default();
    for f in frames {
        spec_broadcast(FrameType::Binary(false), &f);
    }
}

// ─── Instrument pages (browser /psu, /dmm ⇄ gpib.rs inst_request) ───────────

static INST_SENDERS: SenderList = Mutex::new(Vec::new());
static INST_FDS: Mutex<Vec<i32>> = Mutex::new(Vec::new());
/// Pending JSON replies for the instrument pages, oldest first.
static INST_TEXT: Mutex<VecDeque<String>> = Mutex::new(VecDeque::new());

fn inst_text(msg: &str) {
    let mut q = INST_TEXT.lock().unwrap();
    if q.len() >= 32 {
        q.pop_front();
    }
    q.push_back(msg.to_string());
}

/// Report an error (e.g. a rejected command) to the instrument pages.
pub fn inst_error(dev: &str, msg: &str) {
    let m = msg.replace('\\', "\\\\").replace('"', "\\\"");
    inst_text(&format!(r#"{{"type":"error","dev":"{}","msg":"{}"}}"#, dev, m));
}

pub fn inst_register_sender(fd: i32, sender: EspHttpWsDetachedSender) {
    let mut fds = INST_FDS.lock().unwrap();
    if fds.contains(&fd) {
        return;
    }
    fds.push(fd);
    drop(fds);
    INST_SENDERS.lock().unwrap().push((fd, sender));
    info!("[WS] registered instrument page fd={}", fd);
}

pub fn inst_remove_sender(fd: i32) {
    INST_SENDERS.lock().unwrap().retain(|(f, _)| *f != fd);
    INST_FDS.lock().unwrap().retain(|f| *f != fd);
}

pub fn inst_is_registered(fd: i32) -> bool {
    INST_FDS.lock().map(|f| f.contains(&fd)).unwrap_or(false)
}

fn inst_pump() {
    let texts: Vec<String> = INST_TEXT.lock().map(|mut q| q.drain(..).collect()).unwrap_or_default();
    for t in texts {
        broadcast(&INST_SENDERS, &INST_FDS, FrameType::Text(false), t.as_bytes());
    }
}

/// Background thread: interpreter output → browsers, browser input → interpreter.
pub fn start_ws_sender_thread() {
    thread::Builder::new()
        .name("ws_send".into())
        .stack_size(6144)
        .spawn(|| loop {
            let out: Vec<u8> = WS_RING.lock().map(|mut r| r.drain(..).collect()).unwrap_or_default();
            if !out.is_empty() {
                broadcast(&WS_SENDERS, &WS_FDS, FrameType::Binary(false), &out);
            }
            let input: Vec<u8> = WS_TX.lock().map(|mut q| q.drain(..).collect()).unwrap_or_default();
            if !input.is_empty() {
                crate::gpib::input(&input);
            }
            spec_pump();
            inst_pump();
            thread::sleep(Duration::from_millis(5));
        })
        .expect("spawn ws_sender thread");
}
