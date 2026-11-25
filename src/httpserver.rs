// HTTP Config Server for GPIB-Server
// Web UI for device configuration with NVS persistence
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Hiroshi Nakajima

use log::*;
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use esp_idf_svc::io::{Read, Write};
use embedded_svc::http::Headers;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::http::Method;
use esp_idf_sys::*;
use std::ffi::CString;

// ─── App version ─────────────────────────────────────────────────────────────

const APP_VERSION: &str = env!("CARGO_PKG_VERSION");

// ─── NVS namespace ────────────────────────────────────────────────────────────

const NVS_NS: &str = "usbotg_cfg";

// ─── Config struct ────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct NvsConfig {
    pub wifi_ssid:     String,
    pub wifi_psk:      String,
    pub ip_mode:       String,
    pub ip_address:    String,
    pub subnet_mask:   String,
    pub gateway:       String,
    pub dns:           String,
    pub syslog_server: String,
    pub syslog_enable: String,
    pub syslog_host_name: String,
    pub syslog_app_name: String,
    pub ssh_user:      String,
    pub ssh_password:  String,
    pub mini_display_enable: String,
    pub wps_enable:    String,
    pub ntp_server1:   String,
    pub ntp_server2:   String,
    pub ntp_server3:   String,
    pub ntp_server4:   String,
    /// Power-up primary address of the terminal / SSH interpreter (++addr).
    pub gpib_addr:     String,
    /// Instrument addresses used by the /spectrum, /psu and /dmm pages.
    pub sa_addr:       String,
    pub psu_addr:      String,
    pub dmm_addr:      String,
}

/// Normalise a GPIB primary address string (0-30); anything else → `default`.
pub fn norm_addr(v: &str, default: &str) -> String {
    match v.trim().parse::<u8>() {
        Ok(a) if a <= 30 => a.to_string(),
        _ => default.to_string(),
    }
}

// ─── NVS helpers ─────────────────────────────────────────────────────────────

fn nvs_read(key: &str) -> Option<String> {
    let ns = CString::new(NVS_NS).ok()?;
    let k  = CString::new(key).ok()?;
    unsafe {
        let mut handle: nvs_handle_t = 0;
        if nvs_open(ns.as_ptr(), nvs_open_mode_t_NVS_READONLY, &mut handle) != ESP_OK {
            return None;
        }
        // Get required length (pass null for out_value)
        let mut len: usize = 0;
        let ret = nvs_get_str(handle, k.as_ptr(), core::ptr::null_mut::<u8>(), &mut len);
        if ret != ESP_OK || len == 0 {
            nvs_close(handle);
            return None;
        }
        let mut buf = vec![0u8; len];
        let ret = nvs_get_str(handle, k.as_ptr(), buf.as_mut_ptr() as *mut u8, &mut len);
        nvs_close(handle);
        if ret != ESP_OK {
            return None;
        }
        while buf.last() == Some(&0) {
            buf.pop();
        }
        String::from_utf8(buf).ok()
    }
}

fn nvs_write_all(cfg: &NvsConfig) -> bool {
    let ns = match CString::new(NVS_NS) {
        Ok(s) => s,
        Err(_) => return false,
    };
    unsafe {
        let mut handle: nvs_handle_t = 0;
        if nvs_open(ns.as_ptr(), nvs_open_mode_t_NVS_READWRITE, &mut handle) != ESP_OK {
            return false;
        }
        let pairs: &[(&str, &str)] = &[
            ("wifi_ssid",     &cfg.wifi_ssid),
            ("wifi_psk",      &cfg.wifi_psk),
            ("ip_mode",       &cfg.ip_mode),
            ("ip_address",    &cfg.ip_address),
            ("subnet_mask",   &cfg.subnet_mask),
            ("gateway",       &cfg.gateway),
            ("dns",           &cfg.dns),
            ("syslog_server", &cfg.syslog_server),
            ("syslog_enable", &cfg.syslog_enable),
            ("syslog_hostname",    &cfg.syslog_host_name),
            ("syslog_app_name", &cfg.syslog_app_name),
            ("ssh_user",      &cfg.ssh_user),
            ("ssh_password",  &cfg.ssh_password),
            ("wps_enable",    &cfg.wps_enable),
            ("ntp_server1",   &cfg.ntp_server1),
            ("ntp_server2",   &cfg.ntp_server2),
            ("ntp_server3",   &cfg.ntp_server3),
            ("ntp_server4",   &cfg.ntp_server4),
            ("gpib_addr",     &cfg.gpib_addr),
            ("sa_addr",       &cfg.sa_addr),
            ("psu_addr",      &cfg.psu_addr),
            ("dmm_addr",      &cfg.dmm_addr),
        ];
        let mut ok = true;
        for (key, val) in pairs {
            match (CString::new(*key), CString::new(*val)) {
                (Ok(k), Ok(v)) => {
                    if nvs_set_str(handle, k.as_ptr(), v.as_ptr()) != ESP_OK {
                        warn!("NVS: failed to write key '{}'", key);
                        ok = false;
                    }
                }
                _ => {
                    warn!("NVS: CString conversion failed for key '{}'", key);
                    ok = false;
                }
            }
        }
        nvs_commit(handle);
        nvs_close(handle);
        ok
    }
}

fn nvs_erase_namespace() -> bool {
    let ns = match CString::new(NVS_NS) {
        Ok(s) => s,
        Err(_) => return false,
    };
    unsafe {
        let mut handle: nvs_handle_t = 0;
        if nvs_open(ns.as_ptr(), nvs_open_mode_t_NVS_READWRITE, &mut handle) != ESP_OK {
            return false;
        }
        let ok = nvs_erase_all(handle) == ESP_OK;
        nvs_commit(handle);
        nvs_close(handle);
        ok
    }
}

/// Write a single string key to NVS (used when an instrument page changes its address).
fn nvs_write_key(key: &str, val: &str) -> bool {
    let (ns, k, v) = match (CString::new(NVS_NS), CString::new(key), CString::new(val)) {
        (Ok(a), Ok(b), Ok(c)) => (a, b, c),
        _ => return false,
    };
    unsafe {
        let mut handle: nvs_handle_t = 0;
        if nvs_open(ns.as_ptr(), nvs_open_mode_t_NVS_READWRITE, &mut handle) != ESP_OK {
            return false;
        }
        let ok = nvs_set_str(handle, k.as_ptr(), v.as_ptr()) == ESP_OK && nvs_commit(handle) == ESP_OK;
        nvs_close(handle);
        ok
    }
}

/// Write only wifi_ssid and wifi_psk to NVS (called after a successful WPS handshake).
pub fn nvs_write_wifi_creds(ssid: &str, psk: &str) -> bool {
    let ns = match CString::new(NVS_NS) {
        Ok(s) => s,
        Err(_) => return false,
    };
    unsafe {
        let mut handle: nvs_handle_t = 0;
        if nvs_open(ns.as_ptr(), nvs_open_mode_t_NVS_READWRITE, &mut handle) != ESP_OK {
            return false;
        }
        let mut ok = true;
        for (key, val) in [("wifi_ssid", ssid), ("wifi_psk", psk)] {
            match (CString::new(key), CString::new(val)) {
                (Ok(k), Ok(v)) => {
                    if nvs_set_str(handle, k.as_ptr(), v.as_ptr()) != ESP_OK {
                        warn!("NVS: failed to write key '{}'", key);
                        ok = false;
                    }
                }
                _ => { ok = false; }
            }
        }
        nvs_commit(handle);
        nvs_close(handle);
        ok
    }
}

/// Erase all NVS settings and restart the device (returns to cfg.toml defaults on next boot).
pub fn factory_reset() -> ! {
    info!("Factory reset: erasing NVS namespace '{}'...", NVS_NS);
    nvs_erase_namespace();
    info!("NVS erased. Restarting device...");
    unsafe { esp_idf_sys::esp_restart() }
}

/// Load config from NVS, falling back to the provided defaults for any missing keys.
pub fn load_config(defaults: NvsConfig) -> NvsConfig {
    fn or(nvs_val: Option<String>, default: &str) -> String {
        match nvs_val {
            Some(s) if !s.is_empty() => s,
            _ => default.to_string(),
        }
    }
    info!("Loading config from NVS (namespace: {})", NVS_NS);
    NvsConfig {
        wifi_ssid:     or(nvs_read("wifi_ssid"),     &defaults.wifi_ssid),
        wifi_psk:      or(nvs_read("wifi_psk"),      &defaults.wifi_psk),
        ip_mode:       or(nvs_read("ip_mode"),       &defaults.ip_mode),
        ip_address:    or(nvs_read("ip_address"),    &defaults.ip_address),
        subnet_mask:   or(nvs_read("subnet_mask"),   &defaults.subnet_mask),
        gateway:       or(nvs_read("gateway"),       &defaults.gateway),
        dns:           or(nvs_read("dns"),           &defaults.dns),
        syslog_server: or(nvs_read("syslog_server"), &defaults.syslog_server),
        syslog_enable: or(nvs_read("syslog_enable"), &defaults.syslog_enable),
        syslog_host_name: or(nvs_read("syslog_hostname"), &defaults.syslog_host_name),
        syslog_app_name: or(nvs_read("syslog_app_name"), &defaults.syslog_app_name),
        ssh_user:      or(nvs_read("ssh_user"),      &defaults.ssh_user),
        ssh_password:  or(nvs_read("ssh_password"),  &defaults.ssh_password),
        mini_display_enable: or(nvs_read("mini_display_enable"), &defaults.mini_display_enable),
        wps_enable:    or(nvs_read("wps_enable"),    &defaults.wps_enable),
        ntp_server1:   or(nvs_read("ntp_server1"),   &defaults.ntp_server1),
        ntp_server2:   or(nvs_read("ntp_server2"),   &defaults.ntp_server2),
        ntp_server3:   or(nvs_read("ntp_server3"),   &defaults.ntp_server3),
        ntp_server4:   or(nvs_read("ntp_server4"),   &defaults.ntp_server4),
        gpib_addr:     norm_addr(&or(nvs_read("gpib_addr"), &defaults.gpib_addr), &defaults.gpib_addr),
        sa_addr:       norm_addr(&or(nvs_read("sa_addr"),   &defaults.sa_addr),   &defaults.sa_addr),
        psu_addr:      norm_addr(&or(nvs_read("psu_addr"),  &defaults.psu_addr),  &defaults.psu_addr),
        dmm_addr:      norm_addr(&or(nvs_read("dmm_addr"),  &defaults.dmm_addr),  &defaults.dmm_addr),
    }
}

// ─── Session management ───────────────────────────────────────────────────────

fn generate_token() -> String {
    (0..4)
        .map(|_| format!("{:08x}", unsafe { esp_random() }))
        .collect()
}

fn get_session_from_cookie_header(cookie_header: Option<&str>) -> Option<String> {
    let h = cookie_header?;
    for part in h.split(';') {
        let part = part.trim();
        if let Some(v) = part.strip_prefix("session=") {
            return Some(v.to_string());
        }
    }
    None
}

fn is_auth(cookie_header: Option<&str>, session: &Arc<Mutex<Option<String>>>) -> bool {
    let guard = session.lock().unwrap();
    if let Some(token) = guard.as_ref() {
        return get_session_from_cookie_header(cookie_header)
            .as_deref() == Some(token.as_str());
    }
    false
}

// ─── URL decode / form body parse ────────────────────────────────────────────

fn url_decode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'+' => {
                out.push(' ');
                i += 1;
            }
            b'%' if i + 2 < b.len() => {
                let hex = core::str::from_utf8(&b[i + 1..i + 3]).unwrap_or("");
                if let Ok(c) = u8::from_str_radix(hex, 16) {
                    out.push(c as char);
                    i += 3;
                    continue;
                }
                out.push('%');
                i += 1;
            }
            c => {
                out.push(c as char);
                i += 1;
            }
        }
    }
    out
}

fn parse_form(body: &str) -> HashMap<String, String> {
    let mut map = HashMap::new();
    for pair in body.split('&') {
        if let Some(eq) = pair.find('=') {
            map.insert(url_decode(&pair[..eq]), url_decode(&pair[eq + 1..]));
        }
    }
    map
}

// ─── Shared state ─────────────────────────────────────────────────────────────

/// Runtime status shared between the main loop and the HTTP server.
#[derive(Clone)]
pub struct StatusInfo {
    pub ip_address:     Arc<Mutex<String>>,
    pub ssid:           Arc<Mutex<String>>,
    pub rssi:           Arc<Mutex<i32>>,
    pub chip_temp:      Arc<Mutex<f32>>,
}

impl StatusInfo {
    pub fn new() -> Self {
        Self {
            ip_address:     Arc::new(Mutex::new("--".to_string())),
            ssid:           Arc::new(Mutex::new("--".to_string())),
            rssi:           Arc::new(Mutex::new(0)),
            chip_temp:      Arc::new(Mutex::new(0.0)),
        }
    }
    pub fn set_wifi(&self, ip: &str, ssid: &str, rssi: i32) {
        *self.ip_address.lock().unwrap() = ip.to_string();
        *self.ssid.lock().unwrap()       = ssid.to_string();
        *self.rssi.lock().unwrap()       = rssi;
    }
    pub fn set_rssi(&self, rssi: i32) {
        *self.rssi.lock().unwrap() = rssi;
    }
    pub fn set_chip_temp(&self, temp: f32) {
        *self.chip_temp.lock().unwrap() = temp;
    }
}

#[derive(Clone)]
pub struct ConfigState {
    pub config:   Arc<Mutex<NvsConfig>>,
    pub defaults: Arc<NvsConfig>,
    pub status:   StatusInfo,
    session:      Arc<Mutex<Option<String>>>,
}

impl ConfigState {
    pub fn new(config: NvsConfig, defaults: NvsConfig, status: StatusInfo) -> Self {
        Self {
            config:   Arc::new(Mutex::new(config)),
            defaults: Arc::new(defaults),
            status,
            session:  Arc::new(Mutex::new(None)),
        }
    }
}

// ─── HTML helpers ─────────────────────────────────────────────────────────────

fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('"', "&quot;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

#[allow(dead_code)]
fn sel(current: &str, value: &str) -> &'static str {
    if current == value { "selected" } else { "" }
}

fn chk(current: &str, value: &str) -> &'static str {
    if current == value { "checked" } else { "" }
}

// ─── Login page ───────────────────────────────────────────────────────────────

const LOGIN_HTML: &str = concat!(
    r#"<!DOCTYPE html><html lang="ja"><head>"#,
    r#"<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">"#,
    r#"<title data-ja="GPIB-Server ログイン">GPIB-Server Login</title><style>"#,
    r#"*{box-sizing:border-box}"#,
    r#"body{font-family:Arial,sans-serif;background:#1a1a2e;display:flex;justify-content:center;"#,
    r#"align-items:center;height:100vh;margin:0}"#,
    r#".box{background:#16213e;padding:40px;border-radius:12px;"#,
    r#"box-shadow:0 4px 20px rgba(0,0,0,.5);width:320px}"#,
    r#"h1{color:#e94560;text-align:center;margin:0 0 24px;font-size:1.4em}"#,
    r#"label{color:#a8b2d8;font-size:.9em;display:block;margin:12px 0 4px}"#,
    r#"input{width:100%;padding:10px;border:1px solid #304070;border-radius:6px;"#,
    r#"background:#0f3460;color:#e2e8f0;font-size:1em}"#,
    r#"input:focus{outline:none;border-color:#e94560}"#,
    r#"button{width:100%;margin-top:20px;padding:12px;background:#e94560;color:#fff;"#,
    r#"border:none;border-radius:6px;font-size:1em;font-weight:bold;cursor:pointer}"#,
    r#"button:hover{background:#c73652}"#,
    r#"#err{color:#f87171;text-align:center;margin-top:12px;font-size:.9em;display:none}"#,
    r#"</style></head><body>"#,
    r#"<div class="box"><div id="lang-slot" style="text-align:right;margin:-20px -20px 8px 0"></div><h1 data-ja="GPIB-Server 設定">GPIB-Server Config</h1>"#,
    r#"<form method="POST" action="/login">"#,
    r#"<label data-ja="ユーザー名">Username</label>"#,
    r#"<input type="text" name="username" autocomplete="username" required>"#,
    r#"<label data-ja="パスワード">Password</label>"#,
    r#"<input type="password" name="password" autocomplete="current-password" required>"#,
    r#"<button type="submit" data-ja="ログイン">Login</button>"#,
    r#"</form>"#,
    r#"<div id="err" data-ja="ユーザー名またはパスワードが正しくありません">Username or password is incorrect</div></div>"#,
    r#"<script>if(location.search.includes('error=1'))"#,
    r#"document.getElementById('err').style.display='block';</script>"#,
    r#"</body></html>"#
);

// ─── Config page builder ──────────────────────────────────────────────────────

fn build_config_html(cfg: &NvsConfig) -> String {
    format!(
        r#"<!DOCTYPE html><html lang="ja"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title data-ja="GPIB-Server 設定">GPIB-Server Config</title>
<style>
*{{box-sizing:border-box;margin:0;padding:0}}
body{{font-family:'Segoe UI',Arial,sans-serif;background:#111827;color:#e2e8f0;min-height:100vh}}
.page{{max-width:780px;margin:0 auto;padding:16px}}
h1{{color:#f87171;text-align:center;font-size:1.4em;padding:18px 0 14px;letter-spacing:.05em}}
/* Section headers with icon strip */
.section-title{{display:flex;align-items:center;gap:8px;color:#93c5fd;font-size:.82em;
  font-weight:700;text-transform:uppercase;letter-spacing:.08em;
  margin:0 0 12px;padding-bottom:6px;border-bottom:1px solid #1e3a5f}}
/* Cards */
.card{{background:#1e2d45;border-radius:10px;padding:16px 18px;margin-bottom:10px;
  border:1px solid #243b55;box-shadow:0 2px 8px rgba(0,0,0,.35)}}
/* Grid: 2-col on wide, 1-col on narrow */
.grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:10px 14px;margin-bottom:4px}}
.grid-3{{display:grid;grid-template-columns:repeat(3,1fr);gap:10px 14px}}
@media(max-width:520px){{.grid-3{{grid-template-columns:1fr 1fr}}}}
/* Field */
.field label{{font-size:.76em;font-weight:600;color:#94a3b8;display:block;
  margin-bottom:4px;letter-spacing:.03em}}
.field input,.field select{{
  width:100%;padding:8px 10px;background:#0f172a;border:1px solid #2d4a6a;
  border-radius:6px;color:#e2e8f0;font-size:.9em;transition:border-color .2s}}
.field input:focus,.field select:focus{{outline:none;border-color:#60a5fa;
  box-shadow:0 0 0 2px rgba(96,165,250,.15)}}
/* Radio toggle group */
.toggle-group{{display:inline-flex;background:#0f172a;border:1px solid #2d4a6a;
  border-radius:8px;overflow:hidden;margin-bottom:10px}}
.toggle-group label{{padding:7px 18px;font-size:.85em;font-weight:600;cursor:pointer;
  color:#94a3b8;transition:background .2s,color .2s;user-select:none}}
.toggle-group input[type=radio]{{display:none}}
.toggle-group input[type=radio]:checked+span{{background:#1d4ed8;color:#fff;
  border-radius:7px;display:block;padding:7px 18px;margin:-7px -18px}}
/* Indent block shown/hidden based on IP mode */
.indent{{padding-left:4px;border-left:2px solid #1d4ed8;margin-top:6px}}
/* Status rows */
.stat-grid{{display:grid;grid-template-columns:auto 1fr;gap:6px 16px;align-items:center}}
.stat-key{{font-size:.78em;font-weight:600;color:#64748b;text-transform:uppercase;
  letter-spacing:.06em;white-space:nowrap}}
.stat-val{{font-size:.9em;color:#7dd3fc;font-weight:600}}
.badge{{display:inline-block;padding:2px 9px;border-radius:20px;font-size:.8em;font-weight:700}}
.badge-ok{{background:#14532d;color:#4ade80;border:1px solid #166534}}
.badge-warn{{background:#451a03;color:#fb923c;border:1px solid #7c2d12}}
.badge-err{{background:#4c0519;color:#fb7185;border:1px solid #881337}}
/* Message bar */
#msg{{padding:10px 14px;border-radius:7px;margin-bottom:10px;font-size:.88em;display:none;
  border-left:4px solid transparent}}
#msg.ok{{background:#052e16;border-color:#16a34a;color:#4ade80}}
#msg.err{{background:#350005;border-color:#dc2626;color:#fca5a5}}
/* Action buttons */
.actions{{display:flex;flex-wrap:wrap;gap:8px}}
.btn{{flex:1;min-width:110px;padding:11px 8px;border:none;border-radius:7px;
  font-size:.9em;font-weight:700;cursor:pointer;letter-spacing:.02em;transition:filter .2s}}
.btn:hover{{filter:brightness(1.15)}}
.btn-save{{background:#1d4ed8;color:#fff}}
.btn-reset{{background:#b45309;color:#fff}}
.btn-reboot{{background:#be123c;color:#fff}}
.btn-gpio{{background:#0f766e;color:#fff;text-decoration:none;
  display:flex;align-items:center;justify-content:center;border-radius:7px;
  font-weight:700;font-size:.9em;flex:1;min-width:110px;border:none;cursor:pointer;letter-spacing:.02em}}
.btn-gpio:hover{{filter:brightness(1.15)}}
.btn-logout{{background:#374151;color:#e2e8f0;text-decoration:none;
  display:flex;align-items:center;justify-content:center;border-radius:7px;
  font-weight:700;font-size:.9em;flex:1;min-width:110px}}
/* Divider */
hr{{border:none;border-top:1px solid #1e3a5f;margin:10px 0}}
</style></head><body>
<div class="page">
<h1>&#x2699;&#xFE0F; <span data-ja="GPIB-Server 設定">GPIB-Server Config</span> <span style="font-size:.6em;color:#94a3b8">v{version}</span> <span id="lang-slot" style="font-size:.5em;margin-left:8px"></span></h1>
<div id="msg"></div>

<form id="cfg">

<!-- Actions -->
<div class="card">
  <div class="actions">
    <button type="button" class="btn btn-save" onclick="saveConfig()" data-help="Save the settings on this page to NVS (non-volatile memory).&#10;Network settings etc. take effect after a reboot." data-help-ja="この画面の設定を NVS（不揮発メモリ）に保存します。&#10;ネットワーク関連などは再起動後に反映されます。"><span data-ja="&#x1F4BE; NVS に保存">&#x1F4BE; Save to NVS</span></button>
    <button type="button" class="btn btn-reset" onclick="resetDefaults()" data-help="Erase the NVS settings and restore the build-time defaults (cfg.toml)." data-help-ja="NVS の設定を消去し、ビルド時の既定値（cfg.toml）に戻します。"><span data-ja="&#x21BA; 既定値に戻す">&#x21BA; Reset Defaults</span></button>
    <button type="button" class="btn btn-reboot" onclick="reboot()" data-help="Restart the ESP32.&#10;You need to log in again afterwards." data-help-ja="ESP32 を再起動します。&#10;再起動後はログインし直しが必要です。"><span data-ja="&#x1F501; 再起動">&#x1F501; Reboot</span></button>
    <a href="/" class="btn-logout" data-help="Back to the main menu." data-help-ja="メインメニューに戻ります。"><span data-ja="&#x2190; メインメニュー">&#x2190; Main menu</span></a>
  </div>
</div>

<!-- WiFi -->
<div class="card">
  <div class="section-title">&#x1F4F6; WiFi</div>
  <div class="grid">
    <div class="field"><label>SSID</label>
      <input type="text" name="wifi_ssid" value="{wifi_ssid}" maxlength="32" autocomplete="off"></div>
    <div class="field"><label data-ja="パスワード">Password</label>
      <input type="password" name="wifi_psk" value="{wifi_psk}" maxlength="64" autocomplete="off"></div>
  </div>
  <hr>
  <div style="font-size:.8em;color:#64748b;margin-bottom:6px">WPS (PBC)</div>
  <div class="toggle-group">
    <label><input type="radio" name="wps_enable" value="true" {wps_true}><span data-ja="有効">Enable</span></label>
    <label><input type="radio" name="wps_enable" value="false" {wps_false}><span data-ja="無効">Disable</span></label>
  </div>
  <p style="font-size:.78em;color:#aaa;margin:.25em 0 0;" data-ja="WPS が有効で SSID が空の場合、次回起動時に WPS（プッシュボタン方式）で接続します。">When WPS is enabled and SSID is empty, WPS PBC is triggered on next boot.</p>
</div>

<!-- Network -->
<div class="card">
  <div class="section-title">&#x1F310; <span data-ja="ネットワーク">Network</span></div>
  <div class="toggle-group">
    <label><input type="radio" name="ip_mode" value="dhcp" {dhcp_chk} onchange="ipModeChange()"><span>DHCP</span></label>
    <label><input type="radio" name="ip_mode" value="static" {static_chk} onchange="ipModeChange()"><span data-ja="固定 IP">Static IP</span></label>
  </div>
  <div class="indent" id="static-fields">
    <div class="grid">
      <div class="field"><label data-ja="IP アドレス">IP Address</label>
        <input type="text" name="ip_address" value="{ip_address}" placeholder="192.168.1.100"></div>
      <div class="field"><label data-ja="サブネットマスク">Subnet Mask</label>
        <input type="text" name="subnet_mask" value="{subnet_mask}" placeholder="255.255.255.0"></div>
      <div class="field"><label data-ja="ゲートウェイ">Gateway</label>
        <input type="text" name="gateway" value="{gateway}" placeholder="192.168.1.1"></div>
      <div class="field"><label data-ja="DNS サーバー">DNS Server</label>
        <input type="text" name="dns" value="{dns}" placeholder="192.168.1.1"></div>
    </div>
  </div>
</div>

<!-- Syslog -->
<div class="card">
  <div class="section-title">&#x1F4CB; Syslog</div>
  <div class="grid">
    <div class="field" style="grid-column:1/-1"><label data-ja="サーバー（ホスト:ポート）">Server (host:port)</label>
      <input type="text" name="syslog_server" value="{syslog_server}" placeholder="192.168.1.1:514"></div>
    <div class="field"><label data-ja="ホスト名">Host Name</label>
      <input type="text" name="syslog_hostname" value="{syslog_host_name}" placeholder="esp32"></div>
    <div class="field"><label data-ja="アプリ名">App Name</label>
      <input type="text" name="syslog_app_name" value="{syslog_app_name}" placeholder="app"></div>
  </div>
  <hr>
  <div style="font-size:.8em;color:#64748b;margin-bottom:6px" data-ja="Syslog 送信">Enable Syslog</div>
  <div class="toggle-group">
    <label><input type="radio" name="syslog_enable" value="true" {syslog_true}><span data-ja="有効">Enabled</span></label>
    <label><input type="radio" name="syslog_enable" value="false" {syslog_false}><span data-ja="無効">Disabled</span></label>
  </div>
</div>

<!-- Auth -->
<div class="card">
  <div class="section-title">&#x1F510; <span data-ja="認証（SSH・Web 共通）">Authentication (SSH &amp; Web)</span></div>
  <div class="grid">
    <div class="field"><label data-ja="ユーザー名">Username</label>
      <input type="text" name="ssh_user" value="{ssh_user}" maxlength="64" autocomplete="username"></div>
    <div class="field"><label data-ja="パスワード">Password</label>
      <input type="password" name="ssh_password" value="{ssh_password}" maxlength="64" autocomplete="new-password"></div>
  </div>
</div>

<!-- GPIB -->
<div class="card">
  <div class="section-title">&#x1F50C; GPIB</div>
  <div style="font-size:.85em;color:#94a3b8;line-height:1.6">
    <span data-ja="コントローラ（Prologix 互換コマンド）として動作します。">Controller-in-charge (Prologix-compatible commands).</span><br>
    SSH: <code>ssh -tt {ssh_user}@&lt;host&gt; gpib</code> &nbsp;/&nbsp; <span data-ja="スクリプト用:">scripts:</span> <code>ssh {ssh_user}@&lt;host&gt; gpib raw</code><br>
    <span data-ja="ブラウザ:">Browser:</span> <a href="/terminal" style="color:#93c5fd" data-ja="ターミナル">Terminal</a> &mdash; <span data-ja="コマンド一覧は">type</span> <code>++help</code> <span data-ja="で表示">for commands.</span>
  </div>
  <div class="grid" style="margin-top:12px">
    <div class="field"><label data-ja="既定アドレス（ターミナル / SSH）">Default address (terminal / SSH)</label>
      <input type="number" name="gpib_addr" value="{gpib_addr}" min="0" max="30" required
        data-help="Primary address selected at power-up for ++ commands (0-30). Applied after reboot." data-help-ja="起動時に選択される ++ コマンドのアドレス（0〜30）。再起動後に反映されます。"></div>
    <div class="field"><label data-ja="スペクトラムアナライザ（R3267）">Spectrum analyzer (R3267)</label>
      <input type="number" name="sa_addr" value="{sa_addr}" min="0" max="30" required></div>
    <div class="field"><label data-ja="直流電源（6612B）">DC power supply (6612B)</label>
      <input type="number" name="psu_addr" value="{psu_addr}" min="0" max="30" required></div>
    <div class="field"><label data-ja="マルチメータ（34401A）">Multimeter (34401A)</label>
      <input type="number" name="dmm_addr" value="{dmm_addr}" min="0" max="30" required></div>
  </div>
  <div style="font-size:.78em;color:#64748b;margin-top:6px" data-ja="計測器のアドレスは各ページの ADDR を変更したときにも保存されます。">Instrument addresses are also saved when you change ADDR on each page.</div>
</div>

<!-- NTP Servers -->
<div class="card">
  <div class="section-title">&#x1F552; <span data-ja="NTP サーバー">NTP Servers</span></div>
  <div class="grid">
    <div class="field"><label data-ja="NTP サーバー 1（優先）">NTP Server 1 (Primary)</label>
      <input type="text" name="ntp_server1" value="{ntp_server1}" placeholder="time.google.com"></div>
    <div class="field"><label data-ja="NTP サーバー 2">NTP Server 2</label>
      <input type="text" name="ntp_server2" value="{ntp_server2}" placeholder="time.google.com"></div>
    <div class="field"><label data-ja="NTP サーバー 3">NTP Server 3</label>
      <input type="text" name="ntp_server3" value="{ntp_server3}" placeholder="time.cloudflare.com"></div>
    <div class="field"><label data-ja="NTP サーバー 4">NTP Server 4</label>
      <input type="text" name="ntp_server4" value="{ntp_server4}" placeholder="ntp.nict.jp"></div>
  </div>
</div>


</form>
</div>
<script>
// Show/hide static IP fields based on ip_mode radio
function ipModeChange() {{
  var isStatic = document.querySelector('input[name=ip_mode]:checked').value === 'static';
  document.getElementById('static-fields').style.display = isStatic ? 'block' : 'none';
}}
// Apply on load
ipModeChange();
function showMsg(text, ok) {{
  var d = document.getElementById('msg');
  d.textContent = text;
  d.className = ok ? 'ok' : 'err';
  d.style.display = 'block';
  d.scrollIntoView({{behavior:'smooth'}});
}}
function saveConfig() {{
  var fd = new FormData(document.getElementById('cfg'));
  var body = new URLSearchParams(fd).toString();
  authFetch('/api/config', {{
    method: 'POST',
    headers: {{'Content-Type': 'application/x-www-form-urlencoded'}},
    body: body
  }})
  .then(r => r.text())
  .then(t => showMsg(t.indexOf('Settings saved') === 0 ? tr(t, '保存しました。再起動後に反映されます。') : t, true))
  .catch(e => showMsg(tr('Save failed: ', '保存に失敗しました: ') + e, false));
}}
function reboot() {{
  if (!confirm(tr('Reboot the device?', 'デバイスを再起動しますか？'))) return;
  // The device's WiFi occasionally drops for a second or two (AP-side
  // roaming/beacon issues seen on some routers/extenders). If that happens
  // to overlap with this click, the fetch fails and the request never
  // reaches the device at all. Retry a few times with a short delay so a
  // transient drop doesn't silently swallow the reboot request.
  function attempt(retriesLeft) {{
    authFetch('/api/reboot', {{method: 'POST'}})
    .then(() => {{
      showMsg(tr('Rebooting... redirecting to login in 3 seconds.', '再起動中... 3 秒後にログイン画面へ移動します。'), true);
      setTimeout(() => {{ location.href = '/login'; }}, 3000);
    }})
    .catch(e => {{
      if (retriesLeft > 0) {{
        showMsg(tr('Reboot request failed (', '再起動要求に失敗しました (') + e + tr('), retrying...', ')。再試行します...'), false);
        setTimeout(() => attempt(retriesLeft - 1), 1500);
      }} else {{
        showMsg(tr('Reboot request failed: ', '再起動要求に失敗しました: ') + e, false);
      }}
    }});
  }}
  attempt(3);
}}
function resetDefaults() {{
  if (!confirm(tr('Reset all settings to compile-time defaults and reload the page?', 'すべての設定をビルド時の既定値に戻してページを再読み込みしますか？'))) return;
  authFetch('/api/reset', {{method: 'POST'}})
  .then(r => r.text())
  .then(() => {{ location.reload(); }})
  .catch(e => showMsg(tr('Reset failed: ', 'リセットに失敗しました: ') + e, false));
}}
</script>
</body></html>"#,
        wifi_ssid     = esc(&cfg.wifi_ssid),
        wifi_psk      = esc(&cfg.wifi_psk),
        wps_true      = chk(&cfg.wps_enable, "true"),
        wps_false     = chk(&cfg.wps_enable, "false"),
        dhcp_chk      = chk(&cfg.ip_mode, "dhcp"),
        static_chk    = chk(&cfg.ip_mode, "static"),
        ip_address    = esc(&cfg.ip_address),
        subnet_mask   = esc(&cfg.subnet_mask),
        gateway       = esc(&cfg.gateway),
        dns           = esc(&cfg.dns),
        syslog_server = esc(&cfg.syslog_server),
        syslog_true   = chk(&cfg.syslog_enable, "true"),
        syslog_false  = chk(&cfg.syslog_enable, "false"),
        syslog_host_name = esc(&cfg.syslog_host_name),
        syslog_app_name  = esc(&cfg.syslog_app_name),
        ssh_user      = esc(&cfg.ssh_user),
        ssh_password  = esc(&cfg.ssh_password),
        ntp_server1   = esc(&cfg.ntp_server1),
        ntp_server2   = esc(&cfg.ntp_server2),
        ntp_server3   = esc(&cfg.ntp_server3),
        ntp_server4   = esc(&cfg.ntp_server4),
        gpib_addr     = esc(&cfg.gpib_addr),
        sa_addr       = esc(&cfg.sa_addr),
        psu_addr      = esc(&cfg.psu_addr),
        dmm_addr      = esc(&cfg.dmm_addr),
        version       = APP_VERSION,
    )
}

pub fn start_http_server(state: ConfigState) -> anyhow::Result<EspHttpServer<'static>> {
    // 6 sessions: terminal + spectrum + power-supply WebSockets and page loads
    // (httpd uses max_open_sockets + 3 lwIP sockets; LWIP_MAX_SOCKETS = 16).
    let config = Configuration { max_open_sockets: 6, ..Default::default() };
    let mut server = EspHttpServer::new(&config)?;

    // ── GET /login ──────────────────────────────────────────────────────────
    server.fn_handler("/login", Method::Get, |request| {
        let mut response = request.into_ok_response()?;
        response.write_all(with_session_js(LOGIN_HTML).as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── POST /login ─────────────────────────────────────────────────────────
    let state_login = state.clone();
    server.fn_handler("/login", Method::Post, move |mut request| {
        let len = (request.content_len().unwrap_or(0) as usize).min(512);
        let mut buf = vec![0u8; len];
        request.read_exact(&mut buf).unwrap_or(());
        let body = String::from_utf8_lossy(&buf);
        let form = parse_form(&body);

        let cfg = state_login.config.lock().unwrap();
        let valid = form.get("username").map(String::as_str) == Some(cfg.ssh_user.as_str())
            && form.get("password").map(String::as_str) == Some(cfg.ssh_password.as_str());
        drop(cfg);

        if valid {
            let token = generate_token();
            *state_login.session.lock().unwrap() = Some(token.clone());
            let cookie = format!(
                "session={}; Path=/; HttpOnly; SameSite=Strict",
                token
            );
            let headers = [("Set-Cookie", cookie.as_str()), ("Location", "/")];
            request.into_response(302, Some("Found"), &headers)?.write_all(b"")?;
        } else {
            info!("HTTP: failed login attempt");
            let headers = [("Location", "/login?error=1")];
            request.into_response(302, Some("Found"), &headers)?.write_all(b"")?;
        }
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /logout ─────────────────────────────────────────────────────────
    let state_logout = state.clone();
    server.fn_handler("/logout", Method::Get, move |request| {
        *state_logout.session.lock().unwrap() = None;
        let headers = [
            ("Set-Cookie", "session=; Path=/; Max-Age=0"),
            ("Location", "/login"),
        ];
        request.into_response(302, Some("Found"), &headers)?.write_all(b"")?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET / — main menu (requires auth) ───────────────────────────────────
    let state_menu = state.clone();
    server.fn_handler("/", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_menu.session) {
            request.into_response(302, Some("Found"), &[("Location", "/login")])?.write_all(b"")?;
            return Ok::<(), anyhow::Error>(());
        }
        let (sa, psu, dmm) = {
            let c = state_menu.config.lock().unwrap();
            (c.sa_addr.clone(), c.psu_addr.clone(), c.dmm_addr.clone())
        };
        let html = with_session_js(&MENU_HTML.replace("{{VERSION}}", APP_VERSION)
            .replace("{{SA_ADDR}}", &sa).replace("{{PSU_ADDR}}", &psu).replace("{{DMM_ADDR}}", &dmm));
        request.into_response(200, Some("OK"), &[("Content-Type", "text/html; charset=utf-8")])?
            .write_all(html.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /status — device status and boot history (requires auth) ────────
    let state_status = state.clone();
    server.fn_handler("/status", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_status.session) {
            request.into_response(302, Some("Found"), &[("Location", "/login")])?.write_all(b"")?;
            return Ok::<(), anyhow::Error>(());
        }
        let html = with_session_js(&STATUS_HTML.replace("{{VERSION}}", APP_VERSION));
        request.into_response(200, Some("OK"), &[("Content-Type", "text/html; charset=utf-8")])?
            .write_all(html.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /config — configuration page (requires auth) ────────────────────
    let state_main = state.clone();
    server.fn_handler("/config", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_main.session) {
            let headers = [("Location", "/login")];
            request.into_response(302, Some("Found"), &headers)?.write_all(b"")?;
            return Ok::<(), anyhow::Error>(());
        }
        let cfg = state_main.config.lock().unwrap().clone();
        let html = with_session_js(&build_config_html(&cfg));
        let headers = [("Content-Type", "text/html; charset=UTF-8")];
        let mut response = request.into_response(200, Some("OK"), &headers)?;
        response.write_all(html.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── POST /api/config — save config to NVS (requires auth) ───────────────
    let state_cfg = state.clone();
    server.fn_handler("/api/config", Method::Post, move |mut request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_cfg.session) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let len = (request.content_len().unwrap_or(0) as usize).min(4096);
        let mut buf = vec![0u8; len];
        request.read_exact(&mut buf).unwrap_or(());
        let body = String::from_utf8_lossy(&buf);
        let form = parse_form(&body);

        let mut cfg = state_cfg.config.lock().unwrap();
        macro_rules! update {
            ($field:ident, $key:expr) => {
                if let Some(v) = form.get($key) {
                    cfg.$field = v.clone();
                }
            };
        }
        update!(wifi_ssid,     "wifi_ssid");
        update!(wifi_psk,      "wifi_psk");
        update!(ip_mode,       "ip_mode");
        update!(ip_address,    "ip_address");
        update!(subnet_mask,   "subnet_mask");
        update!(gateway,       "gateway");
        update!(dns,           "dns");
        update!(syslog_server, "syslog_server");
        update!(syslog_enable, "syslog_enable");
        update!(syslog_host_name, "syslog_hostname");
        update!(syslog_app_name, "syslog_app_name");
        update!(ssh_user,      "ssh_user");
        update!(ssh_password,  "ssh_password");
        update!(wps_enable,    "wps_enable");
        update!(ntp_server1,   "ntp_server1");
        update!(ntp_server2,   "ntp_server2");
        update!(ntp_server3,   "ntp_server3");
        update!(ntp_server4,   "ntp_server4");
        macro_rules! update_addr {
            ($field:ident, $key:expr) => {
                if let Some(v) = form.get($key) {
                    cfg.$field = norm_addr(v, &cfg.$field);
                }
            };
        }
        update_addr!(gpib_addr, "gpib_addr");
        update_addr!(sa_addr,   "sa_addr");
        update_addr!(psu_addr,  "psu_addr");
        update_addr!(dmm_addr,  "dmm_addr");

        // Apply syslog hostname/app_name changes immediately (no reboot needed)
        if let Err(e) = crate::syslogger::update_logger_config(&cfg.syslog_host_name, &cfg.syslog_app_name) {
            warn!("Failed to update syslog config: {}", e);
        }

        let ok = nvs_write_all(&cfg);
        drop(cfg);

        if ok {
            info!("HTTP: config saved to NVS");
            let headers = [("Content-Type", "text/plain; charset=UTF-8")];
            request.into_response(200, Some("OK"), &headers)?
                .write_all("Settings saved. Reboot to apply.".as_bytes())?;
        } else {
            warn!("HTTP: failed to save config to NVS");
            request.into_status_response(500)?
                .write_all(b"NVS write error")?;
        }
        Ok::<(), anyhow::Error>(())
    })?;

    // ── POST /api/gpib_addr — save an instrument page's address to NVS (requires auth)
    // Body: dev=sa|psu|dmm&addr=0..30
    let state_ga = state.clone();
    server.fn_handler("/api/gpib_addr", Method::Post, move |mut request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_ga.session) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let len = (request.content_len().unwrap_or(0) as usize).min(128);
        let mut buf = vec![0u8; len];
        request.read_exact(&mut buf).unwrap_or(());
        let form = parse_form(&String::from_utf8_lossy(&buf));
        let addr = form.get("addr").map(|v| norm_addr(v, "")).unwrap_or_default();
        let key = match form.get("dev").map(String::as_str) {
            Some("sa") => "sa_addr",
            Some("psu") => "psu_addr",
            Some("dmm") => "dmm_addr",
            _ => "",
        };
        if key.is_empty() || addr.is_empty() {
            request.into_status_response(400)?.write_all(b"Bad request")?;
            return Ok::<(), anyhow::Error>(());
        }
        let ok = {
            let mut cfg = state_ga.config.lock().unwrap();
            let slot = match key { "sa_addr" => &mut cfg.sa_addr, "psu_addr" => &mut cfg.psu_addr, _ => &mut cfg.dmm_addr };
            if *slot == addr { true } else {
                *slot = addr.clone();
                nvs_write_key(key, &addr)
            }
        };
        if ok {
            info!("HTTP: {} = {} saved to NVS", key, addr);
            request.into_ok_response()?.write_all(b"OK")?;
        } else {
            request.into_status_response(500)?.write_all(b"NVS write error")?;
        }
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /api/status — device/wifi/usb status (requires auth) ────────────
    let state_stat = state.clone();
    server.fn_handler("/api/status", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_stat.session) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let ip   = state_stat.status.ip_address.lock().unwrap().clone();
        let ssid = state_stat.status.ssid.lock().unwrap().clone();
        let rssi = *state_stat.status.rssi.lock().unwrap();
        let chip_temp = *state_stat.status.chip_temp.lock().unwrap();
        let json = format!(
            r#"{{"ip":"{ip}","ssid":"{ssid}","rssi":{rssi},"chip_temp":{chip_temp:.1},"gpib_addr":{gpib_addr},"gpib_srq":{gpib_srq},"ssh_sessions":{ssh_sessions},"spectrum_running":{spec},"uptime_s":{uptime},"version":"{version}"}}"#,
            ip           = esc(&ip),
            ssid         = esc(&ssid),
            rssi         = rssi,
            chip_temp    = chip_temp,
            gpib_addr    = crate::gpib::current_addr(),
            gpib_srq     = crate::gpib::srq_asserted(),
            ssh_sessions = crate::bridge::active_ssh_sessions(),
            spec         = crate::gpib::scope_running(),
            uptime       = unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1_000_000,
            version      = APP_VERSION,
        );
        let headers = [("Content-Type", "application/json")];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(json.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── POST /api/reset — erase NVS and restore defaults (requires auth) ────
    let state_rst = state.clone();
    server.fn_handler("/api/reset", Method::Post, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_rst.session) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        nvs_erase_namespace();
        let defaults = (*state_rst.defaults).clone();
        *state_rst.config.lock().unwrap() = defaults;
        info!("HTTP: config reset to defaults");
        let headers = [("Content-Type", "text/plain; charset=UTF-8")];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(b"Reset to defaults. NVS erased.")?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── POST /api/reboot (requires auth) ────────────────────────────────────
    let state_rb = state.clone();
    server.fn_handler("/api/reboot", Method::Post, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_rb.session) {
            warn!("HTTP: /api/reboot rejected — not authenticated");
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        // Log receipt immediately — this line is the only way to tell, from the
        // serial/syslog log, whether the reboot request actually reached the
        // device at all (as opposed to being lost to a transient WiFi drop
        // before it arrived, which looks identical from the browser's side).
        info!("HTTP: /api/reboot received — rebooting in 500ms");
        // Clear session so the login page is shown after reconnect
        *state_rb.session.lock().unwrap() = None;
        request.into_ok_response()?.write_all(b"Rebooting...")?;
        // Use Builder with an explicit, generous stack size. esp_restart() tears
        // down WiFi/drivers/heap before resetting, which needs more than a
        // couple KB of stack — 2048 was too small and caused a stack overflow
        // (task watchdog is disabled via CONFIG_ESP_TASK_WDT_EN=n, so the
        // corrupted task just hangs instead of forcing a reset, making the
        // device appear to "not reboot" at all).
        let spawned = std::thread::Builder::new()
            .name("reboot".into())
            .stack_size(8192)
            .spawn(|| {
                std::thread::sleep(std::time::Duration::from_millis(500));
                // Safety net: esp_restart()'s graceful shutdown handlers (WiFi
                // stop, netif teardown, etc.) can hang if they race with an
                // HTTP server / WebSocket connection that's still tearing
                // itself down on another task (observed in the field: WiFi
                // teardown logs + WS "removed sender"/"connection closed"
                // logs complete, then the device silently never resets).
                // CONFIG_ESP_TASK_WDT_EN=n means no watchdog is running by
                // default, so a stuck shutdown handler would hang forever.
                // Arm the Task Watchdog Timer just for this task with
                // trigger_panic=true: if esp_restart() hasn't completed
                // (i.e. actually reset the chip) within 5s, the TWDT fires a
                // panic, and CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y forces an
                // immediate hardware reboot regardless of what's stuck.
                unsafe {
                    let wdt_config = esp_task_wdt_config_t {
                        timeout_ms: 5000,
                        idle_core_mask: 0,
                        trigger_panic: true,
                    };
                    esp_task_wdt_init(&wdt_config);
                    esp_task_wdt_add(core::ptr::null_mut());
                }
                println!("HTTP: /api/reboot — calling esp_restart() now");
                unsafe { esp_restart(); }
            });
        if spawned.is_err() {
            warn!("HTTP: /api/reboot — thread spawn failed, rebooting from request thread");
            // Fallback: reboot directly if thread spawn failed
            std::thread::sleep(std::time::Duration::from_millis(200));
            unsafe { esp_restart(); }
        }
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /terminal — browser GPIB terminal (requires auth) ───────────────
    // ── GET /spectrum — R3267 real-time spectrum view (requires auth) ──────
    let state_spec = state.clone();
    server.fn_handler("/spectrum", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_spec.session) {
            request.into_response(302, Some("Found"), &[("Location", "/login")])?
                .write_all(b"")?;
            return Ok::<(), anyhow::Error>(());
        }
        let token = get_session_from_cookie_header(cookie_hdr.as_deref()).unwrap_or_default();
        let addr = state_spec.config.lock().unwrap().sa_addr.clone();
        let html = with_session_js(&SPECTRUM_HTML.replace("{{TOKEN}}", &token).replace("{{ADDR}}", &addr));
        let headers = [("Content-Type", "text/html; charset=utf-8")];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(html.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    let state_psu = state.clone();
    server.fn_handler("/psu", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_psu.session) {
            request.into_response(302, Some("Found"), &[("Location", "/login")])?
                .write_all(b"")?;
            return Ok::<(), anyhow::Error>(());
        }
        let token = get_session_from_cookie_header(cookie_hdr.as_deref()).unwrap_or_default();
        let addr = state_psu.config.lock().unwrap().psu_addr.clone();
        let html = with_session_js(&PSU_HTML.replace("{{TOKEN}}", &token).replace("{{ADDR}}", &addr));
        let headers = [("Content-Type", "text/html; charset=utf-8")];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(html.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    let state_dmm = state.clone();
    server.fn_handler("/dmm", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_dmm.session) {
            request.into_response(302, Some("Found"), &[("Location", "/login")])?
                .write_all(b"")?;
            return Ok::<(), anyhow::Error>(());
        }
        let token = get_session_from_cookie_header(cookie_hdr.as_deref()).unwrap_or_default();
        let addr = state_dmm.config.lock().unwrap().dmm_addr.clone();
        let html = with_session_js(&DMM_HTML.replace("{{TOKEN}}", &token).replace("{{ADDR}}", &addr));
        let headers = [("Content-Type", "text/html; charset=utf-8")];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(html.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    let state_term = state.clone();
    server.fn_handler("/terminal", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_term.session) {
            request.into_response(302, Some("Found"), &[("Location", "/login")])?
                .write_all(b"")?;
            return Ok::<(), anyhow::Error>(());
        }
        let token = get_session_from_cookie_header(cookie_hdr.as_deref()).unwrap_or_default();
        let html = with_session_js(&build_terminal_html().replace("{{TOKEN}}", &token));
        let headers = [("Content-Type", "text/html; charset=utf-8")];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(html.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /static/xterm.js — serve embedded xterm.js ───────────────────────
    server.fn_handler("/static/xterm.js", Method::Get, |request| {
        // Stored gzip-compressed (488 KB → 120 KB): keeps the firmware image small.
        // Regenerate with: gzip -9nc static/xterm.min.js > static/xterm.min.js.gz
        let headers = [
            ("Content-Type", "application/javascript"),
            ("Content-Encoding", "gzip"),
            ("Cache-Control", "public, max-age=86400"),
        ];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(include_bytes!("../static/xterm.min.js.gz"))?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /static/xterm.css — serve embedded xterm.css ─────────────────────
    server.fn_handler("/static/xterm.css", Method::Get, |request| {
        let headers = [
            ("Content-Type", "text/css"),
            ("Cache-Control", "public, max-age=86400"),
        ];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(include_bytes!("../static/xterm.min.css"))?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /static/xterm-addon-fit.js — serve embedded fit addon ───────────
    server.fn_handler("/static/xterm-addon-fit.js", Method::Get, |request| {
        let headers = [
            ("Content-Type", "application/javascript"),
            ("Cache-Control", "public, max-age=86400"),
        ];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(include_bytes!("../static/xterm-addon-fit.min.js"))?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /api/boot_log — boot history from NVS (requires auth) ───────────
    let state_bl = state.clone();
    server.fn_handler("/api/boot_log", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_bl.session) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let log = crate::boot_log::read_log();
        let entries: Vec<&str> = if log.is_empty() { Vec::new() } else { log.lines().collect() };
        let json_entries = entries.iter()
            .map(|e| format!("\"{}\"", e.replace('"', "\\\"")))
            .collect::<Vec<_>>()
            .join(",");
        let json = format!("{{\"entries\":[{}]}}", json_entries);
        let headers = [("Content-Type", "application/json")];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(json.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── GET /api/diag_log — one-shot debug entries from NVS (requires auth,
    // temporary diagnostic aid while there's no serial console access) ──────
    let state_dl = state.clone();
    server.fn_handler("/api/diag_log", Method::Get, move |request| {
        let cookie_hdr = request.header("Cookie").map(str::to_owned);
        if !is_auth(cookie_hdr.as_deref(), &state_dl.session) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let log = crate::boot_log::read_diag();
        let entries: Vec<&str> = if log.is_empty() { Vec::new() } else { log.lines().collect() };
        let json_entries = entries.iter()
            .map(|e| format!("\"{}\"", e.replace('"', "\\\"")))
            .collect::<Vec<_>>()
            .join(",");
        let json = format!("{{\"entries\":[{}]}}", json_entries);
        let headers = [("Content-Type", "application/json")];
        request.into_response(200, Some("OK"), &headers)?
            .write_all(json.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    // ── WebSocket /ws/serial — browser terminal ⇄ GPIB interpreter ─────────
    let state_ws = state.clone();
    server.ws_handler("/ws/serial", move |ws| {
        // ── Connection open ─────────────────────────────────────────────
        if ws.is_new() {
            info!("[WS] new terminal connection fd={}", ws.session());
            return Ok::<(), anyhow::Error>(());
        }
        // ── Connection closed ───────────────────────────────────────────
        if ws.is_closed() {
            let fd = ws.session();
            crate::bridge::ws_remove_sender(fd);
            info!("[WS] terminal connection closed fd={}", fd);
            return Ok::<(), anyhow::Error>(());
        }

        // ── Receive data ────────────────────────────────────────────────
        // Phase 1: get frame metadata
        let (frame_type, len) = ws.recv(&mut [])?;
        if len == 0 {
            return Ok(());
        }
        let mut buf = vec![0u8; len];
        ws.recv(&mut buf)?;

        match frame_type {
            embedded_svc::ws::FrameType::Text(_) => {
                // Text frame = control message (JSON): {"cmd":"init","port":"gpib"}
                let msg = String::from_utf8_lossy(&buf);
                if parse_ws_port(&msg).is_some() && !ws_token_ok(&msg, &state_ws.session) {
                    warn!("[WS] terminal: invalid session token");
                    ws.send(embedded_svc::ws::FrameType::Text(false), WS_AUTH_ERROR.as_bytes())?;
                } else if parse_ws_port(&msg).is_some() {
                    let fd = ws.session();
                    let sender = ws.create_detached_sender()?;
                    crate::bridge::ws_remove_sender(fd);
                    crate::bridge::ws_register_sender(fd, sender);
                }
            }
            embedded_svc::ws::FrameType::Binary(_) => {
                // Binary frame = keystrokes from the browser terminal
                if crate::bridge::ws_is_registered(ws.session()) {
                    crate::bridge::ws_input(&buf);
                }
            }
            _ => {}
        }
        Ok::<(), anyhow::Error>(())
    })?;

    // ── WebSocket /ws/spectrum — R3267 spectrum view control + frames ──────
    // Text (browser → ESP32): {"cmd":"hello"|"start"|"stop"|"key","token":"…",
    //                          "addr":30,"cf":119,"span":10,"rbw":100,"text":"CF 100MZ"}
    //   start: cf/span in MHz, rbw in kHz (optional; omitted = keep current).
    //   key:   front-panel GPIB code, validated by gpib::validate_panel_cmd.
    // Text (ESP32 → browser): {"type":"meta"|"state"|"error",…}
    // Binary (ESP32 → browser): one trace, N x u16 big-endian raw values.
    let state_spec_ws = state.clone();
    server.ws_handler("/ws/spectrum", move |ws| {
        if ws.is_new() {
            return Ok::<(), anyhow::Error>(());
        }
        if ws.is_closed() {
            crate::bridge::spec_remove_sender(ws.session());
            return Ok::<(), anyhow::Error>(());
        }
        let (frame_type, len) = ws.recv(&mut [])?;
        if len == 0 || len > 1024 {
            return Ok(());
        }
        let mut buf = vec![0u8; len];
        ws.recv(&mut buf)?;
        if let embedded_svc::ws::FrameType::Text(_) = frame_type {
            let msg = String::from_utf8_lossy(&buf).to_string();
            if !ws_token_ok(&msg, &state_spec_ws.session) {
                warn!("[WS] spectrum: invalid session token");
                ws.send(embedded_svc::ws::FrameType::Text(false), WS_AUTH_ERROR.as_bytes())?;
                return Ok(());
            }
            let fd = ws.session();
            if !crate::bridge::spec_is_registered(fd) {
                let sender = ws.create_detached_sender()?;
                crate::bridge::spec_register_sender(fd, sender);
            }
            match json_str(&msg, "cmd").as_deref() {
                Some("start") => {
                    let p = crate::gpib::ScopeParams {
                        addr:     json_num(&msg, "addr").unwrap_or(30.0) as u8,
                        cf_mhz:   json_num(&msg, "cf"),
                        span_mhz: json_num(&msg, "span"),
                        rbw_khz:  json_num(&msg, "rbw"),
                    };
                    info!("[WS] spectrum start: {:?}", p);
                    crate::gpib::abort(); // interrupt a stuck handshake first
                    crate::gpib::scope_start(p);
                }
                Some("stop") => {
                    crate::gpib::abort();
                    crate::gpib::scope_stop();
                }
                Some("query") => {
                    // Read-only panel queries: {"cmd":"query","id":"menu","q":"ATMIN? ATMINON?"}
                    let addr = json_num(&msg, "addr").unwrap_or(30.0) as u8;
                    let q = json_str(&msg, "q").unwrap_or_default();
                    let id = json_str(&msg, "id").unwrap_or_default();
                    let qs: Vec<&str> = q.split_whitespace().collect();
                    if let Err(e) = crate::gpib::scope_query(addr, &qs, &id) {
                        crate::bridge::spec_error(&format!("{}: {}", q, e));
                    }
                }
                Some("key") => {
                    // Front-panel command, e.g. {"cmd":"key","text":"CF 100MZ"}
                    let addr = json_num(&msg, "addr").unwrap_or(30.0) as u8;
                    let text = json_str(&msg, "text").unwrap_or_default();
                    if let Err(e) = crate::gpib::scope_cmd(addr, &text) {
                        warn!("[WS] panel command rejected '{}': {}", text, e);
                        crate::bridge::spec_error(&format!("{}: {}", text, e));
                    }
                }
                _ => {}
            }
        }
        Ok::<(), anyhow::Error>(())
    })?;

    // ── WebSocket /ws/inst — SCPI instrument pages (/psu HP 6612B, /dmm HP 34401A) ─
    // Text (browser → ESP32): {"cmd":"req","dev":"psu"|"dmm","token":"…","addr":2,"id":"poll",
    //                          "q":"VOLT 5|OUTP ON|MEAS:VOLT?"}  ('|'-separated, each item
    //   validated by gpib::validate_inst_item for that device, executed in order).
    // Text (ESP32 → browser): {"type":"psu"|"dmm","id":…,"addr":2,"vals":{query:reply},"err":"…"}
    let state_psu_ws = state.clone();
    server.ws_handler("/ws/inst", move |ws| {
        if ws.is_new() {
            return Ok::<(), anyhow::Error>(());
        }
        if ws.is_closed() {
            crate::bridge::inst_remove_sender(ws.session());
            return Ok::<(), anyhow::Error>(());
        }
        let (frame_type, len) = ws.recv(&mut [])?;
        if len == 0 || len > 1024 {
            return Ok(());
        }
        let mut buf = vec![0u8; len];
        ws.recv(&mut buf)?;
        if let embedded_svc::ws::FrameType::Text(_) = frame_type {
            let msg = String::from_utf8_lossy(&buf).to_string();
            if !ws_token_ok(&msg, &state_psu_ws.session) {
                warn!("[WS] inst: invalid session token");
                ws.send(embedded_svc::ws::FrameType::Text(false), WS_AUTH_ERROR.as_bytes())?;
                return Ok(());
            }
            let fd = ws.session();
            if !crate::bridge::inst_is_registered(fd) {
                let sender = ws.create_detached_sender()?;
                crate::bridge::inst_register_sender(fd, sender);
            }
            if json_str(&msg, "cmd").as_deref() == Some("req") {
                let dev = json_str(&msg, "dev").as_deref().and_then(crate::gpib::Dev::from_name)
                    .unwrap_or(crate::gpib::Dev::Psu);
                let addr = json_num(&msg, "addr").unwrap_or(2.0) as u8;
                let q = json_str(&msg, "q").unwrap_or_default();
                let id = json_str(&msg, "id").unwrap_or_default();
                let items: Vec<&str> = q.split('|').map(str::trim).filter(|s| !s.is_empty()).collect();
                if let Err(e) = crate::gpib::inst_request_queue(dev, addr, &items, &id) {
                    warn!("[WS] {:?} request rejected '{}': {}", dev, q, e);
                    crate::bridge::inst_error(if dev == crate::gpib::Dev::Dmm { "dmm" } else { "psu" }, &format!("{}: {}", q, e));
                }
            }
        }
        Ok::<(), anyhow::Error>(())
    })?;

    info!("HTTP config server started on port 80");
    Ok(server)
}

/// Check the "token" field of a WebSocket control message against the
/// current login session (the WS upgrade request itself is not authenticated).
fn ws_token_ok(msg: &str, session: &Arc<Mutex<Option<String>>>) -> bool {
    let token = match json_str(msg, "token") {
        Some(t) if !t.is_empty() => t,
        _ => return false,
    };
    session.lock().unwrap().as_deref() == Some(token.as_str())
}

/// Minimal JSON helpers for flat messages (avoid pulling in serde_json).
fn json_str(msg: &str, key: &str) -> Option<String> {
    let pat = format!("\"{}\"", key);
    let rest = &msg[msg.find(&pat)? + pat.len()..];
    let rest = rest.trim_start().strip_prefix(':')?.trim_start().strip_prefix('"')?;
    Some(rest[..rest.find('"')?].to_string())
}

fn json_num(msg: &str, key: &str) -> Option<f64> {
    let pat = format!("\"{}\"", key);
    let rest = &msg[msg.find(&pat)? + pat.len()..];
    let rest = rest.trim_start().strip_prefix(':')?.trim_start();
    let end = rest.find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E')))
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

// ─── Session-expired notice (all pages) ──────────────────────────────────────

/// Reply sent on a WebSocket whose message carries an invalid session token
/// (typically after the ESP32 restarted: sessions are kept in RAM only).
const WS_AUTH_ERROR: &str = r#"{"type":"auth","error":"invalid session token"}"#;

/// Injected into <head> of every page. `authFetch()` wraps fetch() and shows
/// the notice on HTTP 401; pages call `sessionExpired()` on a WS "auth" error.
const SESSION_JS: &str = r#"<script>
// ── UI language (EN / JA), stored per browser ─────────────────────────────
// Static text: elements carry the English text and a data-ja attribute
// (innerHTML used in Japanese mode). Hover help: data-help (EN) / data-help-ja.
// Script text: tr('English', '日本語').
var LANG = (function () {
  try { var l = localStorage.getItem('lang'); if (l === 'en' || l === 'ja') return l; } catch (e) {}
  return (navigator.language || '').toLowerCase().indexOf('ja') === 0 ? 'ja' : 'en';
})();
document.documentElement.lang = LANG;
function tr(en, ja) { return LANG === 'ja' ? ja : en; }
function setLang(l) { try { localStorage.setItem('lang', l); } catch (e) {} location.reload(); }
function applyLang() {
  if (LANG === 'ja') {
    document.querySelectorAll('[data-ja]').forEach(function (el) { el.innerHTML = el.getAttribute('data-ja'); });
    document.querySelectorAll('[data-ja-title]').forEach(function (el) { el.title = el.getAttribute('data-ja-title'); });
  }
  var t = document.querySelector('title[data-ja]');
  if (t && LANG === 'ja') document.title = t.getAttribute('data-ja');
  var sw = document.createElement('span');
  sw.style.cssText = 'display:inline-flex;border:1px solid #2d4a6a;border-radius:7px;overflow:hidden;font:600 12px Segoe UI,Arial,sans-serif;vertical-align:middle';
  [['en', 'EN'], ['ja', '日本語']].forEach(function (o) {
    var b = document.createElement('button');
    b.type = 'button'; b.textContent = o[1];
    b.setAttribute('data-help', 'Switch the display language');
    b.setAttribute('data-help-ja', '表示言語を切り替えます');
    b.style.cssText = 'border:none;border-radius:0;padding:5px 9px;cursor:pointer;font:inherit;' +
      (LANG === o[0] ? 'background:#1d4ed8;color:#fff' : 'background:#0f172a;color:#94a3b8');
    b.onclick = function () { if (LANG !== o[0]) setLang(o[0]); };
    sw.appendChild(b);
  });
  var slot = document.getElementById('lang-slot');
  if (slot) slot.appendChild(sw);
  else { sw.style.position = 'fixed'; sw.style.top = '8px'; sw.style.right = '8px'; sw.style.zIndex = '50'; document.body.appendChild(sw); }
}
document.addEventListener('DOMContentLoaded', applyLang);
var sessionDead = false;
function sessionExpired() {
  if (sessionDead) return;
  sessionDead = true;
  var d = document.createElement('div');
  d.id = 'sess-exp';
  d.style.cssText = 'position:fixed;inset:0;z-index:10000;background:rgba(0,0,0,.72);display:flex;align-items:center;justify-content:center';
  d.innerHTML = '<div style="background:#1e2d45;border:1px solid #f87171;border-radius:12px;padding:24px 28px;max-width:460px;margin:16px;color:#e2e8f0;font-family:Segoe UI,Arial,sans-serif;box-shadow:0 8px 32px rgba(0,0,0,.6)">' +
    '<div style="color:#f87171;font-weight:700;font-size:1.1em;margin-bottom:10px">&#x26A0; ' + tr('Session expired', 'セッションが無効です') + '</div>' +
    '<div style="font-size:.9em;line-height:1.6;margin-bottom:16px">' +
    tr('The login session of this page is no longer valid (the device may have restarted). Please reload the page and log in again.',
       'このページのログインセッションが無効になりました（デバイスが再起動した可能性があります）。ページを再読み込みして、再度ログインしてください。') + '</div>' +
    '<button onclick="location.reload()" style="padding:9px 20px;background:#1d4ed8;color:#fff;border:none;border-radius:7px;font-weight:700;font-size:.95em;cursor:pointer">&#x21BB; ' + tr('Reload', '再読み込み') + '</button></div>';
  document.body.appendChild(d);
}
function authFetch(url, opt) {
  return fetch(url, opt).then(function (r) {
    if (r.status === 401) { sessionExpired(); throw new Error('session expired'); }
    return r;
  });
}
// Hover help: any element with data-help shows its text after the mouse has
// rested on it for 3 s (the native title tooltip delay cannot be changed).
(function () {
  var HELP_DELAY_MS = 3000, timer = null, tip = null, target = null, mx = 0, my = 0;
  function hide() {
    if (timer) { clearTimeout(timer); timer = null; }
    if (tip) { tip.remove(); tip = null; }
  }
  function show() {
    timer = null;
    if (!target || !target.isConnected) return;
    var text = (LANG === 'ja' && target.getAttribute('data-help-ja')) || target.getAttribute('data-help');
    if (!text) return;
    tip = document.createElement('div');
    tip.style.cssText = 'position:fixed;z-index:9999;max-width:320px;padding:9px 12px;background:#0b1220;color:#e2e8f0;' +
      'border:1px solid #60a5fa;border-radius:8px;font:13px/1.55 Segoe UI,Arial,sans-serif;white-space:pre-line;' +
      'box-shadow:0 6px 24px rgba(0,0,0,.55);pointer-events:none';
    tip.textContent = text;
    document.body.appendChild(tip);
    var w = tip.offsetWidth, h = tip.offsetHeight;
    var x = Math.min(mx + 14, window.innerWidth - w - 8), y = my + 18;
    if (y + h > window.innerHeight - 8) y = my - h - 12;
    tip.style.left = Math.max(8, x) + 'px'; tip.style.top = Math.max(8, y) + 'px';
  }
  document.addEventListener('mouseover', function (e) {
    var el = e.target.closest ? e.target.closest('[data-help]') : null;
    if (el === target) return;
    // The element under the mouse was replaced by an equivalent one (page
    // re-rendered while the pointer did not move): keep the tip / timer.
    if (el && target && !target.isConnected && el.getAttribute('data-help') === target.getAttribute('data-help')) {
      target = el; return;
    }
    hide(); target = el;
    if (el) timer = setTimeout(show, HELP_DELAY_MS);
  });
  document.addEventListener('mousemove', function (e) { mx = e.clientX; my = e.clientY; });
  setInterval(function () {
    if (tip && target && target.isConnected) {
      var t = (LANG === 'ja' && target.getAttribute('data-help-ja')) || target.getAttribute('data-help');
      if (t && tip.textContent !== t) tip.textContent = t;
    }
  }, 500);
  document.addEventListener('mousedown', function () { hide(); target = null; }, true);
  document.addEventListener('wheel', hide, {passive: true});
  window.addEventListener('blur', hide);
})();
</script>"#;

/// Insert SESSION_JS into a page.
fn with_session_js(html: &str) -> String {
    html.replacen("</head>", &format!("{}</head>", SESSION_JS), 1)
}

// ─── Spectrum page ───────────────────────────────────────────────────────────

const SPECTRUM_HTML: &str = include_str!("../static/spectrum.html");
const PSU_HTML: &str = include_str!("../static/psu.html");
const DMM_HTML: &str = include_str!("../static/dmm.html");
const MENU_HTML: &str = include_str!("../static/menu.html");
const STATUS_HTML: &str = include_str!("../static/status.html");

// ─── WebSocket helpers ───────────────────────────────────────────────────────

/// Parse port name from WS init message like {"cmd":"init","port":"gpib"}
fn parse_ws_port(msg: &str) -> Option<String> {
    // Simple parse — avoid pulling in serde_json
    if let Some(idx) = msg.find("\"port\"") {
        let rest = &msg[idx + 6..];
        if let Some(start) = rest.find('"') {
            let rest = &rest[start + 1..];
            if let Some(end) = rest.find('"') {
                if &rest[..end] == "gpib" {
                    return Some("gpib".to_string());
                }
            }
        }
    }
    None
}

// ─── Terminal HTML page ──────────────────────────────────────────────────────

fn build_terminal_html() -> String {
    r#"<!DOCTYPE html><html lang="ja"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title data-ja="GPIB-Server ターミナル">GPIB-Server Terminal</title>
<link rel="stylesheet" href="/static/xterm.css">
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111827;color:#e2e8f0;font-family:'Segoe UI',Arial,sans-serif;height:100vh;display:flex;flex-direction:column}
.toolbar{display:flex;align-items:center;gap:10px;padding:8px 14px;background:#1e2d45;
  border-bottom:1px solid #243b55;flex-shrink:0}
.toolbar h1{font-size:1em;color:#93c5fd;letter-spacing:.05em;margin-right:auto}
.toolbar button{padding:6px 14px;border-radius:6px;border:1px solid #2d4a6a;
  background:#0f172a;color:#e2e8f0;font-size:.85em;cursor:pointer}
.toolbar button:focus{outline:none;border-color:#60a5fa}
.toolbar button:hover{filter:brightness(1.2)}
.btn-connect{background:#166534;border-color:#16a34a;font-weight:700}
.btn-connect.connected{background:#991b1b;border-color:#dc2626}
.btn-back{background:#374151;text-decoration:none;color:#e2e8f0;display:inline-flex;
  align-items:center;padding:6px 14px;border-radius:6px;font-size:.85em;font-weight:600}
.status{font-size:.78em;padding:3px 10px;border-radius:12px;font-weight:700}
.status.ok{background:#14532d;color:#4ade80}.status.off{background:#4c0519;color:#fb7185}
.status.warn{background:#451a03;color:#fb923c}
#terminal-container{flex:1;padding:4px;overflow:hidden}
</style></head><body>
<div class="toolbar">
  <h1>&#x1F4BB; <span data-ja="GPIB ターミナル">GPIB Terminal</span></h1>
  <span id="lang-slot"></span>
  <span id="gpib-info" class="status ok" data-help="GPIB address of the current instrument (change with ++addr)." data-help-ja="現在の計測器 GPIB アドレス（++addr で変更）。">ADDR --</span>
  <span id="srq-status" class="status ok" data-help="State of the SRQ (service request) line. Shows SRQ when an instrument requests service; check with ++spoll." data-help-ja="SRQ（サービスリクエスト）線の状態。計測器が要求を出すと SRQ と表示されます。++spoll で確認できます。">SRQ --</span>
  <button id="btn-conn" class="btn-connect" onclick="toggleConnection()" data-help="Connect to / disconnect from the GPIB interpreter.&#10;Once connected, type e.g. *IDN? to send it to the instrument and see the reply.&#10;++help lists the commands; Ctrl-C aborts a pending operation." data-help-ja="GPIB インタプリタに接続／切断します。&#10;接続後は *IDN? などをそのまま入力すると計測器に送信・応答を表示します。&#10;++help でコマンド一覧、Ctrl-C で待ち中の処理を中断します。" data-ja="接続">Connect</button>
  <span id="ws-status" class="status off" data-ja="未接続">Disconnected</span>
  <a href="/" class="btn-back" data-ja="&#x2190; メインメニュー">&#x2190; Main menu</a>
</div>
<div id="terminal-container"></div>
<script src="/static/xterm.js"></script>
<script src="/static/xterm-addon-fit.js"></script>
<script>
const term = new Terminal({
  cursorBlink: true,
  fontSize: 14,
  theme: {
    background: '#0f172a',
    foreground: '#e2e8f0',
    cursor: '#60a5fa',
    selectionBackground: '#334155'
  }
});
const fitAddon = new FitAddon.FitAddon();
term.loadAddon(fitAddon);
term.open(document.getElementById('terminal-container'));
fitAddon.fit();
window.addEventListener('resize', () => fitAddon.fit());
term.write(tr('GPIB controller (Prologix-compatible). Press Connect, then type ++help.', 'GPIB コントローラ（Prologix 互換）。「接続」を押してから ++help を入力してください。') + '\r\n');

let ws = null;
const btnConn = document.getElementById('btn-conn');
const wsStatus = document.getElementById('ws-status');

function toggleConnection() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.close();
  } else {
    connect();
  }
}

function connect() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  ws = new WebSocket(proto + '//' + location.host + '/ws/serial');
  ws.binaryType = 'arraybuffer';

  ws.onopen = () => {
    wsStatus.textContent = tr('Connected (GPIB)', '接続中（GPIB）');
    wsStatus.className = 'status ok';
    btnConn.textContent = tr('Disconnect', '切断');
    btnConn.classList.add('connected');
    ws.send(JSON.stringify({cmd: 'init', port: 'gpib', token: '{{TOKEN}}'}));
    term.focus();
  };

  ws.onmessage = (ev) => {
    if (ev.data instanceof ArrayBuffer) {
      term.write(new Uint8Array(ev.data));
    } else {
      try {
        const m = JSON.parse(ev.data);
        if (m.type === 'auth') { sessionExpired(); ws.close(); return; }
      } catch (e) {}
      term.write(ev.data);
    }
  };

  ws.onclose = () => {
    wsStatus.textContent = tr('Disconnected', '未接続');
    wsStatus.className = 'status off';
    btnConn.textContent = tr('Connect', '接続');
    btnConn.classList.remove('connected');
    ws = null;
  };

  ws.onerror = (err) => {
    console.error('WS error:', err);
    term.write('\r\n\x1b[31m[' + tr('WebSocket error', 'WebSocket エラー') + ']\x1b[0m\r\n');
  };
}

// Terminal input → WebSocket (binary)
term.onData((data) => {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(new TextEncoder().encode(data));
  }
});

// GPIB address / SRQ status
function updateGpibStatus() {
  authFetch('/api/status')
  .then(r => r.json())
  .then(d => {
    document.getElementById('gpib-info').textContent = 'ADDR ' + d.gpib_addr;
    const s = document.getElementById('srq-status');
    s.textContent = d.gpib_srq ? 'SRQ' : tr('SRQ idle', 'SRQ なし');
    s.className = 'status ' + (d.gpib_srq ? 'warn' : 'ok');
  })
  .catch(function(){});
}
updateGpibStatus();
setInterval(updateGpibStatus, 3000);
</script>
</body></html>"#.to_string()
}
