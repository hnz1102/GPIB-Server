use log::info;
use esp_idf_svc::sntp::{EspSntp, SyncStatus, SntpConf, OperatingMode, SyncMode};
use chrono::{DateTime, Utc};
use esp_idf_hal::prelude::*;
use esp_idf_hal::temp_sensor::{TempSensorConfig, TempSensorDriver};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use std::time::{Duration, SystemTime};
use std::thread;

mod wifi;
mod syslogger;
mod bridge;
mod gpib;
mod httpserver;
mod btn_ctrl;
mod boot_log;
mod mini_display;

#[toml_cfg::toml_config]
pub struct Config {
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
    #[default("dhcp")]
    ip_mode: &'static str,
    #[default("")]
    ip_address: &'static str,
    #[default("255.255.255.0")]
    subnet_mask: &'static str,
    #[default("")]
    gateway: &'static str,
    #[default("")]
    dns: &'static str,
    #[default("")]
    syslog_server: &'static str,
    #[default("false")]
    syslog_enable: &'static str,
    #[default("esp32")]
    syslog_host_name: &'static str,
    #[default("app")]
    syslog_app_name: &'static str,
    #[default("admin")]
    ssh_user: &'static str,
    #[default("esp32")]
    ssh_password: &'static str,
    #[default("false")]
    wps_enable: &'static str,
    #[default("false")]
    mini_display_enable: &'static str,
    #[default("time.aws.com")]
    ntp_server1: &'static str,
    #[default("time.google.com")]
    ntp_server2: &'static str,
    #[default("time.cloudflare.com")]
    ntp_server3: &'static str,
    #[default("ntp.nict.jp")]
    ntp_server4: &'static str,
    #[default("1")]
    gpib_addr: &'static str,
    #[default("30")]
    sa_addr: &'static str,
    #[default("2")]
    psu_addr: &'static str,
    #[default("22")]
    dmm_addr: &'static str,
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Peripherals Initialize
    let peripherals = Peripherals::take().unwrap();

    // NVS flash init (required before WiFi for RF calibration data)
    unsafe { esp_idf_sys::nvs_flash_init(); }
    // Record reset reason and boot count to NVS before anything else
    let (boot_count, reset_reason) = boot_log::record_boot();
    // Record Rust panics (message + location + thread) to the NVS diag log so
    // they can be read after the reboot via /api/diag_log.
    std::panic::set_hook(Box::new(|info| {
        let loc = info.location().map(|l| format!("{}:{}", l.file(), l.line())).unwrap_or_default();
        let msg = info.payload().downcast_ref::<&str>().map(|s| s.to_string())
            .or_else(|| info.payload().downcast_ref::<String>().cloned())
            .unwrap_or_default();
        let thread = std::thread::current().name().unwrap_or("?").to_string();
        let line: String = format!("PANIC [{}] {} at {}", thread, msg, loc).chars().take(200).collect();
        println!("{}", line);
        boot_log::record_diag(&line);
    }));
    println!("Boot #{}, reset reason: {}", boot_count, reset_reason);
    // Load config from NVS (overrides cfg.toml defaults where NVS keys are set)
    let cfg_defaults = httpserver::NvsConfig {
        wifi_ssid:     CONFIG.wifi_ssid.to_string(),
        wifi_psk:      CONFIG.wifi_psk.to_string(),
        ip_mode:       CONFIG.ip_mode.to_string(),
        ip_address:    CONFIG.ip_address.to_string(),
        subnet_mask:   CONFIG.subnet_mask.to_string(),
        gateway:       CONFIG.gateway.to_string(),
        dns:           CONFIG.dns.to_string(),
        syslog_server: CONFIG.syslog_server.to_string(),
        syslog_enable: CONFIG.syslog_enable.to_string(),
        syslog_host_name: CONFIG.syslog_host_name.to_string(),
        syslog_app_name: CONFIG.syslog_app_name.to_string(),
        ssh_user:      CONFIG.ssh_user.to_string(),
        ssh_password:  CONFIG.ssh_password.to_string(),
        wps_enable:    CONFIG.wps_enable.to_string(),
        mini_display_enable: CONFIG.mini_display_enable.to_string(),
        ntp_server1:   CONFIG.ntp_server1.to_string(),
        ntp_server2:   CONFIG.ntp_server2.to_string(),
        ntp_server3:   CONFIG.ntp_server3.to_string(),
        ntp_server4:   CONFIG.ntp_server4.to_string(),
        gpib_addr:     httpserver::norm_addr(CONFIG.gpib_addr, "1"),
        sa_addr:       httpserver::norm_addr(CONFIG.sa_addr, "30"),
        psu_addr:      httpserver::norm_addr(CONFIG.psu_addr, "2"),
        dmm_addr:      httpserver::norm_addr(CONFIG.dmm_addr, "22"),
    };
    let mut nvs_config = httpserver::load_config(cfg_defaults.clone());

    // ── Button control thread — always started (GPIO0 / BOOT button) ──────────
    // Very long press (≥ 10s): factory reset to cfg.toml defaults
    btn_ctrl::start_button_thread(peripherals.pins.gpio0);

    // ── Mini Display init (SSD1306, only when enabled) ────────────────────
    // GPIB-Server board: SDA = GPIO13, SCL = GPIO3 (GPIO46 is pulled down for boot strapping).
    if nvs_config.mini_display_enable == "true" {
        info!("Initializing mini display (SSD1306)...");
        let i2c_config = I2cConfig::new().baudrate(400.kHz().into());
        match I2cDriver::new(
            peripherals.i2c0,
            peripherals.pins.gpio13,  // SDA
            peripherals.pins.gpio3,   // SCL
            &i2c_config,
        ) {
            Ok(i2c) => {
                mini_display::start_mini_display_thread(i2c);
                info!("Mini display (SSD1306) initialized");
            }
            Err(e) => {
                info!("Failed to initialize I2C for mini display: {:?}", e);
            }
        }
    } else {
        info!("Mini display disabled");
    }

    // ── GPIB controller (SN75160B/SN75161B via SN74LVC245A) ───────────────
    // Pins are fixed by the board (see src/gpib.rs / doc/gpib-hardware.md).
    gpib::start(bridge::gpib_output, bridge::SCOPE_SINKS, nvs_config.gpib_addr.parse().unwrap_or(1));

    // WiFi connect (using NVS config — falls back to cfg.toml if not set)
    // When wps_enable=true and SSID is empty, WPS PBC is used to obtain credentials.
    let wifi_dev = if nvs_config.wps_enable == "true" && nvs_config.wifi_ssid.is_empty() {
        println!("WPS enabled and SSID not configured — starting WPS PBC (press WPS button on router)...");
        mini_display::show_message("WPS", "Connecting...\nPress WPS button\non router");
        let result = match wifi::wifi_connect_wps(peripherals.modem) {
            Ok((w, ssid, pass)) => {
                println!("WPS success: SSID={}", ssid);
                httpserver::nvs_write_wifi_creds(&ssid, &pass);
                nvs_config.wifi_ssid = ssid;
                nvs_config.wifi_psk  = pass;
                Ok(w)
            }
            Err(e) => {
                println!("WPS failed: {:?}", e);
                Err(e)
            }
        };
        mini_display::clear_message();
        result
    } else {
        println!("Connecting to WiFi: {}", nvs_config.wifi_ssid);
        wifi::wifi_connect(peripherals.modem, &nvs_config.wifi_ssid, &nvs_config.wifi_psk)
    };

    // Create shared status and populate IP/SSID after WiFi connects
    let dev_status = httpserver::StatusInfo::new();
    match &wifi_dev {
        Ok(w) => {
            // Apply static IP unconditionally (interface config, independent of connection state)
            if nvs_config.ip_mode == "static" {
                match wifi::configure_static_ip(w, &nvs_config.ip_address, &nvs_config.subnet_mask, &nvs_config.gateway, &nvs_config.dns) {
                    Ok(_) => println!("Static IP configured"),
                    Err(e) => println!("Failed to set static IP: {:?}, falling back to DHCP", e),
                }
            }
            if w.is_connected().unwrap_or(false) {
                println!("WiFi connected.");
                if let Ok(ip_info) = w.sta_netif().get_ip_info() {
                    println!("IP address: {}", ip_info.ip);
                    dev_status.set_wifi(
                        &ip_info.ip.to_string(),
                        &nvs_config.wifi_ssid,
                        wifi::get_rssi(),
                    );
                    if nvs_config.mini_display_enable == "true" {
                        let rssi = wifi::get_rssi() as i8;
                        mini_display::update_display_info(true, &ip_info.ip.to_string(), rssi, 0.0, gpib::current_addr());
                    }
                }
            } else {
                println!("WiFi initialized (AP not reachable yet — will retry every 30 s).");
                if nvs_config.mini_display_enable == "true" {
                    mini_display::update_display_info(false, "---", 0, 0.0, gpib::current_addr());
                }
            }
        },
        Err(e) => {
            println!("WiFi init failed: {:?}", e);
            if nvs_config.mini_display_enable == "true" {
                mini_display::update_display_info(false, "---", 0, 0.0, gpib::current_addr());
            }
        }
    }

    // Start WiFi monitor thread — reconnects every 30 s when not associated.
    // Start after a 15 s grace period so the initial connect attempt can settle.
    if wifi_dev.is_ok() {
        wifi::start_wifi_monitor_thread(15, 30);
    }

    // Start HTTP config server (login protected, NVS-backed settings)
    let config_state = httpserver::ConfigState::new(nvs_config.clone(), cfg_defaults, dev_status.clone());
    let _http_server = match httpserver::start_http_server(config_state) {
        Ok(s)  => { println!("HTTP config server started on port 80"); Some(s) }
        Err(e) => { println!("HTTP config server failed to start: {:?}", e); None }
    };

    // Start WebSocket sender thread (GPIB interpreter ⇄ browser terminal)
    bridge::start_ws_sender_thread();

    // Syslog init (after wifi is up)
    if nvs_config.syslog_enable == "true" {
        println!("Initializing syslog to {} ...", nvs_config.syslog_server);
        thread::sleep(Duration::from_secs(5));
        match syslogger::init_logger(&nvs_config.syslog_server, &nvs_config.syslog_enable, &nvs_config.syslog_host_name, &nvs_config.syslog_app_name) {
            Ok(_) => {
                log::set_max_level(log::LevelFilter::Info);
                println!("Syslog logger initialized successfully");
                info!("Syslog logger initialized successfully");
                if let Ok(w) = wifi_dev.as_ref() {
                    if let Ok(ip_info) = w.sta_netif().get_ip_info() {
                        info!("WiFi IP address: {}", ip_info.ip);
                    }
                }
            },
            Err(e) => {
                println!("Failed to initialize syslog: {:?}, falling back to ESP logger", e);
                let _ = esp_idf_svc::log::EspLogger::initialize_default();
                log::set_max_level(log::LevelFilter::Info);
            }
        }
    } else {
        esp_idf_svc::log::EspLogger::initialize_default();
        log::set_max_level(log::LevelFilter::Info);
        info!("Using default ESP console logger (syslog disabled)");
    }

    // NTP Server
    let ntp1: &'static str = Box::leak(nvs_config.ntp_server1.clone().into_boxed_str());
    let ntp2: &'static str = Box::leak(nvs_config.ntp_server2.clone().into_boxed_str());
    let ntp3: &'static str = Box::leak(nvs_config.ntp_server3.clone().into_boxed_str());
    let ntp4: &'static str = Box::leak(nvs_config.ntp_server4.clone().into_boxed_str());
    let sntp_conf = SntpConf {
        servers: [ntp1, ntp2, ntp3, ntp4],
        operating_mode: OperatingMode::Poll,
        sync_mode: SyncMode::Immediate,
    };
    let ntp = EspSntp::new(&sntp_conf).unwrap();

    // NTP Sync
    info!("NTP Sync Start..");

    // wait for sync
    let mut sync_count = 0;
    while ntp.get_sync_status() != SyncStatus::Completed {
        sync_count += 1;
        if sync_count > 1000 {
            info!("NTP Sync Timeout");
            break;
        }
        thread::sleep(Duration::from_millis(10));
    }
    let now = SystemTime::now();
    let dt_now : DateTime<Utc> = now.into();
    let formatted = format!("{}", dt_now.format("%Y-%m-%d %H:%M:%S"));
    info!("NTP Sync Completed: {}", formatted);

    // Start SSH server (GPIB console: `ssh -tt admin@host gpib`)
    // Leak NVS strings to obtain the &'static str required by the C bridge.
    let ssh_user_s: &'static str = Box::leak(nvs_config.ssh_user.clone().into_boxed_str());
    let ssh_pass_s: &'static str = Box::leak(nvs_config.ssh_password.clone().into_boxed_str());
    bridge::start_ssh(ssh_user_s, ssh_pass_s);
    boot_log::record_diag(&format!("boot #{} ok (ssh started)", boot_count));

    // ── Internal temperature sensor init ──────────────────────────────────
    let temp_cfg = TempSensorConfig::default();
    let mut temp_sensor = TempSensorDriver::new(&temp_cfg, peripherals.temp_sensor).unwrap();
    temp_sensor.enable().unwrap();

    let mut count = 0u32;
    loop {
        // Check connection state via EspWifi before calling get_rssi(),
        // because esp_wifi_sta_get_rssi crashes (LoadProhibited) when
        // WiFi is not connected.
        let is_connected = wifi_dev.as_ref()
            .map(|w| w.is_connected().unwrap_or(false))
            .unwrap_or(false);

        if count % 50 == 0 {
            let chip_temp = match temp_sensor.get_celsius() {
                Ok(t) => { dev_status.set_chip_temp(t); t },
                Err(_) => 0.0,
            };
            info!("Internal temp: {:.1} °C, GPIB addr: {}, SRQ: {}",
                  chip_temp, gpib::current_addr(), gpib::srq_asserted());

            // Update mini display
            if nvs_config.mini_display_enable == "true" {
                let ip = if is_connected {
                    wifi_dev.as_ref().ok()
                        .and_then(|w| w.sta_netif().get_ip_info().ok())
                        .map(|ip_info| ip_info.ip.to_string())
                        .unwrap_or_else(|| "---".to_string())
                } else {
                    "---".to_string()
                };
                let rssi = if is_connected { wifi::get_rssi() as i8 } else { 0 };
                mini_display::update_display_info(is_connected, &ip, rssi, chip_temp, gpib::current_addr());
            }
        }

        if !is_connected {
            // Reconnection is handled by the wifi_monitor background thread.
            // Nothing to do here except keep the status updated.
        } else {
            let rssi = wifi::get_rssi();
            dev_status.set_rssi(rssi);
            if count % 30 == 0 {
                if let Ok(w) = wifi_dev.as_ref() {
                    if let Ok(ip_info) = w.sta_netif().get_ip_info() {
                        dev_status.set_wifi(&ip_info.ip.to_string(), &nvs_config.wifi_ssid, rssi);
                    }
                }
            }
        }
        count = count.wrapping_add(1);
        thread::sleep(Duration::from_millis(100));
    }
}

