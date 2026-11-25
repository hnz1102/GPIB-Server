// Button control thread — GPIO0 (BOOT button, active-low, internal pull-up)
//
// Very long press  (≥ 10 s):  erase NVS and restart → restores cfg.toml defaults
//
// (The former short-press "display page" and medium-press "DC OUT toggle"
//  functions were removed: the GPIB-Server board has no SPI display, and
//  GPIO12 is now the GPIB TE (direction) line.)

use std::thread;
use std::time::{Duration, Instant};
use log::info;
use esp_idf_hal::gpio::{Gpio0, PinDriver, Pull};

/// Duration in milliseconds for a long-press factory reset (10 seconds).
const FACTORY_RESET_PRESS_MS: u64 = 10000;

/// Countdown is shown on the mini display after this many milliseconds.
const COUNTDOWN_START_MS: u64 = 5000;

/// Spawn the dedicated button-control thread.
pub fn start_button_thread(btn_pin: Gpio0) {
    thread::Builder::new()
        .name("btn_ctrl".into())
        .stack_size(4096)
        .spawn(move || button_task(btn_pin))
        .expect("spawn btn_ctrl thread");
}

fn button_task(btn_pin: Gpio0) {
    let mut btn = PinDriver::input(btn_pin).unwrap();
    btn.set_pull(Pull::Up).unwrap();

    let mut btn_last = true;           // true = released (pin high)
    let mut press_start: Option<Instant> = None;
    let mut last_shown: Option<u64> = None;

    loop {
        let btn_now = btn.is_high(); // high = released

        if !btn_now && btn_last {
            // ── Falling edge: button just pressed ───────────────────────
            press_start = Some(Instant::now());
            last_shown = None;
        } else if !btn_now {
            // ── Button held ─────────────────────────────────────────────
            if let Some(start) = press_start {
                let elapsed = start.elapsed().as_millis() as u64;

                if elapsed >= FACTORY_RESET_PRESS_MS {
                    info!("GPIO0 very long press ({}ms) — factory reset to cfg.toml defaults", elapsed);
                    crate::mini_display::show_message("FACTORY RESET", "Erasing NVS...\nPlease wait...");
                    thread::sleep(Duration::from_millis(500)); // give the display time to update
                    crate::httpserver::factory_reset(); // never returns
                } else if elapsed >= COUNTDOWN_START_MS {
                    let remaining = (FACTORY_RESET_PRESS_MS - elapsed + 999) / 1000;
                    if last_shown != Some(remaining) {
                        crate::mini_display::show_message(
                            "FACTORY RESET",
                            &format!("Reset in {}s...", remaining),
                        );
                        last_shown = Some(remaining);
                    }
                }
            }
        } else if btn_now && !btn_last {
            // ── Rising edge: button released ─────────────────────────────
            if last_shown.is_some() {
                crate::mini_display::clear_message();
            }
            press_start = None;
            last_shown = None;
        }

        btn_last = btn_now;
        thread::sleep(Duration::from_millis(50));
    }
}
