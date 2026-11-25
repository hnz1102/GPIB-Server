//! GPIB (IEEE-488) controller driver and Prologix-compatible command interpreter.
//!
//! Hardware (see doc/gpib-hardware.md and GPIB-Server.kicad_sch):
//!
//! ```text
//! ESP32-S3 ── SN74LVC245A (U4: DIO, U5: DAV/EOI/NRFD/NDAC) ── SN75160B (U7) / SN75161B (U8) ── GPIB
//! ```
//!
//! * TE (GPIO12) drives SN75160B/161B TE and both LVC245A DIR pins.
//!   TE = H: ESP32 talks (DIO/DAV/EOI out, NRFD/NDAC in).
//!   TE = L: ESP32 listens (DIO/DAV/EOI in, NRFD/NDAC out).
//! * OE# (GPIO2) enables both LVC245A (low = enabled).
//! * ATN/REN/IFC are always outputs, SRQ is always an input (SN75161B DC = GND,
//!   i.e. the ESP32 is the system controller).
//! * The bus is negative logic: a LOW level means "asserted" / data bit = 1.
//!
//! The ESP32 is always the controller-in-charge; device mode is not supported.

use esp_idf_sys as sys;
use log::{info, warn};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Mutex;
use std::thread;

// ─── Pin assignment (GPIB-Server.kicad_sch) ──────────────────────────────────

/// DIO1..DIO8 → GPIO4..GPIO11 (via U4 SN74LVC245A → U7 SN75160B).
const PIN_DIO: [i32; 8] = [4, 5, 6, 7, 8, 9, 10, 11];
/// Handshake lines (via U5 SN74LVC245A → U8 SN75161B).
const PIN_DAV: i32 = 15;
const PIN_NRFD: i32 = 16;
const PIN_NDAC: i32 = 17;
const PIN_EOI: i32 = 18;
/// Management lines (direct, 1 kΩ series → U8 SN75161B).
const PIN_ATN: i32 = 21;
const PIN_REN: i32 = 38;
const PIN_IFC: i32 = 39;
/// SRQ input (U8 → U6 SN74LVC1G34 → GPIO40).
const PIN_SRQ: i32 = 40;
/// Direction control: SN75160B/161B TE + LVC245A DIR.
const PIN_TE: i32 = 12;
/// LVC245A output enable (active low).
const PIN_OE_N: i32 = 2;

// ─── GPIB interface messages ─────────────────────────────────────────────────

const GTL: u8 = 0x01;
const SDC: u8 = 0x04;
const GET: u8 = 0x08;
const LLO: u8 = 0x11;
const SPE: u8 = 0x18;
const SPD: u8 = 0x19;
const UNL: u8 = 0x3F;
const UNT: u8 = 0x5F;

/// The controller's own primary address (used for MTA/MLA).
const MY_ADDR: u8 = 0;

fn lad(pad: u8) -> u8 { 0x20 | (pad & 0x1F) }
fn tad(pad: u8) -> u8 { 0x40 | (pad & 0x1F) }

// ─── Low-level pin helpers ───────────────────────────────────────────────────

#[inline(always)]
fn set_level(pin: i32, high: bool) {
    unsafe { sys::gpio_set_level(pin, high as u32); }
}

#[inline(always)]
fn level(pin: i32) -> bool {
    unsafe { sys::gpio_get_level(pin) != 0 }
}

/// Assert a GPIB line (drive LOW).
#[inline(always)]
fn assert_line(pin: i32) { set_level(pin, false); }

/// Release a GPIB line (drive HIGH = unasserted).
#[inline(always)]
fn release_line(pin: i32) { set_level(pin, true); }

/// True when the GPIB line is asserted (LOW).
#[inline(always)]
fn asserted(pin: i32) -> bool { !level(pin) }

fn pin_input(pin: i32) {
    unsafe {
        sys::gpio_set_direction(pin, sys::gpio_mode_t_GPIO_MODE_INPUT);
        // Internal pull-up: never leave an LVC245A I/O floating while the
        // direction changes (SN74LVC245A data sheet recommendation).
        sys::gpio_set_pull_mode(pin, sys::gpio_pull_mode_t_GPIO_PULLUP_ONLY);
    }
}

/// Configure a pin as output, setting the level *before* enabling the driver.
fn pin_output(pin: i32, high: bool) {
    set_level(pin, high);
    unsafe {
        sys::gpio_set_direction(pin, sys::gpio_mode_t_GPIO_MODE_OUTPUT);
    }
    set_level(pin, high);
}

#[inline(always)]
fn delay_us(us: u32) {
    unsafe { sys::esp_rom_delay_us(us); }
}

#[inline(always)]
fn now_us() -> i64 {
    unsafe { sys::esp_timer_get_time() }
}

// ─── Bus driver ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpibError {
    /// Handshake did not complete within the read/write timeout.
    Timeout(&'static str),
    /// Neither NRFD nor NDAC is asserted: nobody is listening.
    NoListener,
    /// Interrupted by the user (browser Stop, Ctrl-C) — see `abort()`.
    Aborted,
}

impl core::fmt::Display for GpibError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            GpibError::Timeout(what) => write!(f, "timeout waiting for {}", what),
            GpibError::NoListener => write!(f, "no listener on the bus (instrument powered off or wrong address?)"),
            GpibError::Aborted => write!(f, "aborted by user"),
        }
    }
}

type GResult<T> = Result<T, GpibError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    Unknown,
    Talk,
    Listen,
}

/// Read termination condition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadEnd {
    /// Stop when EOI is asserted with a byte.
    Eoi,
    /// Stop when this byte is received (or EOI).
    Char(u8),
    /// Read until timeout (Prologix `++read` without argument). EOI also ends it.
    Timeout,
}

/// GPIB address (primary + optional secondary 96..=126).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Addr {
    pub pad: u8,
    pub sad: Option<u8>,
}

pub struct Bus {
    dir: Dir,
    timeout_us: i64,
    ren_on: bool,
}

impl Bus {
    /// Configure all GPIB pins and take control of the bus (IFC + REN).
    fn new() -> Self {
        unsafe {
            for &p in PIN_DIO.iter().chain(
                [PIN_DAV, PIN_NRFD, PIN_NDAC, PIN_EOI, PIN_ATN, PIN_REN, PIN_IFC, PIN_SRQ, PIN_TE, PIN_OE_N].iter(),
            ) {
                sys::gpio_reset_pin(p);
            }
        }
        // Keep the LVC245A isolated while configuring (external pull-up holds OE# high too).
        pin_output(PIN_OE_N, true);
        pin_output(PIN_TE, true);
        pin_output(PIN_ATN, true);
        pin_output(PIN_REN, true);
        pin_output(PIN_IFC, true);
        pin_input(PIN_SRQ);

        let mut bus = Bus { dir: Dir::Unknown, timeout_us: 3_000_000, ren_on: false };
        bus.set_talk();
        bus.ifc();
        bus.set_ren(true);
        info!("[GPIB] bus initialised (controller-in-charge, address {})", MY_ADDR);
        bus
    }

    fn set_timeout_ms(&mut self, ms: u32) {
        self.timeout_us = ms as i64 * 1000;
    }

    /// Switch to talk mode (TE = H): DIO/DAV/EOI outputs, NRFD/NDAC inputs.
    fn set_talk(&mut self) {
        if self.dir == Dir::Talk {
            return;
        }
        set_level(PIN_OE_N, true); // isolate LVC245A while the direction changes
        set_level(PIN_TE, true);
        for &p in PIN_DIO.iter() {
            pin_output(p, true); // released (all data bits 0)
        }
        pin_output(PIN_DAV, true);
        pin_output(PIN_EOI, true);
        pin_input(PIN_NRFD);
        pin_input(PIN_NDAC);
        delay_us(2);
        set_level(PIN_OE_N, false);
        delay_us(2);
        self.dir = Dir::Talk;
    }

    /// Switch to listen mode (TE = L): DIO/DAV/EOI inputs, NRFD/NDAC outputs.
    /// NRFD and NDAC are left asserted (not ready / not accepted).
    fn set_listen(&mut self) {
        if self.dir == Dir::Listen {
            return;
        }
        set_level(PIN_OE_N, true);
        for &p in PIN_DIO.iter() {
            pin_input(p);
        }
        pin_input(PIN_DAV);
        pin_input(PIN_EOI);
        pin_output(PIN_NRFD, false);
        pin_output(PIN_NDAC, false);
        set_level(PIN_TE, false);
        delay_us(2);
        set_level(PIN_OE_N, false);
        delay_us(2);
        self.dir = Dir::Listen;
    }

    /// Busy-wait (yielding after 2 ms) until `cond` becomes true or the timeout expires.
    fn wait_for(&self, what: &'static str, cond: impl Fn() -> bool) -> GResult<()> {
        let start = now_us();
        loop {
            if cond() {
                return Ok(());
            }
            if ABORT.load(Ordering::Relaxed) {
                return Err(GpibError::Aborted);
            }
            let elapsed = now_us() - start;
            if elapsed > self.timeout_us {
                return Err(GpibError::Timeout(what));
            }
            if elapsed > 2000 {
                // Slow instrument: give other tasks a chance to run.
                unsafe { sys::vTaskDelay(1); }
            }
        }
    }

    /// Source handshake: send one byte (talk mode required).
    fn send_byte(&mut self, b: u8, eoi: bool) -> GResult<()> {
        // Wait until all acceptors are ready for data.
        self.wait_for("NRFD", || !asserted(PIN_NRFD))?;
        if !asserted(PIN_NDAC) {
            // NRFD and NDAC both released: there is no acceptor at all.
            return Err(GpibError::NoListener);
        }
        for (i, &p) in PIN_DIO.iter().enumerate() {
            // negative logic: bit 1 = LOW
            set_level(p, (b >> i) & 1 == 0);
        }
        if eoi {
            assert_line(PIN_EOI);
        }
        delay_us(2); // T1 settling time
        assert_line(PIN_DAV);
        let r = self.wait_for("NDAC", || !asserted(PIN_NDAC));
        release_line(PIN_DAV);
        release_line(PIN_EOI);
        r
    }

    /// Acceptor handshake: receive one byte (listen mode required).
    /// Returns (byte, EOI asserted).
    fn recv_byte(&mut self) -> GResult<(u8, bool)> {
        release_line(PIN_NRFD); // ready for data
        if let Err(e) = self.wait_for("DAV", || asserted(PIN_DAV)) {
            assert_line(PIN_NRFD);
            return Err(e);
        }
        assert_line(PIN_NRFD); // not ready for the next byte
        let mut b = 0u8;
        for (i, &p) in PIN_DIO.iter().enumerate() {
            if !level(p) {
                b |= 1 << i;
            }
        }
        let eoi = asserted(PIN_EOI);
        release_line(PIN_NDAC); // data accepted
        let r = self.wait_for("DAV release", || !asserted(PIN_DAV));
        assert_line(PIN_NDAC);
        r.map(|_| (b, eoi))
    }

    /// Assert ATN and send interface commands. ATN stays asserted afterwards.
    fn send_cmds(&mut self, cmds: &[u8]) -> GResult<()> {
        self.set_talk();
        assert_line(PIN_ATN);
        delay_us(2);
        for &c in cmds {
            self.send_byte(c, false)?;
        }
        Ok(())
    }

    /// Release ATN (end of command phase).
    fn release_atn(&mut self) {
        release_line(PIN_ATN);
        delay_us(2);
    }

    /// Take control synchronously after a read: assert ATN while still holding
    /// NRFD (so the talker cannot start another byte), then unaddress everyone.
    fn regain_control(&mut self) {
        assert_line(PIN_ATN);
        delay_us(2);
        self.set_talk();
        if let Err(e) = self.send_cmds(&[UNT, UNL]) {
            warn!("[GPIB] UNT/UNL failed: {}", e);
        }
        self.release_atn();
    }

    fn address_cmds(&self, first: &[u8], addr: Addr, talker: bool) -> Vec<u8> {
        let mut v = first.to_vec();
        v.push(if talker { tad(addr.pad) } else { lad(addr.pad) });
        if let Some(s) = addr.sad {
            v.push(s);
        }
        v
    }

    /// Bring the bus back to a clean idle state after an aborted transfer:
    /// release DAV/EOI/ATN, return to talk mode and send IFC so every device
    /// leaves its talker/listener state (e.g. an instrument stuck holding NRFD).
    fn recover(&mut self) {
        release_line(PIN_DAV);
        release_line(PIN_EOI);
        release_line(PIN_ATN);
        self.set_talk();
        self.ifc();
        warn!("[GPIB] transfer aborted — bus reset with IFC");
    }

    /// Interface clear: assert IFC for >100 µs.
    pub fn ifc(&mut self) {
        release_line(PIN_ATN);
        assert_line(PIN_IFC);
        delay_us(150);
        release_line(PIN_IFC);
        delay_us(10);
    }

    pub fn set_ren(&mut self, on: bool) {
        set_level(PIN_REN, !on);
        self.ren_on = on;
    }

    pub fn ren(&self) -> bool {
        self.ren_on
    }

    pub fn srq(&self) -> bool {
        asserted(PIN_SRQ)
    }

    /// Address `addr` as listener and send `data` (EOI on the last byte if `eoi`).
    pub fn write(&mut self, addr: Addr, data: &[u8], eoi: bool) -> GResult<()> {
        let cmds = self.address_cmds(&[UNL, tad(MY_ADDR)], addr, false);
        let r = (|| {
            self.send_cmds(&cmds)?;
            self.release_atn();
            let n = data.len();
            for (i, &b) in data.iter().enumerate() {
                self.send_byte(b, eoi && i + 1 == n)?;
            }
            Ok(())
        })();
        if r.is_err() {
            self.release_atn();
        }
        r
    }

    /// Address `addr` as talker and read until `end`, passing data chunks to `sink`.
    /// Returns Ok(true) when the read ended with EOI.
    pub fn read(&mut self, addr: Addr, end: ReadEnd, sink: &mut dyn FnMut(&[u8])) -> GResult<bool> {
        let cmds = self.address_cmds(&[UNL, UNT, lad(MY_ADDR)], addr, true);
        if let Err(e) = self.send_cmds(&cmds) {
            self.release_atn();
            return Err(e);
        }
        // Become an acceptor (NRFD/NDAC asserted) before releasing ATN, so the
        // talker cannot see a "no listener" bus.
        self.set_listen();
        self.release_atn();

        let mut chunk: Vec<u8> = Vec::with_capacity(256);
        let mut got_any = false;
        let result = loop {
            match self.recv_byte() {
                Ok((b, eoi)) => {
                    got_any = true;
                    chunk.push(b);
                    if chunk.len() >= 256 {
                        sink(&chunk);
                        chunk.clear();
                    }
                    if eoi {
                        break Ok(true);
                    }
                    if let ReadEnd::Char(c) = end {
                        if b == c {
                            break Ok(false);
                        }
                    }
                }
                Err(e) => {
                    if end == ReadEnd::Timeout && got_any {
                        break Ok(false);
                    }
                    break Err(e);
                }
            }
        };
        if !chunk.is_empty() {
            sink(&chunk);
        }
        self.regain_control();
        result
    }

    /// Serial poll one device; returns its status byte.
    pub fn serial_poll(&mut self, addr: Addr) -> GResult<u8> {
        let cmds = self.address_cmds(&[UNL, SPE, lad(MY_ADDR)], addr, true);
        if let Err(e) = self.send_cmds(&cmds) {
            self.release_atn();
            return Err(e);
        }
        self.set_listen();
        self.release_atn();
        let r = self.recv_byte();
        assert_line(PIN_ATN);
        delay_us(2);
        self.set_talk();
        if let Err(e) = self.send_cmds(&[SPD, UNT]) {
            warn!("[GPIB] SPD/UNT failed: {}", e);
        }
        self.release_atn();
        r.map(|(b, _)| b)
    }

    /// Send an addressed command (SDC, GET, GTL) to one device.
    fn addressed_cmd(&mut self, addr: Addr, cmd: u8) -> GResult<()> {
        let mut cmds = self.address_cmds(&[UNL], addr, false);
        cmds.push(cmd);
        let r = self.send_cmds(&cmds);
        self.release_atn();
        r
    }

    pub fn device_clear(&mut self, addr: Addr) -> GResult<()> { self.addressed_cmd(addr, SDC) }
    pub fn trigger(&mut self, addr: Addr) -> GResult<()> { self.addressed_cmd(addr, GET) }
    pub fn go_to_local(&mut self, addr: Addr) -> GResult<()> { self.addressed_cmd(addr, GTL) }

    pub fn local_lockout(&mut self) -> GResult<()> {
        let r = self.send_cmds(&[LLO]);
        self.release_atn();
        r
    }
}

// ─── Prologix-compatible command interpreter ─────────────────────────────────

const VERSION_STR: &str = concat!(
    "ESP32-S3 GPIB-Server version ",
    env!("CARGO_PKG_VERSION"),
    " (Prologix GPIB-ETHERNET compatible)"
);

const HELP: &str = "\
Prologix-compatible commands (controller mode):\r
  ++addr [pad [sad]]   Get/set instrument address (pad 0-30, sad 96-126)\r
  ++auto [0|1]         Read automatically after each write\r
  ++clr                Selected Device Clear (SDC)\r
  ++eoi [0|1]          Assert EOI with last byte of writes\r
  ++eos [0-3]          Write terminator: 0=CR+LF 1=CR 2=LF 3=none\r
  ++eot_enable [0|1]   Append eot_char to read data when EOI is seen\r
  ++eot_char [0-255]   Character appended when eot_enable=1\r
  ++ifc                Interface Clear\r
  ++llo                Local Lockout\r
  ++loc                Go To Local (GTL) for current address\r
  ++mode [1]           Controller mode only (always 1)\r
  ++read [eoi|<char>]  Read until EOI, <char> (decimal), or timeout\r
  ++read_tmo_ms [ms]   Read/handshake timeout (1-60000 ms)\r
  ++rst                Re-initialise the GPIB bus (IFC + REN)\r
  ++spoll [pad [sad]]  Serial poll\r
  ++srq                SRQ line state (1 = asserted)\r
  ++trg [pad ...]      Group Execute Trigger (GET)\r
  ++ver                Version\r
Extensions:\r
  ++echo [0|1]         Local echo / CRLF conversion (interactive terminals)\r
  ++ren [0|1]          REN line\r
Lines not starting with ++ are sent to the instrument.\r
Escape CR, LF, ESC or '+' in data with ESC (0x1B).\r
";

struct Interp {
    bus: Bus,
    addr: Addr,
    auto: bool,
    eoi: bool,
    eos: u8,
    eot_enable: bool,
    eot_char: u8,
    read_tmo_ms: u32,
    echo: bool,
    /// Current input line: (byte, escaped)
    line: Vec<(u8, bool)>,
    esc_pending: bool,
    /// Previous input byte was CR (so a following LF is not echoed twice).
    prev_cr: bool,
}

/// Output sink function (installed by the bridge module).
type OutFn = fn(&[u8]);

impl Interp {
    fn new(bus: Bus, pad: u8) -> Self {
        let mut s = Interp {
            bus,
            addr: Addr { pad: if pad <= 30 { pad } else { 1 }, sad: None },
            auto: true,
            eoi: true,
            eos: 2,
            eot_enable: false,
            eot_char: b'\n',
            read_tmo_ms: 3000,
            echo: true,
            line: Vec::new(),
            esc_pending: false,
            prev_cr: false,
        };
        s.bus.set_timeout_ms(s.read_tmo_ms);
        CURRENT_ADDR.store(s.addr.pad, Ordering::Relaxed);
        s
    }

    /// Interactive sessions get echo + auto-read; raw sessions behave like a
    /// Prologix adapter (no echo, ++auto 0).
    fn set_interactive(&mut self, on: bool) {
        self.echo = on;
        self.auto = on;
        INTERACTIVE.store(on, Ordering::Relaxed);
    }

    fn reply(&self, o: OutFn, s: &str) {
        let mut v = s.as_bytes().to_vec();
        v.extend_from_slice(b"\r\n");
        o(&v);
    }

    fn error(&self, o: OutFn, s: &str) {
        warn!("[GPIB] {}", s);
        // Prologix adapters are silent on errors; only tell interactive users.
        if self.echo {
            self.reply(o, &format!("Error: {}", s));
        }
    }

    fn feed(&mut self, o: OutFn, data: &[u8]) {
        for &b in data {
            let was_cr = self.prev_cr;
            self.prev_cr = b == b'\r';
            if self.esc_pending {
                self.esc_pending = false;
                self.line.push((b, true));
                if self.echo {
                    o(&[b]);
                }
                continue;
            }
            match b {
                0x1B => self.esc_pending = true,
                b'\r' | b'\n' => {
                    // Echo a newline for CR, and for LF unless it follows CR.
                    if self.echo && (b == b'\r' || !was_cr) {
                        o(b"\r\n");
                    }
                    if !self.line.is_empty() {
                        let line = core::mem::take(&mut self.line);
                        self.process_line(o, &line);
                    }
                }
                0x08 | 0x7F if self.echo => {
                    if self.line.pop().is_some() {
                        o(b"\x08 \x08");
                    }
                }
                0x03 if self.echo => {
                    // Ctrl-C: discard the current line
                    self.line.clear();
                    o(b"^C\r\n");
                }
                _ => {
                    self.line.push((b, false));
                    if self.echo {
                        o(&[b]);
                    }
                }
            }
        }
    }

    fn process_line(&mut self, o: OutFn, line: &[(u8, bool)]) {
        let is_cmd = line.len() >= 2 && line[0] == (b'+', false) && line[1] == (b'+', false);
        if is_cmd {
            let text: String = line[2..].iter().map(|&(b, _)| b as char).collect();
            self.command(o, text.trim());
        } else {
            let mut data: Vec<u8> = line.iter().map(|&(b, _)| b).collect();
            match self.eos {
                0 => data.extend_from_slice(b"\r\n"),
                1 => data.push(b'\r'),
                2 => data.push(b'\n'),
                _ => {}
            }
            if let Err(e) = self.bus.write(self.addr, &data, self.eoi) {
                self.error(o, &format!("write to {}: {}", self.addr.pad, e));
                return;
            }
            if self.auto {
                // Interactive: also stop at LF for instruments that do not assert
                // EOI. Raw (Prologix-style) sessions read until EOI so binary
                // blocks containing 0x0A are not truncated.
                let end = if self.echo { ReadEnd::Char(b'\n') } else { ReadEnd::Eoi };
                self.do_read(o, end, true);
            }
        }
    }

    fn do_read(&mut self, o: OutFn, end: ReadEnd, quiet_timeout: bool) {
        let addr = self.addr;
        let echo = self.echo;
        let mut got_any = false;
        let mut prev = 0u8;
        let mut sink = |chunk: &[u8]| {
            got_any = true;
            if echo {
                let mut v = Vec::with_capacity(chunk.len() + 8);
                for &b in chunk {
                    if b == b'\n' && prev != b'\r' {
                        v.push(b'\r');
                    }
                    v.push(b);
                    prev = b;
                }
                o(&v);
            } else {
                o(chunk);
            }
        };
        let r = self.bus.read(addr, end, &mut sink);
        match r {
            Ok(eoi) => {
                if eoi && self.eot_enable {
                    o(&[self.eot_char]);
                }
                if self.echo && got_any && prev != b'\n' {
                    o(b"\r\n");
                }
            }
            Err(GpibError::Timeout(_)) if quiet_timeout && !got_any => {
                // ++auto after a non-query command: nothing to read.
            }
            Err(e) => {
                if self.echo && got_any && prev != b'\n' {
                    o(b"\r\n");
                }
                self.error(o, &format!("read from {}: {}", addr.pad, e));
            }
        }
    }

    fn parse_addr(&self, args: &[&str]) -> Result<Addr, String> {
        let pad: u8 = args[0].parse().map_err(|_| format!("invalid address '{}'", args[0]))?;
        if pad > 30 {
            return Err("primary address must be 0-30".into());
        }
        let sad = match args.get(1) {
            Some(s) => {
                let v: u8 = s.parse().map_err(|_| format!("invalid secondary address '{}'", s))?;
                if !(96..=126).contains(&v) {
                    return Err("secondary address must be 96-126".into());
                }
                Some(v)
            }
            None => None,
        };
        Ok(Addr { pad, sad })
    }

    fn flag(&self, o: OutFn, args: &[&str], cur: bool) -> Option<bool> {
        match args.first() {
            None => {
                self.reply(o, if cur { "1" } else { "0" });
                None
            }
            Some(&"0") => Some(false),
            Some(&"1") => Some(true),
            Some(v) => {
                self.error(o, &format!("invalid value '{}' (expected 0 or 1)", v));
                None
            }
        }
    }

    fn command(&mut self, o: OutFn, text: &str) {
        let mut parts = text.split_whitespace();
        let cmd = parts.next().unwrap_or("").to_ascii_lowercase();
        let args: Vec<&str> = parts.collect();
        match cmd.as_str() {
            "addr" => {
                if args.is_empty() {
                    match self.addr.sad {
                        Some(s) => self.reply(o, &format!("{} {}", self.addr.pad, s)),
                        None => self.reply(o, &format!("{}", self.addr.pad)),
                    }
                } else {
                    match self.parse_addr(&args) {
                        Ok(a) => {
                            self.addr = a;
                            CURRENT_ADDR.store(a.pad, Ordering::Relaxed);
                        }
                        Err(e) => self.error(o, &e),
                    }
                }
            }
            "auto" => {
                if let Some(v) = self.flag(o, &args, self.auto) {
                    self.auto = v;
                }
            }
            "eoi" => {
                if let Some(v) = self.flag(o, &args, self.eoi) {
                    self.eoi = v;
                }
            }
            "eot_enable" => {
                if let Some(v) = self.flag(o, &args, self.eot_enable) {
                    self.eot_enable = v;
                }
            }
            "echo" => {
                if let Some(v) = self.flag(o, &args, self.echo) {
                    self.echo = v;
                    INTERACTIVE.store(v, Ordering::Relaxed);
                }
            }
            "eos" => match args.first() {
                None => self.reply(o, &self.eos.to_string()),
                Some(v) => match v.parse::<u8>() {
                    Ok(n) if n <= 3 => self.eos = n,
                    _ => self.error(o, "eos must be 0-3"),
                },
            },
            "eot_char" => match args.first() {
                None => self.reply(o, &self.eot_char.to_string()),
                Some(v) => match v.parse::<u8>() {
                    Ok(n) => self.eot_char = n,
                    _ => self.error(o, "eot_char must be 0-255"),
                },
            },
            "read_tmo_ms" => match args.first() {
                None => self.reply(o, &self.read_tmo_ms.to_string()),
                Some(v) => match v.parse::<u32>() {
                    Ok(n) if (1..=60000).contains(&n) => {
                        self.read_tmo_ms = n;
                        self.bus.set_timeout_ms(n);
                    }
                    _ => self.error(o, "read_tmo_ms must be 1-60000"),
                },
            },
            "read" => {
                let end = match args.first() {
                    None => ReadEnd::Timeout,
                    Some(a) if a.eq_ignore_ascii_case("eoi") => ReadEnd::Eoi,
                    Some(a) => match a.parse::<u8>() {
                        Ok(c) => ReadEnd::Char(c),
                        Err(_) => {
                            self.error(o, "usage: ++read [eoi|<char>]");
                            return;
                        }
                    },
                };
                self.do_read(o, end, false);
            }
            "clr" => {
                if let Err(e) = self.bus.device_clear(self.addr) {
                    self.error(o, &format!("clr: {}", e));
                }
            }
            "trg" => {
                let targets: Vec<Addr> = if args.is_empty() {
                    vec![self.addr]
                } else {
                    let mut v = Vec::new();
                    for a in &args {
                        match self.parse_addr(&[a]) {
                            Ok(ad) => v.push(ad),
                            Err(e) => {
                                self.error(o, &e);
                                return;
                            }
                        }
                    }
                    v
                };
                for t in targets {
                    if let Err(e) = self.bus.trigger(t) {
                        self.error(o, &format!("trg {}: {}", t.pad, e));
                    }
                }
            }
            "loc" => {
                if let Err(e) = self.bus.go_to_local(self.addr) {
                    self.error(o, &format!("loc: {}", e));
                }
            }
            "llo" => {
                if let Err(e) = self.bus.local_lockout() {
                    self.error(o, &format!("llo: {}", e));
                }
            }
            "ifc" => self.bus.ifc(),
            "ren" => {
                if let Some(v) = self.flag(o, &args, self.bus.ren()) {
                    self.bus.set_ren(v);
                }
            }
            "rst" => {
                self.bus.set_talk();
                self.bus.ifc();
                self.bus.set_ren(true);
            }
            "spoll" => {
                let addr = if args.is_empty() {
                    self.addr
                } else {
                    match self.parse_addr(&args) {
                        Ok(a) => a,
                        Err(e) => {
                            self.error(o, &e);
                            return;
                        }
                    }
                };
                match self.bus.serial_poll(addr) {
                    Ok(sb) => self.reply(o, &sb.to_string()),
                    Err(e) => self.error(o, &format!("spoll {}: {}", addr.pad, e)),
                }
            }
            "srq" => self.reply(o, if self.bus.srq() { "1" } else { "0" }),
            "mode" => match args.first() {
                None | Some(&"1") => {
                    if args.is_empty() {
                        self.reply(o, "1");
                    }
                }
                _ => self.error(o, "only controller mode (1) is supported"),
            },
            "savecfg" => {
                // Accepted for compatibility; settings are per boot.
            }
            "ver" => self.reply(o, VERSION_STR),
            "help" => o(HELP.as_bytes()),
            "" => {}
            other => self.error(o, &format!("unknown command '++{}' (try ++help)", other)),
        }
    }
}

// ─── Spectrum view (ADVANTEST R3267 / R3200 series) ──────────────────────────
//
// Continuous binary trace capture for the browser spectrum page:
//   CF/SP/RB set the conditions, DL2 makes TBA? a pure 2 x N byte block (EOI at
//   end), and every frame is  TS (take sweep, waits for completion) + TBA?.
// Raw values: 1792 (bottom graticule) .. 14592 (top = reference level), high
// byte first. See R3267 Operation Manual Vol.1, 5.2.10.3.

/// Conditions requested by the browser.
#[derive(Debug, Clone, Copy)]
pub struct ScopeParams {
    pub addr: u8,
    /// `None` keeps the instrument's current setting.
    pub cf_mhz: Option<f64>,
    pub span_mhz: Option<f64>,
    pub rbw_khz: Option<f64>,
}

impl ScopeParams {
    pub fn validate(&self) -> Result<(), &'static str> {
        fn ok(v: Option<f64>, lo: f64, hi: f64) -> bool {
            v.map_or(true, |x| x.is_finite() && (lo..=hi).contains(&x))
        }
        if self.addr > 30 {
            return Err("GPIB address must be 0-30");
        }
        if !ok(self.cf_mhz, 0.0, 8000.0) {
            return Err("center frequency must be 0-8000 MHz");
        }
        if !ok(self.span_mhz, 0.0, 8000.0) {
            return Err("span must be 0-8000 MHz");
        }
        if !ok(self.rbw_khz, 0.001, 10000.0) {
            return Err("RBW must be 0.001-10000 kHz");
        }
        Ok(())
    }
}

/// Argument rules for a front-panel code.
struct PanelCode {
    code: &'static str,
    /// The code may be sent without an argument.
    bare: bool,
    /// `<number>[unit]` argument allowed.
    num: bool,
    /// Keyword arguments allowed (e.g. ON / OFF).
    kw: &'static [&'static str],
}

const fn pc(code: &'static str, bare: bool, num: bool, kw: &'static [&'static str]) -> PanelCode {
    PanelCode { code, bare, num, kw }
}

const ONOFF: &[&str] = &["ON", "OFF"];
const DETS: &[&str] = &["NRM", "POS", "NEG", "SMP"];
const REGS: &[&str] = &["REG_00", "REG_01", "REG_02", "REG_03", "REG_04", "REG_05",
                        "REG_06", "REG_07", "REG_08", "REG_09", "REG_10"];
const TRAB: &[&str] = &["TRA", "TRB"];
const STDEF: &[&str] = &["USR", "MNL", "DEF"];

/// Front-panel codes accepted from the browser panel (R3267 GPIB codes,
/// Operation Manual Vol.1 tables 5-1 … 5-21). Anything else is rejected, in
/// particular DL0-4 (delimiter), TAA/TBA (trace input) and printer / file /
/// memory-card / table-entry commands.
/// Fields: code, bare (no argument), num (<number>[unit]), keywords.
const PANEL_CODES: &[PanelCode] = &[
    // FREQ (5-7)
    pc("CF", false, true, &[]), pc("FA", false, true, &[]), pc("FB", false, true, &[]),
    pc("CS", false, true, &[]), pc("CA", true, false, &[]),
    pc("FO", false, true, ONOFF), pc("PPA", true, false, &[]), pc("PPM", false, true, &[]),
    pc("PRESL", false, false, &["STD", "EXTD"]), pc("BND", false, true, &[]), pc("BNDLC", false, false, ONOFF),
    // SPAN (5-16)
    pc("SP", false, true, &[]), pc("FS", true, false, &[]), pc("ZS", true, false, &[]), pc("LS", true, false, &[]),
    // LEVEL (5-8)
    pc("RL", false, true, &[]), pc("DD", false, true, &[]),
    pc("LL1", true, false, &[]), pc("LL2", true, false, &[]), pc("LL5", true, false, &[]), pc("LL10", true, false, &[]),
    pc("AUNITS", false, false, &["DBM", "DBMV", "DBUV", "DBEMF", "DBPW", "W", "V"]),
    pc("RO", false, true, ONOFF), pc("CR", false, false, ONOFF),
    // ATT (5-2)
    pc("AT", false, true, &[]), pc("AA", true, false, &[]), pc("ATMIN", false, true, ONOFF),
    // COUPLE (5-5)
    pc("RB", false, true, &[]), pc("BA", true, false, &[]), pc("VB", false, true, &[]), pc("VA", true, false, &[]),
    pc("SW", false, true, &[]), pc("AS", true, false, &[]), pc("AL", true, false, &[]),
    pc("CORS", false, true, ONOFF), pc("COVR", false, true, ONOFF),
    pc("PLLBW", false, false, &["AUTO", "WIDE", "MID", "NARW"]), pc("DRBW", false, false, ONOFF),
    // SWP / SINGLE (5-18)
    pc("CONTS", true, false, &[]), pc("SNGLS", true, false, &[]), pc("SR", true, false, &[]),
    pc("TRGSRC", false, false, &["FREE", "LINE", "VIDEO", "EXT"]),
    pc("TRGSLP", false, false, &["FALL", "RISE"]), pc("TRGLVL", false, true, &[]), pc("TRGDT", false, true, &[]),
    pc("GTSWP", false, false, ONOFF), pc("WDOSWP", false, false, ONOFF),
    pc("GTPOS", false, true, &[]), pc("GTWID", false, true, &[]),
    pc("GTSRC", false, false, &["IF", "EXT", "EGT", "RF"]),
    // TRACE A (5-1)
    pc("AW", true, false, &[]), pc("AV", true, false, &[]), pc("AB", true, false, &[]), pc("CWA", true, false, &[]),
    pc("AMAX", false, false, ONOFF), pc("AMIN", false, false, ONOFF), pc("AAVG", false, false, ONOFF),
    pc("ANORM", false, false, ONOFF), pc("AR", true, false, &[]), pc("APAVG", false, false, ONOFF),
    pc("DET", false, false, DETS), pc("DETSEL", false, false, &["AUTO", "MNL"]), pc("ACTRC", false, false, TRAB),
    pc("ANLG", false, false, ONOFF), pc("ANLGDSP", false, false, &["PAUS", "CONT"]), pc("ANLGTM", false, true, &[]),
    pc("XYCSR", false, false, ONOFF), pc("CSRX", false, true, &[]), pc("CSRY", false, true, &[]),
    pc("SETANC", true, false, &[]), pc("RLSANC", true, false, &[]),
    // TRACE B (5-1)
    pc("BW", true, false, &[]), pc("BV", true, false, &[]), pc("BB", true, false, &[]), pc("CWB", true, false, &[]),
    pc("BMAX", false, false, ONOFF), pc("BMIN", false, false, ONOFF), pc("BAVG", false, false, ONOFF),
    pc("BNORM", false, false, ONOFF), pc("BR", true, false, &[]), pc("BPAVG", false, false, ONOFF),
    pc("DETB", false, false, DETS), pc("ACHB", true, false, &[]), pc("SWPCNT", false, true, &[]),
    // FORMAT (5-6)
    pc("DL", false, true, ONOFF), pc("LMTA", false, false, ONOFF), pc("LMTB", false, false, ONOFF),
    pc("PFC", false, false, ONOFF), pc("LIMTYP", false, false, &["FREQ", "TIME"]),
    pc("LARNG", false, false, &["ABOVE", "BELOW"]), pc("LBRNG", false, false, &["ABOVE", "BELOW"]),
    pc("LIMPOS", false, false, &["ABS", "REL"]), pc("LIMXREF", false, false, &["LEFT", "CENT"]),
    pc("LIMAPOS", false, false, &["ABS", "REL"]), pc("LIMYREF", false, false, &["TOP", "BOTM"]),
    pc("LIMSFT", false, true, &[]), pc("LIMASFT", false, true, &[]), pc("LMSFAT", true, false, &[]),
    pc("LOF", true, false, &[]), pc("RESPOS", false, false, &["LOW", "UP"]),
    // WINDOW (5-20)
    pc("WDO", false, false, ONOFF), pc("WLX", false, true, &[]), pc("WDX", false, true, &[]),
    pc("MLTSCR", false, false, &["ZM", "FT", "TT", "OFF"]), pc("ZMON", true, false, &[]), pc("ZMOFF", true, false, &[]),
    pc("ZMPOS", false, true, &[]), pc("ZMWID", false, true, &[]),
    pc("PKZMX", true, false, &[]), pc("PKZMN", true, false, &[]), pc("PKZM3", true, false, &[]),
    pc("SCRSEL", false, false, TRAB),
    // MKR (5-10)
    pc("MK", true, true, &[]), pc("MKOFF", true, false, &[]), pc("MKD", true, true, &[]),
    pc("FX", false, false, ONOFF), pc("REDLT", false, false, ONOFF), pc("SG", false, false, ONOFF),
    pc("MPM", false, true, &[]), pc("MPA", true, false, &[]),
    pc("CPLMK", false, false, &["OFF", "DLT", "ANC", "LLIN", "DLIN", "TRA"]), pc("MKTRACE", false, false, TRAB),
    pc("MLT", false, false, ONOFF), pc("PLS", false, false, &["FREQ", "LEVEL", "OFF"]),
    // SRCH (5-17)
    pc("PS", true, false, &[]), pc("NXP", true, false, &[]), pc("NXL", true, false, &[]),
    pc("NXR", true, false, &[]), pc("MIS", true, false, &[]), pc("NXM", true, false, &[]),
    pc("CP", false, false, ONOFF), pc("MKSX", false, false, &["ALL", "IN", "OUT"]),
    pc("MKSY", false, false, &["ALL", "DLIN", "LLIN"]), pc("DY", false, true, &[]),
    // search condition: X window (position / width, coupled to F), Y above / below
    pc("MKSPOS", false, true, &[]), pc("MKSWID", false, true, &[]), pc("MKSCPL", false, false, ONOFF),
    pc("MKSYDL", false, false, &["ABOVE", "BELOW"]), pc("MKSYLA", false, false, &["ABOVE", "BELOW"]),
    pc("MKSYLB", false, false, &["ABOVE", "BELOW"]),
    // MKR-> (5-11)
    pc("MKCF", true, false, &[]), pc("MKRL", true, false, &[]), pc("MKCS", true, false, &[]),
    pc("PKCF", true, false, &[]), pc("PKRL", true, false, &[]), pc("MTSP", true, false, &[]),
    pc("MTCF", true, false, &[]), pc("MTCS", true, false, &[]), pc("MKMKS", true, false, &[]), pc("MTMKS", true, false, &[]),
    // MEAS (5-9)
    pc("MKBW", false, true, &[]), pc("XDB", true, false, &[]), pc("XDL", true, false, &[]), pc("XDR", true, false, &[]),
    pc("DC0", true, false, &[]), pc("DC1", true, false, &[]), pc("DC2", true, false, &[]), pc("CDB", false, false, ONOFF),
    pc("COUNT", false, false, ONOFF), pc("CN0", true, false, &[]), pc("CN1", true, false, &[]),
    pc("CN2", true, false, &[]), pc("CN3", true, false, &[]),
    pc("NIM", true, false, &[]), pc("NIU", true, false, &[]), pc("NIC", true, false, &[]), pc("NIF", true, false, &[]),
    pc("FXPK", true, false, &[]), pc("PKTHIRD", true, false, &[]), pc("AMMOD", false, false, ONOFF),
    // POWER (5-12)
    pc("PWTM", false, true, &[]), pc("PWCH", true, false, &[]), pc("PWTOTAL", true, false, &[]),
    pc("PWAVG", true, false, &[]), pc("PWM", true, false, &[]),
    pc("ACP", false, false, ONOFF), pc("ACPAVG", false, true, ONOFF), pc("ACPST", false, false, STDEF),
    pc("ACPSCR", false, false, &["FULL", "SEPA", "CARR"]), pc("ADG", false, false, ONOFF),
    pc("SYMRT", false, true, &[]), pc("RFACT", false, true, &[]), pc("NQST", false, false, ONOFF),
    pc("ACPBSW", false, false, ONOFF),
    // UTIL (5-19)
    pc("OBW", false, false, ONOFF), pc("OBWPER", false, true, &[]), pc("AVGOBW", false, true, ONOFF),
    pc("OBWST", false, false, STDEF),
    pc("HARM", false, false, ONOFF), pc("HRMNUM", false, true, &[]), pc("HRMFND", false, true, ONOFF),
    pc("SPURI", false, false, ONOFF), pc("SPRTBL", false, true, &[]),
    pc("EYEOPN", false, false, ONOFF), pc("EYESMP", false, true, &[]), pc("EYEAMPM", true, false, &[]),
    pc("EYECOR", false, false, ONOFF),
    pc("CNIS", false, false, ONOFF), pc("CNSIG", false, false, ONOFF), pc("CNAVG", false, true, &[]),
    pc("PJIT", false, false, ONOFF), pc("PJSRTO", false, true, &[]), pc("PJSTPO", false, true, &[]),
    pc("PJSIG", false, false, ONOFF), pc("PJAVG", false, true, &[]),
    pc("IMM", false, false, ONOFF), pc("IMODR", false, true, &[]), pc("IMLS3", false, true, &[]),
    pc("IMLS5", false, true, &[]), pc("IMLS7", false, true, &[]), pc("IMLS9", false, true, &[]),
    pc("IMPFC", false, false, ONOFF), pc("IMAVG", false, true, &[]), pc("IMHS", false, false, ONOFF),
    // CAL (5-3) — calibration runs; 10 MHz reference adjustment is excluded
    pc("CLALL", true, false, &[]), pc("CLGAIN", true, false, &[]), pc("CLATT", true, false, &[]),
    pc("CLSTEP", true, false, &[]), pc("CLRBW", true, false, &[]), pc("CLLOG", true, false, &[]),
    pc("CLMAG", true, false, &[]), pc("CLPBW", true, false, &[]),
    pc("FC", false, false, ONOFF), pc("CC", false, false, ONOFF),
    // CONFIG (5-4) — printer / file output excluded
    pc("TPS", true, false, &[]), pc("TPL", true, false, &[]), pc("ANNOT", false, false, ONOFF),
    // PRESET / SAVE / RCL (5-13 … 5-15) — internal registers only
    pc("IP", true, false, &[]), pc("SV", false, false, REGS), pc("RC", false, false, REGS), pc("DEL", false, false, REGS),
];

/// Read-only queries the browser panel may issue (soft-key states and
/// measurement results). Queries whose first reply line is a count n,
/// followed by n lines, are listed in MULTI_QUERIES.
const PANEL_QUERIES: &[&str] = &[
    // states
    "ATMIN?", "ATMINON?", "FO?", "FOON?", "PRESL?", "BND?", "BNDLC?", "PPM?",
    "LL?", "AUNITS?", "RO?", "ROON?", "CR?",
    "CORS?", "CORSON?", "COVR?", "COVRON?", "PLLBW?", "DRBW?",
    "TRGSLP?", "TRGLVL?", "TRGDT?", "GTSWP?", "WDOSWP?", "GTPOS?", "GTWID?", "GTSRC?",
    "ANORM?", "APAVG?", "BNORM?", "BPAVG?", "DETSEL?", "ACTRC?", "ANLG?", "XYCSR?", "CSRX?", "CSRY?",
    "LMTA?", "LMTB?", "PFC?", "LIMTYP?", "LARNG?", "LBRNG?", "LIMPOS?", "LIMAPOS?", "RESPOS?",
    "WLX?", "WDX?", "ZMPOS?", "ZMWID?", "SCRSEL?",
    "FX?", "REDLT?", "SG?", "MPM?", "MPA?", "CPLMK?", "MKTRACE?", "MLT?",
    "CP?", "MKSX?", "MKSY?", "DY?", "MKSPOS?", "MKSWID?", "MKSCPL?", "MKSYDL?", "MKSYLA?", "MKSYLB?",
    "MKBW?", "DC?", "CDB?", "COUNT?", "CN?", "NION?", "AMMODON?",
    "PWTM?", "PWCHON?", "PWTOTALON?", "PWAVGON?", "ACPON?", "ACPAVG?", "ACPST?", "ACPSCR?", "ADG?",
    "SYMRT?", "RFACT?", "NQST?", "ACPBSW?",
    "OBWON?", "OBWPER?", "AVGOBW?", "OBWST?", "HARMON?", "HRMNUM?", "HRMFND?", "HRMFNDON?",
    "SPURION?", "SPRTBL?", "EYEOPNON?", "EYESMP?", "EYECOR?", "CNISON?", "CNSIG?", "CNAVG?",
    "PJITON?", "PJSRTO?", "PJSTPO?", "PJSIG?", "PJAVG?",
    "IMMON?", "IMODR?", "IMLS3?", "IMLS5?", "IMLS7?", "IMLS9?", "IMPFC?", "IMAVG?", "IMHS?",
    "FC?", "CC?", "TP?", "ANNOT?", "ERRNO?",
    // results
    "MF?", "ML?", "MFL?", "MDF1?", "MDL1?", "MDF2?", "MDL2?", "CSRDX?", "CSRDY?",
    "CNRES?", "NIRES?", "AMMOD?", "PFJ?", "OPF?",
    "PWCH?", "PWTOTAL?", "PWAVG?", "ACP?", "ACPREF?", "CARRBS?",
    "OBW?", "HARM?", "EYEOPN?", "CNIS?", "PJIT?", "IMMREF?", "IMMDF?", "IMMRES?",
    "PKLST?", "MLSF?", "MLSL?", "FPU?", "FPL?",
];
const MULTI_QUERIES: &[&str] = &["PKLST?", "ACP?", "HARM?", "CNIS?", "IMMRES?", "FPU?", "FPL?"];
/// Timeout for panel queries (a mode that is off may not answer).
const PANEL_QUERY_TIMEOUT_MS: u32 = 1500;

pub fn validate_panel_query(q: &str) -> Result<(), &'static str> {
    if PANEL_QUERIES.contains(&q) { Ok(()) } else { Err("query not allowed from the panel") }
}

/// Validate a panel command: "<CODE>", "<CODE> <number><unit>" or "<CODE> <KEYWORD>",
/// according to PANEL_CODES.
pub fn validate_panel_cmd(text: &str) -> Result<(), &'static str> {
    let text = text.trim();
    if text.is_empty() || text.len() > 40 {
        return Err("empty or too long command");
    }
    let mut it = text.split_whitespace();
    let code = it.next().unwrap_or("");
    let rule = PANEL_CODES.iter().find(|r| r.code == code)
        .ok_or("command not allowed from the panel")?;
    let arg = it.next();
    if it.next().is_some() {
        return Err("too many arguments");
    }
    match arg {
        None if rule.bare => Ok(()),
        None => Err("argument required"),
        Some(a) if rule.kw.contains(&a) => Ok(()),
        Some(a) if rule.num => {
            let unit_at = a.find(|c: char| c.is_ascii_alphabetic()).unwrap_or(a.len());
            let (num, unit) = a.split_at(unit_at);
            let num_ok = !num.is_empty()
                && num.chars().all(|c| c.is_ascii_digit() || matches!(c, '.' | '-' | '+'))
                && num.parse::<f64>().is_ok();
            let unit_ok = matches!(unit, "" | "GZ" | "MZ" | "KZ" | "HZ" | "DB" | "SC" | "MS" | "US");
            if num_ok && unit_ok { Ok(()) } else { Err("argument must be <number>[GZ|MZ|KZ|HZ|DB|SC|MS|US]") }
        }
        Some(_) => Err("invalid argument for this command"),
    }
}

// ─── SCPI instrument pages: DC power supply (HP 6612B), DMM (HP 34401A) ─────

/// Instrument kind of a /ws/inst request: selects the command allowlist,
/// the bus timeout and the reply "type".
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Dev {
    Psu,
    Dmm,
}

impl Dev {
    pub fn from_name(s: &str) -> Option<Dev> {
        match s { "psu" => Some(Dev::Psu), "dmm" => Some(Dev::Dmm), _ => None }
    }
    fn name(self) -> &'static str {
        match self { Dev::Psu => "psu", Dev::Dmm => "dmm" }
    }
    fn cmds(self) -> &'static [(&'static str, PsuArg)] {
        match self { Dev::Psu => PSU_CMDS, Dev::Dmm => DMM_CMDS }
    }
    fn queries(self) -> &'static [&'static str] {
        match self { Dev::Psu => PSU_QUERIES, Dev::Dmm => DMM_QUERIES }
    }
    /// Bus timeout: a 34401A READ? at 100 NPLC with autozero takes ~3.5 s
    /// (longer while autoranging).
    fn timeout_ms(self) -> u32 {
        match self { Dev::Psu => PSU_TIMEOUT_MS, Dev::Dmm => 12_000 }
    }
}

/// Argument rule of an allowed power-supply command.
#[derive(Clone, Copy)]
enum PsuArg {
    Bare,
    Num,
    Kw(&'static [&'static str]),
    NumOrKw(&'static [&'static str]),
    /// Non-volatile state slot 0-3 (*SAV / *RCL).
    Slot,
    /// No argument, a number, or one of the keywords.
    OptNumOrKw(&'static [&'static str]),
}

/// Commands the /psu page may send (programming guide 5962-8198, chapter 4).
/// "LOCAL" is not SCPI: it sends Go To Local (GTL) to unlock the front panel.
const PSU_CMDS: &[(&str, PsuArg)] = &[
    ("VOLT", PsuArg::Num),
    ("CURR", PsuArg::Num),
    ("VOLT:PROT", PsuArg::Num),
    ("OUTP", PsuArg::Kw(ONOFF)),
    ("CURR:PROT:STAT", PsuArg::Kw(ONOFF)),
    ("OUTP:PROT:CLE", PsuArg::Bare),
    ("OUTP:PROT:DEL", PsuArg::Num),
    ("SENS:CURR:RANG", PsuArg::NumOrKw(&["MIN", "MAX"])),
    ("SENS:SWE:POIN", PsuArg::Num),
    ("SENS:SWE:TINT", PsuArg::Num),
    ("DISP", PsuArg::Kw(ONOFF)),
    ("*RST", PsuArg::Bare),
    ("*CLS", PsuArg::Bare),
    ("*SAV", PsuArg::Slot),
    ("*RCL", PsuArg::Slot),
    ("LOCAL", PsuArg::Bare),
];

/// Read-only queries the /psu page may send.
const PSU_QUERIES: &[&str] = &[
    "*IDN?", "MEAS:VOLT?", "MEAS:CURR?", "VOLT?", "CURR?", "VOLT:PROT?", "OUTP?",
    "CURR:PROT:STAT?", "OUTP:PROT:DEL?", "SENS:CURR:RANG?", "SENS:SWE:POIN?",
    "SENS:SWE:TINT?", "STAT:OPER:COND?", "STAT:QUES:COND?",
    "STAT:QUES:EVEN?", "SYST:ERR?", "DISP?", "*ESR?",
    "VOLT? MAX", "CURR? MAX", "VOLT:PROT? MAX",
];

/// Commands the /dmm page may send (HP 34401A User's Guide, chapter 4).
const CONF_ARGS: &[&str] = &["DEF", "MIN", "MAX"];
const NPLCS: &[&str] = &["0.02", "0.2", "1", "10", "100", "MIN", "MAX"];
const MINMAX: &[&str] = &["MIN", "MAX"];
const DMM_CMDS: &[(&str, PsuArg)] = &[
    ("CONF:VOLT:DC", PsuArg::OptNumOrKw(CONF_ARGS)),
    ("CONF:VOLT:AC", PsuArg::OptNumOrKw(CONF_ARGS)),
    ("CONF:CURR:DC", PsuArg::OptNumOrKw(CONF_ARGS)),
    ("CONF:CURR:AC", PsuArg::OptNumOrKw(CONF_ARGS)),
    ("CONF:RES", PsuArg::OptNumOrKw(CONF_ARGS)),
    ("CONF:FRES", PsuArg::OptNumOrKw(CONF_ARGS)),
    ("CONF:FREQ", PsuArg::OptNumOrKw(CONF_ARGS)),
    ("CONF:PER", PsuArg::OptNumOrKw(CONF_ARGS)),
    ("CONF:CONT", PsuArg::Bare),
    ("CONF:DIOD", PsuArg::Bare),
    ("VOLT:DC:RANG", PsuArg::NumOrKw(MINMAX)),
    ("VOLT:AC:RANG", PsuArg::NumOrKw(MINMAX)),
    ("CURR:DC:RANG", PsuArg::NumOrKw(MINMAX)),
    ("CURR:AC:RANG", PsuArg::NumOrKw(MINMAX)),
    ("RES:RANG", PsuArg::NumOrKw(MINMAX)),
    ("FRES:RANG", PsuArg::NumOrKw(MINMAX)),
    ("FREQ:VOLT:RANG", PsuArg::NumOrKw(MINMAX)),
    ("PER:VOLT:RANG", PsuArg::NumOrKw(MINMAX)),
    ("VOLT:DC:RANG:AUTO", PsuArg::Kw(ONOFF)),
    ("VOLT:AC:RANG:AUTO", PsuArg::Kw(ONOFF)),
    ("CURR:DC:RANG:AUTO", PsuArg::Kw(ONOFF)),
    ("CURR:AC:RANG:AUTO", PsuArg::Kw(ONOFF)),
    ("RES:RANG:AUTO", PsuArg::Kw(ONOFF)),
    ("FRES:RANG:AUTO", PsuArg::Kw(ONOFF)),
    ("FREQ:VOLT:RANG:AUTO", PsuArg::Kw(ONOFF)),
    ("PER:VOLT:RANG:AUTO", PsuArg::Kw(ONOFF)),
    ("VOLT:DC:NPLC", PsuArg::Kw(NPLCS)),
    ("CURR:DC:NPLC", PsuArg::Kw(NPLCS)),
    ("RES:NPLC", PsuArg::Kw(NPLCS)),
    ("FRES:NPLC", PsuArg::Kw(NPLCS)),
    ("DET:BAND", PsuArg::Kw(&["3", "20", "200"])),
    ("FREQ:APER", PsuArg::Kw(&["0.01", "0.1", "1"])),
    ("PER:APER", PsuArg::Kw(&["0.01", "0.1", "1"])),
    ("ZERO:AUTO", PsuArg::Kw(&["ON", "OFF", "ONCE"])),
    ("INP:IMP:AUTO", PsuArg::Kw(ONOFF)),
    ("CALC:FUNC", PsuArg::Kw(&["NULL", "DB", "DBM", "AVER", "LIM"])),
    ("CALC:STAT", PsuArg::Kw(ONOFF)),
    ("CALC:NULL:OFFS", PsuArg::Num),
    ("TRIG:SOUR", PsuArg::Kw(&["IMM"])),
    ("SAMP:COUN", PsuArg::Kw(&["1"])),
    ("TRIG:COUN", PsuArg::Kw(&["1"])),
    ("SYST:BEEP:STAT", PsuArg::Kw(ONOFF)),
    ("DISP", PsuArg::Kw(ONOFF)),
    ("*RST", PsuArg::Bare),
    ("*CLS", PsuArg::Bare),
    ("LOCAL", PsuArg::Bare),
];

const DMM_QUERIES: &[&str] = &[
    "*IDN?", "CONF?", "FUNC?", "READ?", "SYST:ERR?", "ROUT:TERM?", "ZERO:AUTO?",
    "INP:IMP:AUTO?", "CALC:FUNC?", "CALC:STAT?", "CALC:NULL:OFFS?", "SYST:BEEP:STAT?",
    "DISP?", "DET:BAND?", "FREQ:APER?", "PER:APER?", "VOLT:DC:NPLC?", "CURR:DC:NPLC?",
    "RES:NPLC?", "FRES:NPLC?", "VOLT:DC:RANG:AUTO?", "VOLT:AC:RANG:AUTO?",
    "CURR:DC:RANG:AUTO?", "CURR:AC:RANG:AUTO?", "RES:RANG:AUTO?", "FRES:RANG:AUTO?",
    "FREQ:VOLT:RANG:AUTO?", "PER:VOLT:RANG:AUTO?", "TRIG:SOUR?", "SAMP:COUN?",
    "TRIG:COUN?", "*ESR?",
];

/// Bus timeout for power-supply requests (MEAS? takes ~30 ms with default sampling).
const PSU_TIMEOUT_MS: u32 = 2000;

fn psu_num_ok(a: &str) -> bool {
    !a.is_empty()
        && a.len() <= 16
        && a.chars().all(|c| c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E'))
        && a.parse::<f64>().map_or(false, |x| x.is_finite() && x.abs() < 1e7)
}

/// Validate one request item: an allowed query, or "<HEADER> [<arg>]" per the
/// device's command table.
pub fn validate_inst_item(dev: Dev, item: &str) -> Result<(), &'static str> {
    if dev.queries().contains(&item) {
        return Ok(());
    }
    let mut it = item.split_whitespace();
    let head = it.next().ok_or("empty command")?;
    let arg = it.next();
    if it.next().is_some() {
        return Err("too many arguments");
    }
    let rule = dev.cmds().iter().find(|(h, _)| *h == head).map(|(_, r)| *r)
        .ok_or("command not allowed from this page")?;
    let ok = match (rule, arg) {
        (PsuArg::Bare, None) => true,
        (PsuArg::Num, Some(a)) => psu_num_ok(a),
        (PsuArg::Kw(kw), Some(a)) => kw.contains(&a),
        (PsuArg::NumOrKw(kw), Some(a)) => kw.contains(&a) || psu_num_ok(a),
        (PsuArg::Slot, Some(a)) => matches!(a, "0" | "1" | "2" | "3"),
        (PsuArg::OptNumOrKw(_), None) => true,
        (PsuArg::OptNumOrKw(kw), Some(a)) => kw.contains(&a) || psu_num_ok(a),
        _ => false,
    };
    if ok { Ok(()) } else { Err("invalid argument for this command") }
}

/// Execute request items in order and publish {"type":"psu"|"dmm","id":…,"vals":{query:reply},"err":…}.
/// Stops at the first bus error (instrument absent: the rest would fail the same way).
fn inst_request(bus: &mut Bus, dev: Dev, addr: Addr, items: &[String], id: &str, sinks: ScopeSinks) {
    let mut vals = String::new();
    let mut err = String::new();
    for it in items {
        let r = if it == "LOCAL" {
            bus.go_to_local(addr).map(|_| None)
        } else if it.contains('?') {
            bus_query(bus, addr, it).map(Some)
        } else {
            bus_cmd(bus, addr, it).map(|_| None)
        };
        match r {
            Ok(Some(v)) => {
                if !vals.is_empty() {
                    vals.push(',');
                }
                vals.push_str(&format!(r#""{}":"{}""#, json_escape(it),
                    json_escape(&v).replace('\n', " ").replace('\r', "")));
            }
            Ok(None) => info!("[GPIB] {} {}: {}", dev.name(), addr.pad, it),
            Err(e) => {
                err = format!("{}: {}", it, e);
                break;
            }
        }
    }
    (sinks.inst)(&format!(r#"{{"type":"{}","id":"{}","addr":{},"vals":{{{}}},"err":"{}"}}"#,
                          dev.name(), json_escape(id), addr.pad, vals, json_escape(&err)));
}

/// Queue an instrument-page request (validated). Does not interrupt a running spectrum
/// frame: the request runs between frames.
pub fn inst_request_queue(dev: Dev, addr: u8, items: &[&str], id: &str) -> Result<(), &'static str> {
    if addr > 30 {
        return Err("GPIB address must be 0-30");
    }
    if items.is_empty() || items.len() > 24 {
        return Err("1-24 items per request");
    }
    for it in items {
        validate_inst_item(dev, it)?;
    }
    let id: String = id.chars().filter(|c| c.is_ascii_alphanumeric() || *c == '_').take(16).collect();
    if let Some(tx) = TX.lock().unwrap().as_ref() {
        let _ = tx.send(Msg::Inst(dev, addr, items.iter().map(|s| s.to_string()).collect(), id));
    }
    Ok(())
}

/// Sinks for the spectrum view (installed by the bridge module).
#[derive(Clone, Copy)]
pub struct ScopeSinks {
    /// JSON text messages (meta / error / state).
    pub text: fn(&str),
    /// One trace frame: raw big-endian u16 values.
    pub frame: fn(&[u8]),
    /// JSON replies for the instrument pages (/psu, /dmm).
    pub inst: fn(&str),
}

/// Bus timeout while the spectrum view runs (long sweeps must not time out).
const SCOPE_TIMEOUT_MS: u32 = 10_000;
/// Re-read the instrument settings every N frames (front-panel changes).
const SCOPE_META_EVERY: u32 = 15; // lite meta

struct Scope {
    addr: Addr,
    npts: usize,
    frames: u32,
    errors: u32,
    /// SINGLE mode: capture one sweep per SINGLE key press, then wait.
    single: bool,
    /// A single sweep has been requested and not yet captured.
    pending: bool,
    /// Trace B display mode from TB? (low nibble: 0 Write 1 View 2 Blank 3 Normalize).
    trace_b: i32,
    /// A full meta (soft-key states) is due after panel commands.
    meta_full_due: bool,
    /// Sweep time [s] from SW? (frame timeout is derived from it).
    swt: f64,
    /// Trigger source from TRGSRC? (0 = Free Run). With any other source the
    /// sweep may wait forever for a trigger, and while a TS (take sweep) is
    /// pending the R3267 accepts no GPIB data at all (not even IFC/SDC help).
    trg: i32,
    /// Two-screen display (WINDOW Zoom / F/T / T/T, MLTSCR 1-3): both screens
    /// are captured by switching SCRSEL; `scrsel` is the screen the user has
    /// selected (0 upper, 1 lower) and is restored after each capture.
    split: bool,
    scrsel: i32,
}

impl Scope {
    fn new(addr: Addr) -> Self {
        Scope { addr, npts: 1001, frames: 0, errors: 0, single: false, pending: false, trace_b: 2, meta_full_due: false, swt: 0.0, trg: 0, split: false, scrsel: 0 }
    }
    /// True when nothing has to be captured right now (SINGLE done).
    fn idle(&self) -> bool {
        self.single && !self.pending
    }
}

fn bus_cmd(bus: &mut Bus, addr: Addr, cmd: &str) -> GResult<()> {
    let mut v = cmd.as_bytes().to_vec();
    v.push(b'\n');
    bus.write(addr, &v, true)
}

/// Query terminated by EOI (works with both DL0 and DL2 delimiters).
fn bus_query(bus: &mut Bus, addr: Addr, cmd: &str) -> GResult<String> {
    bus_cmd(bus, addr, cmd)?;
    let mut v = Vec::new();
    bus.read(addr, ReadEnd::Eoi, &mut |c: &[u8]| v.extend_from_slice(c))?;
    Ok(String::from_utf8_lossy(&v).trim().to_string())
}

fn fmt_num(x: f64) -> String {
    // Advantest accepts plain decimal numbers; trim trailing zeros.
    let s = format!("{:.6}", x);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() { "0".into() } else { s.to_string() }
}

fn json_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// Read the current conditions and publish them as a "meta" message.
/// Read the instrument settings and publish a "meta" message.
/// `full = false` (periodic refresh) reads only the display-relevant values
/// (~11 queries); `full = true` (after panel commands / at start) also reads
/// the soft-key states (~30 queries, ~0.5 s). The browser merges metas.
/// Send a command through the query closure's bus borrow (a reply-less "query").
fn bus_cmd_q(q: &mut dyn FnMut(&str) -> GResult<f64>, cmd: &str) -> GResult<()> {
    // Prefix "!" = command without reading a reply (see the closure below).
    q(&format!("!{}", cmd)).map(|_| ())
}

fn scope_meta(bus: &mut Bus, sc: &mut Scope, sinks: ScopeSinks, full: bool) -> GResult<()> {
    let addr = sc.addr;
    let mut q = |c: &str| -> GResult<f64> {
        if let Some(cmd) = c.strip_prefix('!') {
            bus_cmd(bus, addr, cmd)?;
            return Ok(f64::NAN);
        }
        let r = bus_query(bus, addr, c)?;
        Ok(r.trim().parse::<f64>().unwrap_or(f64::NAN))
    };
    // JSON has no NaN: emit null for unknown values.
    let j = |x: f64| if x.is_finite() { format!("{}", x) } else { "null".to_string() };
    let mut m = format!(r#"{{"type":"meta","full":{},"addr":{}"#, full, addr.pad);
    let mut add = |k: &str, v: String| { m.push_str(&format!(r#","{}":{}"#, k, v)); };

    // Split screen: the settings queries refer to the selected screen, so read
    // the upper screen with SCRSEL TRA (the lower one goes to "meta2").
    if full {
        let mlt = q("MLTSCR?")? as i32;
        sc.split = (1..=3).contains(&mlt);
        sc.scrsel = q("SCRSEL?")? as i32;
        add("scrsel", sc.scrsel.to_string());
    }
    add("split", sc.split.to_string());
    if sc.split {
        bus_cmd_q(&mut q, "SCRSEL TRA")?;
    }

    let cf = q("CF?")?; let sp = q("SP?")?; let rb = q("RB?")?;
    let fa = q("FA?")?; let fb = q("FB?")?; let rl = q("RL?")?;
    let dd = q("DD?")?; let tp = q("TP?")?; let mn = q("MN?")?;
    let (mf, ml) = if mn as i32 != 0 { (q("MF?")?, q("ML?")?) } else { (f64::NAN, f64::NAN) };
    sc.npts = if tp as i32 == 1 { 1001 } else { 501 };
    let ddiv = match dd as i32 { 1 => 5.0, 2 => 2.0, 3 => 1.0, 4 => 0.5, _ => 10.0 };
    add("cf", j(cf)); add("span", j(sp)); add("rbw", j(rb)); add("fa", j(fa)); add("fb", j(fb));
    add("rl", j(rl)); add("ddiv", ddiv.to_string()); add("npts", sc.npts.to_string());
    add("mkr", (mn as i32).to_string()); add("mkf", j(mf)); add("mkl", j(ml));
    add("single", sc.single.to_string());
    // Units: W / V (5 / 6) mean a linear display (trace scaled 0 … reference level).
    let au = q("AUNITS?")?;
    add("aunits", (au as i32).to_string());

    if full {
        for (k, c) in [("vbw", "VB?"), ("swt", "SW?"), ("att", "AT?"), ("cfstep", "CS?"),
                       ("swpcnt", "SWPCNT?"), ("dl", "DL?")] {
            let v = q(c)?;
            if k == "swt" && v.is_finite() {
                sc.swt = v;
            }
            add(k, j(v));
        }
        for (k, c) in [("rbw_auto", "BA?"), ("vbw_auto", "VA?"), ("swt_auto", "AS?"), ("att_auto", "AA?"),
                       ("cfstep_auto", "CA?"), ("swm", "SWM?"), ("ta", "TA?"), ("tb", "TB?"),
                       ("det", "DET?"), ("detb", "DETB?"), ("trg", "TRGSRC?"), ("dlon", "DLON?"),
                       ("wdo", "WDO?"), ("mltscr", "MLTSCR?")] {
            let v = q(c)? as i32;
            if k == "tb" {
                // TA?/TB?: low nibble = mode (0 Write 1 View 2 Blank 3 Normalize),
                // high nibble = add-on (1 MaxHold 2 Average 3 MinHold 4 PowerAvg),
                // e.g. 0x20 = 32 = Write + Average.
                sc.trace_b = v & 0x0F;
            }
            if k == "trg" {
                sc.trg = v;
            }
            add(k, v.to_string());
        }
    }
    m.push('}');
    if sc.split {
        // lower screen axis / scale → "meta2", then back to the user's screen
        bus_cmd_q(&mut q, "SCRSEL TRB")?;
        let mut m2 = String::from(r#"{"type":"meta2""#);
        for (k, c) in [("cf", "CF?"), ("span", "SP?"), ("fa", "FA?"), ("fb", "FB?"), ("swt", "SW?"),
                       ("rl", "RL?"), ("rbw", "RB?")] {
            let v = q(c)?;
            m2.push_str(&format!(r#","{}":{}"#, k, j(v)));
        }
        let dd = q("DD?")?;
        let ddiv = match dd as i32 { 1 => 5.0, 2 => 2.0, 3 => 1.0, 4 => 0.5, _ => 10.0 };
        let au = q("AUNITS?")?;
        let ta = q("TA?")?;
        m2.push_str(&format!(r#","ddiv":{},"aunits":{},"ta":{},"npts":{}}}"#, ddiv, au as i32, ta as i32, sc.npts));
        bus_cmd_q(&mut q, if sc.scrsel == 1 { "SCRSEL TRB" } else { "SCRSEL TRA" })?;
        (sinks.text)(&m);
        (sinks.text)(&m2);
    } else {
        (sinks.text)(&m);
    }
    Ok(())
}

/// Run panel queries and publish {"type":"qres","id":…,"vals":{"Q?":"…",…}}.
/// Multi-set results (MULTI_QUERIES) are returned as "n\nline1\nline2…".
fn panel_queries(bus: &mut Bus, addr: Addr, qs: &[String], id: &str, sinks: ScopeSinks) {
    let mut out = format!(r#"{{"type":"qres","id":"{}","vals":{{"#, json_escape(id));
    for (i, q) in qs.iter().enumerate() {
        let val = match bus_query(bus, addr, q) {
            Ok(first) => {
                let mut v = first.clone();
                if MULTI_QUERIES.contains(&q.as_str()) {
                    let n = first.trim().parse::<usize>().unwrap_or(0).min(50);
                    for _ in 0..n {
                        let mut line = Vec::new();
                        if bus.read(addr, ReadEnd::Eoi, &mut |c: &[u8]| line.extend_from_slice(c)).is_err() {
                            break;
                        }
                        v.push('\n');
                        v.push_str(String::from_utf8_lossy(&line).trim());
                    }
                }
                v
            }
            Err(GpibError::Aborted) => return,
            Err(e) => format!("ERR: {}", e),
        };
        if i > 0 {
            out.push(',');
        }
        out.push_str(&format!(r#""{}":"{}""#, json_escape(q), json_escape(&val).replace('\n', "\\n").replace('\r', "")));
    }
    out.push_str("}}");
    (sinks.text)(&out);
}

fn scope_begin(bus: &mut Bus, p: ScopeParams, sinks: ScopeSinks) -> Result<Scope, String> {
    p.validate()?;
    let addr = Addr { pad: p.addr, sad: None };
    bus.set_timeout_ms(SCOPE_TIMEOUT_MS);
    let mut sc = Scope::new(addr);
    let mut dl2_sent = false;
    let r = (|| -> GResult<()> {
        if let Some(v) = p.cf_mhz {
            bus_cmd(bus, addr, &format!("CF {}MZ", fmt_num(v)))?;
        }
        if let Some(v) = p.span_mhz {
            bus_cmd(bus, addr, &format!("SP {}MZ", fmt_num(v)))?;
        }
        if let Some(v) = p.rbw_khz {
            bus_cmd(bus, addr, &format!("RB {}KZ", fmt_num(v)))?;
        }
        bus_cmd(bus, addr, "DL2")?;
        dl2_sent = true;
        scope_meta(bus, &mut sc, sinks, true)
    })();
    if let Err(e) = r {
        if dl2_sent {
            // Do not leave the instrument with the EOI-only delimiter.
            let _ = bus_cmd(bus, addr, "DL0");
        }
        return Err(format!("R3267 setup failed (addr {}): {}", p.addr, e));
    }
    info!("[GPIB] spectrum view started: {:?}", p);
    Ok(sc)
}

fn scope_end(bus: &mut Bus, sc: &Scope, read_tmo_ms: u32) {
    // Restore the default delimiter (CR LF EOI) for normal ASCII use.
    // Short timeout: if the instrument is stuck, do not block Stop for long.
    bus.set_timeout_ms(2000);
    if let Err(e) = bus_cmd(bus, sc.addr, "DL0") {
        warn!("[GPIB] DL0 restore failed: {}", e);
    }
    bus.set_timeout_ms(read_tmo_ms);
    info!("[GPIB] spectrum view stopped after {} frames", sc.frames);
}

fn scope_step(bus: &mut Bus, sc: &mut Scope, sinks: ScopeSinks) -> GResult<()> {
    if sc.meta_full_due {
        sc.meta_full_due = false;
        scope_meta(bus, sc, sinks, true)?;
    } else if sc.frames > 0 && sc.frames % SCOPE_META_EVERY == 0 {
        scope_meta(bus, sc, sinks, false)?;
    }
    // TS waits for the sweep (and its trigger): allow at least 1.5 x sweep time.
    let tmo = ((sc.swt * 1500.0) as u32 + 2000).max(SCOPE_TIMEOUT_MS);
    bus.set_timeout_ms(tmo);
    FRAME_START_MS.store((now_us() / 1000) as u32, Ordering::SeqCst);
    IN_FRAME.store(true, Ordering::SeqCst);
    let r = scope_frame(bus, sc, sinks);
    IN_FRAME.store(false, Ordering::SeqCst);
    bus.set_timeout_ms(SCOPE_TIMEOUT_MS);
    r
}

fn scope_frame(bus: &mut Bus, sc: &mut Scope, sinks: ScopeSinks) -> GResult<()> {
    // Free Run: TS = take one complete sweep, then read it. With a trigger
    // source (Video/Line/Ext) TS could block the instrument indefinitely, so
    // just read the current trace instead.
    if sc.split {
        return scope_frame_split(bus, sc, sinks);
    }
    if sc.trg == 0 {
        bus_cmd(bus, sc.addr, "TS")?;
    } else {
        // Pace the reads (no TS to wait for).
        unsafe { sys::vTaskDelay(10); }
    }
    // Frame = 1 tag byte ('A' / 'B') + N x u16 big-endian.
    let mut ok = true;
    for (tag, query) in [(b'A', "TBA?"), (b'B', "TBB?")] {
        if tag == b'B' && sc.trace_b == 2 {
            continue; // trace B blank: do not spend bus time on it
        }
        bus_cmd(bus, sc.addr, query)?;
        let mut v = Vec::with_capacity(sc.npts * 2 + 1);
        v.push(tag);
        bus.read(sc.addr, ReadEnd::Eoi, &mut |c: &[u8]| v.extend_from_slice(c))?;
        if v.len() == sc.npts * 2 + 1 {
            (sinks.frame)(&v);
        } else {
            warn!("[GPIB] trace {} length {} (expected {})", tag as char, v.len() - 1, sc.npts * 2);
            ok = false;
        }
    }
    if !ok {
        // Point count changed on the front panel: refresh meta next frame.
        sc.meta_full_due = true;
    }
    sc.frames = sc.frames.wrapping_add(1);
    sc.pending = false;
    Ok(())
}

/// Two-screen frame: upper screen → tag 'A', lower screen → tag 'C'
/// (TBA? returns the selected screen's trace; TBB? does not answer in this mode).
fn scope_frame_split(bus: &mut Bus, sc: &mut Scope, sinks: ScopeSinks) -> GResult<()> {
    let mut ok = true;
    let r = (|| -> GResult<()> {
        for (tag, scr) in [(b'A', "SCRSEL TRA"), (b'C', "SCRSEL TRB")] {
            bus_cmd(bus, sc.addr, scr)?;
            if sc.trg == 0 {
                bus_cmd(bus, sc.addr, "TS")?;
            }
            bus_cmd(bus, sc.addr, "TBA?")?;
            let mut v = Vec::with_capacity(sc.npts * 2 + 1);
            v.push(tag);
            bus.read(sc.addr, ReadEnd::Eoi, &mut |c: &[u8]| v.extend_from_slice(c))?;
            if v.len() == sc.npts * 2 + 1 {
                (sinks.frame)(&v);
            } else {
                warn!("[GPIB] split trace {} length {} (expected {})", tag as char, v.len() - 1, sc.npts * 2);
                ok = false;
            }
        }
        Ok(())
    })();
    // always return to the screen the user selected
    let _ = bus_cmd(bus, sc.addr, if sc.scrsel == 1 { "SCRSEL TRB" } else { "SCRSEL TRA" });
    r?;
    if !ok {
        sc.meta_full_due = true;
    }
    sc.frames = sc.frames.wrapping_add(1);
    sc.pending = false;
    Ok(())
}

// ─── Worker thread / public API ──────────────────────────────────────────────

enum Msg {
    Data(Vec<u8>),
    Interactive(bool),
    ScopeStart(ScopeParams),
    ScopeStop,
    /// Front-panel command from the browser (already validated).
    ScopeCmd(u8, String),
    /// Panel queries (address, queries, request id), already validated.
    ScopeQuery(u8, Vec<String>, String),
    /// Instrument-page request (device, address, commands/queries, request id), already validated.
    Inst(Dev, u8, Vec<String>, String),
}

static TX: Mutex<Option<Sender<Msg>>> = Mutex::new(None);
static STARTED: AtomicBool = AtomicBool::new(false);
static CURRENT_ADDR: AtomicU8 = AtomicU8::new(1);
static SRQ_STATE: AtomicBool = AtomicBool::new(false);
static SCOPE_RUNNING: AtomicBool = AtomicBool::new(false);
/// True while the worker thread is executing a bus operation.
static BUSY: AtomicBool = AtomicBool::new(false);
/// Abort request for the running bus operation (checked in `Bus::wait_for`).
static ABORT: AtomicBool = AtomicBool::new(false);
/// True while a spectrum frame (TS + TBA?) is being captured.
static IN_FRAME: AtomicBool = AtomicBool::new(false);
/// Start time of the current frame capture [ms, wrapping].
static FRAME_START_MS: AtomicU32 = AtomicU32::new(0);
/// A frame running longer than this is considered stuck (trigger wait etc.).
const FRAME_STUCK_MS: u32 = 1000;
/// Interactive console active (Ctrl-C = abort).
static INTERACTIVE: AtomicBool = AtomicBool::new(true);

/// Initialise the GPIB bus and start the interpreter thread.
/// `out` receives every byte the interpreter produces (forwarded to SSH / WebSocket);
/// `sinks` receive the spectrum-view messages and frames.
/// `default_addr` is the primary address selected at power-up (cfg.toml / NVS `gpib_addr`).
pub fn start(out: OutFn, sinks: ScopeSinks, default_addr: u8) {
    if STARTED.swap(true, Ordering::SeqCst) {
        return;
    }
    let (tx, rx): (Sender<Msg>, Receiver<Msg>) = mpsc::channel();
    *TX.lock().unwrap() = Some(tx);
    thread::Builder::new()
        .name("gpib".into())
        .stack_size(12288)
        .spawn(move || {
            let bus = Bus::new();
            let mut interp = Interp::new(bus, default_addr);
            let mut scope: Option<Scope> = None;
            let state = |on: bool| {
                SCOPE_RUNNING.store(on, Ordering::Relaxed);
                (sinks.text)(&format!(r#"{{"type":"state","running":{}}}"#, on));
            };
            loop {
                // While the spectrum view runs, only poll for messages between
                // frames; otherwise block (with a timeout for SRQ sampling).
                let msg = if scope.as_ref().map_or(false, |sc| !sc.idle()) {
                    match rx.try_recv() {
                        Ok(m) => Some(m),
                        Err(mpsc::TryRecvError::Empty) => None,
                        Err(mpsc::TryRecvError::Disconnected) => break,
                    }
                } else {
                    match rx.recv_timeout(std::time::Duration::from_millis(200)) {
                        Ok(m) => Some(m),
                        Err(mpsc::RecvTimeoutError::Timeout) => None,
                        Err(mpsc::RecvTimeoutError::Disconnected) => break,
                    }
                };
                BUSY.store(true, Ordering::SeqCst);
                // Handle every queued message before the next frame, so bursts
                // (jog dial, fast clicks) cost one meta read instead of one each.
                let mut msg = msg;
                let mut meta_addr: Option<u8> = None; // not capturing: full meta due
                while let Some(m) = msg.take() {
                match m {
                    Msg::Data(d) => interp.feed(out, &d),
                    Msg::Interactive(on) => interp.set_interactive(on),
                    Msg::ScopeStart(p) => {
                        if let Some(sc) = scope.take() {
                            scope_end(&mut interp.bus, &sc, interp.read_tmo_ms);
                        }
                        match scope_begin(&mut interp.bus, p, sinks) {
                            Ok(sc) => {
                                scope = Some(sc);
                                state(true);
                            }
                            Err(e) => {
                                warn!("[GPIB] {}", e);
                                interp.bus.set_timeout_ms(interp.read_tmo_ms);
                                (sinks.text)(&format!(r#"{{"type":"error","msg":"{}"}}"#, json_escape(&e)));
                                state(false);
                            }
                        }
                    }
                    Msg::ScopeStop => {
                        if let Some(sc) = scope.take() {
                            scope_end(&mut interp.bus, &sc, interp.read_tmo_ms);
                        }
                        state(false);
                    }
                    Msg::ScopeQuery(addr, qs, id) => {
                        let a = scope.as_ref().map(|sc| sc.addr).unwrap_or(Addr { pad: addr, sad: None });
                        interp.bus.set_timeout_ms(PANEL_QUERY_TIMEOUT_MS);
                        panel_queries(&mut interp.bus, a, &qs, &id, sinks);
                        interp.bus.set_timeout_ms(if scope.is_some() { SCOPE_TIMEOUT_MS } else { interp.read_tmo_ms });
                    }
                    Msg::Inst(dev, pad, items, id) => {
                        interp.bus.set_timeout_ms(dev.timeout_ms());
                        inst_request(&mut interp.bus, dev, Addr { pad, sad: None }, &items, &id, sinks);
                        interp.bus.set_timeout_ms(if scope.is_some() { SCOPE_TIMEOUT_MS } else { interp.read_tmo_ms });
                    }
                    Msg::ScopeCmd(addr, text) => {
                        let r = match scope.as_mut() {
                            Some(sc) => {
                                match text.as_str() {
                                    // SINGLE: one sweep per press, then hold the display.
                                    "SNGLS" => { sc.single = true; sc.pending = true; }
                                    // REPEAT: continuous capture.
                                    "CONTS" => { sc.single = false; sc.pending = false; }
                                    t if t.starts_with("TRGSRC") => {
                                        sc.trg = if t.ends_with("FREE") { 0 } else { 1 };
                                    }
                                    "SCRSEL TRA" => sc.scrsel = 0,
                                    "SCRSEL TRB" => sc.scrsel = 1,
                                    t if t.starts_with("MLTSCR") => {
                                        sc.split = !t.ends_with("OFF");
                                        if !sc.split { sc.scrsel = 0; }
                                    }
                                    _ => {}
                                }
                                // Panel values are re-read once after this batch.
                                sc.meta_full_due = true;
                                bus_cmd(&mut interp.bus, sc.addr, &text)
                            }
                            None => {
                                // Not capturing: execute now, report the state after the batch.
                                interp.bus.set_timeout_ms(SCOPE_TIMEOUT_MS);
                                let r = bus_cmd(&mut interp.bus, Addr { pad: addr, sad: None }, &text);
                                interp.bus.set_timeout_ms(interp.read_tmo_ms);
                                meta_addr = Some(addr);
                                r
                            }
                        };
                        // PRESET / RECALL may reset the delimiter: restore DL2 while capturing.
                        if r.is_ok() && (text == "IP" || text.starts_with("RC ")) {
                            if let Some(sc) = scope.as_mut() {
                                let _ = bus_cmd(&mut interp.bus, sc.addr, "DL2");
                                sc.meta_full_due = true;
                            }
                        }
                        match r {
                            Ok(()) => info!("[GPIB] panel: {}", text),
                            Err(e) => (sinks.text)(&format!(r#"{{"type":"error","msg":"{}"}}"#,
                                                            json_escape(&format!("{}: {}", text, e)))),
                        }
                    }
                }
                // An abort only cancels the operation that was running when it
                // was requested; the next queued command must run normally.
                if ABORT.swap(false, Ordering::SeqCst) {
                    interp.bus.recover();
                }
                msg = rx.try_recv().ok();
                }
                // One meta for the whole batch when no frame will follow.
                if let Some(sc) = scope.as_mut() {
                    if sc.idle() && sc.meta_full_due {
                        sc.meta_full_due = false;
                        if let Err(e) = scope_meta(&mut interp.bus, sc, sinks, true) {
                            warn!("[GPIB] meta failed: {}", e);
                        }
                    }
                } else if let Some(a) = meta_addr {
                    interp.bus.set_timeout_ms(SCOPE_TIMEOUT_MS);
                    let mut tmp = Scope::new(Addr { pad: a, sad: None });
                    if let Err(e) = scope_meta(&mut interp.bus, &mut tmp, sinks, true) {
                        (sinks.text)(&format!(r#"{{"type":"error","msg":"{}"}}"#, json_escape(&format!("meta: {}", e))));
                    }
                    interp.bus.set_timeout_ms(interp.read_tmo_ms);
                }
                if let Some(sc) = scope.as_mut().filter(|sc| !sc.idle()) {
                    match scope_step(&mut interp.bus, sc, sinks) {
                        Ok(()) => sc.errors = 0,
                        // Aborted (Stop / Ctrl-C / panel key): not an instrument error.
                        Err(GpibError::Aborted) => {
                            sc.meta_full_due = true; // re-read settings after IFC
                        }
                        Err(e) => {
                            sc.errors += 1;
                            warn!("[GPIB] spectrum frame failed: {}", e);
                            (sinks.text)(&format!(r#"{{"type":"error","msg":"{}"}}"#,
                                                  json_escape(&format!("frame: {}", e))));
                            if sc.errors >= 3 {
                                let sc = scope.take().unwrap();
                                scope_end(&mut interp.bus, &sc, interp.read_tmo_ms);
                                state(false);
                            }
                        }
                    }
                }
                BUSY.store(false, Ordering::SeqCst);
                if ABORT.swap(false, Ordering::SeqCst) {
                    interp.bus.recover();
                    if let Some(sc) = scope.as_mut() {
                        sc.frames = SCOPE_META_EVERY; // re-read settings after IFC
                    }
                }
                SRQ_STATE.store(interp.bus.srq(), Ordering::Relaxed);
            }
        })
        .expect("spawn gpib thread");
}

/// Abort the bus operation in progress (handshake wait), if any.
/// Safe to call from any thread; the worker resets the bus with IFC afterwards.
/// Does nothing when the bus is idle, so it never cancels a future command.
pub fn abort() {
    if BUSY.load(Ordering::SeqCst) {
        ABORT.store(true, Ordering::SeqCst);
        warn!("[GPIB] abort requested");
    }
}

/// Abort only a spectrum frame capture in progress (e.g. TS waiting for a
/// trigger that never comes), so a panel command can be executed at once.
pub fn abort_frame() {
    let elapsed = ((now_us() / 1000) as u32).wrapping_sub(FRAME_START_MS.load(Ordering::SeqCst));
    if IN_FRAME.load(Ordering::SeqCst) && elapsed > FRAME_STUCK_MS {
        ABORT.store(true, Ordering::SeqCst);
        warn!("[GPIB] frame capture interrupted by panel command");
    }
}

/// Ctrl-C (0x03) from an interactive console aborts the running operation.
/// Raw (Prologix-style) sessions are not affected.
pub fn check_ctrl_c(data: &[u8]) {
    if INTERACTIVE.load(Ordering::Relaxed) && data.contains(&0x03) {
        abort();
    }
}

/// Start (or restart with new conditions) the spectrum view.
pub fn scope_start(p: ScopeParams) {
    if let Some(tx) = TX.lock().unwrap().as_ref() {
        let _ = tx.send(Msg::ScopeStart(p));
    }
}

/// Send a front-panel command (see PANEL_CODES) to the instrument at `addr`
/// (the capture address is used while the spectrum view runs).
pub fn scope_cmd(addr: u8, text: &str) -> Result<(), &'static str> {
    if addr > 30 {
        return Err("GPIB address must be 0-30");
    }
    validate_panel_cmd(text)?;
    if let Some(tx) = TX.lock().unwrap().as_ref() {
        let _ = tx.send(Msg::ScopeCmd(addr, text.trim().to_string()));
    }
    abort_frame();
    Ok(())
}

/// Queue read-only panel queries (validated against PANEL_QUERIES).
pub fn scope_query(addr: u8, qs: &[&str], id: &str) -> Result<(), &'static str> {
    if addr > 30 {
        return Err("GPIB address must be 0-30");
    }
    if qs.is_empty() || qs.len() > 24 {
        return Err("1-24 queries per request");
    }
    for q in qs {
        validate_panel_query(q)?;
    }
    let id: String = id.chars().filter(|c| c.is_ascii_alphanumeric() || *c == '_').take(16).collect();
    if let Some(tx) = TX.lock().unwrap().as_ref() {
        let _ = tx.send(Msg::ScopeQuery(addr, qs.iter().map(|q| q.to_string()).collect(), id));
    }
    Ok(())
}

/// Stop the spectrum view (restores the R3267 delimiter to DL0).
pub fn scope_stop() {
    if let Some(tx) = TX.lock().unwrap().as_ref() {
        let _ = tx.send(Msg::ScopeStop);
    }
}

/// True while the spectrum view is capturing.
pub fn scope_running() -> bool {
    SCOPE_RUNNING.load(Ordering::Relaxed)
}

/// Queue input bytes (from an SSH session or the web terminal).
pub fn input(data: &[u8]) {
    if let Some(tx) = TX.lock().unwrap().as_ref() {
        let _ = tx.send(Msg::Data(data.to_vec()));
    }
}

/// Select interactive (echo + auto-read) or raw (Prologix default) behaviour.
pub fn set_interactive(on: bool) {
    if let Some(tx) = TX.lock().unwrap().as_ref() {
        let _ = tx.send(Msg::Interactive(on));
    }
}

/// Currently selected instrument primary address (for status display).
pub fn current_addr() -> u8 {
    CURRENT_ADDR.load(Ordering::Relaxed)
}

/// Last sampled SRQ line state (true = asserted).
pub fn srq_asserted() -> bool {
    SRQ_STATE.load(Ordering::Relaxed)
}
