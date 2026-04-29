// src/config.rs
use crate::devices::w25q32::{
    INTERFACE_DATA_ADDR, PID_1_DATA_ADDR, PID_2_DATA_ADDR, PID_BG_DATA_ADDR, PUMP_DATA_ADDR,
    W25Q32,
};
use crate::usb::usb_println;
use crate::utils::{Interface, PidData, PumpData};
use arrform::{arrform, ArrForm};
use core::mem;
use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};

// ── Stored structs ────────────────────────────────────────────────────────────
// #[repr(C)] guarantees a stable field layout for raw flash storage.

#[derive(Clone, Default)]
pub struct Config {
    pub pump_data: PumpData,
    pub pid_1_data: PidConfigData,
    pub pid_2_data: PidConfigData,
    pub pid_bg_data: PidConfigData,
    pub interface_temps: InterfaceConfigData,
}

#[repr(C)]
#[derive(Clone, Default)]
pub struct PidConfigData {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub window_size: u32,
    pub max_val: f32,
    pub osr: u32,
    pub target: f32,
    // size = 7 × 4 = 28 bytes; fits within BLOCK_SIZE (32 bytes)
}

#[repr(C)]
#[derive(Clone, Default)]
pub struct InterfaceConfigData {
    pub coffee_temperature: f32,
    pub brew_head_temperature: f32,
    pub steam_temperature: f32,
    // size = 3 × 4 = 12 bytes
}

// ── From conversions ──────────────────────────────────────────────────────────

impl From<&PidData> for PidConfigData {
    fn from(pid: &PidData) -> Self {
        PidConfigData {
            kp: pid.kp,
            ki: pid.ki,
            kd: pid.kd,
            window_size: pid.window_size,
            max_val: pid.max_val,
            osr: pid.osr,
            target: pid.target,
        }
    }
}

impl From<&Interface> for InterfaceConfigData {
    fn from(interface: &Interface) -> Self {
        InterfaceConfigData {
            coffee_temperature: interface.coffee_temperature,
            brew_head_temperature: interface.brew_head_temperature,
            steam_temperature: interface.steam_temperature,
        }
    }
}

// ── Flash block helpers ───────────────────────────────────────────────────────
// Each config section occupies one 32-byte block.  The last 4 bytes hold a
// simple additive checksum over the preceding struct bytes so we can detect
// erased/corrupt flash without adding external crates.

const BLOCK_SIZE: usize = 32;
const CHECKSUM_OFFSET: usize = 28; // bytes [28..32] = u32 checksum

fn compute_checksum(data: &[u8]) -> u32 {
    data.iter().fold(0u32, |acc, &b| acc.wrapping_add(b as u32))
}

// ── Validation ────────────────────────────────────────────────────────────────

fn is_valid_f32(v: f32, min: f32, max: f32) -> bool {
    v.is_finite() && v >= min && v <= max
}

fn validate_pid_config(cfg: &PidConfigData) -> bool {
    is_valid_f32(cfg.kp, 0.0, 10_000.0)
        && is_valid_f32(cfg.ki, 0.0, 10_000.0)
        && is_valid_f32(cfg.kd, 0.0, 10_000.0)
        && cfg.window_size >= 10
        && cfg.window_size <= 10_000
        && is_valid_f32(cfg.max_val, 0.0, 1.5)
        && cfg.osr >= 1
        && cfg.osr <= 100
        && is_valid_f32(cfg.target, 0.0, 160.0)
}

fn validate_pump_data(pump: &PumpData) -> bool {
    is_valid_f32(pump.heat_up_power, 0.0, 100.0)
        && is_valid_f32(pump.pre_infuse_power, 0.0, 100.0)
        && is_valid_f32(pump.steam_power, 0.0, 100.0)
        && is_valid_f32(pump.extract_power, 0.0, 100.0)
        && is_valid_f32(pump.extraction_timeout, 100.0, 120_000.0)
}

fn validate_interface_config(cfg: &InterfaceConfigData) -> bool {
    is_valid_f32(cfg.coffee_temperature, 0.0, 150.0)
        && is_valid_f32(cfg.brew_head_temperature, 0.0, 150.0)
        && is_valid_f32(cfg.steam_temperature, 0.0, 165.0)
}

// ── ConfigManager ─────────────────────────────────────────────────────────────

pub struct ConfigManager<SPI, CS> {
    flash: W25Q32<SPI, CS>,
}

impl<SPI, CS, E> ConfigManager<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Result<Self, E> {
        let mut flash = W25Q32::new(spi, cs);
        let id = flash.read_id()?;
        usb_println(
            arrform!(64, "W25Q32 ID: {:02X} {:02X} {:02X}", id[0], id[1], id[2]).as_str(),
        );
        Ok(ConfigManager { flash })
    }

    // ── Low-level block I/O ───────────────────────────────────────────────────

    /// Write `val` as a checked 32-byte block at `addr`.
    /// The last 4 bytes of the block store the checksum.
    fn write_block<T: Sized>(&mut self, addr: u32, val: &T) -> Result<(), E> {
        let data = unsafe {
            core::slice::from_raw_parts(val as *const T as *const u8, mem::size_of::<T>())
        };
        let mut block = [0u8; BLOCK_SIZE];
        block[..data.len()].copy_from_slice(data);
        let cs = compute_checksum(&block[..data.len()]);
        block[CHECKSUM_OFFSET..CHECKSUM_OFFSET + 4].copy_from_slice(&cs.to_le_bytes());
        self.flash.page_program(addr, &block)
    }

    /// Read a 32-byte block from `addr`, verify checksum, validate ranges.
    /// Returns `T::default()` on any failure so the firmware stays safe.
    fn read_block<T: Sized + Default + Clone>(
        &mut self,
        addr: u32,
        validate: fn(&T) -> bool,
    ) -> Result<T, E> {
        let mut block = [0u8; BLOCK_SIZE];
        self.flash.read_data(addr, &mut block)?;

        let data_len = mem::size_of::<T>();
        let stored_cs =
            u32::from_le_bytes([block[28], block[29], block[30], block[31]]);
        let computed_cs = compute_checksum(&block[..data_len]);

        if stored_cs != computed_cs {
            usb_println("[CFG] checksum mismatch – using defaults");
            return Ok(T::default());
        }

        // SAFETY: repr(C) guarantees layout.  read_unaligned handles the
        // fact that `block` is u8-aligned, not necessarily T-aligned.
        let val = unsafe { core::ptr::read_unaligned(block.as_ptr() as *const T) };

        if !validate(&val) {
            usb_println("[CFG] range check failed – using defaults");
            return Ok(T::default());
        }

        Ok(val)
    }

    // ── Public API ────────────────────────────────────────────────────────────

    pub fn load_config(&mut self) -> Result<Config, E> {
        Ok(Config {
            pump_data: self.read_block::<PumpData>(PUMP_DATA_ADDR, validate_pump_data)?,
            pid_1_data: self
                .read_block::<PidConfigData>(PID_1_DATA_ADDR, validate_pid_config)?,
            pid_2_data: self
                .read_block::<PidConfigData>(PID_2_DATA_ADDR, validate_pid_config)?,
            pid_bg_data: self
                .read_block::<PidConfigData>(PID_BG_DATA_ADDR, validate_pid_config)?,
            interface_temps: self
                .read_block::<InterfaceConfigData>(INTERFACE_DATA_ADDR, validate_interface_config)?,
        })
    }

    /// Erase sector 0 once, then write all five blocks.
    /// This is the only function that touches the flash erase.
    pub fn save_config(&mut self, config: &Config) -> Result<(), E> {
        // All five blocks live in sector 0 (addresses 0x000000–0x0004FF).
        // One sector_erase clears all of them.
        self.flash.sector_erase(PUMP_DATA_ADDR)?;

        self.write_block(PUMP_DATA_ADDR, &config.pump_data)?;
        self.write_block(PID_1_DATA_ADDR, &config.pid_1_data)?;
        self.write_block(PID_2_DATA_ADDR, &config.pid_2_data)?;
        self.write_block(PID_BG_DATA_ADDR, &config.pid_bg_data)?;
        self.write_block(INTERFACE_DATA_ADDR, &config.interface_temps)?;

        usb_println("[CFG] all config saved");
        Ok(())
    }

    /// Save all data in one erase cycle.  Used by `SaveAll` to avoid the
    /// three-erase penalty of calling the individual `update_from_*` helpers.
    pub fn save_all_data(
        &mut self,
        pid_1: &PidData,
        pid_2: &PidData,
        pid_bg: &PidData,
        pump: &PumpData,
        interface: &Interface,
    ) -> Result<(), E> {
        let config = Config {
            pump_data: pump.clone(),
            pid_1_data: PidConfigData::from(pid_1),
            pid_2_data: PidConfigData::from(pid_2),
            pid_bg_data: PidConfigData::from(pid_bg),
            interface_temps: InterfaceConfigData::from(interface),
        };
        self.save_config(&config)
    }

    // ── Individual-section save/load helpers ──────────────────────────────────
    // Each helper does load → mutate → save_config (one erase, preserves other sections).

    pub fn update_from_pid_data(
        &mut self,
        pid_1: &PidData,
        pid_2: &PidData,
        pid_bg: &PidData,
    ) -> Result<(), E> {
        let mut config = self.load_config()?;
        config.pid_1_data = PidConfigData::from(pid_1);
        config.pid_2_data = PidConfigData::from(pid_2);
        config.pid_bg_data = PidConfigData::from(pid_bg);
        self.save_config(&config)
    }

    pub fn update_from_pump_data(&mut self, pump: &PumpData) -> Result<(), E> {
        let mut config = self.load_config()?;
        config.pump_data = pump.clone();
        self.save_config(&config)
    }

    pub fn update_from_interface(&mut self, interface: &Interface) -> Result<(), E> {
        let mut config = self.load_config()?;
        config.interface_temps = InterfaceConfigData::from(interface);
        self.save_config(&config)
    }

    pub fn apply_to_pid_data(
        &mut self,
        pid_1: &mut PidData,
        pid_2: &mut PidData,
        pid_bg: &mut PidData,
    ) -> Result<(), E> {
        let config = self.load_config()?;
        apply_pid(&config.pid_1_data, pid_1);
        apply_pid(&config.pid_2_data, pid_2);
        apply_pid(&config.pid_bg_data, pid_bg);
        usb_println("[CFG] PID config loaded");
        Ok(())
    }

    pub fn apply_to_pump_data(&mut self, pump: &mut PumpData) -> Result<(), E> {
        let config = self.load_config()?;
        *pump = config.pump_data.clone();
        Ok(())
    }

    pub fn apply_to_interface(&mut self, interface: &mut Interface) -> Result<(), E> {
        let config = self.load_config()?;
        interface.coffee_temperature = config.interface_temps.coffee_temperature;
        interface.brew_head_temperature = config.interface_temps.brew_head_temperature;
        interface.steam_temperature = config.interface_temps.steam_temperature;
        Ok(())
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

fn apply_pid(src: &PidConfigData, dst: &mut PidData) {
    dst.kp = src.kp;
    dst.ki = src.ki;
    dst.kd = src.kd;
    dst.window_size = src.window_size;
    dst.max_val = src.max_val;
    dst.osr = src.osr;
    dst.target = src.target;
}
