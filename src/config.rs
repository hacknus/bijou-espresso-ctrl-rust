// src/config.rs
use crate::devices::w25q32::{
    INTERFACE_DATA_ADDR, PID_1_DATA_ADDR, PID_2_DATA_ADDR, PID_BG_DATA_ADDR, PUMP_DATA_ADDR, W25Q32,
};
use crate::usb::usb_println;
use crate::utils::{Interface, PidData, PumpData};
use arrform::{arrform, ArrForm};
use core::mem;
use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};

// Configuration struct to store our settings
#[derive(Clone, Default)]
pub struct Config {
    pub pump_data: PumpData,
    pub pid_1_data: PidConfigData,
    pub pid_2_data: PidConfigData,
    pub pid_bg_data: PidConfigData,
    pub interface_temps: InterfaceConfigData,
}

// Only store the essential PID parameters
#[derive(Clone, Default)]
pub struct PidConfigData {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub window_size: u32,
    pub max_val: f32,
    pub osr: u32,
    pub target: f32,
}

// Only store temperature settings
#[derive(Clone, Default)]
pub struct InterfaceConfigData {
    pub coffee_temperature: f32,
    pub brew_head_temperature: f32,
    pub steam_temperature: f32,
}

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
        usb_println(arrform!(164, "W25Q32 ID: {:02X} {:02X} {:02X}", id[0], id[1], id[2]).as_str());
        Ok(ConfigManager { flash })
    }

    pub fn load_config(&mut self) -> Result<Config, E> {
        let mut config = Config::default();

        // Buffer for reading data
        let mut buffer = [0u8; 256];

        // Read PumpData
        self.flash
            .read_data(PUMP_DATA_ADDR, &mut buffer[0..mem::size_of::<PumpData>()])?;
        if is_valid_f32_data(&buffer[0..mem::size_of::<PumpData>()]) {
            unsafe {
                config.pump_data = core::ptr::read(buffer.as_ptr() as *const PumpData);
            }
        }

        // Read PID 1 data
        self.flash.read_data(
            PID_1_DATA_ADDR,
            &mut buffer[0..mem::size_of::<PidConfigData>()],
        )?;
        if is_valid_f32_data(&buffer[0..mem::size_of::<PidConfigData>()]) {
            unsafe {
                config.pid_1_data = core::ptr::read(buffer.as_ptr() as *const PidConfigData);
            }
        }

        // Read PID 2 data
        self.flash.read_data(
            PID_2_DATA_ADDR,
            &mut buffer[0..mem::size_of::<PidConfigData>()],
        )?;
        if is_valid_f32_data(&buffer[0..mem::size_of::<PidConfigData>()]) {
            unsafe {
                config.pid_2_data = core::ptr::read(buffer.as_ptr() as *const PidConfigData);
            }
        }

        // Read PID BG data
        self.flash.read_data(
            PID_BG_DATA_ADDR,
            &mut buffer[0..mem::size_of::<PidConfigData>()],
        )?;
        if is_valid_f32_data(&buffer[0..mem::size_of::<PidConfigData>()]) {
            unsafe {
                config.pid_bg_data = core::ptr::read(buffer.as_ptr() as *const PidConfigData);
            }
        }

        // Read Interface data
        self.flash.read_data(
            INTERFACE_DATA_ADDR,
            &mut buffer[0..mem::size_of::<InterfaceConfigData>()],
        )?;
        if is_valid_f32_data(&buffer[0..mem::size_of::<InterfaceConfigData>()]) {
            unsafe {
                config.interface_temps =
                    core::ptr::read(buffer.as_ptr() as *const InterfaceConfigData);
            }
        }

        Ok(config)
    }

    pub fn save_config(&mut self, config: &Config) -> Result<(), E> {
        // Erase the sectors before writing
        self.flash.sector_erase(PUMP_DATA_ADDR)?;

        // Write PumpData
        let pump_data_bytes = unsafe {
            core::slice::from_raw_parts(
                &config.pump_data as *const PumpData as *const u8,
                mem::size_of::<PumpData>(),
            )
        };
        self.flash.page_program(PUMP_DATA_ADDR, pump_data_bytes)?;

        // Write PID 1 data
        let pid_1_bytes = unsafe {
            core::slice::from_raw_parts(
                &config.pid_1_data as *const PidConfigData as *const u8,
                mem::size_of::<PidConfigData>(),
            )
        };
        self.flash.page_program(PID_1_DATA_ADDR, pid_1_bytes)?;

        // Write PID 2 data
        let pid_2_bytes = unsafe {
            core::slice::from_raw_parts(
                &config.pid_2_data as *const PidConfigData as *const u8,
                mem::size_of::<PidConfigData>(),
            )
        };
        self.flash.page_program(PID_2_DATA_ADDR, pid_2_bytes)?;

        // Write PID BG data
        let pid_bg_bytes = unsafe {
            core::slice::from_raw_parts(
                &config.pid_bg_data as *const PidConfigData as *const u8,
                mem::size_of::<PidConfigData>(),
            )
        };
        self.flash.page_program(PID_BG_DATA_ADDR, pid_bg_bytes)?;

        // Write Interface data
        let interface_bytes = unsafe {
            core::slice::from_raw_parts(
                &config.interface_temps as *const InterfaceConfigData as *const u8,
                mem::size_of::<InterfaceConfigData>(),
            )
        };
        self.flash
            .page_program(INTERFACE_DATA_ADDR, interface_bytes)?;

        usb_println("Configuration saved to flash");
        Ok(())
    }

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

        pid_1.kp = config.pid_1_data.kp;
        pid_1.ki = config.pid_1_data.ki;
        pid_1.kd = config.pid_1_data.kd;
        pid_1.window_size = config.pid_1_data.window_size;
        pid_1.max_val = config.pid_1_data.max_val;
        pid_1.osr = config.pid_1_data.osr;
        pid_1.target = config.pid_1_data.target;

        pid_2.kp = config.pid_2_data.kp;
        pid_2.ki = config.pid_2_data.ki;
        pid_2.kd = config.pid_2_data.kd;
        pid_2.window_size = config.pid_2_data.window_size;
        pid_2.max_val = config.pid_2_data.max_val;
        pid_2.osr = config.pid_2_data.osr;
        pid_2.target = config.pid_2_data.target;

        pid_bg.kp = config.pid_bg_data.kp;
        pid_bg.ki = config.pid_bg_data.ki;
        pid_bg.kd = config.pid_bg_data.kd;
        pid_bg.window_size = config.pid_bg_data.window_size;
        pid_bg.max_val = config.pid_bg_data.max_val;
        pid_bg.osr = config.pid_bg_data.osr;
        pid_bg.target = config.pid_bg_data.target;

        usb_println("Configuration loaded from flash");
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

// Helper function to validate if the data contains valid f32 values
// Simple check to make sure we're not reading uninitialized flash memory
fn is_valid_f32_data(data: &[u8]) -> bool {
    // Check if all bytes are 0xFF (erased flash)
    if data.iter().all(|&b| b == 0xFF) {
        return false;
    }

    // Check if data is all zeros
    if data.iter().all(|&b| b == 0x00) {
        return false;
    }

    // For more sophisticated validation, we could parse the f32 values and check ranges
    true
}
