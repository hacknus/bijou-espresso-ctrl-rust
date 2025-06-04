use crate::config::ConfigManager;
use crate::usb::usb_println;
use crate::utils::{Interface, PidData, PumpData};
use alloc::sync::Arc;
use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};
use freertos_rust::{Duration, Mutex};

// Helper function to load all configurations
pub fn load_all_config<SPI, CS, E>(
    config_mgr: &mut ConfigManager<SPI, CS>,
    pid_1_data_container: &Arc<Mutex<PidData>>,
    pid_2_data_container: &Arc<Mutex<PidData>>,
    pid_bg_data_container: &Arc<Mutex<PidData>>,
    pump_data_container: &Arc<Mutex<PumpData>>,
    interface_data_container: &Arc<Mutex<Interface>>,
) -> Result<(), ()>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
    E: core::fmt::Debug,
{
    // Try to load configuration from flash
    if let (Ok(mut pid_1), Ok(mut pid_2), Ok(mut pid_bg), Ok(mut pump), Ok(mut interface)) = (
        pid_1_data_container.lock(Duration::ms(5)),
        pid_2_data_container.lock(Duration::ms(5)),
        pid_bg_data_container.lock(Duration::ms(5)),
        pump_data_container.lock(Duration::ms(5)),
        interface_data_container.lock(Duration::ms(5)),
    ) {
        // Load configuration in order of importance
        let _ = config_mgr.apply_to_pid_data(&mut pid_1, &mut pid_2, &mut pid_bg);
        let _ = config_mgr.apply_to_pump_data(&mut pump);
        let _ = config_mgr.apply_to_interface(&mut interface);
        usb_println("All configurations loaded from flash");
        return Ok(());
    }

    usb_println("Failed to lock data containers for loading config");
    Err(())
}

// Helper function to save all configurations
pub fn save_all_config<SPI, CS, E>(
    config_mgr: &mut ConfigManager<SPI, CS>,
    pid_1_data_container: &Arc<Mutex<PidData>>,
    pid_2_data_container: &Arc<Mutex<PidData>>,
    pid_bg_data_container: &Arc<Mutex<PidData>>,
    pump_data_container: &Arc<Mutex<PumpData>>,
    interface_data_container: &Arc<Mutex<Interface>>,
) -> Result<(), ()>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
    E: core::fmt::Debug,
{
    // Try to save configuration to flash
    if let (Ok(pid_1), Ok(pid_2), Ok(pid_bg), Ok(pump), Ok(interface)) = (
        pid_1_data_container.lock(Duration::ms(5)),
        pid_2_data_container.lock(Duration::ms(5)),
        pid_bg_data_container.lock(Duration::ms(5)),
        pump_data_container.lock(Duration::ms(5)),
        interface_data_container.lock(Duration::ms(5)),
    ) {
        // First update PID data as it's most critical
        let _ = config_mgr.update_from_pid_data(&pid_1, &pid_2, &pid_bg);
        // Then update other configurations
        let _ = config_mgr.update_from_pump_data(&pump);
        let _ = config_mgr.update_from_interface(&interface);
        usb_println("All configurations saved to flash");
        return Ok(());
    }

    usb_println("Failed to lock data containers for saving config");
    Err(())
}
