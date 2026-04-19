use crate::config::ConfigManager;
use crate::usb::usb_println;
use crate::utils::{Interface, PidData, PumpData};
use alloc::sync::Arc;
use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};
use freertos_rust::{Duration, Mutex};

/// Load all config sections from flash into their runtime containers.
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
    if let (Ok(mut pid_1), Ok(mut pid_2), Ok(mut pid_bg), Ok(mut pump), Ok(mut interface)) = (
        pid_1_data_container.lock(Duration::ms(5)),
        pid_2_data_container.lock(Duration::ms(5)),
        pid_bg_data_container.lock(Duration::ms(5)),
        pump_data_container.lock(Duration::ms(5)),
        interface_data_container.lock(Duration::ms(5)),
    ) {
        let _ = config_mgr.apply_to_pid_data(&mut pid_1, &mut pid_2, &mut pid_bg);
        let _ = config_mgr.apply_to_pump_data(&mut pump);
        let _ = config_mgr.apply_to_interface(&mut interface);
        usb_println("[CFG] all config loaded");
        return Ok(());
    }

    usb_println("[CFG] failed to lock containers for load");
    Err(())
}

/// Save all config sections to flash in a single erase cycle.
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
    if let (Ok(pid_1), Ok(pid_2), Ok(pid_bg), Ok(pump), Ok(interface)) = (
        pid_1_data_container.lock(Duration::ms(5)),
        pid_2_data_container.lock(Duration::ms(5)),
        pid_bg_data_container.lock(Duration::ms(5)),
        pump_data_container.lock(Duration::ms(5)),
        interface_data_container.lock(Duration::ms(5)),
    ) {
        // One call → one sector erase → five page writes.
        let _ = config_mgr.save_all_data(&pid_1, &pid_2, &pid_bg, &pump, &interface);
        return Ok(());
    }

    usb_println("[CFG] failed to lock containers for save");
    Err(())
}
