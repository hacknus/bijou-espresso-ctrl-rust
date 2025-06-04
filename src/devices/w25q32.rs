// src/devices/w25q32.rs
pub use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::{Mode, Phase, Polarity};

pub const FLASH_SPI_MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

// W25Q32 commands
const WRITE_ENABLE: u8 = 0x06;
const WRITE_DISABLE: u8 = 0x04;
const READ_STATUS_REG1: u8 = 0x05;
const WRITE_STATUS_REG1: u8 = 0x01;
const PAGE_PROGRAM: u8 = 0x02;
const QUAD_PAGE_PROGRAM: u8 = 0x32;
const BLOCK_ERASE_64K: u8 = 0xD8;
const BLOCK_ERASE_32K: u8 = 0x52;
const SECTOR_ERASE: u8 = 0x20;
const CHIP_ERASE: u8 = 0xC7;
const READ_DATA: u8 = 0x03;
const FAST_READ: u8 = 0x0B;

// Memory map for configuration storage
pub const PUMP_DATA_ADDR: u32 = 0x000000; // 32 bytes
pub const PID_1_DATA_ADDR: u32 = 0x000100; // 32 bytes
pub const PID_2_DATA_ADDR: u32 = 0x000200; // 32 bytes
pub const PID_BG_DATA_ADDR: u32 = 0x000300; // 32 bytes
pub const INTERFACE_DATA_ADDR: u32 = 0x000400; // 32 bytes

pub struct W25Q32<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS, E> W25Q32<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        W25Q32 { spi, cs }
    }

    fn select(&mut self) -> Result<(), E> {
        self.cs.set_low().map_err(|_| panic!("CS error"))
    }

    fn deselect(&mut self) -> Result<(), E> {
        self.cs.set_high().map_err(|_| panic!("CS error"))
    }

    pub fn read_id(&mut self) -> Result<[u8; 3], E> {
        let mut cmd_buf = [0x9F, 0, 0, 0];
        let mut id = [0; 3];

        self.select()?;
        self.spi.transfer(&mut cmd_buf)?;
        self.deselect()?;

        id.copy_from_slice(&cmd_buf[1..4]);
        Ok(id)
    }

    pub fn is_busy(&mut self) -> Result<bool, E> {
        let mut cmd_buf = [READ_STATUS_REG1, 0];

        self.select()?;
        self.spi.transfer(&mut cmd_buf)?;
        self.deselect()?;

        Ok((cmd_buf[1] & 0x01) != 0)
    }

    pub fn wait_busy(&mut self) -> Result<(), E> {
        while self.is_busy()? {}
        Ok(())
    }

    pub fn write_enable(&mut self) -> Result<(), E> {
        self.select()?;
        self.spi.write(&[WRITE_ENABLE])?;
        self.deselect()?;
        Ok(())
    }

    pub fn sector_erase(&mut self, addr: u32) -> Result<(), E> {
        self.write_enable()?;

        let cmd = [
            SECTOR_ERASE,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];

        self.select()?;
        self.spi.write(&cmd)?;
        self.deselect()?;

        self.wait_busy()?;
        Ok(())
    }

    pub fn page_program(&mut self, addr: u32, data: &[u8]) -> Result<(), E> {
        if data.len() > 256 {
            panic!("Data too large for page program");
        }

        self.write_enable()?;

        let cmd = [
            PAGE_PROGRAM,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];

        self.select()?;
        self.spi.write(&cmd)?;
        self.spi.write(data)?;
        self.deselect()?;

        self.wait_busy()?;
        Ok(())
    }

    pub fn read_data(&mut self, addr: u32, data: &mut [u8]) -> Result<(), E> {
        let cmd = [
            READ_DATA,
            ((addr >> 16) & 0xFF) as u8,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];

        self.select()?;
        self.spi.write(&cmd)?;

        // Create a buffer filled with zeros for the read operation
        let mut read_buf = [0u8; 256]; // Adjust size as needed
        let read_len = core::cmp::min(data.len(), read_buf.len());

        self.spi.transfer(&mut read_buf[0..read_len])?;
        self.deselect()?;

        // Copy the read data to the output buffer
        data[0..read_len].copy_from_slice(&read_buf[0..read_len]);

        Ok(())
    }
}
