use core::cell::{Cell, RefCell};
use core::sync::atomic::Ordering;
use panic_halt as _;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
    pac::{self, interrupt},
    gpio::{self, Input},
    prelude::*,
};
use stm32f4xx_hal::timer::Timer;

// Create a Global Variable for the GPIO Peripheral that I'm going to pass around.
pub static G_ENC_PIN_A: Mutex<RefCell<Option<gpio::PE11<Input>>>> = Mutex::new(RefCell::new(None));
pub static G_ENC_PIN_B: Mutex<RefCell<Option<gpio::PE9<Input>>>> = Mutex::new(RefCell::new(None));
// Create a Global Variable for the state
pub static G_ENC_STATE: Mutex<Cell<isize>> = Mutex::new(Cell::new(0));

const ENCODER_STEP: isize = 1;

#[interrupt]
fn EXTI9_5() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain access to Global Encoder Peripheral and Clear Interrupt Pending Flag
        let mut button_a = G_ENC_PIN_A.borrow(cs).borrow_mut();
        let mut button_b = G_ENC_PIN_B.borrow(cs).borrow_mut();
        if button_a.as_ref().unwrap().is_high() && button_b.as_ref().unwrap().is_low() {
            G_ENC_STATE
                .borrow(cs)
                .set(G_ENC_STATE.borrow(cs).get() - ENCODER_STEP);
        } else if button_a.as_ref().unwrap().is_low() && button_b.as_ref().unwrap().is_high() {
            G_ENC_STATE
                .borrow(cs)
                .set(G_ENC_STATE.borrow(cs).get() - ENCODER_STEP);
        } else if button_a.as_ref().unwrap().is_low() && button_b.as_ref().unwrap().is_low() {
            G_ENC_STATE
                .borrow(cs)
                .set(G_ENC_STATE.borrow(cs).get() + ENCODER_STEP);
        } else if button_a.as_ref().unwrap().is_high() && button_b.as_ref().unwrap().is_high() {
            G_ENC_STATE
                .borrow(cs)
                .set(G_ENC_STATE.borrow(cs).get() + ENCODER_STEP);
        }
        button_b.as_mut().unwrap().clear_interrupt_pending_bit();
    });
}

