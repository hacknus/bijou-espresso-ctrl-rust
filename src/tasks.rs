use freertos_rust::Duration;
use crate::{LED, usb_println};

pub fn blink_led_1_task<const P: char, const N: u8>(mut led: LED<P, N>) {
    loop {
        freertos_rust::CurrentTask::delay(Duration::ms(333));
        led.on();
        freertos_rust::CurrentTask::delay(Duration::ms(333));
        led.off();
    }
}

pub fn blink_led_2_task<const P: char, const N: u8>(mut led: LED<P, N>) {
    loop {
        freertos_rust::CurrentTask::delay(Duration::ms(500));
        led.on();
        freertos_rust::CurrentTask::delay(Duration::ms(500));
        led.off();
    }
}

pub fn print_usb_task() {
    loop {
        freertos_rust::CurrentTask::delay(Duration::ms(333));
        usb_println("testing...");
    }
}