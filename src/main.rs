#![no_std]
#![no_main]

use panic_reset as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::pac::Peripherals;

use crate::sdvx_controller::SdvxController;

mod sdvx_controller;
mod sdvx_keycode;
mod sdvx_bcm;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let mut controller = SdvxController::new(dp);

    loop {
        controller.tick();
    }
}
