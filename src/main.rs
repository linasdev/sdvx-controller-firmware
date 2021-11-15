#![no_std]
#![no_main]

use panic_reset as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::pac::{CorePeripherals, Peripherals};

use crate::sdvx_animation::{SdvxAnimation, SdvxFadeAnimation};
use crate::sdvx_controller::SdvxController;

mod sdvx_animation;
mod sdvx_bcm;
mod sdvx_controller;
mod sdvx_cos_table;
mod sdvx_keycode;
mod sdvx_status;

#[entry]
fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut controller = SdvxController::new(SdvxFadeAnimation::new(), cp, dp);

    loop {
        controller.tick();
    }
}
