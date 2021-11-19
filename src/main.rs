#![no_std]
#![no_main]

use panic_reset as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::pac::Peripherals;

use crate::sdvx_animation::{SdvxAnimation, SdvxFadeAnimation};
use crate::sdvx_controller::SdvxController;

mod sdvx_animation;
mod sdvx_bcm;
mod sdvx_controller;
mod sdvx_cos_table;
mod sdvx_keycode;
mod sdvx_status;
mod sdvx_sys_clock;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let mut controller = SdvxController::new(SdvxFadeAnimation::new(), dp);

    loop {
        controller.tick();
    }
}
