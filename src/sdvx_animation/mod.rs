use crate::sdvx_bcm::BCM_LED_COUNT;
use crate::sdvx_status::SdvxStatus;

mod sdvx_fade_animation;
pub use sdvx_fade_animation::*;

pub trait SdvxAnimation {
    fn new() -> Self;
    fn tick(&mut self, status: &SdvxStatus, current_tick: u64) -> [u8; BCM_LED_COUNT];
}
