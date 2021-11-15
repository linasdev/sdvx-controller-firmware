use cortex_m::interrupt::free;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::gpio::gpiob::{PB13, PB14, PB15};
use stm32f1xx_hal::gpio::{Alternate, Output, PushPull};
use stm32f1xx_hal::pac::{NVIC, SPI2, TIM2};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::rcc::{Clocks, APB1};
use stm32f1xx_hal::spi::{Mode, NoMiso, Phase, Polarity, Spi, Spi2NoRemap};
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::timer::{CountDownTimer, Event, Timer};

use crate::sdvx_animation::SdvxAnimation;
use crate::sdvx_status::SdvxStatus;

// Initial frequency for binary code modulation in Hertz
static BCM_INITIAL_FREQUENCY_HZ: u32 = 65_536;
// Frequency for data transfer to the shift registers
static BCM_SPI_FREQUENCY_HZ: u32 = 8_000_000;
// Total amount of led outputs (must be a multiple of 8)
pub const BCM_LED_COUNT: usize = 24;
const BCM_LED_BRIGHTNESS_MULTIPLIER: f32 = 1.0;
// Timer for binary code modulation
static mut BCM_TIMER: Option<CountDownTimer<TIM2>> = None;
// Current frequency for binary code modulation in Hertz
static mut BCM_CURRENT_FREQUENCY_HZ: u32 = BCM_INITIAL_FREQUENCY_HZ;
// Current bitmask for binary code modulation
static mut BCM_CURRENT_BITMASK: u8 = 0x01;
// LED brightness values for binary code modulation
static mut BCM_LED_BRIGHTNESS: [u8; BCM_LED_COUNT] = [0x00; BCM_LED_COUNT];
static mut SHIFT_LATCH: Option<PB14<Output<PushPull>>> = None;
static mut SHIFT_SPI: Option<
    Spi<SPI2, Spi2NoRemap, (PB13<Alternate<PushPull>>, NoMiso, PB15<Alternate<PushPull>>), u8>,
> = None;

pub struct SdvxBcm<A: SdvxAnimation> {
    led_brightness: &'static mut [u8; BCM_LED_COUNT],
    animation: A,
}

impl<A: SdvxAnimation> SdvxBcm<A> {
    pub fn new(
        animation: A,
        shift_clock: PB13<Alternate<PushPull>>,
        shift_latch: PB14<Output<PushPull>>,
        shift_data: PB15<Alternate<PushPull>>,
        spi2: SPI2,
        clocks: Clocks,
        apb1: &mut APB1,
        tim2: TIM2,
    ) -> Self {
        let shift_spi = {
            let pins = (shift_clock, NoMiso, shift_data);

            let spi_mode = Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            };

            Spi::spi2(
                spi2,
                pins,
                spi_mode,
                BCM_SPI_FREQUENCY_HZ.hz(),
                clocks,
                apb1,
            )
        };

        unsafe {
            SHIFT_LATCH = Some(shift_latch);
            SHIFT_SPI = Some(shift_spi);
        }

        let led_brightness = unsafe { &mut BCM_LED_BRIGHTNESS };

        let mut bcm_timer =
            Timer::tim2(tim2, &clocks, apb1).start_count_down(BCM_INITIAL_FREQUENCY_HZ.hz());
        bcm_timer.listen(Event::Update);

        unsafe {
            BCM_TIMER = Some(bcm_timer);

            // For binary code modulation
            NVIC::unmask(Interrupt::TIM2);
        }

        SdvxBcm {
            led_brightness,
            animation,
        }
    }

    pub fn tick(&mut self, status: &SdvxStatus, current_tick: u32) {
        let new_led_brightness = self.animation.tick(status, current_tick);
        self.swap_led_values(&new_led_brightness);
    }

    fn swap_led_values(&mut self, new_led_brightness: &[u8; BCM_LED_COUNT]) {
        free(|_| {
            for i in 0..self.led_brightness.len() {
                (*self.led_brightness)[i] =
                    (new_led_brightness[i] as f32 * BCM_LED_BRIGHTNESS_MULTIPLIER) as u8;
            }
        });
    }
}

#[interrupt]
fn TIM2() {
    let shift_latch = unsafe { SHIFT_LATCH.as_mut().unwrap() };
    let shift_spi = unsafe { SHIFT_SPI.as_mut().unwrap() };

    let bcm_timer = unsafe { BCM_TIMER.as_mut().unwrap() };
    let bcm_current_frequency_hz = unsafe { &mut BCM_CURRENT_FREQUENCY_HZ };
    let bcm_current_bitmask = unsafe { &mut BCM_CURRENT_BITMASK };

    let bcm_led_brightness = unsafe { &BCM_LED_BRIGHTNESS };

    // Decrease the frequency by a factor of 2
    *bcm_current_frequency_hz >>= 1;
    // Move onto the next bit
    *bcm_current_bitmask <<= 1;

    if *bcm_current_bitmask == 0 {
        *bcm_current_frequency_hz = BCM_INITIAL_FREQUENCY_HZ;
        *bcm_current_bitmask = 0x01;
    }

    shift_latch.set_low().unwrap();

    for i in 0..bcm_led_brightness.len() / 8 {
        let mut value = 0x00;

        for j in 0..=7 {
            let brightness = bcm_led_brightness[bcm_led_brightness.len() - (i * 8 + j) - 1];

            value <<= 1;

            if (brightness & (*bcm_current_bitmask)) != 0 {
                value |= 1;
            }
        }

        let buffer = [value; 1];
        shift_spi.write(&buffer).unwrap();
    }

    shift_latch.set_high().unwrap();

    bcm_timer.start((*bcm_current_frequency_hz).hz());
    bcm_timer.wait().unwrap();
}
