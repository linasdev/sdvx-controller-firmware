use cortex_m::interrupt::free;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::gpio::gpiob::{PB13, PB14, PB15};
use stm32f1xx_hal::gpio::{Alternate, Output, PushPull};
use stm32f1xx_hal::pac::{NVIC, SPI2, SYST, TIM2};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::rcc::{Clocks, APB1};
use stm32f1xx_hal::spi::{Mode, NoMiso, Phase, Polarity, Spi, Spi2NoRemap};
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::timer::{CountDownTimer, Event, Timer};

use crate::sdvx_sin_table::SDVX_SIN_TABLE;
use crate::sdvx_status::SdvxStatus;

// Initial frequency for binary code modulation in Hertz
static BCM_INITIAL_FREQUENCY_HZ: u32 = 65_536;
// Frequency for data transfer to the shift registers
static BCM_SPI_FREQUENCY_HZ: u32 = 8_000_000;
// Total amount of led outputs (must be a multiple of 8)
const BCM_LED_COUNT: usize = 24;
const BCM_LED_BRIGHTNESS_MULTIPLIER: f32 = 1.0;
// This multiplied by 255 is used for the systick timer reload value. The higher the value, the slower the animation. Must be in the range 0 - 65793
const BCM_ANIMATION_SPEED: u32 = 800_000;
const BCM_ANIMATION_TIME_TICKS: u32 = 50_000_000;
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

pub struct SdvxBcm {
    led_brightness: &'static mut [u8; BCM_LED_COUNT],
    start_tick: u32,
    button1_tick: u32,
    button2_tick: u32,
    button3_tick: u32,
    button4_tick: u32,
    fx_l_tick: u32,
    fx_r_tick: u32,
    rotary1_tick_ccw: u32,
    rotary1_tick_cw: u32,
    rotary2_tick_ccw: u32,
    rotary2_tick_cw: u32,
}

impl SdvxBcm {
    pub fn new(
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
            start_tick: 0,
            button1_tick: 0,
            button2_tick: 0,
            button3_tick: 0,
            button4_tick: 0,
            fx_l_tick: 0,
            fx_r_tick: 0,
            rotary1_tick_ccw: 0,
            rotary1_tick_cw: 0,
            rotary2_tick_ccw: 0,
            rotary2_tick_cw: 0,
        }
    }

    pub fn tick(&mut self, status: &SdvxStatus, current_tick: u32) {
        let sin_value = SDVX_SIN_TABLE[(current_tick / BCM_ANIMATION_SPEED) as usize % 256];
        let mut new_led_brightness = [0x00u8; BCM_LED_COUNT];

        if status.button1_pressed || self.button1_tick + BCM_ANIMATION_TIME_TICKS > current_tick {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 0, sin_value, 0xff - sin_value, 0);

            if status.button1_pressed {
                self.button1_tick = current_tick;
            }
        }

        if status.button2_pressed || self.button2_tick + BCM_ANIMATION_TIME_TICKS > current_tick {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 1, 0, sin_value, 0xff - sin_value);

            if status.button2_pressed {
                self.button2_tick = current_tick;
            }
        }

        if status.button3_pressed || self.button3_tick + BCM_ANIMATION_TIME_TICKS > current_tick {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 2, 0, sin_value, 0xff - sin_value);

            if status.button3_pressed {
                self.button3_tick = current_tick;
            }
        }

        if status.button4_pressed || self.button4_tick + BCM_ANIMATION_TIME_TICKS > current_tick {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 3, sin_value, 0xff - sin_value, 0);

            if status.button4_pressed {
                self.button4_tick = current_tick;
            }
        }

        if status.fx_l_pressed || self.fx_l_tick + BCM_ANIMATION_TIME_TICKS > current_tick {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 4, sin_value, 0xff - sin_value, 0);

            if status.fx_l_pressed {
                self.fx_l_tick = current_tick;
            }
        }

        if status.fx_r_pressed || self.fx_r_tick + BCM_ANIMATION_TIME_TICKS > current_tick {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 5, sin_value, 0xff - sin_value, 0);

            if status.fx_r_pressed {
                self.fx_r_tick = current_tick;
            }
        }

        if status.rotary1_rotated_ccw || self.rotary1_tick_ccw + BCM_ANIMATION_TIME_TICKS > current_tick  {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 0, sin_value, 0xff - sin_value, 0);
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 4, sin_value, 0, 0xff - sin_value);

            if status.rotary1_rotated_ccw {
                self.rotary1_tick_ccw = current_tick;
                self.rotary1_tick_cw = 0;
            }
        }

        if status.rotary1_rotated_cw || self.rotary1_tick_cw + BCM_ANIMATION_TIME_TICKS > current_tick  {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 0, sin_value, 0, 0xff - sin_value);
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 4, sin_value, 0xff - sin_value, 0);

            if status.rotary1_rotated_cw {
                self.rotary1_tick_ccw = 0;
                self.rotary1_tick_cw = current_tick;
            }
        }

        if status.rotary2_rotated_ccw || self.rotary2_tick_ccw + BCM_ANIMATION_TIME_TICKS > current_tick  {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 3, sin_value, 0xff - sin_value, 0);
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 5, sin_value, 0, 0xff - sin_value);

            if status.rotary2_rotated_ccw {
                self.rotary2_tick_ccw = current_tick;
                self.rotary2_tick_cw = 0;
            }
        }

        if status.rotary2_rotated_cw || self.rotary2_tick_cw + BCM_ANIMATION_TIME_TICKS > current_tick  {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 3, sin_value, 0, 0xff - sin_value);
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 5, sin_value, 0xff - sin_value, 0);

            if status.rotary2_rotated_cw {
                self.rotary2_tick_ccw = 0;
                self.rotary2_tick_cw = current_tick;
            }
        }

        if status.start_pressed || self.start_tick + BCM_ANIMATION_TIME_TICKS > current_tick  {
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 1, sin_value, 0, 0xff - sin_value);
            SdvxBcm::modify_led_value_rgb(&mut new_led_brightness, 2, sin_value, 0, 0xff - sin_value);

            if status.start_pressed {
                self.start_tick = current_tick;
            }
        }

        SdvxBcm::modify_led_values(self.led_brightness, &new_led_brightness);
    }

    fn modify_led_value(led_brightness: &mut [u8; BCM_LED_COUNT], led_index: usize, led_value: u8) {
        free(|_| {
            (*led_brightness)[led_index] = (led_value as f32 * BCM_LED_BRIGHTNESS_MULTIPLIER) as u8;
        });
    }

    fn modify_led_value_rgb(
        led_brightness: &mut [u8; BCM_LED_COUNT],
        index: usize,
        red_value: u8,
        green_value: u8,
        blue_value: u8,
    ) {
        SdvxBcm::modify_led_value(led_brightness, index * 3 + 0, red_value);
        SdvxBcm::modify_led_value(led_brightness, index * 3 + 1, green_value);
        SdvxBcm::modify_led_value(led_brightness, index * 3 + 2, blue_value);
    }

    fn modify_led_values(led_brightness: &mut [u8; BCM_LED_COUNT], new_led_brightness: &[u8; BCM_LED_COUNT]) {
        free(|_| {
            for i in 0..led_brightness.len() {
                (*led_brightness)[i] = new_led_brightness[i];
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
        shift_latch.set_low().unwrap();
        shift_spi.write(&buffer).unwrap();
        shift_latch.set_high().unwrap();
    }

    bcm_timer.start((*bcm_current_frequency_hz).hz());
    bcm_timer.wait().unwrap();
}
