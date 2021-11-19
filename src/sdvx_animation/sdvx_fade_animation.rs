use core::cmp::max;

use crate::sdvx_animation::SdvxAnimation;
use crate::sdvx_bcm::BCM_LED_COUNT;
use crate::sdvx_cos_table::SDVX_COS_TABLE;
use crate::sdvx_status::SdvxStatus;

// Amount of ticks for the animation to end after a button is released
const FADE_ANIMATION_TIME_TICKS: u64 = 20_000_000;
const BUTTON1_LED_INDEX: usize = 18;
const BUTTON2_LED_INDEX: usize = 19;
const BUTTON3_LED_INDEX: usize = 20;
const BUTTON4_LED_INDEX: usize = 21;
const FX_L_LED_INDEX: usize = 22;
const FX_R_LED_INDEX: usize = 23;

pub struct SdvxFadeAnimation {
    new_led_brightness: [u8; BCM_LED_COUNT],
    start_tick: u64,
    button1_tick: u64,
    button2_tick: u64,
    button3_tick: u64,
    button4_tick: u64,
    fx_l_tick: u64,
    fx_r_tick: u64,
    rotary1_tick_ccw: u64,
    rotary1_tick_cw: u64,
    rotary2_tick_ccw: u64,
    rotary2_tick_cw: u64,
}

impl SdvxAnimation for SdvxFadeAnimation {
    fn new() -> Self {
        Self {
            new_led_brightness: [0; BCM_LED_COUNT],
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

    fn tick(&mut self, status: &SdvxStatus, current_tick: u64) -> [u8; BCM_LED_COUNT] {
        self.clear_led_values();
        self.set_button_ticks(&status, current_tick);

        if Self::should_animate(
            status.rotary1_rotated_ccw,
            self.rotary1_tick_ccw,
            current_tick,
        ) {
            let cos_value = Self::get_cos_value(self.rotary1_tick_ccw, current_tick);
            self.modify_led_value_rb(0, 0xff - cos_value, cos_value);
        }

        if Self::should_animate(
            status.rotary1_rotated_cw,
            self.rotary1_tick_cw,
            current_tick,
        ) {
            let cos_value = Self::get_cos_value(self.rotary1_tick_cw, current_tick);
            self.modify_led_value_rb(1, 0xff - cos_value, cos_value);
        }

        if Self::should_animate(
            status.rotary2_rotated_ccw,
            self.rotary2_tick_ccw,
            current_tick,
        ) {
            let cos_value = Self::get_cos_value(self.rotary2_tick_ccw, current_tick);
            self.modify_led_value_rb(2, 0xff - cos_value, cos_value);
        }

        if Self::should_animate(
            status.rotary2_rotated_cw,
            self.rotary2_tick_cw,
            current_tick,
        ) {
            let cos_value = Self::get_cos_value(self.rotary2_tick_cw, current_tick);
            self.modify_led_value_rb(3, 0xff - cos_value, cos_value);
        }

        if Self::should_animate(status.button1_pressed, self.button1_tick, current_tick) {
            let cos_value = Self::get_cos_value(self.button1_tick, current_tick);
            self.modify_led_value_rb(0, 0xff - cos_value, cos_value);
            self.modify_led_value(BUTTON1_LED_INDEX, cos_value);
        }

        if Self::should_animate(status.button2_pressed, self.button2_tick, current_tick) {
            let cos_value = Self::get_cos_value(self.button2_tick, current_tick);
            self.modify_led_value_rb(1, 0xff - cos_value, cos_value);
            self.modify_led_value(BUTTON2_LED_INDEX, cos_value);
        }

        if Self::should_animate(status.button3_pressed, self.button3_tick, current_tick) {
            let cos_value = Self::get_cos_value(self.button3_tick, current_tick);
            self.modify_led_value_rb(2, 0xff - cos_value, cos_value);
            self.modify_led_value(BUTTON3_LED_INDEX, cos_value);
        }

        if Self::should_animate(status.button4_pressed, self.button4_tick, current_tick) {
            let cos_value = Self::get_cos_value(self.button4_tick, current_tick);
            self.modify_led_value_rb(3, 0xff - cos_value, cos_value);
            self.modify_led_value(BUTTON4_LED_INDEX, cos_value);
        }

        if Self::should_animate(status.fx_l_pressed, self.fx_l_tick, current_tick) {
            let cos_value = Self::get_cos_value(self.fx_l_tick, current_tick);
            self.modify_led_value_rb(4, 0xff - cos_value, cos_value);
            self.modify_led_value(FX_L_LED_INDEX, cos_value);
        }

        if Self::should_animate(status.fx_r_pressed, self.fx_r_tick, current_tick) {
            let cos_value = Self::get_cos_value(self.fx_r_tick, current_tick);
            self.modify_led_value_rb(5, 0xff - cos_value, cos_value);
            self.modify_led_value(FX_R_LED_INDEX, cos_value);
        }

        if Self::should_animate(status.start_pressed, self.start_tick, current_tick) {
            let cos_value = Self::get_cos_value(self.start_tick, current_tick);
            self.modify_led_value_rb(0, 0xff - cos_value, cos_value);
            self.modify_led_value_rb(1, 0xff - cos_value, cos_value);
            self.modify_led_value_rb(2, 0xff - cos_value, cos_value);
            self.modify_led_value_rb(3, 0xff - cos_value, cos_value);
            self.modify_led_value_rb(4, 0xff - cos_value, cos_value);
            self.modify_led_value_rb(5, 0xff - cos_value, cos_value);
        }

        return self.new_led_brightness;
    }
}

impl SdvxFadeAnimation {
    fn should_animate(button_pressed: bool, button_tick: u64, current_tick: u64) -> bool {
        return button_pressed || button_tick + FADE_ANIMATION_TIME_TICKS > current_tick;
    }

    fn get_cos_value(button_tick: u64, current_tick: u64) -> u8 {
        let floating_index = (current_tick - button_tick) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
        let index = (floating_index * 255f32) as usize;
        return SDVX_COS_TABLE[index % 256];
    }

    fn clear_led_values(&mut self) {
        self.new_led_brightness = [0x00; BCM_LED_COUNT];
    }

    fn set_button_ticks(&mut self, status: &SdvxStatus, current_tick: u64) {
        if status.start_pressed {
            self.start_tick = current_tick;
        }

        if status.button1_pressed {
            self.button1_tick = current_tick;
        }

        if status.button2_pressed {
            self.button2_tick = current_tick;
        }

        if status.button3_pressed {
            self.button3_tick = current_tick;
        }

        if status.button4_pressed {
            self.button4_tick = current_tick;
        }

        if status.fx_l_pressed {
            self.fx_l_tick = current_tick;
        }

        if status.fx_r_pressed {
            self.fx_r_tick = current_tick;
        }

        if status.rotary1_rotated_ccw {
            self.rotary1_tick_ccw = current_tick;
            self.rotary1_tick_cw = 0;
        }

        if status.rotary1_rotated_cw {
            self.rotary1_tick_ccw = 0;
            self.rotary1_tick_cw = current_tick;
        }

        if status.rotary2_rotated_ccw {
            self.rotary2_tick_ccw = current_tick;
            self.rotary2_tick_cw = 0;
        }

        if status.rotary2_rotated_cw {
            self.rotary2_tick_ccw = 0;
            self.rotary2_tick_cw = current_tick;
        }
    }

    fn modify_led_value_rb(&mut self, index: usize, red_value: u8, blue_value: u8) {
        self.new_led_brightness[index * 3 + 0] = red_value;
        self.new_led_brightness[index * 3 + 1] = 0x00;
        self.new_led_brightness[index * 3 + 2] = blue_value;
    }

    fn modify_led_value(&mut self, index: usize, value: u8) {
        self.new_led_brightness[index] = max(self.new_led_brightness[index], value);
    }
}
