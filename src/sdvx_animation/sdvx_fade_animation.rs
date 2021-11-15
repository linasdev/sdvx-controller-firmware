use crate::sdvx_cos_table::SDVX_COS_TABLE;
use crate::sdvx_status::SdvxStatus;
use crate::sdvx_bcm::BCM_LED_COUNT;
use crate::sdvx_animation::SdvxAnimation;

// Amount of ticks for the animation to end after a button is released
const FADE_ANIMATION_TIME_TICKS: u32 = 20_000_000;
const BUTTON1_LED_INDEX: usize = 18;
const BUTTON2_LED_INDEX: usize = 19;
const BUTTON3_LED_INDEX: usize = 20;
const BUTTON4_LED_INDEX: usize = 21;
const FX_L_LED_INDEX: usize = 22;
const FX_R_LED_INDEX: usize = 23;

pub struct SdvxFadeAnimation {
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

impl SdvxAnimation for SdvxFadeAnimation {
    fn new() -> Self {
        SdvxFadeAnimation {
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

    fn tick(&mut self, status: &SdvxStatus, current_tick: u32) -> [u8; BCM_LED_COUNT] {
        let mut new_led_brightness = [0x00u8; BCM_LED_COUNT];

        if status.start_pressed || self.start_tick + FADE_ANIMATION_TIME_TICKS > current_tick {
            let floating_index = (current_tick - self.start_tick) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                0,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                1,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                2,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                3,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                4,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                5,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            if status.start_pressed {
                self.start_tick = current_tick;
            }
        }

        if status.button1_pressed || self.button1_tick + FADE_ANIMATION_TIME_TICKS > current_tick {
            let floating_index = (current_tick - self.button1_tick) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                0,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value(
                &mut new_led_brightness,
                BUTTON1_LED_INDEX,
                cos_value,
            );

            if status.button1_pressed {
                self.button1_tick = current_tick;
            }
        }

        if status.button2_pressed || self.button2_tick + FADE_ANIMATION_TIME_TICKS > current_tick {
            let floating_index = (current_tick - self.button2_tick) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                1,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value(
                &mut new_led_brightness,
                BUTTON2_LED_INDEX,
                cos_value,
            );

            if status.button2_pressed {
                self.button2_tick = current_tick;
            }
        }

        if status.button3_pressed || self.button3_tick + FADE_ANIMATION_TIME_TICKS > current_tick {
            let floating_index = (current_tick - self.button3_tick) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                2,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value(
                &mut new_led_brightness,
                BUTTON3_LED_INDEX,
                cos_value,
            );

            if status.button3_pressed {
                self.button3_tick = current_tick;
            }
        }

        if status.button4_pressed || self.button4_tick + FADE_ANIMATION_TIME_TICKS > current_tick {
            let floating_index = (current_tick - self.button4_tick) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                3,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value(
                &mut new_led_brightness,
                BUTTON4_LED_INDEX,
                cos_value,
            );

            if status.button4_pressed {
                self.button4_tick = current_tick;
            }
        }

        if status.fx_l_pressed || self.fx_l_tick + FADE_ANIMATION_TIME_TICKS > current_tick {
            let floating_index = (current_tick - self.fx_l_tick) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                4,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value(
                &mut new_led_brightness,
                FX_L_LED_INDEX,
                cos_value,
            );

            if status.fx_l_pressed {
                self.fx_l_tick = current_tick;
            }
        }

        if status.fx_r_pressed || self.fx_r_tick + FADE_ANIMATION_TIME_TICKS > current_tick {
            let floating_index = (current_tick - self.fx_r_tick) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                5,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value(
                &mut new_led_brightness,
                FX_R_LED_INDEX,
                cos_value,
            );

            if status.fx_r_pressed {
                self.fx_r_tick = current_tick;
            }
        }

        if status.rotary1_rotated_ccw || self.rotary1_tick_ccw + FADE_ANIMATION_TIME_TICKS > current_tick
        {
            let floating_index = (current_tick - self.rotary1_tick_ccw) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                0,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                4,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            if status.rotary1_rotated_ccw {
                self.rotary1_tick_ccw = current_tick;
                self.rotary1_tick_cw = 0;
            }
        }

        if status.rotary1_rotated_cw || self.rotary1_tick_cw + FADE_ANIMATION_TIME_TICKS > current_tick
        {
            let floating_index = (current_tick - self.rotary1_tick_cw) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                1,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            if status.rotary1_rotated_cw {
                self.rotary1_tick_ccw = 0;
                self.rotary1_tick_cw = current_tick;
            }
        }

        if status.rotary2_rotated_ccw || self.rotary2_tick_ccw + FADE_ANIMATION_TIME_TICKS > current_tick
        {
            let floating_index = (current_tick - self.rotary2_tick_ccw) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                2,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            if status.rotary2_rotated_ccw {
                self.rotary2_tick_ccw = current_tick;
                self.rotary2_tick_cw = 0;
            }
        }

        if status.rotary2_rotated_cw || self.rotary2_tick_cw + FADE_ANIMATION_TIME_TICKS > current_tick
        {
            let floating_index = (current_tick - self.rotary2_tick_cw) as f32 / FADE_ANIMATION_TIME_TICKS as f32;
            let index = (floating_index * 255f32) as usize;
            let cos_value = SDVX_COS_TABLE[index % 256];

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                3,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            SdvxFadeAnimation::modify_led_value_rgb(
                &mut new_led_brightness,
                5,
                0xff - cos_value,
                0x00,
                cos_value,
            );

            if status.rotary2_rotated_cw {
                self.rotary2_tick_ccw = 0;
                self.rotary2_tick_cw = current_tick;
            }
        }

        return new_led_brightness;
    }
}

impl SdvxFadeAnimation {
    fn modify_led_value_rgb(
        led_brightness: &mut [u8; BCM_LED_COUNT],
        index: usize,
        red_value: u8,
        green_value: u8,
        blue_value: u8,
    ) {
        (*led_brightness)[index * 3 + 0] = red_value;
        (*led_brightness)[index * 3 + 1] = green_value;
        (*led_brightness)[index * 3 + 2] = blue_value;
    }

    fn modify_led_value(
        led_brightness: &mut [u8; BCM_LED_COUNT],
        index: usize,
        value: u8,
    ) {
        (*led_brightness)[index] = value;
    }
}
