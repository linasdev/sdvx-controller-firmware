use embedded_hal::digital::v2::InputPin;
use rotary_encoder_hal::{Direction, Rotary};
use stm32f1xx_hal::gpio::gpioa::*;
use stm32f1xx_hal::gpio::{Input, PullDown, PullUp};
use stm32f1xx_hal::pac::{Peripherals, NVIC};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;

use crate::sdvx_animation::SdvxAnimation;
use crate::sdvx_bcm::SdvxBcm;
use crate::sdvx_keycode::SdvxKeycode;
use crate::sdvx_status::SdvxStatus;
use crate::sdvx_sys_clock::SdvxSysClock;

// The higher the sensitivity, the less keypresses will be sent to the computer
static ROTARY_SENSITIVITY: i8 = 2;
// Maximum amount of keycodes per frame (default: 6)
static MAX_KEYCODE_COUNT: usize = 6;
// HID report to send when the amount of keycodes exceeds MAX_KEYCODE_COUNT
static ROLLOVER_ERROR_REPORT: KeyboardReport = KeyboardReport {
    modifier: 0x00,
    reserved: 0x00,
    leds: 0x00,
    keycodes: [
        SdvxKeycode::ErrorRollOver as u8,
        SdvxKeycode::ErrorRollOver as u8,
        SdvxKeycode::ErrorRollOver as u8,
        SdvxKeycode::ErrorRollOver as u8,
        SdvxKeycode::ErrorRollOver as u8,
        SdvxKeycode::ErrorRollOver as u8,
    ],
};

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_HID: Option<HIDClass<UsbBusType>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBusType>> = None;

pub struct SdvxController<A: SdvxAnimation> {
    start_input: PA0<Input<PullDown>>,
    button1_input: PA1<Input<PullDown>>,
    button2_input: PA2<Input<PullDown>>,
    button3_input: PA3<Input<PullDown>>,
    button4_input: PA4<Input<PullDown>>,
    fx_l_input: PA5<Input<PullDown>>,
    fx_r_input: PA6<Input<PullDown>>,
    rotary1: Rotary<PA7<Input<PullUp>>, PA8<Input<PullUp>>>,
    rotary2: Rotary<PA9<Input<PullUp>>, PA10<Input<PullUp>>>,
    rotary1_counter: i8,
    rotary2_counter: i8,
    usb_hid: &'static HIDClass<'static, UsbBusType>,
    status: SdvxStatus,
    bcm: SdvxBcm<A>,
    last_keycodes: [u8; 6],
    sys_clock: SdvxSysClock,
}

impl<A: SdvxAnimation> SdvxController<A> {
    pub fn new(animation: A, mut dp: Peripherals) -> Self {
        let sys_clock = SdvxSysClock::new(dp.TIM1, dp.TIM2, dp.TIM3, &mut dp.RCC);

        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .hclk(72.mhz())
            .pclk1(36.mhz())
            .pclk2(72.mhz())
            .freeze(&mut flash.acr);

        let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

        let start_input = gpioa.pa0.into_pull_down_input(&mut gpioa.crl);
        let button1_input = gpioa.pa1.into_pull_down_input(&mut gpioa.crl);
        let button2_input = gpioa.pa2.into_pull_down_input(&mut gpioa.crl);
        let button3_input = gpioa.pa3.into_pull_down_input(&mut gpioa.crl);
        let button4_input = gpioa.pa4.into_pull_down_input(&mut gpioa.crl);
        let fx_l_input = gpioa.pa5.into_pull_down_input(&mut gpioa.crl);
        let fx_r_input = gpioa.pa6.into_pull_down_input(&mut gpioa.crl);
        let rotary1_a_input = gpioa.pa7.into_pull_up_input(&mut gpioa.crl);
        let rotary1_b_input = gpioa.pa8.into_pull_up_input(&mut gpioa.crh);
        let rotary2_a_input = gpioa.pa9.into_pull_up_input(&mut gpioa.crh);
        let rotary2_b_input = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

        let rotary1 = Rotary::new(rotary1_a_input, rotary1_b_input);
        let rotary2 = Rotary::new(rotary2_a_input, rotary2_b_input);

        let usb = Peripheral {
            usb: dp.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };

        unsafe { USB_BUS = Some(UsbBus::new(usb)) }

        let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };
        let usb_hid = HIDClass::new(usb_bus, KeyboardReport::desc(), 1);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x69af, 0x420b))
            .device_class(0x03) // HID (Human Interface Device)
            .device_sub_class(0x00) // No subclass (can't be used as a Boot Device)
            .device_protocol(0x00) // No protocol (can't be used as a Boot Device)
            .device_release(0x0100) // 1.00
            .self_powered(false)
            .manufacturer("Linas Nikiperavičius <linas@linasdev.com>")
            .product("SDVX Controller")
            .max_power(500) // 500mA
            .build();

        unsafe {
            USB_HID = Some(usb_hid);
            USB_DEV = Some(usb_dev);
        }

        let usb_hid = unsafe { USB_HID.as_ref().unwrap() };

        let bcm = SdvxBcm::new(
            animation,
            gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
            gpiob.pb14.into_push_pull_output(&mut gpiob.crh),
            gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
            dp.SPI2,
            clocks,
            &mut rcc.apb1,
            dp.TIM4,
        );

        unsafe {
            // For usb polling
            NVIC::unmask(Interrupt::USB_HP_CAN_TX);
            NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        }

        SdvxController {
            start_input,
            button1_input,
            button2_input,
            button3_input,
            button4_input,
            fx_l_input,
            fx_r_input,
            rotary1,
            rotary2,
            rotary1_counter: 0,
            rotary2_counter: 0,
            usb_hid,
            status: SdvxStatus::new(),
            bcm,
            last_keycodes: [SdvxKeycode::No as u8; 6],
            sys_clock,
        }
    }

    pub fn tick(&mut self) {
        self.update_status();
        self.bcm
            .tick(&self.status, self.sys_clock.get_current_tick());

        let mut keycodes = [SdvxKeycode::No as u8; 6];
        let mut current_keycode = 0;

        if self.status.start_pressed {
            keycodes[current_keycode] = SdvxKeycode::Kb1 as u8;
            current_keycode += 1;
        }

        if self.status.button1_pressed {
            keycodes[current_keycode] = SdvxKeycode::D as u8;
            current_keycode += 1;
        }

        if self.status.button2_pressed {
            keycodes[current_keycode] = SdvxKeycode::F as u8;
            current_keycode += 1;
        }

        if self.status.button3_pressed {
            keycodes[current_keycode] = SdvxKeycode::J as u8;
            current_keycode += 1;
        }

        if self.status.button4_pressed {
            keycodes[current_keycode] = SdvxKeycode::K as u8;
            current_keycode += 1;
        }

        if self.status.fx_l_pressed {
            keycodes[current_keycode] = SdvxKeycode::C as u8;
            current_keycode += 1;
        }

        if self.status.fx_r_pressed {
            if self.check_and_push_rollover(current_keycode + 1) {
                return;
            }

            keycodes[current_keycode] = SdvxKeycode::M as u8;
            current_keycode += 1;
        }

        if self.status.rotary1_rotated_ccw {
            if self.check_and_push_rollover(current_keycode + 1) {
                return;
            }

            keycodes[current_keycode] = SdvxKeycode::W as u8;
            current_keycode += 1;
        } else if self.status.rotary1_rotated_cw {
            if self.check_and_push_rollover(current_keycode + 1) {
                return;
            }

            keycodes[current_keycode] = SdvxKeycode::E as u8;
            current_keycode += 1;
        }

        if self.status.rotary2_rotated_ccw {
            if self.check_and_push_rollover(current_keycode + 1) {
                return;
            }

            keycodes[current_keycode] = SdvxKeycode::O as u8;
        } else if self.status.rotary2_rotated_cw {
            if self.check_and_push_rollover(current_keycode + 1) {
                return;
            }

            keycodes[current_keycode] = SdvxKeycode::P as u8;
        }

        if keycodes == self.last_keycodes {
            return;
        }

        let report = KeyboardReport {
            modifier: 0x00,
            reserved: 0x00,
            leds: 0x00,
            keycodes,
        };

        // Update the last_keycodes variable only if the hid report was sent successfully
        match self.usb_hid.push_input(&report) {
            Ok(_) => self.last_keycodes = keycodes,
            Err(_) => {}
        }
    }

    fn update_status(&mut self) {
        self.status.start_pressed = self.start_input.is_high().unwrap();
        self.status.button1_pressed = self.button1_input.is_high().unwrap();
        self.status.button2_pressed = self.button2_input.is_high().unwrap();
        self.status.button3_pressed = self.button3_input.is_high().unwrap();
        self.status.button4_pressed = self.button4_input.is_high().unwrap();
        self.status.fx_l_pressed = self.fx_l_input.is_high().unwrap();
        self.status.fx_r_pressed = self.fx_r_input.is_high().unwrap();

        self.rotary1_counter += match self.rotary1.update().unwrap() {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
            Direction::None => 0,
        };

        self.rotary2_counter += match self.rotary2.update().unwrap() {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
            Direction::None => 0,
        };

        self.status.rotary1_rotated_ccw = self.rotary1_counter <= -ROTARY_SENSITIVITY;
        self.status.rotary1_rotated_cw = self.rotary1_counter >= ROTARY_SENSITIVITY;
        self.status.rotary2_rotated_ccw = self.rotary2_counter <= -ROTARY_SENSITIVITY;
        self.status.rotary2_rotated_cw = self.rotary2_counter >= ROTARY_SENSITIVITY;

        if self.status.rotary1_rotated_ccw || self.status.rotary1_rotated_cw {
            self.rotary1_counter = 0;
        }

        if self.status.rotary2_rotated_ccw || self.status.rotary2_rotated_cw {
            self.rotary2_counter = 0;
        }
    }

    fn check_and_push_rollover(&self, keycode_count: usize) -> bool {
        if keycode_count > MAX_KEYCODE_COUNT {
            // Ignore errors as the transmission will likely be attempted again
            // in the next loop iteration
            let _ = self.usb_hid.push_input(&ROLLOVER_ERROR_REPORT);
            return true;
        }

        return false;
    }
}

fn poll_usb() {
    let usb_hid = unsafe { USB_HID.as_mut().unwrap() };
    let usb_dev = unsafe { USB_DEV.as_mut().unwrap() };

    if usb_dev.poll(&mut [usb_hid]) {
        // Read and discard incoming USB packets, so the interrupt flag gets cleared and
        // the mcu doesn't get stuck.
        let mut data = [0u8, 64];
        let _ = usb_hid.pull_raw_output(&mut data);
    }
}

#[interrupt]
fn USB_HP_CAN_TX() {
    poll_usb();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    poll_usb();
}
