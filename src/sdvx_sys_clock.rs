use stm32f1xx_hal::pac::{TIM1, TIM2, TIM3, RCC};

const SYS_CLOCK_PRESCALER: u32 = 0x0000;

pub struct SdvxSysClock {
    tim1: TIM1,
    tim2: TIM2,
    tim3: TIM3,
}

impl SdvxSysClock {
    pub fn new(
        tim1: TIM1,
        tim2: TIM2,
        tim3: TIM3,
        rcc: &mut RCC,
    ) -> Self {
        unsafe {
            // Disable timers
            tim1.cr1.modify(|_r, w| w.cen().disabled());
            tim2.cr1.modify(|_r, w| w.cen().disabled());
            tim3.cr1.modify(|_r, w| w.cen().disabled());

            // TIM1 setup
            rcc.apb2enr.modify(|_r, w| w.tim1en().enabled());       // Enable timer peripheral
            tim1.psc.modify(|_r, w| w.bits(SYS_CLOCK_PRESCALER));   // Set prescaler
            tim1.cr1.modify(|_r, w| w.ckd().div1());                // Set clock division
            tim1.cr2.modify(|_r, w| w.mms().update());              // Use reset signal as trigger output

            // TIM2 setup
            rcc.apb1enr.modify(|_r, w| w.tim2en().enabled());       // Enable timer peripheral
            tim2.psc.modify(|_r, w| w.bits(0x0000));                // Set prescaler
            tim2.cr1.modify(|_r, w| w.ckd().div1());                // Set clock division
            tim2.cr2.modify(|_r, w| w.mms().update());              // Use reset signal as trigger output
            tim2.smcr.modify(|_r, w| w.ts().itr0());                // Select internal trigger 0 (ITR0)
            tim2.smcr.modify(|_r, w| w.sms().ext_clock_mode());     // Clock timer from trigger

            // TIM3 setup
            rcc.apb1enr.modify(|_r, w| w.tim3en().enabled());       // Enable timer peripheral
            tim3.psc.modify(|_r, w| w.bits(0x0000));                // Set prescaler
            tim3.cr1.modify(|_r, w| w.ckd().div1());                // Set clock division
            tim3.smcr.modify(|_r, w| w.ts().itr1());                // Select internal trigger 1 (ITR1)
            tim3.smcr.modify(|_r, w| w.sms().ext_clock_mode());     // Clock timer from trigger

            // Enable timers
            tim1.cr1.modify(|_r, w| w.cen().enabled());
            tim2.cr1.modify(|_r, w| w.cen().enabled());
            tim3.cr1.modify(|_r, w| w.cen().enabled());
        }

        Self {
            tim1,
            tim2,
            tim3,
        }
    }

    pub fn get_current_tick(&self) -> u64 {
        return (self.tim3.cnt.read().bits() as u64) << 32 |
               (self.tim2.cnt.read().bits() as u64) << 16 |
               (self.tim1.cnt.read().bits() as u64) << 0;
    }
}
