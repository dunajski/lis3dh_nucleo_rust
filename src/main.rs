#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use stm32g0::stm32g071::{self, interrupt, Interrupt, NVIC};

mod uart;

static mut G_BLINK_RATE: u32 = 0xFF_FF;

enum ButtonState {
    Unpressed,
    Debouncing,
    Pressed,
}

#[entry]
fn main() -> ! {
    let p = stm32g071::Peripherals::take().unwrap();

    let clock_r = &p.RCC;
    // enable GPIOA and GPIOC clocks
    clock_r.iopenr.write(|w| {
        w.iopaen().set_bit();
        w.iopcen().set_bit()
    });

    // enable clock for TIM3
    clock_r.apbenr1.write(|w| w.tim3en().set_bit());

    let tim3_r = p.TIM3;
    prepare_tim3(&tim3_r);

    unsafe {
        NVIC::unmask(Interrupt::TIM3);
    }

    // Nucleo G071RB has LED on PA5
    let gpioa_r = &p.GPIOA;
    gpioa_r
        .moder
        .modify(unsafe { |_, w| w.moder5().bits(0b01) });

    // ... and Blue Button on PC13, which is hardware pulled to VDD
    // moder reset value == 0, even that set as input
    let gpioc_r = &p.GPIOC;
    gpioc_r.moder.write(unsafe { |w| w.moder13().bits(0b00) });

    uart::init_uart();


    uart::put_to_serial(b"Application started.\n");

    loop {
        handle_blinking();
    }
}

fn handle_blinking() {
    static mut DELAY_CNT: u32 = 0;
    let gpioa = unsafe { stm32g071::Peripherals::steal().GPIOA };

    unsafe {
        if DELAY_CNT < G_BLINK_RATE {
            DELAY_CNT += 1;
        } else {
            if gpioa.odr.read().odr5().bit_is_set() {
                gpioa.odr.modify(|_, w| w.odr5().clear_bit());
            } else {
                gpioa.odr.modify(|_, w| w.odr5().set_bit());
            }
            DELAY_CNT = 0;
        }
    }
}

#[interrupt]
fn TIM3() {
    static mut KEY_LEVEL: ButtonState = ButtonState::Unpressed;
    static mut KEY_CNT: u16 = 0;
    const DEBOUNCING_TIME: u16 = 20; // 20 ms
    unsafe {
        let press_state: bool;

        let gpioc = stm32g071::Peripherals::steal().GPIOC;
        if gpioc.idr.read().idr13().bit_is_clear() {
            press_state = true;
        } else {
            press_state = false;
        }

        // clear ISR invoking bit
        let tim3 = stm32g071::Peripherals::steal().TIM3;
        tim3.sr.modify(|_, w| w.uif().clear_bit());

        if *KEY_CNT == 0 {
            match KEY_LEVEL {
                ButtonState::Unpressed => {
                    if press_state {
                        *KEY_LEVEL = ButtonState::Debouncing;
                        *KEY_CNT = DEBOUNCING_TIME;
                    }
                }
                ButtonState::Debouncing => {
                    if press_state {
                        change_blinking_ratio();
                        let msg = b"Blue button was pressed\n";
                        uart::put_to_serial(msg);
                    }
                    *KEY_LEVEL = ButtonState::Pressed;
                }
                ButtonState::Pressed => {
                    if !press_state {
                        *KEY_LEVEL = ButtonState::Unpressed;
                    }
                }
            }
        }

        if *KEY_CNT > 0 {
            *KEY_CNT -= 1;
        }
    }
}

fn change_blinking_ratio() {
    unsafe {
        if G_BLINK_RATE == 0x4_FF_FF {
            G_BLINK_RATE = 0xFF_FF;
        } else {
            G_BLINK_RATE = 0x4_FF_FF;
        }
    }
}

fn prepare_tim3(tim3_r: &stm32g071::TIM3) {
    // disable timer at first
    tim3_r.cr1.write(|w| w.cen().clear_bit());
    // default clock is HSI 16 MHz
    // 16 MHz / 1600 => 10 000 Hz => 10 kHz (prescaler 1600)
    tim3_r.psc.write(|w| unsafe { w.psc().bits(1600) });
    // 10 kHz / 10 => 1 kHz, Interrupt exectued every 1 ms (auto reload)
    tim3_r.arr.write(|w| unsafe { w.arr_l().bits(10) });
    // update interrupt settings
    tim3_r.egr.write(|w| w.ug().set_bit());
    // interruput enable
    tim3_r.dier.write(|w| w.uie().set_bit());
    // enable timer
    tim3_r.cr1.write(|w| w.cen().set_bit());
}
