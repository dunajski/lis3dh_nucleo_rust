#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;

use stm32g0::stm32g071::{self, interrupt, Interrupt, NVIC};

use core::cell::Cell;
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::interrupt::free as critical_section; // I've to change it because stm32g0 got `interrupt` too.
use cortex_m::interrupt::Mutex;

static G_TIM3: Mutex<RefCell<Option<stm32g071::TIM3>>> = Mutex::new(RefCell::new(None));
static G_GPIOA: Mutex<RefCell<Option<stm32g071::GPIOA>>> = Mutex::new(RefCell::new(None));
static G_GPIOC: Mutex<RefCell<Option<stm32g071::GPIOC>>> = Mutex::new(RefCell::new(None));
static G_BLINK_RATE: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

enum ButtonState {
    Unpressed,
    Debouncing,
    Pressed,
}

#[entry]
fn main() -> ! {
    let p = stm32g071::Peripherals::take().unwrap();

    let gpioa = &p.GPIOA;
    let gpioc = &p.GPIOC;
    let clock_r = &p.RCC;
    let tim3_r = p.TIM3;

    // enable GPIOA and GPIOC clocks
    clock_r.iopenr.modify(|_, w| {
        w.iopaen().set_bit();
        w.iopcen().set_bit()
    });

    // enable clock for TIM3
    clock_r.apbenr1.write(|w| {
        w.tim3en().set_bit()
    });

    // opening critical section for TIM3 initialization
    critical_section(|cs| {
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
        // replace value of G_TIM3 with register
        // and from now I will use G_TIM3 to get to register
        G_TIM3.borrow(cs).replace(Some(tim3_r));
    });

    unsafe {
        NVIC::unmask(Interrupt::TIM3);
    };

    // Nucleo G071RB has LED on PA5
    gpioa.moder.modify(|_, w| unsafe { w.moder5().bits(0b01) });

    // ... and Blue Button on PC13, which is hardware pulled to VDD
    // moder reset value == 0, anyway sets as input
    gpioc.moder.write(|w| unsafe { w.moder13().bits(0b00) });

    critical_section(|cs| G_BLINK_RATE.borrow(cs).set(0));

    let mut delay_cnt = 0u32;
    let gpioa = p.GPIOA;
    let gpioc = p.GPIOC;

    critical_section(|cs| {
        G_GPIOA.borrow(cs).replace(Some(gpioa));
        G_GPIOC.borrow(cs).replace(Some(gpioc));
    });

    loop {
        critical_section(|cs| {
            if delay_cnt < G_BLINK_RATE.borrow(cs).get() {
                delay_cnt += 1;
            } else {
                // change LED state
                if let Some(ref mut gpioa) = G_GPIOA.borrow(cs).borrow_mut().deref_mut() {
                    if gpioa.odr.read().odr5().bit_is_set() {
                        gpioa.odr.modify(|_, w| w.odr5().clear_bit());
                    } else {
                        gpioa.odr.modify(|_, w| w.odr5().set_bit());
                    }
                }
                delay_cnt = 0;
            }
        });
    }
}

#[interrupt]
fn TIM3() {
    static mut KEYLEV: ButtonState = ButtonState::Unpressed;
    static mut KEYCNT: u16 = 0;
    const DEBOUNCING_TIME: u16 = 20; // 20 ms
    let mut press_state: bool = false;

    critical_section(|cs| {
        if let Some(ref mut gpioc) = G_GPIOC.borrow(cs).borrow_mut().deref_mut() {
            if gpioc.idr.read().idr13().bit_is_clear() {
                press_state = true;
            } else {
                press_state = false;
            }
        }
        // clear ISR invoking bit
        if let Some(ref mut tim3) = G_TIM3.borrow(cs).borrow_mut().deref_mut() {
            tim3.sr.write(|w| w.uif().clear_bit());
        }
    });

    if *KEYCNT == 0 {
        match KEYLEV {
            ButtonState::Unpressed => {
                if press_state {
                    *KEYLEV = ButtonState::Debouncing;
                    *KEYCNT = DEBOUNCING_TIME;
                }
            }
            ButtonState::Debouncing => {
                if press_state {
                    change_blinking_ratio();
                }
                *KEYLEV = ButtonState::Pressed;
            }
            ButtonState::Pressed => {
                if !press_state {
                    *KEYLEV = ButtonState::Unpressed;
                }
            }
        }
    }

    if *KEYCNT > 0 {
        *KEYCNT -= 1;
    }
}

fn change_blinking_ratio() {
    let mut blinking_rate = 0u32;

    critical_section(|cs| {
        blinking_rate = G_BLINK_RATE.borrow(cs).get();

        if blinking_rate == 0xFF_FF {
            blinking_rate = 0x0F_FF;
        } else {
            blinking_rate = 0xFF_FF;
        }

        G_BLINK_RATE.borrow(cs).set(blinking_rate);
    });
}