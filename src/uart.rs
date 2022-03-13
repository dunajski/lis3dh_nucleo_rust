use stm32g0::stm32g071::{self, interrupt, Interrupt, NVIC};

const BUFF_SIZE: usize = 1024;
struct CircularBuff {
    buf: [u8; BUFF_SIZE],
    ri: usize,
    wi: usize,
}

static mut TX_CBUF : CircularBuff = CircularBuff {buf: [0; BUFF_SIZE], wi:0, ri:0};
// static mut RX_CBUF : CircularBuff = CircularBuff {buf: [0; BUFF_SIZE], wi:0, ri:0};

pub fn init_uart() {
    let usart2_r = unsafe { stm32g071::Peripherals::steal().USART2 };
    let gpioa_r = unsafe { stm32g071::Peripherals::steal().GPIOA };

    let clock_r = unsafe { stm32g071::Peripherals::steal().RCC };
    clock_r.iopenr.modify(|_, w| w.iopaen().set_bit());
    clock_r.apbenr1.modify(|_, w| w.usart2en().set_bit());

    // Set RX/TX pins as Outputs
    gpioa_r.moder.modify(unsafe {
        |_, w| {
            w.moder2().bits(0b10);
            w.moder3().bits(0b10)
        }
    });

    // Set RX/TX pins as High-Speed Outputs
    gpioa_r.ospeedr.modify(unsafe {
        |_, w| {
            w.ospeedr3().bits(0b11);
            w.ospeedr2().bits(0b11)
        }
    });

    // RX/TX pins Pull-up
    gpioa_r.pupdr.modify(unsafe {
        |_, w| {
            w.pupdr2().bits(0b01);
            w.pupdr3().bits(0b01)
        }
    });

    // Set alternate fucntions as USART TX/RX
    gpioa_r.afrl.modify(unsafe {
        |_, w| {
            w.afsel2().bits(0b0001);
            w.afsel3().bits(0b0001)
        }
    });

    // set baud-rate to 115200
    let brr = 16_000_000 / 115200;
    usart2_r.brr.write(unsafe { |w| w.bits(brr) });

    unsafe { NVIC::unmask(Interrupt::USART2) }

    usart2_r.cr1.modify(|_, w| {
        w.ue().set_bit();
        w.re().set_bit();
        w.te().set_bit();
        w.rxneie().set_bit();
        w.tcie().set_bit()
    });
}

pub fn put_to_serial(buff: &[u8])
{
    let usart2_r = unsafe { stm32g071::Peripherals::steal().USART2 };
    unsafe {
        // put all bytes to cbuffer
        for b in buff {
            TX_CBUF.buf[TX_CBUF.wi] = *b;
            
            if TX_CBUF.wi == BUFF_SIZE - 1 {
                TX_CBUF.wi = 0;
            } else {
                TX_CBUF.wi += 1;
            }
        }
        usart2_r.tdr.write(|w| { w.bits(TX_CBUF.buf[TX_CBUF.ri] as u32) });
        TX_CBUF.ri += 1;
        usart2_r.cr1.modify(|_, w| {
            w.tcie().set_bit()
        });
    }
}

fn get_next_byte() -> (u8, bool) {
    let mut byte = b'\0';
    let mut result = false;

    unsafe {
        // check if read index is diffrent from write index
        // if true means that there is something to put to serial
        if TX_CBUF.ri != TX_CBUF.wi {
            byte = TX_CBUF.buf[TX_CBUF.ri];
            // if it is last index, move to the beginning of cbuf
            if TX_CBUF.ri == BUFF_SIZE - 1 {
                TX_CBUF.ri = 0;
            } else {
                TX_CBUF.ri += 1;
            }
            result = true;
        }
    }

    return (byte, result);
}

#[interrupt]
fn USART2() {
    let usart2_r = unsafe { stm32g071::Peripherals::steal().USART2 };

    if usart2_r.isr.read().rxne().bit_is_set() {
        let byte = usart2_r.rdr.read().bits() as u8;

        if byte == b'e' {
            put_to_serial(b"Received 'e' character\n");
        } else {
            put_to_serial(b"Received another character\n")
        }
    }

    if usart2_r.isr.read().tc().bit_is_set() {
        usart2_r.icr.write(|w| {
            w.tccf().set_bit()
        });

        let (byte, result) = get_next_byte();
            if result == false {
                usart2_r.cr1.modify(|_, w| {
                    w.tcie().clear_bit()
                });
            }
            else {
                usart2_r.tdr.write(|w| unsafe { w.bits(byte as u32) });
            }
    }
}
