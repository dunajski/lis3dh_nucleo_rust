use stm32g0::stm32g071::{self, interrupt, Interrupt, NVIC};

const BUFF_SIZE: usize = 1024;
struct CircularBuff {
    buf: [u8; BUFF_SIZE],
    ri: usize,
    wi: usize,
}

impl CircularBuff {
    fn put_byte(&mut self, byte: u8) {
        self.buf[self.wi] = byte;

        if self.wi == BUFF_SIZE - 1 {
            self.wi = 0;
        } else {
            self.wi += 1;
        }
    }

    fn put_bytes(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.put_byte(*b);
        }
    }

    fn get_byte(&mut self) -> (u8, bool) {
        let mut byte_found = false;
        let mut byte = b'\0';
        if self.wi != self.ri {
            byte = self.buf[self.ri];
            byte_found = true;

            if self.ri == BUFF_SIZE - 1 {
                self.ri = 0;
            } else {
                self.ri += 1;
            }
        }
        (byte, byte_found)
    }

    fn get_bytes(&mut self) -> () {
        loop {
            let (byte, result) = self.get_byte();
            if result == false {
                break;
            } else {
                put_to_serial(&[byte]);
            }
        }
    }
}

static mut TX_CBUF: CircularBuff = CircularBuff {
    buf: [0; BUFF_SIZE],
    wi: 0,
    ri: 0,
};
static mut RX_CBUF: CircularBuff = CircularBuff {
    buf: [0; BUFF_SIZE],
    wi: 0,
    ri: 0,
};

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

pub fn put_to_serial(buff: &[u8]) {
    let usart2_r = unsafe { stm32g071::Peripherals::steal().USART2 };
    unsafe {
        TX_CBUF.put_bytes(buff);
        let (byte, _) = TX_CBUF.get_byte();

        usart2_r.tdr.write(|w| w.bits(byte as u32));

        usart2_r.cr1.modify(|_, w| w.tcie().set_bit());
    }
}

pub fn rx_buffer_read() {
    unsafe {
        RX_CBUF.get_bytes();
    }
}

#[interrupt]
fn USART2() {
    let usart2_r = unsafe { stm32g071::Peripherals::steal().USART2 };

    // receive buffer not empty isr
    if usart2_r.isr.read().rxne().bit_is_set() {
        let byte = usart2_r.rdr.read().bits() as u8;

        unsafe {
            RX_CBUF.put_byte(byte);
        }
    }

    // transmission complete isr
    if usart2_r.isr.read().tc().bit_is_set() {
        usart2_r.icr.write(|w| w.tccf().set_bit());

        unsafe {
            let (byte, result) = TX_CBUF.get_byte();
            if result == false {
                usart2_r.cr1.modify(|_, w| w.tcie().clear_bit());
            } else {
                usart2_r.tdr.write(|w| w.bits(byte as u32));
            }
        }
    }
}
