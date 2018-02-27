use core::cell::RefCell;

use arraydeque::ArrayDeque;
use bare_metal::Mutex;
use bit_field::BitField;
use cortex_m::interrupt;
use cortex_m::peripheral::NVIC;
use volatile_register::RW;

use interrupts::Interrupt;
use port::{I2cScl, I2cSda};
use sim::ClockGate;

#[repr(C,packed)]
struct I2cRegs {
    a1:     RW<u8>,
    f:      RW<u8>,
    c1:     RW<u8>,
    s:      RW<u8>,
    d:      RW<u8>,
    c2:     RW<u8>,
    flt:    RW<u8>,
    ra:     RW<u8>,
    smb:    RW<u8>,
    a2:     RW<u8>,
    slth:   RW<u8>,
    sltl:   RW<u8>,
}

pub struct I2cMaster<'a, 'b> {
    reg: &'static mut I2cRegs,
    _scl: I2cScl<'a>,
    _sda: I2cSda<'b>,
    _gate: ClockGate,
    bus: u8,
    op_mode: OpMode,
}

#[cfg(feature = "i2c-slave")]
pub struct I2cSlave<'a, 'b> {
    reg: &'static mut I2cRegs,
    _scl: I2cScl<'a>,
    _sda: I2cSda<'b>,
    _gate: ClockGate,
    bus: u8,
}

pub enum Address {
    Bits7(u8),
    Bits10(u16)
}

impl Address {
    /// Only uses lower 7 bits of value.
    pub fn new_7bit(addr: u8) -> Address {
        Address::Bits7(addr & 0x7F)
    }

    /// Only uses lower 10 bits of value.
    pub fn new_10bit(addr: u16) -> Address {
        Address::Bits10(addr & 0x03FF)
    }
}

#[derive(PartialEq)]
pub enum OpMode {
    /// Immediate mode - All transmits and receives are blocking. Doesn't use ISR.
    IMM,
    #[cfg(feature = "i2c-isr")]
    /// Interrupt mode - Transmits and receives are non-blocking. Uses ISR.
    ISR,
    #[cfg(feature = "i2c-dma")]
    /// DMA mode - Like interrupt mode, but it uses DMA to do the magic
    DMA
}

#[derive(Debug)]
pub enum Error {
    Timeout,
    Nak,
    //AddrNak,
    //DataNak,
    ArbLost,
    BufOverflow,
    Fatal,
}

// TODO: write a drop impl
// TODO: impl Read
// TODO: impl Write
// TODO: support DMA mode
// TODO: support slave address range
// TODO: way to set slave callbacks
// TODO: support 10 bit read
// TODO: maybe rework to use a builder
// TODO: support higher bus numbers for other chips
impl<'a, 'b> I2cMaster<'a, 'b> {
    pub unsafe fn new(scl: I2cScl<'a>,
                      sda: I2cSda<'b>,
                      nvic: &mut NVIC,
                      (mul, clkdiv): (u8, u8),
                      op_mode: OpMode,
                      gate: ClockGate)
                      -> Result<I2cMaster<'a, 'b>, ()> {
        if mul >= 3 {
            return Err(())
        }
        if clkdiv >= 64 {
            return Err(())
        }

        let bus = sda.bus();

        let reg = match bus {
            0 => &mut *(0x40066000 as *mut I2cRegs),
            1 => &mut *(0x40067000 as *mut I2cRegs),
            _ => unimplemented!()
        };

        // init ram vars for tx (IICEN and IICIE)
        // init ram vars
        interrupt::free(|cs| {
            match bus {
                0 => *I2C0_STATE.borrow(cs).borrow_mut() = Some(IsrState::new()),
                1 => *I2C1_STATE.borrow(cs).borrow_mut() = Some(IsrState::new()),
                _ => unreachable!(),
            };
        });

        // f - clkdiv to set baud
        reg.f.write((mul << 6) | (clkdiv & 0x3F));

        // TODO: pass this stuff in
        let bus_freq: u32 = 36_000_000;
        if bus_freq >= 32 * 12_000_000 {
            reg.flt.write(4);
        } else {
            reg.flt.write((bus_freq/12_000_000) as u8);
        }

        reg.a1.write(0);
        reg.ra.write(0);

        // Official Teensy code does this, but doesn't state why
        // TODO: test
        reg.c2.modify(|mut c2| {
            c2.set_bit(5, true); // high drive select
            c2
        });

        if OpMode::IMM != op_mode {
            let interrupt = match bus {
                0 => Interrupt::I2C0,
                1 => Interrupt::I2C0,
                _ => unreachable!(),
            };
            interrupt::free(|_| {
                nvic.enable(interrupt);
            });
        }

        reg.c1.write(0x80); // enable module

        Ok(I2cMaster { reg: reg, _scl: scl, _sda: sda, _gate: gate, bus: bus, op_mode: op_mode })
    }

    pub fn wait_for_interrupt(&mut self) {
        while !self.reg.s.read().get_bit(1) {}
        unsafe { self.reg.s.write(0x02); }
    }

    // TODO: add timeout
    fn acquire_bus(&mut self) -> Result<bool, Error> {
        if self.reg.c1.read().get_bit(5) {
            // already master, send repeated start
            unsafe {
                self.reg.c1.modify(|mut c1| {
                    c1.set_bit(2, true); // repeated start
                    c1.set_bit(4, true); // tx
                    c1
                });
            }
        } else {
            while self.reg.s.read().get_bit(5) {} // wait for bus to not be busy
            unsafe {
                self.reg.c1.modify(|mut c1| {
                    c1.set_bit(5, true); // master mode, send start
                    c1.set_bit(4, true); // tx
                    c1
                });
            }

            // TODO: auto-retry behind a feature?

            // check if master
            // only really will matter when timeouts are added
            if !self.reg.c1.read().get_bit(5) {
                // was timeout
                return Err(Error::Timeout)
            }
        }

        // following only matters for ISR and DMA operation
        // escalates IRQ priority to overcome current execution priority
        /*
        #[cfg(feature = "i2c-isr")]
        if self.op_mode != OpMode::Imm {
            let currPriority = //nvic_execution_priority();
            let irqPriority = match self.bus {
                0 => NVIC_GET_PRIORITY(IRQ_I2C0),
                1 => NVIC_GET_PRIORITY(IRQ_I2C1),
                _ => unreachable!(),
            };

            if currPriority <= irqPriority {
                if currPriority < 16 {
                    // current priority cannot be surpassed, force Immediate mode
                    return Ok(true);
                } else {
                    match self.bus {
                        0 => NVIC_SET_PRIORITY(IRQ_I2C0, currPriority - 16),
                        1 => NVIC_SET_PRIORITY(IRQ_I2C1, currPriority - 16),
                        _ => unreachable!(),
                    }
                }
            }
        }
        */
        Ok(false)
    }

    /// Setup Master
    ///
    /// Initializes TX buffer with slave address.
    pub fn begin_transmission<'c>(&'c mut self, addr: Address) -> Transmission<'a, 'b, 'c> {
        interrupt::free(|cs| {
            let mut state = match self.bus {
                0 => I2C0_STATE.borrow(cs).borrow_mut(),
                1 => I2C1_STATE.borrow(cs).borrow_mut(),
                _ => unreachable!(),
            };
            // TODO: format! does not exist,find another way
            let state = state.as_mut().expect("I2CX_STATE uninitialized");

            state.tx_buf.clear();
            match addr {
                Address::Bits7(addr) => {
                    if state.tx_buf.push_back(addr << 1).is_some() {
                        panic!("Failed to push address to TX buffer");
                    }
                },
                Address::Bits10(addr) => {
                    let upper = *0.set_bits(1..3, (addr >> 8) as u8).set_bits(3..8, 0b11110);
                    let lower = addr as u8;
                    if state.tx_buf.push_back(upper).is_some() {
                        panic!("Failed to push upper byte of address to TX buffer");
                    }
                    if state.tx_buf.push_back(lower).is_some() {
                        panic!("Failed to push lower byte of address to TX buffer");
                    }
                }
            }
            state.status = Status::Idle;
        });

        Transmission { i2c: self }
    }

    // TODO: look into returning a `Future`
    // TODO: add timeout
    /// Send Master Transmit
    ///
    /// Starts transmit of Tx buffer to slave.
    ///
    /// Routine is non-blocking unless I2C is setup in Immediate mode.
    ///
    /// stop parameter can be used to indicate if command should end with a
    /// STOP (STOP) or not (NOSTOP).
    ///
    /// Use done() (async) or finish() (sync) to
    /// determine completion and status() to determine success/fail.
    fn send_transmission(&mut self, stop: Stop) -> Result<(), Error> {
        interrupt::free(|cs| {
            let mut state = match self.bus {
                0 => I2C0_STATE.borrow(cs).borrow_mut(),
                1 => I2C1_STATE.borrow(cs).borrow_mut(),
                _ => unreachable!(),
            };
            // TODO: format! does not exist,find another way
            let state = state.as_mut().expect("I2CX_STATE uninitialized");

            if state.tx_buf.is_empty() {
                return Ok(());
            }

            // clear ARBL and IICIF
            unsafe { self.reg.s.write(0x12); }

            let force_imm = self.acquire_bus()?;

            state.status = Status::Transmit;
            state.stop = stop;

            if force_imm || self.op_mode == OpMode::IMM {
                while let Some(d) = state.tx_buf.pop_front() {
                    // write address/data to bus
                    unsafe { self.reg.d.write(d); }
                    self.wait_for_interrupt();
                    let status = self.reg.s.read();
                    
                    // TODO: find some way to test this
                    // This is not called out in the Master mode branch in the
                    // in the flowchart in the TRM, but the Teensy folks do it
                    // so I'm putting it in
                    if status.get_bit(4) { // check arbitration
                        state.status = Status::ArbLost;
                        unsafe { self.reg.s.write(0x10); } // clear ARBL
                        // TODO: this is clearly not right. After ARBL it should
                        // drop into IMM slave mode if IAAS=1. Right now Rx
                        // message would be ignored regardless of IAAS

                        // change to Rx mode, intr disabled
                        // (does this send STOP if ARBL flagged?)
                        unsafe { self.reg.c1.write(0x80); } // Only module enabled
                        return Err(Error::ArbLost);
                    } else if status.get_bit(0) { // check if slave acked
                        // TODO: separate out data and addr so we can give different errors
                        state.status = Status::Nak;
                        unsafe { self.reg.c1.write(0x80); } // Only module enabled
                        return Err(Error::Nak);
                    }
                }

                state.status = Status::Idle;

                match state.stop {
                    Stop::Yes => unsafe { self.reg.c1.write(0x80) }, // Only module enabled
                    Stop::No => {}, //unsafe { self.reg.c1.write(0xB0) }, // Effectively a nop
                }

                Ok(())
            } else {
                #[cfg(feature = "i2c-dma")]
                { // All this because attributes are not allowed on if statements yet
                    // limit transfers less than 5 bytes to ISR method
                    if self.op_mode == OpMode::DMA && state.tx_buf.len() >= 5 {
                        // init DMA, let the hack begin
                        /*
                        i2c->activeDMA = I2C_DMA_ADDR;
                        i2c->DMA->sourceBuffer(&i2c->txBuffer[2],i2c->txBufferLength-3); // DMA sends all except first/second/last bytes
                        i2c->DMA->destination(*(i2c->D));
                        */
                    }
                }

                // enable interrupts and tx
                unsafe { self.reg.c1.write(0xF0); }
                // writing first data byte will start ISR
                unsafe { self.reg.d.write(state.tx_buf.pop_front().expect("Failed to pop first TX byte")); }

                Ok(())
            }
        })
    }

    // TODO: look into returning a `Future`
    // TODO: add timeout
    // TODO: support 10 bit address
    /// Send Master Receive
    ///
    /// Starts transmit of Tx buffer to slave.
    ///
    /// Routine is non-blocking unless I2C is setup in Immediate mode.
    ///
    /// i2c_stop parameter can be used to indicate if command should end with a
    /// STOP (STOP) or not (NOSTOP).
    ///
    /// Use done() determine completion or finish() to wait for completion.
    fn send_request(&mut self, len: usize, stop: Stop) -> Result<(), Error> {
        interrupt::free(|cs| {
            let mut state = match self.bus {
                0 => I2C0_STATE.borrow(cs).borrow_mut(),
                1 => I2C1_STATE.borrow(cs).borrow_mut(),
                _ => unreachable!(),
            };
            // TODO: format! does not exist,find another way
            let state = state.as_mut().expect("I2CX_STATE uninitialized");

            if len == 0 {
                return Ok(());
            }
            if len > state.rx_buf.capacity() {
                state.status = Status::BufOverflow;
                return Err(Error::BufOverflow);
            }

            state.rx_req = len;
            state.rx_buf.clear();

            // clear ARBL and IICIF
            unsafe { self.reg.s.write(0x12); }

            let force_imm = self.acquire_bus()?;

            state.status = Status::Address;
            state.stop = stop;
            state.timeout_rx_nak = false;

            if force_imm || self.op_mode == OpMode::IMM {
                // write address to bus
                unsafe { self.reg.d.write(state.tx_buf.pop_front().expect("Failed to pop address byte") | 0x01); }
                self.wait_for_interrupt();
                let status = self.reg.s.read();
                
                // TODO: find some way to test this
                // This is not called out in the Master mode branch in the
                // in the flowchart in the TRM, but the Teensy folks do it
                // so I'm putting it in
                if status.get_bit(4) { // check arbitration
                    state.status = Status::ArbLost;
                    unsafe { self.reg.s.write(0x10); } // clear ARBL
                    // TODO: this is clearly not right. After ARBL it should
                    // drop into IMM slave mode if IAAS=1. Right now Rx
                    // message would be ignored regardless of IAAS

                    // change to Rx mode, intr disabled
                    // (does this send STOP if ARBL flagged?)
                    unsafe { self.reg.c1.write(0x80); } // Only module enabled
                    return Err(Error::ArbLost);
                } else if status.get_bit(0) { // check if slave acked
                    // TODO: separate out data and addr so we can give different errors
                    state.status = Status::Nak;
                    unsafe { self.reg.c1.write(0x80); } // Only module enabled
                    return Err(Error::Nak);
                } else {
                    state.status = Status::Receive;
                    // switch to rx mode
                    unsafe {
                        self.reg.c1.modify(|mut c1| {
                            c1.set_bit(4, false);
                            if state.rx_req <= 1 {
                                c1.set_bit(3, true);
                            }
                            c1
                        });
                    }
                    self.reg.d.read(); // dummy read

                    while state.rx_req > state.rx_buf.len() && state.status == Status::Receive {
                        self.wait_for_interrupt();
                        /*
                        if(timeout != 0 && deltaT >= timeout) {
                            state.status = Status::Timeout;
                        }
                        */

                        if state.rx_buf.len() + 1 == state.rx_req ||
                           (state.status == Status::Timeout && state.timeout_rx_nak) {
                            state.timeout_rx_nak = false;

                            if state.status != Status::Timeout {
                                state.status = Status::Idle;
                            }
                            unsafe {
                                // change to tx
                                self.reg.c1.modify(|mut c1| {
                                    c1.set_bit(4, true);
                                    c1
                                });
                            }

                            match state.stop {
                                Stop::Yes => {
                                    //delayMicroseconds(1); // empirical patch, lets things settle before issuing STOP
                                    unsafe { self.reg.c1.write(0x80); } // only module enabled
                                }
                                Stop::No => unsafe { self.reg.c1.write(0xB0) }, // no stop, tx, intr disabled
                            }
                        } else if state.rx_buf.len() + 2 == state.rx_req ||
                                  (state.status == Status::Timeout && !state.timeout_rx_nak) {
                            if state.status == Status::Timeout && !state.timeout_rx_nak {
                                state.timeout_rx_nak = true;
                            }

                            unsafe {
                                // set TXACK
                                self.reg.c1.modify(|mut c1| {
                                    c1.set_bit(3, true);
                                    c1
                                });
                            }
                        }

                        state.rx_buf.push_back(self.reg.d.read()); // read in data
                    }

                    if state.status == Status::Receive || state.status == Status::Timeout {
                        Err(Error::Timeout)
                    } else {
                        Ok(())
                    }
                }
            } else {
                #[cfg(feature = "i2c-dma")]
                { // All this because attributes are not allowed on if statements yet
                    // limit transfers less than 5 bytes to ISR method
                    if self.op_mode == OpMode::DMA && state.rx_req >= 5 {
                        // init DMA, let the hack begin
                        /*
                        i2c->activeDMA = I2C_DMA_ADDR;
                        i2c->DMA->source(*(i2c->D));
                        i2c->DMA->destinationBuffer(&i2c->rxBuffer[0],i2c->reqCount-1); // DMA gets all except last byte
                        */
                    }
                }

                // enable interrupts
                unsafe { self.reg.c1.write(0xF0); }
                // address + read bit
                unsafe { self.reg.d.write(state.tx_buf.pop_front().expect("Failed to pop address byte") | 0x01); }

                Ok(())
            }
        })
    }

    // TODO: return enum rather than boolean?
    /// Returns `Ok(true)` when the I2C transfer is complete.
    /// Returns `Ok(false)` when the I2C transfer is still running.
    /// Returns `Err()` when the I2C transfer has error-ed out.
    pub fn done(&self) -> Result<bool, Error> {
        interrupt::free(|cs| {
            let state = match self.bus {
                0 => I2C0_STATE.borrow(cs).borrow(),
                1 => I2C1_STATE.borrow(cs).borrow(),
                _ => unreachable!(),
            };
            // TODO: format! does not exist,find another way
            let state = state.as_ref().expect("I2CX_STATE uninitialized");

            match state.status {
                Status::Idle => Ok(true),
                Status::Address => Ok(false),
                Status::Transmit => Ok(false),
                Status::Receive => Ok(false),
                Status::Timeout => Err(Error::Timeout),
                Status::Nak => Err(Error::Nak),
                //Status::AddrNak => Err(Error::AddrNak),
                //Status::DataNak => Err(Error::DataNak),
                Status::ArbLost => Err(Error::ArbLost),
                Status::BufOverflow => Err(Error::BufOverflow),
                #[cfg(feature = "i2c-slave")]
                Status::SlaveTransmit => Ok(false),
                #[cfg(feature = "i2c-slave")]
                Status::SlaveReceive => Ok(false),
                #[cfg(feature = "i2c-isr")]
                Status::Fatal => Err(Error::Fatal),
            }
        })
    }

    // TODO: add timeout
    pub fn finish(&mut self) -> Result<(), Error> {
        loop {
            match self.done() {
                Ok(false) => {},
                _ => break,
            }
        }

        // TODO: timeout when using DMA

        // only valid with timeout in place
        /*
        if let Ok(false) = self.done() {
            interrupt::free(|cs| {
                let state = match self.bus {
                    0 => unsafe { I2C0_STATE.as_mut().expect("I2C0_STATE uninitialized").borrow_mut(cs) },
                    1 => unsafe { I2C1_STATE.as_mut().expect("I2C1_STATE uninitialized").borrow_mut(cs) },
                    _ => unreachable!(),
                };

                state.status = Status::Timeout;
            })
        }
        */

        // delay to allow bus to settle (eg. allow STOP to complete and be recognized,
        //                               not just on our side, but on slave side also)
        //delayMicroseconds(4);
        match self.done() {
            Ok(true) => Ok(()),
            Ok(false) => Err(Error::Timeout), // Technically should never happen
            Err(err) => Err(err),
        }
    }

    // TODO: add timeout
    // TODO: maybe rename to `pop_byte`
    pub fn read_byte(&self) -> Result<u8, ()> {
        interrupt::free(|cs| {
            let mut state = match self.bus {
                0 => I2C0_STATE.borrow(cs).borrow_mut(),
                1 => I2C1_STATE.borrow(cs).borrow_mut(),
                _ => unreachable!(),
            };
            // TODO: format! does not exist,find another way
            let state = state.as_mut().expect("I2CX_STATE uninitialized");

            state.rx_buf.pop_front().ok_or(())
        })
    }

    /// Write byte to transmit buffer.
    ///
    /// Returns `Ok` if the write was successful, `Err` if the buffer was full.
    fn write_byte(&mut self, b: u8) -> Result<(), ()> {
        interrupt::free(|cs| {
            let mut state = match self.bus {
                0 => I2C0_STATE.borrow(cs).borrow_mut(),
                1 => I2C1_STATE.borrow(cs).borrow_mut(),
                _ => unreachable!(),
            };
            // TODO: format! does not exist,find another way
            let state = state.as_mut().expect("I2CX_STATE uninitialized");

            if state.tx_buf.push_back(b).is_none() {
                Ok(())
            } else {
                Err(())
            }
        })
    }
}

pub struct Transmission<'a:'c, 'b:'c, 'c> {
    i2c: &'c mut I2cMaster<'a, 'b>
}

impl<'a, 'b, 'c> Transmission<'a, 'b, 'c> {
    pub fn send_transmission(self, stop: Stop) -> Result<(), Error> {
        self.i2c.send_transmission(stop)
    }

    pub fn send_request(self, len: usize, stop: Stop) -> Result<(), Error> {
        self.i2c.send_request(len, stop)
    }

    pub fn write_byte(&mut self, b: u8) -> Result<(), ()> {
        self.i2c.write_byte(b)
    }
}

#[cfg(feature = "i2c-slave")]
impl<'a, 'b> I2cSlave<'a, 'b> {
    pub unsafe fn new(scl: I2cScl<'a>,
                      sda: I2cSda<'b>,
                      nvic: &mut NVIC,
                      addr: Address,
                      general_call: bool,
                      gate: ClockGate)
                      -> Result<I2cSlave<'a, 'b>, ()> {
        let bus = sda.bus();

        let reg = match bus {
            0 => &mut *(0x40066000 as *mut I2cRegs),
            1 => &mut *(0x40067000 as *mut I2cRegs),
            _ => unimplemented!()
        };

        // init ram vars for tx (IICEN and IICIE)
        // init ram vars
        interrupt::free(|cs| {
            match bus {
                0 => *I2C0_STATE.borrow(cs).borrow_mut() = Some(IsrState::new()),
                1 => *I2C1_STATE.borrow(cs).borrow_mut() = Some(IsrState::new()),
                _ => unreachable!(),
            };
        });

        // c2
        // - e/d general call
        // - 10/7 bit addr
        reg.c2.modify(|mut c2| {
            c2.set_bit(7, general_call);
            if let Address::Bits10(addr) = addr {
                c2.set_bit(6, true);
                c2.set_bits(0..3, ((addr >> 7) & 0x7) as u8);
            } else {
                c2.set_bit(6, false);
            }
            c2
        });
        // a1 - slave addr
        let addr = match addr {
            Address::Bits7(addr) => addr,
            Address::Bits10(addr) => addr as u8,
        };
        reg.a1.write(addr << 1);

        // Official Teensy code does this, but doesn't state why
        // TODO: test
        reg.c2.modify(|mut c2| {
            c2.set_bit(5, true); // high drive select
            c2
        });

        let interrupt = match bus {
            0 => Interrupt::I2C0,
            1 => Interrupt::I2C0,
            _ => unreachable!(),
        };
        interrupt::free(|_| {
            nvic.enable(interrupt);
        });

        reg.c1.write(0xC0); // enable module and interrupts

        Ok(I2cSlave { reg: reg, _scl: scl, _sda: sda, _gate: gate, bus: bus })
    }

    pub fn set_rx_callback(&mut self, callback: Option<fn(u8, &ArrayDeque<[u8; 64]>)>) {
        unimplemented!()
    }

    pub fn set_tx_callback(&mut self, callback: Option<fn(u8, &mut ArrayDeque<[u8; 64]>)>) {
        unimplemented!()
    }
}

// TODO: maybe spit to enum for master/slave functionality
// TODO: somehow allow users to specify the size of th buffers. may require const generics
struct IsrState {
    status: Status,
    stop: Stop,
    rx_req: usize, // TODO: maybe make option
    timeout_rx_nak: bool,
    tx_buf: ArrayDeque<[u8; 64]>,
    rx_buf: ArrayDeque<[u8; 64]>,

    #[cfg(feature = "i2c-slave")]
    rx_addr: u8,
    #[cfg(feature = "i2c-slave")]
    slave_rx_callback: Option<fn(u8, &ArrayDeque<[u8; 64]>)>,
    #[cfg(feature = "i2c-slave")]
    slave_tx_callback: Option<fn(u8, &mut ArrayDeque<[u8; 64]>)>
}

impl IsrState {
    fn new() -> Self {
        IsrState {
            status: Status::Idle,
            stop: Stop::Yes,
            rx_req: 0,
            timeout_rx_nak: false,
            tx_buf: ArrayDeque::new(),
            rx_buf: ArrayDeque::new(),

            #[cfg(feature = "i2c-slave")]
            rx_addr: 0,
            #[cfg(feature = "i2c-slave")]
            slave_rx_callback: None,
            #[cfg(feature = "i2c-slave")]
            slave_tx_callback: None,
        }
    }
}

#[derive(Debug,PartialEq)]
enum Status {
    Idle,
    Address,
    Transmit,
    Receive,
    Timeout,
    Nak,
    //AddrNak,
    //DataNak,
    ArbLost,
    BufOverflow,

    #[cfg(feature = "i2c-slave")]
    SlaveTransmit,
    #[cfg(feature = "i2c-slave")]
    SlaveReceive,

    #[cfg(feature = "i2c-isr")]
    Fatal
}

#[derive(PartialEq)]
pub enum Stop {
    Yes,
    No
}

static I2C0_STATE: Mutex<RefCell<Option<IsrState>>> = Mutex::new(RefCell::new(None));
static I2C1_STATE: Mutex<RefCell<Option<IsrState>>> = Mutex::new(RefCell::new(None));

#[cfg(feature = "i2c-isr")]
interrupt!(I2C0, isr0);
#[cfg(feature = "i2c-isr")]
interrupt!(I2C1, isr1);

#[cfg(feature = "i2c-isr")]
fn isr0() {
    unsafe {
        interrupt::free(|cs| {
            let reg = &mut *(0x40066000 as *mut I2cRegs);
            let mut state = I2C0_STATE.borrow(cs).borrow_mut();
            let state = state.as_mut().expect("I2C0_STATE uninitialized");

            isr(reg, state);
        });
    }
}

#[cfg(feature = "i2c-isr")]
fn isr1() {
    unsafe {
        interrupt::free(|cs| {
            let reg = &mut *(0x40067000 as *mut I2cRegs);
            let mut state = I2C1_STATE.borrow(cs).borrow_mut();
            let state = state.as_mut().expect("I2C1_STATE uninitialized");

            isr(reg, state);
        });
    }
}

// TODO: support SMBus
// TODO: support DMA
#[cfg(feature = "i2c-isr")]
unsafe fn isr(reg: &mut I2cRegs, state: &mut IsrState) {
    let status = reg.s.read();
    let c1 = reg.c1.read();

    reg.s.write(0x02); // clear IICIF

    // Check if master or slave
    if c1.get_bit(5) {
        // Master
        if c1.get_bit(4) { // check for currently transmitting or receiving
            match state.status {
                Status::Transmit => {
                    // TODO: find some way to test this
                    // This is not called out in the Master mode branch in the
                    // in the flowchart in the TRM, but the Teensy folks do it
                    // so I'm putting it in
                    // The switch to slave rx might be automatically handled by
                    // the hardware so this would never get hit
                    if status.get_bit(4) { // check arbitration
                        state.status = Status::ArbLost;
                        reg.s.write(0x10); // clear ARBL
                        // TODO: this is clearly not right. After ARBL it should
                        // drop into IMM slave mode if IAAS=1. Right now Rx
                        // message would be ignored regardless of IAAS

                        // change to Rx mode, intr disabled
                        // (does this send STOP if ARBL flagged?)
                        reg.c1.write(0x80); // Only module enabled
                    } else if status.get_bit(0) { // check if slave acked
                        // TODO: separate out data and addr so we can give different errors
                        state.status = Status::Nak;
                        reg.c1.write(0x80); // Only module enabled
                    } else {
                        if let Some(d) = state.tx_buf.pop_front() {
                            reg.d.write(d);
                        } else {
                            state.status = Status::Idle;

                            match state.stop {
                                Stop::Yes => reg.c1.write(0x80), // only module enabled
                                Stop::No => reg.c1.write(0xB0), // no stop, still in tx, intr disabled
                            }
                        }
                    }
                }
                Status::Address => {
                    // TODO: find some way to test this
                    // This is not called out in the Master mode branch in the
                    // in the flowchart in the TRM, but the Teensy folks do it
                    // so I'm putting it in
                    if status.get_bit(4) { // check arbitration
                        state.status = Status::ArbLost;
                        reg.s.write(0x10); // clear ARBL
                        // TODO: this is clearly not right. After ARBL it should
                        // drop into IMM slave mode if IAAS=1. Right now Rx
                        // message would be ignored regardless of IAAS

                        // change to Rx mode, intr disabled
                        // (does this send STOP if ARBL flagged?)
                        reg.c1.write(0x80); // Only module enabled
                    } else if status.get_bit(0) { // check if slave acked
                        // TODO: separate out data and addr so we can give different errors
                        state.status = Status::Nak;
                        reg.c1.write(0x80); // Only module enabled
                    } else {
                        if let Some(d) = state.tx_buf.pop_front() {
                            reg.d.write(d);
                        } else {
                            state.status = Status::Receive;
                            // switch to rx mode
                            reg.c1.modify(|mut c1| {
                                c1.set_bit(4, false);
                                if state.rx_req <= 1 {
                                    c1.set_bit(3, true);
                                }
                                c1
                            });
                            reg.d.read(); // dummy read
                        }
                    }
                }
                Status::Timeout => {
                    match state.stop {
                        Stop::Yes => reg.c1.write(0x80), // only module enabled
                        Stop::No => reg.c1.write(0xB0), // no stop, still in tx, intr disabled
                    }
                }
                _ => {
                    // we should not be here. something went really wrong
                    state.status = Status::Fatal;
                    reg.c1.write(0x80); // only module enabled
                }
            }
        } else {
            if state.status == Status::Receive { // Not REALLY needed, but might was well double check
                if state.rx_buf.len() + 1 == state.rx_req ||
                   (state.status == Status::Timeout && state.timeout_rx_nak) {
                    state.timeout_rx_nak = false;

                    if state.status != Status::Timeout {
                        state.status = Status::Idle;
                    }
                    // change to tx
                    reg.c1.modify(|mut c1| {
                        c1.set_bit(4, true);
                        c1
                    });

                    match state.stop {
                        Stop::Yes => {
                            //delayMicroseconds(1); // empirical patch, lets things settle before issuing STOP
                            reg.c1.write(0x80); // only module enabled
                        }
                        Stop::No => reg.c1.write(0xB0), // no stop, tx, intr disabled
                    }
                } else if state.rx_buf.len() + 2 == state.rx_req ||
                          (state.status == Status::Timeout && !state.timeout_rx_nak) {
                    if state.status == Status::Timeout && !state.timeout_rx_nak {
                        state.timeout_rx_nak = true;
                    }

                    // set TXACK
                    reg.c1.modify(|mut c1| {
                        c1.set_bit(3, true);
                        c1
                    });
                }
                
                state.rx_buf.push_back(reg.d.read()); // read in data
            } else {
                // we should not be here. something went really wrong
                state.status = Status::Fatal;
                reg.c1.write(0x80); // only module enabled
            }
        }
    } else {
        // Slave
        if status.get_bit(4) { // check arbitration
            state.status = Status::ArbLost;
            reg.s.write(0x10); // clear ARBL
            if !status.get_bit(6) { // not addressed as slave
                return;
            }
        }

        #[cfg(feature = "i2c-slave")]
        {
            if status.get_bit(6) { // addressed as slave
                // repeated START occurred, run callback
                if let (&Status::SlaveReceive, Some(callback)) = (&state.status, state.slave_rx_callback) {
                    callback(state.rx_addr, &state.rx_buf);
                    state.rx_buf.clear();
                }
                // set read or write mode
                if status.get_bit(2) {
                    // write
                    state.status = Status::SlaveTransmit;
                    state.tx_buf.clear();
                    // change to tx
                    reg.c1.modify(|mut c1| {
                        c1.set_bit(4, true);
                        c1
                    });
                    if let Some(callback) = state.slave_tx_callback {
                        let addr = reg.d.read() >> 1; // read target addr
                        callback(addr, &mut state.tx_buf);
                    }
                    if let Some(d) = state.tx_buf.pop_front() {
                        reg.d.write(d);
                    } else {
                        reg.d.write(0);
                    }
                } else {
                    // read
                    state.status = Status::SlaveReceive;
                    state.rx_buf.clear();
                    // change to rx
                    reg.c1.modify(|mut c1| {
                        c1.set_bit(4, false);
                        c1
                    });
                    state.rx_addr = reg.d.read() >> 1; // read target addr

                    // TODO: attach interrupt on pin
                    /*
                    i2c->irqCount = 0;
                    if(i2c->currentPins == I2C_PINS_18_19)
                        attachInterrupt(18, i2c_t3::sda0_rising_isr, RISING);
                    else if(i2c->currentPins == I2C_PINS_16_17)
                        attachInterrupt(17, i2c_t3::sda0_rising_isr, RISING);
                    else if(i2c->currentPins == I2C_PINS_29_30)
                        attachInterrupt(30, i2c_t3::sda1_rising_isr, RISING);
                    else if(i2c->currentPins == I2C_PINS_26_31)
                        attachInterrupt(31, i2c_t3::sda1_rising_isr, RISING);
                    */
                }
            } else {
                // data transfer
                if c1.get_bit(4) {
                    // slave transmit
                    if !status.get_bit(0) {
                        // master ack - continue transmitting
                        if let Some(d) = state.tx_buf.pop_front() {
                            reg.d.write(d);
                        } else {
                            reg.d.write(0);
                        }
                    } else {
                        // master nak - stop transmitting
                        reg.c1.write(0xE0); // rx mode
                        reg.d.read(); // dummy read
                        state.status = Status::Idle;
                    }
                } else if state.status == Status::SlaveReceive {
                    // ^ done so this doesn't just happen after arbitration is lost
                    // slave receive

                    // TODO: attach interrupt on pin
                    /*
                    i2c->irqCount = 0;
                    if(i2c->currentPins == I2C_PINS_18_19)
                        attachInterrupt(18, i2c_t3::sda0_rising_isr, RISING);
                    else if(i2c->currentPins == I2C_PINS_16_17)
                        attachInterrupt(17, i2c_t3::sda0_rising_isr, RISING);
                    else if(i2c->currentPins == I2C_PINS_29_30)
                        attachInterrupt(30, i2c_t3::sda1_rising_isr, RISING);
                    else if(i2c->currentPins == I2C_PINS_26_31)
                        attachInterrupt(31, i2c_t3::sda1_rising_isr, RISING);
                    */

                    state.rx_buf.push_back(reg.d.read());
                }
            }
        }
    }
}