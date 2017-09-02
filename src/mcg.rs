use core::mem;
use core::sync::atomic::{AtomicBool,ATOMIC_BOOL_INIT,Ordering};

use volatile::Volatile;
use bit_field::BitField;

use osc::OscToken;

#[repr(C,packed)]
struct McgRegs {
    c1:     Volatile<u8>,
    c2:     Volatile<u8>,
    c3:     Volatile<u8>,
    c4:     Volatile<u8>,
    c5:     Volatile<u8>,
    c6:     Volatile<u8>,
    s:      Volatile<u8>,
    _pad0:  u8,
    sc:     Volatile<u8>,
    _pad1:  u8,
    atcvh:  Volatile<u8>,
    atcvl:  Volatile<u8>,
    c7:     Volatile<u8>,
    c8:     Volatile<u8>,
}

pub struct Mcg {
    reg: &'static mut McgRegs
}

pub enum OscRange {
    Low = 0,
    High = 1,
    VeryHigh = 2
}

enum OscSource {
    LockedLoop = 0,
    Internal = 1,
    External = 2
}

pub enum Clock {
    Fei(Fei),
    Fee(Fee),
    Fbi(Fbi),
    Fbe(Fbe),
    Pee(Pee),
    Pbe(Pbe),
    Blpi(Blpi),
    Blpe(Blpe),
    Stop(Stop)
}

static MCG_INIT: AtomicBool = ATOMIC_BOOL_INIT;

impl Mcg {
    pub fn new() -> Mcg {
        let was_init = MCG_INIT.swap(true, Ordering::Relaxed);
        if was_init {
            panic!("Cannot initialize MCG: It's already active");
        }
        let reg = unsafe { &mut *(0x40064000 as *mut McgRegs) };
        Mcg { reg: reg }
    }

    //TODO: Stop
    pub fn clock(self) -> Clock {
        let source: OscSource = unsafe {
            mem::transmute(self.reg.c1.read().get_bits(6..8))
        };
        let fll_internal = self.reg.c1.read().get_bit(2);
        let pll_enabled = self.reg.c6.read().get_bit(6);
        let low_power = self.reg.c2.read().get_bit(1);

        match (source, fll_internal, pll_enabled, low_power) {
            (OscSource::LockedLoop, true, false, _) => Clock::Fei(Fei { mcg: self }),
            (OscSource::LockedLoop, false, false, _) => Clock::Fee(Fee { mcg: self }),
            (OscSource::Internal, true, false, false) => Clock::Fbi(Fbi { mcg: self }),
            (OscSource::External, false, false, false) => Clock::Fbe(Fbe { mcg: self }),
            (OscSource::LockedLoop, false, true, _) => Clock::Pee(Pee { mcg: self }),
            (OscSource::External, false, true, false) => Clock::Pbe(Pbe { mcg: self }),
            (OscSource::Internal, true, false, true) => Clock::Blpi(Blpi { mcg: self }),
            (OscSource::External, false, _, true) => Clock::Blpe(Blpe { mcg: self }),
            _ => panic!("The current clock mode cannot be represented as a known struct")
        }
    }
}

impl Drop for Mcg {
    fn drop(&mut self) {
        MCG_INIT.store(false, Ordering::Relaxed);
    }
}

pub struct Fei {
    mcg: Mcg
}

pub struct Fee {
    mcg: Mcg
}

pub struct Fbi {
    mcg: Mcg
}

pub struct Fbe {
    mcg: Mcg
}

pub struct Pbe {
    mcg: Mcg
}

pub struct Pee {
    mcg: Mcg
}

pub struct Blpi {
    mcg: Mcg
}

pub struct Blpe {
    mcg: Mcg
}

pub struct Stop {
    mcg: Mcg
}

impl Fei {
    pub fn enable_xtal(&mut self, range: OscRange, _token: OscToken) {
        self.mcg.reg.c2.update(|c2| {
            c2.set_bits(4..6, range as u8); // frequency range
            c2.set_bit(2, true); // internal osc
        });

        while !self.mcg.reg.s.read().get_bit(1) {}
    }

    pub fn disable_xtal(&mut self) -> OscToken {
        self.mcg.reg.c2.update(|c2| {
            c2.set_bits(4..6, 0); // frequency range
            c2.set_bit(2, false); // internal osc
        });

        while !self.mcg.reg.s.read().get_bit(1) {}

        OscToken::new()
    }

    pub fn use_external_bypass(self, divide: u32) -> Fbe {
        let osc = self.mcg.reg.c2.read().get_bits(4..6);
        let frdiv = if osc == OscRange::Low as u8 {
            match divide {
                1 => 0,
                2 => 1,
                4 => 2,
                8 => 3,
                16 => 4,
                32 => 5,
                64 => 6,
                128 => 7,
                _ => panic!("Invalid external clock divider: {}", divide)
            }
        } else {
            match divide {
                32 => 0,
                64 => 1,
                128 => 2,
                256 => 3,
                512 => 4,
                1024 => 5,
                1280 => 6,
                1536 => 7,
                _ => panic!("Invalid external clock divider: {}", divide)
            }
        };

        self.mcg.reg.c1.update(|c1| {
            c1.set_bits(6..8, OscSource::External as u8);
            c1.set_bits(3..6, frdiv);
            c1.set_bit(2, false); // external clock
        });

        // Once we write to the control register, we need to wait for
        // the new clock to stabilize before we move on.
        // First: Wait for the FLL to be pointed at the crystal
        // Then: Wait for our clock source to be the crystal osc
        while self.mcg.reg.s.read().get_bit(4) {}
        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::External as u8 {}

        Fbe { mcg: self.mcg }
    }
}

impl Fbe {
    pub fn enable_pll(self, numerator: u8, denominator: u8) -> Pbe {
        if numerator < 24 || numerator > 55 {
            panic!("Invalid PLL VCO divide factor: {}", numerator);
        }

        if denominator < 1 || denominator > 25 {
            panic!("Invalid PLL reference divide factor: {}", denominator);
        }

        self.mcg.reg.c5.update(|c5| {
            c5.set_bits(0..5, denominator - 1);
        });

        self.mcg.reg.c6.update(|c6| {
            c6.set_bits(0..5, numerator - 24);
            c6.set_bit(6, true);
        });

        // Wait for PLL to be enabled
        while !self.mcg.reg.s.read().get_bit(5) {}
        // Wait for the PLL to be "locked" and stable
        while !self.mcg.reg.s.read().get_bit(6) {}

        Pbe { mcg: self.mcg }
    }
}

impl Pbe {
    pub fn use_pll(self) -> Pee {
        self.mcg.reg.c1.update(|c1| {
            c1.set_bits(6..8, OscSource::LockedLoop as u8);
        });

        // mcg.c1 and mcg.s have slightly different behaviors.  In c1,
        // we use one value to indicate "Use whichever LL is
        // enabled". In s, it is differentiated between the FLL at 0,
        // and the PLL at 3. Instead of adding a value to OscSource
        // which would be invalid to set, we just check for the known
        // value "3" here.
        while self.mcg.reg.s.read().get_bits(2..4) != 3 {}

        Pee { mcg: self.mcg }
    }

    pub fn disable_pll(self) -> Fbe {
        self.mcg.reg.c6.update(|c6| {
            c6.set_bit(6, false);
        });

        // Wait for FLL to be enabled
        while self.mcg.reg.s.read().get_bit(5) {}

        Fbe { mcg: self.mcg }
    }
}

impl Pee {
    pub fn bypass_pll(self) -> Pbe {
        self.mcg.reg.c1.update(|c1| {
            c1.set_bits(6..8, OscSource::External as u8);
        });

        while self.mcg.reg.s.read().get_bits(2..4) != OscSource::External as u8 {}

        Pbe { mcg: self.mcg }
    }
}
