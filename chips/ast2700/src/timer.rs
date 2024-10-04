// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Timer driver.

use kernel::hil::time::{Freq1MHz, Ticks, Ticks64};
use kernel::hil::{self, time};
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::ReadWrite;
use kernel::utilities::registers::{register_bitfields, register_structs};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

register_structs! {
    pub TimerRegisters {
        (0x00 => count_l: ReadWrite<u32>),
        (0x04 => count_h: ReadWrite<u32>),
        (0x08 => alarm_l: ReadWrite<u32>),
        (0x0C => alarm_h: ReadWrite<u32>),
        (0x10 => ctrl: ReadWrite<u32, CTRL::Register>),
        (0x14 => @END),
    }
}

register_bitfields! [u32,
    CTRL [
        INT_STS    OFFSET(7) NUMBITS(1) [],
        CNT_CLR    OFFSET(4) NUMBITS(1) [],
        WDTRST_EN  OFFSET(3) NUMBITS(1) [],
        CLK_SEL    OFFSET(1) NUMBITS(1) [ CLK_PCLK = 0, CLK_1M = 1 ],
        INT_EN     OFFSET(0) NUMBITS(1) [],
    ],
];

pub struct Timer<'a> {
    regs: StaticRef<TimerRegisters>,
    alarm_client: OptionalCell<&'a dyn time::AlarmClient>,
}

impl<'a> Timer<'a> {
    pub fn new(base: StaticRef<TimerRegisters>) -> Timer<'a> {
        Timer {
            regs: base,
            alarm_client: OptionalCell::empty(),
        }
    }

    pub fn setup(&self) {
        self.regs.ctrl.modify(
            CTRL::INT_STS::SET + CTRL::WDTRST_EN::SET + CTRL::CLK_SEL::CLK_1M + CTRL::INT_EN::CLEAR,
        );
    }

    pub fn service_interrupt(&self) {
        self.alarm_client.map(|client| client.alarm());
    }
}

impl<'a> hil::time::Time for Timer<'_> {
    type Frequency = Freq1MHz;
    type Ticks = Ticks64;

    fn now(&self) -> Self::Ticks {
        let cnt_l = self.regs.count_l.get();
        let cnt_h = self.regs.count_h.get();

        Ticks64::from(((cnt_h as u64) << 32) | cnt_l as u64)
    }
}

impl<'a> time::Alarm<'a> for Timer<'a> {
    fn set_alarm_client(&self, client: &'a dyn time::AlarmClient) {
        self.alarm_client.set(client);
    }

    fn set_alarm(&self, reference: Self::Ticks, dt: Self::Ticks) {
        let expire = reference.wrapping_add(dt);
        let expire_u64 = expire.into_u64();

        self.regs.alarm_h.set(0xffff_ffff);

        self.regs.alarm_l.set(expire_u64 as u32);
        self.regs.alarm_h.set((expire_u64 >> 32) as u32);

        self.regs.ctrl.modify(CTRL::INT_EN::SET);
    }

    fn get_alarm(&self) -> Self::Ticks {
        let mut val = (self.regs.alarm_h.get() as u64) << 32;
        val |= self.regs.alarm_l.get() as u64;

        Ticks64::from(val)
    }

    fn disarm(&self) -> Result<(), ErrorCode> {
        self.regs.ctrl.modify(CTRL::INT_STS::SET);
        self.regs.ctrl.modify(CTRL::INT_EN::CLEAR);

        Ok(())
    }

    fn is_armed(&self) -> bool {
        self.regs.ctrl.is_set(CTRL::INT_EN)
    }

    fn minimum_dt(&self) -> Self::Ticks {
        Ticks64::from(1u64)
    }
}
