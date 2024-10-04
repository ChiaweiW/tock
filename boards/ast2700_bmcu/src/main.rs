// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Tock kernel for the AST2700 BootMCU

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]

use ast2700::chip::{Ast2700, Ast2700DefaultPeripheral};
use capsules_core::virtualizers::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use core::fmt::Write;
use core::panic::PanicInfo;
use kernel::platform::scheduler_timer::VirtualSchedulerTimer;

use core::ptr::{addr_of, addr_of_mut, write_volatile};
use kernel::component::Component;
use kernel::debug::IoWrite;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::cooperative::CooperativeSched;
use kernel::utilities::registers::interfaces::ReadWriteable;
use kernel::{capabilities, create_capability, static_init};
use kernel::{debug, hil};
use rv32i::csr;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: capsules_system::process_policies::PanicFaultPolicy =
    capsules_system::process_policies::PanicFaultPolicy {};

static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; 1] = [None; 1];

struct Platform {
    console: &'static capsules_core::console::Console<'static>,
    scheduler: &'static CooperativeSched<'static>,
    scheduler_timer:
        &'static VirtualSchedulerTimer<VirtualMuxAlarm<'static, ast2700::timer::Timer<'static>>>,
}

impl SyscallDriverLookup for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules_core::console::DRIVER_NUM => f(Some(self.console)),
            _ => f(None),
        }
    }
}

impl<'a> KernelResources<ast2700::chip::Ast2700<'a>> for Platform {
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = CooperativeSched<'static>;
    type SchedulerTimer =
        VirtualSchedulerTimer<VirtualMuxAlarm<'static, ast2700::timer::Timer<'static>>>;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        self.scheduler_timer
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    rv32i::configure_trap_handler();

    let peripheral = static_init!(Ast2700DefaultPeripheral, Ast2700DefaultPeripheral::new());

    peripheral.timer.setup();

    let chip = static_init!(Ast2700, Ast2700::new(&peripheral.timer));

    // initlaize capabilities
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&*addr_of!(PROCESSES)));

    let uart_mux = components::console::UartMuxComponent::new(&peripheral.uart, 115200)
        .finalize(components::uart_mux_component_static!());

    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules_core::console::DRIVER_NUM,
        uart_mux,
    )
    .finalize(components::console_component_static!());

    components::debug_writer::DebugWriterComponent::new(uart_mux)
        .finalize(components::debug_writer_component_static!());

    let mux_alarm = static_init!(
        MuxAlarm<'static, ast2700::timer::Timer>,
        MuxAlarm::new(&peripheral.timer)
    );
    hil::time::Alarm::set_alarm_client(&peripheral.timer, mux_alarm);

    let virtual_alarm = static_init!(
        VirtualMuxAlarm<'static, ast2700::timer::Timer>,
        VirtualMuxAlarm::new(mux_alarm)
    );
    virtual_alarm.setup();

    let scheduler =
        components::sched::cooperative::CooperativeComponent::new(&*addr_of!(PROCESSES))
            .finalize(components::cooperative_component_static!(1));

    let scheduler_timer = static_init!(
        VirtualSchedulerTimer<VirtualMuxAlarm<'static, ast2700::timer::Timer>>,
        VirtualSchedulerTimer::new(virtual_alarm)
    );

    // These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            core::ptr::addr_of!(_sapps),
            core::ptr::addr_of!(_eapps) as usize - core::ptr::addr_of!(_sapps) as usize,
        ),
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(_sappmem),
            core::ptr::addr_of!(_eappmem) as usize - core::ptr::addr_of!(_sappmem) as usize,
        ),
        &mut *addr_of_mut!(PROCESSES),
        &FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    // enable timer interrupt only
    csr::CSR.mie.modify(
        csr::mie::mie::mext::CLEAR + csr::mie::mie::msoft::CLEAR + csr::mie::mie::mtimer::SET,
    );

    debug!("AST2700 BMCU @ TockOS: HELLO WORLD!");

    let platform = Platform {
        console,
        scheduler,
        scheduler_timer,
    };

    board_kernel.kernel_loop(
        &platform,
        chip,
        None::<&kernel::ipc::IPC<0>>,
        &main_loop_capability,
    );
}

struct Writer {}

static mut WRITER: Writer = Writer {};

impl Write for Writer {
    fn write_str(&mut self, s: &str) -> ::core::fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

impl IoWrite for Writer {
    fn write(&mut self, buf: &[u8]) -> usize {
        for b in buf {
            // Print to a special address for simulation output
            unsafe {
                write_volatile(0x14c33b00 as *mut u8, *b);
            }
        }
        buf.len()
    }
}

static mut CHIP: Option<&'static ast2700::chip::Ast2700> = None;
static mut PROCESS_PRINTER: Option<&'static capsules_system::process_printer::ProcessPrinterText> =
    None;

/// Panic handler.
#[no_mangle]
#[panic_handler]
pub unsafe fn panic_fmt(pi: &PanicInfo) -> ! {
    use core::ptr::{addr_of, addr_of_mut};

    let writer = &mut *addr_of_mut!(WRITER);

    debug::panic_print(
        writer,
        pi,
        &rv32i::support::nop,
        &*addr_of!(PROCESSES),
        &*addr_of!(CHIP),
        &*addr_of!(PROCESS_PRINTER),
    );

    unreachable!()
}
