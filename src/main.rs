//! # Pico PWM Micro Servo Example
//!
//! Moves the micro servo on a Pico board using the PWM peripheral.
//!
//! This will move in different positions the motor attached to GP1.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

mod utils;

// The macro for our start-up function
use cortex_m_rt::entry;

use cortex_m::prelude::*;
use panic_probe as _;

// GPIO traits
use embedded_hal::PwmPin;

// Traits for converting integers to amounts of time
use embedded_time::duration::Extensions;

use embedded_time::fixed_point::FixedPoint;
use rp_pico::hal::gpio::{FunctionPwm, Pin};
use rp_pico::hal::pwm::ValidPwmOutputPin;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::{pac, Clock};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use utils::calc_div;

use core::fmt::Debug;
use defmt::*;
use defmt_rtt as _;

use crate::utils::enable_pwm_for_pin;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let _clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sys_clk: u32 = _clocks.system_clock.freq().0;

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, _clocks.system_clock.freq().integer());

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut led_pin = pins.led.into_mode::<FunctionPwm>();

    enable_pwm_for_pin(&mut pwm_slices.pwm4, &mut led_pin, 50, sys_clk);

    // Output channel B on PWM0 to the GPIO1 pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    channel.set_duty(1000);

    // Infinite loop, moving micro servo from one position to another.
    // You may need to adjust the pulse width since several servos from
    // different manufacturers respond differently.
    loop {
        // 0° to 90°
        delay.delay_ms(500);
    }
}

// End of file
