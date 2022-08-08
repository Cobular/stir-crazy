//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{
        gpio::{AnyPin, Floating, FunctionPwm, Input, InputConfig},
        pwm::{InputHighRunning, Slices, ValidPwmOutputPin},
    },
    hal::{
        gpio::{Pin, SomePinId},
        Adc,
    },
};
use core::fmt::Debug;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{
    adc::{Channel, OneShot},
    digital::v2::InputPin,
    digital::v2::OutputPin,
    PwmPin,
};
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

struct PollRes {
    pot_val: u16,
    button_val: bool,
}

fn poll<A, BC, B>(adc: &mut Adc, adc_pin: &mut A, button_pin: &mut B) -> PollRes
where
    A: AnyPin<Mode = Input<Floating>> + Channel<Adc, ID = u8>,
    BC: InputConfig,
    B: AnyPin<Mode = Input<BC>> + InputPin,
    <B as embedded_hal::digital::v2::InputPin>::Error: Debug,
{
    PollRes {
        pot_val: adc.read(adc_pin).unwrap(),
        button_val: button_pin.is_low().unwrap(),
    }
}

const TOTAL_DELAY: u32 = 50_000;

const STOP_US_ON: u32 = 1500;
const MAX_REV_US_ON: u32 = 1000;

const MAX_VS_MIN: u32 = 5;

const MAX_POT_VAL: u16 = 4000;
const MAX_POT_VAL_SMOL: u16 = 40;
const MIN_POT_VAL: u16 = 100;
const MIN_POT_VAL_SMOL: u16 = 1;

fn make_delays(fwd: bool, pot_val: u16) -> (u32, u32) {
    let percent: u32 = match pot_val {
        _ if pot_val < MIN_POT_VAL => 0,
        _ if pot_val > MAX_POT_VAL => 100,
        _ => ((pot_val - MIN_POT_VAL) / (MAX_POT_VAL_SMOL - MIN_POT_VAL_SMOL)) as u32,
    };

    let short_delay = if fwd {
        ((percent * MAX_VS_MIN) + STOP_US_ON) as u32
    } else {
        (STOP_US_ON - (percent * MAX_VS_MIN)) as u32
    };

    info!("short_delay: {=u32}", short_delay);

    let long_delay = TOTAL_DELAY - short_delay;
    (short_delay, long_delay)
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    let mut led_pin = pins.led.into_push_pull_output();
    let mut servo_pin = pins.gpio0.into_push_pull_output();
    let mut button_pin = pins.gpio22.into_pull_down_input();
    let mut pot_pin = pins.gpio26.into_floating_input();

    let mut delays = (1500, TOTAL_DELAY - 1500);

    loop {
        let poll_res = poll(&mut adc, &mut pot_pin, &mut button_pin);
        delays = make_delays(poll_res.button_val, poll_res.pot_val);
        info!("{=u16}, {=bool}", poll_res.pot_val, poll_res.button_val);
        led_pin.set_high().unwrap();
        servo_pin.set_high().unwrap();
        delay.delay_us(delays.0);
        led_pin.set_low().unwrap();
        servo_pin.set_low().unwrap();
        delay.delay_us(delays.1);
    }
}

// End of file
