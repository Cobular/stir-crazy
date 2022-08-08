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
        button_val: button_pin.is_high().unwrap(),
    }
}

fn ms_to_duty(freq: u8, ms: f32) -> u16 {
    (((ms / 1000f32) / (1f32 / freq as f32)) * (u16::MAX as f32)) as u16
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
    let mut button_pin = pins.gpio22.into_pull_down_input();
    let mut pot_pin = pins.gpio26.into_floating_input();
    let mut servo_pin: Pin<_, FunctionPwm> = pins.gpio1.into_mode();

    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.enable();

    let mut pwm = pwm.into_mode::<InputHighRunning>();
    pwm.set_ph_correct();
    pwm.set_div_int(20u8); // 50 hz
    pwm.enable();

    // Use A channel (which outputs to GPIO 24)
    let mut channel_a = pwm.channel_b;
    let _channel_pin_a = channel_a.output_to(servo_pin);

    let fwd = ms_to_duty(50, 2.0);
    info!("{=u16}", fwd);
    channel_a.set_duty(fwd);

    loop {
        let poll_res = poll(&mut adc, &mut pot_pin, &mut button_pin);
        // let button_val
        info!("{=u16}, {=bool}", poll_res.pot_val, poll_res.button_val);
        led_pin.set_high().unwrap();
        delay.delay_ms(50);
        // info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(50);
    }
}

// End of file
