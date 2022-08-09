use embedded_time::Clock;
use rp_pico::hal::{
    gpio::{bank0::BankPinId, Pin, PinId, PinMode, ValidPinMode},
    pwm::{
        ChannelId, DynChannelId, DynSliceId, FreeRunning, Slice, SliceId, SliceMode,
        ValidPwmOutputPin, ValidSliceMode, A,
    },
};

const fn ceil(a: u32, b: u32) -> u32 {
    (a + b - 1) / b
}

pub struct ClockData {
    pub div_16: u16,
    pub div_int: u8,
    pub div_frac: u8,
    pub wrap: u16,
}

pub const fn calc_div(sys_clk: u32, pwm_freq: u32) -> ClockData {
    let div_16 = ceil(sys_clk, 4096 * pwm_freq);

    ClockData {
        div_16: div_16 as u16,
        div_int: (div_16 / 16) as u8,
        div_frac: (div_16 & 0x0Fu32) as u8,
        wrap: (sys_clk * 16 / div_16 / pwm_freq - 1) as u16,
    }
}

pub fn enable_pwm_for_pin<S, M, C, PM, PI>(
    pwm_slice: &mut Slice<S, FreeRunning>,
    pin: &mut Pin<PI, PM>,
    freq: u32,
    sys_clk: u32,
) where
    S: SliceId,
    M: SliceMode + ValidSliceMode<S>,
    C: ChannelId,
    PI: PinId + BankPinId + ValidPwmOutputPin<S, C>,
    PM: PinMode + ValidPinMode<PI>,
{
    let pwm_data = calc_div(sys_clk, freq);

    pwm_slice.set_ph_correct();
    pwm_slice.set_top(pwm_data.wrap);
    pwm_slice.set_div_int(pwm_data.div_int);
    pwm_slice.set_div_frac(pwm_data.div_frac);
    pwm_slice.enable();
}
