use crate::hal::{
    clocks::{Clocks, LfOscConfiguration, HFCLK_FREQ},
    gpio::{p0, p1, Floating, Input, Level, Output, Pin, PullUp, PushPull},
    pac,
    prelude::*,
    pwm::{Channel, Pwm},
    saadc::{InternalVddHdiv5, Reference, SaadcConfig, Time},
    Saadc,
};
use rtic_monotonics::systick::Systick;

pub struct BoardLeds {
    pwm: Pwm<pac::PWM0>,
}

impl BoardLeds {
    pub fn levels(&mut self, red: f32, green: f32) {
        let (red_led, green_led, _, _) = self.pwm.split_channels();

        if red > 0. || green > 0. {
            let max_duty = self.pwm.max_duty();
            let red_level = red.min(1.).max(0.) * max_duty as f32 * 0.1 + 0.5;
            let green_level = green.min(1.).max(0.) * max_duty as f32 + 0.5;

            self.pwm.enable();
            red_led.set_duty_off(red_level as u16);
            green_led.set_duty_off(green_level as u16);
        } else {
            red_led.set_duty_off(0);
            green_led.set_duty_off(0);
            self.pwm.disable();
        }
    }
}

pub struct Voltages {
    vbat: InternalVddHdiv5,
    adc: Saadc,
}

impl Voltages {
    pub fn measure_vbat(&mut self) -> f32 {
        if let Ok(v) = self.adc.read(&mut self.vbat) {
            // VP = (RES / (2^13)) * 2 * (5/12)

            let res = v as f32;
            let vp = (res / ((1 << 14) as f32 * (5. / 12.))) * 5.;

            vp
        } else {
            f32::NEG_INFINITY
        }
    }
}

pub struct ChargerStatus {
    pg: Pin<Input<PullUp>>,
    stat1: Pin<Input<PullUp>>,
    stat2: Pin<Input<PullUp>>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format, Hash)]
pub enum ChargingStatus {
    /// There is no cable connected.
    No5V,
    /// The charger is powered, but not charging.
    Standby,
    /// The battery is charging.
    Charging,
    // The charging has finished.
    ChargeComplete,
}

impl ChargerStatus {
    pub fn usb_connected(&self) -> bool {
        matches!(self.pg.is_low(), Ok(true))
    }

    //
    pub fn status(&self) -> ChargingStatus {
        if !self.usb_connected() {
            return ChargingStatus::No5V;
        }

        let (stat1_low, stat2_low) = (
            matches!(self.stat1.is_low(), Ok(true)),
            matches!(self.stat2.is_low(), Ok(true)),
        );

        match (stat1_low, stat2_low) {
            (true, true) => ChargingStatus::Standby, // System test mode
            (true, false) => ChargingStatus::Charging,
            (false, true) => ChargingStatus::ChargeComplete,
            (false, false) => ChargingStatus::Standby,
        }
    }
}

#[inline(always)]
pub fn init(c: cortex_m::Peripherals, p: pac::Peripherals) -> (BoardLeds, Voltages, ChargerStatus) {
    let _clocks = Clocks::new(p.CLOCK)
        .enable_ext_hfosc()
        .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk();

    // let port0 = p0::Parts::new(p.P0);
    // let vbat_pin = port0.p0_02.into_floating_input();
    // let vusb_pin = port0.p0_29.into_floating_input();

    let adc = Saadc::new(
        p.SAADC,
        SaadcConfig {
            time: Time::_40US,
            reference: Reference::INTERNAL,
            ..Default::default()
        },
    );

    let voltages = Voltages {
        vbat: InternalVddHdiv5,
        adc,
    };

    let port1 = p1::Parts::new(p.P1);

    let led_red = port1.p1_10.into_push_pull_output(Level::Low);
    let led_green = port1.p1_13.into_push_pull_output(Level::Low);

    let pwm = Pwm::new(p.PWM0);
    pwm.set_period(500u32.hz())
        .set_output_pin(Channel::C0, led_red.degrade())
        .set_output_pin(Channel::C1, led_green.degrade());

    let leds = BoardLeds { pwm };

    let pg = port1.p1_07.into_pullup_input().degrade();
    let stat1 = port1.p1_04.into_pullup_input().degrade();
    let stat2 = port1.p1_02.into_pullup_input().degrade();

    let charger_status = ChargerStatus { pg, stat1, stat2 };

    let systick_token = rtic_monotonics::make_systick_handler!();
    Systick::start(c.SYST, HFCLK_FREQ, systick_token);

    (leds, voltages, charger_status)
}
