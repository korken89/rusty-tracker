use crate::hal::{
    clocks::{Clocks, LfOscConfiguration, HFCLK_FREQ},
    gpio::{
        p0::{self, P0_02, P0_29},
        p1::{self, P1_10, P1_13},
        Floating, Input, Level, Output, PushPull,
    },
    pac,
    prelude::*,
    saadc::{Reference, SaadcConfig, Time},
    Saadc,
};
use rtic_monotonics::systick::Systick;

pub struct BoardLeds {
    red: P1_10<Output<PushPull>>,
    green: P1_13<Output<PushPull>>,
}

impl BoardLeds {
    pub fn green(&mut self, level: bool) {
        self.green.set_state(level.into()).ok();
    }

    pub fn red(&mut self, level: bool) {
        self.red.set_state(level.into()).ok();
    }
}

pub struct Voltages {
    vbat: P0_02<Input<Floating>>,
    vusb: P0_29<Input<Floating>>,
    adc: Saadc,
}

impl Voltages {
    pub fn measure_vbat(&mut self) -> f32 {
        if let Ok(v) = self.adc.read(&mut self.vbat) {
            // VP = (RES / (2^13)) * 2 * (5/12)

            let res = v as f32;
            let vp = (res / ((1 << 14) as f32 * (5. / 12.))) * 3.2;

            vp
        } else {
            f32::NEG_INFINITY
        }
    }

    pub fn measure_vusb(&mut self) -> f32 {
        if let Ok(v) = self.adc.read(&mut self.vusb) {
            // VP = (RES / (2^13)) * 2 * (5/12)

            let res = v as f32;
            let vp = (res / ((1 << 14) as f32 * (5. / 12.))) * 3.2;

            vp
        } else {
            f32::NEG_INFINITY
        }
    }
}

#[inline(always)]
pub fn init(c: cortex_m::Peripherals, p: pac::Peripherals) -> (BoardLeds, Voltages) {
    let _clocks = Clocks::new(p.CLOCK)
        .enable_ext_hfosc()
        .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk();

    let port0 = p0::Parts::new(p.P0);

    let vbat_pin = port0.p0_02.into_floating_input();
    let vusb_pin = port0.p0_29.into_floating_input();
    let adc = Saadc::new(
        p.SAADC,
        SaadcConfig {
            time: Time::_40US,
            reference: Reference::INTERNAL,
            ..Default::default()
        },
    );

    let voltages = Voltages {
        vbat: vbat_pin,
        vusb: vusb_pin,
        adc,
    };

    let port1 = p1::Parts::new(p.P1);

    let led_red = port1.p1_10.into_push_pull_output(Level::Low);
    let led_green = port1.p1_13.into_push_pull_output(Level::Low);

    let leds = BoardLeds {
        red: led_red,
        green: led_green,
    };

    let systick_token = rtic_monotonics::make_systick_handler!();
    Systick::start(c.SYST, HFCLK_FREQ, systick_token);

    (leds, voltages)
}
