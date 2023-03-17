use crate::hal::{
    clocks::{Clocks, LfOscConfiguration, HFCLK_FREQ},
    gpio::p0,
    pac,
};
use rtic_monotonics::systick::Systick;

#[inline(always)]
pub fn init(c: cortex_m::Peripherals, p: pac::Peripherals) -> () {
    let clocks = Clocks::new(p.CLOCK)
        .enable_ext_hfosc()
        .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk();

    let port0 = p0::Parts::new(p.P0);

    let systick_token = rtic_monotonics::make_systick_handler!();
    Systick::start(c.SYST, HFCLK_FREQ, systick_token);

    ()
}
