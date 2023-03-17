use crate::hal::{
    clocks::{Clocks, LfOscConfiguration},
    gpio::p0,
    pac,
};

#[inline(always)]
pub fn init(c: cortex_m::Peripherals, p: pac::Peripherals) -> () {
    let _clocks = Clocks::new(p.CLOCK)
        .enable_ext_hfosc()
        .set_lfclk_src_external(LfOscConfiguration::NoExternalNoBypass)
        .start_lfclk();

    let port0 = p0::Parts::new(p.P0);

    ()
}
