#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

use fugit::MicrosDurationU32;
use rtic_monotonics::{systick::Systick, Monotonic};
use rusty_tracker as _; // global logger + panicking-behavior + memory layout

defmt::timestamp!("{=u64:us}", {
    let time_us: MicrosDurationU32 = Systick::now().duration_since_epoch().convert();

    time_us.ticks() as u64
});

#[rtic::app(device = rusty_tracker::hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use rtic_monotonics::systick::*;
    use rusty_tracker::bsp::{self, BoardLeds, Voltages};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        leds: BoardLeds,
        voltages: Voltages,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::println!("pre init");

        let (leds, voltages) = bsp::init(cx.core, cx.device);

        defmt::println!("init");

        task1::spawn().ok();

        (Shared {}, Local { leds, voltages })
    }

    #[task(local = [leds, voltages])]
    async fn task1(cx: task1::Context) {
        loop {
            let vbat = cx.local.voltages.measure_vbat();
            let vusb = cx.local.voltages.measure_vusb();
            defmt::info!("hello, vbat = {} V, vusb = {} V", vbat, vusb);
            Systick::delay(200.millis()).await;
        }
    }
}
