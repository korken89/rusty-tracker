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
    use rusty_tracker::bsp::{self, BoardLeds, ChargerStatus, Voltages};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::println!("pre init");

        let (leds, voltages, charger_status) = bsp::init(cx.core, cx.device);

        defmt::println!("init");

        led_control::spawn(leds, voltages, charger_status).ok();

        (Shared {}, Local {})
    }

    #[task]
    async fn led_control(
        _: led_control::Context,
        leds: BoardLeds,
        voltages: Voltages,
        charger_status: ChargerStatus,
    ) {
        rusty_tracker::tasks::led_control(leds, voltages, charger_status).await;
    }
}