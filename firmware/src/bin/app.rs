#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

use rusty_tracker as _; // global logger + panicking-behavior + memory layout

// defmt::timestamp!("{=u64:us}", {
//     let time_us: dwm1001_async::rtc_monotonic::fugit::MicrosDurationU64 =
//         app::monotonics::now().duration_since_epoch().convert();
//
//     time_us.ticks()
// });

#[rtic::app(device = rusty_tracker::hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use rusty_tracker::bsp;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let () = bsp::init(cx.core, cx.device);

        (Shared {}, Local {})
    }
}
