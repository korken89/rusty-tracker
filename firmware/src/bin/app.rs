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

pub mod pac {
    pub const NVIC_PRIO_BITS: u8 = 2;
    // pub use cortex_m_rt::interrupt;
    pub use embassy_nrf::pac::Interrupt as interrupt;
    pub use embassy_nrf::pac::*;
}

#[rtic::app(device = crate::pac, dispatchers = [SWI0_EGU0], peripherals = false)]
mod app {
    use embassy_nrf::peripherals::UARTE0;
    use embassy_nrf::pwm::{Prescaler, SimplePwm};
    use embassy_nrf::{bind_interrupts, uarte};
    use rtic_monotonics::systick::*;
    // use rusty_tracker::bsp::{self, BoardLeds, ChargerStatus, Voltages};

    bind_interrupts!(struct Irqs {
        UARTE0_UART0 => uarte::InterruptHandler<UARTE0>;
    });

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::println!("pre init");

        // let (leds, voltages, charger_status) = bsp::init(cx.core, cx.device);

        // defmt::println!("init");

        // led_control::spawn(leds, voltages, charger_status).ok();

        let p = embassy_nrf::init(Default::default());

        let mut pwm = SimplePwm::new_2ch(p.PWM0, p.P1_10, p.P1_13);

        pwm.set_prescaler(Prescaler::Div128);
        pwm.set_max_duty(1000);

        pwm.set_duty(0, 200);
        pwm.set_duty(1, 200);

        // let mut config = uarte::Config::default();
        // config.parity = uarte::Parity::EXCLUDED;
        // config.baudrate = uarte::Baudrate::BAUD115200;

        // let uart = uarte::Uarte::new(p.UARTE0, Irqs, p.P0_08, p.P0_06, config);
        // let (mut tx, mut rx) = uart.split_with_idle(p.TIMER0, p.PPI_CH0, p.PPI_CH1);

        // defmt::info!("uarte initialized!");

        let systick_token = rtic_monotonics::make_systick_handler!();
        Systick::start(cx.core.SYST, 64_000_000, systick_token);

        (Shared {}, Local {})
    }

    #[task]
    async fn led_control(
        _: led_control::Context,
        // leds: BoardLeds,
        // voltages: Voltages,
        // charger_status: ChargerStatus,
    ) {
        // rusty_tracker::tasks::led_control(leds, voltages, charger_status).await;
    }
}
