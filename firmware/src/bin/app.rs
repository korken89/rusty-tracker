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
    use embassy_nrf::config::HfclkSource;
    use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
    use embassy_nrf::peripherals::{P0_04, P0_08, PWM0, TIMER0, UARTE0};
    use embassy_nrf::pwm::{Prescaler, SimplePwm};
    use embassy_nrf::uarte::{UarteRxWithIdle, UarteTx};
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

        let mut config = embassy_nrf::config::Config::default();
        config.hfclk_source = HfclkSource::ExternalXtal;
        let p = embassy_nrf::init(config);

        //
        // Disable vibrator and buzzer
        //
        let vib = Output::new(p.P0_30, Level::Low, OutputDrive::Standard);
        core::mem::forget(vib);
        let buz = Output::new(p.P0_02, Level::Low, OutputDrive::Standard);
        core::mem::forget(vib);

        //
        // Setup LED PWM
        //
        let red = p.P1_13;
        let green = p.P0_30;
        let blue = p.P1_10;

        let mut pwm = SimplePwm::new_3ch(p.PWM0, red, green, blue);

        pwm.set_prescaler(Prescaler::Div128);
        pwm.set_max_duty(1000);

        // pwm.disable();

        pwm.set_duty(0, 1000);
        pwm.set_duty(1, 1000);
        pwm.set_duty(2, 1000);

        //
        // Charger
        //

        //
        // Button
        //

        //
        // LTE modem connections
        //
        let mut config = uarte::Config::default();
        config.parity = uarte::Parity::EXCLUDED;
        config.baudrate = uarte::Baudrate::BAUD115200;

        let uart = uarte::Uarte::new_with_rtscts(
            p.UARTE0, Irqs,    // ..
            p.P0_06, // TX
            p.P0_13, // RX
            p.P0_26, // CTS
            p.P0_12, // RTS
            config,
        );
        let (tx, rx) = uart.split_with_idle(p.TIMER0, p.PPI_CH0, p.PPI_CH1);

        let reset = Input::new(p.P0_11, Pull::None);
        core::mem::forget(reset);

        let lte_on = Input::new(p.P0_04, Pull::None);
        let lte_pwr = Output::new(p.P0_08, Level::High, OutputDrive::Standard0Disconnect1);

        defmt::info!("uarte initialized!");

        //
        // GPS connections
        //

        let systick_token = rtic_monotonics::make_systick_handler!();
        Systick::start(cx.core.SYST, 64_000_000, systick_token);

        defmt::println!("init done");

        led_control::spawn(pwm).ok();
        modem_talker::spawn(tx).ok();
        modem_listener::spawn(rx).ok();
        modem_status::spawn(lte_on, lte_pwr).ok();

        (Shared {}, Local {})
    }

    #[task]
    async fn modem_status(
        _: modem_status::Context,
        mut lte_on: Input<'static, P0_04>,
        mut lte_pwr: Output<'static, P0_08>,
    ) {
        'outer: loop {
            defmt::println!("Trying to start LTE modem");
            lte_pwr.set_low();
            Systick::delay(500.millis()).await;
            lte_pwr.set_high();

            for _ in 0..10 {
                if lte_on.is_high() {
                    break 'outer;
                }
                Systick::delay(500.millis()).await;
            }
        }

        defmt::println!("LTE modem started!");

        loop {
            Systick::delay(500.millis()).await;
        }
    }

    #[task]
    async fn modem_talker(_: modem_talker::Context, mut tx: UarteTx<'static, UARTE0>) {
        loop {
            defmt::println!("Sending request");
            tx.write(b"AT+CIMI\r\n").await.unwrap();
            Systick::delay(1_000.millis()).await;
        }
    }
    #[task]
    async fn modem_listener(
        _: modem_listener::Context,
        mut rx: UarteRxWithIdle<'static, UARTE0, TIMER0>,
    ) {
        let mut buf = [0; 1024];
        loop {
            let r = rx.read_until_idle(&mut buf).await.unwrap();
            defmt::println!("Received {} bytes: {:x}", r, &buf[..r]);
            if let Ok(s) = core::str::from_utf8(&buf[..r]) {
                defmt::println!("String: {}", s);
            }
            Systick::delay(100.millis()).await;
        }
    }

    #[task]
    async fn led_control(
        _: led_control::Context,
        mut leds: SimplePwm<'static, PWM0>, // leds: BoardLeds,
                                            // voltages: Voltages,
                                            // charger_status: ChargerStatus,
    ) {
        // rusty_tracker::tasks::led_control(leds, voltages, charger_status).await;
        loop {
            defmt::println!("...");
            leds.set_duty(0, 990);
            leds.set_duty(1, 1000);
            Systick::delay(500.millis()).await;
            leds.set_duty(0, 1000);
            leds.set_duty(1, 990);
            Systick::delay(500.millis()).await;
        }
    }
}
