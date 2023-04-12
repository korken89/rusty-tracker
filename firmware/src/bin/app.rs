#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

use rtic_monotonics::{nrf::timer::fugit::MicrosDurationU32, systick::Systick, Monotonic};
use rusty_tracker as _; // global logger + panicking-behavior + memory layout

defmt::timestamp!("{=u64:us}", {
    let time_us: MicrosDurationU32 = Systick::now().duration_since_epoch().convert();

    time_us.ticks() as u64
});

pub mod pac {
    // pub use cortex_m_rt::interrupt;
    pub use embassy_nrf::pac::Interrupt as interrupt;
    pub use embassy_nrf::pac::*;
}

#[rtic::app(device = crate::pac, dispatchers = [SWI0_EGU0], peripherals = false)]
mod app {
    use at_commands::parser::CommandParser;
    use embassy_nrf::config::HfclkSource;
    use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
    use embassy_nrf::peripherals::{P0_04, P0_08, PWM0, TIMER0, UARTE0};
    use embassy_nrf::pwm::{Prescaler, SimplePwm};
    use embassy_nrf::saadc::Saadc;
    use embassy_nrf::uarte::{UarteRxWithIdle, UarteTx};
    use embassy_nrf::{bind_interrupts, saadc, uarte};
    use rtic_monotonics::systick::*;
    // use rusty_tracker::bsp::{self, BoardLeds, ChargerStatus, Voltages};

    bind_interrupts!(struct Irqs {
        UARTE0_UART0 => uarte::InterruptHandler<UARTE0>;
        SAADC => saadc::InterruptHandler;
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
        core::mem::forget(buz);

        //
        // Battery measurement
        //
        let mut config = saadc::Config::default();
        config.resolution = saadc::Resolution::_12BIT;
        let mut channel_config = saadc::ChannelConfig::single_ended(saadc::VddhDiv5Input);
        channel_config.time = saadc::Time::_40US;
        channel_config.gain = saadc::Gain::GAIN1_4;
        let saadc = Saadc::new(p.SAADC, Irqs, config, [channel_config]);

        //
        // Setup LED PWM
        //
        let red = p.P1_13;
        let green = p.P0_29;
        let blue = p.P1_10;

        let mut pwm = SimplePwm::new_3ch(p.PWM0, red, green, blue);

        pwm.set_prescaler(Prescaler::Div128);
        pwm.set_max_duty(1000);

        pwm.set_duty(0, 1000);
        pwm.set_duty(1, 1000);
        pwm.set_duty(2, 1000);

        pwm.disable();

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

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 64_000_000, systick_token);

        defmt::println!("init done");

        led_control::spawn(pwm).ok();
        battery_measurement::spawn(saadc).ok();
        modem_talker::spawn(lte_on, lte_pwr, tx).ok();
        modem_listener::spawn(rx).ok();

        (Shared {}, Local {})
    }

    #[task]
    async fn battery_measurement(_: battery_measurement::Context, mut adc: Saadc<'static, 1>) {
        loop {
            let mut buf = [0; 1];
            adc.sample(&mut buf).await;
            let voltage = (buf[0] as f32 / ((1 << 12) as f32 * (5. / 12.))) * 5.;
            defmt::println!("Battery voltage: {} V", voltage);
            Systick::delay(10.secs()).await;
        }
    }

    #[task]
    async fn modem_talker(
        _: modem_talker::Context,
        lte_on: Input<'static, P0_04>,
        mut lte_pwr: Output<'static, P0_08>,
        mut tx: UarteTx<'static, UARTE0>,
    ) {
        'outer: loop {
            defmt::println!("Trying to start LTE modem");
            lte_pwr.set_low();
            Systick::delay(500.millis()).await;
            lte_pwr.set_high();

            for _ in 0..10 {
                if lte_on.is_high() {
                    defmt::println!("LTE modem started!");
                    break 'outer;
                }
                Systick::delay(500.millis()).await;
            }
        }

        defmt::println!("Sending: ATI");
        tx.write(b"ATI\r\n").await.unwrap(); // Module name
        Systick::delay(100.millis()).await;

        defmt::println!("Sending: AT+CIMI");
        tx.write(b"AT+CIMI\r\n").await.unwrap(); // IMSI
        Systick::delay(100.millis()).await;

        defmt::println!("Sending: AT+CGSN");
        tx.write(b"AT+CGSN\r\n").await.unwrap(); // IMEI
        Systick::delay(100.millis()).await;

        defmt::println!("Sending: AT+URAT?");
        tx.write(b"AT+URAT?\r\n").await.unwrap();
        Systick::delay(100.millis()).await;

        // tx.write(b"AT+UBANDMASK?\r\n").await.unwrap(); // IMEI
        // Systick::delay(100.millis()).await;
        // tx.write(b"AT+UMNOPROF?\r\n").await.unwrap(); // IMEI
        // Systick::delay(100.millis()).await;
        // defmt::println!("Sending APN");
        //     tx.write(b"AT+UPSD=0,1,\"iot.1nce.net\"\r\n").await.unwrap(); // IMEI
        // Systick::delay(100.millis()).await;
        // tx.write(b"AT+UPSD=0\r\n").await.unwrap(); // IMEI
        // Systick::delay(100.millis()).await;

        // defmt::println!("Sending request");
        // tx.write(b"AT+UDNSRN=0,\"korken89.duckdns.org\"\r\n")
        //     .await
        //     .unwrap(); // IMEI

        // Systick::delay(3000.millis()).await;

        // // tx.write(b"AT+CSQ?\r\n").await.unwrap(); // IMEI
        // tx.write(b"AT+UCFSCAN=7\r\n").await.unwrap(); // IMEI

        defmt::println!("Sending: AT+USOCR=6");
        tx.write(b"AT+USOCR=6\r\n").await.unwrap();
        Systick::delay(100.millis()).await;

        defmt::println!("Sending: AT+USOCO=0,\"79.136.27.216\",5684");
        tx.write(b"AT+USOCO=0,\"79.136.27.216\",5684\r\n")
            .await
            .unwrap();
        Systick::delay(100.millis()).await;

        loop {
            defmt::println!("Sending: AT+USOWR=0,12,\"Hello world!\"");
            tx.write(b"AT+USOWR=0,12,\"Hello world!\"\r\n")
                .await
                .unwrap();
            Systick::delay(100.millis()).await;

            // Read how many bytes on socket
            defmt::println!("Sending: AT+USORD=0,0");
            tx.write(b"AT+USORD=0,0\r\n").await.unwrap();
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
            defmt::println!(
                "Received {} bytes: {}\n{}",
                r,
                &buf[..r],
                core::str::from_utf8(&buf[..r]).unwrap()
            );
            // Systick::delay(100.millis()).await;
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
            leds.enable();
            // defmt::println!("...");
            leds.set_duty(0, 990);
            leds.set_duty(1, 1000);
            leds.set_duty(2, 1000);
            Systick::delay(200.millis()).await;

            leds.set_duty(0, 1000);
            leds.set_duty(1, 990);
            leds.set_duty(2, 1000);
            Systick::delay(200.millis()).await;

            leds.set_duty(0, 1000);
            leds.set_duty(1, 1000);
            leds.set_duty(2, 990);
            Systick::delay(200.millis()).await;

            leds.set_duty(0, 1000);
            leds.set_duty(1, 1000);
            leds.set_duty(2, 1000);
            leds.disable();
            Systick::delay(1000.millis()).await;
        }
    }
}
