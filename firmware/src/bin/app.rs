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
    use rusty_tracker::bsp::{BoardLeds, ChargerStatus, LteComponents, Voltages};
    use rusty_tracker::tasks;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("pre init");

        let (leds, voltages, charger_status, lte_components) = rusty_tracker::bsp::init(cx.core);

        led_control::spawn(leds, voltages, charger_status).ok();
        modem_worker::spawn(lte_components).ok();
        modem_test::spawn().ok();

        (Shared {}, Local {})
    }

    #[task]
    async fn modem_test(_: modem_test::Context) {
        tasks::modem_test().await;
    }

    #[task]
    async fn modem_worker(_: modem_worker::Context, lte_components: LteComponents) {
        tasks::modem_worker(lte_components).await;
    }

    #[task]
    async fn led_control(
        _: led_control::Context,
        leds: BoardLeds,
        voltages: Voltages,
        charger_status: ChargerStatus,
    ) {
        tasks::led_control(leds, voltages, charger_status).await;
    }

    // #[task]
    // async fn modem_talker(
    //     _: modem_talker::Context,
    //     lte_on: Input<'static, P0_04>,
    //     mut lte_pwr: Output<'static, P0_08>,
    //     mut tx: UarteTx<'static, UARTE0>,
    // ) {
    //     'outer: loop {
    //         defmt::println!("Trying to start LTE modem");
    //         lte_pwr.set_low();
    //         Systick::delay(500.millis()).await;
    //         lte_pwr.set_high();

    //         for _ in 0..10 {
    //             if lte_on.is_high() {
    //                 defmt::println!("LTE modem started!");
    //                 break 'outer;
    //             }
    //             Systick::delay(500.millis()).await;
    //         }
    //     }

    //     defmt::println!("Sending: ATI");
    //     tx.write(b"ATI\r\n").await.unwrap(); // Module name
    //     Systick::delay(100.millis()).await;

    //     defmt::println!("Sending: AT+CIMI");
    //     tx.write(b"AT+CIMI\r\n").await.unwrap(); // IMSI
    //     Systick::delay(100.millis()).await;

    //     defmt::println!("Sending: AT+CGSN");
    //     tx.write(b"AT+CGSN\r\n").await.unwrap(); // IMEI
    //     Systick::delay(100.millis()).await;

    //     defmt::println!("Sending: AT+URAT?");
    //     tx.write(b"AT+URAT?\r\n").await.unwrap();
    //     Systick::delay(100.millis()).await;

    //     // tx.write(b"AT+UBANDMASK?\r\n").await.unwrap(); // IMEI
    //     // Systick::delay(100.millis()).await;
    //     // tx.write(b"AT+UMNOPROF?\r\n").await.unwrap(); // IMEI
    //     // Systick::delay(100.millis()).await;
    //     // defmt::println!("Sending APN");
    //     //     tx.write(b"AT+UPSD=0,1,\"iot.1nce.net\"\r\n").await.unwrap(); // IMEI
    //     // Systick::delay(100.millis()).await;
    //     // tx.write(b"AT+UPSD=0\r\n").await.unwrap(); // IMEI
    //     // Systick::delay(100.millis()).await;

    //     defmt::println!("Sending DNS");
    //     tx.write(b"AT+UDNSRN=0,\"one.one.one.one\"\r\n")
    //         .await
    //         .unwrap(); // IMEI

    //     Systick::delay(3000.millis()).await;

    //     // // tx.write(b"AT+CSQ?\r\n").await.unwrap(); // IMEI
    //     // tx.write(b"AT+UCFSCAN=7\r\n").await.unwrap(); // IMEI

    //     defmt::println!("Sending: AT+USOCR=6");
    //     tx.write(b"AT+USOCR=6\r\n").await.unwrap();
    //     Systick::delay(100.millis()).await;

    //     defmt::println!("Sending: AT+USOCO=0,\"79.136.27.216\",5684");
    //     tx.write(b"AT+USOCO=0,\"79.136.27.216\",5684\r\n")
    //         .await
    //         .unwrap();
    //     Systick::delay(100.millis()).await;

    //     loop {
    //         defmt::println!("Sending: AT+USOWR=0,12,\"Hello world!\"");
    //         tx.write(b"AT+USOWR=0,12,\"Hello world!\"\r\n")
    //             .await
    //             .unwrap();
    //         Systick::delay(100.millis()).await;

    //         // Read how many bytes on socket
    //         defmt::println!("Sending: AT+USORD=0,0");
    //         tx.write(b"AT+USORD=0,0\r\n").await.unwrap();
    //         Systick::delay(1_000.millis()).await;
    //     }
    // }

    // #[task]
    // async fn modem_listener(
    //     _: modem_listener::Context,
    //     mut rx: UarteRxWithIdle<'static, UARTE0, TIMER0>,
    // ) {
    //     let mut buf = [0; 1024];
    //     loop {
    //         let r = rx.read_until_idle(&mut buf).await.unwrap();
    //         defmt::println!(
    //             "Received {} bytes: {}\n{}",
    //             r,
    //             &buf[..r],
    //             core::str::from_utf8(&buf[..r]).unwrap()
    //         );
    //         // Systick::delay(100.millis()).await;
    //     }
    // }

    // #[task]
    // async fn led_control(
    //     _: led_control::Context,
    //     mut leds: SimplePwm<'static, PWM0>, // leds: BoardLeds,
    //                                         // voltages: Voltages,
    //                                         // charger_status: ChargerStatus,
    // ) {
    //     // rusty_tracker::tasks::led_control(leds, voltages, charger_status).await;
    //     loop {
    //         leds.enable();
    //         // defmt::println!("...");
    //         leds.set_duty(0, 990);
    //         leds.set_duty(1, 1000);
    //         leds.set_duty(2, 1000);
    //         Systick::delay(200.millis()).await;

    //         leds.set_duty(0, 1000);
    //         leds.set_duty(1, 990);
    //         leds.set_duty(2, 1000);
    //         Systick::delay(200.millis()).await;

    //         leds.set_duty(0, 1000);
    //         leds.set_duty(1, 1000);
    //         leds.set_duty(2, 990);
    //         Systick::delay(200.millis()).await;

    //         leds.set_duty(0, 1000);
    //         leds.set_duty(1, 1000);
    //         leds.set_duty(2, 1000);
    //         leds.disable();
    //         Systick::delay(1000.millis()).await;
    //     }
    // }
}
