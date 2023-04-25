use crate::{
    bsp::{BoardLeds, ChargerStatus, ChargingStatus, LteComponents, Voltages},
    sara_r4xx,
};
use rtic_monotonics::systick::*;

fn volt_to_rgb(v: f32) -> (f32, f32, f32) {
    let vmax = 4.2;
    let vmin = 3.5;
    let vspan = vmax - vmin;

    let t = (v.min(vmax).max(vmin) - vmin) / vspan;

    if t < 0.5 {
        (1., t / 0.5, 0.)
    } else {
        ((1. - t) / 0.5, 1., 0.)
    }
}

pub async fn led_control(
    mut leds: BoardLeds,
    mut voltages: Voltages,
    charger_status: ChargerStatus,
) -> ! {
    loop {
        let vbat = voltages.measure_vbat().await;
        let charging_status = charger_status.status();

        let (r, g, b) = volt_to_rgb(vbat);

        defmt::info!(
            "Charging status: {}, Vbat = {} V, RGB = {}",
            charging_status,
            vbat,
            (r, g, b)
        );

        match charging_status {
            ChargingStatus::No5V => {
                let s = 0.3;
                leds.levels(s * r, s * g, s * b);
                Systick::delay(15.millis()).await;

                leds.levels(0., 0., 0.);
                Systick::delay(5000.millis()).await;

                continue;
            }
            ChargingStatus::Standby => leds.levels(0., 0., 0.),
            ChargingStatus::Charging => {
                for i in 0..100 {
                    let s = i as f32 / 500.;
                    leds.levels(s * r, s * g, s * b);
                    Systick::delay(2.millis()).await;
                }

                for i in 0..100 {
                    let s = (100 - i) as f32 / 500.;
                    leds.levels(s * r, s * g, s * b);
                    Systick::delay(2.millis()).await;
                }

                leds.levels(0., 0., 0.);
                Systick::delay(200.millis()).await;

                continue;
            }
            ChargingStatus::ChargeComplete => leds.levels(0.0, 0.1, 0.),
        }

        Systick::delay(500.millis()).await;
    }
}

pub async fn modem_worker(lte_components: LteComponents) -> ! {
    // Get the interface to the modem
    let LteComponents {
        io_interface,
        tx,
        rx,
    } = lte_components;

    // Modem configuration
    let config = sara_r4xx::Config { apn: None };

    // Initialize the modem
    let r = sara_r4xx::Modem::init(config, (rx, tx), io_interface).await;

    match r {
        Ok((info, mut comms)) => {
            defmt::info!("Modem init complete, info: {}", info);

            // Start communication worker, this should never return
            comms.communication_worker().await;

            defmt::error!("Modem comms worker returned?!");
        }
        Err(e) => defmt::error!("Modem init failed with error: {}", e),
    }

    loop {
        Systick::delay(500.millis()).await;
    }
}
