use crate::bsp::{BoardLeds, ChargerStatus, ChargingStatus, Voltages};
use rtic_monotonics::systick::*;

pub async fn led_control(
    mut leds: BoardLeds,
    mut voltages: Voltages,
    charger_status: ChargerStatus,
) -> ! {
    loop {
        let vbat = voltages.measure_vbat();
        let charging_status = charger_status.status();

        defmt::info!("Charging status: {}, Vbat = {} V", charging_status, vbat);

        match charging_status {
            ChargingStatus::No5V => {
                if vbat < 3.5 {
                    leds.levels(1.0, 0.);
                    Systick::delay(10.millis()).await;

                    leds.levels(0., 0.);
                    Systick::delay(5000.millis()).await;

                    continue;
                } else {
                    leds.levels(0., 0.);
                }
            }
            ChargingStatus::Standby => leds.levels(0., 0.),
            ChargingStatus::Charging => {
                for i in 0..100 {
                    leds.levels(0.0, i as f32 / 500.);
                    Systick::delay(2.millis()).await;
                }

                for i in 0..100 {
                    leds.levels(0.0, (100 - i) as f32 / 500.);
                    Systick::delay(2.millis()).await;
                }

                leds.levels(0., 0.);
            }
            ChargingStatus::ChargeComplete => leds.levels(0.0, 0.1),
        }

        Systick::delay(500.millis()).await;
    }
}
