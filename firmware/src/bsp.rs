use embassy_nrf::{
    config::HfclkSource,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    peripherals::{P0_04, P0_08, P0_11, P1_02, P1_04, P1_07, PWM0, TIMER0, UARTE0},
    pwm::{Prescaler, SimplePwm},
    saadc::Saadc,
    uarte::{UarteRxWithIdle, UarteTx},
    {bind_interrupts, saadc, uarte},
};
use rtic_monotonics::systick::Systick;

use crate::sara_r4xx::{AsyncReadUntilIdle, AsyncWriter, IoControl};

pub struct BoardLeds {
    pwm: SimplePwm<'static, PWM0>,
}

impl BoardLeds {
    pub fn levels(&mut self, red: f32, green: f32, blue: f32) {
        let max_duty = 1000;

        if red > 0. || green > 0. || blue > 0. {
            let red_level = red.min(1.).max(0.) * max_duty as f32 + 0.5;
            let green_level = green.min(1.).max(0.) * max_duty as f32 + 0.5;
            let blue_level = blue.min(1.).max(0.) * max_duty as f32 + 0.5;

            self.pwm.enable();
            self.pwm.set_duty(0, max_duty - red_level as u16);
            self.pwm.set_duty(1, max_duty - green_level as u16);
            self.pwm.set_duty(2, max_duty - blue_level as u16);
        } else {
            self.pwm.disable();
        }
    }
}

pub struct Voltages {
    adc: Saadc<'static, 1>,
}

impl Voltages {
    pub async fn measure_vbat(&mut self) -> f32 {
        let mut buf = [0; 1];
        self.adc.sample(&mut buf).await;

        (buf[0] as f32 / ((1 << 12) as f32 * (5. / 12.))) * 5.
    }
}

pub struct ChargerStatus {
    pg: Input<'static, P1_07>,
    stat1: Input<'static, P1_04>,
    stat2: Input<'static, P1_02>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, defmt::Format, Hash)]
pub enum ChargingStatus {
    /// There is no cable connected.
    No5V,
    /// The charger is powered, but not charging.
    Standby,
    /// The battery is charging.
    Charging,
    // The charging has finished.
    ChargeComplete,
}

impl ChargerStatus {
    pub fn usb_connected(&self) -> bool {
        self.pg.is_low()
    }

    //
    pub fn status(&self) -> ChargingStatus {
        if !self.usb_connected() {
            return ChargingStatus::No5V;
        }

        let (stat1_low, stat2_low) = (self.stat1.is_low(), self.stat2.is_low());

        match (stat1_low, stat2_low) {
            (true, true) => ChargingStatus::Standby, // System test mode
            (true, false) => ChargingStatus::Charging,
            (false, true) => ChargingStatus::ChargeComplete,
            (false, false) => ChargingStatus::Standby,
        }
    }
}

pub struct UTx {
    pub tx: UarteTx<'static, UARTE0>,
}

impl AsyncWriter for UTx {
    async fn write(&mut self, buf: &[u8]) {
        self.tx.write(buf).await.expect("ICE: Write")
    }
}

pub struct URx {
    pub rx: UarteRxWithIdle<'static, UARTE0, TIMER0>,
}

impl AsyncReadUntilIdle for URx {
    async fn read_until_idle(&mut self, buf: &mut [u8]) -> usize {
        self.rx.read_until_idle(buf).await.expect("ICE: Read")
    }
}

pub struct LteComponents {
    pub io_interface:
        IoControl<Output<'static, P0_08>, Input<'static, P0_04>, Output<'static, P0_11>, Systick>,
    pub tx: UTx,
    pub rx: URx,
}

bind_interrupts!(struct Irqs {
    UARTE0_UART0 => uarte::InterruptHandler<UARTE0>;
    SAADC => saadc::InterruptHandler;
});

#[inline(always)]
pub fn init(c: cortex_m::Peripherals) -> (BoardLeds, Voltages, ChargerStatus, LteComponents) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    config.dcdc.reg0 = true;
    config.dcdc.reg1 = true;
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

    let voltages = Voltages { adc: saadc };

    //
    // Setup LED PWM
    //
    let red = p.P1_13;
    let green = p.P0_29;
    let blue = p.P1_10;

    let mut pwm = SimplePwm::new_3ch(p.PWM0, red, green, blue);

    pwm.set_prescaler(Prescaler::Div1);
    pwm.set_max_duty(1000);

    pwm.set_duty(0, 1000);
    pwm.set_duty(1, 1000);
    pwm.set_duty(2, 1000);

    let leds = BoardLeds { pwm };

    //
    // Charger
    //

    let pg = Input::new(p.P1_07, Pull::Up);
    let stat1 = Input::new(p.P1_04, Pull::Up);
    let stat2 = Input::new(p.P1_02, Pull::Up);
    let charger_status = ChargerStatus { pg, stat1, stat2 };

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

    let lte_reset = Output::new(p.P0_11, Level::High, OutputDrive::Standard0Disconnect1);
    let lte_on = Input::new(p.P0_04, Pull::None);

    // LTE Power needs to be open drain
    let lte_pwr = Output::new(p.P0_08, Level::High, OutputDrive::Standard0Disconnect1);

    let io_interface = IoControl {
        pwr_ctrl: lte_pwr,
        v_int: lte_on,
        reset: lte_reset,
        delay: Systick {},
    };
    let lte_components = LteComponents {
        io_interface,
        tx: UTx { tx },
        rx: URx { rx },
    };

    defmt::info!("uarte initialized!");

    //
    // GPS connections
    //

    let systick_token = rtic_monotonics::create_systick_token!();
    Systick::start(c.SYST, 64_000_000, systick_token);
    defmt::info!("init done");

    (leds, voltages, charger_status, lte_components)
}
