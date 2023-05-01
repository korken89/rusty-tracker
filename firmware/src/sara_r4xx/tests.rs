use super::*;
use core::convert::Infallible;
use std::vec::Vec;
use tokio::sync::mpsc::{channel, Receiver, Sender};

struct FakeTx {
    tx: Sender<Vec<u8>>,
}

impl at::AsyncWriter for FakeTx {
    async fn write(&mut self, buf: &[u8]) {
        println!("Sending: {:x?}", buf);
    }
}

struct FakeRx {
    rx: Receiver<Vec<u8>>,
}

impl at::AsyncReadUntilIdle for FakeRx {
    async fn read_until_idle(&mut self, buf: &mut [u8]) -> usize {
        // todo!()
        0
    }
}

struct FakePowerPin {}

impl embedded_hal::digital::ErrorType for FakePowerPin {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for FakePowerPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

struct FakeResetPin {}

impl embedded_hal::digital::ErrorType for FakeResetPin {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for FakeResetPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

struct FakeOnPin {}

impl embedded_hal::digital::ErrorType for FakeOnPin {
    type Error = Infallible;
}

// The modem is always on
impl embedded_hal::digital::InputPin for FakeOnPin {
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(true)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(false)
    }
}

struct TestDelay {}

impl embedded_hal_async::delay::DelayUs for TestDelay {
    async fn delay_us(&mut self, us: u32) {
        tokio::time::sleep(tokio::time::Duration::from_micros(us.into())).await;
    }

    async fn delay_ms(&mut self, ms: u32) {
        tokio::time::sleep(tokio::time::Duration::from_millis(ms.into())).await;
    }
}

#[tokio::test]
async fn it_works() {
    let config = Config {
        apn: Some("iot.1nce.net"),
    };

    let (tx, rx) = channel(1);

    let (info, mut interface) = Modem::init(
        config,
        (FakeRx { rx }, FakeTx { tx }),
        IoControl {
            pwr_ctrl: FakePowerPin {},
            v_int: FakeOnPin {},
            reset: FakeResetPin {},
            delay: TestDelay {},
        },
    )
    .await
    .unwrap();

    tokio::task::spawn_local(async move { interface.communication_worker().await });

    println!("info: {info:#?}");

    let socket = Modem::allocate_socket().await.unwrap();

    println!("socket: {socket:#?}");

    let conn = socket
        .connect(SocketAddr::from(([127, 0, 0, 1], 443)), Protocol::Tcp)
        .await
        .unwrap();
    // let (rx, tx) = tokio::sync::mpsc::channel(100);
    // loop {}
}
