//! # Abstration layer for the uBlox SARA-R422-M10S 4G modem.
//!
//! The modem has 2 different kinds of messages that can be received by the host:
//!
//! 1. Response to send commands
//! 2. Unsolicited messages (eg a notification that there is data to read on a socket)
//!
//! Both need to work concurrently.
//!
//!
//!
//!
//!
//!
//! Stuff to enable:
//!
//! - Network registration status +CREG
//!
//!

use arrayvec::ArrayString;
use core::{cell::UnsafeCell, marker::PhantomData, pin::pin, sync::atomic::AtomicBool};
use futures::future::select;
use rtic_sync::arbiter::Arbiter;

pub trait AsyncWriter {
    async fn write(&mut self, buf: &[u8]);
}

pub trait AsyncReadUntilIdle {
    async fn read_until_idle(&mut self, buf: &mut [u8]) -> usize;
}

#[derive(Debug, defmt::Format)]
pub struct Config {}

pub struct AtInterface<RX, TX> {
    rx: RX,
    tx: Arbiter<TX>, // TODO: Maybe have cmd/response as part of the arbiter to link tx to expected return
    notifications: (), // TODO: Place unsolicited messages here
}

impl<RX, TX> AtInterface<RX, TX>
where
    RX: AsyncReadUntilIdle,
    TX: AsyncWriter,
{
    // TODO: Implement the command tx/rx plus unsolicited notifications here
    pub async fn communication_worker(&self) {
        let tx_block = async {
            loop {
                // TODO
            }
        };

        let rx_block = async {
            loop {
                // TODO
            }
        };

        select(pin!(rx_block), pin!(tx_block)).await;
    }
}

pub mod state {
    pub struct Uninitialized;
    // pub struct Initialized;
}

pub struct Socket<const N: usize, T> {
    _0: PhantomData<T>,
}

pub struct R422Modem<RX, TX> {
    // RX has only one accessor at any time so a `UnsafeCell` does the trick,
    // only `init` and the communication worker uses RX and is protected by the atomic bool
    rx: UnsafeCell<RX>,
    // TX is used by all sockets
    tx: Arbiter<TX>,
    // Indicates if initialization is complete, after it is the communication worker owns `self.rx`
    initialized: AtomicBool,
}

impl<RX, TX> R422Modem<RX, TX>
where
    RX: AsyncReadUntilIdle,
    TX: AsyncWriter,
{
    pub fn new(rx: RX, tx: TX) -> Self {
        Self {
            rx: UnsafeCell::new(rx),
            tx: Arbiter::new(tx),
            initialized: AtomicBool::new(false),
        }
    }

    pub async fn init(
        &mut self,
        config: Config,
    ) -> (
        Socket<0, state::Uninitialized>,
        Socket<1, state::Uninitialized>,
        Socket<2, state::Uninitialized>,
        Socket<3, state::Uninitialized>,
        Socket<4, state::Uninitialized>,
        Socket<5, state::Uninitialized>,
    ) {
        // TODO: init modem

        let tx = self.tx.access().await;

        (
            Socket { _0: PhantomData },
            Socket { _0: PhantomData },
            Socket { _0: PhantomData },
            Socket { _0: PhantomData },
            Socket { _0: PhantomData },
            Socket { _0: PhantomData },
        )
    }

    /// Run this worker to make communication work after `init`.
    pub async fn communication_worker(&self) {
        let tx_block = async {
            loop {
                // TODO
            }
        };

        let rx_block = async {
            loop {
                // TODO
            }
        };

        select(pin!(rx_block), pin!(tx_block)).await;
    }
}

#[cfg(test)]
mod tests {
    #[tokio::test]
    async fn it_works() {}
}
