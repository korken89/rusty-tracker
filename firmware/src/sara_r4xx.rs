//! # Abstration layer for the uBlox SARA-R422-M10S 4G modem.
//!
//! The modem has 2 different kinds of messages that can be received by the host:
//!
//! 1. Response to sent commands
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

use core::mem::MaybeUninit;

use atomic_polyfill::{AtomicU8, Ordering};
use heapless::String;
use no_std_net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4};
use rtic_common::waker_registration::CriticalSectionWakerRegistration;
use rtic_sync::arbiter::{Arbiter, ExclusiveAccess};

use crate::ssq::{self, SingleSlotQueue};

pub mod at;

/// Configuration for the modem initialization.
#[derive(Debug, defmt::Format)]
pub struct Config {
    /// APN address.
    pub apn: Option<&'static str>,
    // TODO: More to come
}

/// Protocol selection for sockets.
#[derive(Debug, defmt::Format)]
pub enum Protocol {
    /// TCP socket
    Tcp = 6,
    /// UDP socket
    Udp = 17,
}

/// Different errors that can happen to a connection.
#[derive(Debug, defmt::Format)]
pub enum ConnectionError {
    /// The connection was closed.
    Closed,
    // TODO: more?
}

/// Main modem API.
pub struct Modem {}

struct ModemState {
    command_tx: ssq::Sender<'static, at::Command>,
    response_rx: ssq::Receiver<'static, at::Response>,
}

mod consts {
    pub const MODEM_UNINITIALIZED: u8 = 0;
    pub const MODEM_INITIALIZING: u8 = 1;
    pub const MODEM_INITIALIZED: u8 = 2;
}

static MODEM_STATE: Arbiter<MaybeUninit<ModemState>> = Arbiter::new(MaybeUninit::uninit());

// Check so we have not initialized the modem before.
static MODEM_INITIALIZED: AtomicU8 = AtomicU8::new(consts::MODEM_UNINITIALIZED);

/// I/O Controls of the modem. This includes checking if the module in on and handling the input
/// pins of the module.
pub struct IoControl<LtePowerControl, LteOnCheck, LteReset, Delay> {
    /// Power control pin, this is used to turn the module on or off.
    pub pwr_ctrl: LtePowerControl,
    /// Power on signal, this is used to check if the V_INT has risen.
    pub v_int: LteOnCheck,
    /// Reset signal, used to force reset the modem if needed.
    pub reset: LteReset,
    /// Async delay implementation.
    pub delay: Delay,
}

impl<LtePowerControl, LteOnCheck, LteReset, Delay>
    IoControl<LtePowerControl, LteOnCheck, LteReset, Delay>
where
    LtePowerControl: embedded_hal::digital::OutputPin,
    LteOnCheck: embedded_hal::digital::InputPin,
    LteReset: embedded_hal::digital::OutputPin,
    Delay: embedded_hal_async::delay::DelayUs,
{
    async fn start_modem(&mut self) -> Result<(), ModemInitError> {
        for i in 0..10 {
            defmt::println!("Trying to start LTE modem (try {})", i);
            self.pwr_ctrl.set_low().ok();
            self.delay.delay_ms(500).await;
            self.pwr_ctrl.set_high().ok();

            for _ in 0..10 {
                if matches!(self.v_int.is_high(), Ok(true)) {
                    defmt::println!("LTE modem started!");
                    return Ok(());
                }
                self.delay.delay_ms(500).await;
            }
        }

        Err(ModemInitError::FailedToPowerUp)
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
pub enum ModemInitError {
    AlreadyInitialized,
    FailedToPowerUp,
}

impl Modem {
    /// Initialize the modem according to the configuration.
    ///
    /// Note: This can only be called once.
    pub async fn init<RX, TX, LtePowerControl, LteOnCheck, LteReset, Delay>(
        config: Config,
        mut at_interface: (RX, TX),
        mut io_interface: IoControl<LtePowerControl, LteOnCheck, LteReset, Delay>,
    ) -> Result<(SystemInfo, at::Communication<'static, RX, TX>), ModemInitError>
    where
        RX: at::AsyncReadUntilIdle,
        TX: at::AsyncWriter,
        LtePowerControl: embedded_hal::digital::OutputPin,
        LteOnCheck: embedded_hal::digital::InputPin,
        LteReset: embedded_hal::digital::OutputPin,
        Delay: embedded_hal_async::delay::DelayUs,
    {
        // Mark the driver as under initialization.
        MODEM_INITIALIZED
            .compare_exchange(
                consts::MODEM_UNINITIALIZED,
                consts::MODEM_INITIALIZING,
                Ordering::SeqCst,
                Ordering::SeqCst,
            )
            .map_err(|_| ModemInitError::AlreadyInitialized)?;

        // Start the modem if it is turned off
        io_interface.start_modem().await?;

        //
        // Start modem
        //
        at_interface.1.write(b"AT+CFUN=0\r\n").await;

        //
        // TODO: Initialize modem
        //

        // TODO: Fill system info
        let system_info = SystemInfo {
            imsi: 0x0,
            imei: 0x1,
            model: String::new(),
            modem_version: String::new(),
            application_version: String::new(),
        };

        // Create the communication channels
        let (command_tx, command_rx) = unsafe {
            static mut COMMAND_Q: SingleSlotQueue<at::Command> = SingleSlotQueue::new();
            COMMAND_Q.split()
        };

        let (response_tx, response_rx) = unsafe {
            static mut RESPONSE_Q: SingleSlotQueue<at::Response> = SingleSlotQueue::new();
            RESPONSE_Q.split()
        };

        let modem_state = ModemState {
            command_tx,
            response_rx,
        };

        {
            // Initialize modem state global
            MODEM_STATE
                .try_access()
                .expect("ICE: Modem state had access before initialization")
                .write(modem_state);
        }

        // Create communication interface
        let at_comms =
            at::Communication::new(at_interface.0, at_interface.1, command_rx, response_tx, ());

        // Mark the modem as initialized
        MODEM_INITIALIZED.store(consts::MODEM_INITIALIZED, Ordering::SeqCst);

        Ok((system_info, at_comms))
    }

    /// Checks if the modem is initialized.
    pub fn is_initialized() -> bool {
        MODEM_INITIALIZED.load(Ordering::Relaxed) == consts::MODEM_INITIALIZED
    }

    /// Allocate a socket from the modem, the maximum concurrent sockets supported is 7.
    pub async fn allocate_socket(protocol: Protocol) -> Result<Socket, ()> {
        if !Self::is_initialized() {
            return Err(());
        }

        todo!()
    }

    /// DNS Lookup a hostname or address.
    pub async fn dns_lookup(hostname: &'static str) -> Result<heapless::Vec<IpAddr, 3>, ()> {
        if !Self::is_initialized() {
            return Err(());
        }

        todo!()
    }

    // TODO: Other functions
}

/// Represents a socket handle.
#[derive(Debug, defmt::Format)]
pub struct Socket {
    // The ID (0-6) given from the modem on creation.
    id: u8,
    // Protocol it is allocated for.
    protocol: Protocol,
}

impl Socket {
    /// Connect to an address and port.
    pub async fn connect(self, addr: SocketAddr) -> Result<Connection, ()> {
        // SAFETY: The `MaybeUninit` is initialized, it's the only way to get the `Socket` type.
        let modem = unsafe { MODEM_STATE.access().await.assume_init_mut() };

        todo!()
    }

    /// Release the socket, making it available for allocation again.
    pub async fn release(self) {
        todo!()
    }

    /// Enable TLS.
    pub async fn enable_tls(/* TLS config? */) {
        todo!()
    }
}

impl Drop for Socket {
    fn drop(&mut self) {
        // TODO: release the socket by setting a flag or something as this can't be async
        todo!()
    }
}

/// Represents a connection made from a socket handle.
#[derive(Debug, defmt::Format)]
pub struct Connection {
    socket: Socket,
    // TODO
}

const NEW_WAKER: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();
static SOCKET_WAKERS: [CriticalSectionWakerRegistration; 7] = [NEW_WAKER; 7];

impl Connection {
    /// Returns the number of bytes available for reading.
    pub async fn bytes_available(&mut self) -> Result<usize, ConnectionError> {
        // SAFETY: The `MaybeUninit` is initialized, it's the only way to get the `Connection` type.
        let modem = unsafe { MODEM_STATE.access().await.assume_init_mut() };

        modem.

        todo!()
    }

    /// Reads from the socket, waiting until at least 1 bytes is available.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ConnectionError> {
        // SAFETY: The `MaybeUninit` is initialized, it's the only way to get the `Connection` type.
        let modem = unsafe { MODEM_STATE.access().await.assume_init_mut() };

        let _ = buf;
        todo!()
    }

    /// Writes to the socket, waiting until all bytes are sent.
    pub async fn write(&mut self, buf: &[u8]) -> Result<(), ConnectionError> {
        // SAFETY: The `MaybeUninit` is initialized, it's the only way to get the `Connection` type.
        let modem = unsafe { MODEM_STATE.access().await.assume_init_mut() };

        let _ = buf;
        todo!()
    }

    /// Close the connection, returning the socket.
    pub async fn close(self) -> Socket {
        // SAFETY: The `MaybeUninit` is initialized, it's the only way to get the `Connection` type.
        let modem = unsafe { MODEM_STATE.access().await.assume_init_mut() };

        todo!()
    }
}

/// System information returned from `init`.
#[derive(Debug, defmt::Format, Clone)]
pub struct SystemInfo {
    /// IMSI number
    pub imsi: u64,
    /// IMEI number
    pub imei: u64,
    /// Model info
    pub model: String<32>,
    /// Modem version
    pub modem_version: String<16>,
    /// Application version
    pub application_version: String<16>,
}

pub async fn api_test() {}

#[cfg(test)]
mod tests;
