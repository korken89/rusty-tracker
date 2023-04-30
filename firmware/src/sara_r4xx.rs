//! # Abstraction layer for the uBlox SARA-R422-M10S 4G modem.
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

use crate::ssq::{self, SingleSlotQueue};
pub use at::{AsyncReadUntilIdle, AsyncWriter};
use atomic_polyfill::{AtomicU8, Ordering};
use heapless::String;
use no_std_net::{IpAddr, SocketAddr};
use rtic_common::waker_registration::CriticalSectionWakerRegistration;
use rtic_sync::arbiter::Arbiter;

mod at;

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
    socket_buffer: Option<at::StaticPingPongBuffer>,
}

mod consts {
    pub const MODEM_UNINITIALIZED: u8 = 0;
    pub const MODEM_INITIALIZING: u8 = 1;
    pub const MODEM_INITIALIZED: u8 = 2;
}

static MODEM_STATE: Arbiter<Option<ModemState>> = Arbiter::new(None);

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
            defmt::debug!("Trying to start LTE modem (try {})", i);
            self.pwr_ctrl.set_low().ok();
            self.delay.delay_ms(500).await;
            self.pwr_ctrl.set_high().ok();

            for _ in 0..10 {
                if matches!(self.v_int.is_high(), Ok(true)) {
                    defmt::debug!("LTE modem started!");
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
    InitializationFailed,
    ResponseNotUtf,
    ResponseNotNumber,
    ResponseTooShort,
}

fn parse_u64(s: &str) -> Result<u64, ()> {
    let mut result = 0;

    for digit in s.chars() {
        if digit >= '0' && digit <= '9' {
            result = 10 * result + digit as u64 - '0' as u64;
        } else {
            return Err(());
        }
    }

    Ok(result)
}

impl Modem {
    /// Helper to be used in `init` for sending a command and getting a response.
    async fn early_command<'a, RX, TX>(
        rxtx: &mut (RX, TX),
        cmd: &str,
        buf: &'a mut [u8],
    ) -> Result<&'a str, ModemInitError>
    where
        RX: at::AsyncReadUntilIdle,
        TX: at::AsyncWriter,
    {
        defmt::debug!("AT[init] -> {}", cmd.trim());
        rxtx.1.write(cmd.as_bytes()).await;
        let len = rxtx.0.read_until_idle(buf).await;

        if buf[..len].ends_with(b"\r\nOK\r\n") {
            // Strip leading `\r\n` and trailing `\r\n\r\nOK\r\n`
            let s = core::str::from_utf8(&buf[..len - 6])
                .map_err(|_| ModemInitError::ResponseNotUtf)?
                .trim();

            defmt::debug!("AT[init] <- {} [OK]", s);

            Ok(s)
        } else {
            let s = core::str::from_utf8(&buf)
                .map_err(|_| ModemInitError::ResponseNotUtf)?
                .trim();
            defmt::debug!("AT[init] <- {} [ERR]", s);
            Err(ModemInitError::InitializationFailed)
        }
    }

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
        let rx_buf = &mut [0; 1024];

        // Mark the driver as under initialization.
        MODEM_INITIALIZED
            .compare_exchange(
                consts::MODEM_UNINITIALIZED,
                consts::MODEM_INITIALIZING,
                Ordering::SeqCst,
                Ordering::SeqCst,
            )
            .map_err(|_| ModemInitError::AlreadyInitialized)?;

        defmt::debug!("Modem: starting modem");

        //
        // Start modem
        //
        io_interface.start_modem().await?;

        //
        // Initialize modem
        //

        defmt::debug!("Modem: initializing");

        // Into minimal functionality
        Self::early_command(&mut at_interface, "AT+CFUN=0\r\n", rx_buf).await?;

        // Close all sockets (if some is open from last run), TODO: Check that this actually works
        Self::early_command(&mut at_interface, "AT+USOCL=0\r\n", rx_buf)
            .await
            .ok();
        Self::early_command(&mut at_interface, "AT+USOCL=1\r\n", rx_buf)
            .await
            .ok();
        Self::early_command(&mut at_interface, "AT+USOCL=2\r\n", rx_buf)
            .await
            .ok();
        Self::early_command(&mut at_interface, "AT+USOCL=3\r\n", rx_buf)
            .await
            .ok();
        Self::early_command(&mut at_interface, "AT+USOCL=4\r\n", rx_buf)
            .await
            .ok();
        Self::early_command(&mut at_interface, "AT+USOCL=5\r\n", rx_buf)
            .await
            .ok();
        Self::early_command(&mut at_interface, "AT+USOCL=6\r\n", rx_buf)
            .await
            .ok();

        // Read IMSI
        let imsi = parse_u64(Self::early_command(&mut at_interface, "AT+CIMI\r\n", rx_buf).await?)
            .map_err(|_| ModemInitError::ResponseNotNumber)?;

        // Read IMEI
        let imei = parse_u64(Self::early_command(&mut at_interface, "AT+CGSN\r\n", rx_buf).await?)
            .map_err(|_| ModemInitError::ResponseNotNumber)?;

        // Read module name
        let model = String::from(Self::early_command(&mut at_interface, "ATI\r\n", rx_buf).await?);

        // Read versions
        let (modem_version, app_version) =
            Self::early_command(&mut at_interface, "ATI9\r\n", rx_buf)
                .await?
                .split_once(',')
                .ok_or(ModemInitError::ResponseTooShort)?;

        let modem_version = String::from(modem_version);
        let application_version = String::from(app_version);

        let system_info = SystemInfo {
            imsi,
            imei,
            model,
            modem_version,
            application_version,
        };

        // TODO: Write config

        // Back to normal functionality
        Self::early_command(&mut at_interface, "AT+CFUN=1\r\n", rx_buf).await?;

        // Create the communication channels
        let (command_tx, command_rx) = unsafe {
            static mut COMMAND_Q: SingleSlotQueue<at::Command> = SingleSlotQueue::new();
            COMMAND_Q.split()
        };

        let (response_tx, response_rx) = unsafe {
            static mut RESPONSE_Q: SingleSlotQueue<at::Response> = SingleSlotQueue::new();
            RESPONSE_Q.split()
        };

        let socket_buffer = unsafe {
            static mut B: [u8; at::SOCKET_BUFFER_SIZE] = [0; at::SOCKET_BUFFER_SIZE];
            Some(at::StaticPingPongBuffer::new(&mut B))
        };

        let modem_state = ModemState {
            command_tx,
            response_rx,
            socket_buffer,
        };

        {
            // Initialize modem state global
            *MODEM_STATE
                .try_access()
                .expect("ICE: Modem state locked before initialization") = Some(modem_state);
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
    pub async fn allocate_socket() -> Result<Socket, ()> {
        if !Self::is_initialized() {
            return Err(());
        }

        let mut modem = MODEM_STATE.access().await;
        let modem = modem.as_mut().expect("ICE: Modem not initialized");

        modem.command_tx.send(at::Command::AllocateSocket).await;
        match modem.response_rx.receive().await {
            at::Response::AllocateSocket { id } => Ok(Socket { id }),
            r => panic!(
                "ICE: The comms worker returned an invalid response: {:?}",
                r
            ),
        }
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
}

impl Socket {
    /// Connect to an address and port.
    pub async fn connect(self, addr: SocketAddr, protocol: Protocol) -> Result<Connection, ()> {
        let mut modem = MODEM_STATE.access().await;
        let modem = modem.as_mut().expect("ICE: Modem not initialized");

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
    // Protocol it is allocated for.
    protocol: Protocol,
}

const NEW_WAKER: CriticalSectionWakerRegistration = CriticalSectionWakerRegistration::new();
static SOCKET_WAKERS: [CriticalSectionWakerRegistration; 7] = [NEW_WAKER; 7];

impl Connection {
    /// Returns the number of bytes available for reading.
    pub async fn bytes_available(&mut self) -> Result<usize, ConnectionError> {
        let modem = MODEM_STATE
            .access()
            .await
            .as_mut()
            .expect("ICE: Modem not initialized");

        todo!()
    }

    /// Reads from the socket, waiting until at least 1 bytes is available.
    pub async fn read(&mut self) -> Result<&[u8], ConnectionError> {
        let modem = MODEM_STATE
            .access()
            .await
            .as_mut()
            .expect("ICE: Modem not initialized");

        todo!()
    }

    /// Writes to the socket, waiting until all bytes are sent.
    pub async fn write(&mut self, buf: &[u8]) -> Result<(), ConnectionError> {
        let modem = MODEM_STATE
            .access()
            .await
            .as_mut()
            .expect("ICE: Modem not initialized");

        let _ = buf;
        todo!()
    }

    /// Close the connection, returning the socket.
    pub async fn close(self) -> Socket {
        let modem = MODEM_STATE
            .access()
            .await
            .as_mut()
            .expect("ICE: Modem not initialized");

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
