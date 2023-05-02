use self::socket_buffer::WriteHandle;

use super::Protocol;
use crate::ssq;
use core::{cell::RefCell, pin::pin};
use futures::future::select;
use heapless::{String, Vec};
use no_std_net::{IpAddr, SocketAddr};

pub trait AsyncWriter {
    /// Write data.
    async fn write(&mut self, buf: &[u8]);
}

pub trait AsyncReadUntilIdle {
    /// Reading needs to be able to run concurrently with writing.
    async fn read_until_idle(&mut self, buf: &mut [u8]) -> usize;
}

// tx.write(b"ATI\r\n").await.unwrap(); // Module name

// tx.write(b"AT+CIMI\r\n").await.unwrap(); // IMSI

// tx.write(b"AT+CGSN\r\n").await.unwrap(); // IMEI

// tx.write(b"AT+UPSD=0,1,\"iot.1nce.net\"\r\n").await.unwrap(); // IMEI

// tx.write(b"AT+UDNSRN=0,\"one.one.one.one\"\r\n").await.unwrap(); // DNS

// tx.write(b"AT+USOCR=6\r\n").await.unwrap();

// tx.write(b"AT+USOCO=0,\"79.136.27.216\",5684\r\n").await.unwrap();

// tx.write(b"AT+USOWR=0,12,\"Hello world!\"\r\n").await.unwrap();

// tx.write(b"AT+USORD=0,0\r\n").await.unwrap(); // Num bytes in socket

/// Helpers to share a buffer between the RX/TX worker and the driver when sending, potentially
/// large, amounts of data over a socket.
///
/// While one can add buffers to the Command/Response queue it means that multiple X of stack space
/// will be used, as the queues work by move. With this abstraction only references are moved.
pub mod socket_buffer {
    use atomic_polyfill::{AtomicU8, Ordering};
    use core::{cell::UnsafeCell, future::poll_fn, task::Poll};
    use rtic_common::waker_registration::CriticalSectionWakerRegistration;

    /// Socket buffer size.
    pub const SOCKET_BUFFER_SIZE: usize = 1024;

    const PING: u8 = 0;
    const PONG: u8 = 1;

    /// Buffer used for moving data between modem and communication worker.
    pub struct StaticPingPongBuffer {
        owner: AtomicU8,
        backing_store: UnsafeCell<BackingStore>,
        waker: CriticalSectionWakerRegistration,
    }

    /// Definition of the data backing the buffer.
    #[derive(Debug, defmt::Format)]
    pub struct BackingStore {
        buffer: [u8; SOCKET_BUFFER_SIZE],
        len: usize,
    }

    impl core::fmt::Debug for StaticPingPongBuffer {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            if self.owner.load(Ordering::Relaxed) == PING {
                write!(f, "StaticPingPongBuffer {{ owner: us, data: ... }}")
            } else {
                write!(f, "StaticPingPongBuffer {{ owner: handle, data: ... }}")
            }
        }
    }

    impl defmt::Format for StaticPingPongBuffer {
        fn format(&self, fmt: defmt::Formatter) {
            if self.owner.load(Ordering::Relaxed) == PING {
                defmt::write!(fmt, "StaticPingPongBuffer {{ owner: us, data: ... }}")
            } else {
                defmt::write!(fmt, "StaticPingPongBuffer {{ owner: handle, data: ... }}")
            }
        }
    }

    impl StaticPingPongBuffer {
        /// Create a new buffer from static storage.
        pub const fn new() -> Self {
            Self {
                owner: AtomicU8::new(PING),
                backing_store: UnsafeCell::new(BackingStore {
                    buffer: [0; SOCKET_BUFFER_SIZE],
                    len: 0,
                }),
                waker: CriticalSectionWakerRegistration::new(),
            }
        }

        /// Create a write handle.
        pub fn write_handle(&self) -> WriteHandle {
            WriteHandle { inner: self }
        }
    }

    /// Read handle of a ping pong buffer.
    #[derive(Debug, defmt::Format)]
    pub struct WriteHandle<'a> {
        inner: &'a StaticPingPongBuffer,
    }

    impl<'a> WriteHandle<'a> {
        /// Access the underlying buffer.
        pub fn access_buffer(&mut self) -> Option<&mut BackingStore> {
            if self.inner.owner.load(Ordering::Relaxed) == PING {
                Some(unsafe { &mut *self.inner.backing_store.get() })
            } else {
                None
            }
        }

        /// Get a read handle from
        pub fn read_handle(&mut self) -> Option<ReadHandle> {
            if self
                .inner
                .owner
                .compare_exchange(PING, PONG, Ordering::SeqCst, Ordering::Relaxed)
                .is_ok()
            {
                Some(ReadHandle { inner: &self.inner })
            } else {
                None
            }
        }

        /// Wait for the read handle to be returned and access the underlying storage.
        pub async fn wait_for_access(&mut self) -> &mut BackingStore {
            poll_fn(|cx| {
                self.inner.waker.register(cx.waker());

                if self.inner.owner.load(Ordering::Relaxed) == PING {
                    Poll::Ready(unsafe { &mut *self.inner.backing_store.get() })
                } else {
                    Poll::Pending
                }
            })
            .await
        }
    }

    /// Read handle of a ping pong buffer.
    #[derive(Debug, defmt::Format)]
    pub struct ReadHandle<'a> {
        inner: &'a StaticPingPongBuffer,
    }

    impl<'a> ReadHandle<'a> {
        /// Access the buffer the read handle points to.
        pub fn access_buffer(&mut self) -> &[u8] {
            let bs = unsafe { &*self.inner.backing_store.get() };
            let len = bs.len;
            &bs.buffer[..len]
        }

        /// Release the read handle.
        pub fn release(self) {
            // Via `drop`
        }
    }

    impl<'a> Drop for ReadHandle<'a> {
        fn drop(&mut self) {
            self.inner.owner.store(PING, Ordering::SeqCst);
            self.inner.waker.wake();
        }
    }
}

// TODO: Needs to be able to be parsed into an AT command string using `at_command`
#[derive(Debug, defmt::Format)]
pub enum Command {
    AllocateSocket,
    DnsLookup {
        host: &'static str, //socket_buffer::ReadHandle<'static>,
    }, // UDNSRN
    ConnectSocket {
        socket_id: u8,
        socket_type: Protocol,
        #[defmt(Debug2Format)]
        addr: SocketAddr,
    }, // USOCO
    SendData {
        socket_id: u8,
        data: socket_buffer::ReadHandle<'static>,
    }, // USOWR
    ReadData {
        socket_id: u8,
    }, // USORD
    DataAvailable {
        socket_id: u8,
    }, // USORD=X,0
}

impl Command {
    /// Converts the command into an AT command string.
    fn to_at<'a>(&self, buf: &'a mut [u8]) -> Result<&'a [u8], ()> {
        match self {
            Command::AllocateSocket => defmt::error!("Not implemented"),
            Command::DnsLookup { host } => {
                // tx.write(b"AT+UDNSRN=0,\"one.one.one.one\"\r\n").await.unwrap(); // DNS
                return Ok(at_commands::builder::CommandBuilder::create_set(buf, false)
                    .named("AT+UDNSRN")
                    .with_int_parameter(0)
                    .with_string_parameter(host)
                    .finish()
                    .unwrap());
            }
            Command::ConnectSocket {
                socket_id,
                socket_type,
                addr,
            } => defmt::error!("Not implemented"),
            Command::SendData { socket_id, data } => defmt::error!("Not implemented"),
            Command::ReadData { socket_id } => defmt::error!("Not implemented"),
            Command::DataAvailable { socket_id } => defmt::error!("Not implemented"),
        }

        Err(())
    }

    fn to_command_hint(&self) -> CommandHint {
        match self {
            Command::AllocateSocket => CommandHint::AllocateSocket,
            Command::DnsLookup { .. } => CommandHint::DnsLookup,
            Command::ConnectSocket { .. } => CommandHint::ConnectSocket,
            Command::SendData { .. } => CommandHint::SendData,
            Command::ReadData { .. } => CommandHint::ReadData,
            Command::DataAvailable { .. } => CommandHint::DataAvailable,
        }
    }
}

// Used in the TX/RX worker to hint from TX to RX what response is expected.
#[derive(Debug, defmt::Format, Copy, Clone, PartialEq, Eq)]
pub enum CommandHint {
    AllocateSocket,
    DnsLookup,
    ConnectSocket,
    SendData,
    ReadData,
    DataAvailable,
}

impl CommandHint {
    /// Converts the hint into a string header.
    fn to_hint(&self) -> String<16> {
        let mut s = String::new();

        match self {
            CommandHint::AllocateSocket => defmt::error!("Not implemented"),
            CommandHint::DnsLookup => {
                s.push_str("+UDNSRN").ok();
            }
            CommandHint::ConnectSocket => defmt::error!("Not implemented"),
            CommandHint::SendData => defmt::error!("Not implemented"),
            CommandHint::ReadData => defmt::error!("Not implemented"),
            CommandHint::DataAvailable => defmt::error!("Not implemented"),
        };

        s
    }

    fn to_timeout_ms(&self) -> u32 {
        match self {
            CommandHint::AllocateSocket => 0,
            CommandHint::DnsLookup => 332_000,
            CommandHint::ConnectSocket => 0,
            CommandHint::SendData => 0,
            CommandHint::ReadData => 0,
            CommandHint::DataAvailable => 0,
        }
    }

    // The buffer holds the string without the hint and the ending `\r\nOK\r\n`
    fn parse(&self, buf: &[u8]) -> Result<(usize, Response), FromAtError> {
        match self {
            CommandHint::AllocateSocket => todo!(),
            CommandHint::DnsLookup => {
                let mut ips = heapless::Vec::new();
                for ip in buf.split(|ch| *ch == ',' as u8) {
                    let ip = trim_ascii(ip);

                    // Each IP string, remove surrounding quotes
                    ips.push(heapless::String::from(
                        core::str::from_utf8(&ip[1..ip.len() - 1]).unwrap(),
                    ))
                    .ok();
                }

                return Ok((buf.len(), Response::DnsLookup { ips }));
            }
            CommandHint::ConnectSocket => todo!(),
            CommandHint::SendData => todo!(),
            CommandHint::ReadData => todo!(),
            CommandHint::DataAvailable => todo!(),
        }
    }
}

// TODO: Needs to be able to be parsed from an AT command string using `at_command`
#[derive(Debug, defmt::Format)]
pub enum Response {
    AllocateSocket {
        id: u8,
    },
    DnsLookup {
        #[defmt(Debug2Format)]
        ips: Vec<heapless::String<15>, 3>,
    },
    ConnectSocket {
        success: bool,
    },
    SendData {
        success: bool,
    },
    ReadData {
        success: bool,
        data: socket_buffer::ReadHandle<'static>,
    },
    DataAvailable {
        num_bytes: usize,
    },
}

// #[derive(Debug, defmt::Format, Copy, Clone, PartialEq, Eq)]
// enum FindSubsequence {
//     /// The needle was not found.
//     NotFound,
//     /// The needle is inside the haystack and starts at this position.
//     Inside(usize),
//     /// The haystack ends with the needle.
//     EndsWith,
// }
//
// fn find_subsequence(haystack: &[u8], needle: &[u8]) -> FindSubsequence {
//     if let Some(pos) = haystack
//         .windows(needle.len())
//         .position(|window| window == needle)
//     {
//         if pos == haystack.len() - needle.len() {
//             FindSubsequence::EndsWith
//         } else {
//             FindSubsequence::Inside(pos)
//         }
//     } else {
//         FindSubsequence::NotFound
//     }
// }

pub const fn trim_ascii_start(mut bytes: &[u8]) -> &[u8] {
    while let [first, rest @ ..] = bytes {
        if first.is_ascii_whitespace() {
            bytes = rest;
        } else {
            break;
        }
    }
    bytes
}

pub const fn trim_ascii_end(mut bytes: &[u8]) -> &[u8] {
    while let [rest @ .., last] = bytes {
        if last.is_ascii_whitespace() {
            bytes = rest;
        } else {
            break;
        }
    }
    bytes
}

pub const fn trim_ascii(bytes: &[u8]) -> &[u8] {
    trim_ascii_end(trim_ascii_start(bytes))
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
pub enum FromAtError {
    ErrorResponse,
    ExpectsMoreData,
    Unknown,
}

impl Response {
    /// Converts the AT string into a response. Returns the response and how much of the buffer
    /// that was used.
    fn from_at(buf: &[u8], hint: CommandHint) -> Result<(usize, Response), FromAtError> {
        // TODO: Error check the message.
        // - Does it start with an error code? (+CME / +CMS)
        // - Does it start with a command/response code?
        // - Does it end with the correct ending, or is there more data to come?

        // TODO
        let hint_str = hint.to_hint();

        let original_length = buf.len();
        let buf = trim_ascii(buf);

        if buf.starts_with(hint_str.as_bytes()) {
            // TODO: Is this overkill?
            // if find_subsequence(buf, b"\r\nOK\r\n") {
            //     // There is a complete command in the buffer
            // }
            // VS this:
            if !buf.ends_with(b"\r\nOK") {
                // More data is expected in this command
                return Err(FromAtError::ExpectsMoreData);
            }

            // Expected command found, remove header including `:` and ending OK, and trim any
            // other whitespace
            let len = buf.len();
            hint.parse(trim_ascii(&buf[hint_str.len() + 1..len - 4]))
        } else if buf.starts_with(b"+CME") {
            defmt::error!("Got CME for {}", hint);
            Err(FromAtError::ErrorResponse)
        } else if buf.starts_with(b"+CMS") {
            defmt::error!("Got CMS for {}", hint);
            Err(FromAtError::ErrorResponse)
        } else {
            // ???
            Err(FromAtError::Unknown)
        }
    }
}

// TODO: Needs to be able to be parsed from an AT command string using `at_command`
#[derive(Debug, defmt::Format, Clone)]
pub enum Unsolicited {
    ToDo,
}

impl Unsolicited {
    /// Converts the AT string into a unsolicited notification. Returns the response and how much
    /// of the buffer that was used.
    fn from_at(buf: &[u8]) -> Result<(usize, Unsolicited), ()> {
        // TODO

        Err(())
    }
}

pub struct Communication<'a, RX, TX> {
    rx: RX,
    tx: TX,
    command_rx: ssq::Receiver<'a, Command>, // 1. We receive commands to send here
    response_tx: ssq::Sender<'a, Response>, // 2. And send the responses back here
    rx_socket_buffer: WriteHandle<'a>,
    notifications: (), // TODO: Place unsolicited messages here? Or talk directly.
}

impl<'a, RX, TX> Communication<'a, RX, TX>
where
    RX: AsyncReadUntilIdle,
    TX: AsyncWriter,
{
    pub fn new(
        rx: RX,
        tx: TX,
        command_rx: ssq::Receiver<'a, Command>,
        response_tx: ssq::Sender<'a, Response>,
        rx_socket_buffer: WriteHandle<'a>,
        notifications: (),
    ) -> Self {
        Self {
            rx,
            tx,
            command_rx,
            response_tx,
            rx_socket_buffer,
            notifications,
        }
    }

    // TODO: Implement the command tx/rx plus unsolicited notifications here
    pub async fn communication_worker(&mut self) {
        let Self {
            rx,
            tx,
            command_rx,
            response_tx,
            rx_socket_buffer, // TODO: Receive SOCKET RX data here
            notifications,
        } = self;

        let cmd_hint: &RefCell<Option<CommandHint>> = &RefCell::new(None);

        // TODO: Refactor the RX and TX block into methods

        let tx_block = async {
            let tx_buf = &mut [0; 1024];

            defmt::debug!("Starting modem TX worker");

            loop {
                let msg = command_rx.receive().await;

                // On command, store the command hint for RX to use
                cmd_hint.replace(Some(msg.to_command_hint()));

                // Generate and send the command
                let to_send = msg.to_at(tx_buf).expect("ICE: Comms worker");

                if let Ok(msg) = core::str::from_utf8(to_send) {
                    defmt::debug!("AT[worker] -> {}", msg);
                } else {
                    defmt::debug!("AT[worker] -> [garbled] {:x}", to_send);
                }

                tx.write(to_send).await;
            }
        };

        let rx_block = async {
            // The RX buffer needs to be quite large, e.g. a band scan response can be huge.
            let rx_buf = &mut [0; 4096];

            defmt::debug!("Starting modem RX worker");

            let mut continue_from = 0;

            loop {
                // TODO: If we did not get a complete message we need to continue filling the
                // partial buffer.

                let len = rx.read_until_idle(&mut rx_buf[continue_from..]).await + continue_from;

                if len == 0 {
                    continue;
                }

                // Trunkate the rx buffer to the message
                let rx_buf = &rx_buf[..len];

                if let Ok(msg) = core::str::from_utf8(rx_buf) {
                    defmt::debug!("AT[worker] <- {}", msg);
                } else {
                    defmt::debug!("AT[worker] <- [garbled] {:x}", rx_buf);
                }

                // TODO: Refactor into something like
                // handle_command(&cmd_hint, rx_buf)

                // If there is a command hint, we are expecting a response from a command.
                let hint = *cmd_hint.borrow_mut();
                if let Some(hint) = hint {
                    match Response::from_at(rx_buf, hint) {
                        Ok((_, resp)) => {
                            // Clear command hint, the response has been found
                            cmd_hint.take();

                            // Guaranteed to succeed
                            assert!(
                                response_tx.try_send(resp).is_none(),
                                "ICE: There was already something in the response queue"
                            );
                            continue_from = 0;
                            continue;

                            // Current assumption: we won't get data so close together so response
                            // and unsolicited will be within the idle timeout.
                            // if used_len < len {
                            //     // Trunkate rx_buf for unsolicited parsing
                            //     rx_buf = &rx_buf[used_len..];

                            //     defmt::warn!(
                            //         "Unhandled buffer (len = {}) from command '{}': {}",
                            //         len - used_len,
                            //         hint,
                            //         core::str::from_utf8(rx_buf).ok()
                            //     );

                            //     // Try to parse unsolicited, fall through to the next block
                            // } else {
                            //     // All of the buffer was used, wait for the next message
                            //     continue;
                            // }
                        }
                        Err(FromAtError::ExpectsMoreData) => {
                            // TODO: What to do if there is an error in parsing? E.g. we did not yet
                            // get the full response for some reason. This commonly happens for
                            // commands that need some time to finish, e.g. a band-scan and DNS
                            // (easy to test with one.one.one.one where responses come 100 ms appart).
                            continue_from = len;
                            defmt::error!("FromAtError, expecting more data, len = {}", len);
                            continue;
                        }
                        Err(e) => {
                            continue_from = 0;
                            defmt::error!("FromAtError: {}", e);
                            continue;
                        }
                    }
                }

                // TODO: Refactor into something like
                // handle_notification(rx_buf)

                // Check if unsolicited notification
                if let Ok((_, resp)) = Unsolicited::from_at(rx_buf) {
                    // TODO: Handle unsolicited notifications

                    // TODO: On `+UUSORD: <socket>,<length>`, wake the correct socket to make
                    // is start reading data or error if it's not used
                }

                defmt::error!("Unhandled RX buffer (len = {})", len,);
            }
        };

        select(pin!(rx_block), pin!(tx_block)).await;

        // TODO: If the futures return, the modem is sleeping
    }
}
