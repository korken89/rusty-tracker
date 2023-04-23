use crate::ssq;
use core::{cell::RefCell, pin::pin};
use futures::future::select;
use heapless::{String, Vec};
use no_std_net::{IpAddr, SocketAddr};

use super::Protocol;

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

/// Socket buffer size.
pub const SOCKET_BUFFER_SIZE: usize = 1024;

/// Buffer used for moving data between modem and communication worker.
#[derive(Debug, defmt::Format)]
pub struct StaticBuffer {
    backing_store: &'static mut [u8; SOCKET_BUFFER_SIZE],
    len: usize,
}

impl StaticBuffer {
    pub fn new(backing_store: &'static mut [u8; SOCKET_BUFFER_SIZE]) -> Self {
        Self {
            backing_store,
            len: 0,
        }
    }
}

// static mut BUF: StaticBuffer = {
//     static mut B: [u8; SOCKET_BUFFER_SIZE] = [0; SOCKET_BUFFER_SIZE];
//     StaticBuffer::new(unsafe { &mut B })
// };

// TODO: Needs to be able to be parsed into an AT command string using `at_command`
#[derive(Debug, defmt::Format)]
pub enum Command {
    // ReadModuleName, // ATI0
    // ReadVersions,   // ATI9
    // ReadImsi,       // CIMI
    // ReadImei,       // CGSN
    // SetApn {
    //     apd: StaticBuffer,
    // }, // UPSD
    DnsLookup {
        host: StaticBuffer,
    }, // UDNSRN
    ConnectSocket {
        socket_id: u8,
        socket_type: Protocol,
        #[defmt(Debug2Format)]
        addr: SocketAddr,
    }, // USOCO
    SendData {
        socket_id: u8,
        data: StaticBuffer,
    }, // USOWR
    ReadData {
        socket_id: u8,
        data: StaticBuffer,
    }, // USORD
    DataAvailable {
        socket_id: u8,
    }, // USORD=X,0
}

impl Command {
    /// Converts the command into an AT command string.
    fn to_at(&self, buf: &mut [u8]) -> Result<&[u8], ()> {
        // TODO

        Err(())
    }

    fn to_command_hint(&self) -> CommandHint {
        todo!()
    }
}

// Used in the TX/RX worker to hint from TX to RX what response is expected.
#[derive(Debug, defmt::Format, Copy, Clone, PartialEq, Eq)]
enum CommandHint {
    // ReadModuleName,
    // ReadVersions,
    // ReadImsi,
    // ReadImei,
    // SetApn,
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

        // match self {
        //     CommandHint::ToDo => s.push_str("+TODO").ok(),
        // };

        s
    }

    fn to_timeout_ms(&self) -> u32 {
        todo!()
    }
}

// TODO: Needs to be able to be parsed from an AT command string using `at_command`
#[derive(Debug, defmt::Format)]
pub enum Response {
    // ReadModuleName {
    //     name: String<32>,
    // },
    // ReadVersions {
    //     modem_version: String<16>,
    //     application_version: String<16>,
    // },
    // ReadImsi {
    //     imsi: u64,
    // },
    // ReadImei {
    //     imei: u64,
    // },
    // SetApn {
    //     success: bool,
    //     ret: StaticBuffer,
    // },
    DnsLookup {
        #[defmt(Debug2Format)]
        ips: Vec<IpAddr, 3>,
        ret: StaticBuffer,
    },
    ConnectSocket {
        success: bool,
    },
    SendData {
        success: bool,
        ret: StaticBuffer,
    },
    ReadData {
        success: bool,
        data: StaticBuffer,
    },
    DataAvailable {
        num_bytes: usize,
    },
}

#[derive(Debug, defmt::Format, Copy, Clone, PartialEq, Eq)]
enum FindSubsequence {
    /// The needle was not found.
    NotFound,
    /// The needle is inside the haystack and starts at this position.
    Inside(usize),
    /// The haystack ends with the needle.
    EndsWith,
}

fn find_subsequence(haystack: &[u8], needle: &[u8]) -> FindSubsequence {
    if let Some(pos) = haystack
        .windows(needle.len())
        .position(|window| window == needle)
    {
        if pos == haystack.len() - needle.len() {
            FindSubsequence::EndsWith
        } else {
            FindSubsequence::Inside(pos)
        }
    } else {
        FindSubsequence::NotFound
    }
}

impl Response {
    /// Converts the AT string into a response. Returns the response and how much of the buffer
    /// that was used.
    fn from_at(buf: &[u8], hint: CommandHint) -> Result<(usize, Response), ()> {
        // TODO: Is this overkill?
        // if find_subsequence(buf, b"\r\nOK\r\n") {
        //     // There is a complete command in the buffer
        // }
        // VS this:
        if buf.ends_with(b"\r\nOK\r\n") {
            // There is a complete command in the buffer
        }

        // TODO
        let hint = hint.to_hint();

        Err(())
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
        notifications: (),
    ) -> Self {
        Self {
            rx,
            tx,
            command_rx,
            response_tx,
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
            notifications,
        } = self;

        let cmd_hint: &RefCell<Option<CommandHint>> = &RefCell::new(None);

        // TODO: Refactor the RX and TX block into methods

        let tx_block = async {
            let tx_buf = &mut [0; 1024];
            loop {
                let msg = command_rx.receive().await;

                // On command, store the command hint for RX to use
                cmd_hint.replace(Some(msg.to_command_hint()));

                // Generate and send the command
                let to_send = msg.to_at(tx_buf).expect("ICE: Comms worker");

                if let Ok(msg) = core::str::from_utf8(to_send) {
                    defmt::info!("AT[worker] -> {}", msg);
                } else {
                    defmt::info!("AT[worker] -> [garbled] {:x}", to_send);
                }

                tx.write(to_send).await;
            }
        };

        let rx_block = async {
            // The RX buffer needs to be quite large, e.g. a band scan response can be huge.
            let rx_buf = &mut [0; 4096];

            loop {
                // TODO: If we did not get a complete message we need to move the partial buffer,
                // and continue filling from where we expected more data

                let len = rx.read_until_idle(rx_buf).await;

                if len == 0 {
                    continue;
                }

                // Trunkate the rx buffer to the message
                let mut rx_buf = &rx_buf[..len];

                if let Ok(msg) = core::str::from_utf8(rx_buf) {
                    defmt::info!("AT[worker] <- {}", msg);
                } else {
                    defmt::info!("AT[worker] <- [garbled] {:x}", rx_buf);
                }

                // TODO: Refactor into something like
                // handle_command(&cmd_hint, rx_buf)

                // If there is a command hint, we are expecting a response from a command.
                if let Some(hint) = *cmd_hint.borrow_mut() {
                    if let Ok((used_len, resp)) = Response::from_at(rx_buf, hint) {
                        // Guaranteed to succeed
                        response_tx
                            .try_send(resp)
                            .expect("ICE: There was already something in the response queue");

                        // Clear command hint, the response has been found
                        cmd_hint.take();

                        // Current assumption: we won't get data so close together so response
                        // and unsolicited will be within the idle timeout.
                        if used_len < len {
                            // Trunkate rx_buf for unsolicited parsing
                            rx_buf = &rx_buf[used_len..];

                            defmt::warn!(
                                "Unhandled buffer (len = {}) from command '{}': {}",
                                len - used_len,
                                hint,
                                core::str::from_utf8(rx_buf).ok()
                            );

                            // Try to parse unsolicited, fall through to the next block
                        } else {
                            // All of the buffer was used, wait for the next message
                            continue;
                        }
                    } else {
                        // TODO: What to do if there is an error in parsing? E.g. we did not yet
                        // get the full response for some reason. This commonly happens for
                        // commands that need some time to finish, e.g. a band-scan and DNS
                        // (easy to test with one.one.one.one where responses come 100 ms appart).
                    }
                }

                // TODO: Refactor into something like
                // handle_notification(rx_buf)

                // Check if unsolicited notification
                if let Ok((used_len, resp)) = Unsolicited::from_at(rx_buf) {
                    // TODO: Handle unsolicited notifications

                    // TODO: On `+UUSORD: <socket>,<length>`, wake the correct socket to make
                    // is start reading data or error if it's not used
                }

                defmt::error!(
                    "Unhandled RX buffer (len = {}): {}",
                    len,
                    core::str::from_utf8(&rx_buf[..len]).ok()
                );
            }
        };

        select(pin!(rx_block), pin!(tx_block)).await;

        // TODO: If the futures return, the modem is sleeping
    }
}
