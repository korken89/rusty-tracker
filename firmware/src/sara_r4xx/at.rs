use crate::ssq;
use core::{cell::RefCell, pin::pin};
use futures::future::select;
use heapless::String;

pub trait AsyncWriter {
    /// Write data.
    async fn write(&mut self, buf: &[u8]);
}

pub trait AsyncReadUntilIdle {
    /// Reading needs to be able to run concurrently with writing.
    async fn read_until_idle(&mut self, buf: &mut [u8]) -> usize;
}

// TODO: Needs to be able to be parsed into an AT command string using `at_command`
#[derive(Debug, defmt::Format, Clone)]
pub enum Command {
    ToDo,
}

impl Command {
    /// Converts the command into an AT command string.
    fn to_at(&self, buf: &mut [u8]) -> Result<usize, ()> {
        // TODO

        Err(())
    }

    fn to_command_hint(&self) -> CommandHint {
        match self {
            Command::ToDo => CommandHint::ToDo,
        }
    }
}

// Used in the TX/RX worker to hint from TX to RX what response is expected.
#[derive(Debug, defmt::Format, Copy, Clone, PartialEq, Eq)]
enum CommandHint {
    ToDo,
}

impl CommandHint {
    /// Converts the hint into a string header.
    fn to_hint(&self) -> String<16> {
        let mut s = String::new();

        match self {
            CommandHint::ToDo => s.push_str("+TODO").ok(),
        };

        s
    }
}

// TODO: Needs to be able to be parsed from an AT command string using `at_command`
#[derive(Debug, defmt::Format, Clone)]
pub enum Response {
    ToDo,
}

impl Response {
    /// Converts the AT string into a response.
    fn from_at(buf: &[u8], hint: CommandHint) -> Result<(usize, Response), ()> {
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
    /// Converts the AT string into a unsolicited notification.
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
        let tx_buf = &mut [0; 1024];
        let rx_buf = &mut [0; 1024];

        let tx_block = async {
            loop {
                let msg = command_rx.receive().await;

                // On command, store the command hint for RX to use
                cmd_hint.replace(Some(msg.to_command_hint()));

                // Generate and send the command
                let len = msg.to_at(tx_buf).expect("Comms worker ICE");
                tx.write(&tx_buf[..len]).await;
            }
        };

        let rx_block = async {
            loop {
                let len = rx.read_until_idle(rx_buf).await;

                if len == 0 {
                    continue;
                }

                if let Some(hint) = *cmd_hint.borrow_mut() {
                    if let Ok((len, resp)) = Response::from_at(rx_buf, hint) {
                        // Guaranteed to succeed
                        response_tx.try_send(resp);

                        // Clear command hint, the reponse has been found
                        cmd_hint.take();

                        // Current assumption: we won't get data so close together so response
                        // and unsolicited will be within the idle timeout
                        if len > 0 {
                            defmt::error!(
                                "Unhandled buffer (len = {}) from command '{}': {}",
                                len,
                                hint,
                                core::str::from_utf8(&rx_buf[len..]).ok()
                            );

                            // TODO: If there is data, try to parse unsolicited
                        }

                        continue;
                    }
                }

                // Check if unsolicited notification
                if let Ok((len, resp)) = Unsolicited::from_at(rx_buf) {
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
