//! A single slot queue.
//!
//! Especially useful for sharing wakers between async HAL drivers and their futures.

use atomic_polyfill::{AtomicBool, Ordering};
use core::{cell::UnsafeCell, future::poll_fn, mem::MaybeUninit, ptr, task::Poll};
use rtic_common::waker_registration::CriticalSectionWakerRegistration;

/// Single slot queue.
pub struct SingleSlotQueue<T> {
    full: AtomicBool,
    val: UnsafeCell<MaybeUninit<T>>,
    sender_waker: CriticalSectionWakerRegistration,
    receiver_waker: CriticalSectionWakerRegistration,
}

impl<T> SingleSlotQueue<T> {
    /// Create a new SSQ.
    pub const fn new() -> Self {
        SingleSlotQueue {
            full: AtomicBool::new(false),
            val: UnsafeCell::new(MaybeUninit::uninit()),
            sender_waker: CriticalSectionWakerRegistration::new(),
            receiver_waker: CriticalSectionWakerRegistration::new(),
        }
    }

    /// Split the queue into producer and consumer.
    pub fn split<'a>(&'a mut self) -> (Sender<'a, T>, Receiver<'a, T>) {
        (Sender { ssq: self }, Receiver { ssq: self })
    }
}

impl<T> Drop for SingleSlotQueue<T> {
    fn drop(&mut self) {
        if self.full.load(Ordering::Relaxed) {
            unsafe {
                ptr::drop_in_place(self.val.get() as *mut T);
            }
        }
    }
}

/// Read handle to a single slot queue.
pub struct Receiver<'a, T> {
    ssq: &'a SingleSlotQueue<T>,
}

impl<'a, T> Receiver<'a, T> {
    /// Receive a value from the queue, waiting until one is available.
    pub async fn receive(&mut self) -> T {
        poll_fn(|cx| {
            // We register the waker here so we don't have a race between waking and try_receive
            self.ssq.receiver_waker.register(cx.waker());

            match self.try_receive() {
                Some(val) => Poll::Ready(val),
                None => Poll::Pending,
            }
        })
        .await
    }

    /// Try reading a value from the queue.
    #[inline]
    pub fn try_receive(&mut self) -> Option<T> {
        if self.ssq.full.load(Ordering::Acquire) {
            let r = Some(unsafe { ptr::read(self.ssq.val.get().cast()) });
            self.ssq.full.store(false, Ordering::Release);

            // Wake if anyone is trying to send more.
            self.ssq.sender_waker.wake();

            r
        } else {
            None
        }
    }

    /// Check if there is a value in the queue.
    #[inline]
    pub fn is_empty(&self) -> bool {
        !self.ssq.full.load(Ordering::Relaxed)
    }
}

/// Safety: We gurarantee the safety using an `AtomicBool` to gate the read of the `UnsafeCell`.
unsafe impl<'a, T> Send for Receiver<'a, T> {}

/// Write handle to a single slot queue.
pub struct Sender<'a, T> {
    ssq: &'a SingleSlotQueue<T>,
}

impl<'a, T> Sender<'a, T> {
    /// Send a value to the queue, waiting until there is space in the queue.
    pub async fn send(&mut self, val: T) {
        // Wait for space in the queue.
        poll_fn(|cx| {
            // We register the waker here so we don't have a race between waking and try_send
            self.ssq.sender_waker.register(cx.waker());

            if self.is_empty() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        // Guaranteed to succeed
        self.try_send(val);
    }

    /// Write a value into the queue. If there is a value already in the queue this will
    /// return the value given to this method.
    #[inline]
    pub fn try_send(&mut self, val: T) -> Option<T> {
        if !self.ssq.full.load(Ordering::Acquire) {
            unsafe { ptr::write(self.ssq.val.get().cast(), val) };
            self.ssq.full.store(true, Ordering::Release);

            // Wake if anyone is trying to receive more.
            self.ssq.receiver_waker.wake();

            None
        } else {
            Some(val)
        }
    }

    /// Check if there is a value in the queue.
    #[inline]
    pub fn is_empty(&self) -> bool {
        !self.ssq.full.load(Ordering::Relaxed)
    }
}

/// Safety: We gurarantee the safety using an `AtomicBool` to gate the write of the
/// `UnsafeCell`.
unsafe impl<'a, T> Send for Sender<'a, T> {}
