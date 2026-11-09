//! USBSS host mode (RM §27, EVT `USBSS/HOST`).
//!
//! This is the root-port bring-up only: the PHY and LTSSM are started in
//! downstream mode, the host answers the Port Capability LMPs, and
//! [`HostDriver::wait_connected`] resolves when a SuperSpeed device completes
//! training. Pipe allocation and transfers are not implemented yet; the
//! register flow for those is in the EVT `U30HostCtrlTransfer`.

use core::sync::atomic::Ordering;

use super::{init_clocks, init_host, Instance, LinkInterruptHandler, LinkState, LINK_STATE_UNKNOWN};
use crate::interrupt::typelevel::Interrupt;
use crate::Peri;

/// Root-port link status snapshot (`LINK_STATUS`, RM §27.2.6.5).
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinkStatus {
    pub state: LinkState,
    /// `LINK_STATUS.LINK_RX_TERM_PRES`: receiver termination detected.
    pub term_present: bool,
    /// `LINK_STATUS.LINK_RX_DETECT`: the PHY is in P2 and running RxDetect.
    pub rx_detect: bool,
    /// `LINK_STATUS.LINK_READY`: the link finished initialization in U0.
    pub ready: bool,
    /// `LINK_STATUS.LINK_BUSY`: a `PD_MODE` change is still in progress.
    pub busy: bool,
    /// `LINK_STATUS.LINK_PD_MODE_MASK`: PHY power state, 0 = P0 .. 3 = P3.
    pub pd_mode: u8,
}

/// SuperSpeed host driver.
pub struct HostDriver<'d, T: Instance> {
    _peri: Peri<'d, T>,
}

impl<'d, T: Instance> HostDriver<'d, T> {
    /// Initializes the block in host mode and enables the LINK interrupt.
    ///
    /// Only the LINK interrupt is needed: the transfer interrupt stays off
    /// until pipes are implemented.
    pub fn new(
        peri: Peri<'d, T>,
        _irqs: impl crate::interrupt::typelevel::Binding<T::LinkInterrupt, LinkInterruptHandler<T>> + 'd,
    ) -> Self {
        unsafe {
            init_clocks::<T>();
            init_host::<T>();
            T::LinkInterrupt::enable();
        }

        let state = T::host().link_status().read().link_state();
        T::state().link_state.store(state, Ordering::Relaxed);
        T::state()
            .host_reported_state
            .store(LINK_STATE_UNKNOWN, Ordering::Relaxed);

        Self { _peri: peri }
    }

    /// Current link state.
    pub fn link_state(&self) -> LinkState {
        LinkState::from_raw(T::host().link_status().read().link_state())
    }

    /// Link status snapshot, including termination detection and PHY state.
    pub fn status(&self) -> LinkStatus {
        let status = T::host().link_status().read();
        LinkStatus {
            state: LinkState::from_raw(status.link_state()),
            term_present: status.link_rx_term_pres(),
            rx_detect: status.link_rx_detect(),
            ready: status.link_ready(),
            busy: status.link_busy(),
            pd_mode: status.link_pd_mode_mask(),
        }
    }

    /// Restarts SS.RX_DETECT (RM §27.2.6.2: set `PD_MODE` to P2 first, then
    /// `LINK_GO_RX_DET`, then poll `LINK_RX_TERM_PRES`). The bit self-clears.
    pub fn start_rx_detect(&self) {
        T::host().link_ctrl().modify(|w| w.set_link_go_rx_det(true));
    }

    /// Starts the POLLING handshake once RxDetect has found a receiver
    /// (RM §27.2.6.2, `LINK_POLLING_EN`).
    pub fn start_polling(&self) {
        T::host().link_ctrl().modify(|w| w.set_link_polling_en(true));
    }

    /// A SuperSpeed device finished the Port Configuration handshake.
    pub fn connected(&self) -> bool {
        T::state().connected.load(Ordering::Relaxed)
    }

    /// Waits for the next link-state change.
    pub async fn wait_link_change(&self) -> LinkState {
        core::future::poll_fn(|cx| {
            let state = T::state();
            state.link_waker.register(cx.waker());

            let raw = state.link_state.load(Ordering::Relaxed);
            if raw != state.host_reported_state.load(Ordering::Relaxed) {
                state.host_reported_state.store(raw, Ordering::Relaxed);
                core::task::Poll::Ready(LinkState::from_raw(raw))
            } else {
                core::task::Poll::Pending
            }
        })
        .await
    }

    /// Waits for the next SuperSpeed connection.
    ///
    /// Resolves once per connection, on the Port Configuration handshake,
    /// which the LINK interrupt runs after the LTSSM reaches U0.
    pub async fn wait_connected(&self) {
        core::future::poll_fn(|cx| {
            let state = T::state();
            state.device_waker.register(cx.waker());

            if state.connected.load(Ordering::Relaxed) && !state.connected_reported.swap(true, Ordering::Relaxed) {
                core::task::Poll::Ready(())
            } else {
                core::task::Poll::Pending
            }
        })
        .await
    }
}
