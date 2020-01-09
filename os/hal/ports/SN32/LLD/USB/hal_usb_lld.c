/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_usb_lld.c
 * @brief   PLATFORM USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */
#include <SN32F240B.h>
#include "hal.h"
#include "usb.h"
#include "usbhw.h"
#include "usbuser.h"
#include "usbepfunc.h"
#include "usbram.h"

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define SN32_USB_IRQ_VECTOR    Vector44
#define SN32_USB_PMA_SIZE      256

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if (PLATFORM_USB_USE_USB1 == TRUE) || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

// uint8_t *const usbd_sram = (uint8_t *)(SN_USB_BASE + 0x100);

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static union {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
  1,
  ep0setup_buffer
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Common ISR code, serves the EP-related interrupts.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void usb_serve_endpoints(USBDriver *usbp, uint32_t ep) {
    size_t                   n;
    uint32_t                 iwIntFlag = SN_USB->INSTS;
    const USBEndpointConfig *epcp = usbp->epc[ep];

    if (iwIntFlag & mskEP0_IN) {
        /* IN endpoint, transmission.*/
        USBInEndpointState *isp = epcp->in_state;

        // EPR_CLEAR_CTR_TX(ep);
        __USB_CLRINSTS((mskEP0_SETUP|mskEP0_PRESETUP|mskEP0_OUT_STALL|mskEP0_IN_STALL));

        isp->txcnt += isp->txlast;
        n = isp->txsize - isp->txcnt;
        if (n > 0) {
            /* Transfer not completed, there are more packets to send.*/
            if (n > epcp->in_maxsize) n = epcp->in_maxsize;

            /* Writes the packet from the defined buffer.*/
            isp->txbuf += isp->txlast;
            isp->txlast = n;
            // usb_packet_write_from_buffer(ep, isp->txbuf, n);

            /* Starting IN operation.*/
            // EPR_SET_STAT_TX(ep, EPR_STAT_TX_VALID);
        } else {
            /* Transfer completed, invokes the callback.*/
            _usb_isr_invoke_in_cb(usbp, ep);
        }
    }
    if (iwIntFlag & mskEP0_OUT) {
        /* OUT endpoint, receive.*/

        // EPR_CLEAR_CTR_RX(ep);
        __USB_CLRINSTS(mskEP0_IN);

        if (iwIntFlag & mskEP0_SETUP) {
            /* Setup packets handling, setup packets are handled using a
               specific callback.*/
            _usb_isr_invoke_setup_cb(usbp, ep);
        } else {
            USBOutEndpointState *osp = epcp->out_state;

            /* Reads the packet into the defined buffer.*/
            // n = usb_packet_read_to_buffer(ep, osp->rxbuf);
            osp->rxbuf += n;

            /* Transaction data updated.*/
            osp->rxcnt += n;
            osp->rxsize -= n;
            osp->rxpkts -= 1;

            /* The transaction is completed if the specified number of packets
               has been received or the current packet is a short packet.*/
            if ((n < epcp->out_maxsize) || (osp->rxpkts == 0)) {
                /* Transfer complete, invokes the callback.*/
                _usb_isr_invoke_out_cb(usbp, ep);
            } else {
                /* Transfer not complete, there are more packets to receive.*/
                EPR_SET_STAT_RX(ep, EPR_STAT_RX_VALID);
            }
        }
    }
}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

OSAL_IRQ_HANDLER(SN32_USB_IRQ_VECTOR) {
    uint32_t   iwIntFlag;
    USBDriver *usbp = &USBD1;

    OSAL_IRQ_PROLOGUE();

	//** Get Interrupt Status and clear immediately.
	iwIntFlag = SN_USB->INSTS;
	SN_USB->INSTSC = 0xFEFBFFFF;	//** Don't clear PRESETUP & ERR_SETUP flag

	if(iwIntFlag == 0)
	{
		//@20160902 add for EMC protection
		USB_ReturntoNormal();
		return;
	}

	/////////////////////////////////////////////////
	/* Device Status Interrupt (BusReset, Suspend) */
	/////////////////////////////////////////////////
	if (iwIntFlag & (mskBUS_RESET | mskBUS_SUSPEND | mskBUS_RESUME))
	{
		if (iwIntFlag & mskBUS_RESET)
		{
			/* BusReset */
			USB_ReturntoNormal();
			USB_ResetEvent();
            _usb_reset(usbp);
		}
		else if (iwIntFlag & mskBUS_SUSPEND)
		{
			/* Suspend */
			USB_SuspendEvent();
            _usb_suspend(usbp);
		}
		else if(iwIntFlag & mskBUS_RESUME)
		{
			/* Resume */
			USB_ReturntoNormal();
			USB_ResumeEvent();
            _usb_wakeup(usbp);
		}
	}
	/////////////////////////////////////////////////
	/* Device Status Interrupt (SETUP, IN, OUT) 	 */
	/////////////////////////////////////////////////
	else if (iwIntFlag & (mskEP0_SETUP|mskEP0_IN|mskEP0_OUT|mskEP0_IN_STALL|mskEP0_OUT_STALL))
	{
		if (iwIntFlag & mskEP0_SETUP)
		{
			/* SETUP */
			USB_EP0SetupEvent();
		}
		else if (iwIntFlag & mskEP0_IN)
		{
			/* IN */
			USB_EP0InEvent();
		}
		else if (iwIntFlag & mskEP0_OUT)
		{
			/* OUT */
			USB_EP0OutEvent();
		}
		else if (iwIntFlag & (mskEP0_IN_STALL|mskEP0_OUT_STALL))
		{
			/* EP0_IN_OUT_STALL */
			SN_USB->INSTSC = (mskEP0_IN_STALL|mskEP0_OUT_STALL);
			USB_EPnStall(USB_EP0);
		}
	}
	/////////////////////////////////////////////////
	/* Device Status Interrupt (EPnACK) 					 */
	/////////////////////////////////////////////////
	else if (iwIntFlag & (mskEP4_ACK|mskEP3_ACK|mskEP2_ACK|mskEP1_ACK))
	{
		if (iwIntFlag & mskEP1_ACK)
		{
			/* EP1 ACK */
			USB_EP1AckEvent();
		}
		if (iwIntFlag & mskEP2_ACK)
		{
			/* EP2 ACK */
			USB_EP2AckEvent();
		}
		if (iwIntFlag & mskEP3_ACK)
		{
			/* EP3 ACK */
			USB_EP3AckEvent();
		}
		if (iwIntFlag & mskEP4_ACK)
		{
			/* EP4 ACK */
			USB_EP4AckEvent();
		}

    OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {
  /* Driver initialization.*/
  usbObjectInit(&USBD1);
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {

    if (usbp->state == USB_STOP) {
        /* Enables the peripheral.*/
    #if PLATFORM_USB_USE_USB1 == TRUE
        if (&USBD1 == usbp) {
            USB_Init();
            nvicEnableVector(USB_IRQn, 5);
        }
    #endif
        // usb_lld_reset(usbp);
  }
  /* Configures the peripheral.*/
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {
  if (usbp->state == USB_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if PLATFORM_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {
        nvicDisableVector(USB_IRQn);
        SN_SYS1->AHBCLKEN = (0x0 << 4);  // Disable USBCLKEN
    }
#endif
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {/
    /* EP0 initialization.*/
    usbp->epc[0] = &ep0config;
    usb_lld_init_endpoint(usbp, 0);
    SN_USB->ADDR = 0;
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
    SN_USB->ADDR = (uint32_t)(usbp->address);
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
    unsigned i;

    /* Disabling all endpoints.*/
    for (i = 1; i <= USB_ENDOPOINTS_NUMBER; i++) {
        // EPR_TOGGLE(i, 0);
        USB_ClrEPnToggle(i);
        // EPR_SET(i, 0);
    }
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  switch (SN_USB->INSTS) {
      case mskEP0_IN:
          return EP_STATUS_DISABLED;
      case mskEP0_IN_STALL:
          return EP_STATUS_STALLED;
      default:
          return EP_STATUS_ACTIVE;
  }
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  switch (SN_USB->INSTS) {
    case mskEP0_OUT:
      return EP_STATUS_DISABLED;
    case mskEP0_OUT_STALL:
      return EP_STATUS_STALLED;
    default:
      return EP_STATUS_ACTIVE;
  }
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
    (void)usbp;
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
