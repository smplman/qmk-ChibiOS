/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
#include "usbdesc.h"

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
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_ENDPOINT_TYPE_CONTROL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out
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
  size_t n;
  uint32_t usbCfg = SN_USB->CFG;
  uint32_t iwIntFlag = SN_USB->INSTS;
  const USBEndpointConfig *epcp = usbp->epc[ep];

  if (~(usbCfg>>(ep-1)) & mskEP1_DIR) {
    /* IN endpoint, transmission.*/
    USBInEndpointState *isp = epcp->in_state;

    // EPR_CLEAR_CTR_TX(ep);

    isp->txcnt += isp->txlast;
    n = isp->txsize - isp->txcnt;
    if (n > 0) {
      /* Transfer not completed, there are more packets to send.*/
      if (n > epcp->in_maxsize)
        n = epcp->in_maxsize;

      /* Writes the packet from the defined buffer.*/
      isp->txbuf += isp->txlast;
      isp->txlast = n;
    //   usb_packet_write_from_buffer(ep, isp->txbuf, n);
      fnUSBINT_WriteFIFO((uint32_t)*isp->txbuf, n);
      SN_USB->RWADDR = 0x00;//(uint32_t)*isp->txbuf;
      SN_USB->RWDATA = (uint32_t)*isp->txbuf;
      SN_USB->RWSTATUS = 0x01;
      while (SN_USB->RWSTATUS &0x01);

      /* Starting IN operation.*/
    //   EPR_SET_STAT_TX(ep, EPR_STAT_TX_VALID);
    }
    else {
      /* Transfer completed, invokes the callback.*/
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  }
  if (usbCfg>>(ep-1) & mskEP1_DIR) {
    /* OUT endpoint, receive.*/

    // EPR_CLEAR_CTR_RX(ep);

    if (iwIntFlag & mskEP0_SETUP) {
      /* Setup packets handling, setup packets are handled using a
         specific callback.*/
      _usb_isr_invoke_setup_cb(usbp, ep);
    }
    else {
      USBOutEndpointState *osp = epcp->out_state;

      /* Reads the packet into the defined buffer.*/
    //   n = usb_packet_read_to_buffer(ep, osp->rxbuf);
    //   fnUSBINT_ReadFIFO((uint32_t)*osp->rxbuf);
      SN_USB->RWADDR = (uint32_t)*osp->rxbuf;
      SN_USB->RWSTATUS = 0x02;
      while (SN_USB->RWSTATUS &0x02);
      wUSBINT_ReadDataBuf = SN_USB->RWDATA;

      osp->rxbuf += wUSBINT_ReadDataBuf;

      /* Transaction data updated.*/
      osp->rxcnt  += wUSBINT_ReadDataBuf;
      osp->rxsize -= wUSBINT_ReadDataBuf;
      osp->rxpkts -= 1;

      /* The transaction is completed if the specified number of packets
         has been received or the current packet is a short packet.*/
      if ((wUSBINT_ReadDataBuf < epcp->out_maxsize) || (osp->rxpkts == 0)) {
        /* Transfer complete, invokes the callback.*/
        _usb_isr_invoke_out_cb(usbp, ep);
      }
      else {
        /* Transfer not complete, there are more packets to receive.*/
        // EPR_SET_STAT_RX(ep, EPR_STAT_RX_VALID);
      }
    }
  }
}

/**
 * @brief   USB shared ISR.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
static void usb_lld_serve_interrupt(USBDriver *usbp) {
	uint32_t iwIntFlag;

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
			// USB_EP0SetupEvent();
            usb_serve_endpoints(usbp, 0);
            // _usb_isr_invoke_setup_cb(usbp, 0);
		}
		else if (iwIntFlag & mskEP0_IN)
		{
			/* IN */
			// USB_EP0InEvent();
            usb_serve_endpoints(usbp, 0);
            // USBInEndpointState *iesp = usbp->epc[0]->in_state;
            // _usb_isr_invoke_in_cb(usbp, 0);
		}
		else if (iwIntFlag & mskEP0_OUT)
		{
			/* OUT */
			// USB_EP0OutEvent();
            usb_serve_endpoints(usbp, 0);
		}
		else if (iwIntFlag & (mskEP0_IN_STALL|mskEP0_OUT_STALL))
		{
			/* EP0_IN_OUT_STALL */
			// SN_USB->INSTSC = (mskEP0_IN_STALL|mskEP0_OUT_STALL);
			// USB_EPnStall(USB_EP0);
            usb_serve_endpoints(usbp, 0);
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
			// USB_EP1AckEvent();
            // USB_EPnAck(USB_EP1,0);
            usb_serve_endpoints(usbp, 1);
		}
		if (iwIntFlag & mskEP2_ACK)
		{
			/* EP2 ACK */
			// USB_EP2AckEvent();
            // USB_EPnAck(USB_EP2,0);
            usb_serve_endpoints(usbp, 2);
		}
		if (iwIntFlag & mskEP3_ACK)
		{
			/* EP3 ACK */
			// USB_EP3AckEvent();
            // USB_EPnAck(USB_EP3,0);
            usb_serve_endpoints(usbp, 3);
		}
		if (iwIntFlag & mskEP4_ACK)
		{
			/* EP4 ACK */
			// USB_EP4AckEvent();
            // USB_EPnAck(USB_EP4,0);
            usb_serve_endpoints(usbp, 4);
		}
	}

	/////////////////////////////////////////////////
	/* Device Status Interrupt (EPnNAK) 					 */
	/////////////////////////////////////////////////
	else if (iwIntFlag & (mskEP4_NAK|mskEP3_NAK|mskEP2_NAK|mskEP1_NAK))
	{
		if (iwIntFlag & mskEP1_NAK)
		{
			/* EP1 NAK */
			// USB_EP1NakEvent();
            // USB_EPnNak(USB_EP1);
            usb_serve_endpoints(usbp, 1);
		}
		if (iwIntFlag & mskEP2_NAK)
		{
			/* EP2 NAK */
			// USB_EP2NakEvent();
            // USB_EPnNak(USB_EP2);
            usb_serve_endpoints(usbp, 2);
		}
		if (iwIntFlag & mskEP3_NAK)
		{
			/* EP3 NAK */
			// USB_EP3NakEvent();
            // USB_EPnNak(USB_EP3);
            usb_serve_endpoints(usbp, 3);
		}
		if (iwIntFlag & mskEP4_NAK)
		{
			/* EP4 NAK */
			// USB_EP4NakEvent();
            // USB_EPnNak(USB_EP4);
            usb_serve_endpoints(usbp, 4);
		}
	}

	/////////////////////////////////////////////////
	/* Device Status Interrupt (SOF) 							 */
	/////////////////////////////////////////////////
	if ((iwIntFlag & mskUSB_SOF) && (SN_USB->INTEN & mskUSB_SOF_IE))
	{
		/* SOF */
		USB_SOFEvent();
        _usb_isr_invoke_sof_cb(usbp);
	}
}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

/**
 * @brief   OTG1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_USB_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();
  usb_lld_serve_interrupt(&USBD1);
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

#if PLATFORM_USB_USE_USB1 == TRUE
  /* Driver initialization.*/
  usbObjectInit(&USBD1);
#endif
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
        nvicEnableVector(USB_IRQn, 2);
    }
#endif
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
void usb_lld_reset(USBDriver *usbp) {

    /* Post reset initialization.*/

    /* EP0 initialization.*/
    usbp->epc[0] = &ep0config;
    usb_lld_init_endpoint(usbp, 0);
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
    uint32_t 	wTmp = 0;
    const USBEndpointConfig *epcp = usbp->epc[ep];

    /* IN and OUT common parameters.*/
    switch (epcp->ep_mode & USB_ENDPOINT_TYPE_MASK) {
    case USB_ENDPOINT_TYPE_CONTROL:
        // ctl = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_CTRL;
        break;
    case USB_ENDPOINT_TYPE_ISOCHRONOUS:
        // ctl = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_ISO;
        break;
    case USB_ENDPOINT_TYPE_BULK:
        // ctl = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_BULK;
        break;
    case USB_ENDPOINT_TYPE_INTERRUPT:
        // ctl = DIEPCTL_SD0PID | DIEPCTL_USBAEP | DIEPCTL_EPTYP_INTR;
        break;
    default:
        return;
    }

    // dp = USB_GET_DESCRIPTOR(ep);

    /* IN endpoint activation or deactivation.*/
    if (epcp->in_state != NULL) {
        // USB_DIRECTION_IN
        epcp->in_state->txcnt = 0;
        epcp->in_state->txbuf = (uint8_t)&wUSB_EPnOffset[ep-1]; //(uint8_t)&wUSBINT_WriteDataBuf;
        USB_EPnNak(ep);
        switch (ep)
        {
        case 1:
            wTmp |= ~mskEP1_DIR;
            break;
        case 2:
            wTmp |= ~mskEP2_DIR;
            break;
        case 3:
            wTmp |= ~mskEP3_DIR;
            break;
        case 4:
            wTmp |= ~mskEP4_DIR;
            break;
        }
    }

    /* OUT endpoint activation or deactivation.*/
    if (epcp->out_state != NULL) {
        // USB_DIRECTION_OUT
        // USB_ENDPOINT_OUT(ep);
        epcp->out_state->rxcnt = 0;
        epcp->out_state->rxbuf = (uint8_t)&wUSB_EPnOffset[ep-1];//0; //&wUSBINT_ReadDataBuf;
        USB_EPnNak(ep);
        switch (ep)
        {
        case 1:
            wTmp |= mskEP1_DIR;
            break;
        case 2:
            wTmp |= mskEP2_DIR;
            break;
        case 3:
            wTmp |= mskEP3_DIR;
            break;
        case 4:
            wTmp |= mskEP4_DIR;
            break;
        }
    }

    SN_USB->CFG |= wTmp;

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
        USB_EPnDisable(i);
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
   if (SN_USB->INSTS & mskEP0_OUT) {
       return EP_STATUS_DISABLED;
   } else if (SN_USB->INSTS & mskEP0_OUT_STALL) {
       return EP_STATUS_STALLED;
   } else {
       return EP_STATUS_ACTIVE;
   }
   // (mskEP0_IN_STALL|mskEP0_OUT_STALL))
//   switch (SN_USB->INSTS) {
//       case mskEP0_IN:
//           return EP_STATUS_DISABLED;
//       case mskEP0_IN_STALL:
//           return EP_STATUS_ACTIVE;
//       default:
//           return EP_STATUS_ACTIVE;
//   }
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
   if (SN_USB->INSTS & mskEP0_IN) {
       return EP_STATUS_DISABLED;
   } else if (SN_USB->INSTS & mskEP0_IN_STALL) {
       return EP_STATUS_STALLED;
   } else {
       return EP_STATUS_ACTIVE;
   }

//   switch (SN_USB->INSTS) {
//     case mskEP0_OUT:
//       return EP_STATUS_DISABLED;
//     case mskEP0_OUT_STALL:
//       return EP_STATUS_STALLED;
//     default:
//       return EP_STATUS_ACTIVE;
//   }
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

  fnUSBINT_ReadFIFO((uint32_t)buf);

//   stm32_usb_pma_t *pmap;
//   stm32_usb_descriptor_t *udp;
//   uint32_t n;

//   (void)usbp;
//   udp = USB_GET_DESCRIPTOR(ep);
//   pmap = USB_ADDR2PTR(udp->RXADDR0);
//   for (n = 0; n < 4; n++) {
//     *(uint16_t *)buf = (uint16_t)*pmap++;
//     buf += 2;
//   }

}

// /**
//  * @brief   Prepares for a receive operation.
//  *
//  * @param[in] usbp      pointer to the @p USBDriver object
//  * @param[in] ep        endpoint number
//  *
//  * @notapi
//  */
// void usb_lld_prepare_receive(USBDriver *usbp, usbep_t ep) {

//   (void)usbp;
//   (void)ep;

// }

// /**
//  * @brief   Prepares for a transmit operation.
//  *
//  * @param[in] usbp      pointer to the @p USBDriver object
//  * @param[in] ep        endpoint number
//  *
//  * @notapi
//  */
// void usb_lld_prepare_transmit(USBDriver *usbp, usbep_t ep) {

//   (void)usbp;
//   (void)ep;

// }

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {

  USBOutEndpointState *osp = usbp->epc[ep]->out_state;

  /* Transfer initialization.*/
  if (osp->rxsize == 0)         /* Special case for zero sized packets.*/
    osp->rxpkts = 1;
  else
    osp->rxpkts = (uint16_t)((osp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                             usbp->epc[ep]->out_maxsize);

//   EPR_SET_STAT_RX(ep, EPR_STAT_RX_VALID);

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
  size_t n;
  USBInEndpointState *isp = usbp->epc[ep]->in_state;

  /* Transfer initialization.*/
  n = isp->txsize;
  if (n > (size_t)usbp->epc[ep]->in_maxsize)
    n = (size_t)usbp->epc[ep]->in_maxsize;

  isp->txlast = n;
//   usb_packet_write_from_buffer(ep, isp->txbuf, n);
  fnUSBINT_WriteFIFO((uint32_t)*isp->txbuf, n);

//   EPR_SET_STAT_TX(ep, EPR_STAT_TX_VALID);

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

  USB_EPnStall(ep);

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

   USB_EPnStall(ep);

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

  __USB_CLRINSTS(mskEP0_OUT);

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

  __USB_CLRINSTS(mskEP0_IN);

}

#endif /* HAL_USE_USB == TRUE */

/** @} */
