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
 * @file    USBv1/sn32_usb.h
 * @brief   SN32 USB registers layout header.
 * @note    This file requires definitions from the ST STM32 header files
 *          sn32f124x.h
 *
 * @addtogroup USB
 * @{
 */

#ifndef SN32_USB_H
#define SN32_USB_H

/**
 * @brief   Number of the available endpoints.
 * @details This value does not include the endpoint 0 which is always present.
 */
#define USB_ENDOPOINTS_NUMBER           4

/**
 * @brief   Width of USB packet memory accesses.
 */
typedef uint32_t                        sn32_usb_pma_t;

/**
 * @brief   USB descriptor registers block.
 */
typedef struct {
  /**
   * @brief   TX buffer offset register.
   */
  volatile sn32_usb_pma_t      TXADDR0;
  /**
   * @brief   TX counter register 0.
   */
  volatile sn32_usb_pma_t      TXCOUNT0;
  /**
   * @brief   RX buffer offset register.
   */
  volatile sn32_usb_pma_t      RXADDR0;
  /**
   * @brief   RX counter register 0.
   */
  volatile sn32_usb_pma_t      RXCOUNT0;
} sn32_usb_descriptor_t;
/** @} */

/**
 * @brief USB registers block numeric address.
 */
#define SN32_USB_BASE          SN_USB_BASE

/**
 * @brief USB RAM numeric address.
 */
#define SN32_USBRAM_BASE       SN_USB_BASE + 0x100

/**
 * @brief Pointer to the USB registers block.
 */
// #define SN32_USB               ((sn32_usb_t *)SN32_USB_BASE)

/**
 * @brief   Pointer to the USB RAM.
 */
#define SN32_USBRAM            ((sn32_usb_pma_t *)SN32_USBRAM_BASE)

/**
 * @brief   Mask of all the toggling bits in the EPR register.
 */
#define EPR_TOGGLE_MASK         (EPR_STAT_TX_MASK | EPR_DTOG_TX |           \
                                 EPR_STAT_RX_MASK | EPR_DTOG_RX |           \
                                 EPR_SETUP)

#define EPR_EA_MASK             0x000F
#define EPR_STAT_TX_MASK        0x0030
#define EPR_STAT_TX_DIS         0x0000
#define EPR_STAT_TX_STALL       0x0010
#define EPR_STAT_TX_NAK         0x0020
#define EPR_STAT_TX_VALID       0x0030
#define EPR_DTOG_TX             0x0040
#define EPR_SWBUF_RX            EPR_DTOG_TX
#define EPR_CTR_TX              0x0080
#define EPR_EP_KIND             0x0100
#define EPR_EP_DBL_BUF          EPR_EP_KIND
#define EPR_EP_STATUS_OUT       EPR_EP_KIND
#define EPR_EP_TYPE_MASK        0x0600
#define EPR_EP_TYPE_BULK        0x0000
#define EPR_EP_TYPE_CONTROL     0x0200
#define EPR_EP_TYPE_ISO         0x0400
#define EPR_EP_TYPE_INTERRUPT   0x0600
#define EPR_SETUP               0x0800
#define EPR_STAT_RX_MASK        0x3000
#define EPR_STAT_RX_DIS         0x0000
#define EPR_STAT_RX_STALL       0x1000
#define EPR_STAT_RX_NAK         0x2000
#define EPR_STAT_RX_VALID       0x3000
#define EPR_DTOG_RX             0x4000
#define EPR_SWBUF_TX            EPR_DTOG_RX
#define EPR_CTR_RX              0x8000

#define CNTR_FRES               0x0001
#define CNTR_PDWN               0x0002
#define CNTR_LP_MODE            0x0004
#define CNTR_FSUSP              0x0008
#define CNTR_RESUME             0x0010
#define CNTR_ESOFM              0x0100
#define CNTR_SOFM               0x0200
#define CNTR_RESETM             0x0400
#define CNTR_SUSPM              0x0800
#define CNTR_WKUPM              0x1000
#define CNTR_ERRM               0x2000
#define CNTR_PMAOVRM            0x4000
#define CNTR_CTRM               0x8000

#define ISTR_EP_ID_MASK         0x000F
#define ISTR_DIR                0x0010
#define ISTR_ESOF               0x0100
#define ISTR_SOF                0x0200
#define ISTR_RESET              0x0400
#define ISTR_SUSP               0x0800
#define ISTR_WKUP               0x1000
#define ISTR_ERR                0x2000
#define ISTR_PMAOVR             0x4000
#define ISTR_CTR                0x8000

#define FNR_FN_MASK             0x07FF
#define FNR_LSOF                0x1800
#define FNR_LCK                 0x2000
#define FNR_RXDM                0x4000
#define FNR_RXDP                0x8000

#define DADDR_ADD_MASK          0x007F
#define DADDR_EF                0x0080

#define RXCOUNT_COUNT_MASK      0x03FF
#define TXCOUNT_COUNT_MASK      0x03FF

#define EPR_CTR_MASK            (EPR_CTR_TX | EPR_CTR_RX)

/**
 * @brief   Returns an endpoint descriptor pointer.
 */
#define USB_GET_ENDPOINT_DESCRIPTOR(ep)                                          \
  ((sn32_usb_descriptor_t *)((uint32_t)SN32_USBRAM_BASE +               \
                              (uint32_t)(ep) *                          \
                              sizeof(sn32_usb_descriptor_t)))

/**
 * @brief   Converts from a PMA address to a physical address.
 */
#define USB_ADDR2PTR(addr)                                                  \
  ((sn32_usb_pma_t *)((addr) *                                             \
                       (sizeof(sn32_usb_pma_t) / 2) +                      \
                       SN32_USBRAM_BASE))

#endif /* SN32_USB_H */

/** @} */
