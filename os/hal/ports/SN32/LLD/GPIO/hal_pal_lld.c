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
 * @file    GPIOv3/hal_pal_lld.c
 * @brief   SN32 PAL low level driver code.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void initgpio(sn32_gpio_t *gpiop, const sn32_gpio_setup_t *config) {

    // volatile uint32_t     DATA;
    // volatile uint32_t     MODE;
    // volatile uint32_t     CFG;
    // volatile uint32_t     IS;
    // volatile uint32_t     IBS;
    // volatile uint32_t     IEV;
    // volatile uint32_t     IE;
    // volatile uint32_t     RIS;
    // volatile uint32_t     IC;
    // volatile uint32_t     BSET;
    // volatile uint32_t     BCLR;

    gpiop->DATA  = config->data;
    gpiop->MODE  = config->mode;
    gpiop->CFG   = config->cfg;
    gpiop->IBS   = config->ibs;
    gpiop->IEV   = config->iev;
    gpiop->IE    = config->ie;
    gpiop->RIS   = config->ris;
    gpiop->IC    = config->ic;
    gpiop->BSET  = config->bset;
    gpiop->BCLR  = config->bclr;

	// GPIO_Mode			(GPIO_PORT2, GPIO_PIN0, GPIO_MODE_INPUT);
	// GPIO_Config		(GPIO_PORT2, GPIO_PIN0, GPIO_CFG_PULL_DOWN);
	// //P2.0 as rising edge
	// GPIO_P2Trigger(GPIO_PIN0,	GPIO_IS_EDGE, GPIO_IBS_EDGE_TRIGGER, GPIO_IEV_RISING_EDGE);
	// GPIO_Interrupt(GPIO_PORT2, GPIO_PIN0,	GPIO_IE_EN);
    // GPIO_Clr			(GPIO_PORT2, GPIO_PIN5);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   SN32 I/O ports configuration.
 * @details Ports A-D clocks enabled.
 *
 * @param[in] config    the SN32 ports configuration
 *
 * @notapi
 */
void _pal_lld_init(const PALConfig *config) {

  /*
   * Initial GPIO setup.
   */

#if SN32_HAS_GPIOA
  initgpio(GPIOA, &config->PAData);
#endif
#if SN32_HAS_GPIOB
  initgpio(GPIOB, &config->PBData);
#endif
#if SN32_HAS_GPIOC
  initgpio(GPIOC, &config->PCData);
#endif
#if SN32_HAS_GPIOD
  initgpio(GPIOD, &config->PDData);
#endif

}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    @p PAL_MODE_UNCONNECTED is implemented as push pull at minimum
 *          speed.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           iomode_t mode) {

//   uint32_t moder   = (mode & PAL_SN32_MODE_MASK) >> 0;
//   uint32_t otyper  = (mode & PAL_SN32_OTYPE_MASK) >> 2;
//   uint32_t ospeedr = (mode & PAL_SN32_OSPEED_MASK) >> 3;
//   uint32_t pupdr   = (mode & PAL_SN32_PUPDR_MASK) >> 5;
//   uint32_t altr    = (mode & PAL_SN32_ALTERNATE_MASK) >> 7;
//   uint32_t ascr    = (mode & PAL_SN32_ASCR_MASK) >> 11;
//   uint32_t lockr   = (mode & PAL_SN32_LOCKR_MASK) >> 12;
//   uint32_t bit     = 0;
//   while (true) {
//     if ((mask & 1) != 0) {
//       uint32_t altrmask, m1, m2, m4;

//       altrmask = altr << ((bit & 7) * 4);
//       m1 = 1 << bit;
//       m2 = 3 << (bit * 2);
//       m4 = 15 << ((bit & 7) * 4);
//       port->OTYPER  = (port->OTYPER & ~m1) | otyper;
//       port->ASCR    = (port->ASCR & ~m1) | ascr;
//       port->OSPEEDR = (port->OSPEEDR & ~m2) | ospeedr;
//       port->PUPDR   = (port->PUPDR & ~m2) | pupdr;
//        if ((mode & PAL_SN32_MODE_MASK) == PAL_SN32_MODE_ALTERNATE) {
//         /* If going in alternate mode then the alternate number is set
//            before switching mode in order to avoid glitches.*/
//         if (bit < 8)
//           port->AFRL = (port->AFRL & ~m4) | altrmask;
//         else
//           port->AFRH = (port->AFRH & ~m4) | altrmask;
//         port->MODER   = (port->MODER & ~m2) | moder;
//       }
//       else {
//         /* If going into a non-alternate mode then the mode is switched
//            before setting the alternate mode in order to avoid glitches.*/
//         port->MODER   = (port->MODER & ~m2) | moder;
//         if (bit < 8)
//           port->AFRL = (port->AFRL & ~m4) | altrmask;
//         else
//           port->AFRH = (port->AFRH & ~m4) | altrmask;
//       }
//       port->LOCKR   = (port->LOCKR & ~m1) | lockr;
//     }
//     mask >>= 1;
//     if (!mask)
//       return;
//     otyper <<= 1;
//     ospeedr <<= 2;
//     pupdr <<= 2;
//     moder <<= 2;
//     bit++;
//   }
}

#endif /* HAL_USE_PAL */

/** @} */
