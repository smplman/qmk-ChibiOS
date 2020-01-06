/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbhw.c
 * Purpose: USB Custom User Module
 * Version: V1.01
 * Date:		2013/11
 *------------------------------------------------------------------------------*/
#include	<SN32F240B.h>
// #include	"..\type.h"
// #include	"..\Utility\Utility.h"

#include	"usb.h"
#include	"usbram.h"
#include	"usbhw.h"
#include	"usbuser.h"
#include	"usbdesc.h"
#include	"usbsystem.h"
#include	"hiduser.h"


/*****************************************************************************
* Function		: USB_IRQHandler
* Description	: USB Interrupt USB_BUS, USB SOF, USB_IE
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void USB_IRQHandler(void)
{
    uint32_t iwIntFlag;
	iwIntFlag = SN_USB->INSTS;		/* Get Interrupt Status and clear immediately. */

	if (iwIntFlag & mskBUS_WAKEUP)
	{									/* Wakeup */
		USB_WakeupEvent();
		return;
	}
	if ((iwIntFlag & mskUSB_SOF) & (SN_USB->INTEN & mskUSB_SOF_IE))
	{									/* SOF */
		USB_SOFEvent();
	}
	/* Device Status Interrupt (BusReset, Suspend, Resume) */
	if (iwIntFlag & (mskBUS_RESET|mskBUS_SUSPEND|mskBUS_RESUME))
	{
		if (iwIntFlag & mskBUS_RESET)
		{									/* BusReset */
			USB_ResetEvent();
		}
		else if (iwIntFlag & mskBUS_SUSPEND)
		{									/* Suspend */
			USB_SuspendEvent();
		}
		else if	( (iwIntFlag & mskBUS_RESUME))
		{									/* Resume */
			USB_ResumeEvent();
		}
	}
	else if (iwIntFlag & (mskEP0_SETUP|mskEP0_IN|mskEP0_OUT|mskEP0_IN_STALL|mskEP0_OUT_STALL))
	{									/* Device Status Interrupt (SETUP, IN, OUT) */
		if (iwIntFlag &  mskEP0_SETUP)
		{									/* SETUP */
			USB_EP0SetupEvent();
		}
		else if (iwIntFlag &  mskEP0_IN)
		{									/* IN */
			USB_EP0InEvent();
		}
		else if (iwIntFlag & mskEP0_OUT)
		{									/* OUT */
			USB_EP0OutEvent();
		}
		else if (iwIntFlag & (mskEP0_IN_STALL|mskEP0_OUT_STALL))
		{
			SN_USB->INSTSC = (mskEP0_IN_STALL|mskEP0_OUT_STALL);
			USB_EPnStall(USB_EP0);
		}
	}
	else if (iwIntFlag & (mskEP4_ACK|mskEP3_ACK|mskEP2_ACK|mskEP1_ACK))
	{
		if (iwIntFlag & mskEP1_ACK)
		{									/* EP1 ACK */
			USB_EP1AckEvent();
		}
		if (iwIntFlag & mskEP2_ACK)
		{									/* EP2 ACK */
			USB_EP2AckEvent();
		}
		if (iwIntFlag & mskEP3_ACK)
		{									/* EP3 ACK */
			USB_EP3AckEvent();
		}
		if (iwIntFlag & mskEP4_ACK)
		{									/* EP4 ACK */
			USB_EP4AckEvent();
		}
		if (iwIntFlag & mskEP5_ACK)
		{									/* EP5 ACK */
			USB_EP5AckEvent();
		}
		if (iwIntFlag & mskEP6_ACK)
		{									/* EP6 ACK */
			USB_EP6AckEvent();
		}
	}
  return;
}

/*****************************************************************************
* Function		: USB_EP1AckEvent
* Description	: USB Clear EP1 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP1AckEvent(void)
{
	__USB_CLRINSTS(mskEP1_ACK);
}

/*****************************************************************************
* Function		: USB_EP2AckEvent
* Description	: USB Clear EP2 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP2AckEvent(void)
{
	__USB_CLRINSTS(mskEP2_ACK);
}

/*****************************************************************************
* Function		: USB_EP3AckEvent
* Description	: USB Clear EP3 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP3AckEvent(void)
{
	__USB_CLRINSTS(mskEP3_ACK);
}

/*****************************************************************************
* Function		: USB_EP4AckEvent
* Description	: USB Clear EP4 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP4AckEvent(void)
{
	__USB_CLRINSTS(mskEP4_ACK);
}

/*****************************************************************************
* Function		: USB_EP5AckEvent
* Description	: USB Clear EP5 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP5AckEvent(void)
{
	__USB_CLRINSTS(mskEP5_ACK);
}
/*****************************************************************************
* Function		: USB_EP6AckEvent
* Description	: USB Clear EP6 ACK interrupt status
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EP6AckEvent(void)
{
	__USB_CLRINSTS(mskEP6_ACK);
}

/*****************************************************************************
* Function		: USB_ClrEPnToggle
* Description	: USB Clear EP1~EP6 toggle bit to DATA0
								write 1: toggle bit Auto. write0:clear EPn toggle bit to DATA0
* Input				: hwEPNum ->EP1~EP6
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_ClrEPnToggle(uint32_t hwEPNum)
{
	SN_USB->EPTOGGLE &= ~(0x1<<hwEPNum);
}

/*****************************************************************************
* Function		: USB_Init
* Description	: 1. setting IDLE_TIME, REPORT_PROTOCOL, S_USB_EP0setupdata.wUSB_Status
*								2. set EP1~EP6 FIFO RAM address.
*								3. save	EP1~EP6 FIFO RAM point address.
*								4. save EP1~EP6 Package Size.
*								5. Enable USB function and setting EP1~EP6 Direction.
*								6. NEVER REMOVE !! USB D+/D- Dischage
*								7. Enable USB PHY and USB interrupt.
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_Init(void)
{
	volatile uint32_t	*pRam;
	uint32_t 	wTmp, i;

    // USB_StandardVar_Init();
	USB_HidVar_Init();

	/* Initialize clock and Enable USB PHY. */
	USB_SystemInit();																// enable System,PLL,EHS XTAL by user setting
    // SN_SYS1->AHBCLKEN |= 0x02;                                                       // Enable USBCLKEN
    SN_SYS1->AHBCLKEN |= (0x1 << 4);
    __USB_PHY_ENABLE;			// enable ESD_EN & PHY_EN

	/* Initialize USB  EP1~EP6 RAM address base on 64-bytes. */
	USB_EPnBufferOffset(1,EP1_BUFFER_OFFSET_VALUE);
	USB_EPnBufferOffset(2,EP2_BUFFER_OFFSET_VALUE);
	USB_EPnBufferOffset(3,EP3_BUFFER_OFFSET_VALUE);
	USB_EPnBufferOffset(4,EP4_BUFFER_OFFSET_VALUE);
	// USB_EPnBufferOffset(5,EP5_BUFFER_OFFSET_VALUE);
	// USB_EPnBufferOffset(6,EP6_BUFFER_OFFSET_VALUE);

	/* Initialize EP1~EP6 RAM point address to array(wUSB_EPnOffset).*/
	pRam = &wUSB_EPnOffset[0];
	*(pRam+0) = (uint32_t)(&USB_SRAM_EP0_W0) + EP1_BUFFER_OFFSET_VALUE;
	*(pRam+1) = (uint32_t)(&USB_SRAM_EP0_W0) + EP2_BUFFER_OFFSET_VALUE;
	*(pRam+2) = (uint32_t)(&USB_SRAM_EP0_W0) + EP3_BUFFER_OFFSET_VALUE;
	*(pRam+3) = (uint32_t)(&USB_SRAM_EP0_W0) + EP4_BUFFER_OFFSET_VALUE;
	*(pRam+4) = (uint32_t)(&USB_SRAM_EP0_W0) + EP5_BUFFER_OFFSET_VALUE;
	// *(pRam+5) = (uint32_t)(&USB_SRAM_EP0_W0) + EP6_BUFFER_OFFSET_VALUE;

	/* Initialize EP1~EP6 package size to array(wUSB_EPnPacketsize).*/
	pRam = &wUSB_EPnPacketsize[0];
	*(pRam+0) = USB_EP0_PACKET_SIZE;
	*(pRam+1) = USB_EP1_PACKET_SIZE;
	*(pRam+2) = USB_EP2_PACKET_SIZE;
	*(pRam+3) = USB_EP3_PACKET_SIZE;
	*(pRam+4) = USB_EP4_PACKET_SIZE;
	// *(pRam+5) = USB_EP5_PACKET_SIZE;
	// *(pRam+6) = USB_EP6_PACKET_SIZE;

    /* Enable the USB Interrupt */
	SN_USB->INTEN = (mskBUS_IE|mskUSB_IE);
    // SN_USB->INTEN = (0xF << 28);
    /* BUS_DRVEN = 0, BUS_DP = 1, BUS_DN = 0 */
	SN_USB->SGCTL = mskBUS_J_STATE;
    /* VREG33_EN = 1, PHY_EN = 1, DPPU_EN = 1, SIE_EN = 1, USBRAM_EN = 1, FLTDET_PUEN = 1 */
	// wTmp = (mskVREG33_EN|mskPHY_EN|mskDPPU_EN|mskSIE_EN|mskESD_EN|mskUSBRAM_EN|mskVREG33DIS_EN|mskFLTDET_PUEN_DISABLE);
    wTmp = (mskVREG33_EN|mskPHY_EN|mskDPPU_EN|mskSIE_EN|mskESD_EN|mskVREG33DIS_EN|mskFLTDET_PUEN_DISABLE);

    /*	setting EP1~EP6 Direction	*/
    #if (USB_EP1_DIRECTION == USB_DIRECTION_OUT)
        wTmp |= mskEP1_DIR;
    #endif
    #if (USB_EP2_DIRECTION == USB_DIRECTION_OUT)
        wTmp |= mskEP2_DIR;
    #endif
    #if (USB_EP3_DIRECTION == USB_DIRECTION_OUT)
        wTmp |= mskEP3_DIR;
    #endif
    #if (USB_EP4_DIRECTION == USB_DIRECTION_OUT)
        wTmp |= mskEP4_DIR;
    #endif
    #if (USB_EP5_DIRECTION == USB_DIRECTION_OUT)
        wTmp |= mskEP5_DIR;
    #endif
    #if (USB_EP6_DIRECTION == USB_DIRECTION_OUT)
        wTmp |= mskEP6_DIR;
    #endif

	//!!NEVER REMOVE!!!
 	SN_USB->CFG = wTmp;
	for (i = 0; i < DISCHARE_DELAY; i++);
    // wait_ms((USB_PLL_DLEYA_TIME/4)*(USB_AHB_PRESCALAR+1));
	SN_USB->CFG = (wTmp&(~mskVREG33DIS_EN))|mskDPPU_EN;
	//!!NEVER REMOVE!!!

	SN_USB->PHYPRM = (0x01U<<31);
    // NVIC_ClearPendingIRQ(USB_IRQn);
    // NVIC_EnableIRQ(USB_IRQn);
	//NVIC_DisableIRQ(USB_IRQn);
	return;
}

/*****************************************************************************
* Function		: USB_EPnDisable
* Description	: Disable EP1~EP6
* Input				: wEPNum
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnDisable (uint32_t	wEPNum)
{
	volatile uint32_t	*pEPn_ptr;
	if(wEPNum > USB_EP6)
		return;
	pEPn_ptr = &SN_USB->EP0CTL + wEPNum;
	*pEPn_ptr = 0;								//SET DISABLE. No handshake IN/OUT token.
}

/*****************************************************************************
* Function		: USB_EPnNak
* Description	: SET EP1~EP6 is NAK. For IN will handshake NAK to IN token.
*																		For OUT will handshake NAK to OUT token.
* Input				: wEPNum
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnNak (uint32_t	wEPNum)
{
	volatile	uint32_t	*pEPn_ptr;
	if(wEPNum > USB_EP6)
		return;
	pEPn_ptr = &SN_USB->EP0CTL + wEPNum;
	*pEPn_ptr = mskEPn_ENDP_EN;			//SET NAK
}

/*****************************************************************************
* Function		: USB_EPnAck
* Description	: SET EP1~EP6 is ACK. For IN will handshake bBytent to IN token.
*																		For OUT will handshake ACK to OUT token.
* Input				: wEPNum:EP1~EP6.
*								bBytecnt: Byte Number of Handshake.
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnAck(uint32_t wEPNum, uint8_t bBytecnt)
{
	volatile	uint32_t	*pEPn_ptr;
	if (wEPNum > USB_EP6)
		return;
	pEPn_ptr = &SN_USB->EP0CTL + wEPNum;
	*pEPn_ptr = (mskEPn_ENDP_EN|mskEPn_ENDP_STATE_ACK|bBytecnt);

}

/*****************************************************************************
* Function		: USB_EPnAck
* Description	: SET EP1~EP6 is STALL. For IN will handshake STALL to IN token.
*																			For OUT will handshake STALL to OUT token.
* Input				: wEPNum:EP1~EP6.
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnStall(uint32_t wEPNum)
{
	volatile uint32_t	*pEPn_ptr;
	if(wEPNum > USB_EP6)				//wEPNum != EP0~EP6
		return;
	pEPn_ptr = &SN_USB->EP0CTL + wEPNum;
	if (wEPNum == USB_EP0)
	{
			if(SN_USB->INSTS & mskEP0_PRESETUP)
				return;
	}
	*pEPn_ptr = (mskEPn_ENDP_EN|mskEPn_ENDP_STATE_STALL);
}

/*****************************************************************************
* Function		: USB_Suspend
* Description	: USB Suspend state SYS_CLK is runing slow mode.
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_Suspend (void)
{
	uint32_t	wWakeupstatus;

	sUSB_EumeData.wUSB_Status &= ~mskBUSSUSPEND;							// Clear BusSuspend
	wWakeupstatus = SN_GPIO2->DATA & 0xFF;

	if(!(SN_USB->INSTS & mskBUS_SUSPEND))				// double check Suspend flag
		return;

	SN_SYS0->AHBCP = 0;													// switch SYSCLK/1
	SN_SYS0->CLKCFG = 0x1;											// switch ILRC
	while((SN_SYS0->CLKCFG & 0x10) != 0x10);		// check ILRC status

    // SN_SYS0->PLLCTRL &= 0xFFFF7FFF;							// disable PLL
	SN_SYS0->ANBCTRL &= 0xFFFFFEE;							// disable IHRC & EHS

	SN_USB->CFG &= ~(mskESD_EN|mskPHY_EN);			// disable ESD_EN & PHY_EN
	SN_FLASH->LPCTRL = 0x5AFA0002;							// Slow mode power saving

	while	(SN_USB->INSTS & mskBUS_SUSPEND)
	{
		if (sUSB_EumeData.wUSB_Status & mskREMOTE_WAKEUP)				//cehck Remote
		{
			if ((SN_GPIO2->DATA & 0xFF) != wWakeupstatus)
			{
				//SN_SYS0->CLKCFG = 0x00;								// switch IHRC
				if (sUSB_EumeData.wUSB_SetConfiguration == USB_CONFIG_VALUE)
				{
					USB_SystemInit();												// enable System,PLL,EHS XTAL by user setting
					__USB_PHY_ENABLE;	// enable ESD_EN & PHY_EN
					USB_RemoteWakeUp();
					return;
				}
			}
		}
	}
	USB_SystemInit();				// enable System,PLL,EHS XTAL by user setting
	__USB_PHY_ENABLE;		// enable ESD_EN & PHY_EN
}

/*****************************************************************************
* Function		: USB_RemoteWakeUp
* Description	: USB Remote wakeup: USB D+/D- siganl is J-K state.
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_RemoteWakeUp()
{
	__USB_JSTATE_DRIVER;			// J state ;Full speed D+ = 1, D- = 0
	USB_DelayJstate();
	__USB_KSTATE_DRIVER;			// K state ;Full speed D+ = 0, D- = 1
	USB_DelayKstate();
	SN_USB->SGCTL &= ~mskBUS_DRVEN;
}

/*****************************************************************************
* Function		: USB_DelayJstate
* Description	: For J state delay. about 180us
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_DelayJstate()
{
	uint32_t	i;
	for (i=0; i<300; i++);	// delay 180us
}

/*****************************************************************************
* Function		: USB_DelayKstate
* Description	: For K state delay. about 14 ~ 14.5ms
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_DelayKstate()
{
	uint32_t	i;
	for (i=0; i < K_STATE_DELAY; i++);	// require delay 1ms ~ 15ms
}


/*****************************************************************************
* Function		: USB_EPnBufferOffset
* Description	: SET EP1~EP6 RAM point address
* Input				: wEPNum: EP1~EP6
*								wAddr of device address
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnBufferOffset(uint32_t wEPNum, uint32_t wAddr)
{
	volatile	uint32_t	*pEPn_ptr;
	if ((wEPNum > USB_EP0) && (wEPNum <= USB_EP6))	//wEPNum = EP1 ~ EP6
	{
		pEPn_ptr = &SN_USB->EP1BUFOS; 		// Assign point to EP1 RAM address
		*(pEPn_ptr+wEPNum-1) = wAddr;			// SET point to EPn RAM address
	}
}

/*****************************************************************************
* Function		: USB_EPnReadByteData
* Description	: EP1~EP6 Read RAM data by Bytes
* Input				: wEPNum: EP1~EP6
*								wByteIndex: Read Bytes count 0~63
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
uint32_t USB_EPnReadByteData(uint32_t wEPNum, uint32_t wByteIndex)
{
	uint32_t	*pUsbSram, wVal;
	if (wByteIndex < 64)				// 0~63 Byte index
	{
		pUsbSram = (uint32_t*)(*&wUSB_EPnOffset[wEPNum-1]) + (wByteIndex>>2);
		wVal = ((*pUsbSram)>>((wByteIndex&0x3)<<3))&0xFF;		// (wByteIndex%4)*8
		return(wVal);
	}
	return 0;
}

/*****************************************************************************
* Function		: USB_EPnReadWordData
* Description	: EP1~EP6 Read RAM data by Words
* Input				: wEPNum: EP1~EP6
*								wWordIndex: Read Words count 0~15
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
uint32_t USB_EPnReadWordData(uint32_t wEPNum, uint32_t wWordIndex)
{
	uint32_t	*pUsbSram;
	if (wWordIndex < 16)				// 0~15 Word index
	{
		pUsbSram = (uint32_t*)(*&wUSB_EPnOffset[wEPNum-1]) + wWordIndex;
		return (*pUsbSram);
	}
	return 0;
}

/*****************************************************************************
* Function		: USB_EPnWriteByteData
* Description	: EP1~EP6 Write RAM data by Bytes
* Input				: wEPNum: EP1~EP6
*								wByteindex: Write Bytes count 0~63
*								wBytedata:  Write Data by Bytes
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnWriteByteData(uint32_t wEPNum, uint32_t wByteindex, uint32_t wBytedata)
{
	uint32_t	*pUsbSram;
	if (wByteindex < 64)				// 0~63 Byte index
	{
		pUsbSram = (uint32_t*)(*&wUSB_EPnOffset[wEPNum-1]) + (wByteindex>>2);
		*pUsbSram &= ~(0xFF<<((wByteindex&0x3)<<3));			// (wByteindex%4)*8
		*pUsbSram |= (wBytedata<<((wByteindex&0x3)<<3));
	}
}

/*****************************************************************************
* Function		: USB_EPnWriteWordData
* Description	: EP1~EP6 Write RAM data by Words
* Input				: wEPNum: EP1~EP6
*								wByteindex: Write Words count 0~15
*								wBytedata:  Write Data by Words
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_EPnWriteWordData(uint32_t wEPNum, uint32_t	wWordindex, uint32_t wWorddata)
{
	uint32_t	*pUsbSram;
	if (wWordindex < 16)				// 0~15 Word index
	{
		pUsbSram = (uint32_t*)(*&wUSB_EPnOffset[wEPNum-1]) + wWordindex;
		*pUsbSram = wWorddata;
	}
}
