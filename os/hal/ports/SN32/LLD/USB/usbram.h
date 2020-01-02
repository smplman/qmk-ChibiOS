/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbuser.h
 * Purpose: USB Custom User Definitions
 * Version: V1.20
 *----------------------------------------------------------------------------*/

#ifndef __USBRAM_H__
#define __USBRAM_H__



typedef struct
{
	uint8_t	bUSB_bmRequestType;
	uint8_t	bUSB_bRequest;
	uint8_t	bUSB_wValueL;
	uint8_t	bUSB_wValueH;
	uint8_t	bUSB_wIndexL;
	uint8_t	bUSB_wIndexH;
	uint16_t	bUSB_wLength;
	volatile	uint32_t wUSB_SetConfiguration;
	uint8_t		bUSB_DeviceAddr;
	volatile	uint32_t	wUSB_Status;
}SUSB_EUME_DATA;

extern  SUSB_EUME_DATA	sUSB_EumeData;
//extern S_USB_SETUP_DATA	S_USB_EP0setupdata;


/*_____ D E C L A R A T I O N S ____________________________________________*/

extern	volatile uint32_t	wUSB_EPnOffset[6];
extern	volatile uint32_t	wUSB_EPnPacketsize[7];
extern	volatile uint8_t		wUSB_EndpHalt[7];

extern	const	uint8_t	*pUSB_TableIndex;
extern	volatile uint32_t	wUSB_TableLength;
extern	volatile uint8_t	wUSB_IfAlternateSet[6];

extern	uint16_t	mode;
extern	uint16_t	cnt;
extern	uint32_t	wUSB_MouseData;



#endif  /* __USBRAM_H__ */
