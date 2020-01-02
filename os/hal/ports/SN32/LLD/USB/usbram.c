/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbram.c
 * Purpose: USB Custom User Module
 * Version: V1.01
 * Date:		2013/11
 *------------------------------------------------------------------------------*/
#include	<SN32F240B.h>
#include	"usbram.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/
/*_____________ USB Variable ____________________________________________*/
volatile	uint32_t	wUSB_EPnOffset[6];
volatile	uint32_t	wUSB_EPnPacketsize[7];
volatile	uint8_t		wUSB_EndpHalt[7];
const	    uint8_t	    *pUSB_TableIndex;
volatile	uint32_t	wUSB_TableLength;
volatile	uint8_t	    wUSB_IfAlternateSet[6];

uint16_t	mode;
uint16_t	cnt;
uint32_t	wUSB_MouseData;



SUSB_EUME_DATA	sUSB_EumeData;
