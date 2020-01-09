/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbepfunc.h
 * Purpose: USB Descriptors Definitions
 * Version: V1.20
 *----------------------------------------------------------------------------*/

#ifndef __USBEPFUNC_H__
#define __USBEPFUNC_H__

// Return value definition for USB_EPnINFunction
enum EPn_RETURN_VALUE
{
	EPn_RETURN_ACK_OK = 0,
	EPn_RETURN_ACK_BUSY = 1,
	EPn_RETURN_DISABLE = 2,
	EPn_RETURN_STALL = 3,
	EPn_RETURN_NOT_SUPPORT = 4,
	EPn_RETURN_OVER_MAX_SIZE = 5
};


extern	uint32_t USB_EPnINFunction(uint32_t	wEPNum, uint32_t	*pData, uint32_t	wBytecnt);

extern	void	USB_Test(void);

#endif  /* __USBEPFUNC_H__ */
