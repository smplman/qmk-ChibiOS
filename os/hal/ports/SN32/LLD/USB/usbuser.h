/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbuser.h
 * Purpose: USB Custom User Definitions
 * Version: V1.20
 *-----------------------------------------------------------------------------*/

#ifndef	__USBUSER_H__
#define	__USBUSER_H__

/* USB Standard Device Requests */
// #define	USB_GET_STATUS					0
// #define	USB_CLEAR_FEATURE				1
// #define	USB_SET_FEATURE					3
// #define	USB_SET_ADDRESS					5
// #define	USB_GET_DESCRIPTOR			6
// #define	USB_SET_DESCRIPTOR			7
// #define	USB_GET_CONFIGURATION		8
// #define	USB_SET_CONFIGURATION		9
// #define	USB_GET_INTERFACE				10
// #define	USB_SET_INTERFACE				11
// #define	USB_SYNCH_FRAME					12

extern	void	USB_StandardRequest(void);
extern	void	USB_HIDRequest(void);
extern	void	USB_TableTransmit(void);
extern	void	USB_StandardVar_Init(void);

extern	void	USB_GetStatusEvent(void);
extern	void	USB_ClearFeatureEvent(void);
extern	void	USB_SetFeatureEvent(void);
extern	void	USB_SetAddressEvent(void);
extern	void	USB_GetDescriptorEvent(void);
extern	void	USB_SetDescriptorEvent(void);
extern	void	USB_GetConfigurationEvent(void);
extern	void	USB_SetConfigurationEvent(void);
extern	void	USB_GetInterfaceEvent(void);
extern	void	USB_SetInterfaceEvent(void);
extern	void	USB_SynchFrameEvent(void);

extern	void USB_EP0InEvent(void);
extern	void USB_EP0OutEvent(void);

extern uint32_t	USB_Comb_Bytetoword	(uint8_t	data0, uint8_t	data1, uint8_t	data2, uint8_t	data3);

#endif  /* __USBUSER_H__ */
