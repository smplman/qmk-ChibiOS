/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 *      Name:    HIDUSER.H
 *      Purpose: HID Custom User Definitions
 *      Version: V1.20
 *----------------------------------------------------------------------------*/

#ifndef __HIDUSER_H__
#define __HIDUSER_H__


	

/* HID Requests Functions */
extern	void	HID_GetReportEvent(void);
extern	void	HID_GetIdleEvent(void);
extern	void	HID_GetProtocolEvent(void);
extern	void	HID_SetReportEvent(void);
extern	void	HID_SetIdleEvent(void);
extern	void	HID_SetProtocolEvent(void);
extern	void	HID_GetReportInputEvent(void);
extern	void	HID_GetReportInputIF1Event(void);
extern	void	HID_GetReportInputIF2Event(void);
extern	void	HID_GetReportOutputEvent(void);
extern	void	HID_GetReportFeatureEvent(void);
extern	void	HID_SetReportOutputEvent(void);
extern	void	HID_SetReportFeatureEvent(void);
extern	void	USB_HidVar_Init(void);

#endif  /* __HIDUSER_H__ */
