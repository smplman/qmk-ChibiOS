/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    hiduser.c
 * Purpose: USB Custom User Module
 * Version: V1.01
 * Date:		2017/07
 *------------------------------------------------------------------------------*/
#include	<SN32F240B.h>
// #include	"..\type.h"
#include	"hid.h"
#include	"hiduser.h"
#include	"hidram.h"
#include	"usb.h"
#include	"usbhw.h"
#include	"usbram.h"
#include	"usbuser.h"
#include	"usbdesc.h"


/*****************************************************************************
* Function		: USB_HIDRequest
* Description	: sUSB_EumeData.bUSB_bRequest of HID_request type
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_HIDRequest(void)
{
	switch	(sUSB_EumeData.bUSB_bRequest)
	{
		case	HID_REQUEST_GET_REPORT:
		{
			HID_GetReportEvent();
			break;
		}
		case	HID_REQUEST_GET_IDLE:
		{
			HID_GetIdleEvent();
			break;
		}
		case	HID_REQUEST_GET_PROTOCOL:
		{
			HID_GetProtocolEvent();
			break;
		}
		case	HID_REQUEST_SET_REPORT:
		{
			HID_SetReportEvent();
			break;
		}
		case	HID_REQUEST_SET_IDLE:
		{
			HID_SetIdleEvent();
			break;
		}
		case	HID_REQUEST_SET_PROTOCOL:
		{
			HID_SetProtocolEvent();
			break;
		}
		default:
		{
			USB_EPnStall(USB_EP0);					// EP0 STALL
			break;
		}
	}
}


/*****************************************************************************
* Function		: HID_GetReportEvent
* Description	: sUSB_EumeData.bUSB_wValueH of Get report type
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_GetReportEvent(void)
{
	if (sUSB_EumeData.bUSB_bmRequestType == HID_REQUEST_GET)		//double check request = HID_Get report
	{	// request = HID_Get report
		if ((sUSB_EumeData.wUSB_SetConfiguration == USB_CONFIG_VALUE))
		{
			if (sUSB_EumeData.bUSB_wIndexL == USB_INTERFACE_0)	// Interface 0
			{
				if (sUSB_EumeData.bUSB_wValueH == HID_REPORT_INPUT)
				{	//TYPE = INPUT
					HID_GetReportInputEvent();
					return;
				}
				else if (sUSB_EumeData.bUSB_wValueH == HID_REPORT_OUTPUT)
				{	//TYPE = OUTPUT
					HID_GetReportOutputEvent();
					return;
				}
				else if(sUSB_EumeData.bUSB_wValueH ==HID_REPORT_FEATURE)
				{	//TYPE = FEATURE
					/**************************************************************************/
					//Note!! The feature of get report may not necessary in HID,
					//user still can design by add the code here!!
					/**************************************************************************/
					HID_GetReportFeatureEvent();
					return;
				}
			}
		#if (USB_LIBRARY_TYPE != USB_MOUSE_TYPE)
			else if (sUSB_EumeData.bUSB_wIndexL == USB_INTERFACE_1)	// Interface 1
			{
				if (sHID_Data.wHID_Protocol == USB_REPORT_PROTOCOL)
				{
					if (sUSB_EumeData.bUSB_wValueH == HID_REPORT_INPUT)
					{
						HID_GetReportInputIF1Event();
						return;
					}
				}
			}
		#endif
		#if (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE1)
			else if (sUSB_EumeData.bUSB_wIndexL == USB_INTERFACE_2)	// Interface 2
			{
				if (sHID_Data.wHID_Protocol == USB_REPORT_PROTOCOL)
				{
					if (sUSB_EumeData.bUSB_wValueH == HID_REPORT_INPUT)
					{
						HID_GetReportInputIF2Event();
						return;
					}
				}
			}
		#endif
		}
	}
	USB_EPnStall(USB_EP0);							// EP0 STALL
}


/*****************************************************************************
* Function		: HID_GetIdleEvent
* Description	: wUSB_HidIdleTimeIf0ID of device HID idle time
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_GetIdleEvent(void)
{
	if (sUSB_EumeData.bUSB_bmRequestType == HID_REQUEST_GET)
	{
		sHID_Data.wHID_Status |= mskPROTOCOL_SET_IDLE;	// Set Protocol_set_idle = 1
		if ((sUSB_EumeData.bUSB_wIndexL | sUSB_EumeData.bUSB_wIndexL) == USB_INTERFACE_0)	// Interface 0
		{
			fnUSBINT_WriteFIFO(0x00, sHID_Data.wHID_IdleTimeIf0ID);
			sUSB_EumeData.wUSB_Status |= mskSETUP_OUT;		// Set Setup_OUT = 1
			USB_EPnAck(USB_EP0,1);					// EP0 ACK 1 byte
			return;
		}
	}
	USB_EPnStall(USB_EP0);							// EP0 STALL
}


/*****************************************************************************
* Function		: HID_GetProtocolEvent
* Description	: S_HID_data.wUSB_HidProtocol of device Protocol type
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_GetProtocolEvent(void)
{
	if (sUSB_EumeData.bUSB_bmRequestType == HID_REQUEST_GET)
	{
		if ((sHID_Data.wHID_Status & mskPROTOCOL_ARRIVAL) == 0)
		{
			sHID_Data.wHID_Protocol = USB_REPORT_PROTOCOL;
		}
	#if (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE1)
		if (sUSB_EumeData.bUSB_wIndexL <= USB_INTERFACE_2)	// Interface 0 ~ Interface 2
	#elif (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
		if (sUSB_EumeData.bUSB_wIndexL == USB_INTERFACE_0)	// Interface 0
	#elif (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE2)
		if (sUSB_EumeData.bUSB_wIndexL <= USB_INTERFACE_1)	// Interface 0 ~ Interface 1
	#endif
		{
			fnUSBINT_WriteFIFO(0x00, sHID_Data.wHID_Protocol);
			sUSB_EumeData.wUSB_Status |= mskSETUP_OUT;		// Set Setup_OUT = 1
			USB_EPnAck(USB_EP0,1);					// EP0 ACK 1 byte
			return;
		}
	}
	USB_EPnStall(USB_EP0);							// EP0 STALL
}


/*****************************************************************************
* Function		: HID_SetReportEvent
* Description	: sUSB_EumeData.bUSB_wValueH of Set report type
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_SetReportEvent(void)
{
	#if (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE1)
		if ((sUSB_EumeData.bUSB_bmRequestType == HID_REQUEST_SET) && ((sUSB_EumeData.bUSB_wIndexL == 0) || (sUSB_EumeData.bUSB_wIndexL == 3)))// Interface 0 & 3
	#elif (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
		if ((sUSB_EumeData.bUSB_bmRequestType == HID_REQUEST_SET) && (sUSB_EumeData.bUSB_wIndexL <= 1))// Interface 0 & 1
	#elif (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE2)
		if ((sUSB_EumeData.bUSB_bmRequestType == HID_REQUEST_SET) && ((sUSB_EumeData.bUSB_wIndexL == 0) || (sUSB_EumeData.bUSB_wIndexL == 2)))// Interface 0 & 2
	#endif
	{
		if (sUSB_EumeData.bUSB_wValueH == HID_REPORT_OUTPUT)
		{ //TYPE = OUTPUT
			HID_SetReportOutputEvent();
			return;
		}
		else if (sUSB_EumeData.bUSB_wValueH == HID_REPORT_FEATURE)
		{	//TYPE = FEATURE
			HID_SetReportFeatureEvent();
			return;
		}
	}
	USB_EPnStall(USB_EP0);							// EP0 STALL
}


/*****************************************************************************
* Function		: HID_SetIdleEvent
* Description	: sUSB_EumeData.bUSB_wValueH of HID idle time
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_SetIdleEvent(void)
{
	if (sUSB_EumeData.bUSB_bmRequestType == HID_REQUEST_SET)
	{
	#if (USB_LIBRARY_TYPE != USB_MOUSE_TYPE)
		// Set Protocol_set_idle & Setup_IN = 1
		sUSB_EumeData.wUSB_Status |= mskSETUP_IN;
		sHID_Data.wHID_Status |=mskPROTOCOL_SET_IDLE;
		if (sUSB_EumeData.bUSB_wIndexL <=USB_INTERFACE_1)		// Interface 0 & Interface 1
		{
			sHID_Data.wHID_IdleTimeIf0ID = sUSB_EumeData.bUSB_wValueH;
			USB_EPnAck(USB_EP0,0);							// EP0 ACK 0 byte
			return;
		}
	#endif
	}
	USB_EPnStall(USB_EP0);									// EP0 STALL
}


/*****************************************************************************
* Function		: HID_SetProtocolEvent
* Description	: sUSB_EumeData.bUSB_wValueL of HidProtocol
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_SetProtocolEvent(void)
{
	if (sUSB_EumeData.bUSB_bmRequestType == HID_REQUEST_SET)
	{
		sHID_Data.wHID_Status |= mskPROTOCOL_ARRIVAL;			// Set Protocol_arrival = 1
	#if (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE1)
		if (sUSB_EumeData.bUSB_wIndexL <= USB_INTERFACE_2)		// Interface 0 ~ Interface 2
	#elif (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
		if (sUSB_EumeData.bUSB_wIndexL == USB_INTERFACE_0)		// Interface 0
	#elif (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE2)
		if (sUSB_EumeData.bUSB_wIndexL <= USB_INTERFACE_1)		// Interface 0 ~ Interface 1
	#endif
		{
			if (sUSB_EumeData.bUSB_wValueL == USB_BOOT_PROTOCOL)
			{
				// Clear Protocol_get_report & Protocol_set_idle = 0
				sHID_Data.wHID_Status &= ~(mskPROTOCOL_GET_REPORT | mskPROTOCOL_SET_IDLE);
			}
			sHID_Data.wHID_Protocol = sUSB_EumeData.bUSB_wValueL;
			sUSB_EumeData.wUSB_Status |= mskSETUP_IN;			// Set Setup_IN = 1
			USB_EPnAck(USB_EP0,0);					// EP0 ACK 0 byte
			return;
		}
	}
	USB_EPnStall(USB_EP0);							// EP0 STALL
}


/*****************************************************************************
* Function		: HID_GetReportInputEvent
* Description	: Get report INPUT
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_GetReportInputEvent(void)
{
	wUSB_TableLength = 0;
	sUSB_EumeData.wUSB_Status |= mskSETUP_OUT;// Set Setup_OUT = 1
	USB_EPnAck(USB_EP0,8);			// EP0 ACK 8 byte
}


/*****************************************************************************
* Function		: HID_GetReportInputIF1Event
* Description	: Get report INPUT
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_GetReportInputIF1Event(void)
{
	wUSB_TableLength = 0;
	sUSB_EumeData.wUSB_Status |= mskSETUP_OUT;// Set Setup_OUT = 1
	USB_EPnAck(USB_EP0,5);			// EP0 ACK 5 byte
}


/*****************************************************************************
* Function		: HID_GetReportInputIF2Event
* Description	: Get report INPUT
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_GetReportInputIF2Event(void)
{
	wUSB_TableLength = 0;
	sUSB_EumeData.wUSB_Status |= mskSETUP_OUT;// Set Setup_OUT = 1
	USB_EPnAck(USB_EP0,5);			// EP0 ACK 5 byte
}


/*****************************************************************************
* Function		: HID_GetReportOutputEvent
* Description	: Get report OUTPUT
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_GetReportOutputEvent(void)
{
	fnUSBINT_WriteFIFO(0x00, sHID_Data.wHID_SetRptByte[0]);
	sUSB_EumeData.wUSB_Status |= mskSETUP_OUT;// Set Setup_OUT = 1
	USB_EPnAck(USB_EP0,1);			// EP0 ACK 1 byte
}


/*****************************************************************************
* Function		: HID_GetReportFeatureEvent
* Description	: Note!! The feature of get report may not necessary in HID,
*								user still can design by add the code here!!
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void HID_GetReportFeatureEvent(void)
{	//	Send data output to EP0 FIFO, for EP (Interrupt IN)
		USB_EPnAck(USB_EP0,0);			// EP0 ACK 0 byte
}


/*****************************************************************************
* Function		: HID_SetReportOutputEvent
* Description	: Set report OUTPUT
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_SetReportOutputEvent(void)
{
	// Set Set_report_flag & Setup_IN = 1
	sUSB_EumeData.wUSB_Status |= mskSETUP_IN;
	sHID_Data.wHID_Status |= mskSET_REPORT_FLAG;
	USB_EPnAck(USB_EP0,0);			// EP0 ACK 0 byte
}


/*****************************************************************************
* Function		: HID_SetReportFeatureEvent
* Description	: Set report Feature
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void	HID_SetReportFeatureEvent(void)
{
	sHID_Data.wHID_SetReportFeature = mskSET_REPORTFEATURE_FLAG;			//NEVER REMOVE!!
	USB_EPnAck(USB_EP0,0);			// EP0 ACK 0 byte
}


/*****************************************************************************
* Function		: USB_HidVar_Init
* Description	: USB HID Variable initialtion
* Input				: None
* Output			: None
* Return			: None
* Note				: None
*****************************************************************************/
void USB_HidVar_Init(void)
{
	sHID_Data.wHID_IdleTimeIf0ID = USB_IDLE_TIME_INITIAL;
	sHID_Data.wHID_Protocol = USB_REPORT_PROTOCOL;
	sHID_Data.wHID_SetReportFeature = 0;
	sHID_Data.wHID_Status = 0;
}

