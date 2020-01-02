/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbdesc.c
 * Purpose: USB Custom User Module
 * Version: V1.01
 * Date:		2013/11
 *------------------------------------------------------------------------------*/
// #include	"..\type.h"
#include    "stdint.h"
#include	"usb.h"
#include	"usbdesc.h"
#include	"usbram.h"

#include	"hid.h"
#include	"hiduser.h"




/*****************************************************************************
* Description	: USB_DeviceDescriptor[]
*****************************************************************************/
const	uint8_t USB_DeviceDescriptor[] = {
	USB_DEVICE_DESC_SIZE,								/* bLength */
	USB_DEVICE_DESCRIPTOR_TYPE,					/* bDescriptorType */
	USB_WBVAL(0x0200), /* 2.00 */				/* bcdUSB */
	0x00,																/* bDeviceClass */
	0x00,																/* bDeviceSubClass */
	0x00,																/* bDeviceProtocol */
	USB_EP0_PACKET_SIZE,								/* bMaxPacketSize0 */
	USB_WBVAL(USB_VID),									/* idVendor */
	USB_WBVAL(USB_PID),									/* idProduct */
	USB_WBVAL(USB_REV), /* 1.05 */			/* bcdDevice */
	USB_DEVICE_STRING_MANUFACTURER,			/* iManufacturer */
	USB_DEVICE_STRING_PRODUCT,					/* iProduct */
	USB_DEVICE_STRING_RESERVED,					/* iSerialNumber */
	0x01,																/* bNumConfigurations: one possible configuration*/
};

/*****************************************************************************
* Description	: HID_ReportDescriptor[]
*****************************************************************************/
const uint8_t HID_ReportDescriptor[] = {
#if (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),	// usage page (generic desktop)
	HID_Usage(HID_USAGE_GENERIC_MOUSE),			// usage (mouse)
	HID_Collection(HID_Application),				// collection (application)
	HID_Usage(HID_USAGE_GENERIC_POINTER),		// usage (pointer)
	HID_Collection(0),											// collection (linked)
	HID_UsagePage(HID_USAGE_PAGE_BUTTON),		// usage page (buttons)
	HID_UsageMin(1),												// usage minimum (1)
	HID_UsageMax(3),												// usage maximum (3)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMax(1),											// logical maximum (1)
	HID_ReportCount(3),											// report count (3)
	HID_ReportSize(1),											// report size (1)
	HID_Input(HID_Data|HID_Variable|HID_Absolute),	// input (data, variable, absolute)
	HID_ReportCount(1),											// report count (1)
	HID_ReportSize(5),											// report size (5)
	HID_Input(HID_Constant),								// input (constant)
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),	// usage page (generic desktop)
	HID_Usage(HID_USAGE_GENERIC_X),					// usage (X)
	HID_Usage(HID_USAGE_GENERIC_Y),					// usage (Y)
	HID_Usage(HID_USAGE_GENERIC_WHEEL),			// usage (Wheel)
	HID_LogicalMin(0x81),										// logical minimum (-127)
	HID_LogicalMax(0x7F),										// logical maximum (127)
	HID_ReportSize(8),											// report size (8)
	HID_ReportCount(3),											// report count (3)
	HID_Input(HID_Data|HID_Variable|HID_Relative),	// input (data, variable, relative)
	HID_EndCollection,											// end collection

	HID_UsagePage(HID_USAGE_PAGE_CONSUMER),					// usage page (consumer)
	HID_Usage(HID_USAGE_PAGE_UNDEFINED),						// usage (consumer control)
	HID_LogicalMin(0x80),														// logical minimum (-128)
	HID_LogicalMax(0x7F),														// logical maximum (+127)
	HID_ReportCount(64),															// report count (64)
	HID_ReportSize(8),															// report size (8)
	HID_Feature(HID_Data|HID_Variable|HID_Absolute),// feature (data, variable, absolute)

	HID_EndCollection,											// end collection
#else
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),	// usage page (generic desktop)
	HID_Usage(HID_USAGE_GENERIC_KEYBOARD),	// usage (keyboard)
	HID_Collection(HID_Application),				// collection (application)
	HID_UsagePage(HID_USAGE_PAGE_KEYBOARD),	// usage page (keyboard)
	HID_UsageMin(HID_USAGE_KEYBOARD_LCTRL),	// usage minimum (224)
	HID_UsageMax(HID_USAGE_KEYBOARD_RGUI),	// usage maximum (231)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMax(1),											// logical maximum (1)
	HID_ReportSize(1),											// report size (1)
	HID_ReportCount(8),											// report count (8)
	HID_Input(HID_Data|HID_Variable|HID_Absolute),	// input (data, variable, absolute)
	HID_ReportCount(1),											// report count (1)
	HID_ReportSize(8),											// report size (8)
	HID_Input(HID_Constant),								// input (constant)
	HID_ReportCount(5),											// report count (5)
	HID_ReportSize(1),											// report size (1)
	HID_Usage(HID_USAGE_PAGE_LED),					// usage (LEDs)
	HID_UsageMin(HID_USAGE_LED_NUM_LOCK),		// usage minimum (1)
	HID_UsageMax(HID_USAGE_LED_KANA),				// usage maximum (5)
	HID_Output(HID_Data|HID_Variable|HID_Absolute),	// output (data, variable, absolute)
	HID_ReportCount(1),											// report count (1)
	HID_ReportSize(3),											// report size (3)
	HID_Output(HID_Constant),								// output (constant)
	HID_ReportCount(6),											// report count (6)
	HID_ReportSize(8),											// report size (8)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMaxS(255),										// logical maximum (255)
	HID_UsagePage(HID_USAGE_PAGE_KEYBOARD),	// usage page (keyboard)
	HID_UsageMin(0),												// usage minimum (0)
	HID_UsageMaxS(HID_USAGE_KEYBOARD_APPLICATION),	// usage maximum (101)
	HID_Input(HID_Data|HID_Array|HID_Absolute),	// input (data, array, absolute)

	HID_UsagePage(HID_USAGE_PAGE_CONSUMER),					// usage page (consumer)
	HID_Usage(HID_USAGE_PAGE_UNDEFINED),						// usage (consumer control)
	HID_LogicalMin(0x80),														// logical minimum (-128)
	HID_LogicalMax(0x7F),														// logical maximum (+127)
	HID_ReportCount(64),															// report count (64)
	HID_ReportSize(8),															// report size (8)
	HID_Feature(HID_Data|HID_Variable|HID_Absolute),// feature (data, variable, absolute)

	HID_EndCollection,											// end collection
#endif
};
// const uint16_t HID_ReportDescSize = sizeof(HID_ReportDescriptor);
#define HID_ReportDescSize sizeof(HID_ReportDescriptor)

/*****************************************************************************
* Description	: HID_ReportDescriptor2[]
*****************************************************************************/
const uint8_t HID_ReportDescriptor2[] = {

	HID_UsagePage(HID_USAGE_PAGE_GENERIC),	// usage page (generic desktop)
	HID_Usage(HID_USAGE_GENERIC_MOUSE),			// usage (mouse)
	HID_Collection(HID_Application),				// collection (application)
	HID_ReportID(1),												// report ID	(1)
	HID_Usage(HID_USAGE_GENERIC_POINTER),		// usage (pointer)
	HID_Collection(0),											// collection (linked)
	HID_UsagePage(HID_USAGE_PAGE_BUTTON),		// usage page (buttons)
	HID_UsageMin(1),												// usage minimum (1)
	HID_UsageMax(3),												// usage maximum (3)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMax(1),											// logical maximum (1)
	HID_ReportCount(5),											// report count (5)
	HID_ReportSize(1),											// report size (1)
	HID_Input(HID_Data|HID_Variable|HID_Absolute),	// input (data, variable, absolute)
	HID_ReportCount(1),											// report count (1)
	HID_ReportSize(3),											// report size (3)
	HID_Input(HID_Constant),								// input (constant)
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),	// usage page (generic desktop)
	HID_Usage(HID_USAGE_GENERIC_X),					// usage (X)
	HID_Usage(HID_USAGE_GENERIC_Y),					// usage (Y)
	HID_Usage(HID_USAGE_GENERIC_WHEEL),			// usage (Wheel)
	HID_LogicalMin(0x81),										// logical minimum (-127)
	HID_LogicalMax(0x7F),										// logical maximum (127)
	HID_ReportSize(8),											// report size (8)
	HID_ReportCount(3),											// report count (3)
	HID_Input(HID_Data|HID_Variable|HID_Relative),	// input (data, variable, relative)
	HID_EndCollection,											// end collection
	HID_EndCollection,											// end collection

#if (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE2)
	/// system control
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),	// usage page (generic desktop)
	HID_Usage(HID_USAGE_GENERIC_SYSTEM_CTL),// usage (System Control)
	HID_Collection(HID_Application),				// collection (application)
	HID_ReportID(2),												// report ID	(2)
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),		// usage page (generic desktop)
	HID_UsageMin(HID_USAGE_GENERIC_SYSCTL_POWER),	// usage minimum (System Power Down)
	HID_UsageMax(HID_USAGE_GENERIC_SYSCTL_WAKE),	// usage maximum (System Wake Up)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMax(1),											// logical maximum (1)
	HID_ReportCount(3),											// report count (3)
	HID_ReportSize(1),											// report size (1)
	HID_Input(HID_Data|HID_Variable|HID_Absolute),	// input (data, variable, absolute)
	HID_ReportCount(1),											// report count (1)
	HID_ReportSize(5),											// report size (5)
	HID_Input(HID_Constant),								// input (constant)
	HID_EndCollection,											// end collection
	HID_UsagePage(HID_USAGE_PAGE_CONSUMER),	// usage page (consumer)
	HID_Usage(HID_USAGE_CONSUMER_CONTROL),	// usage (consumer control)
	HID_Collection(HID_Application),				// collection (application)
	HID_ReportID(3),												// report ID	(3)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMax(1),											// logical maximum (1)
	HID_Usage(HID_USAGE_CONSUMER_SCAN_NEXT_TRACK),	// usage (Scan Next Track)
	HID_Usage(HID_USAGE_CONSUMER_SCAN_PREVIOUS_TRACK),	// usage (Scan Previous Track)
	HID_Usage(HID_USAGE_CONSUMER_STOP),			// usage (Stop)
	HID_Usage(HID_USAGE_CONSUMER_PLAYPAUSE),// usage (Play/pause)
	HID_Usage(HID_USAGE_CONSUMER_VOLUME),		// usage (Volume)
	HID_Usage(HID_USAGE_CONSUMER_MUTE),			// usage (Mute)
	HID_Usage(HID_USAGE_CONSUMER_BASS),			// usage (Bass)
	HID_Usage(HID_USAGE_CONSUMER_TREBLE),		// usage (Treble)
	HID_Usage(HID_USAGE_CONSUMER_BASS_BOOST),	// usage (Bass Boost)
	HID_Usage(HID_USAGE_CONSUMER_VOLUME_INC),	// usage (Volume Increment)
	HID_Usage(HID_USAGE_CONSUMER_VOLUME_DEC),	// usage (Volume Decrement)
	HID_UsageS(HID_USAGE_CONSUMER_BASS_INC),	// usage (Bass Increment)
	HID_UsageS(HID_USAGE_CONSUMER_BASS_DEC),	// usage (Bass Decrement)
	HID_UsageS(HID_USAGE_CONSUMER_TREBLE_INC),// usage (Treble Increment)
	HID_UsageS(HID_USAGE_CONSUMER_TREBLE_DEC),// usage (Treble Decrement)
	HID_UsageS(HID_USAGE_CONSUMER_EMAIL),		// usage (Mail)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_SEARCH),// usage (WWW Search)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_HOME),	// usage (WWW Home)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_BACK),	// usage (WWW Back)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_FORWARD),	// usage (WWW Forward)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_STOP),		// usage (WWW Stop)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_REFRESH),	// usage (WWW Refresh)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_BOOKMARKS),	// usage (WWW Bookmarks)
	HID_UsageS(HID_USAGE_CONSUMER_CALCULATOR),	// usage (Calculator)
	HID_UsageS(HID_USAGE_CONSUMER_BROWSER),	// usage (Browser)
	HID_UsageS(HID_USAGE_CONSUMER_CONSUMER_CONTROL),	// usage (Consumer Control)
	HID_UsageS(HID_USAGE_CONSUMER_OPEN),		// usage (Open)
	HID_UsageS(HID_USAGE_CONSUMER_CLOSE),		// usage (Close)
	HID_UsageS(HID_USAGE_CONSUMER_SAVE),		// usage (Save)
	HID_UsageS(HID_USAGE_CONSUMER_USER),		// usage (User)
	HID_UsageS(HID_USAGE_CONSUMER_UNDO),		// usage (Undo)
	HID_Usage(HID_USAGE_CONSUMER_EJECT),		// usage (Eject)
	HID_ReportSize(1),											// report size (1)
	HID_ReportCount(32),										// report count (32)
	HID_Input(HID_Data|HID_Variable|HID_Absolute),	// input (data, variable, absolute)	81 02
	HID_EndCollection,											// end collection
#endif
};
// const uint16_t HID_ReportDescSize2 = sizeof(HID_ReportDescriptor2);
#define HID_ReportDescSize2 sizeof(HID_ReportDescSize2)

/*****************************************************************************
* Description	: HID_ReportDescriptor3[]
*****************************************************************************/
const uint8_t HID_ReportDescriptor3[] = {
		/* consumer control */
	HID_UsagePage(HID_USAGE_PAGE_CONSUMER),	// usage page (consumer)
	HID_Usage(HID_USAGE_CONSUMER_CONTROL),	// usage (consumer control)
	HID_Collection(HID_Application),				// collection (application)
	HID_ReportID(1),												// report ID	(1)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMax(1),											// logical maximum (1)
	HID_ReportCount(32),										// report count (32)
	HID_ReportSize(1),											// report size (1)
	HID_Usage(HID_USAGE_CONSUMER_SCAN_NEXT_TRACK),	// usage (Scan Next Track)
	HID_Usage(HID_USAGE_CONSUMER_SCAN_PREVIOUS_TRACK),	// usage (Scan Previous Track)
	HID_Usage(HID_USAGE_CONSUMER_STOP),			// usage (Stop)
	HID_Usage(HID_USAGE_CONSUMER_PLAYPAUSE),// usage (Play/pause)
	HID_Usage(HID_USAGE_CONSUMER_VOLUME),		// usage (Volume)
	HID_Usage(HID_USAGE_CONSUMER_MUTE),			// usage (Mute)
	HID_Usage(HID_USAGE_CONSUMER_BASS),			// usage (Bass)
	HID_Usage(HID_USAGE_CONSUMER_TREBLE),		// usage (Treble)
	HID_Usage(HID_USAGE_CONSUMER_BASS_BOOST),	// usage (Bass Boost)
	HID_Usage(HID_USAGE_CONSUMER_VOLUME_INC),	// usage (Volume Increment)
	HID_Usage(HID_USAGE_CONSUMER_VOLUME_DEC),	// usage (Volume Decrement)
	HID_UsageS(HID_USAGE_CONSUMER_BASS_INC),	// usage (Bass Increment)
	HID_UsageS(HID_USAGE_CONSUMER_BASS_DEC),	// usage (Bass Decrement)
	HID_UsageS(HID_USAGE_CONSUMER_TREBLE_INC),// usage (Treble Increment)
	HID_UsageS(HID_USAGE_CONSUMER_TREBLE_DEC),// usage (Treble Decrement)
	HID_UsageS(HID_USAGE_CONSUMER_EMAIL),		// usage (Mail)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_SEARCH),// usage (WWW Search)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_HOME),	// usage (WWW Home)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_BACK),	// usage (WWW Back)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_FORWARD),	// usage (WWW Forward)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_STOP),		// usage (WWW Stop)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_REFRESH),	// usage (WWW Refresh)
	HID_UsageS(HID_USAGE_CONSUMER_WWW_BOOKMARKS),	// usage (WWW Bookmarks)
	HID_UsageS(HID_USAGE_CONSUMER_CALCULATOR),	// usage (Calculator)
	HID_UsageS(HID_USAGE_CONSUMER_BROWSER),	// usage (Browser)
	HID_UsageS(HID_USAGE_CONSUMER_CONSUMER_CONTROL),	// usage (Consumer Control)
	HID_UsageS(HID_USAGE_CONSUMER_OPEN),		// usage (Open)
	HID_UsageS(HID_USAGE_CONSUMER_CLOSE),		// usage (Close)
	HID_UsageS(HID_USAGE_CONSUMER_SAVE),		// usage (Save)
	HID_UsageS(HID_USAGE_CONSUMER_USER),		// usage (User)
	HID_UsageS(HID_USAGE_CONSUMER_UNDO),		// usage (Undo)
	HID_Usage(HID_USAGE_CONSUMER_EJECT),		// usage (Eject)
	HID_Input(HID_Data|HID_Variable|HID_Absolute),	// input (data, variable, absolute)
	HID_EndCollection,											// end collection
	/* system control */
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),	// usage page (generic desktop)
	HID_Usage(HID_USAGE_GENERIC_SYSTEM_CTL),// usage (System Control)
	HID_Collection(HID_Application),				// collection (application)
	HID_ReportID(2),												// report ID	(2)
	HID_UsageMin(HID_USAGE_GENERIC_SYSCTL_POWER),	// usage minimum (System Power Down)
	HID_UsageMax(HID_USAGE_GENERIC_SYSCTL_WAKE),	// usage maximum (System Wake Up)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMax(1),											// logical maximum (1)
	HID_ReportSize(1),											// report size (1)
	HID_ReportCount(3),											// report count (3)
	HID_Input(HID_Data|HID_Variable|HID_Absolute),	// input (data, variable, absolute)
	HID_ReportSize(5),											// report size (5)
	HID_ReportCount(1),											// report count (1)
	HID_Input(HID_Constant),								// input (constant)
	HID_EndCollection,											// end collection
	/* Vendor hot keys */
	HID_UsagePageVendor(0),									// usage page (Vendor 0xFF00)
	HID_Usage(1),														// usage (1)
	HID_Collection(HID_Application),				// collection (application)
	HID_ReportID(3),												// report ID	(3)
	HID_UsageMinS(0xF1),										// usage minimum (241)
	HID_UsageMaxS(0xF8),										// usage maximum (248)
	HID_LogicalMin(0),											// logical minimum (0)
	HID_LogicalMax(1),											// logical maximum (1)
	HID_ReportSize(8),											// report size (8)
	HID_ReportCount(2),											// report count (2)
	HID_Input(HID_Data|HID_Variable|HID_Absolute),	// input (data, variable, absolute)
	HID_EndCollection,											// end collection
};
// const uint16_t HID_ReportDescSize3 = sizeof(HID_ReportDescriptor3);
#define HID_ReportDescSize3 sizeof(HID_ReportDescSize3)

#if (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE1)
#define nUsb_TotalLength	(USB_CONFIGUARTION_DESC_SIZE+USB_INTERFACE_DESC_SIZE*3+USB_HID_DESC_SIZE*3+USB_ENDPOINT_DESC_SIZE*3)
#define nUsb_NumInterfaces 3

#elif (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
#define nUsb_TotalLength (USB_CONFIGUARTION_DESC_SIZE+USB_INTERFACE_DESC_SIZE+USB_HID_DESC_SIZE+USB_ENDPOINT_DESC_SIZE)
#define nUsb_NumInterfaces 1

#elif (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE2)
#define nUsb_TotalLength (USB_CONFIGUARTION_DESC_SIZE+USB_INTERFACE_DESC_SIZE*2+USB_HID_DESC_SIZE*2+USB_ENDPOINT_DESC_SIZE*2)
#define nUsb_NumInterfaces 2
#endif




/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor) */
const uint8_t USB_ConfigDescriptor[] = {
/* Configuration 1 */
	USB_CONFIGUARTION_DESC_SIZE,			/* bLength */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,/* bDescriptorType */
	USB_WBVAL(nUsb_TotalLength),			/* wTotalLength */
	nUsb_NumInterfaces,								/* bNumInterfaces */
	USB_CONFIG_VALUE,									/* bConfigurationValue */
	0x00,															/* iConfiguration */
	0xA0,
	0x32,															/* bMaxPower */

	/*****************************************************************************
* Description	: Interface 0[]
*****************************************************************************/
/* Interface 0, Alternate Setting 0, HID Class */
	USB_INTERFACE_DESC_SIZE,					/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_INTERFACE_0,									/* bInterfaceNumber */
	0x00,															/* bAlternateSetting */
	0x01,															/* bNumEndpoints */
	0x03,															/* bInterfaceClass */
	HID_SUBCLASS_BOOT,								/* bInterfaceSubClass */
#if (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
	HID_PROTOCOL_MOUSE,						/* bInterfaceProtocol */
#else
	HID_PROTOCOL_KEYBOARD,
#endif
	0x00,															/* iInterface */
/* HID Class Descriptor */
	USB_HID_DESC_SIZE,								/* bLength */
	HID_HID_DESCRIPTOR_TYPE,					/* bDescriptorType */
	USB_WBVAL(0x0111), /* 1.11 */			/* bcdHID */
	0x00,															/* bCountryCode */
	0x01,															/* bNumDescriptors */
	HID_REPORT_DESCRIPTOR_TYPE,				/* bDescriptorType */
	USB_WBVAL(HID_ReportDescSize),		/* wDescriptorLength */
/* Endpoint1, HID Interrupt In */
	USB_ENDPOINT_DESC_SIZE,						/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,			/* bDescriptorType */
#if (USB_EP1_DIRECTION == USB_DIRECTION_IN)
	USB_ENDPOINT_IN(USB_EP1),					/* bEndpointAddress */
#else
	USB_ENDPOINT_OUT(USB_EP1),				/* bEndpointAddress */
#endif
	USB_ENDPOINT_TYPE_INTERRUPT,			/* bmAttributes */
	USB_WBVAL(USB_EP1_PACKET_SIZE),		/* wMaxPacketSize */
#if (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
		0x01,          /* 1ms */					/* bInterval */
#else
		0x08,          /* 1ms */					/* bInterval */
#endif

/*****************************************************************************
* Description	: Interface 1[]
*****************************************************************************/
#if ((USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE1) || (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE2) )
/* Interface 1, Alternate Setting 0, HID Class */
	USB_INTERFACE_DESC_SIZE,					/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_INTERFACE_1,									/* bInterfaceNumber */
	0x00,															/* bAlternateSetting */
	0x01,															/* bNumEndpoints */
	0x03,															/* bInterfaceClass */
	HID_SUBCLASS_BOOT,								/* bInterfaceSubClass */
	HID_PROTOCOL_MOUSE,								/* bInterfaceProtocol */
	0x00,															/* iInterface */
/* HID Class Descriptor */
	USB_HID_DESC_SIZE,								/* bLength */
	HID_HID_DESCRIPTOR_TYPE,					/* bDescriptorType */
	USB_WBVAL(0x0111), /* 1.11 */			/* bcdHID */
	0x00,															/* bCountryCode */
	0x01,															/* bNumDescriptors */
	HID_REPORT_DESCRIPTOR_TYPE,				/* bDescriptorType */
	USB_WBVAL(HID_ReportDescSize2),		/* wDescriptorLength */
/* Endpoint2, HID Interrupt In */
	USB_ENDPOINT_DESC_SIZE,						/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,			/* bDescriptorType */
#if (USB_EP2_DIRECTION == USB_DIRECTION_IN)
	USB_ENDPOINT_IN(USB_EP2),					/* bEndpointAddress */
#else
	USB_ENDPOINT_OUT(USB_EP2),				/* bEndpointAddress */
#endif
	USB_ENDPOINT_TYPE_INTERRUPT,			/* bmAttributes */
	USB_WBVAL(USB_EP2_PACKET_SIZE),		/* wMaxPacketSize */
	0x01,          /* 1ms */					/* bInterval */
#endif

/*****************************************************************************
* Description	: Interface 2[]
*****************************************************************************/
#if (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE1)
/* Interface 2, Alternate Setting 0, HID Class */
	USB_INTERFACE_DESC_SIZE,					/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_INTERFACE_2,									/* bInterfaceNumber */
	0x00,															/* bAlternateSetting */
	0x01,															/* bNumEndpoints */
	0x03,															/* bInterfaceClass */
	HID_SUBCLASS_NONE,								/* bInterfaceSubClass */
	HID_PROTOCOL_NONE,								/* bInterfaceProtocol */
	0x00,															/* iInterface */
/* HID Class Descriptor */
	USB_HID_DESC_SIZE,								/* bLength */
	HID_HID_DESCRIPTOR_TYPE,					/* bDescriptorType */
	USB_WBVAL(0x0111), /* 1.11 */			/* bcdHID */
	0x00,															/* bCountryCode */
	0x01,															/* bNumDescriptors */
	HID_REPORT_DESCRIPTOR_TYPE,				/* bDescriptorType */
	USB_WBVAL(HID_ReportDescSize3),		/* wDescriptorLength */
/* Endpoint3, HID Interrupt In */
	USB_ENDPOINT_DESC_SIZE,						/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,			/* bDescriptorType */
	#if (USB_EP3_DIRECTION == USB_DIRECTION_IN)
		USB_ENDPOINT_IN(USB_EP3),					/* bEndpointAddress */
	#else
		USB_ENDPOINT_OUT(USB_EP3),				/* bEndpointAddress */
	#endif
	USB_ENDPOINT_TYPE_INTERRUPT,			/* bmAttributes */
	USB_WBVAL(USB_EP3_PACKET_SIZE),		/* wMaxPacketSize */
	0x08,          /* 1ms */					/* bInterval */
#endif
};


/* USB String Descriptor (optional) */
/*****************************************************************************
* Description	: USB_LanguageStringDescriptor[]
*****************************************************************************/
const uint8_t USB_LanguageStringDescriptor[] = {
/* Index 0x00: LANGID Codes */
	0x04,                              /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	USB_WBVAL(0x0409), /* US English *//* wLANGID */
};

/*****************************************************************************
* Description	: USB_ManufacturerStringDescriptor[]
*****************************************************************************/
const uint8_t USB_ManufacturerStringDescriptor[] = {
/* Index 0x01: Manufacturer */
	(5*2 + 2),													/* bLength (5 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,					/* bDescriptorType */
	'S',0,
	'O',0,
	'N',0,
	'i',0,
	'X',0,
};

/*****************************************************************************
* Description	: USB_ProductStringDescriptor[]
*****************************************************************************/
const uint8_t USB_ProductStringDescriptor[] = {
/* Index 0x02: Product */
	(10*2 + 2),													/* bLength ( 10 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,					/* bDescriptorType */
	'U',0,
	'S',0,
	'B',0,
	' ',0,
	'D',0,
	'E',0,
	'V',0,
	'I',0,
	'C',0,
	'E',0,
};

/*****************************************************************************
* Description	: USB_SerialNumberStringDescriptor[]
*****************************************************************************/
const uint8_t USB_SerialNumberStringDescriptor[] = {
/* Index 0x03: Serial Number */
	(4*2 + 2),													/* bLength (4 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,					/* bDescriptorType */
	'D',0,
	'E',0,
	'M',0,
	'O',0,
};


/*****************************************************************************
* Function		: STRUCT_DESCRIPTOR_INFO_A DesInfo[]
* Description	: Array of USB Descritpor
*****************************************************************************/
STRUCT_DESCRIPTOR_INFO_A DesInfo[] = {
//////
{USB_DEVICE_DESCRIPTOR_TYPE, USB_INTERFACE_0, 0, 		//{type, interface0, StringTypes, size, descriptor_matrix}
USB_DEVICE_DESC_SIZE, USB_DeviceDescriptor}, 				//NO.1 Device descriptor
//////
{USB_CONFIGURATION_DESCRIPTOR_TYPE,USB_INTERFACE_0,0,  	//{type, interface0, StringTypes, size, descriptor_matrix}
nUsb_TotalLength, USB_ConfigDescriptor},								//NO.2 Config descriptor
/////
{USB_STRING_DESCRIPTOR_TYPE, USB_INTERFACE_0, USB_STRING_LANGUAGE, 			//{type, interface0, StringTypes, size, descriptor_matrix}
sizeof(USB_LanguageStringDescriptor), USB_LanguageStringDescriptor},    //NO.3 String Lang
/////
{USB_STRING_DESCRIPTOR_TYPE, LANG_ID_H, USB_STRING_LANGUAGE, 											//{type, interface0, StringTypes, size, descriptor_matrix}
sizeof(USB_LanguageStringDescriptor), USB_LanguageStringDescriptor},    //NO.4 String Lang
/////
{USB_STRING_DESCRIPTOR_TYPE, USB_INTERFACE_0, USB_STRING_MANUFACTURER, 				//{type, interface0, StringTypes, size, descriptor_matrix}
sizeof(USB_ManufacturerStringDescriptor), USB_ManufacturerStringDescriptor},  //NO.5 String Manu
/////
{USB_STRING_DESCRIPTOR_TYPE, LANG_ID_H, USB_STRING_MANUFACTURER, 														//{type, interface0, StringTypes, size, descriptor_matrix}
sizeof(USB_ManufacturerStringDescriptor), USB_ManufacturerStringDescriptor},  //NO.6 String Manu
/////
{USB_STRING_DESCRIPTOR_TYPE, USB_INTERFACE_0, USB_STRING_PRODUCT, 				//{type, interface0, StringTypes, size, descriptor_matrix}
sizeof(USB_ProductStringDescriptor), USB_ProductStringDescriptor}, 				//NO.7	String Product
/////
{USB_STRING_DESCRIPTOR_TYPE, LANG_ID_H, USB_STRING_PRODUCT, 												//{type, interface0, StringTypes, size, descriptor_matrix}
sizeof(USB_ProductStringDescriptor), USB_ProductStringDescriptor}, 				//NO.8	String Product
/////
{USB_ENDPOINT_DESCRIPTOR_TYPE,USB_INTERFACE_0,0,							//{type, interface0, StringTypes, size, descriptor_matrix}
USB_ENDPOINT_DESC_SIZE,ENDPOINT_DESCRIPTOR_INDEX}, 						//NO.9 Endoption1
/////
{USB_ENDPOINT_DESCRIPTOR_TYPE,USB_INTERFACE_0,0,							//{type, interface0, StringTypes, size, descriptor_matrix}
USB_ENDPOINT_DESC_SIZE,ENDPOINT_DESCRIPTOR_INDEX}, 						//NO.10 Endoption2
/////
{HID_HID_DESCRIPTOR_TYPE,USB_INTERFACE_0, 0, 									//{type, interface0, StringTypes, size, descriptor_matrix}
USB_HID_DESC_SIZE, HID_DESCRIPTOR_INDEX0},										//NO.11 HID
/////
#if (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)

#elif (USB_LIBRARY_TYPE == USB_KB_MOUSE_TYPE2)
{HID_HID_DESCRIPTOR_TYPE,USB_INTERFACE_1, 0, 							//{type, interface0, StringTypes, size, descriptor_matrix}
USB_HID_DESC_SIZE, HID_DESCRIPTOR_INDEX1},								//NO.12 HID
#else
{HID_HID_DESCRIPTOR_TYPE,USB_INTERFACE_1, 0, 				//{type, interface0, StringTypes, size, descriptor_matrix}
USB_HID_DESC_SIZE, HID_DESCRIPTOR_INDEX1},					//NO.12 HID
///
{HID_HID_DESCRIPTOR_TYPE,USB_INTERFACE_2, 0, 				//{type, interface0, StringTypes, size, descriptor_matrix}
USB_HID_DESC_SIZE, HID_DESCRIPTOR_INDEX2},					//NO.12 HID
#endif

#if (USB_LIBRARY_TYPE == USB_MOUSE_TYPE)
{HID_REPORT_DESCRIPTOR_TYPE, USB_INTERFACE_0, 0,				//{type, interface0, StringTypes, size, descriptor_matrix}
HID_ReportDescSize, HID_ReportDescriptor},							//NO.13 REPORT INT0
#else
{HID_REPORT_DESCRIPTOR_TYPE, USB_INTERFACE_0, 0, 				//{type, interface0, StringTypes, size, descriptor_matrix}
HID_ReportDescSize, HID_ReportDescriptor},							//NO.13 REPORT INT0
#endif
{HID_REPORT_DESCRIPTOR_TYPE, USB_INTERFACE_1, 0, 				//{type, interface0, StringTypes, size, descriptor_matrix}
HID_ReportDescSize2, HID_ReportDescriptor2},						//NO.13	REPORT INT1
///
{HID_REPORT_DESCRIPTOR_TYPE, USB_INTERFACE_2, 0, 				//{type, interface0, StringTypes, size, descriptor_matrix}
HID_ReportDescSize3, HID_ReportDescriptor3}							//NO.14	REPORT INT2

};

