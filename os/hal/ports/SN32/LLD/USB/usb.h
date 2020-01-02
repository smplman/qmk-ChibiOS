/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usb.h
 * Purpose: USB Definitions
 * Version: V1.20
 *----------------------------------------------------------------------------*/

#ifndef __USB_H__
#define __USB_H__

// #include "..\compiler.h"

#define USB_EP0_PACKET_SIZE 			64					// only 8, 64
#define USB_ENDPOINT_NUM				0x7F
#define	USB_SETREPORT_SIZE				USB_SETUP_PACKET_SIZE/4

/* EP1~EP6 Direction define */
#define USB_EP1_DIRECTION					1					// IN = 1; OUT = 0
#define USB_EP2_DIRECTION					1					// IN = 1; OUT = 0
#define USB_EP3_DIRECTION					1					// IN = 1; OUT = 0
#define USB_EP4_DIRECTION					1					// IN = 1; OUT = 0
#define USB_EP5_DIRECTION					1					// IN = 1; OUT = 0
#define USB_EP6_DIRECTION					1					// IN = 1; OUT = 0

/* EP1~EP6 Transfer mode define */
#define USB_INTERRUPT_MODE				0					// INTERRUPT Transfer
#define USB_BULK_MODE					1					// BULK Transfer
#define USB_ISOCHRONOUS_MODE			2					// ISOCHRONOUS Transfer

#define USB_EP1_TRANSFER_MODE			0					// INTERRUPT = 0; BULK = 0; ISO = 1
#define USB_EP1_TRANSFER_MODE			0					// INTERRUPT = 0; BULK = 0; ISO = 1
#define USB_EP1_TRANSFER_MODE			0					// INTERRUPT = 0; BULK = 0; ISO = 1
#define USB_EP1_TRANSFER_MODE			0					// INTERRUPT = 0; BULK = 0; ISO = 1
#define USB_EP1_TRANSFER_MODE			0					// INTERRUPT = 0; BULK = 0; ISO = 1
#define USB_EP1_TRANSFER_MODE			0					// INTERRUPT = 0; BULK = 0; ISO = 1

/* bmRequestType.Dir */
#define REQUEST_HOST_TO_DEVICE		(0<<7)
#define REQUEST_DEVICE_TO_HOST		(1<<7)
#define REQUEST_DIR_MASK		    (1<<7)

/* bmRequestType.Type */
#define REQUEST_STANDARD					(0<<5)
#define REQUEST_CLASS						(1<<5)
#define REQUEST_VENDOR						(2<<5)
#define REQUEST_RESERVED					(3<<5)
#define REQUEST_TYPE_MASK					(3<<5)

/* bmRequestType.Recipient */
#define REQUEST_TO_DEVICE					0
#define REQUEST_TO_INTERFACE			    1
#define REQUEST_TO_ENDPOINT				    2
#define REQUEST_TO_OTHER					3
#define REQUEST_MASK						3

/* USB Standard Request Codes */
#define USB_REQUEST_GET_STATUS									0
#define USB_REQUEST_CLEAR_FEATURE								1
#define USB_REQUEST_SET_FEATURE									3
#define USB_REQUEST_SET_ADDRESS									5
#define USB_REQUEST_GET_DESCRIPTOR								6
#define USB_REQUEST_SET_DESCRIPTOR								7
#define USB_REQUEST_GET_CONFIGURATION							8
#define USB_REQUEST_SET_CONFIGURATION							9
#define USB_REQUEST_GET_INTERFACE								10
#define USB_REQUEST_SET_INTERFACE								11
#define USB_REQUEST_SYNC_FRAME									12

/* USB GET_STATUS Bit Values */
#define USB_GETSTATUS_SELF_POWERED								0x01
#define USB_GETSTATUS_REMOTE_WAKEUP								0x02
#define USB_GETSTATUS_ENDPOINT_STALL							0x01

/* USB Standard Feature selectors */
#define USB_FEATURE_ENDPOINT_STALL								0
#define USB_FEATURE_REMOTE_WAKEUP								1

/* USB Descriptor Types */
#define USB_DEVICE_DESCRIPTOR_TYPE								1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE					    2
#define USB_STRING_DESCRIPTOR_TYPE								3
#define USB_INTERFACE_DESCRIPTOR_TYPE							4
#define USB_ENDPOINT_DESCRIPTOR_TYPE							5
#define USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE			        6
#define USB_OTHER_SPEED_CONFIG_DESCRIPTOR_TYPE		            7
#define USB_INTERFACE_POWER_DESCRIPTOR_TYPE				        8
#define USB_OTG_DESCRIPTOR_TYPE									9
#define USB_DEBUG_DESCRIPTOR_TYPE								10
#define USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE	            11
/* Wireless USB extension Descriptor Type. */
#define USB_SECURITY_TYPE										12
#define USB_KEY_TYPE											13
#define USB_ENCRIPTION_TYPE										14
#define USB_BOS_TYPE											15
#define USB_DEVICE_CAPABILITY_TYPE								16
#define USB_WIRELESS_ENDPOINT_COMPANION_TYPE			        17

/* USB Protocol Value */
#define USB_BOOT_PROTOCOL										0
#define USB_REPORT_PROTOCOL										1

#define USB_IDLE_TIME_INITIAL									0x7D	// 125*4 = 500ms

/* USB Interface Address */
#define USB_INTERFACE_0												0x0
#define USB_INTERFACE_1												0x1
#define USB_INTERFACE_2												0x2
#define USB_INTERFACE_3												0x3
#define USB_INTERFACE_4												0x4
#define USB_INTERFACE_5												0x5

/* USB Endpoint Address */
#define USB_EP0																0x0
#define USB_EP1																0x1
#define USB_EP2																0x2
#define USB_EP3																0x3
#define USB_EP4																0x4
#define USB_EP5																0x5
#define USB_EP6																0x6

/* USB Endpoint Direction */
#define USB_DIRECTION_OUT											0
#define USB_DIRECTION_IN											1

/* USB Endpoint Max Packet Size */
#define USB_EP1_PACKET_SIZE										0x08
#define USB_EP2_PACKET_SIZE										0x08
#define USB_EP3_PACKET_SIZE										0x08
#define USB_EP4_PACKET_SIZE										0x08
#define USB_EP5_PACKET_SIZE										0x08
#define USB_EP6_PACKET_SIZE										0x08

/* USB Endpoint Halt Value */
#define USB_EPn_NON_HALT										0x0
#define USB_EPn_HALT											0x1

#define	LANG_ID_H												0x09
#define	LANG_ID_L												0X04

#endif  /* __USB_H__ */
