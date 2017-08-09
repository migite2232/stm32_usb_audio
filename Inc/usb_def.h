// usb_def.h
// Copyright (c) 2017 Sugioka Y.
// This file is released under the MIT License.
// http://opensource.org/licenses/mit-license.php

#ifndef __USB_DEF_H
#define __USB_DEF_H

#ifndef NULL
#define NULL  0
#endif

#ifndef LOBYTE
#define LOBYTE(x) (x & 0xFF)
#endif

#ifndef HIBYTE
#define HIBYTE(x) ((x >> 8) & 0xFF)
#endif

#ifndef MIN
#define MIN(x,y)	(x <= y ? x : y)
#endif

#define DEVICE_DSCR				0x01
#define CONFIGURATION_DSCR		0x02
#define STRING_DSCR				0x03
#define INTERFACE_DSCR			0x04
#define ENDPOINT_DSCR			0x05
#define DEVICE_QUALIFIER_DSCR	0x06
#define OTHER_SPEED_CONFIGURATION_DSCR 0x07

#define EP_TYPE_CONTROL		0
#define EP_TYPE_ISOCHRONOUS 1
#define EP_TYPE_BULK		2
#define EP_TYPE_INTERRUPT	3

#define USB_REQ_TYPE_STANDARD		(0<<5)
#define USB_REQ_TYPE_CLASS			(1<<5)
#define USB_REQ_TYPE_VENDOR			(2<<5)
#define USB_REQ_TYPE_DEVICE			0
#define USB_REQ_TYPE_INTERFACE		1
#define USB_REQ_TYPE_ENDPOINT		2
#define USB_REQ_STD_SET_ADDRESS		0x05
#define USB_REQ_STD_GET_DESCRIPTOR	0x06
#define USB_REQ_STD_SET_DESCRIPTOR	0x07
#define USB_REQ_STD_CLEAR_FEATURE	0x01
#define USB_REQ_STD_SET_FEATURE		0x03
#define USB_REQ_STD_GET_CONFIGURATION	0x08
#define USB_REQ_STD_SET_CONFIGURATION	0x09
#define USB_REQ_STD_GET_INTERFACE	0x0A
#define USB_REQ_STD_SET_INTERFACE	0x0B
#define USB_REQ_STD_GET_STATUS		0x00
#define USB_REQ_STD_SYNCH_FRAME		0x0C

#define USB_REQ_STD_GD_DEVICE			0x01
#define USB_REQ_STD_GD_CONFIGURATION	0x02
#define USB_REQ_STD_GD_STRING			0x03
#define USB_REQ_STD_GD_DEVICE_QUALIFIER 0x06
#define USB_REQ_STD_GD_OTHER_SPEED_CONFIGURATION 0x07

#define USB_REQ_STD_FT_REMOTEWAKEUP	0x02

//Class request
#define USB_REQ_CLASS_SET_CUR	0x01
#define USB_REQ_CLASS_GET_CUR	0x81

#endif