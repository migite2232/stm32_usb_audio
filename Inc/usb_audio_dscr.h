// usb_audio_dscr.h
// Copyright (c) 2017 Sugioka Y.
// This file is released under the MIT License.
// http://opensource.org/licenses/mit-license.php

#include "stm32f1xx_hal.h"

#ifndef __USB_AUDIO_DSCR_H
#define __USB_AUDIO_DSCR_H

#define USB_VID		1155 //STMicroelectronics
//#define USB_PID		22336 //Test device
#define USB_PID		22337

//Attributes
#define SELF_POWERED	0
#define REMOTE_WAKEUP	0

//STM32F103 has 512 bytes PMA
#define PMA_SIZE		512

//Control packet size
#define EP0_PACKET_SIZE		8
//Audio packet size
#define AUDIO_PACKET_SIZE	200 //(48+1)*2*2=196
//Feedback packet size, must be 3
#define FEEDBACK_PACKET_SIZE	3

//Audio packet buffering length
#define AUDIO_BUFFER_LEN	16
//
#define FEEDBACK_RATE		5 //1-9, 2^5=32[ms]

//---------------------------------

#define USBAUDIO_MANUFACTURER_STRING_DSCR_LEN	38
#define USBAUDIO_PRODUCT_STRING_DSCR_LEN		36

#define USBAUDIO_DEVICE_DSCR_LEN				18
#define USBAUDIO_CONFIGURATION_DSCR_LEN			112

#define USBAUDIO_MANUFACTURER_STRING_DSCR_ID	1
#define USBAUDIO_PRODUCT_STRING_DSCR_ID			2

extern uint8_t USBAudio_Manufacturer_StringDscr[];
extern uint8_t USBAudio_Product_StringDscr[];
extern uint8_t USBAudio_DeviceDscr[];
//extern uint8_t USBAudio_DeviceQualifierDscr[];
extern uint8_t USBAudio_ConfigurationDscr[];

#endif