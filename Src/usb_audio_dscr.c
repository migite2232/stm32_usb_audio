// usb_audio_dscr.c
// Copyright (c) 2017 Sugioka Y.
// This file is released under the MIT License.
// http://opensource.org/licenses/mit-license.php

#include "usb_audio_dscr.h"
#include "usb_def.h"

uint8_t USBAudio_Manufacturer_StringDscr[USBAUDIO_MANUFACTURER_STRING_DSCR_LEN] = 
{
	USBAUDIO_MANUFACTURER_STRING_DSCR_LEN,
	STRING_DSCR,
	'S', 0x00,
	'T', 0x00,
	'M', 0x00,
	'i', 0x00,
	'c', 0x00,
	'r', 0x00,
	'o', 0x00,
	'e', 0x00,
	'l', 0x00,
	'e', 0x00,
	'c', 0x00,
	't', 0x00,
	'r', 0x00,
	'o', 0x00,
	'n', 0x00,
	'i', 0x00,
	'c', 0x00,
	's', 0x00
};

uint8_t USBAudio_Product_StringDscr[USBAUDIO_PRODUCT_STRING_DSCR_LEN] = 
{
	USBAUDIO_PRODUCT_STRING_DSCR_LEN,
	STRING_DSCR,
	'S', 0x00,
	'T', 0x00,
	'M', 0x00,
	'3', 0x00,
	'2', 0x00,
	' ', 0x00,
	'A', 0x00,
	'u', 0x00,
	'd', 0x00,
	'i', 0x00,
	'o', 0x00,
	' ', 0x00,
	'C', 0x00,
	'l', 0x00,
	'a', 0x00,
	's', 0x00,
	's', 0x00
};

uint8_t USBAudio_DeviceDscr[USBAUDIO_DEVICE_DSCR_LEN] =
{
	USBAUDIO_DEVICE_DSCR_LEN, //Descriptor length
	DEVICE_DSCR, //Descriptor type
	0x00,
	0x02, //bcdUSB
	0x00, //Device class
	0x00, //Device sub class
	0x00, //Device protocol
	EP0_PACKET_SIZE, //Maximum Packet size
	LOBYTE(USB_VID),
	HIBYTE(USB_VID), //Vendor ID
	LOBYTE(USB_PID),
	HIBYTE(USB_PID), //Product ID
	0x00,
	0x00, //Product version
	1, //Manufacturer string ID
	2, //Product string ID
	0, //SerialNumber string ID
	1 //Number of configurations
};

uint8_t USBAudio_ConfigurationDscr[USBAUDIO_CONFIGURATION_DSCR_LEN] = 
{
	9, //Descriptor length
	CONFIGURATION_DSCR, //Descriptor type
	LOBYTE(USBAUDIO_CONFIGURATION_DSCR_LEN),
	HIBYTE(USBAUDIO_CONFIGURATION_DSCR_LEN), //Descriptor total size
	2, //Number of interfaces
	1, //Configuration number
	0x00, //Configuration string
	0x80 | (SELF_POWERED << 6) | (REMOTE_WAKEUP << 5), //Attributes b7-rsvd, b6-selfpwr, b5-rwu
	50, //Power requirement (100mA)
//Interface descriptor
	9, //Descriptor length
	INTERFACE_DSCR, //Descriptor type
	0x00, //Interface index
	0x00, //Alternate setting
	0x00, //Number of endpoints
	0x01, //Interface class (Audio)
	0x01, //Interface sub class (Audio control)
	0, //Interface sub sub class
	0, //Interface descriptor string index
//Audio Control Interface Header Descriptor
	9, //Descriptor length
	0x24, //Descriptor type (Audio class Interface descriptor)
	0x01, //Descriptor sub type
	0x00,
	0x01, //BCD (Audio class 1.0)
	0x1E,
	0x00, //Length (30)
	0x01, //bInCollection
	0x01, //baInterfaceNr
//Audio Control Input Terminal Descriptor
	12, //Descriptor length
	0x24, //Descriptor type (Audio class Interface descriptor)
	0x02, //Descriptor sub type
	0x01, //Terminal ID
	0x01,
	0x01, //Terminal Type (USB Streaming)
	0, //bAssocTerminal
	0x02, //bNrChannels
	0x03,
	0x00, //wChannelConfig
	0, //iChannelNames
	0, //iTerminal
//Audio Control Output Terminal Descriptor
	9, //Desecriptor length
	0x24, //Descriptor type (Audio Class Interface Descriptor)
	0x03, //Descriptor sub type
	0x02, //Terminal ID
	0x02,
	0x06, //Terminal type
	0, //bAssocTerminal
	0x01, //bSourceID
	0, //iTerminal
//Interface Descriptor
	9, //Descriptor length
	INTERFACE_DSCR, //Descriptor type
	0x01, //Interface index
	0x00, //Alternate setting
	0x00, //Number of endpoints
	0x01, //Interface class (Audio)
	0x02, //Interface sub-class (Audio streaming)
	0, //Interface sub-sub class
	0, //Interface descriptor string index
//Interface Descriptor
	9, //Descriptor length
	INTERFACE_DSCR, //Descriptor type
	0x01, //Interface index
	0x01, //Alternate setting
	0x02, //Number of endpoints
	0x01, //Interface class (Audio)
	0x02, //Interface sub-class (Audio streaming)
	0, //Interface sub-sub class
	0, //Interface descriptor string index
//Audio Streaming Class Specific Interface Descriptor
	7, //Descriptor length
	0x24, //Descriptor type (Audio Class Interface Descriptor)
	0x01, //Descriptor sub type
	0x01, //bTerminalLink
	0x04, //bDelay
	0x01,
	0x00, //PCM
//Audio Streaming Format Type Descriptor
	14, //Descriptor length
//	20, //Descriptor length
	0x24, //Descriptor type (Audio Class Interface Descriptor)
	0x02, //Descriptor sub type
	0x01, //Format L-PCM
	0x02, //bNrChannels
	0x02, //bSubFrameSize, 2[byte]
	0x10, //bBitResolution, must be (bSubFrameSize * 8)[bit]
	0x02, //bSamFreqType - Number of supported frequency - 2
//	0x04, //bSamFreqType - Number of supported frequency - 4
	0x44,
	0xAC,
	0x00, //tSamFreq0 0x00AC44 = 44100
	0x80,
	0xBB,
	0x00, //tSamFreq1 0x00BB80 = 48000
//	0x88,
//	0x58,
//	0x01, //tSamFreq2 0x015888 = 88200
//	0x00,
//	0x77,
//	0x01, //tSamFreq3 0x017700 = 96000
// Standard AS Isochronous Audio Data Endpoint Descriptor
	9, //Descriptor length
	ENDPOINT_DSCR, //Descriptor type
	0x02, //Endpoint number (EP2 OUT)
	0x05, //bmAttributes
	LOBYTE(AUDIO_PACKET_SIZE),
	HIBYTE(AUDIO_PACKET_SIZE), //wMaxPacketSize
	0x01, //Polling interval
	0x00, //bRefresh
	0x86, //bSyncAddress (EP6 IN)
//Audio Streaming Class Specific Audio Data Endpoint Descriptor
	7, //Descriptor length
	0x25, //Descriptor type
	0x01, //Descriptor sub type
	(0x80 | 0x01), //bmAttributes, MaxPacketOnly, SamplingFrequencyControl
	0x00, //bLockDelayUnits
	0x00,
	0x00, //bLockDelay
//Standard AS Isochronous Synch Endpoint Descriptor
	9, //Descriptor length
	ENDPOINT_DSCR, //Descriptor type
	0x86, //Endpoint number (EP6 IN)
	0x01, //bmAttributes
	LOBYTE(FEEDBACK_PACKET_SIZE), 
	HIBYTE(FEEDBACK_PACKET_SIZE), //wMaxPacketSize (3byte)
	0x01, //Polling interval
	FEEDBACK_RATE, //bRefresh 
	0 //bSyncAddress
};
