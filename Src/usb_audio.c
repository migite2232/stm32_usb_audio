// usb_audio.c
// Copyright (c) 2017 Sugioka Y.
// This file is released under the MIT License.
// http://opensource.org/licenses/mit-license.php

#include "stm32f1xx_hal.h"
#include "usb_audio.h"
#include "usb_audio_dscr.h"
#include "usb_def.h"

#define ZPO	1638 // Zero point One, 0.1*2^14 = 1638.4

PCD_HandleTypeDef *hpcd_usb_audio;

USBAudio_CallbackFunc Callback_StartPlayback;
USBAudio_CallbackFunc Callback_StopPlayback;
USBAudio_CallbackFunc Callback_FreqChanged;

uint8_t aubuf[AUDIO_BUFFER_LEN][AUDIO_PACKET_SIZE];
uint16_t aubuf_size[AUDIO_BUFFER_LEN];
uint8_t aubuf_count;
uint8_t aubuf_raddr;
uint8_t aubuf_waddr;
uint8_t aubuf_ravail;
uint8_t aubuf_wavail;

uint8_t SETUP_bmRequestType;
uint8_t SETUP_bRequest;
uint16_t SETUP_wValue;
uint16_t SETUP_wIndex;
uint16_t SETUP_wLength;

uint8_t ep0_out_buf[8]; //Now only used for SET_CUR - SAMPLING_FREQ_CONTROL request (wLength=3)

uint8_t *ep0_in_buf;
uint16_t ep0_in_len;
uint16_t ep0_in_zlp; //Transmit Zero length Packet flag

uint8_t remote_wakeup;
uint8_t configuration;
uint8_t interface;
uint32_t audio_freq;

uint8_t buffer_overflow;
uint8_t callback_audio_playing;

//--------------------------------

static void EP0_Transmit(PCD_HandleTypeDef *hpcd, uint8_t *ptr, uint16_t len);
static uint8_t EP0_Transmit_Handle(PCD_HandleTypeDef *hpcd);
static void EP0_Transmit_Reset(void);

static void AudioBuffer_Init(void);
static void AudioBuffer_Reset(void);
static void AudioBuffer_IncrementWaddr(void);
static void AudioBuffer_IncrementRaddr(void);

//--------------------------------

void USBAudio_Init(PCD_HandleTypeDef *hpcd)
{
	hpcd_usb_audio = hpcd;
	//Configurate Endpoint buffers
	// I found one of implimentation about HAL_PCDEx_PMAConfig funtion which seems fine
	// https://developer.mbed.org/users/devanlai/code/USBDevice_STM32F103/raw-file/390c4a31db54/USBDevice/USBHAL_STM32F1.cpp
	uint16_t pmaaddr = PMA_SIZE; //tail of PMA
	pmaaddr -= EP0_PACKET_SIZE;
	HAL_PCDEx_PMAConfig(hpcd, PCD_ENDP0 | 0x00, PCD_SNG_BUF, pmaaddr);
	pmaaddr -= EP0_PACKET_SIZE;
	HAL_PCDEx_PMAConfig(hpcd, PCD_ENDP0 | 0x80, PCD_SNG_BUF, pmaaddr);
	//Isochronous endpoint must be double buffered (RM0008 22.4.4)
	pmaaddr -= AUDIO_PACKET_SIZE;
	pmaaddr -= AUDIO_PACKET_SIZE;
	HAL_PCDEx_PMAConfig(hpcd, PCD_ENDP2 | 0x00, PCD_DBL_BUF, pmaaddr | ((pmaaddr+AUDIO_PACKET_SIZE) << 16));
	pmaaddr -= 4;
	pmaaddr -= 4;
	HAL_PCDEx_PMAConfig(hpcd, PCD_ENDP6 | 0x80, PCD_DBL_BUF, pmaaddr | ((pmaaddr+4) << 16));
	
	//Init Audio buffer
	AudioBuffer_Init();
	//Callbacks
	Callback_StartPlayback = NULL;
	Callback_StopPlayback = NULL;
	Callback_FreqChanged = NULL;
}

void USBAudio_RegisterCallback(USBAudio_CallbackFunc startPlayback,
								USBAudio_CallbackFunc stopPlayback,
								USBAudio_CallbackFunc freqChanged)
{
	Callback_StartPlayback = startPlayback;
	Callback_StopPlayback = stopPlayback;
	Callback_FreqChanged = freqChanged;
}

void USBAudio_DeInit(void)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
	//Close EP0
	HAL_PCD_EP_Close(hpcd, PCD_ENDP0 | 0x00);
	HAL_PCD_EP_Close(hpcd, PCD_ENDP0 | 0x80);
	HAL_PCD_EP_Close(hpcd, PCD_ENDP2 | 0x00);
	HAL_PCD_EP_Close(hpcd, PCD_ENDP6 | 0x80);
	hpcd_usb_audio = NULL;
	Callback_StartPlayback = NULL;
	Callback_StopPlayback = NULL;
	Callback_FreqChanged = NULL;
}

void USBAudio_Start(void)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
    __HAL_RCC_USB_CLK_ENABLE();
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
	HAL_PCD_Start(hpcd);
}

void USBAudio_Stop(void)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
	HAL_PCD_Stop(hpcd);
    __HAL_RCC_USB_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
}

uint32_t USBAudio_AudioFreq(void)
{
	return audio_freq;
}

uint8_t USBAudio_Enough(void)
{
	//When aubuf_count is low, the number of coming packet exceeds playing, so it is not needed to wait buffering.
	return aubuf_count > 0;
//	return aubuf_count > (AUDIO_BUFFER_LEN >> 1);
}

uint8_t* USBAudio_GetPacketBuffer(uint16_t *length)
{
	if (!aubuf_ravail)
	{
		*length = 0;
		return NULL;
	}
	*length = aubuf_size[aubuf_raddr];
	uint8_t *buf = aubuf[aubuf_raddr];
	AudioBuffer_IncrementRaddr();
	return buf;
}

//----------------------------

static void ReceiveAudioPacket(void)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
	uint16_t rcvd_len = HAL_PCD_EP_GetRxCount(hpcd, PCD_ENDP2);
	if (rcvd_len != 0)
	{
		if (aubuf_wavail)
		{
			aubuf_size[aubuf_waddr] = rcvd_len;
			AudioBuffer_IncrementWaddr();
		}
		else
			buffer_overflow = 1;
	}
	//Receive next audio packet
	HAL_PCD_EP_Receive(hpcd, PCD_ENDP2, aubuf[aubuf_waddr], AUDIO_PACKET_SIZE);
	
	if (USBAudio_Enough())
	{
		if (!callback_audio_playing && Callback_StartPlayback != NULL) Callback_StartPlayback();
		callback_audio_playing = 1;
	}
}

static void TransmitFeedback(void)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
	// see audio10.pdf 3.7.2.2 and usb11.pdf 5.10.4.2
	uint32_t fb;
	uint8_t major = aubuf_count > ((AUDIO_BUFFER_LEN >> 1) + 1);
	uint8_t minor = aubuf_count < ((AUDIO_BUFFER_LEN >> 1) - 1);
	switch (audio_freq)
	{
		case 192000: //192kHz
			fb = major ? (191 << 14) | 0*ZPO : minor ? (193 << 14) | 0*ZPO : (192 << 14) | 0*ZPO; //10.14 format
			break;
		case 176400: //176.4kHz
			fb = major ? (175 << 14) | 4*ZPO : minor ? (177 << 14) | 4*ZPO : (176 << 14) | 4*ZPO;
			break;
		case 96000: //96kHz
			fb = major ? (95 << 14) | 0*ZPO : minor ? (97 << 14) | 0*ZPO : (96 << 14) | 0*ZPO;
			break;
		case 88200: //88.2kHz
			fb = major ? (87 << 14) | 2*ZPO : minor ? (89 << 14) | 2*ZPO : (88 << 14) | 2*ZPO;
			break;
		case 48000: //48kHz
			fb = major ? (47 << 14) | 0*ZPO : minor ? (49 << 14) | 0*ZPO : (48 << 14) | 0*ZPO;
			break;
		case 44100: //44.1kHz
			fb = major ? (43 << 14) | 1*ZPO : minor ? (45 << 14) | 1*ZPO : (44 << 14) | 1*ZPO;
			break;
		default:
			fb = 0;
			break;
	}
	uint8_t buf[FEEDBACK_PACKET_SIZE];
	buf[0] = fb & 0xFF;
	buf[1] = (fb >> 8) & 0xFF;
	buf[2] = (fb >> 16) & 0xFF;
	HAL_PCD_EP_Transmit(hpcd, PCD_ENDP6 | 0x80, buf, FEEDBACK_PACKET_SIZE);
}

static void SetAudioFrequency(uint32_t freq)
{
	uint32_t oldFreq = audio_freq;
	audio_freq = freq;
	TransmitFeedback();
	
	if (oldFreq != freq)
		if (Callback_FreqChanged != NULL) Callback_FreqChanged();
}

static void GetAudioFrequency(uint32_t* freq)
{
	*freq = audio_freq;
}

static void StartPlayback(void)
{
	AudioBuffer_Reset();
	buffer_overflow = 0;
	
	ReceiveAudioPacket();
}

static void StopPlayback(void)
{
	if (callback_audio_playing && Callback_StopPlayback != NULL) Callback_StopPlayback();
	callback_audio_playing = 0;
	
	AudioBuffer_Reset();
	buffer_overflow = 0;
}

static void DeviceReset(void)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
	//Default address
	HAL_PCD_SetAddress(hpcd, 0);
	//Open EP0
	HAL_PCD_EP_Open(hpcd, PCD_ENDP0 | 0x00, EP0_PACKET_SIZE, EP_TYPE_CONTROL); //OUT
	HAL_PCD_EP_Open(hpcd, PCD_ENDP0 | 0x80, EP0_PACKET_SIZE, EP_TYPE_CONTROL); //IN
	
	AudioBuffer_Reset();
	EP0_Transmit_Reset();
	
	remote_wakeup = 0;
	configuration = 0;
	interface = 0;
	audio_freq = 0;
	buffer_overflow = 0;
	
	if (callback_audio_playing && Callback_StopPlayback != NULL) Callback_StopPlayback();
	callback_audio_playing = 0;
}

static void DeviceConfigurated(void)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
	//Streaming Endpoint
	HAL_PCD_EP_Open(hpcd, PCD_ENDP2 | 0x00, AUDIO_PACKET_SIZE, EP_TYPE_ISOCHRONOUS); //OUT
	//Feedback Endpoint
	HAL_PCD_EP_Open(hpcd, PCD_ENDP6 | 0x80, FEEDBACK_PACKET_SIZE, EP_TYPE_ISOCHRONOUS); //IN
}

static void SetupStandardRequest(uint8_t *stall)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
	switch (SETUP_bRequest)
	{
		case USB_REQ_STD_SET_ADDRESS:
			//Handle in DataInCallback
			break;
		case USB_REQ_STD_GET_DESCRIPTOR:
		{
			uint8_t *dscr_ptr = NULL;
			uint16_t dscr_len = 0;
			switch (HIBYTE(SETUP_wValue))
			{
				case USB_REQ_STD_GD_DEVICE:
					dscr_ptr = USBAudio_DeviceDscr;
					dscr_len = USBAUDIO_DEVICE_DSCR_LEN;
					break;
				case USB_REQ_STD_GD_DEVICE_QUALIFIER:
					*stall = 1;
					//High-speed device only
					//dscr_ptr = USBAudio_DeviceQualifierDscr;
					//dscr_len = USBAUDIO_DEVICE_QUALIFIER_DSCR_LEN;
					break;
				case USB_REQ_STD_GD_CONFIGURATION:
					dscr_ptr = USBAudio_ConfigurationDscr;
					dscr_len = USBAUDIO_CONFIGURATION_DSCR_LEN;
					break;
				case USB_REQ_STD_GD_OTHER_SPEED_CONFIGURATION:
					*stall = 1;
					//High-speed device only
					//dscr_ptr = USBAudio_OtherSpeedConfigurationDscr;
					//dscr_len = USBAUDIO_OTHER_SPEED_CONFIGURATION_DSCR_LEN;
					break;
				case USB_REQ_STD_GD_STRING:
					switch (LOBYTE(SETUP_wValue))
					{
						case USBAUDIO_MANUFACTURER_STRING_DSCR_ID:
							dscr_ptr = USBAudio_Manufacturer_StringDscr;
							dscr_len = USBAUDIO_MANUFACTURER_STRING_DSCR_LEN;
							break;
						case USBAUDIO_PRODUCT_STRING_DSCR_ID:
							dscr_ptr = USBAudio_Product_StringDscr;
							dscr_len = USBAUDIO_PRODUCT_STRING_DSCR_LEN;
							break;
						default:
							*stall = 1;
							break;
					}
					break;
				default:
					*stall = 1;
					break;
			}
			if (dscr_ptr != NULL)
				EP0_Transmit(hpcd, dscr_ptr, MIN(dscr_len, SETUP_wLength));
			else
				*stall = 1;
			break;
		}
		case USB_REQ_STD_SET_DESCRIPTOR:
			*stall = 1;
			break;
		case USB_REQ_STD_CLEAR_FEATURE:
			switch (SETUP_bmRequestType & 0x1F) // [4:0]
			{
				case USB_REQ_TYPE_DEVICE:
					if (SETUP_wValue == USB_REQ_STD_FT_REMOTEWAKEUP)
						remote_wakeup = 0;
					else
						*stall = 1;
					break;
				case USB_REQ_TYPE_ENDPOINT:
					HAL_PCD_EP_ClrStall(hpcd, SETUP_wIndex & 0xFF);
					break;
				case USB_REQ_TYPE_INTERFACE:
					*stall = 1;
					break;
				default:
					break;
			}
			break;
		case USB_REQ_STD_SET_FEATURE:
			switch (SETUP_bmRequestType & 0x1F) // [4:0]
			{
				case USB_REQ_TYPE_DEVICE:
					if (SETUP_wValue == USB_REQ_STD_FT_REMOTEWAKEUP)
						remote_wakeup = 1;
					else
						*stall = 1;
					break;
				case USB_REQ_TYPE_ENDPOINT:
					HAL_PCD_EP_SetStall(hpcd, SETUP_wIndex & 0xFF);
					break;
				case USB_REQ_TYPE_INTERFACE:
					*stall = 1;
					break;
				default:
					break;
			}
			break;
		case USB_REQ_STD_GET_INTERFACE:
			HAL_PCD_EP_Transmit(hpcd, PCD_ENDP0 | 0x80, &interface, 1);
			break;
		case USB_REQ_STD_SET_INTERFACE:
			interface = (uint8_t)(SETUP_wValue & 0xFF);
			if (interface == 0) StopPlayback(); //Mute
			if (interface == 1) StartPlayback(); //Play
			break;
		case USB_REQ_STD_GET_CONFIGURATION:
			HAL_PCD_EP_Transmit(hpcd, PCD_ENDP0 | 0x80, &configuration, 1);
			break;
		case USB_REQ_STD_SET_CONFIGURATION:
			configuration = (uint8_t)(SETUP_wValue & 0xFF);
			break;
		case USB_REQ_STD_GET_STATUS:
		{
			uint8_t buf[2] = {0x00, 0x00};
			switch (SETUP_bmRequestType & 0x1F) // [4:0]
			{
				case USB_REQ_TYPE_DEVICE:
					buf[0] = (remote_wakeup << 1) | SELF_POWERED;
					break;
				case USB_REQ_TYPE_ENDPOINT:
					break;
				case USB_REQ_TYPE_INTERFACE:
					break;
				default:
					break;
			}
			HAL_PCD_EP_Transmit(hpcd, PCD_ENDP0 | 0x80, buf, SETUP_wLength);
			break;
		}
		default:
			*stall = 1;
			break;
	}
}

static void SetupClassRequest(uint8_t *stall)
{
	PCD_HandleTypeDef *hpcd = hpcd_usb_audio;
	switch (SETUP_bmRequestType & 0x1F) // [4:0]
	{
		case USB_REQ_TYPE_ENDPOINT:
			switch(SETUP_bRequest)
			{
				case USB_REQ_CLASS_SET_CUR: //0x01
					if (HIBYTE(SETUP_wValue) == 0x01 && // CS == SAMPLING_FREQ_CONTROL
						SETUP_wLength == 3)
					{
						//SAMPLING_FREQ_CONTROL
						HAL_PCD_EP_Receive(hpcd, PCD_ENDP0, ep0_out_buf, SETUP_wLength);
						//Handle in DataOutCallback
					}
					else
						*stall = 1;
					break;
				case USB_REQ_CLASS_GET_CUR: //0x81
					if (HIBYTE(SETUP_wValue) == 0x01 && // CS == SAMPLING_FREQ_CONTROL
						SETUP_wLength == 3)
					{
						//SAMPLING_FREQ_CONTROL
						uint32_t freq;
						GetAudioFrequency(&freq);
						uint8_t buf[3] = {freq & 0xFF, (freq >> 8) & 0xFF, (freq >> 16) & 0xFF};
						HAL_PCD_EP_Transmit(hpcd, PCD_ENDP0 | 0x80, buf, SETUP_wLength);
					}
					else
						*stall = 1;
					break;
				default:
					*stall = 1;
					break;
			}
			break;
		default:
			*stall = 1;
			break;
	}
}

static void SetupVendorRequest(uint8_t *stall)
{
	*stall = 1;
}

// -----------------PCD Callback------------------

void USBAudio_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
	if (hpcd != hpcd_usb_audio) return;
	uint8_t* setup_dat = (uint8_t*)(hpcd->Setup);
	SETUP_bmRequestType = (uint8_t)setup_dat[0];
	SETUP_bRequest = (uint8_t)setup_dat[1];
	SETUP_wValue = (uint16_t)setup_dat[2] | (uint16_t)(setup_dat[3] << 8);
	SETUP_wIndex = (uint16_t)setup_dat[4] | (uint16_t)(setup_dat[5] << 8);
	SETUP_wLength = (uint16_t)setup_dat[6] | (uint16_t)(setup_dat[7] << 8);
	
	uint8_t stall = 0;
	
	switch (SETUP_bmRequestType & 0x60) // [6:5]
	{
		case USB_REQ_TYPE_STANDARD:
			SetupStandardRequest(&stall);
			break;
		case USB_REQ_TYPE_CLASS:
			SetupClassRequest(&stall);
			break;
		case USB_REQ_TYPE_VENDOR:
			SetupVendorRequest(&stall);
			break;
		default:
			stall = 1;
			break;
	}
	
	if (stall)
	{
		//Set stall to EP0(IN/OUT)
		HAL_PCD_EP_SetStall(hpcd, PCD_ENDP0 | 0x00);
		HAL_PCD_EP_SetStall(hpcd, PCD_ENDP0 | 0x80);
	}
	else
	{
		if ((SETUP_bmRequestType & 0x80) != 0x80 && //Control OUT transfer (Status stage=IN)
			SETUP_wLength == 0) //No data packets
		{
			//Transmit status packet
			HAL_PCD_EP_Transmit(hpcd, PCD_ENDP0 | 0x80, NULL, 0);
		}
	}
}

void USBAudio_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	if (hpcd != hpcd_usb_audio) return;
	if (epnum == 0)
	{
		//EP0
		if ((SETUP_bmRequestType & 0x80) != 0x80) //Control transfer DATA direction = OUT
		{
			//Data packet receive completed
			uint8_t tx_status = 0;
			if ((SETUP_bmRequestType & 0x1F) == USB_REQ_TYPE_ENDPOINT &&
				SETUP_bRequest == USB_REQ_CLASS_SET_CUR &&
				HIBYTE(SETUP_wValue) == 0x01)
			{
				//SAMPLING_FREQ_CONTROL
				uint32_t freq = ep0_out_buf[0] | (ep0_out_buf[1] << 8) | (ep0_out_buf[2] << 16);
				SetAudioFrequency(freq);
				tx_status = 1;
			}
			if (tx_status)
				//Transmit status packet
				HAL_PCD_EP_Transmit(hpcd, PCD_ENDP0 | 0x80, NULL, 0);
		}
		else
		{
			//Status packet receive completed
			EP0_Transmit_Reset();
		}
	}
	else if (epnum == 0x02)
	{
		//EP2 Isochronous Audio Data Stream Endpoint
		ReceiveAudioPacket();
	}
	else
		HAL_PCD_EP_SetStall(hpcd, epnum);
}

void USBAudio_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	if (hpcd != hpcd_usb_audio) return;
	if (epnum == 0)
	{
		//EP0
		if ((SETUP_bmRequestType & 0x80) == 0x80) //Control transfer DATA direction = IN
		{
			//Data packet transmit completed
			if (EP0_Transmit_Handle(hpcd))
			{
				//All data packets transmit completed
				
			}
			//Receive status packet
			// コントロール転送が正常に機能していればパケット送信完了後に呼ぶだけでいいはずだが, なぜかダメ.
			// Middlewareの実装も同じ. どうやらゴミパケットがあるらしい?
			HAL_PCD_EP_Receive(hpcd, PCD_ENDP0, NULL, 0);
		}
		else
		{
			//Status packet transmit completed
			if (SETUP_bRequest == USB_REQ_STD_SET_ADDRESS)
			{
				//SET_ADDRESS request must be handled after Status stage
				HAL_PCD_SetAddress(hpcd, SETUP_wValue & 0xFF);
				//
				DeviceConfigurated();
			}
		}
	}
	else if (epnum == 0x06)
	{
		//EP6 Isochronous Audio Synch Endpoint
		TransmitFeedback();
	}
	else
		HAL_PCD_EP_SetStall(hpcd, epnum | 0x80);
}

void USBAudio_SOFCallback(PCD_HandleTypeDef *hpcd)
{
	if (hpcd != hpcd_usb_audio) return;
}

void USBAudio_ResetCallback(PCD_HandleTypeDef *hpcd)
{
	if (hpcd != hpcd_usb_audio) return;
	DeviceReset();
}

void USBAudio_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
	if (hpcd != hpcd_usb_audio) return;
}

void USBAudio_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
	if (hpcd != hpcd_usb_audio) return;
}

void USBAudio_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	if (hpcd != hpcd_usb_audio) return;
}

void USBAudio_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	if (hpcd != hpcd_usb_audio) return;
}

void USBAudio_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
	if (hpcd != hpcd_usb_audio) return;
}

void USBAudio_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
	if (hpcd != hpcd_usb_audio) return;
}

//--------------------------

static void EP0_Transmit(PCD_HandleTypeDef *hpcd, uint8_t *ptr, uint16_t len)
{
	uint8_t epnum = PCD_ENDP0;
	uint16_t len_max = EP0_PACKET_SIZE;
	if (len_max == len)
		//If last packet size equals max packet size, zero length packet is transmitted
		// after last packet to notify the host end of transmit.
		ep0_in_zlp = 1;
	uint16_t k = MIN(len_max, len);
	HAL_PCD_EP_Transmit(hpcd, epnum | 0x80, ptr, k);
	ep0_in_buf = ptr + k;
	ep0_in_len = len - k;
}

static uint8_t EP0_Transmit_Handle(PCD_HandleTypeDef *hpcd)
{
	if (ep0_in_zlp || ep0_in_len > 0)
	{
		if (ep0_in_zlp) ep0_in_zlp = 0;
		//Transmit next packet
		EP0_Transmit(hpcd, ep0_in_buf, ep0_in_len);
	}
	else
		//Transmit complete
		return 1;
	return 0;
}

static void EP0_Transmit_Reset(void)
{
	ep0_in_buf = NULL;
	ep0_in_len = 0;
}

static void AudioBuffer_Init(void)
{
	AudioBuffer_Reset();
}

static void AudioBuffer_Reset(void)
{
	aubuf_count = 0;
	aubuf_raddr = 0;
	aubuf_waddr = 0;
	aubuf_ravail = 0;
	aubuf_wavail = 1;
}

static void AudioBuffer_IncrementWaddr(void)
{
	if (!aubuf_wavail) return;
	aubuf_waddr = (aubuf_waddr+1) % AUDIO_BUFFER_LEN;
	aubuf_count++;
	if( aubuf_waddr == aubuf_raddr)
	{
		aubuf_ravail = 1;
		aubuf_wavail = 0;
	}
	else
	{
		aubuf_ravail = 1;
		aubuf_wavail = 1;
	}
}

static void AudioBuffer_IncrementRaddr(void)
{
	if (!aubuf_ravail) return;
	aubuf_raddr = (aubuf_raddr+1) % AUDIO_BUFFER_LEN;
	aubuf_count--;
	if( aubuf_waddr == aubuf_raddr)
	{
		aubuf_ravail = 0;
		aubuf_wavail = 1;
	}
	else
	{
		aubuf_ravail = 1;
		aubuf_wavail = 1;
	}
}