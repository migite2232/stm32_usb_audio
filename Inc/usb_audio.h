// usb_audio.h
// Copyright (c) 2017 Sugioka Y.
// This file is released under the MIT License.
// http://opensource.org/licenses/mit-license.php

#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

//All configurations are declared in usb_audio_dscr.h

typedef void (*USBAudio_CallbackFunc)(void);

void USBAudio_Init(PCD_HandleTypeDef *hpcd);
void USBAudio_DeInit(void);
void USBAudio_RegisterCallback(USBAudio_CallbackFunc startPlayback,
								USBAudio_CallbackFunc stopPlayback,
								USBAudio_CallbackFunc freqChanged);
void USBAudio_Start(void);
void USBAudio_Stop(void);
//return: Sampling frequency e.g) 44100, 48000
uint32_t USBAudio_AudioFreq(void);
//return: Ready to play or not
uint8_t USBAudio_Enough(void);
//Get audio packet and increment buffer read addr
//return: Pointer to audio packet buffer or NULL(buffer is not ready)
uint8_t* USBAudio_GetPacketBuffer(uint16_t *length);

//should be called from HAL_PCD_xxCallback function. don't call other anywhere
void USBAudio_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void USBAudio_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void USBAudio_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void USBAudio_SOFCallback(PCD_HandleTypeDef *hpcd);
void USBAudio_ResetCallback(PCD_HandleTypeDef *hpcd);
void USBAudio_SuspendCallback(PCD_HandleTypeDef *hpcd);
void USBAudio_ResumeCallback(PCD_HandleTypeDef *hpcd);
void USBAudio_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void USBAudio_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void USBAudio_ConnectCallback(PCD_HandleTypeDef *hpcd);
void USBAudio_DisconnectCallback(PCD_HandleTypeDef *hpcd);

#endif