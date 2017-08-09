// usb_pcd.c
// Copyright (c) 2017 Sugioka Y.
// This file is released under the MIT License.
// http://opensource.org/licenses/mit-license.php

#include "stm32f1xx_hal.h"
#include "usb_pcd.h"
#include "usb_audio.h"

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
	USBAudio_SetupStageCallback(hpcd);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	USBAudio_DataOutStageCallback(hpcd, epnum);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	USBAudio_DataInStageCallback(hpcd, epnum);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
	USBAudio_SOFCallback(hpcd);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{ 
	USBAudio_ResetCallback(hpcd);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
	USBAudio_SuspendCallback(hpcd);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
	USBAudio_ResumeCallback(hpcd);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	USBAudio_ISOOUTIncompleteCallback(hpcd, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
	USBAudio_ISOINIncompleteCallback(hpcd, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
	USBAudio_ConnectCallback(hpcd);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
	USBAudio_DisconnectCallback(hpcd);
}