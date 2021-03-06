/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

// CAN message IDs. Check CAN protocol description document for detailed information
#define CAN_CONTROL_MSG_ID				0x500	// same for all nodes
#define CAN_CALIBRATION_MESSAGE_ID		0x501	// 0x501, 0x502 or 0x503 depending on which node

#define CAN_STATUS_MESSAGE_ID			0x510	// 0x510, 0x520 or 0x530
#define CAN_SENSOR_GROUP_1_MSG_ID		0x511	// 0x511, 0x521 or 0x531
#define CAN_SENSOR_GROUP_2_MSG_ID		0x512	// 0x512, 0x522 or 0x532
#define CAN_SENSOR_GROUP_3_MSG_ID		0x513	// 0x513, 0x523 or 0x533
#define CAN_SENSOR_GROUP_4_MSG_ID		0x514	// 0x514, 0x524 or 0x534
#define CAN_SENSOR_GROUP_5_MSG_ID		0x515	// 0x515, 0x525 or 0x535


/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

void CAN_SendSimple(uint32_t ID, uint32_t DLC, uint8_t *data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
