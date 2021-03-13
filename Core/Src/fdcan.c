/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 11;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 11;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */
    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_SendSimple(uint32_t ID, uint32_t DLC, uint8_t *data)
{
	// CAN message for transmission
	FDCAN_TxHeaderTypeDef txh;
	txh.Identifier = ID;
	txh.IdType = FDCAN_STANDARD_ID;
	txh.TxFrameType = FDCAN_DATA_FRAME;
	txh.DataLength = DLC << 16;	// shift is needed because then it matches FDCAN_data_length_code, which sets register value directly
	txh.FDFormat = FDCAN_CLASSIC_CAN;
	txh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txh.BitRateSwitch = FDCAN_BRS_OFF;
	txh.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txh.MessageMarker = 0;

	uint32_t timeout = 1000000;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {	// wait while CAN TX FIFO has no empty slots
		--timeout;
		if (timeout == 0) {break;}
	}
	if (timeout != 0) {
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txh, data);
	}
	//HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txh, data);

}

void CAN_SetupFilters(){
	FDCAN_FilterTypeDef filter;
	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = CAN_CONTROL_MSG_ID;
	filter.FilterID2 = 0x7ff;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

	filter.FilterIndex = 1;
	filter.FilterID1 = CAN_CALIBRATION_MESSAGE_ID;
	filter.FilterID2 = 0x7ff;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
}


// sets up RX FIFO0 interrupt
void CAN_SetupInterrupts(){
	// configures RX FIFO0 interrupt group to interrupt line 0
	HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_LIST_RX_FIFO0, FDCAN_INTERRUPT_LINE0);

	// activates RX FIFO0 interrupt group
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_TX_BUFFER0);
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
