/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 200;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
 {        
        CAN_FilterTypeDef sFilterConfig;                             // CAN filter configuration structure definition
        sFilterConfig.FilterBank           = 0;                      // CAN过滤器编号，范围0-27
        sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;  // CAN过滤器模式，掩码模式或列表模
        sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;  // CAN过滤器尺度，16位或32
        sFilterConfig.FilterIdHigh         = 0x000 << 5;             // 32位下，存储要过滤ID的高16
        sFilterConfig.FilterIdLow          = 0x0000;                 // 32位下，存储要过滤ID的低16
        sFilterConfig.FilterMaskIdHigh     = 0x0000;                 //掩码模式下，存储的是掩码
        sFilterConfig.FilterMaskIdLow      = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  //报文通过过滤器的匹配后，存储到哪个FIFO
        sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;  //�?活活过滤�?
        sFilterConfig.SlaveStartFilterBank = 0;

        HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
        // /**eg.**/
        // u16 Std_ID = 0x6D1;
        // u32 Ext_ID = 0x1EFEDFEA;
        // u32 mask   = 0;

        // CAN_FilterInitTypeDef CAN_FilterInitStructure;                     //定义个结构体变量
        // CAN_FilterInitStructure.CAN_FilterNumber = 0;                      //设置过滤器组0
        // CAN_FilterInitStructure.CAN_FilterMode   = CAN_FilterMode_IdMask;  //设置过滤器组0为屏蔽模??
        // CAN_FilterInitStructure.CAN_FilterScale  = CAN_FilterScale_32bit;  //设置过滤器组0位宽??32??
        // /**************************************************************************************************************************************
        // 标识符寄存器的设置，Ext_ID<<3对齐，再>>16取高16??
        // ***************************************************************************************************************************************/
        // CAN_FilterInitStructure.CAN_FilterIdHigh=??(Ext_ID<<3) >>16) & 0xffff;  //设置标识符寄存器高字�????
        // CAN_FilterInitStructure.CAN_FilterIdLow = (u16)(Ext_ID << 3) | CAN_ID_EXT;  //设置标识符寄存器低字??

        // /***********************************************************************************************************************************
        // 这里也可以这样设置，设置标识符寄存器高字??.这里为什么是左移5位呢？从上图可以看出，CAN_FilterIdHigh包含的是STD[0~10]和EXID[13~17],标准CAN
        // ID本身是不包含扩展ID数据，因此为了要将标准CAN ID放入此寄存器，标准CAN
        // ID首先应左??5位后才能对齐。设置标识符寄存器低字节,这里也可以设置为CAN_ID_STD
        // CAN_FilterInitStructure.CAN_FilterIdHigh=Std_ID<<5;
        // CAN_FilterInitStructure.CAN_FilterIdLow=0 | CAN_ID_EXT;*/
        // /*************************************************************************************************************************
        // 屏蔽寄存器的设置这里�????路是先将标准CAN ID和扩展CAN ID对应的ID值先异或后取反，为什么？异或是为了找出两个CAN
        // ID有哪些位是相同的，是相同的位则说明需
        // 要关心，??要关心的位对应的屏蔽码位应该设置??1，因此需要取反一�????�?后再整体左移3�????
        // ****************************************************************************************************************************/

        // mask = (Std_ID << 18); /*这里为什么左??18位？因为在标准CAN
        //          ID占ID18~ID28，为了与CAN_FilterIdHigh对齐，应左移2位，接着为了与扩??
        //          CAN对应，还应该再左??16位，因此�????共应�???2 + 16??18�????
        //          也可以用另一个方式来理解：直接看Mapping的内容，发现STDID相对EXID[0] 偏移??18??,因此左移18??. */
        // mask ^= Ext_ID;        //将对齐后的标准CAN与扩展CAN异或后取??
        // mask = ~mask;
        // mask <<= 3;    //再整体左??3??
        // mask |= 0x02;  //只接收数据帧，不接收远程??
        // CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = (mask >> 16) & 0xffff;  //设置屏蔽寄存器高字节
        // CAN_FilterInitStructure.CAN_FilterMaskIdLow      = mask & 0xffff;          //设置屏蔽寄存器低字节
        // CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;              //此过滤器组关联到接收FIFO0
        // CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;                 //??活此过滤器组
        // CAN_FilterInit(&CAN_FilterInitStructure);  //设置过滤??
    }
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
