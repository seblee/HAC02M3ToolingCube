/**
 * @file user_data.h
 * @author  xiaowine (xiaowine@sina.cn)
 * @brief
 * @version 01.00
 * @date    2021-05-23
 *
 * @copyright Copyright (c) {2020}  xiaowine
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-05-23 <td>1.0     <td>wangh     <td>内容
 * </table>
 * ******************************************************************
 * *                   .::::
 * *                 .::::::::
 * *                ::::::::::
 * *             ..:::::::::::
 * *          '::::::::::::
 * *            .:::::::::
 * *       '::::::::::::::..        女神助攻,流量冲天
 * *            ..::::::::::::.     永不宕机,代码无bug
 * *          ``:::::::::::::::
 * *           ::::``:::::::::'        .:::
 * *          ::::'   ':::::'       .::::::::
 * *        .::::'      ::::     .:::::::'::::
 * *       .:::'       :::::  .:::::::::' ':::::
 * *      .::'        :::::.:::::::::'      ':::::
 * *     .::'         ::::::::::::::'         ``::::
 * * ...:::           ::::::::::::'              ``::
 * *```` ':.          ':::::::::'                  ::::.
 * *                   '.:::::'                    ':'````.
 * ******************************************************************
 */
#ifndef __USER_DATA_H
#define __USER_DATA_H

/* Private includes ----------------------------------------------------------*/
#include "cmsis_os.h"  // CMSIS RTOS header file
#include "stm32f1xx_hal.h"
#include "userType.h"
/* Private typedef -----------------------------------------------------------*/

#define VOL_12V_ERROR    0X0001
#define VOL_05V_ERROR    0X0002
#define VOL_3_3V_ERROR   0X0004
#define VOL_EEV12V_ERROR 0X0008
#define DIAT_0_ERROR     0X0010
#define DIAT_1_ERROR     0X0020
#define AO1AI1_ERROR     0X0040
#define AO2AI2_ERROR     0X0080
#define NTC_ERROR        0X0100
#define UART_P2_ERROR    0X0200
#define UART_P24_ERROR   0X0400
#define UART_P29_ERROR   0X0800
#define CAN_ERROR        0X1000
#define COM_P3_ERROR     0X2000
#define EEPROM_ERROR     0X4000

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE 128

#define USART_DIR2_TX HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define USART_DIR2_RX HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define UART_DIR4_TX  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define UART_DIR4_RX  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_uart4_rx;
extern DMA_HandleTypeDef  hdma_uart4_tx;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern DMA_HandleTypeDef  hdma_usart2_tx;

extern uint16_t ADCVAlueAverage[6];
extern uint8_t  tx2DMAbuffer[BUFFER_SIZE];
extern uint8_t  tx4DMAbuffer[BUFFER_SIZE];

extern volatile _TKS_FLAGA_type CONNECTEFlags;
#define CANOKP28FLAG  CONNECTEFlags.bits.b0
#define UARTOKP29FLAG CONNECTEFlags.bits.b1
#define UARTOKP2FLAG  CONNECTEFlags.bits.b2
#define UARTOKP24FLAG CONNECTEFlags.bits.b3
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

#endif