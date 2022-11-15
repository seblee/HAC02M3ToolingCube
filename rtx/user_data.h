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

#define VOL_12V_ERROR      0X0001
#define VOL_05V_ERROR      0X0002
#define VOL_3_3V_ERROR     0X0004
#define VOL_EEV12V_ERROR   0X0008
#define DIAT_0_ERROR       0X0010
#define DIAT_1_ERROR       0X0020
#define AO1AI1_ERROR       0X0040
#define AO2AI2_ERROR       0X0080
#define NTC_ERROR          0X0100
#define UART_HANDOPT_ERROR 0X0200
#define UART_MONITOR_ERROR 0X0400
#define UART_P25_ERROR     0X0800
#define CAN_ERROR          0X1000
#define COM_P3_ERROR       0X2000
#define EEPROM_ERROR       0X4000
#define POWER_ERROR        0X8000

#define EEV_ERROR   0X0001
#define AC48V_ERROR 0X0002
#define CUR_ERROR   0X0004
/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE 128

#define USART_DIR1_TX HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define USART_DIR1_RX HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define USART_DIR2_TX HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define USART_DIR2_RX HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define UART_DIR4_TX  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define UART_DIR4_RX  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
/* Private macro -------------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
#define __DEBUG
#ifdef __DEBUG
extern __IO uint32_t uwTick;
#define normal_info(format, ...)  printf("[%d][%d]:\033[32m" format "\033[32;0m", uwTick, __LINE__, ##__VA_ARGS__)
#define warning_info(format, ...) printf("[%d][%d]:\033[33m" format "\033[32;0m", uwTick, __LINE__, ##__VA_ARGS__)
#define error_info(format, ...)   printf("[%d][%d]:\033[31m" format "\033[32;0m", uwTick, __LINE__, ##__VA_ARGS__)
#else
#define normal_info(format, ...)
#define warn_info(format, ...)
#define error_info(format, ...)
#endif

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
extern CAN_HandleTypeDef  hcan;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_uart4_rx;
extern DMA_HandleTypeDef  hdma_uart4_tx;
extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart1_tx;
extern DMA_HandleTypeDef  hdma_usart2_rx;
extern DMA_HandleTypeDef  hdma_usart2_tx;

extern uint16_t ADCVAlueAverage[6];
extern uint8_t  tx2DMAbuffer[BUFFER_SIZE];
extern uint8_t  tx4DMAbuffer[BUFFER_SIZE];

extern volatile _TKS_FLAGA_type CONNECTEFlags;
#define CAN_OK_Flag     CONNECTEFlags.bits.b0
#define UARTOKP25FLAG   CONNECTEFlags.bits.b1
#define HANDOPT_OK_FLAG CONNECTEFlags.bits.b2
#define MONITOR_OK_FLAG CONNECTEFlags.bits.b3
#define COMOKP3FLAG     CONNECTEFlags.bits.b4

extern uint16_t volFan;
extern uint16_t vol48V;
extern uint16_t curCheck;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

#endif
