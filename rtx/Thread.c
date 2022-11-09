/**
 * @file Thread.c
 * @brief
 * @author  xiaowine (xiaowine@sina.cn)
 * @version 01.00
 * @date    2021-05-14
 *
 * @copyright Copyright (c) {2020}  xiaowine
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-05-14 <td>1.0     <td>wangh     <td>内容
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

/* Private includes ----------------------------------------------------------*/
#include "cmsis_os.h"  // CMSIS RTOS header file
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#define osObjectsPublic  // define objects in main module
#include "osObjects.h"   // RTOS object definitions
#include "modbus_host.h"
#include "user_data.h"
#include "bsp_user_lib.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osEvent tid_ThreadEvent;
osMessageQId mid_MsgQueue;                                // message queue id
#define MSGQUEUE_OBJECTS 16                               // number of Message Queue Objects
osMessageQDef(MsgQueue, MSGQUEUE_OBJECTS, unsigned int);  // message queue object

extern ADC_HandleTypeDef hadc1;

uint16_t ADC_Value[6 * 20]  = {0};
uint16_t ADCVAlueAverage[6] = {0};

uint8_t tx2DMAbuffer[BUFFER_SIZE] = {"uart2 DMA test\r\n"};
uint8_t rx2DMAbuffer[BUFFER_SIZE] = {0};
uint8_t rx2Count                  = 0;
uint8_t rx2Buffer[BUFFER_SIZE]    = {0};

uint8_t tx4DMAbuffer[BUFFER_SIZE] = {"uart4 DMA test\r\n"};
uint8_t rx4DMAbuffer[BUFFER_SIZE] = {0};
uint8_t rx4Count                  = 0;
uint8_t rx4Buffer[BUFFER_SIZE]    = {0};

extern CAN_HandleTypeDef hcan;
CAN_TxHeaderTypeDef TxMeg;
CAN_RxHeaderTypeDef RxMeg;

/* Public variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static void adcValueProcess(uint16_t *src, uint16_t *dst, uint16_t times);
uint8_t CANx_SendNormalData(CAN_HandleTypeDef *hcan, uint32_t ide, uint32_t rtr, uint16_t ID, uint8_t *pData,
                            uint16_t Len);
void uart4_scan(void);
void uart4DataCheck(void);
/* Private user code ---------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/

void Thread(void const *argument);            // thread function
osThreadId tid_Thread;                        // thread id
osThreadDef(Thread, osPriorityNormal, 1, 0);  // thread object

int Init_Thread(void)
{
    tid_Thread = osThreadCreate(osThread(Thread), NULL);
    if (!tid_Thread)
        return (-1);

    printf("###########Thread ready######\r\n");
    return (0);
}
int Init_MsgQueue(void)
{
    mid_MsgQueue = osMessageCreate(osMessageQ(MsgQueue), NULL);  // create msg queue
    if (!mid_MsgQueue)
    {
        printf("\"Init_MsgQueue error!!!\r\n");
        // Message Queue object not created, handle failure
        return (-1);
    }
    return (0);
}

void Thread(void const *argument)
{
    Init_MsgQueue();
    // Init_Timers();
    HAL_UART_Receive_DMA(&huart4, rx4DMAbuffer, BUFFER_SIZE);
    HAL_UART_Receive_DMA(&huart2, rx2DMAbuffer, BUFFER_SIZE);
    USART_DIR2_RX;  // 485_DIR2
    UART_DIR4_RX;   // 485_DIR4
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value, 6 * 20);
    while (1)
    {
        tid_ThreadEvent = osMessageGet(mid_MsgQueue, 100);  // wait for message
        if (tid_ThreadEvent.status == osEventTimeout)
        {
            adcValueProcess(ADC_Value, ADCVAlueAverage, 20);
            // CANx_SendNormalData(&hcan, CAN_ID_EXT, CAN_RTR_DATA, 0x8006, CanData, 8);
            uart4_scan();
            // Insert thread code here...
            // osThreadYield();  // suspend thread
        }
        else if (tid_ThreadEvent.status == osEventMessage)
        {
            if (tid_ThreadEvent.value.v == 2)
            {
                MODH_RxData(&g_tModH, rx2Buffer, rx2Count);
                rx2Count = 0;
            }
            if (tid_ThreadEvent.value.v == 4)
            {
                uart4DataCheck();
                memset(rx4Buffer, rx4Count, 0);
                rx4Count = 0;
            }
            // printf("rx%dCount %02d message %04x\r\n", tid_ThreadEvent.value.v, rxCount, tid_ThreadEvent.value.v);
        }
    }
}

static void adcValueProcess(uint16_t *src, uint16_t *dst, uint16_t times)
{
    uint32_t cache[6] = {0};
    uint16_t i;
    for (i = 0; i < times; i++)
    {
        cache[0] += *(src + 0 + 6 * i);
        cache[1] += *(src + 1 + 6 * i);
        cache[2] += *(src + 2 + 6 * i);
        cache[3] += *(src + 3 + 6 * i);
        cache[4] += *(src + 4 + 6 * i);
        cache[5] += *(src + 5 + 6 * i);
    }
    *(dst + 0) = cache[0] / times;
    *(dst + 1) = cache[1] / times;
    *(dst + 2) = cache[2] / times;
    *(dst + 3) = cache[3] / times;
    *(dst + 4) = cache[4] / times;
    *(dst + 5) = cache[5] / times;
}

//串口接收空闲中断
void UsartReceive_IDLE(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        if (RESET != __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE))  //判断是否是空闲中断
        {
            uint8_t rx_len;
            __HAL_UART_CLEAR_IDLEFLAG(&huart4);  //清楚空闲中断标志（否则会一直不断进入中断）
            HAL_UART_DMAStop(&huart4);           //停止本次DMA传输
            rx_len = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);  //计算接收到的数据长度
            if (rx_len > 0)
            {
                if (rx4Count + rx_len > BUFFER_SIZE)
                    rx4Count = 0;
                memcpy(rx4Buffer + rx4Count, rx4DMAbuffer, rx_len);
                rx4Count += rx_len;
                osMessagePut(mid_MsgQueue, 4, 0);  // Send Message
            }
            memset(rx4DMAbuffer, 0, rx_len);                           //清零接收缓冲区
            HAL_UART_Receive_DMA(&huart4, rx4DMAbuffer, BUFFER_SIZE);  //重新打开DMA接收
        }
    }
    if (huart == &huart2)
    {
        if (RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))  //判断是否是空闲中断
        {
            uint8_t rx_len;
            __HAL_UART_CLEAR_IDLEFLAG(&huart2);  //清楚空闲中断标志（否则会一直不断进入中断）
            HAL_UART_DMAStop(&huart2);           //停止本次DMA传输
            rx_len = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);  //计算接收到的数据长度
            if (rx_len > 0)
            {
                if (rx2Count + rx_len > BUFFER_SIZE)
                    rx2Count = 0;
                memcpy(rx2Buffer + rx2Count, rx2DMAbuffer, rx_len);
                rx2Count += rx_len;
                osMessagePut(mid_MsgQueue, 2, 0);  // Send Message
            }
            memset(rx2DMAbuffer, 0, rx_len);                           //清零接收缓冲区
            HAL_UART_Receive_DMA(&huart2, rx2DMAbuffer, BUFFER_SIZE);  //重新打开DMA接收
        }
    }
}

void HAL_UART_DMAStopRX(UART_HandleTypeDef *huart)
{
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    HAL_DMA_Abort(huart->hdmarx);
    CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    if (huart == &huart2)
    {
        USART_DIR2_RX;  // 485_DIR2
    }
    if (huart == &huart4)
    {
        UART_DIR4_RX;  // 485_DIR4
    }
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_TxCpltCallback could be implemented in the user file
     */
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#ifdef __cplusplus
extern "C"
{
#endif  //__cplusplus

    PUTCHAR_PROTOTYPE
    {
        // RS485A_DE = DOsnt;
        //发送脚使能，RS485为发送数据状态
        // delay_us(10);
        //等待发送脚的电平稳定
        HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xFFFF);
        //调用STM32的HAL库，发送一个字节
        // delay_us(10);
        //避免数据信号震荡造成回环数据
        // RS485A_DE = DOrec;
        //发送脚除能，RS485恢复到接收数据状态
        return (ch);
    }
#ifdef __cplusplus
}
#endif  //__cplusplus

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t Data[8];
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);
    HAL_StatusTypeDef HAL_RetVal;
    // if (hcan == &hcan1)
    {
        HAL_RetVal = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMeg, Data);
        if (HAL_OK == HAL_RetVal)
        {
            CANOKP28FLAG = 1;
            printf("%08x ", RxMeg.Timestamp);
            if (RxMeg.IDE == CAN_ID_STD)
            {
                if (RxMeg.StdId == 0x601)
                {
                    Data[0] = Data[0];
                }
                printf("StdId %08x", RxMeg.StdId);
            }
            else if (RxMeg.IDE == CAN_ID_EXT)
            {
                if (RxMeg.ExtId == 0x601)
                {
                    Data[0] = Data[0];
                }
                printf("ExtId %08x", RxMeg.ExtId);
            }
            printf(" %02x %02x %02x %02x %02x %02x %02x %02x\r\n", Data[0], Data[1], Data[2], Data[3], Data[4], Data[5],
                   Data[6], Data[7]);
        }
    }
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
              user file
     */
}

uint8_t CANx_SendNormalData(CAN_HandleTypeDef *hcan, uint32_t ide, uint32_t rtr, uint16_t ID, uint8_t *pData,
                            uint16_t Len)
{
    HAL_StatusTypeDef HAL_RetVal;
    uint16_t SendTimes, SendCNT = 0;
    uint8_t FreeTxNum = 0;
    uint32_t CAN_TX_MB0;
    if (ide == CAN_ID_STD)
    {
        TxMeg.IDE   = CAN_ID_STD;
        TxMeg.StdId = ID;
        TxMeg.ExtId = 0;
    }
    else
    {
        TxMeg.IDE   = CAN_ID_EXT;
        TxMeg.StdId = 0;
        TxMeg.ExtId = ID;
    }

    if (!hcan || !pData || !Len)
        return 1;
    SendTimes                = Len / 8 + (Len % 8 ? 1 : 0);
    FreeTxNum                = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
    TxMeg.DLC                = 8;
    TxMeg.RTR                = rtr;
    TxMeg.TransmitGlobalTime = DISABLE;
    while (SendTimes--)
    {
        if (0 == SendTimes)
        {
            if (Len % 8)
                TxMeg.DLC = Len % 8;
        }
        while (0 == FreeTxNum)
        {
            FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
        }
        HAL_Delay(1);  //没有延时很有可能会发送失败
        HAL_RetVal = HAL_CAN_AddTxMessage(hcan, &TxMeg, pData + SendCNT, (uint32_t *)&CAN_TX_MB0);
        if (HAL_RetVal != HAL_OK)
        {
            return 2;
        }
        // printf("CAN_TX_MB0 %d\r\n", CAN_TX_MB0);
        SendCNT += 8;
    }
    return 0;
}

void uart4_scan(void)
{
    uint16_t crc;
    tx4DMAbuffer[0] = 0x01;
    tx4DMAbuffer[1] = 0x03;
    tx4DMAbuffer[2] = 0x00;
    tx4DMAbuffer[3] = 0x00;
    tx4DMAbuffer[4] = 0x00;
    tx4DMAbuffer[5] = 0x01;
    crc             = CRC16_Modbus(tx4DMAbuffer, 6);
    tx4DMAbuffer[6] = crc >> 8;
    tx4DMAbuffer[7] = crc;
    UART_DIR4_TX;
    HAL_UART_Transmit_DMA(&huart4, tx4DMAbuffer, 8);
}

void uart4DataCheck(void)
{
    uint8_t cache[10] = {0x01, 0x03, 0x02, 0x00, 0x00};
    uint8_t *p        = searchData(rx4Buffer, rx4Count, 0x01);
    if (p && ((p - rx4Buffer) < (rx4Count - 5)))
    {
        if (memcmp(cache, p, 5) == 0)
        {
            UARTOKP29FLAG = 1;
        }
    }
    else
    {
    }
}
