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
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"    // CMSIS RTOS header file
#define osObjectsPublic  // define objects in main module
#include "bsp_user_lib.h"
#include "modbus_host.h"
#include "osObjects.h"  // RTOS object definitions
#include "stm32f1xx_hal.h"
#include "user_data.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osEvent      tid_ThreadEvent;
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

uint8_t tx1DMAbuffer[BUFFER_SIZE] = {0x0A, 0x03, 0x00, 0x16, 0x00, 0x05, 0x65, 0x76};
uint8_t rx1DMAbuffer[BUFFER_SIZE] = {0};
uint8_t rx1Count                  = 0;
uint8_t rx1Buffer[BUFFER_SIZE]    = {0};

CAN_TxHeaderTypeDef TxMeg;
CAN_RxHeaderTypeDef RxMeg;

/* Public variables ---------------------------------------------------------*/
uint16_t volFan   = 0;
uint16_t vol48V   = 0;
uint16_t curCheck = 0;
/* Private function prototypes -----------------------------------------------*/
static void uartDMAStart(DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
static void adcValueProcess(uint16_t *src, uint16_t *dst, uint16_t times);
uint8_t     CANx_SendNormalData(CAN_HandleTypeDef *hcan, uint32_t ide, uint32_t rtr, uint16_t ID, uint8_t *pData, uint16_t Len);
static void uart4_scan(void);
static void uart4DataCheck(void);
static void usart1Scan(void);
static void uart1DataCheck(void);
/* Private user code ---------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/

void       Thread(void const *argument);      // thread function
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
    if (!mid_MsgQueue) {
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
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx4DMAbuffer, BUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2DMAbuffer, BUFFER_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx2DMAbuffer, BUFFER_SIZE);
    USART_DIR2_RX;  // 485_DIR2
    UART_DIR4_RX;   // 485_DIR4
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value, 6 * 20);
    while (1) {
        tid_ThreadEvent = osMessageGet(mid_MsgQueue, 100);  // wait for message
        if (tid_ThreadEvent.status == osEventTimeout) {
            adcValueProcess(ADC_Value, ADCVAlueAverage, 20);
            // CANx_SendNormalData(&hcan, CAN_ID_EXT, CAN_RTR_DATA, 0x8006, CanData, 8);
            uart4_scan();
            usart1Scan();
            // Insert thread code here...
            // osThreadYield();  // suspend thread
        } else if (tid_ThreadEvent.status == osEventMessage) {
            if (tid_ThreadEvent.value.v == 1) {
                uart1DataCheck();
                memset(rx1Buffer, rx1Count, 0);
                rx1Count = 0;
            }
            if (tid_ThreadEvent.value.v == 2) {
                MODH_RxData(&g_tModH, rx2Buffer, rx2Count);
                rx2Count = 0;
            }
            if (tid_ThreadEvent.value.v == 4) {
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
    for (i = 0; i < times; i++) {
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    UNUSED(Size);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UARTEx_RxEventCallback can be implemented in the user file.
     */
    if (huart == &huart1) {
        if (Size > 0) {
            if (rx1Count + Size > BUFFER_SIZE)
                rx1Count = 0;
            memcpy(rx1Buffer + rx1Count, rx1DMAbuffer, Size);
            rx1Count += Size;
            osMessagePut(mid_MsgQueue, 1, 0);  // Send Message
            memset(rx1DMAbuffer, 0, Size);     // 清零接收缓冲区
        }
        uartDMAStart(&hdma_usart1_rx, &huart1, rx1DMAbuffer, BUFFER_SIZE);  // 重新打开DMA接收
    }
    if (huart == &huart2) {
        if (Size > 0) {
            if (rx2Count + Size > BUFFER_SIZE)
                rx2Count = 0;
            memcpy(rx2Buffer + rx2Count, rx2DMAbuffer, Size);
            rx2Count += Size;
            osMessagePut(mid_MsgQueue, 2, 0);  // Send Message
            memset(rx2DMAbuffer, 0, Size);     // 清零接收缓冲区
        }
        uartDMAStart(&hdma_usart2_rx, &huart2, rx2DMAbuffer, BUFFER_SIZE);  // 重新打开DMA接收
    }
    if (huart == &huart4) {
        if (Size > 0) {
            if (rx4Count + Size > BUFFER_SIZE)
                rx4Count = 0;
            memcpy(rx4Buffer + rx4Count, rx4DMAbuffer, Size);
            rx4Count += Size;
            osMessagePut(mid_MsgQueue, 4, 0);  // Send Message
            memset(rx4DMAbuffer, 0, Size);     // 清零接收缓冲区
        }
        uartDMAStart(&hdma_uart4_rx, &huart4, rx4DMAbuffer, BUFFER_SIZE);  // 重新打开DMA接收
    }
}
static void uartDMAStart(DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
    __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    if (huart == &huart1) {
        USART_DIR1_RX;  // 485_DIR1
    }
    if (huart == &huart2) {
        USART_DIR2_RX;  // 485_DIR2
    }
    if (huart == &huart4) {
        UART_DIR4_RX;  // 485_DIR4
    }
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_TxCpltCallback could be implemented in the user file
     */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_ErrorCallback could be implemented in the user file
     */
    if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE) {
        __HAL_UART_FLUSH_DRREGISTER(huart);  // 读DR寄存器，就可以清除ORE错误标志位
    }
    if (huart == &huart1) {
        uartDMAStart(&hdma_usart1_rx, &huart1, rx1DMAbuffer, BUFFER_SIZE);
        USART_DIR1_RX;
    } else if (huart == &huart2) {
        uartDMAStart(&hdma_usart2_rx, &huart2, rx2DMAbuffer, BUFFER_SIZE);
        USART_DIR2_RX;
    } else if (huart == &huart4) {
        uartDMAStart(&hdma_uart4_rx, &huart4, rx4DMAbuffer, BUFFER_SIZE);
        UART_DIR4_RX;
    }
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#ifdef __cplusplus
extern "C" {
#endif  //__cplusplus

PUTCHAR_PROTOTYPE
{
    // RS485A_DE = DOsnt;
    // 发送脚使能，RS485为发送数据状态
    // delay_us(10);
    // 等待发送脚的电平稳定
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xFFFF);
    // 调用STM32的HAL库，发送一个字节
    //  delay_us(10);
    // 避免数据信号震荡造成回环数据
    //  RS485A_DE = DOrec;
    // 发送脚除能，RS485恢复到接收数据状态
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
        if (HAL_OK == HAL_RetVal) {
            CAN_OK_Flag = 1;
            printf("%08x ", RxMeg.Timestamp);
            if (RxMeg.IDE == CAN_ID_STD) {
                if (RxMeg.StdId == 0x601) {
                    Data[0] = Data[0];
                }
                printf("StdId %08x", RxMeg.StdId);
            } else if (RxMeg.IDE == CAN_ID_EXT) {
                if (RxMeg.ExtId == 0x601) {
                    Data[0] = Data[0];
                }
                printf("ExtId %08x", RxMeg.ExtId);
            }
            printf(" %02x %02x %02x %02x %02x %02x %02x %02x\r\n", Data[0], Data[1], Data[2], Data[3], Data[4], Data[5], Data[6], Data[7]);
        }
    }
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
              user file
     */
}

uint8_t CANx_SendNormalData(CAN_HandleTypeDef *hcan, uint32_t ide, uint32_t rtr, uint16_t ID, uint8_t *pData, uint16_t Len)
{
    HAL_StatusTypeDef HAL_RetVal;
    uint16_t          SendTimes, SendCNT = 0;
    uint8_t           FreeTxNum = 0;
    uint32_t          CAN_TX_MB0;
    if (ide == CAN_ID_STD) {
        TxMeg.IDE   = CAN_ID_STD;
        TxMeg.StdId = ID;
        TxMeg.ExtId = 0;
    } else {
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
    while (SendTimes--) {
        if (0 == SendTimes) {
            if (Len % 8)
                TxMeg.DLC = Len % 8;
        }
        while (0 == FreeTxNum) {
            FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
        }
        HAL_Delay(1);  // 没有延时很有可能会发送失败
        HAL_RetVal = HAL_CAN_AddTxMessage(hcan, &TxMeg, pData + SendCNT, (uint32_t *)&CAN_TX_MB0);
        if (HAL_RetVal != HAL_OK) {
            return 2;
        }
        // printf("CAN_TX_MB0 %d\r\n", CAN_TX_MB0);
        SendCNT += 8;
    }
    return 0;
}

static void uart4_scan(void)
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

static void uart4DataCheck(void)
{
    uint8_t  cache[10] = {0x01, 0x03, 0x02, 0x00, 0x00};
    uint8_t *p         = searchData(rx4Buffer, rx4Count, 0x01);
    if (p && ((p - rx4Buffer) < (rx4Count - 5))) {
        if (memcmp(cache, p, 5) == 0) {
            MONITOR_OK_FLAG = 1;
        }
    } else {
    }
}

static void usart1Scan(void)
{
    static uint8_t count = 0;
    if (++count > 10)  // 100ms*10
    {
        count = 0;
        USART_DIR1_TX;
        HAL_UART_Transmit_DMA(&huart1, tx1DMAbuffer, 8);
    }
}

static void uart1DataCheck(void)
{
    if (CRC16_Modbus(rx1Buffer, rx1Count) == 0) {
        volFan   = rx1Buffer[3] << 8 | rx1Buffer[4];
        vol48V   = rx1Buffer[5] << 8 | rx1Buffer[6];
        curCheck = rx1Buffer[11] << 8 | rx1Buffer[12];
        // normal_info("volFan:%d,vol48V:%d,curCheck:%d,\r\n", volFan, vol48V, curCheck);
    } else {
        warning_info("crc error\r\n");
    }
}
