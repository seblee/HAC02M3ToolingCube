/**
 * @file testThread.c
 * @brief
 * @author  xiaowine (xiaowine@sina.cn)
 * @version 01.00
 * @date    2021-05-17
 *
 * @copyright Copyright (c) {2020}  xiaowine
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-05-17 <td>1.0     <td>wangh     <td>内容
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
#include "stdlib.h"
#include "stm32f1xx_hal.h"
#include "userType.h"
#include "user_data.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define keyNUM        2
#define RESTAIN_TIMES 200  // 200 × 10ms = 2s

#define VOL_12V    1800     // 2.49 * 12 * 4095 / (21* 3.3)
#define VOL_05V    735      // 2.49 * 5 * 4095 / (21* 3.3)
#define VOL_3_3V   485      // 2.49 * 3.3 * 4095 / (21* 3.3)
#define VOL_EEV12V VOL_12V  // 2.49 * 12 * 4095 / (21* 3.3)
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/*******key***********/
unsigned                 keyTime[keyNUM] = {0};
volatile _TKS_FLAGA_type keyTrg[2];
volatile unsigned char   k_count[2];
#define NO1(x) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, (x));  // NO1
#define NO2(x) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, (x));  // NO2
#define NO3(x) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, (x));  // NO3
#define NO4(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (x));  // NO4
#define NO5(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (x));  // NO5
#define NO6(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (x));  // NO6
#define NO7(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (x));  // NO7
#define NO8(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (x));  // NO8
#define NO9(x) HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, (x));  // NO9

#define KEY0Trg keyTrg[0].bits.b0
#define KEY1Trg keyTrg[0].bits.b1
#define KEY2Trg keyTrg[0].bits.b2

#define KEY0RestainTrg keyTrg[1].bits.b0
#define KEY1RestainTrg keyTrg[1].bits.b1
#define KEY2RestainTrg keyTrg[1].bits.b2

#define STARTTrg KEY0Trg
#define STOPTrg  KEY1Trg

status_st    _status_ICT = {0};
local_reg_st l_sys       = {0};

volatile _TKS_FLAGA_type CONNECTEFlags = {0};
#define CANOKP28FLAG  CONNECTEFlags.bits.b0
#define UARTOKP29FLAG CONNECTEFlags.bits.b1
#define UARTOKP2FLAG  CONNECTEFlags.bits.b2
#define UARTOKP24FLAG CONNECTEFlags.bits.b3
#define COMOKP3FLAG   CONNECTEFlags.bits.b4
/* Public variables ---------------------------------------------------------*/

#define __DEBUG

#ifdef __DEBUG
#define normal_info(format, ...)  printf("[Line: %d][%s]: \033[32m" format "\033[32;0m", __LINE__, __func__, ##__VA_ARGS__)
#define warning_info(format, ...) printf("[Line: %d][%s]: \033[33m" format "\033[32;0m", __LINE__, __func__, ##__VA_ARGS__)
#define error_info(format, ...)   printf("[Line: %d][%s]: \033[31m" format "\033[32;0m", __LINE__, __func__, ##__VA_ARGS__)
#else
#define normal_info(format, ...)
#define warn_info(format, ...)
#define error_info(format, ...)
#endif
/* Private function prototypes -----------------------------------------------*/
static void keyScan(void);
void        testFun(void);
/* Private user code ---------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/

void       testThread(void const *argument);      // thread function
osThreadId tid_testThread;                        // thread id
osThreadDef(testThread, osPriorityNormal, 1, 0);  // thread object

int Init_testThread(void)
{
    tid_testThread = osThreadCreate(osThread(testThread), NULL);
    if (!tid_testThread)
        return (-1);

    NO1(GPIO_PIN_RESET);
    NO2(GPIO_PIN_RESET);
    NO3(GPIO_PIN_RESET);
    NO4(GPIO_PIN_RESET);
    NO5(GPIO_PIN_RESET);
    NO6(GPIO_PIN_RESET);
    NO7(GPIO_PIN_RESET);
    NO8(GPIO_PIN_RESET);
    printf("###########testThread ready######\r\n");
    return (0);
}

void testThread(void const *argument)
{
    while (1) {
        osDelay(10);
        keyScan();
        testFun();
    }
}

/**
 * @brief
 */
static void keyScan(void)
{
    static uint8_t keyDataBak   = 0;
    static uint8_t keyDataValue = 0;
    unsigned char  keyRestain = 0, keyData = 0, i;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) {
        keyData = 1;
    }
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)) {
        keyData |= 2;
    }
    keyData = ~keyData;

    if (keyData != keyDataBak) {
        keyDataBak = keyData;
    } else {
        keyDataValue = keyData;
    }

    keyTrg[0].byte = keyDataValue & (keyDataValue ^ k_count[0]);
    k_count[0]     = keyDataValue;

    for (i = 0; i < keyNUM; i++) {
        if (k_count[0] & (1 << i)) {
            if (keyTime[i] < RESTAIN_TIMES)
                keyTime[i]++;
            else
                keyRestain |= (1 << i);
        } else {
            // if (keyTime[i] > 0)  // short press
            //     Key_Up_Trg |= (1 << i);
            // else
            //     Key_Up_Trg &= (~(1 << i));

            keyTime[i] = 0;
            keyRestain &= (~(1 << i));
        }
    }
    keyTrg[1].byte = keyRestain & (keyRestain ^ k_count[1]);
    k_count[1]     = keyRestain;
}

void testFun(void)
{
    static uint16_t ICT_test = 0;
    static uint16_t ICTStep  = 0;

    if (STARTTrg) {
        printf("ICT_START\r\n");
        l_sys.u8ICT_PowerKey = TRUE;
        if (_status_ICT.u16Status == ICT_STOP)
            _status_ICT.u16Status = ICT_IDLE;
    }
    if (STOPTrg) {
        _status_ICT.u16Status = ICT_STOP;
        _status_ICT.u16Fsm    = ICT_ST_NO;
    }

    switch (_status_ICT.u16Status) {
        case ICT_IDLE: {
            if (l_sys.u8ICT_PowerKey == TRUE) {  // 上电
                l_sys.u16ICT_Delay       = 500;
                l_sys.u8ICT_Start        = TRUE;
                _status_ICT.VALUE_12V    = 0;  // 00
                _status_ICT.VALUE_05V    = 0;  // 01
                _status_ICT.VALUE_3_3V   = 0;  // 02
                _status_ICT.VALUE_EEV12V = 0;  // 03
                _status_ICT.IO_0         = 0;  // 04
                _status_ICT.IO_1         = 0;  // 05
                _status_ICT.AO1AI1       = 0;  // 06
                _status_ICT.AO2AI2       = 0;  // 07
                _status_ICT.NTC_COM      = 0;  // 08
                _status_ICT.u16Test      = 0;  // 09

                _status_ICT.u16Fsm    = ICT_ST_START;  // 0A
                _status_ICT.u16Status = ICT_START;     // 0B

                NO1(GPIO_PIN_SET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_RESET);
                NO5(GPIO_PIN_RESET);
                NO6(GPIO_PIN_RESET);
                NO7(GPIO_PIN_RESET);
                NO8(GPIO_PIN_SET);

                ICT_test           = 0;
                ICTStep            = 0;
                CONNECTEFlags.byte = 0;
                if (MODH_WriteParam_06H(SlaveHMIAddr, 0xa02a, _status_ICT.u16Fsm) == 1) {
                    // printf("**HMIAddr WriteParam 06H OK line:%d\r\n", __LINE__);
                } else {
                    error_info("hmi write06 error\r\n");
                }
            } else {
                _status_ICT.u16Status = ICT_IDLE;
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_RESET);
                NO5(GPIO_PIN_RESET);
                NO6(GPIO_PIN_RESET);
                NO7(GPIO_PIN_RESET);
                NO8(GPIO_PIN_RESET);
            }
            break;
        }
        case ICT_START: {
            if (l_sys.u16ICT_Delay) {
                l_sys.u16ICT_Delay--;
            } else {
                /**
                 * clear data;
                 * @brief
                 */
                if (MODH_WriteParam_10H(SlaveHMIAddr, 0xA020, 10, (uint8_t *)(&_status_ICT.VALUE_12V)) == 1) {
                    // printf("**HMIAddr WriteParam 10H OK line:%d\r\n", __LINE__);
                } else {
                    error_info("hmi write10 error\r\n");
                }
                l_sys.u16ICT_Delay    = 300;  // wait AI ready
                _status_ICT.u16Status = ICT_TEST;
            }
            break;
        }
        case ICT_TEST: {
            static uint16_t ICTDelay[4] = {0};
            static uint8_t  timeout[4]  = {0};
            // LINE_1:
            if (ICTDelay[0]) {
                ICTDelay[0]--;
                goto LINE_2;
            }

            if ((ICTStep & 0x0001) == 0) {
                printf("ICTStep:0x0001-->12V\t");
                _status_ICT.VALUE_12V = ADCVAlueAverage[1] * 330 * 20 / 2.49 / 4095;
                _status_ICT.VALUE_12V = BEBufToUint16((uint8_t *)&_status_ICT.VALUE_12V);
                if (abs(ADCVAlueAverage[1] - VOL_12V) > 125) {  // VOL_12V
                    printf("abs:%d ", abs(ADCVAlueAverage[1] - VOL_12V));
                    ICT_test |= VOL_12V_ERROR;
                }
                printf("AI:%d\tv:%.04fv\r\n", ADCVAlueAverage[1], (float)(ADCVAlueAverage[1] * 3.3 * 20 / 2.49 / 4095));
                ICTStep |= 0x0001;
                ICTDelay[0] = 50;
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_RESET);
                goto LINE_2;
            }

            if ((ICTStep & 0x0002) == 0) {
                ICTStep |= 0x0002;
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_SET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_RESET);
                ICTDelay[0] = 90;
                goto LINE_2;
            }
            if ((ICTStep & 0x0004) == 0) {
                printf("ICTStep:0x0004-->05V\t");
                _status_ICT.VALUE_05V = ADCVAlueAverage[1] * 330 * 20 / 2.49 / 4095;
                _status_ICT.VALUE_05V = BEBufToUint16((uint8_t *)&_status_ICT.VALUE_05V);
                if (abs(ADCVAlueAverage[1] - VOL_05V) > 125) {  // VOL_05V
                    printf("abs:%d ", abs(ADCVAlueAverage[1] - VOL_05V));
                    ICT_test |= VOL_05V_ERROR;
                }
                printf("AI:%d\tv:%.04fv\r\n", ADCVAlueAverage[1], (float)(ADCVAlueAverage[1] * 3.3 * 20 / 2.49 / 4095));
                ICTStep |= 0x0004;
                ICTDelay[0] = 50;
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_RESET);
                goto LINE_2;
            }
            if ((ICTStep & 0x0008) == 0) {
                ICTStep |= 0x0008;
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_SET);
                NO4(GPIO_PIN_RESET);
                ICTDelay[0] = 90;
                goto LINE_2;
            }
            if ((ICTStep & 0x0010) == 0) {
                printf("ICTStep:0x0010-->3.3V\t");
                _status_ICT.VALUE_3_3V = ADCVAlueAverage[1] * 330 * 20 / 2.49 / 4095;
                _status_ICT.VALUE_3_3V = BEBufToUint16((uint8_t *)&_status_ICT.VALUE_3_3V);
                if (abs(ADCVAlueAverage[1] - VOL_3_3V) > 125) {  // VOL_3_3V
                    printf("abs:%d ", abs(ADCVAlueAverage[1] - VOL_3_3V));
                    ICT_test |= VOL_3_3V_ERROR;
                }
                printf("AI:%d\tv:%.04fv\r\n", ADCVAlueAverage[1], (float)(ADCVAlueAverage[1] * 3.3 * 20 / 2.49 / 4095));
                ICTStep |= 0x0010;
                ICTDelay[0] = 50;
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_RESET);
                goto LINE_2;
            }
            if ((ICTStep & 0x0020) == 0) {
                ICTStep |= 0x0020;
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_SET);
                ICTDelay[0] = 90;
                goto LINE_2;
            }
            if ((ICTStep & 0x0040) == 0) {
                printf("ICTStep:0x0040-->EEV12V\t");
                _status_ICT.VALUE_EEV12V = ADCVAlueAverage[1] * 330 * 20 / 2.49 / 4095;
                _status_ICT.VALUE_EEV12V = BEBufToUint16((uint8_t *)&_status_ICT.VALUE_EEV12V);
                if (abs(ADCVAlueAverage[1] - VOL_EEV12V) > 125) {  // VOL_EEV12V
                    printf("abs:%d ", abs(ADCVAlueAverage[1] - VOL_EEV12V));
                    ICT_test |= VOL_EEV12V_ERROR;
                }
                printf("AI:%d\tv:%.04fv\r\n", ADCVAlueAverage[1], (float)(ADCVAlueAverage[1] * 3.3 * 20 / 2.49 / 4095));
                ICTStep |= 0x0040;
                if (MODH_WriteParam_10H(SlaveHMIAddr, 0xA020, 4, (uint8_t *)(&_status_ICT.VALUE_12V)) == 1) {
                    // printf("**HMIAddr WriteParam 10H OK line:%d\r\n", __LINE__);
                } else {
                    error_info("hmi write10 error\r\n");
                }

                printf("ICT_test:%04x\r\n", ICT_test);
                goto LINE_2;
            }
        LINE_2:  // DI DO  AI AO
            if (ICTDelay[1]) {
                ICTDelay[1]--;
                if (timeout[1] > 10) {
                    ICTStep |= 0x0780;
                    ICT_test |= (DIAT_0_ERROR | DIAT_1_ERROR | AO1AI1_ERROR | AO2AI2_ERROR | NTC_ERROR | UART_P2_ERROR | UART_P24_ERROR | UART_P29_ERROR | CAN_ERROR | COM_P3_ERROR);
                }
                goto LINE3;
            }
            if ((ICTStep & 0x0080) == 0) {
                printf("ICTStep:0x0080-->");
                if (MODH_WriteParam_06H(SlaveBoardAddr, 264, 0xffff) == 1) {
                    printf("change DI polarity OK\r\n");
                    timeout[1]   = 0;
                    UARTOKP2FLAG = 1;
                } else {
                    timeout[1]++;
                    ICTDelay[1] = 60;
                    error_info("write06 error\r\n");
                    goto LINE3;
                }
                printf("ICTStep:0x0080-->");
                if (MODH_WriteParam_06H(SlaveBoardAddr, 366, 0) == 1) {
                    printf("change 878, 0 OK\r\n");
                    timeout[1]   = 0;
                    UARTOKP2FLAG = 1;
                } else {
                    timeout[1]++;
                    ICTDelay[1] = 60;
                    error_info("write06 error\r\n");
                    goto LINE3;
                }
                printf("ICTStep:0x0080-->");
                if (MODH_WriteParam_06H(SlaveBoardAddr, 267, 0xffff) == 1) {
                    printf("change DI mask OK\r\n");
                    timeout[1]  = 0;
                    ICTDelay[1] = 300;
                    ICTStep |= 0x0080;
                } else {
                    timeout[1]++;
                    ICTDelay[1] = 60;
                    error_info("write06 error\r\n");
                    goto LINE3;
                }
                goto LINE3;
            }
            if ((ICTStep & 0x0100) == 0) {  // read DI
                if (MODH_ReadParam_03H(SlaveBoardAddr, 836, 2) == 1) {
                    printf("ICTStep:0x0100-->P29 pa_volt:%d\r\n", g_tVar[SlaveBoardAddr - 1].P01[0]);
                    if ((g_tVar[SlaveBoardAddr - 1].P01[0] >= 230) && (g_tVar[SlaveBoardAddr - 1].P01[0] <= 270)) {
                        UARTOKP24FLAG = 1;
                    }
                } else {
                    error_info("ICTStep:0x0100-->read03H error\r\n");
                    timeout[1]++;
                    ICTDelay[1] = 60;
                }

                if (MODH_ReadParam_03H(SlaveBoardAddr, 885, 1) == 1) {
                    _status_ICT.IO_0 = BEBufToUint16((uint8_t *)&g_tVar[SlaveBoardAddr - 1].P01[0]);
                    printf("ICTStep:0x0100-->read DI when NO = 0 OK,DI:%04x\r\n", g_tVar[SlaveBoardAddr - 1].P01[0]);
                    if (g_tVar[SlaveBoardAddr - 1].P01[0] != 0x0000) {  //**************
                        ICT_test |= DIAT_0_ERROR;
                    }
                    timeout[1] = 0;

                    if (MODH_WriteParam_06H(SlaveBoardAddr, 262, 1) == 1) {  // test mode
                        printf("ICTStep:0x0100-->testmode OK\r\n");
                        timeout[1] = 0;
                        ICTStep |= 0x0100;
                        ICTDelay[1] = 500;
                    } else {
                        error_info("ICTStep:0x0100-->write06 error\r\n");
                        ICTDelay[1] = 60;
                    }
                } else {
                    error_info("ICTStep:0x0100-->read03H error\r\n");
                    timeout[1]++;
                    ICTDelay[1] = 60;
                }
                goto LINE3;
            }
            if ((ICTStep & 0x0200) == 0) {
                printf("ICTStep:0x0200-->");
                if (MODH_ReadParam_03H(SlaveBoardAddr, 885, 1) == 1) {  // read DI
                    _status_ICT.IO_1 = BEBufToUint16((uint8_t *)&g_tVar[SlaveBoardAddr - 1].P01[0]);
                    printf("read DI when NO = 1 OK,DI:%04x\r\n", g_tVar[SlaveBoardAddr - 1].P01[0]);
                    if (g_tVar[SlaveBoardAddr - 1].P01[0] != 0x07f3) {  //**************
                        ICT_test |= DIAT_1_ERROR;
                    }
                } else {
                    timeout[1]++;
                    error_info("write03 error\r\n");
                    ICTDelay[1] = 60;
                    goto LINE3;
                }
                printf("ICTStep:0x0200-->");
                if (MODH_ReadParam_03H(SlaveBoardAddr, 867, 3) == 1) {  // ReadAO
                    _status_ICT.AO1AI1 = BEBufToUint16((uint8_t *)&g_tVar[SlaveBoardAddr - 1].P01[0]);
                    _status_ICT.AO2AI2 = BEBufToUint16((uint8_t *)&g_tVar[SlaveBoardAddr - 1].P01[2]);

                    printf("ReadAO when NO7:1 OK,AI1:%0d,AI2:%0d\r\n", g_tVar[SlaveBoardAddr - 1].P01[0], g_tVar[SlaveBoardAddr - 1].P01[2]);

                    if (abs(g_tVar[SlaveBoardAddr - 1].P01[0] - 515) > 20) {
                        ICT_test |= AO1AI1_ERROR;
                    }
                    if (abs(g_tVar[SlaveBoardAddr - 1].P01[2] - 515) > 20) {
                        ICT_test |= AO2AI2_ERROR;
                    }
                    ICTStep |= 0x0200;
                    ICTDelay[1] = 200;
                    if (MODH_WriteParam_10H(SlaveHMIAddr, 0xA024, 4, (uint8_t *)(&_status_ICT.IO_0)) == 1) {
                        // printf("**HMIAddr WriteParam 10H OK line:%d\r\n", __LINE__);
                    } else {
                        error_info("hmi write10 error\r\n");
                    }
                } else {
                    error_info("write03 error\r\n");
                    ICTDelay[1] = 60;
                }
                goto LINE3;
            }
            if ((ICTStep & 0x0400) == 0) {
                printf("ICTStep:0x0400-->");
                if (MODH_ReadParam_03H(SlaveBoardAddr, 871, 6) == 1)  // NTC
                {
                    printf("NTC:1 OK,%0d,%0d,%0d,%0d\r\n", g_tVar[SlaveBoardAddr - 1].P01[0], g_tVar[SlaveBoardAddr - 1].P01[2], g_tVar[SlaveBoardAddr - 1].P01[4], g_tVar[SlaveBoardAddr - 1].P01[5]);

                    if (abs(g_tVar[SlaveBoardAddr - 1].P01[0] - 280) > 20)
                        ICT_test |= NTC_ERROR;
                    else
                        _status_ICT.NTC_COM |= NTC1Flag;

                    if (abs(g_tVar[SlaveBoardAddr - 1].P01[2] - 280) > 20)
                        ICT_test |= NTC_ERROR;
                    else
                        _status_ICT.NTC_COM |= NTC2Flag;

                    if (abs(g_tVar[SlaveBoardAddr - 1].P01[4] - 280) > 20)
                        ICT_test |= NTC_ERROR;
                    else
                        _status_ICT.NTC_COM |= NTC3Flag;

                    if (abs(g_tVar[SlaveBoardAddr - 1].P01[5] - 280) > 20)
                        ICT_test |= NTC_ERROR;
                    else
                        _status_ICT.NTC_COM |= NTC4Flag;
                } else {
                    error_info("write03 error\r\n");
                    goto LINE3;
                }
                printf("ICTStep:0x0400-->");
                if (MODH_ReadParam_03H(SlaveBoardAddr, 806, 2) == 1) {  // UART_P3_ERROR
                    printf("TEMP1:%0d,HUMM1:%0d\r\n", g_tVar[SlaveBoardAddr - 1].P01[0], g_tVar[SlaveBoardAddr - 1].P01[1]);
                    ICTStep |= 0x0400;
                    if ((g_tVar[SlaveBoardAddr - 1].P01[0] > 5) && (g_tVar[SlaveBoardAddr - 1].P01[1] < 1001)) {
                        COMOKP3FLAG = 1;
                    }
                } else {
                    error_info("write03 error\r\n");
                }
                goto LINE3;
            }
            if ((ICTStep & 0x0800) == 0) {
                static char flag = 0;
                if (flag < 10) {
                    printf("ICTStep:0x0800-->");
                    if (MODH_WriteParam_06H(SlaveBoardAddr, 282, 240) == 1) {
                        printf("282:write OK\r\n");
                        ICTDelay[1] = 100;
                        flag        = 10;
                    } else {
                        flag++;
                        error_info("282 write06 error\r\n");
                        ICTDelay[1] = 10;
                    }
                    goto LINE3;
                }
                flag = 0;
                printf("ICTStep:0x0800-->power off\r\n");
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_RESET);
                NO5(GPIO_PIN_RESET);
                NO6(GPIO_PIN_RESET);
                NO7(GPIO_PIN_RESET);
                NO8(GPIO_PIN_RESET);
                ICTStep |= 0x0800;
                ICTDelay[1] = 100;
                goto LINE3;
            }
            if ((ICTStep & 0x1000) == 0) {
                printf("ICTStep:0x1000-->\r\n");
                NO1(GPIO_PIN_RESET);
                NO2(GPIO_PIN_RESET);
                NO3(GPIO_PIN_RESET);
                NO4(GPIO_PIN_RESET);
                NO5(GPIO_PIN_RESET);
                NO6(GPIO_PIN_RESET);
                NO7(GPIO_PIN_RESET);
                NO8(GPIO_PIN_SET);
                ICTStep |= 0x1000;
                ICTDelay[1] = 350;
                goto LINE3;
            }
            if ((ICTStep & 0x2000) == 0) {
                printf("ICTStep:0x2000-->");
                if (MODH_ReadParam_03H(SlaveBoardAddr, 282, 1) == 1) {  // NTC
                    if (g_tVar[SlaveBoardAddr - 1].P01[0] == 240) {
                        _status_ICT.NTC_COM |= EEPROMFlag;
                        printf("eeprom OK,%0d\r\n", g_tVar[SlaveBoardAddr - 1].P01[0]);
                    } else {
                        printf("eeprom error,%0d\r\n", g_tVar[SlaveBoardAddr - 1].P01[0]);
                        ICT_test |= EEPROM_ERROR;
                    }
                } else {
                    error_info("write03 error\r\n");
                    goto LINE3;
                }
                ICTStep |= 0x2000;
                goto LINE3;
            }
        LINE3:  // Connectivity
            if (ICTStep == 0x3fff) {
                if (!UARTOKP2FLAG) {
                    ICT_test |= UART_P2_ERROR;
                } else {
                    printf("UARTOKP2FLAG\r\n");
                    _status_ICT.NTC_COM |= P2Flag;
                }
                if (!UARTOKP24FLAG) {
                    ICT_test |= UART_P24_ERROR;
                } else {
                    _status_ICT.NTC_COM |= P24Flag;
                    printf("UARTOKP24FLAG\r\n");
                }
                if (!UARTOKP29FLAG) {
                    ICT_test |= UART_P29_ERROR;
                } else {
                    printf("UARTOKP29FLAG\r\n");
                    _status_ICT.NTC_COM |= P29Flag;
                }
                if (!CANOKP28FLAG) {
                    ICT_test |= CAN_ERROR;
                } else {
                    printf("CANOKP28FLAG\r\n");
                    _status_ICT.NTC_COM |= P28Flag;
                }
                if (!COMOKP3FLAG) {
                    ICT_test |= COM_P3_ERROR;
                } else {
                    printf("COMOKP3FLAG\r\n");
                    _status_ICT.NTC_COM |= P3Flag;
                }

                printf("ICTStep:%04x ICT_test:%04x\r\n", ICTStep, ICT_test);
                _status_ICT.u16Test = BEBufToUint16((uint8_t *)&ICT_test);
                if (ICT_test) {
                    _status_ICT.u16Fsm = ICT_ST_ERR;
                } else {
                    _status_ICT.u16Fsm = ICT_ST_OK;
                }
                if (MODH_WriteParam_10H(SlaveHMIAddr, 0xA028, 2, (uint8_t *)(&_status_ICT.NTC_COM)) == 1) {
                    // printf("**HMIAddr WriteParam 10H OK line:%d\r\n", __LINE__);
                } else {
                    error_info("hmi write10 error\r\n");
                }

                timeout[1] = 0;
            RESET_PARA:
                if (MODH_WriteParam_06H(SlaveBoardAddr, 1, 2) == 1) {
                    printf("reset write 1, 2\r\n");
                    if (MODH_WriteParam_06H(SlaveBoardAddr, 0, 17) == 1) {
                        printf("reset write 0, 17\r\n");
                        osDelay(5000);
                    } else {
                        if (timeout[1] < 3) {
                            timeout[1]++;
                            goto RESET_PARA;
                        }
                        error_info("write06 error\r\n");
                    }
                } else {
                    if (timeout[1] < 3) {
                        timeout[1]++;
                        goto RESET_PARA;
                    }
                    error_info("write06 error\r\n");
                }

                _status_ICT.u16Status = ICT_STOP;
            }

            break;
        }
        case ICT_STOP: {
            _status_ICT.u16Status = ICT_IDLE;
            l_sys.u8ICT_PowerKey  = FALSE;
            if (MODH_WriteParam_06H(SlaveHMIAddr, 0xA02a, _status_ICT.u16Fsm) == 1) {
                // printf("**HMIAddr WriteParam 06H OK line:%d\r\n",__LINE__);
            } else {
                error_info("hmi write06 error\r\n");
            }
            printf("ICT_STOP\r\n\n");

            NO1(GPIO_PIN_RESET);
            NO2(GPIO_PIN_RESET);
            NO3(GPIO_PIN_RESET);
            NO4(GPIO_PIN_RESET);
            NO5(GPIO_PIN_RESET);
            NO6(GPIO_PIN_RESET);
            NO7(GPIO_PIN_RESET);
            NO8(GPIO_PIN_RESET);
        } break;
        default: {
            _status_ICT.u16Status = ICT_IDLE;
            l_sys.u8ICT_PowerKey  = FALSE;
        }
    }
}
