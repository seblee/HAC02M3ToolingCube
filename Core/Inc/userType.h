/**
 * @file userType.h
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
#ifndef _USER_TYPE_H_
#define _USER_TYPE_H_
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

typedef struct {
    unsigned char b0 : 1;
    unsigned char b1 : 1;
    unsigned char b2 : 1;
    unsigned char b3 : 1;
    unsigned char b4 : 1;
    unsigned char b5 : 1;
    unsigned char b6 : 1;
    unsigned char b7 : 1;
} _FLAG_bits;

typedef union {
    _FLAG_bits    bits;
    unsigned char byte;
} _TKS_FLAGA_type;

// ICT
enum {
    ICT_IDLE = 0,
    ICT_START,
    ICT_TEST,
    ICT_STOP,
};
enum {
    ICT_ST_NO = 0,
    ICT_ST_START,
    ICT_ST_OK,
    ICT_ST_ERR,
};

typedef struct {
    uint8_t  u8ICT_PowerKey;  // 开始测试
    uint8_t  u8ICT_Fsm;       // 测试
    uint16_t u16ICT_Delay;    // 测试
    uint8_t  u8ICT_Start;     // 测试
} local_reg_st;

enum {
    NTC1Flag    = 0x0001,
    NTC2Flag    = 0x0002,
    NTC3Flag    = 0x0004,
    NTC4Flag    = 0x0008,
    NTC5Flag    = 0x0010,
    NTC6Flag    = 0x0020,
    NTC7Flag    = 0x0040,
    NTC8Flag    = 0x0080,
    HANDOPTFlag = 0x0100,
    MONITORFlag = 0X0200,
    P25Flag     = 0X0400,
    CANFlag     = 0X0800,
    P3Flag      = 0X1000,
    EEPROMFlag  = 0X2000,
    EEVFlag     = 0X4000,
};
enum {
    PAFlag  = 0x0001,
    PBFlag  = 0x0002,
    PCFlag  = 0x0004,
    WHFlag  = 0x0008,
    CURFlag = 0x0010,
    FANFlag = 0x0020,
};

typedef struct {
    uint16_t VALUE_12V;     // 00
    uint16_t VALUE_05V;     // 01
    uint16_t VALUE_3_3V;    // 02
    uint16_t VALUE_EEV12V;  // 03
    uint16_t IO_0;          // 04
    uint16_t IO_1;          // 05
    uint16_t AO1AI1;        // 06
    uint16_t AO2AI2;        // 07
    uint16_t NTC_COM;       // 08
    uint16_t power;         // 09
    uint16_t u16Test;       // 0A
    uint16_t u16Fsm;        // 0B
    uint16_t u16Status;     // 0C
    uint16_t u16Test1;      // 0D
    uint16_t vol48V;        // 0E
    uint16_t L1_V;          // 0F
    uint16_t curCheck;      // 10
    uint16_t L1_C;          // 11
} status_st;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#endif
