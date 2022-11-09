
#include "cmsis_os.h"  // CMSIS RTOS header file
#include "modbus_host.h"
/*----------------------------------------------------------------------------
 *      Timer: Sample timer functions
 *---------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart4;

/*----- One-Shoot Timer Example -----*/
static void Timer1_Callback(void const *arg);  // prototype for timer callback function

osTimerId timer1Id;                          // timer id
static uint32_t exec1;                       // argument for the timer call back function
static osTimerDef(Timer1, Timer1_Callback);  // define timers

// One-Shoot Timer Function
static void Timer1_Callback(void const *arg)
{
    // add user code here
    if (huart4.RxState == HAL_UART_STATE_READY)
    {
        MODH_RxTimeOut();
    }
}

/*----- Periodic Timer Example -----*/
static void Timer2_Callback(void const *arg);  // prototype for timer callback function

static osTimerId id2;   // timer id
static uint32_t exec2;  // argument for the timer call back function
static osTimerDef(Timer2, Timer2_Callback);

// Periodic Timer Example
static void Timer2_Callback(void const *arg)
{
    // add user code here
}

// Example: Create and Start timers
void Init_Timers(void)
{
   // osStatus status;  // function return status

    // Create one-shoot timer
    exec1    = 1;
    timer1Id = osTimerCreate(osTimer(Timer1), osTimerOnce, &exec1);
    if (timer1Id != NULL)
    {  // One-shot timer created
        // start timer with delay 100ms
        // status = osTimerStart(timer1Id, 100);
        // if (status != osOK)
        {
            // Timer could not be started
        }
    }

    // Create periodic timer
    exec2 = 2;
    id2   = osTimerCreate(osTimer(Timer2), osTimerPeriodic, &exec2);
    if (id2 != NULL)
    {  // Periodic timer created
        // start timer with periodic 1000ms interval
        // status = osTimerStart(id2, 1000);
        // if (status != osOK)
        {
            // Timer could not be started
        }
    }
}
