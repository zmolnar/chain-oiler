/**
 * @file ChainOilerThread.c
 * @brief Chain oiler module implementation.
 */

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/
#include "ChainOilerThread.h"
#include "GpsReaderThread.h"
#include "Averager.h"
#include "OilPump.h"

#include "ch.h"
#include "hal.h"

#include <string.h>

/*****************************************************************************/
/* DEFINED CONSTANTS                                                         */
/*****************************************************************************/
#define FIXED_PERIOD 4U

#define SPEED_MIN (double)(10)
#define PERIOD_MIN (double)(120)

#define SPEED_MAX (double)(100)
#define PERIOD_MAX (double)(60)

#define LINEAR_SLOPE ((PERIOD_MAX - PERIOD_MIN) / (SPEED_MAX - SPEED_MIN))
#define LINEAR_OFFSET (PERIOD_MIN - (LINEAR_SLOPE * SPEED_MIN))

#define ARRAY_LENGTH(array) (sizeof((array))/(sizeof((array)[0])))

/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/
/**
 * @brief State machine of the chain oiler.
 */
typedef enum {
  COT_STATE_INIT,            /**< First state after system start.*/
  COT_STATE_FIXED,           /**< Fixed length period. */
  COT_STATE_ADAPTIVE,        /**< Adaptively changing period length. */
  COT_STATE_FORCED_FIXED,    /**< Oil is forced to flow continuously from fixed state.*/
  COT_STATE_FORCED_ADAPTIVE, /**< Oil is forced to flow continuously from adaptive state.*/
} COT_State_t;

typedef enum {
  COT_MODE_FIXED,
  COT_MODE_ADAPTIVE,
} COT_Mode_t;

/**
 * @brief Input commands of the chain oiler.
 */
typedef enum {
  COT_CMD_ADAPTIVE_START,
  COT_CMD_ADAPTIVE_STOP,
  COT_CMD_FORCE_START,     /**< Enter force mode.*/
  COT_CMD_FORCE_STOP,      /**< Exit force mode.*/
  COT_CMD_UPDATE_SPEED,    /**< Update speed and recalculate the average.*/ 
  COT_CMD_SHOOT,           /**< Release one oil drop.*/
  COT_CMD_ONE_SHOT,        /**< Release one oil drop immediately.*/
} COT_Command_t;

/**
 * @brief Chain oiler struct.
 */
typedef struct ChainOiler_s {
    /**
     * @brief Command buffer for the mailbox.
     */
    msg_t commands[10];

    /**
     * @brief Mailbox to use for communication with the chain oiler thread.
     */
    mailbox_t mailbox;

    /**
     * @brief State machine of the chain oiler module.
     */
    COT_State_t state;

    /**
     * @brief Speed averager.
     */
    Averager_t speedAverager;

    struct Period_s {
      /**
       * @brief Virtual timer to measure time for releasing oil drops.
       */
      virtual_timer_t timer;
      
      /**
       * @brief Start time-stamp of period measurement.
       */
      systime_t start;

      /**
       * @brief Average speed of the last period.
       */
      double speed;

      /**
       * @brief Length of the last period in seconds.
       */
      time_secs_t length;
    } period;
} ChainOiler_t;

/*****************************************************************************/
/* MACRO DEFINITIONS                                                         */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINITION OF GLOBAL CONSTANTS AND VARIABLES                              */
/*****************************************************************************/
/**
 * @brief Chain oiler module.
 */
static ChainOiler_t chainOiler;

/*****************************************************************************/
/* DECLARATION OF LOCAL FUNCTIONS                                            */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINITION OF LOCAL FUNCTIONS                                             */
/*****************************************************************************/
static void l_timerCallbackI(virtual_timer_t *tim, void *p)
{
  (void)tim;
  (void)p;

  chSysLockFromISR();
  chainOiler.period.start = chVTGetSystemTimeX();
  chMBPostI(&chainOiler.mailbox, COT_CMD_SHOOT);
  chSysUnlockFromISR();
}

static time_secs_t l_calculatePeriodLength(double speed)
{
  double sec = 0;
  if (speed < SPEED_MIN) {
    sec = 0.0;
  } else if ((SPEED_MIN <= speed) && (speed < SPEED_MAX)) {
    sec = LINEAR_SLOPE * speed + LINEAR_OFFSET;
  } else {
    sec = PERIOD_MAX;
  }

  return (time_secs_t)sec;
}

static void l_startTimer(uint32_t delayInSec)
{
  chSysLock();
  chVTResetI(&chainOiler.period.timer);
  chVTSetI(&chainOiler.period.timer,
           TIME_S2I(delayInSec),
           l_timerCallbackI,
           NULL);
  chSysUnlock();  
}

static void l_stopTimer(void)
{
  chSysLock();
  chVTResetI(&chainOiler.period.timer);
  chSysUnlock();  
}

static void l_sendSelfMessage(COT_Command_t cmd)
{
  chSysLock();
  chMBPostI(&chainOiler.mailbox, cmd);
  chSysUnlock();
}

static time_secs_t l_getElapsedTime(systime_t start)
{
  sysinterval_t timeSinceStart = chVTTimeElapsedSinceX(start);
  return TIME_I2S(timeSinceStart);
}

static void l_updateSpeedAndRecalculateTiming(void)
{
  double speed = GPS_GetSpeed();
  AVG_Put(&chainOiler.speedAverager, speed);
  double avgSpeed = AVG_GetAverage(&chainOiler.speedAverager);

  chainOiler.period.speed = avgSpeed;
  chainOiler.period.length = l_calculatePeriodLength(avgSpeed);

  if (0 < chainOiler.period.length) {

    time_secs_t elapsedTime = l_getElapsedTime(chainOiler.period.start);

    if (elapsedTime < chainOiler.period.length) {
      time_secs_t timeToWait = chainOiler.period.length - elapsedTime;
      l_startTimer(timeToWait);
    } else {
      l_stopTimer();
      l_sendSelfMessage(COT_CMD_SHOOT);
    }
  } else {
    AVG_Clear(&chainOiler.speedAverager);
    l_stopTimer();
  }
}

static void l_fixedStateHandler(COT_Command_t cmd)
{
  switch (cmd) {
  case COT_CMD_ADAPTIVE_START: {
    l_stopTimer();
    chainOiler.period.start = chVTGetSystemTimeX();
    chainOiler.state = COT_STATE_ADAPTIVE;    
    break;
  }
  case COT_CMD_FORCE_START: {
    l_stopTimer();
    OLP_Start();    
    chainOiler.state = COT_STATE_FORCED_FIXED;
    break;
  }
  case COT_CMD_SHOOT: {
    l_startTimer(FIXED_PERIOD);
    OLP_ReleaseOneDrop();
    break;
  }
  case COT_CMD_ONE_SHOT: {
    OLP_ReleaseOneDrop();
    break;
  }
  case COT_CMD_ADAPTIVE_STOP:
  case COT_CMD_UPDATE_SPEED: 
  case COT_CMD_FORCE_STOP:
  default:
    break;
  }
}

static void l_adaptiveStateHandler(COT_Command_t cmd)
{
  switch (cmd) {
  case COT_CMD_ADAPTIVE_STOP: {
    
    l_stopTimer();

    time_secs_t elapsedTime = l_getElapsedTime(chainOiler.period.start);

    if(elapsedTime < FIXED_PERIOD) {  
      time_secs_t timeToWait = FIXED_PERIOD - elapsedTime;
      l_startTimer(timeToWait);
    } else {
      l_sendSelfMessage(COT_CMD_SHOOT);
    }
    
    chainOiler.state = COT_STATE_FIXED;
    break;
  }
  case COT_CMD_FORCE_START: {
    l_stopTimer();
    OLP_Start();    
    chainOiler.state = COT_STATE_FORCED_ADAPTIVE;
    break;
  }
  case COT_CMD_SHOOT: {

    double speed = GPS_GetSpeed();
    
    if (SPEED_MIN < speed) {
      AVG_Clear(&chainOiler.speedAverager);
      OLP_ReleaseOneDrop();
    }
    break;
  }
  case COT_CMD_ONE_SHOT: {
    OLP_ReleaseOneDrop();
    break;
  }
  case COT_CMD_UPDATE_SPEED: {
    l_updateSpeedAndRecalculateTiming();
    break;
  }
  case COT_CMD_ADAPTIVE_START:
  case COT_CMD_FORCE_STOP:
  default:
    break;
  }
}

static void l_forcedFixedStateHandler(COT_Command_t cmd)
{
  switch (cmd) {
  case COT_CMD_FORCE_STOP: {    
    OLP_Stop();
    l_startTimer(FIXED_PERIOD);    
    chainOiler.period.start = chVTGetSystemTimeX();
    chainOiler.state = COT_STATE_FIXED;
    break;
  }
  case COT_CMD_ADAPTIVE_START:
  case COT_CMD_ADAPTIVE_STOP:
  case COT_CMD_FORCE_START:
  case COT_CMD_UPDATE_SPEED:
  case COT_CMD_SHOOT:
  case COT_CMD_ONE_SHOT:
  default:
    break;
  }
}

static void l_forcedAdaptiveStateHandler(COT_Command_t cmd)
{
  switch (cmd) {
  case COT_CMD_FORCE_STOP: {    
    OLP_Stop();
    chainOiler.period.start = chVTGetSystemTimeX();
    chainOiler.state = COT_STATE_ADAPTIVE;
  }
  case COT_CMD_ADAPTIVE_START:
  case COT_CMD_ADAPTIVE_STOP:
  case COT_CMD_FORCE_START:
  case COT_CMD_UPDATE_SPEED:
  case COT_CMD_SHOOT:
  case COT_CMD_ONE_SHOT:
  default:
    break;
  }
}

/*****************************************************************************/
/* DEFINITION OF GLOBAL FUNCTIONS                                            */
/*****************************************************************************/
THD_FUNCTION(COT_Thread, arg)
{
  (void)arg;
  chRegSetThreadName(CHAIN_OILER_THREAD_NAME);

  chSysLock();
  chVTResetI(&chainOiler.period.timer);
  chVTSetI(&chainOiler.period.timer,
           TIME_S2I(FIXED_PERIOD),
           l_timerCallbackI,
           NULL);
  chSysUnlock();

  while (true) {
    msg_t msg;
    if (MSG_OK == chMBFetchTimeout(&chainOiler.mailbox, &msg, TIME_INFINITE)) {
      COT_Command_t cmd = (COT_Command_t)msg;
      switch (chainOiler.state) {
      case COT_STATE_FIXED: {
        l_fixedStateHandler(cmd);
        break;
      }
      case COT_STATE_ADAPTIVE: {
        l_adaptiveStateHandler(cmd);
        break;
      }
      case COT_STATE_FORCED_FIXED: {
        l_forcedFixedStateHandler(cmd);
        break;
      }
      case COT_STATE_FORCED_ADAPTIVE: {
        l_forcedAdaptiveStateHandler(cmd);
        break;
      }
      case COT_STATE_INIT: 
      default: {
        break;
      }
      }
    }
  }
}

void COT_Init(void)
{
  memset(&chainOiler.commands, 0, sizeof(chainOiler.commands));
  chMBObjectInit(&chainOiler.mailbox, chainOiler.commands, ARRAY_LENGTH(chainOiler.commands));
  chainOiler.state = COT_STATE_FIXED;
  AVG_Init(&chainOiler.speedAverager);
  chVTObjectInit(&chainOiler.period.timer);
  chainOiler.period.start = 0;
  chainOiler.period.speed = 0;
  chainOiler.period.length = 0;
}

void COT_AdaptiveStartI(void)
{
  chMBPostI(&chainOiler.mailbox, COT_CMD_ADAPTIVE_START);
}

void COT_AdaptiveStart(void)
{
  chSysLock();
  COT_AdaptiveStartI();
  chSysUnlock();
}

void COT_AdaptiveStopI(void)
{
  chMBPostI(&chainOiler.mailbox, COT_CMD_ADAPTIVE_STOP);
}

void COT_AdaptiveStop(void)
{
  chSysLock();
  COT_AdaptiveStopI();
  chSysUnlock();
}

void COT_ForceStartI(void)
{
  chMBPostI(&chainOiler.mailbox, COT_CMD_FORCE_START);
}

void COT_ForceStart(void)
{
  chSysLock();
  COT_ForceStartI();
  chSysUnlock();
}

void COT_ForceStopI(void)
{
  chMBPostI(&chainOiler.mailbox, COT_CMD_FORCE_STOP);
}

void COT_ForceStop(void)
{
  chSysLock();
  COT_ForceStopI();
  chSysUnlock();
}

void COT_UpdateSpeedI(void)
{
  chMBPostI(&chainOiler.mailbox, COT_CMD_UPDATE_SPEED);
}

void COT_UpdateSpeed(void)
{
  chSysLock();
  COT_UpdateSpeedI();
  chSysUnlock();
}

void COT_OneShotI(void)
{
  chMBPostI(&chainOiler.mailbox, COT_CMD_ONE_SHOT);
}

void COT_OneShot(void)
{
  chSysLock();
  COT_OneShotI();
  chSysUnlock();
}

/****************************** END OF FILE **********************************/
