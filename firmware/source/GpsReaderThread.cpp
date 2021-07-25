/**
 * @file GpsReaderThread.c
 * @brief
 */

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/
#include "GpsReaderThread.h"
#include "ChainOilerThread.h"
#include "MicroNMEA.h"

#include "hal.h"

/*****************************************************************************/
/* DEFINED CONSTANTS                                                         */
/*****************************************************************************/

/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/
typedef enum {
  GPS_INIT,
  GPS_ACQUIRING,
  GPS_LOCKED,
  GPS_ERROR,
} GpsState_t;

typedef struct Gps_s {
  GpsState_t state;
  virtual_timer_t timer;
  double speed;
  uint32_t numOfSatsInView;
} Gps_t;

/*****************************************************************************/
/* MACRO DEFINITIONS                                                         */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINITION OF GLOBAL CONSTANTS AND VARIABLES                              */
/*****************************************************************************/
static SerialConfig gps_uart_config = {
  .speed = 9600,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0,
};

static Gps_t gps;

static bool parsedSuccessfully = true;

/*****************************************************************************/
/* DECLARATION OF LOCAL FUNCTIONS                                            */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINITION OF LOCAL FUNCTIONS                                             */
/*****************************************************************************/
static void l_unknownSentenceHandler(MicroNMEA &nmea)
{
  (void)nmea;
  parsedSuccessfully = false;
}

static void l_updateStatusLedI(virtual_timer_t *tim, void *par);

static void l_stateAcquiringLedBlinkerI(virtual_timer_t *tim, void *par)
{
  (void)par;

  bool ledIsOff = (PAL_LOW == palReadLine(LINE_GPS_STATUS));

  if(ledIsOff) {
    palSetLine(LINE_GPS_STATUS);
    
    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(500), l_stateAcquiringLedBlinkerI, NULL);
    chSysUnlockFromISR();
  } else {
    palClearLine(LINE_GPS_STATUS);

    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(500), l_updateStatusLedI, NULL);
    chSysUnlockFromISR();
  }
}

static void l_stateLockedLedBlinkerI(virtual_timer_t *tim, void *par)
{
  uint32_t pulseCount = (uint32_t)par;

  if(0 < pulseCount) {

    bool ledIsOff = (PAL_LOW == palReadLine(LINE_GPS_STATUS));

    if(ledIsOff) {
      palSetLine(LINE_GPS_STATUS);
    } else {
      palClearLine(LINE_GPS_STATUS);
      pulseCount--;
    }

    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(125), l_stateLockedLedBlinkerI, (void*)pulseCount);
    chSysUnlockFromISR();

  } else {
    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(1000), l_updateStatusLedI, NULL);
    chSysUnlockFromISR();
  }
}

static void l_stateErrorLedBlinkerI(virtual_timer_t *tim, void *par)
{
  (void)par;

  bool ledIsOff = (PAL_LOW == palReadLine(LINE_GPS_STATUS));

  if(ledIsOff) {
    palSetLine(LINE_GPS_STATUS);
    
    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(1000), l_stateErrorLedBlinkerI, NULL);
    chSysUnlockFromISR();
  } else {
    palClearLine(LINE_GPS_STATUS);

    l_updateStatusLedI(tim, NULL);
  }
}

static void l_updateStatusLedI(virtual_timer_t *tim, void *par)
{
  (void)par;

  switch(gps.state) {
    case GPS_ACQUIRING: 
      l_stateAcquiringLedBlinkerI(tim, NULL);
      break;
    case GPS_LOCKED:
      l_stateLockedLedBlinkerI(tim, (void*)gps.numOfSatsInView);
      break;
    case GPS_ERROR:
      l_stateErrorLedBlinkerI(tim, NULL);
      break;
    case GPS_INIT:
      palClearLine(LINE_GPS_STATUS);
      chSysLockFromISR();
      chVTSetI(tim, chTimeMS2I(1000), l_updateStatusLedI, NULL);
      chSysUnlockFromISR();
    default: 
      break;
  }
}

static inline uint32_t l_thousandKnotToKmh(long thousandKnot)
{
  return ((double)thousandKnot * 0.001852);
}

static void l_3DFixPinInterrupt(void *p)
{
  (void)p;

  bool isRisingEdge = (PAL_HIGH == palReadLine(LINE_GPS_3DFIX));
  if(isRisingEdge) {
    COT_AdaptiveStartI();
    gps.state = GPS_LOCKED;
  } else {
    COT_AdaptiveStopI();
    gps.state = GPS_ACQUIRING;
  }
}

static void l_configure3DFixPin(void)
{
  palSetLineMode(LINE_GPS_3DFIX, PAL_MODE_INPUT);
  palSetLineCallback(LINE_GPS_3DFIX, l_3DFixPinInterrupt, NULL);
  palEnableLineEvent(LINE_GPS_3DFIX, PAL_EVENT_MODE_BOTH_EDGES);
}

/*****************************************************************************/
/* DEFINITION OF GLOBAL FUNCTIONS                                            */
/*****************************************************************************/
THD_FUNCTION(GPS_Thread, arg) {

  (void)arg;
  
  chRegSetThreadName("gps-reader-thread");

  sdStart(&SD1, &gps_uart_config);

  static char buffer[200];
  MicroNMEA nmea(buffer, (uint8_t)sizeof(buffer));
  nmea.setUnknownSentenceHandler(l_unknownSentenceHandler);
  nmea.clear();

  chVTSet(&gps.timer, chTimeMS2I(1000), l_updateStatusLedI, NULL);

  if(PAL_HIGH == palReadLine(LINE_GPS_3DFIX)) {
    gps.state = GPS_LOCKED;
    COT_AdaptiveStart();
  } else {
    gps.state = GPS_ACQUIRING;
    COT_AdaptiveStop();
  }

  l_configure3DFixPin();

  while(true) {

    msg_t c = sdGetTimeout(&SD1, TIME_MS2I(2000));
    
    if(0 < c) {

      ITM_SendChar(c);

      bool sentenceComplete = nmea.process(c);

      if (sentenceComplete) {
        if(parsedSuccessfully) {
          gps.speed = l_thousandKnotToKmh(nmea.getSpeed());
          gps.numOfSatsInView = nmea.getNumSatellites();

          COT_UpdateSpeed();
        }

        // Set this variable to flush unknown sentences.
        parsedSuccessfully = true;
      }
    } else {
      gps.state = GPS_ERROR;
      gps.speed = 0.0;
      gps.numOfSatsInView = 0U;
    }
  }
} 

void GPS_ThreadInit(void)
{
  gps.state = GPS_INIT;
  chVTObjectInit(&gps.timer);
  gps.speed = 0.0;
  gps.numOfSatsInView = 0U;
}

double GPS_GetSpeed(void)
{
  return gps.speed;
}

/****************************** END OF FILE **********************************/
