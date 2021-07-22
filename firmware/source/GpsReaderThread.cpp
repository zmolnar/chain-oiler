/**
 * @file GpsReaderThread.c
 * @brief
 */

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/
#include "GpsReaderThread.h"
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
static void unknownSentenceHandler(MicroNMEA &nmea)
{
  (void)nmea;
  parsedSuccessfully = false;
}

static bool isGpsLocked(void)
{
  return (PAL_HIGH == palReadLine(LINE_GPS_3DFIX));
}

static void updateStatusLedI(virtual_timer_t *tim, void *par);

static void stateAcquiringLedBlinkerI(virtual_timer_t *tim, void *par)
{
  (void)par;

  bool ledIsOff = (PAL_LOW == palReadLine(LINE_GPS_STATUS));

  if(ledIsOff) {
    palSetLine(LINE_GPS_STATUS);
    
    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(500), stateAcquiringLedBlinkerI, NULL);
    chSysUnlockFromISR();
  } else {
    palClearLine(LINE_GPS_STATUS);

    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(500), updateStatusLedI, NULL);
    chSysUnlockFromISR();
  }
}

static void stateLockedLedBlinkerI(virtual_timer_t *tim, void *par)
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
    chVTSetI(tim, chTimeMS2I(125), stateLockedLedBlinkerI, (void*)pulseCount);
    chSysUnlockFromISR();

  } else {
    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(1000), updateStatusLedI, NULL);
    chSysUnlockFromISR();
  }
}

static void stateErrorLedBlinkerI(virtual_timer_t *tim, void *par)
{
  (void)par;

  bool ledIsOff = (PAL_LOW == palReadLine(LINE_GPS_STATUS));

  if(ledIsOff) {
    palSetLine(LINE_GPS_STATUS);
    
    chSysLockFromISR();
    chVTSetI(tim, chTimeMS2I(1000), stateErrorLedBlinkerI, NULL);
    chSysUnlockFromISR();
  } else {
    palClearLine(LINE_GPS_STATUS);

    updateStatusLedI(tim, NULL);
  }
}

static void updateStatusLedI(virtual_timer_t *tim, void *par)
{
  (void)par;

  switch(gps.state) {
    case GPS_ACQUIRING: 
      stateAcquiringLedBlinkerI(tim, NULL);
      break;
    case GPS_LOCKED:
      stateLockedLedBlinkerI(tim, (void*)gps.numOfSatsInView);
      break;
    case GPS_ERROR:
      stateErrorLedBlinkerI(tim, NULL);
      break;
    case GPS_INIT:
      palClearLine(LINE_GPS_STATUS);
      chSysLockFromISR();
      chVTSetI(tim, chTimeMS2I(1000), updateStatusLedI, NULL);
      chSysUnlockFromISR();
    default: 
      break;
  }
}

static inline uint32_t thousandKnotToKmh(long thousandKnot)
{
  return ((double)thousandKnot * 0.001852);
}

/*****************************************************************************/
/* DEFINITION OF GLOBAL FUNCTIONS                                            */
/*****************************************************************************/
THD_FUNCTION(GpsReaderThread, arg) {

  (void)arg;
  
  chRegSetThreadName("gps-reader-thread");

  sdStart(&SD1, &gps_uart_config);

  static char buffer[200];
  MicroNMEA nmea(buffer, (uint8_t)sizeof(buffer));
  nmea.setUnknownSentenceHandler(unknownSentenceHandler);
  nmea.clear();

  chVTSet(&gps.timer, chTimeMS2I(1000), updateStatusLedI, NULL);

  while(true) {

    msg_t c = sdGetTimeout(&SD1, TIME_MS2I(2000));
    
    if(0 < c) {

      ITM_SendChar(c);

      bool sentenceComplete = nmea.process(c);

      if (sentenceComplete) {
        if(parsedSuccessfully) {
          gps.speed = thousandKnotToKmh(nmea.getSpeed());
          gps.numOfSatsInView = nmea.getNumSatellites();
        }

        // Set this variable to flush unknown sentences.
        parsedSuccessfully = true;
      }

      gps.state = isGpsLocked() ? GPS_LOCKED : GPS_ACQUIRING;

      if(GPS_LOCKED == gps.state) {
        // TODO send speed to chain oiler thread
      }

    } else {
      gps.state = GPS_ERROR;
      gps.speed = 0.0;
      gps.numOfSatsInView = 0U;
    }
  }
} 

void GpsReaderThreadInit(void)
{
  gps.state = GPS_INIT;
  chVTObjectInit(&gps.timer);
  gps.speed = 0.0;
  gps.numOfSatsInView = 0U;
}

double GpsGetSpeed(void)
{
  return gps.speed;
}

/****************************** END OF FILE **********************************/
