/**
 * @file ChainOilerThread.h
 * @brief Declaration of the chain oiler module interface.
 */

#ifndef CHAIN_OILER_THREAD_H
#define CHAIN_OILER_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/
#include "ch.h"

/*****************************************************************************/
/* DEFINED CONSTANTS                                                         */
/*****************************************************************************/
#define CHAIN_OILER_THREAD_NAME "chainoiler"

/*****************************************************************************/
/* MACRO DEFINITIONS                                                         */
/*****************************************************************************/

/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* DECLARATION OF GLOBAL VARIABLES                                           */
/*****************************************************************************/

/*****************************************************************************/
/* DECLARATION OF GLOBAL FUNCTIONS                                           */
/*****************************************************************************/
/**
 * @brief Thread function of the chain oiler.
 */
THD_FUNCTION(COT_Thread, arg);

/**
 * @brief Initialize resources that are used by the chain oiler module.
 */
void COT_Init(void);

/**
 * @brief Start chain oiler from kernel locked state.
 */
void COT_AdaptiveStartI(void);

/**
 * @brief Start chain oiler.
 */
void COT_AdaptiveStart(void);

/**
 * @brief Stop chain oiler from kernel locked state.
 */
void COT_AdaptiveStopI(void);

/**
 * @brief Stop chain oiler.
 */
void COT_AdaptiveStop(void);

/**
 * @brief Force chain oiler to operate continuously from kernel locked state.
 */
void COT_ForceStartI(void);

/**
 * @brief Force chain oiler to operate continuously.
 */
void COT_ForceStart(void);

/**
 * @brief Stop chain oiler continuous operation, return to normal, enabled
 * state. I-class API function.
 */
void COT_ForceStopI(void);

/**
 * @brief Stop chain oiler continuous operation, return to normal, enabled
 * state.
 */
void COT_ForceStop(void);

/**
 * @brief Speed value is ready to read from kernel locked state.
 */
void COT_UpdateSpeedI(void);

/**
 * @brief Speed value is ready to read.
 */
void COT_UpdateSpeed(void);

/**
 * @brief Release one oil drop from kernel locked state.
 */
void COT_OneShotI(void);

/**
 * @brief Release one oil drop.
 */
void COT_OneShot(void);

/**
 * @brief Get the string describing the state of the chain oiler module.
 */
const char *COT_GetStateString(void);

/**
 * @brief Get the error descriptor string of the chain oiler module.
 */
const char *COT_GetErrorString(void);

#ifdef __cplusplus
}
#endif

#endif /* CHAIN_OILER_THREAD_H */

/****************************** END OF FILE **********************************/
