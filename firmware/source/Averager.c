/**
 * @file Averager.c
 * @brief
 */

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/
#include "Averager.h"
#include <string.h>

/*****************************************************************************/
/* DEFINED CONSTANTS                                                         */
/*****************************************************************************/

/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* MACRO DEFINITIONS                                                         */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINITION OF GLOBAL CONSTANTS AND VARIABLES                              */
/*****************************************************************************/

/*****************************************************************************/
/* DECLARATION OF LOCAL FUNCTIONS                                            */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINITION OF LOCAL FUNCTIONS                                             */
/*****************************************************************************/

/*****************************************************************************/
/* DEFINITION OF GLOBAL FUNCTIONS                                            */
/*****************************************************************************/
void AVG_Init(Averager_t *pavg)
{
    pavg->index = 0;
    memset(pavg->buffer, 0, sizeof(pavg->buffer));
    pavg->isFull = false;
}

void AVG_Clear(Averager_t *pavg)
{
    AVG_Init(pavg);
}

void AVG_Put(Averager_t *pavg, double value)
{
    pavg->buffer[pavg->index] = value;

    if (!pavg->isFull) {
        pavg->isFull = ((CIRCULAR_BUFFER_LENGTH-1) <= pavg->index);
    }

    pavg->index = (pavg->index+1) % CIRCULAR_BUFFER_LENGTH;
}

double AVG_GetAverage(Averager_t *pavg)
{
    size_t length = (pavg->isFull) ? CIRCULAR_BUFFER_LENGTH : pavg->index;

    double sum;
    size_t i;
    for(i = 0, sum = 0.0; i < length; ++i)
        sum += pavg->buffer[i];

    double average = sum / (double)length;
    return average;
}

/****************************** END OF FILE **********************************/
