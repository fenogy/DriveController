/*
 * sync_algorithm.h
 *
 *  Created on: 23 Nov 2022
 *      Author: kasunprabash.ka
 */

#ifndef INC_SYNC_ALGORITHM_H_
#define INC_SYNC_ALGORITHM_H_

#include <string.h>
#include <math.h>
#include "command_decoder.h"

double getMeanCurrent(DriveTypeDef *drive);
void ClearCurrentBuffer(DriveTypeDef *drive);
#endif /* INC_SYNC_ALGORITHM_H_ */
