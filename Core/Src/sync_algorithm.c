/*
 * sync_algorithm.c
 *
 *  Created on: 23 Nov 2022
 *      Author: kasunprabash.ka
 *      This will calculate the best sync current, will include the following
 *      implementations.
 *      1.Mean filter + Proportion
 *      2.PID controller
 *      3.Low pass filter
 */


#include "sync_algorithm.h"

uint8_t	samples = 20;

double getMeanCurrent(DriveTypeDef *drive){

        double mean = 0.0;
        double sum = 0.0;
        //calculate the average
        for(int i =0; i< samples;i++){
            sum = sum + drive->currentRecords[i];
        }
        mean  = sum/samples;
        return mean;

}

void ClearCurrentBuffer(DriveTypeDef *drive){ /*Clear Buffer*/
	drive->currentRecordIndex = 0;
	memset(drive->currentRecords, 0, samples);

}


