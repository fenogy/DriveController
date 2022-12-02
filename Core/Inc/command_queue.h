/*
 * command_queue.h
 *
 *  Created on: 24 Nov 2022
 *      Author: kasunprabash.ka
 */

#ifndef INC_COMMAND_QUEUE_H_
#define INC_COMMAND_QUEUE_H_

#include "main.h"
#include "command_decoder.h"
#include<stdlib.h>
#include<string.h>

extern uint8_t PollingList[4][10];
extern uint8_t MasterInitList[12][15];
extern uint8_t SlaveInitList[12][15];
extern uint8_t MasterQueue[5][20];
extern uint8_t SlaveQueue[5][20];

extern uint8_t masterInitCmdIndex;
extern uint8_t slaveInitCmdIndex;

typedef struct Node{

	uint8_t *str;
	struct node *next;

}node;

typedef struct {
	node *head;
	node *tail;
}queue;


void InitQueue(queue *q);
uint8_t Enqueue(queue *q, uint8_t *str);
uint8_t* Dequeue();

extern queue q1,q2; //has to put below ques typedef, learning for C
//extern DriveTypeDef drive1, drive2;
void copyArrays(uint8_t* source, uint8_t destination);
void CreatePollingList();
void ClearMasterQueue();
void ClearSlaveQueue();
void AddSlaveQueue();
void AddMasterQueue();
void RemoveMasterQueue();
void RemoveSlaveQueue();
void getNextMasterCommand(uint8_t *cmd);
void getNextSlaveCommand(uint8_t *cmd);


#endif /* INC_COMMAND_QUEUE_H_ */
