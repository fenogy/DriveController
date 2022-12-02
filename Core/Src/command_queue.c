/*
 * command_queue.c
 *
 *  Created on: 24 Nov 2022
 *      Author: kasunprabash.ka
 */

#include "command_queue.h"

/*adding this on .h file caused an error, Need to check*/
extern DriveTypeDef drive1, drive2;

void InitQueue(queue *q){

	q->head = NULL;
	q->tail = NULL;
}

uint8_t Enqueue(queue *q, uint8_t *str){

	/*Create a new node */
	node *newNode 	= malloc(sizeof(node));

	if(newNode == NULL) return FALSE;

	newNode->str	= str;
	newNode->next 		= NULL;

	/*Set tails*/
	if(q->tail != NULL){

		q->tail->next 	= newNode;
	}

	q->tail 		= newNode;

	/*Set head*/
	if(q->head == NULL){
		q->head 	= newNode;
	}

	return TRUE;
}

uint8_t* Dequeue(queue *q){

	/*Check the emptiness*/
	if(q->head == NULL) return NULL;

	/*Save the head of the Queue*/
	node *tmp 		= q->head;
	uint8_t *str 	= tmp->str;
	q->head 		= q->head->next;

	if(q->head == NULL){
		q->tail = NULL;
	}
	free(tmp);
	return str;
}


void CreatePollingList(){

	uint8_t SR_CMD[] = "SR\r\0";
	uint8_t PX_CMD[] = "PX\r\0";
	uint8_t IQ_CMD[] = "IQ\r\0";

	memcpy(PollingList[0],IQ_CMD,4);
	memcpy(PollingList[1],PX_CMD,4);
	memcpy(PollingList[2],SR_CMD,4);

}

void CreateInitList(){

	memcpy(MasterInitList[0],"SR\r\0",4);
	memcpy(MasterInitList[1],"MO\r\0",4);
	memcpy(MasterInitList[2],"LD\r\0",4);
	memcpy(MasterInitList[3],"UM=5\r\0",6);
	memcpy(MasterInitList[4],"AC=3000\r\0",9);
	memcpy(MasterInitList[5],"DC=3000\r\0",9);
	memcpy(MasterInitList[6],"UM\r\0",4);
	memcpy(MasterInitList[7],"SP=20000\r\0",10);
	memcpy(MasterInitList[8],"MO=1\r\0",6);
	memcpy(MasterInitList[9],"SR\r\0",4);
	//memcpy(MasterInitList[10],"LD\r\0",4);
	memcpy(MasterInitList[10],"MO\r\0",4);
//	memcpy(MasterInitList[0],"SR\r\0",4);
//	memcpy(MasterInitList[1],"SR\r\0",4);
//	memcpy(MasterInitList[2],"AC\r\0",4);
//	memcpy(MasterInitList[3],"DC\r\0",4);
//	memcpy(MasterInitList[4],"IQ\r\0",4);
//	memcpy(MasterInitList[5],"IQ\r\0",4);
//	memcpy(MasterInitList[6],"IQ\r\0",4);
//	memcpy(MasterInitList[7],"IQ\r\0",4);
//	memcpy(MasterInitList[8],"IQ\r\0",4);
//	memcpy(MasterInitList[9],"SR\r\0",4);
//	memcpy(MasterInitList[10],"IQ\r\0",4);

	memcpy(SlaveInitList[0],"SR\r\0",4);
	memcpy(SlaveInitList[1],"MO\r\0",4);
	memcpy(SlaveInitList[2],"LD\r\0",4);
	memcpy(SlaveInitList[3],"UM=1\r\0",6);
	memcpy(SlaveInitList[4],"AC=3000\r\0",9);
	memcpy(SlaveInitList[5],"DC=3000\r\0",9);
	memcpy(SlaveInitList[6],"UM\r\0",4);
	memcpy(SlaveInitList[7],"SP=20000\r\0",10);
	memcpy(SlaveInitList[8],"MO=1\r\0",6);
	memcpy(SlaveInitList[9],"SR\r\0",4);
	//memcpy(SlaveInitList[10],"LD\r\0",4);
	memcpy(SlaveInitList[10],"MO=1\r\0",4);

}

void getNextMasterCommand(uint8_t *cmd){

	if(isMasterInit == FALSE){

		if(MasterInitList[masterInitCmdIndex][0] != 0 && masterInitCmdIndex < MAX_INIT ){


			for(int i = 0; i < 14; i++){
				*(cmd + i) = 0;
				*(cmd + i) = MasterInitList[masterInitCmdIndex][i];
			}

			//copyArrays((uint8_t*)MasterInitList[masterInitCmdIndex], cmd);
			fetchNextMaster = FALSE;
			masterInitCmdIndex++;


		}else if(masterInitCmdIndex >= MAX_INIT){
			isMasterInit = TRUE;

		}else{

		}
	}else if(isMasterInit == TRUE){

		/*Check if there is any user command or sync command*/
		uint8_t * tmp = Dequeue(&q1);
		if(tmp != NULL){

			//cmd = tmp; // this will not modify the pointer, either return pointer or fill it
			for(int i = 0; i < 14; i++){
				*(cmd + i) = 0;
				*(cmd + i) = *(tmp+i);
			}
			//copyArrays(tmp, cmd);
			return;

		}else{	/*If no user command check whether need to poll and send poll cmd*/

			if(isPolling == TRUE && isPositionCheck == TRUE){
				return;
			}else if(isPolling == TRUE && isPositionCheck == FALSE){

				for(int i = 0; i < 14; i++){
					*(cmd + i) = 0;
					*(cmd + i) = PollingList[0][i];
				}
				return;
			}
		}
	}
}

void getNextSlaveCommand(uint8_t *cmd){

	if(isSlaveInit == FALSE){

			if(SlaveInitList[slaveInitCmdIndex][0] != 0 && slaveInitCmdIndex < MAX_INIT ){


				for(int i = 0; i < 14; i++){
					*(cmd + i) = 0;
					*(cmd + i) = SlaveInitList[slaveInitCmdIndex][i];
				}

				//copyArrays((uint8_t*)SlaveInitList[slaveInitCmdIndex], cmd);
				fetchNextSlave = FALSE;
				slaveInitCmdIndex++;


			}else if(slaveInitCmdIndex >= MAX_INIT){
				isSlaveInit = TRUE;

			}else{

			}
		}else if(isSlaveInit == TRUE){

			/*Check if there is any user command or sync command*/
			uint8_t * tmp = Dequeue(&q2);
			if(tmp != NULL){

				//cmd = tmp; // this will not modify the pointer, either return pointer or fill it
				for(int i = 0; i < 14; i++){
					*(cmd + i) = 0;
					*(cmd + i) = *(tmp+i);
				}
				//copyArrays(tmp, cmd);
				return;

			}else{	/*If no user command check whether need to poll and send poll cmd*/

				if(isPolling == TRUE){

					/*Create the current command using the drive1->syncCmd*/
					/*syncCmd is created with algorithmically derived drive1->syncCurrent*/
					CreateCurrentCommand(&drive1);

					/*drive1->syncCmd is populated, copy to cmd*/
					for(int i = 0; i < 14; i++){
						*(cmd + i) = 0;
						*(cmd + i) = drive1.syncCmd[i];
					}
					return;

				}else{


				}
			}
		}
}

//void copyArrays(uint8_t *source, uint8_t *destination){
//
//	for(int i = 0; i < 14; i++){
//		*(destination + i) = 0;
//		*(destination + i) = *(source+i);
//	}
//
//}
void ClearMasterQueue(){


}

void ClearSlaveQueue(){


}

void AddSlaveQueue(){

}

void AddMasterQueue(){

}

void RemoveMasterQueue(){

}

void RemoveSlaveQueue(){

}
