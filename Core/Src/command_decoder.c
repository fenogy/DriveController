/*
 * commanddecoder.c
 *
 *  Created on: Nov 2, 2022
 *      Author: kasunprabash.ka
 */
#include "command_decoder.h"
#include <string.h>
#include <math.h> /*Needed for round off and other basic calculations*/

#define MAX_PRECISION	(10)

extern DAC_HandleTypeDef hdac;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;


extern uint8_t UART1_rxBuffer[2];
extern uint8_t UART2_rxBuffer[2];
extern uint8_t UART3_rxBuffer[2];

extern DriveTypeDef drive1;			/*Drive 1 object */
extern DriveTypeDef drive2;			/*Drive 2 object*/
extern UserCommandTypeDef userCmd;	/*User command structure*/
extern bool isStart;
extern bool isPolling;				/*Need to poll drive currents and status continuously*/
extern bool isLogging;				/*Need to log drive communications to PC over UART3*/
extern bool isGraphing;				/*Need to send PC the simple values for real time graphing*/
extern bool isSyncing;				/*Sync Motors or run solo mode*/
extern bool isPositionCheck;
//extern bool isDelayUpdate;
//extern bool isMasterInit;
//extern bool isSlaveInit;
extern uint16_t interCmdDelay;
//extern bool fetchNextMaster;
//extern bool fetchNextSlave;


uint8_t masterInitCmdIndex;
uint8_t slaveInitCmdIndex;

uint8_t intTimes 		= 0;
uint8_t commaIndex 		= 0;
uint8_t commaNumber 	= 0;
uint8_t equalsIndex 	= 0;
uint8_t bracketIndex 	= 0;
uint8_t junkIndex 		= 0;
uint8_t crIndex 		= 0;
//uint8_t *arg, *cmd;
uint8_t commandStartIndex 	= 0;
uint8_t argumentStartIndex 	= 0;
uint8_t length 				= 0;
uint8_t cmd[2];
uint8_t arg[12];
double value;



static uint8_t SYNC_CMD[] 		= "SYNC";
static uint8_t NOSYNC_CMD[] 	= "NOSYNC";
static uint8_t DELAY_CMD[] 		= "DELAY=";
static uint8_t POLL_CMD[] 		= "POLL";
static uint8_t NOPOLL_CMD[] 	= "NOPOLL";
static uint8_t GRAPH_ON_CMD[] 	= "GON";
static uint8_t GRAPH_OFF_CMD[] 	= "GOFF";
static uint8_t POS_ON_CMD[] 	= "PON";
static uint8_t POS_OFF_CMD[] 	= "POFF";
static uint8_t CONNECT_CMD[] 	= "CON";
static uint8_t BEGIN_CMD[] 		= "BG";

uint8_t PollingList[4][10];
uint8_t MasterInitList[12][15];
uint8_t SlaveInitList[12][15];
uint8_t MasterQueue[5][20];
uint8_t SlaveQueue[5][20];
uint8_t pollingIndex;

extern uint8_t	samples;
extern double getMeanCurrent(DriveTypeDef *drive);
//uint8_t masterUser

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1){

		  HAL_UART_Receive_IT (&huart1, UART1_rxBuffer, 1);
		  //writeStringPC(UART1_rxBuffer);
		  ProcessReceiveBuffer(&drive1, UART1_rxBuffer[0]);


	}
	if(huart->Instance==USART2){

		  HAL_UART_Receive_IT (&huart2, UART2_rxBuffer, 1);
		  ProcessReceiveBuffer(&drive2, UART2_rxBuffer[0]);
	}
	if(huart->Instance==USART3){

		//Since the incoming speed is slow we process more in call back
		HAL_UART_Receive_IT (&huart3, UART3_rxBuffer, 1);
		userCmd.rxBuffer[userCmd.rxBufPtr++] = UART3_rxBuffer[0];
		//Too long
		if(userCmd.rxBufPtr > 11){
			memset(userCmd.rxBuffer, 0, 12);
			userCmd.rxBufPtr = 0;
		}
		//check if we receive an CR
		if(UART3_rxBuffer[0] == '\r'){
			//ProcessUserBuffer(&userCmd, UART3_rxBuffer[0]);
		}


	}
}

void InitDriveData(DriveTypeDef *drive){

	drive->acceleration = 0.0;
	drive->currentReal = 0.0;
	drive->currentImaginary = 0.0;
	drive->syncCurrent = 0.0;
	drive->driveMode = 5;
	drive->isConnected = FALSE;
	drive->isForwardMotion = TRUE;
	drive->deceleration = 2000;
	drive->acceleration = 2000;
	drive->speed = 10000;
	drive->isHome = FALSE;
	drive->rxBufPtr = 0;
	drive->status = 0;
	drive->isCmdAvailable = FALSE;
	drive->inSync  = FALSE;
	drive->motorOn = 0;
	drive->currentReal = value;
	memset(drive->currentRecords,0,100);
	memset(drive->syncCmd,0,15);
	memset(drive->rxBuffer,0,30);
	memset(drive->cmd,0,3);
	memset(drive->nextCmd,0,15);
	memset(drive->args,0,15);
	drive->currentRecordIndex = 0;

	if(drive->isMaster == FALSE){

		drive->driveMode = 1;
	}

}

void ProcessReceiveBuffer(DriveTypeDef *drive, uint8_t singleData){

	commaIndex = 0;
	commaNumber = 0;
	equalsIndex = 0;
	bracketIndex = 0;
	junkIndex = 0;
	crIndex = 0;


	commandStartIndex = 0;
	argumentStartIndex = 0;
	length = 0;

	uint8_t i = 0;

	//Buffering
	//if(singleData >=10 ){
		if(singleData >=10 && singleData <= 94){

		drive->rxBuffer[drive->rxBufPtr++] = singleData;
		//return;
		if(drive->rxBufPtr > 25){

			ClearReceiveBuffer(drive);

		}

	}

	if(drive->rxBufPtr > 3){
		CalculateIndices(drive->rxBuffer);
		length = strlen((char *)drive->rxBuffer);

		if(commaIndex > 0 && drive->isCmdAvailable == FALSE ){	/*When ; present and flag is down, command has arrived*/

			if(commaIndex >= 0 && length < 3){ /*When ; present and length is 2 or less, not a full command*/
				ClearReceiveBuffer(drive);
				return;
			}

			if(junkIndex > 0 || commaNumber > 1 || commaIndex < crIndex ){
				ClearReceiveBuffer(drive);
				return;
			}

			//
			if(commaIndex > length || (drive->rxBuffer[0] < 65 ||drive->rxBuffer[0]  > 90)){ /*Non ASCII start of command*/
				ClearReceiveBuffer(drive);
				return;
			}

			//Check Format of a command is OK
			if(commaIndex > crIndex){


				//Clear stuff
				drive->isCmdAvailable = TRUE;
				memset(drive->cmd, 0, 3);
				memset(drive->args, 0, 15);

				//Update the drive with the values received from the command.
				for(i = (crIndex +1); i<commaIndex;i++){
					arg[i-crIndex -1] = drive->rxBuffer[i];
					drive->args[i-crIndex -1] = drive->rxBuffer[i];
				}

				//Check both Initials are Uppercase Alpahabet
				if((drive->rxBuffer[0] > 65 ||drive->rxBuffer[0]  < 90) &&
					(drive->rxBuffer[1] > 65 ||drive->rxBuffer[1]  < 90)){

					drive->cmd[0] = drive->rxBuffer[0];
					drive->cmd[1] = drive->rxBuffer[1];
					drive->cmd[2] = 0;
				}

				value = atof(drive->args);
				commandStartIndex = 1;



			}

		}

	}

}

void CheckReceiveBuffer(DriveTypeDef *drive){

	commaIndex = 0;
	commaNumber = 0;
	equalsIndex = 0;
	bracketIndex = 0;
	junkIndex = 0;
	crIndex = 0;


	commandStartIndex = 0;
	argumentStartIndex = 0;
	length = 0;

	uint8_t i = 0;

	/*Buffering is already done inside the ISR*/
	if(drive->rxBufPtr > 25){

			ClearReceiveBuffer(drive);

	}


	/*At least 3 chars need to check for a command*/
	if(drive->rxBufPtr > 3){
		CalculateIndices(drive->rxBuffer);
		length = strlen((char *)drive->rxBuffer);

		if(commaIndex > 0 && drive->isCmdAvailable == FALSE ){	/*When ; present and flag is down, command has arrived*/

			if(commaIndex >= 0 && length < 3){ /*When ; present and length is 2 or less, not a full command*/
				ClearReceiveBuffer(drive);
				return;
			}

			if(junkIndex > 0 || commaNumber > 1 || commaIndex < crIndex ){
				ClearReceiveBuffer(drive);
				return;
			}

			//
			if(commaIndex > length || (drive->rxBuffer[0] < 65 ||drive->rxBuffer[0]  > 90)){ /*Non ASCII start of command*/
				ClearReceiveBuffer(drive);
				return;
			}

			//Check Format of a command is OK
			if(commaIndex > crIndex){


				//Clear stuff
				drive->isCmdAvailable = TRUE;
				memset(drive->cmd, 0, 3);
				memset(drive->args, 0, 15);

				//Update the drive with the values received from the command.
				for(i = (crIndex +1); i<commaIndex;i++){
					arg[i-crIndex -1] = drive->rxBuffer[i];
					drive->args[i-crIndex -1] = drive->rxBuffer[i];
				}

				//Check both Initials are Uppercase Alpahabet
				if((drive->rxBuffer[0] > 65 ||drive->rxBuffer[0]  < 90) &&
					(drive->rxBuffer[1] > 65 ||drive->rxBuffer[1]  < 90)){

					drive->cmd[0] = drive->rxBuffer[0];
					drive->cmd[1] = drive->rxBuffer[1];
					drive->cmd[2] = 0;
				}

				value = atof(drive->args);
				commandStartIndex = 1;

				/*Dealing with rxbuffer is over, modify it to accomodate better logging*/
				/*Logging readability enhancement*/
				drive->rxBuffer[drive->rxBufPtr] = '\r';
				drive->rxBuffer[(drive->rxBufPtr) + 1] = '\n';
				drive->rxBuffer[crIndex] = 32;

			}

		}

	}

}

void ProcessUserBuffer(UserCommandTypeDef *cmd){

	uint8_t length = 0;
	uint16_t tmp = 0;

	/*If CR is received, User command is avaialble*/
	if(userCmd.isCmdAvailable == TRUE){

		/*If Solo CR is received, it means to toggle logging*/
		if(cmd->rxBuffer[0] == '\r'){

//			if(isLogging == TRUE) {
//				isLogging = FALSE;
//			}else{
//				isLogging = TRUE;
//			}
			if(isPolling == TRUE) {
				isPolling = FALSE;
			}else{
				isPolling = TRUE;
			}
			ClearUserBuffer(cmd);
			writeStringPC(">Polling mode changed\r\n>\0");
			return;
		}

		/*Process the user command*/
		cmd->isCmdAvailable = TRUE;
		memset(cmd->Command,0,16);
		if(cmd->rxBuffer[0] == 'M' && cmd->rxBuffer[1] == ','){

			//Enque the command to sent to Uart1
			cmd->Scope = MASTER;
			memcpy (cmd->Command, cmd->rxBuffer + 2, (cmd->rxBufPtr)-2);
			Enqueue(&q1, cmd->Command);
			if(cmd->Command[0]=='B'&&cmd->Command[1]=='G'){
				isPolling = TRUE;
				writeStringPC(">Begin Motion\r\n>\0");
			}else{
				writeStringPC(">Sending to Master Drive\r\n>\0");
			}
			__NOP();

		}else if(cmd->rxBuffer[0] == 'S' && cmd->rxBuffer[1] == ','){

			//Enque the command to sent to Uart2
			cmd->Scope = SLAVE;
			memcpy (cmd->Command, cmd->rxBuffer + 2, (cmd->rxBufPtr)-2);
			Enqueue(&q2, cmd->Command);
			writeStringPC(">Sending to Slave Drive\r\n>\0");
			__NOP();

		}else if(cmd->rxBuffer[0] == 'D' && cmd->rxBuffer[1] == ','){

			//Enque the command to sent to Uart1/Uart2
			cmd->Scope = DUAL;
			memcpy (cmd->Command, cmd->rxBuffer + 2, (cmd->rxBufPtr)-2);
			Enqueue(&q1, cmd->Command);
			Enqueue(&q2, cmd->Command);
			writeStringPC(">Sending to both Drives\r\n>\0");
			__NOP();

		}else{

			//remove CR. copy and compare the result
			memcpy (cmd->Command, cmd->rxBuffer, (cmd->rxBufPtr) - 1);
			cmd->Scope = LOCAL;
			cmd->isCmdAvailable = TRUE;


			__NOP();

			/*String comparison of the command with allowed values*/
			if(strcmp(SYNC_CMD,cmd->Command) == 0){
				isSyncing = TRUE;
				writeStringPC(">Sync mode On\r\n>\0");
			}else if(strcmp(NOSYNC_CMD,cmd->Command) == 0){
				isSyncing = FALSE;
				writeStringPC(">Sync mode off\r\n>\0");
			}else if(strcmp(POLL_CMD,cmd->Command) == 0){
				isPolling = TRUE;
				writeStringPC(">Polling On\r\n>\0");
			}else if(strcmp(NOPOLL_CMD,cmd->Command) == 0){
				isPolling = FALSE;
				writeStringPC(">Polling Off\r\n>\0");
			}else if(strcmp(GRAPH_ON_CMD,cmd->Command) == 0){
				isGraphing = TRUE;
				writeStringPC(">Graph mode On\r\n>\0");
			}else if(strcmp(GRAPH_OFF_CMD,cmd->Command) == 0){
				isGraphing = FALSE;
				writeStringPC(">Graph mode Off\r\n>\0");
			}else if(strcmp(POS_OFF_CMD,cmd->Command) == 0){
				isPositionCheck = FALSE;
				writeStringPC(">Position query mode On\r\n>\0");
			}else if(strcmp(POS_ON_CMD,cmd->Command) == 0){
				isPositionCheck = TRUE;
				writeStringPC(">Position query mode Off\r\n>\0");
			}else if(strcmp(CONNECT_CMD,cmd->Command) == 0){
				isStart = TRUE;
			}else if(strcmp(BEGIN_CMD,cmd->Command) == 0){
				writeStringPC(">Sending BG to start motion\r\n>\0");
			}else {

				if(cmd->Command[5] == '='){
					length = strlen((char *)cmd->Command);
					if(length >= 6){
						memcpy (cmd->strValue, (cmd->Command) + 6, (length - 6));

						tmp = atoi(cmd->strValue);
						if(tmp >= 1 && tmp <= 1000){
							interCmdDelay = tmp;
							isDelayUpdate = TRUE;
							setDelay(interCmdDelay);
							writeStringPC(">Delay set\r\n>\0");
							//htim2.Init.Prescaler = interCmdDelay;
						}
					}else{
						ClearUserBuffer(cmd);
					}
				}
			}

			ClearUserBuffer(cmd);
		}
	}


}

void DecodeCommand(DriveTypeDef *drive){



	//memset(drive->rxBuffer, 0, 30);
	if(drive->cmd[0] == 'I' && drive->cmd[1] == 'Q' ){
		/*Update the master IQ and update teh buffer*/
		if(value < 15.0 && value > -15.0){

			drive->currentReal = value;
			drive->currentRecords[drive->currentRecordIndex % samples] = value;
			drive->currentRecordIndex++;

			/*Will limit the range of index below the array length as a safety measurement*/
			if(drive->currentRecordIndex >= 99){
				drive->currentRecordIndex = 0;
			}
		}

	}

	if(drive->cmd[0] == 'M' && drive->cmd[1] == 'O' ){
		drive->motorOn = value;
	}

	if(drive->cmd[0] == 'P' && drive->cmd[1] == 'X' ){
		drive->position = value;
	}

	if(drive->cmd[0] == 'P' && drive->cmd[1] == 'R' ){
		drive->targetPosition = value;
	}

	if(drive->cmd[0] == 'A' && drive->cmd[1] == 'C' ){
		drive->acceleration = value;
	}

	if(drive->cmd[0] == 'D' && drive->cmd[1] == 'C' ){
		drive->deceleration = value;
	}

	if(drive->cmd[0] == 'S' && drive->cmd[1] == 'P' ){
		drive->speed = value;
	}

	if(drive->cmd[0] == 'S' && drive->cmd[1] == 'R' ){
			drive->status = (uint32_t) value;
			drive->Status.status = (uint32_t) value;
			__NOP();
	}
	if(drive->cmd[0] == 'U' && drive->cmd[1] == 'M' ){
			drive->driveMode = value;
	}

	drive->isCmdAvailable = FALSE;
}

void substr(char *s, int a, int b, char *t)
{
    strncpy(t, s+a, b);
}

void ClearReceiveBuffer(DriveTypeDef *drive){ /*Clear Buffer a char*/
	drive->rxBufPtr = 0;
	memset(drive->rxBuffer, 0, 30);

}
void ClearUserBuffer(UserCommandTypeDef *cmd){
	cmd->rxBufPtr = 0;
	cmd->isCmdAvailable = FALSE;
	 memset(cmd->rxBuffer, 0, 12);
	 //Need to clear the rest
}

void setDelay(uint16_t delay){

	if(isDelayUpdate == TRUE){

		uint16_t tmp = delay * (8333/200);
			if(tmp < 65535){
				htim2.Init.Period = tmp;
				htim2.Instance->ARR = tmp;
			}else{
				htim2.Init.Period = 40000;
				htim2.Instance->ARR = 40000;
			}

			isDelayUpdate = FALSE;
	}


}

uint8_t IndexOf(uint8_t * source, uint8_t * test){ /*Helper for IndexOf a char*/

	uint8_t sLen = strlen((char *)source);
	uint8_t tLen = strlen((char *)test);

	uint8_t i;
	if(sLen > 25) sLen = 25;
	if(sLen > 2 && tLen > 1){

		sLen = sLen -1; //loop will stop n-1
		for(i = 0; i<sLen-1;i++){

			if(source[i] == test[0]){

				if(source[i+1] == test[1]){

					return i;
				}
			}
		}
	}else if(sLen > 1 && tLen == 1){

		for(i = 0; i<sLen;i++){

			if(source[i] == test[0]){

				return i;
			}
		}
	}
	return 0;
}

void CreateCurrentCommand(DriveTypeDef *drive){

	uint8_t tc[10];
	uint8_t len = 0;

	/*set all 0s*/
    memset(tc,0,10);

    /*Run the algorithm and get update the sync current*/
    drive->syncCurrent = getMeanCurrent(drive);

    /*This is not the function from stdlib*/
	ftoa(drive->syncCurrent, tc,3);
	len = strlen((char *)tc);

	/*Create the torque command*/
	strcpy(drive->syncCmd,"TC=\0");
	strcpy(drive->syncCmd+3,tc);
	strcpy(drive->syncCmd+len+3,"\r\0");
	__NOP();

}
void CalculateIndices(uint8_t * source){ /*Helper for IndexOf a char*/

	uint8_t sLen = strlen((char *)source);
	//uint8_t tLen = strlen((char *)test);

	uint8_t i;
	if(sLen > 25) sLen = 25;

	if(sLen > 1 ){

		for(i = 0; i<sLen;i++){

			if(source[i] == ';'){

				commaIndex = i;
				commaNumber++;
			}
			if(source[i] == '\r'){

				crIndex = i;
			}
			if(source[i] == '?'){

				junkIndex = i;
			}
			if(source[i] == '['){

				bracketIndex = i;
			}
			if(source[i] == '='){

				equalsIndex = i;
			}
		}
	}

}
uint8_t NumberOf(uint8_t * source, uint8_t * test){ /*Helper for number of occurrences of a char*/

	uint8_t sLen = strlen((char *)source);
	uint8_t count = 0;
	uint8_t i;

	if(sLen > 25) sLen = 25;

	if(sLen > 1 ){

		for(i = 0; i<sLen;i++){

			if(source[i] == test[0]){

				count++;
			}
		}
	}
	return count;
}


static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};

char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}





