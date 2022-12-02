

#ifndef INC_COMMAND_DECODER_H_
#define INC_COMMAND_DECODER_H_

#include "main.h"
#include "string.h"
#include <stdlib.h>
#include <errno.h>
//#include "stdbool.h"

typedef enum { FALSE = 0, TRUE = !FALSE } bool;
enum _Scope {LOCAL = 1, MASTER = 2,SLAVE = 3,DUAL = 4};
typedef union __Drive_Status
{

	/*Bitwise values*/
	struct{
		uint8_t				driveRead :1;		/*(0)	0-OK, 1-Problem*/
		uint8_t				servoStatus :3;		/*(1-3) Servo status 7 states*/
		uint8_t				motorOn :1;			/*(4) 1-On, 0-Off*/
		uint8_t				refMode :1;			/*(5) Reference Mode*/
		uint8_t				motorFailure :1;	/*(6) Motor failure latched*/
		uint8_t				unitMode :3 ;		/*(7-9)Possible 5 modes encoded to 3 bits*/
		uint8_t				gainScheduling :1;	/*(10) Gain scheduling On-1*/
		uint8_t				homingMode :1;		/*(11) Either main or auxilary homing used*/
		uint8_t				programRunning :1;	/*(12)Compiled program is running in side drive*/
		uint8_t				currentLimit :1;	/*(13) 1-ON, 0-OFF*/
		uint8_t				motionStatus :2;	/*(14-15) MS output*/
		uint8_t				recorderStatus :2;	/*(16-17) 0-inactive, 1-wait trigger, 2- finished, 3-ongoing*/
		uint8_t				UNUSED :5;
		uint8_t				hallSensors:3;		/*(24-26) hall sensors ABC*/
		uint8_t				cpuStatus: 1;		/*(27) 0-OK, 1-Exception*/
		uint8_t				limitStop :1;		/*(28) Stopped by any limit*/
		uint8_t				errorInProgram :1;	/*(29) Error in user program*/
		uint8_t				UNUSED2 :1;

	};

	/*Full value */
	uint32_t                status;        /*!< UART registers base address        */


} Drive_StatusTypeDef;

typedef struct __Command
{
	uint8_t                  	*Command;        /*Command String      */
	uint8_t 					*strValue;			 /*Values*/
	double 						value;

} CommandTypeDef;

typedef struct __Drive
{
	Drive_StatusTypeDef         Status;        /* UART registers base address        */
	UART_HandleTypeDef 			comPort;
	CommandTypeDef				curCmd;
	//CommandTypeDef				nextCmd;
	uint32_t					status;
	double 						currentReal;
	double 						currentImaginary;
	double 						syncCurrent;
    uint8_t						syncCmd[15];
	double 						speed;
	double 						acceleration;
	double 						deceleration;
	double						currentRecords[100];
	uint8_t						currentRecordIndex;
	bool						isConnected;
    bool 						isHome;
    uint16_t 					position;
    uint16_t 					targetPosition;
    uint8_t 					driveMode;
    uint16_t 					maxSpeed;
    uint8_t 					*serialNumber;
    uint16_t 					baudrate;
    bool 						isMaster;
    bool 						isForwardMotion;
    uint16_t 					syncDelay;
    uint8_t 					rxBuffer[30];
    uint8_t						rxBufPtr;
    uint8_t						cmd[3];
    uint8_t						nextCmd[15];
    uint8_t						args[15];
    uint8_t						zero;
    uint8_t						motorOn;
    bool						isCmdAvailable;
    bool						inCorrectMode;
    bool						inSync;

} DriveTypeDef;



typedef struct __UserCommand
{
	uint8_t                  	Scope;        	/*Command Scope, L-Local, M-Master, S-Slave, D-Dual      */
	uint8_t                  	Command[16];       /*Command String      */
	uint8_t 					strValue[11];		/*Values*/
	double 						value;
    bool						isCmdAvailable;
    uint8_t 					rxBuffer[12];
    uint8_t						rxBufPtr;

} UserCommandTypeDef;

typedef struct __DecodeIndices
{
	uint8_t                  	commaIndex;        	/*Command Scope, L-Local, M-Master, S-Slave, D-Dual      */
	uint8_t                  	commaNumber;       /*Command String      */
	uint8_t 					equalsIndex;		/*Values*/
	uint8_t 					bracketIndex;		/*Values*/
	uint8_t 					junkIndex;		/*Values*/
	uint8_t 					crIndex;		/*Values*/
	uint8_t 					commandStartIndex;		/*Values*/
	uint8_t 					argumentStartIndex;		/*Values*/

} DecodeIndicesTypeDef;
//DriveTypeDef drive1;
//DriveTypeDef drive2;

void InitDriveData(DriveTypeDef *drive);
void ProcessReceiveBuffer(DriveTypeDef *drive, uint8_t singleData);
void ProcessUserBuffer(UserCommandTypeDef *cmd);
void ClearReceiveBuffer(DriveTypeDef *drive);
void CheckReceiveBuffer(DriveTypeDef *drive);
void ClearUserBuffer(UserCommandTypeDef *cmd);
uint8_t IndexOf(uint8_t * source, uint8_t * test);
uint8_t NumberOf(uint8_t * source, uint8_t * test);
void substr(char *s, int a, int b, char *t);
void CalculateIndices(uint8_t * source);
void DecodeCommand(DriveTypeDef *drive);
void setDelay(uint16_t delay);
void reverse(char* str, int len);
int intToStr(int x, char str[], int d);
char * ftoa(double f, char * buf, int precision);
void CreateCurrentCommand(DriveTypeDef *drive);
void ProcessMO();
void ProcessSR();
void ProcessAC();
void ProcessDC();
void ProcessSP();
void ProcessIQ();
void ProcessTC();
void ProcessUM();
void ProcessPX();
void ProcessPR();
void ProcessLD();
void ProcessVE();

#define IQ_INDEX 0
#define SR_INDEX 1
#define PX_INDEX 2
#define MAX_INIT 11

#define QUEUE_EMPTY 5;
#define QUEUE_FULL 10;

extern bool isPolling;
extern bool isLogging;
extern bool isGraphing;
extern bool isSyncing;
extern bool isPositionCheck;
extern bool isMasterInit;
extern bool isSlaveInit;
extern bool isDelayUpdate;
extern bool fetchNextMaster;
extern bool fetchNextSlave;


extern uint8_t PollingList[4][10];
extern uint8_t MasterInitList[12][15];
extern uint8_t SlaveInitList[12][15];
extern uint8_t MasterQueue[5][20];
extern uint8_t SlaveQueue[5][20];
extern uint8_t pollingIndex;

void CreatePollingList();
void ClearMasterQueue();
void ClearSlaveQueue();
void AddSlaveQueue();
void AddMasterQueue();
void RemoveMasterQueue();
void RemoveSlaveQueue();
void getNextMasterCommand(uint8_t *cmd);
void getNextSlaveCommand(uint8_t *cmd);

extern uint8_t masterInitCmdIndex;
extern uint8_t slaveInitCmdIndex;


#endif
