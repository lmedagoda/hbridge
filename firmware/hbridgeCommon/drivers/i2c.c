#include "inc/stm32f10x_i2c.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "i2c.h"
#include "printf.h"

enum I2CModes {
    I2C_READ,
    I2C_WRITE,
    I2C_WRITE_READ,
    I2C_FINISHED,
};

enum I2CState {
    START_WRITTEN,
    ADDRESS_WRITTEN,
    HANDLING_DATA,
    STOP_WRITTEN,
};

struct I2C_Data {
    enum I2CState state;
    uint8_t I2C_Buffer_Tx[4];
    uint8_t I2C_Buffer_Rx[4];
    uint8_t I2C_Tx_Idx;
    uint8_t I2C_Rx_Idx;
    uint8_t I2C_Tx_Size;
    uint8_t I2C_Rx_Size;
    uint8_t I2CMode;
    uint8_t I2CError;
    uint32_t I2CErrorReason;
    // counter to detect i2c interrupt storm (flooding)
    uint32_t i2cSafetyCounter;
    uint8_t curI2CAddr;
    int curI2CSpeed;
    FunctionalState curI2CIsRemapped;
};

void I2C_EV_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);
void I2C_ER_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data);

volatile struct I2C_Data I2C1_Data;
volatile struct I2C_Data I2C2_Data;

struct I2C_CommandQueue;
uint8_t handleI2CxErrors(I2C_TypeDef* I2Cx, volatile struct I2C_Data* I2Cx_Data);

struct I2C_Handle 
{
    volatile uint8_t hasResult;
    volatile uint8_t isSending;
    volatile uint8_t hadError;
    struct I2C_CommandResult pendingResult;
    struct I2C_CommandQueue *queue;
};

struct I2C_Command {
    //type of the command
    enum I2CModes type;
    //data to be transmitted
    uint8_t txData[4];
    uint8_t txSize;
    //number of bytes which should be received
    uint8_t rxSize;
    
    //adress of the target peer
    uint8_t address;
    
    //handle of the issuer of the command
    struct I2C_Handle *handle;
};


#define MAX_I2C_COMMANDS 10

struct I2C_CommandQueue
{
    struct I2C_Command queue[MAX_I2C_COMMANDS];
    volatile uint8_t writePointer;
    volatile uint8_t readPointer;
    struct I2C_Command *curCommand;
    volatile struct I2C_Data *I2C_Data;
    I2C_TypeDef* I2Cx;
};

struct I2C_CommandQueue I2C1_CommandQueue;
struct I2C_CommandQueue I2C2_CommandQueue;

static uint8_t handleCnt = 0;

#define MAX_I2C_HANDLES 5
struct I2C_Handle handles[MAX_I2C_HANDLES];

void I2C_print_state(struct I2C_Handle* handle)
{
    uint32_t curCmd = (uint32_t) handle->queue->curCommand;
    uint16_t hasResult = handle->hasResult;
    uint16_t isSending = handle->isSending;
    uint16_t rp = handle->queue->readPointer;
    uint16_t wp = handle->queue->writePointer;
    uint16_t i2cmode = handle->queue->I2C_Data->I2CMode;
    uint16_t i2cerror = handle->queue->I2C_Data->I2CError;
    uint32_t i2cerrorreason = handle->queue->I2C_Data->I2CErrorReason;
    uint32_t i2cSafetyCounter = handle->queue->I2C_Data->i2cSafetyCounter;
    uint32_t mslIsSet = I2C_GetFlagStatus(handle->queue->I2Cx, I2C_FLAG_MSL);
    
    printf("I2C dbg curCmd %lu, hasRes %hu, sending %hu, rp %hu, wp %hu, i2cmode %hu, i2cError %hu, i2cErRes %lu, i2cScnt %lu i2cMLS %lu\n", curCmd, hasResult, isSending, rp, wp, i2cmode, i2cerror, i2cerrorreason, i2cSafetyCounter, mslIsSet);
}

struct I2C_Handle *I2C_getHandle(I2C_TypeDef* I2Cx)
{
    if(handleCnt >= MAX_I2C_HANDLES)
    {
	printf("Error no free I2C handles any more\n");
	assert_failed((uint8_t *)__FILE__, __LINE__);
    }
    
    struct I2C_Handle *ret = handles + handleCnt;    
    handleCnt++;
    
    if(I2Cx == I2C1)
    {
	ret->queue = &I2C1_CommandQueue;
    }else{
	ret->queue = &I2C2_CommandQueue;
    }
    
    ret->hasResult = 0;
    ret->isSending = 0;
    ret->hadError = 0;

    return ret;
}

void I2C_triggerCommandQueue(struct I2C_CommandQueue *cmdQueue)
{
    //check for errors and handle them
    if(handleI2CxErrors(cmdQueue->I2Cx, cmdQueue->I2C_Data))
    {
	if(cmdQueue->curCommand)
	{
	    cmdQueue->curCommand->handle->hadError = 1;
	} else
	{
	    printf("Error, got error while no i2c command was issued\n");
	}
    }
    
    //check if bus is busy
    if((cmdQueue->I2C_Data->I2CMode != I2C_FINISHED) || (I2C_GetFlagStatus(cmdQueue->I2Cx, I2C_FLAG_MSL) == SET))
	return;
    
    //move finished command to result
    if(cmdQueue->curCommand)
    {
	struct I2C_CommandResult *res = &(cmdQueue->curCommand->handle->pendingResult);
	cmdQueue->curCommand->handle->hasResult = 1;
	
	res->I2CError = cmdQueue->curCommand->handle->hadError;
	res->I2CErrorReason = cmdQueue->I2C_Data->I2CErrorReason;
	res->rxSize = cmdQueue->I2C_Data->I2C_Rx_Size;
	int i;
	for(i=0; i < res->rxSize;i++)
	{
	    res->rxData[i] = cmdQueue->I2C_Data->I2C_Buffer_Rx[i];
	}
	
	cmdQueue->curCommand->handle->hadError = 0;
	cmdQueue->curCommand = 0;
    }
    
    //check if queue is empty
    if(cmdQueue->readPointer == cmdQueue->writePointer)
	return;

    if(I2C_GetFlagStatus(cmdQueue->I2Cx, I2C_FLAG_MSL)) {
	printf("BAD, I am still a master \n");
	return;
    }

    //issue next command
    struct I2C_Command *cmd = cmdQueue->queue + cmdQueue->readPointer;
  
    cmdQueue->I2C_Data->i2cSafetyCounter = 0;

    //setup interrupt handler state machine
    cmdQueue->I2C_Data->I2C_Tx_Idx = 0;
    cmdQueue->I2C_Data->I2C_Tx_Size = cmd->txSize;
    cmdQueue->I2C_Data->I2C_Rx_Idx = 0;
    cmdQueue->I2C_Data->I2C_Rx_Size = cmd->rxSize;
    cmdQueue->I2C_Data->curI2CAddr = cmd->address;
    cmdQueue->I2C_Data->I2CMode = cmd->type;
    cmdQueue->I2C_Data->state = START_WRITTEN;
  
    int i;  
    for(i = 0; i < cmd->txSize; i++) {
	cmdQueue->I2C_Data->I2C_Buffer_Tx[i] = cmd->txData[i];
    }

    cmdQueue->curCommand = cmd;
    
    //move read pointer forward
    cmdQueue->readPointer = (cmdQueue->readPointer + 1) % MAX_I2C_COMMANDS;
    
    //disable orphaned stop conditions
    I2C_GenerateSTOP(cmdQueue->I2Cx, DISABLE);

    // Enable Acknowledgement
    I2C_AcknowledgeConfig(cmdQueue->I2Cx, ENABLE);

    /* Send I2C1 START condition */
    I2C_GenerateSTART(cmdQueue->I2Cx, ENABLE);
}

uint8_t I2C_issueCommand(struct I2C_Handle *handle, enum I2CModes type, uint8_t *txdata, uint8_t txsize, uint8_t rxsize, uint8_t addr)
{
    //trigger queue to get 'other' commands processed
    I2C_triggerCommandQueue(handle->queue);
    
    //check if we are allready sending
    if(handle->isSending)
	return 1;
    
    struct I2C_CommandQueue *cmdQueue = handle->queue;
    uint8_t nextWritePointer = (cmdQueue->writePointer + 1) % MAX_I2C_COMMANDS;
    //check if command queue is full
    if(nextWritePointer == cmdQueue->readPointer)
	return 1;
    
    struct I2C_Command *cmd = cmdQueue->queue + cmdQueue->writePointer;
    
    cmd->type = type;
    cmd->rxSize = rxsize;
    cmd->txSize = txsize;
    cmd->address = addr;
    cmd->handle = handle;
    int i;
    for(i = 0; i < txsize; i++)
    {
	cmd->txData[i] = txdata[i];
    }

    handle->isSending = 1;
    
    cmdQueue->writePointer = nextWritePointer;
    
    I2C_triggerCommandQueue(handle->queue);
    
    return 0;
}

uint8_t I2C_writeReadBytes(struct I2C_Handle *handle, uint8_t *txdata, uint8_t txsize, uint8_t rxsize, uint8_t addr) 
{
    return I2C_issueCommand(handle, I2C_WRITE_READ, txdata, txsize, rxsize, addr);
}

uint8_t I2C_writeBytes(struct I2C_Handle *handle, uint8_t *data, uint8_t size, uint8_t addr) 
{
    return I2C_issueCommand(handle, I2C_WRITE, data, size, 0, addr);    
}    

uint8_t I2C_readBytes(struct I2C_Handle *handle, uint8_t size, uint8_t addr)
{
    return I2C_issueCommand(handle, I2C_READ, 0, 0, size, addr);
}

/**
 * Checks if the last issued I2C operation was finished.
 * If this is the case a pointer to an I2C_CommandResult 
 * is returned.
 * 
 * If the operation is still pending 0 is returned
 * */
struct I2C_CommandResult *I2C_getResult(struct I2C_Handle *handle)
{
    I2C_triggerCommandQueue(handle->queue);
    
    if(!handle->hasResult)
	return 0;
    
    //mark current operation as finished
    handle->isSending = 0;
    handle->hasResult = 0;
    return &(handle->pendingResult);
}

void I2C_writeBytesBlocking(struct I2C_Handle *handle, uint8_t *data, uint8_t size, uint8_t addr)
{
    uint8_t error = 0;
    error = I2C_writeBytes(handle, data, size, addr);
    struct I2C_CommandResult *res;
    
    while(1)
    {
	if(!handle->isSending && error)
	    error = I2C_writeBytes(handle, data, size, addr);
	    
	res = I2C_getResult(handle);
	if(res)
	{
	    if(!res->I2CError)
		break;
	    else 
		printf("e");
	}
    }
}

void resetI2C(struct I2C_Handle *handle)
{
    //store read and write pointer of queue, so that command don't get lost
    volatile uint8_t readP = handle->queue->readPointer;
    volatile uint8_t writeP = handle->queue->writePointer;
    
    struct I2C_Command *curCmd = handle->queue->curCommand;
    
    //resetup bus
    setupI2Cx(handle->queue->I2C_Data->curI2CAddr, handle->queue->I2C_Data->curI2CSpeed, handle->queue->I2Cx,  handle->queue->I2C_Data->curI2CIsRemapped);
    
    handle->queue->readPointer = readP;
    handle->queue->writePointer = writeP;
    
    if(curCmd)
    {
	//reissue command
	I2C_issueCommand(curCmd->handle, curCmd->type, curCmd->txData, curCmd->txSize, curCmd->rxSize, curCmd->address);
    }
}

void setupI2CxIntern(uint16_t address, int speed, I2C_TypeDef* I2Cx, FunctionalState remapped, uint8_t keepQueue) 
{
    volatile int wait;
    volatile struct I2C_Data *I2Cx_Data;
    uint8_t EV_IRQChannel;
    uint8_t ER_IRQChannel;
    uint16_t GPIOs;
    GPIO_TypeDef* GPIO_Block = GPIOB;

    struct I2C_CommandQueue *queue = 0;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    if(I2Cx == I2C1)
    {
	I2Cx_Data = &I2C1_Data;
	EV_IRQChannel = I2C1_EV_IRQn;
	ER_IRQChannel = I2C1_ER_IRQn;
	
	if(remapped)
	{    
	    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
	    GPIOs = GPIO_Pin_8 | GPIO_Pin_9;
	}
	else
	{    
	    GPIOs = GPIO_Pin_6 | GPIO_Pin_7;
	}
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    	
	queue = &I2C1_CommandQueue;
    }
    else
    {
	I2Cx_Data = &I2C2_Data;
	EV_IRQChannel = I2C2_EV_IRQn;
	ER_IRQChannel = I2C2_ER_IRQn;
	
	GPIOs = GPIO_Pin_10 | GPIO_Pin_11;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	
	queue = &I2C2_CommandQueue;
    }
    
    if(!keepQueue)
    {
	queue->I2C_Data = I2Cx_Data;
	queue->I2Cx = I2Cx;
	queue->curCommand = 0;
	queue->readPointer = 0;
	queue->writePointer = 0;
    }
    
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* Configure and enable I2C2 interrupt ------------------------------------*/
    NVIC_InitStructure.NVIC_IRQChannel = EV_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = ER_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    
    //be shure bus is not beeing resetet
    I2C_SoftwareResetCmd(I2Cx, DISABLE);

    I2C_DeInit(I2Cx);

    //configure as in floating, before (RE) initalizing.
    //this works around the buggy i2c controller that 
    //does not release the line in case of error
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIOs;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIO_Block, &GPIO_InitStructure);  

    wait = 10000;
    while(wait--)
	;

    //Configure I2C2 Pins, SDA and SCL
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIOs;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIO_Block, &GPIO_InitStructure);  

    I2C_InitTypeDef  I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = address;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = speed;

    if(speed <= 100000)
    {
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; // for < 100 kHz
    } else
    {
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
    }
    I2C_Init(I2Cx, &I2C_InitStructure);

    //Enable I2C2
    I2C_Cmd(I2Cx, ENABLE);

    // Disable PEC for I2C2
    I2C_CalculatePEC(I2Cx, DISABLE);

    //setup mode correctly
    I2Cx_Data->I2CMode = I2C_FINISHED;
    I2Cx_Data->curI2CAddr = address;
    I2Cx_Data->curI2CSpeed = speed;
    I2Cx_Data->curI2CIsRemapped = remapped;
    I2Cx_Data->I2C_Tx_Idx = 0;
    I2Cx_Data->I2C_Rx_Idx = 0;
    I2Cx_Data->I2C_Tx_Size = 0;
    I2Cx_Data->I2C_Rx_Size = 0;
    I2Cx_Data->I2CError = 0;
    I2Cx_Data->I2CErrorReason = 0;
    I2Cx_Data->i2cSafetyCounter = 0;

    // Enable I2C1 and I2C2 event and buffer interrupt
    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE); 
}

void setupI2Cx(uint16_t address, int speed, I2C_TypeDef* I2Cx, FunctionalState remapped)
{
    setupI2CxIntern(address, speed, I2Cx, remapped, 0);
}

void setupI2C2(uint16_t address, int speed) {
    //i2c2 can not be remapped
    setupI2Cx(address, speed, I2C2, DISABLE);
}

void setupI2C1(uint16_t address, int speed, FunctionalState remapped) {
    setupI2Cx(address, speed, I2C1, remapped);
}

/**
 * This function checks weather an error occured during the
 * current I2C operation and tries to handle it. If an error
 * occured the the current operation is canceled and the 
 * error is stored in the CommandResult
 *
 * Known Bugs: In case of Bus error the recovery does not allways work
 *
 * Return 1 if an error occured 0 otherwise
 */
uint8_t handleI2CxErrors(I2C_TypeDef* I2Cx, volatile struct I2C_Data* I2Cx_Data)
{
    uint8_t hasError = 0;

    if(I2Cx_Data->I2CError == 2) {
	printf("Bus Amok\n");
    }
    //handle I2C errors
    if(I2Cx_Data->I2CError) {
	switch(I2Cx_Data->I2CErrorReason) {
	case I2C_FLAG_ARLO:
	    printf("ARLO");
	    break;
	case I2C_FLAG_BERR:
	    printf("BERR");
	    I2C_SoftwareResetCmd(I2Cx, DISABLE);
	    setupI2CxIntern(I2Cx_Data->curI2CAddr, I2Cx_Data->curI2CSpeed, I2Cx, I2Cx_Data->curI2CIsRemapped, 1);
	    break;
	case I2C_FLAG_OVR:
	    printf("OVR");
	    break;
	case I2C_FLAG_AF:
	    if((I2C_GetFlagStatus(I2Cx, I2C_FLAG_MSL) == SET))
		I2C_GenerateSTOP(I2Cx, ENABLE);
	    printf("AF");
	    break;	
	default:
	    I2C_SoftwareResetCmd(I2Cx, ENABLE);
	    int wait = 1000;
	    while(wait--)
		;
	    I2C_SoftwareResetCmd(I2Cx, DISABLE);
	    setupI2CxIntern(I2Cx_Data->curI2CAddr, I2Cx_Data->curI2CSpeed, I2Cx, I2Cx_Data->curI2CIsRemapped, 1);
	    printf("Unknown %lu", I2Cx_Data->I2CErrorReason);
	}
	
	I2Cx_Data->I2CMode = I2C_FINISHED;
	
	I2Cx_Data->I2CError = 0;
	hasError = 1;    
    }

    return hasError;
}

// define I2C register
#define I2C_SMBALERT (1<<15)
#define I2C_TIMEOUT (1<<14)
#define I2C_PECERR (1<<12)
#define I2C_OVR (1<<11)
#define I2C_AF (1<<10)
#define I2C_ARLO (1<<9)
#define I2C_BERR (1<<8)
#define I2C_TxE (1<<7)
#define I2C_RxNE (1<<6)
#define I2C_STOPF (1<<4)
#define I2C_ADD10 (1<<3)
#define I2C_BTF (1<<2)
#define I2C_ADDR (1<<1)
#define I2C_SB (1<<0)
#define I2C_DUALF (1<<7)
#define I2C_SMBHOST (1<<6)
#define I2C_SMBDEFAULT (1<<5)
#define I2C_GENCALL (1<<4)
#define I2C_TRA (1<<2)
#define I2C_BUSY (1<<1)
#define I2C_MSL (1<<0)

/*******************************************************************************
* Function Name  : I2C2_ER_IRQHandler
* Description    : This function handles I2C2 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_ER_IRQHandler(void)
{
  I2C_ER_IRQHandler(I2C2, &I2C2_Data);
}

struct i2cDebug {
    uint16_t sr1;
    uint16_t sr2;
    uint8_t state;
    uint8_t mode;
    uint8_t idx;
    uint8_t size;
    uint16_t cnt;
    uint8_t error;
};

#ifdef I2C_DEBUG
#define dbgBufferSize 20
volatile uint16_t dbgWrite = 0;
volatile uint16_t dbgRead = 0;
volatile struct i2cDebug dbgBuffer[dbgBufferSize];

void printfI2CDbg() {
    while(dbgWrite != dbgRead) {
	volatile struct i2cDebug *dbg = &(dbgBuffer[dbgRead]);
	dbgRead = (dbgRead +1 ) % dbgBufferSize;
	printf("cnt %hu, SR1 %hu, SR2 %hu, S %hu, M %hu, idx %hu, size %hu, error %hu\n", dbg->cnt, dbg->sr1, dbg->sr2, dbg->state, dbg->mode, dbg->idx, dbg->size, dbg->error);
    }
}
#endif

void I2C_EV_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data)
{
    uint16_t sr1 = I2Cx->SR1;
    uint16_t sr2 = I2Cx->SR2;
    static uint16_t cnt = 0;

#ifdef I2C_DEBUG
    //flush debug buffer
    if(I2Cx_Data->i2cSafetyCounter == 0)
	dbgWrite = dbgRead;
#endif

    ++(I2Cx_Data->i2cSafetyCounter);

    // interrupt storm detected, disable irq
    if(I2Cx_Data->i2cSafetyCounter > 120) {
	I2Cx_Data->I2CError = 2;
	I2Cx_Data->I2CErrorReason = I2C_FLAG_BERR;
	I2C_SoftwareResetCmd(I2Cx, ENABLE);
	I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    }
    
#ifdef I2C_DEBUG
    volatile uint8_t nextRxWritePointer = (dbgWrite + 1) % dbgBufferSize;

    if(nextRxWritePointer != dbgRead) {
	volatile struct i2cDebug *dbg = &(dbgBuffer[dbgWrite]);
	dbg->sr1 = sr1;
	dbg->sr2 = sr2;
	dbg->state = I2Cx_Data->state;
	dbg->mode = I2Cx_Data->I2CMode;
	dbg->idx = I2Cx_Data->I2C_Rx_Idx;
	dbg->size = I2Cx_Data->I2C_Rx_Size;
	dbg->cnt = cnt;
	dbg->error = I2Cx_Data->I2CError;
	dbgWrite = nextRxWritePointer;
    }
#endif
    
    cnt++;
    
    switch(I2Cx_Data->state) {
	case START_WRITTEN:
	    if(sr1& I2C_SB) {
		//we are master
		//write adress of slave
		if(I2Cx_Data->I2CMode == I2C_READ) {
		    // Send I2C2 slave Address for write
		    I2C_Send7bitAddress(I2Cx, I2Cx_Data->curI2CAddr, I2C_Direction_Receiver);
		} else {//I2Cx_Data->I2CMode == I2C_WRITE || I2Cx_Data->I2CMode == I2C_WRITE_READ
		    if(I2Cx_Data->I2CMode != I2C_FINISHED) { 
			// Send I2C2 slave Address for write
			I2C_Send7bitAddress(I2Cx, I2Cx_Data->curI2CAddr, I2C_Direction_Transmitter);
		    }
		}
		//enable buf it, so we get an interrupt if TxE or RxNE
		I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);

		I2Cx_Data->state = ADDRESS_WRITTEN;
	    }
	    break;
	case ADDRESS_WRITTEN:
	    //advance state for debug purposes
	    if(sr1 & I2C_ADDR) {
		I2Cx_Data->state = HANDLING_DATA;		
	    }
	    //no break here by purpose
	case HANDLING_DATA:
	    //this basicly means, we are not in transmission
	    //mode, but the chip is waiting for data to transmit
	    //as we had this error, we are going to handle this case
	    if((sr1 & I2C_TxE) && !(sr2 & I2C_TRA)) {
		I2C_SendData(I2Cx, 0);		
	    }
	    
	    if(sr2 & I2C_TRA) {
		//Transmitter
		if((sr1 & I2C_TxE) || (sr1 & I2C_BTF)) {
		    if(I2Cx_Data->I2C_Tx_Idx < I2Cx_Data->I2C_Tx_Size) {
			/* Transmit buffer data */
			I2C_SendData(I2Cx, I2Cx_Data->I2C_Buffer_Tx[I2Cx_Data->I2C_Tx_Idx]);
			I2Cx_Data->I2C_Tx_Idx++;
		    } else {
			if(I2Cx_Data->I2CMode == I2C_WRITE) {
                            /* Disable I2C1 BUF interrupts */
                            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
                            // Send STOP Condition
                            I2C_GenerateSTOP(I2Cx, ENABLE);

                            //write some data to DR, so that BTF is never set
                            //note this data is NOT sended
                            I2C_SendData(I2Cx, 0);

                            I2Cx_Data->I2CMode = I2C_FINISHED;
			} else {
			    if(I2Cx_Data->I2CMode == I2C_WRITE_READ) {
				//Write phase finished, switch to read
				I2Cx_Data->I2CMode = I2C_READ;

				/* Disable I2C1 BUF interrupts */
				I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
				
				/* Send I2C1 START condition */
				I2C_GenerateSTART(I2Cx, ENABLE);	  

				//set state back to start written
				I2Cx_Data->state = START_WRITTEN;
				
				//write some data to DR, so that BTF is never set
				//note this data is NOT sended
				I2C_SendData(I2Cx, 0);
			    } else {
				//handle TxE or BTF in weird state
                                //note we should never end up here !
				I2C_SendData(I2Cx, 0);
			    }
			}
		    }
		}
	    } else {
		//receiver
		//for one byte receiving ack needs to be disabled just after writing the address
		if(I2Cx_Data->I2C_Rx_Size == 1 && (sr1 & I2C_ADDR)) {
		    // Disable Acknowledgement
		    I2C_AcknowledgeConfig(I2Cx, DISABLE);

		    // Send STOP Condition
		    I2C_GenerateSTOP(I2Cx, ENABLE);
		}

		//read data if rx is not empty
		if((sr1 & I2C_RxNE) || (sr1 & I2C_BTF)) {
		    if(I2Cx_Data->I2C_Rx_Idx < I2Cx_Data->I2C_Rx_Size) {
			I2Cx_Data->I2C_Buffer_Rx[I2Cx_Data->I2C_Rx_Idx] = I2C_ReceiveData(I2Cx);
			I2Cx_Data->I2C_Rx_Idx++;
			//set mode to finish if we got all bytes
			if(I2Cx_Data->I2C_Rx_Idx >= I2Cx_Data->I2C_Rx_Size)
			    I2Cx_Data->I2CMode = I2C_FINISHED;	
		    } else {
			//clear RxNE
			I2C_ReceiveData(I2Cx);
		    }
		    
		    //test if next byte is last one and disable ack
		    if(I2Cx_Data->I2C_Rx_Idx + 1 == I2Cx_Data->I2C_Rx_Size) {
			// Disable Acknowledgement
			I2C_AcknowledgeConfig(I2Cx, DISABLE);
			// Send STOP Condition
			I2C_GenerateSTOP(I2Cx, ENABLE);
		    }
		}
	    }
	    break;
	case STOP_WRITTEN:
	    break;
    }    
}


void I2C_ER_IRQHandler(I2C_TypeDef* I2Cx, volatile struct I2C_Data *I2Cx_Data) {
  I2Cx_Data->I2CError = 1;
  uint16_t sr2 = I2Cx->SR2;
  uint16_t cr1 = I2Cx->CR1;
  
  //we got an error, be sure we don't stop
  //later by an orphan start condition
  I2C_GenerateSTART(I2Cx, DISABLE);
  
  (I2Cx_Data->i2cSafetyCounter)++;

  if(I2Cx_Data->i2cSafetyCounter > 120) {
      I2Cx_Data->I2CError = 2;
      I2Cx_Data->I2CErrorReason = I2C_FLAG_BERR;
      I2C_SoftwareResetCmd(I2Cx, ENABLE);
      I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
  }
  
#ifdef I2C_DEBUG
  volatile uint8_t nextRxWritePointer = (dbgWrite + 1) % dbgBufferSize;
  //if(nextRxWritePointer != dbgRead) {
	volatile struct i2cDebug *dbg = &(dbgBuffer[dbgWrite]);
	dbg->sr1 = cr1;
	dbg->sr2 = sr2;
	dbg->state = I2Cx_Data->state;
	dbg->mode = I2Cx_Data->I2CMode;
	dbg->idx = I2Cx_Data->I2C_Rx_Idx;
	dbg->size = I2Cx_Data->I2C_Rx_Size;
	dbg->cnt = 666;
	dbg->error = I2Cx_Data->I2CError;
	dbgWrite = nextRxWritePointer;
    //}
#endif
	
  //Acknowledge failure
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_AF)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_AF;
  
    //Note, master only error handling
    if(!((cr1 & (1<<9)) || (cr1 & (1<<8))) && (sr2 & I2C_BUSY))
	I2C_GenerateSTOP(I2Cx, ENABLE);

    /* Clear AF flag */
    I2C_ClearFlag(I2Cx, I2C_FLAG_AF);
  }

  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ARLO)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_ARLO;
    // Send STOP Condition
    if(!((cr1 & (1<<9)) || (cr1 & (1<<8))) && (sr2 & I2C_BUSY))
	I2C_GenerateSTOP(I2Cx, ENABLE);

    I2C_ClearFlag(I2Cx, I2C_FLAG_ARLO);
  }

  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_TIMEOUT)) {
      //I2Cx_Data->I2CErrorReason = I2C_FLAG_TIMEOUT;
    // Send STOP Condition
    if(!((cr1 & (1<<9)) || (cr1 & (1<<8))) && (sr2 & I2C_BUSY))
	I2C_GenerateSTOP(I2Cx, ENABLE);

    I2C_ClearFlag(I2Cx, I2C_FLAG_TIMEOUT);
  }
  
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BERR)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_BERR;
    I2C_SoftwareResetCmd(I2Cx, ENABLE);

    I2C_ClearFlag(I2Cx, I2C_FLAG_BERR);
  }
  
  if(I2C_GetFlagStatus(I2Cx, I2C_FLAG_OVR)) {
    I2Cx_Data->I2CErrorReason = I2C_FLAG_OVR;
    // Send STOP Condition
    I2C_GenerateSTOP(I2Cx, ENABLE);

    I2C_ClearFlag(I2Cx, I2C_FLAG_OVR);
  }
}

/*******************************************************************************
* Function Name  : I2C1_EV_IRQHandler
* Description    : This function handles I2C1 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void I2C1_EV_IRQHandler(void)
{
  I2C_EV_IRQHandler(I2C1, &I2C1_Data);
}

/*******************************************************************************
* Function Name  : I2C1_ER_IRQHandler
* Description    : This function handles I2C1 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_ER_IRQHandler(void)
{
  I2C_ER_IRQHandler(I2C1, &I2C1_Data);
}

/*******************************************************************************
* Function Name  : I2C2_EV_IRQHandler
* Description    : This function handles I2C2 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_EV_IRQHandler(void)
{ 
  I2C_EV_IRQHandler(I2C2, &I2C2_Data);
}

