/******************************************************* Include ****************************************************/

/********************************************************************************************************************/

/******************************************************* Define *****************************************************/
// <editor-fold defaultstate="collapsed" desc="DEFINE">
#define ACK                                         0
#define NACK                                        1

#define I2C1_RETRIES_NUMBER                         100                             // Retry number of an I2C command

#define I2C1_RECEIVE_REG                            I2C1RCV                         // Defines the receive register used to receive data
#define I2C1_TRANSMIT_REG                           I2C1TRN                         // Defines the transmit register used to send data

#define I2C1_STOP_STATUS_BIT                        I2C1STATbits.P                  // I2C Stop status bit
#define I2C1_START_STATUS_BIT                       I2C1STATbits.S                  // I2C Start status bit  
#define I2C1_TRANSMIT_STATUS_BIT                    I2C1STATbits.TRSTAT             // I2C Transmit Status bit
#define I2C1_ACKNOWLEDGE_STATUS_BIT                 I2C1STATbits.ACKSTAT            // I2C ACK status bit
#define I2C1_BUS_COLLISION_STATUS_BIT               I2C1STATbits.BCL                // I2C Bus Collision Detect bit
#define I2C1_WRITE_COLLISION_STATUS_BIT             I2C1STATbits.IWCOL              // I2C Write collision status bit
#define I2C1_RECEIVER_OVERFLOW_STATUS_BIT           I2C1STATbits.I2COV              // I2C Receive Overflow Flag bit
#define I2C1_TRANSMIT_BUFFER_FULL_STATUS_BIT        I2C1STATbits.TBF                // I2C Transmit Buffer Full Status bit
#define I2C1_RECEIVER_BUFFER_FULL_STATUS_BIT        I2C1STATbits.RBF                // I2C Receive Buffer Full Status bit

#define I2C1_RECEIVE_ENABLE_BIT                     I2C1CONbits.RCEN                // I2C Receive enable control bit
#define I2C1_ACKNOWLEDGE_DATA_BIT                   I2C1CONbits.ACKDT               // I2C ACK data control bit
#define I2C1_ACKNOWLEDGE_ENABLE_BIT                 I2C1CONbits.ACKEN               // I2C ACK start control bit
#define I2C1_STOP_CONDITION_ENABLE_BIT              I2C1CONbits.PEN                 // I2C STOP control bit
#define I2C1_START_CONDITION_ENABLE_BIT             I2C1CONbits.SEN                 // I2C START control bit
#define I2C1_REPEAT_START_CONDITION_ENABLE_BIT      I2C1CONbits.RSEN                // I2C Repeated START control bit
// </editor-fold>
/********************************************************************************************************************/

/****************************************************** Constant ****************************************************/

/********************************************************************************************************************/

/**************************************************** Global Variable ***********************************************/

/********************************************************************************************************************/

/************************************************ Extern Global Variable ********************************************/

/********************************************************************************************************************/

/************************************************** Function Prototype **********************************************/
// <editor-fold defaultstate="collapsed" desc="FUNCTION PROTOTYPE">
void I2C1_Initialize(void);

bool I2C1Stop(void);
bool I2C1Start(void);
bool I2C1BusIsIdle(void);
bool I2C1RepeatStart(void);
bool I2C1StopTransfer(void);
bool I2C1StartTransfer(bool Restart);


unsigned char I2C1ReadOneByte(void);
bool I2C1TransmitOneByte(unsigned char Data);
bool I2C1AcknowledgeByte(bool ack_Type);
bool I2C1ByteWasAcknowledged(void);


unsigned char I2C1GetByte(void);
bool I2C1SendByte(unsigned char data);


bool I2C1ReceiverEnable(bool state);
bool I2C1TransmitterIsReady(void);
bool I2C1ReceivedDataIsAvailable(void);
bool I2C1TransmissionHasCompleted(void);
// </editor-fold>
/********************************************************************************************************************/
/********************************************************************************************************************/

/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/