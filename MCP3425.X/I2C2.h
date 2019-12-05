/******************************************************* Include ****************************************************/

/********************************************************************************************************************/

/******************************************************* Define *****************************************************/
// <editor-fold defaultstate="collapsed" desc="DEFINE">
#define ACK                                         0
#define NACK                                        1

#define I2C2_RETRIES_NUMBER                         100                             // Retry number of an I2C command

#define I2C2_RECEIVE_REG                            I2C2RCV                         // Defines the receive register used to receive data
#define I2C2_TRANSMIT_REG                           I2C2TRN                         // Defines the transmit register used to send data

#define I2C2_STOP_STATUS_BIT                        I2C2STATbits.P                  // I2C Stop status bit
#define I2C2_START_STATUS_BIT                       I2C2STATbits.S                  // I2C Start status bit  
#define I2C2_TRANSMIT_STATUS_BIT                    I2C2STATbits.TRSTAT             // I2C Transmit Status bit
#define I2C2_ACKNOWLEDGE_STATUS_BIT                 I2C2STATbits.ACKSTAT            // I2C ACK status bit
#define I2C2_BUS_COLLISION_STATUS_BIT               I2C2STATbits.BCL                // I2C Bus Collision Detect bit
#define I2C2_WRITE_COLLISION_STATUS_BIT             I2C2STATbits.IWCOL              // I2C Write collision status bit
#define I2C2_RECEIVER_OVERFLOW_STATUS_BIT           I2C2STATbits.I2COV              // I2C Receive Overflow Flag bit
#define I2C2_TRANSMIT_BUFFER_FULL_STATUS_BIT        I2C2STATbits.TBF                // I2C Transmit Buffer Full Status bit
#define I2C2_RECEIVER_BUFFER_FULL_STATUS_BIT        I2C2STATbits.RBF                // I2C Receive Buffer Full Status bit

#define I2C2_RECEIVE_ENABLE_BIT                     I2C2CONbits.RCEN                // I2C Receive enable control bit
#define I2C2_ACKNOWLEDGE_DATA_BIT                   I2C2CONbits.ACKDT               // I2C ACK data control bit
#define I2C2_ACKNOWLEDGE_ENABLE_BIT                 I2C2CONbits.ACKEN               // I2C ACK start control bit
#define I2C2_STOP_CONDITION_ENABLE_BIT              I2C2CONbits.PEN                 // I2C STOP control bit
#define I2C2_START_CONDITION_ENABLE_BIT             I2C2CONbits.SEN                 // I2C START control bit
#define I2C2_REPEAT_START_CONDITION_ENABLE_BIT      I2C2CONbits.RSEN                // I2C Repeated START control bit
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
void I2C2_Initialize(void);

bool I2C2Stop(void);
bool I2C2Start(void);
bool I2C2BusIsIdle(void);
bool I2C2RepeatStart(void);
bool I2C2StopTransfer(void);
bool I2C2StartTransfer(bool Restart);


unsigned char I2C2ReadOneByte(void);
bool I2C2TransmitOneByte(unsigned char Data);
bool I2C2AcknowledgeByte(bool ack_Type);
bool I2C2ByteWasAcknowledged(void);


unsigned char I2C2GetByte(void);
bool I2C2SendByte(unsigned char data);


bool I2C2ReceiverEnable(bool state);
bool I2C2TransmitterIsReady(void);
bool I2C2ReceivedDataIsAvailable(void);
bool I2C2TransmissionHasCompleted(void);
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