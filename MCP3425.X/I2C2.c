/***************************************************** Include ****************************************************/
// <editor-fold defaultstate="collapsed" desc="INCLUDE">
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "I2C2.h"
#include "p24FJ128GC006.h"
// </editor-fold>
/******************************************************************************************************************/

/****************************************************** Define ****************************************************/

/******************************************************************************************************************/

/***************************************************** Constant ***************************************************/

/******************************************************************************************************************/

/*********************************************** Extern Global Variable *******************************************/

/******************************************************************************************************************/

/*************************************************** Global Variable **********************************************/

/******************************************************************************************************************/

/************************************************* Function Prototype *********************************************/

/******************************************************************************************************************/
/******************************************************************************************************************/

/***************************************************** Function ***************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
// <editor-fold defaultstate="collapsed" desc="FUNCTION">
// <editor-fold defaultstate="collapsed" desc="I2C2_Initialize">
void I2C2_Initialize(void)
{   
        I2C2CONbits.I2CEN = false;     
        
        IEC3bits.MI2C2IE = 0;                                                       // disable the master interrupt    
        IFS3bits.MI2C2IF = 0;                                                       // 0 = Interrupt request has not occurred
        IPC1bits.IC2IP = 3;                                                         // priority 3 
       
        
        I2C2_RECEIVE_REG = 0x0000;
        I2C2_TRANSMIT_REG = 0x0000;       
    
        /*
        * I2C2BRG: I2C2 BAUD RATE GENERATOR REGISTER
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | BIT 15   | BIT 14   | BIT 13   | BIT 12   | BIT 11   | BIT 10   | BIT 9    | BIT 8    | BIT 7    | BIT 6    | BIT 5    | BIT 4    | BIT 3    | BIT 2    | BIT 1    | BIT 0    |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | -        | -        | -        | -        | -        | -        | -        | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0    |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | -        | -        | -        | -        | -        | -        | -        |                                         I2C2BRG<8:0>                                             |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        BIT 15:     I2C2BRG<8:0> (Output Compare x Primary Comp. Register Value bits) 
        BIT 14:                                                                                                                                                                                            
        BIT 13:     p/ 400KHz
        BIT 12:     I2C2BRG = ((FCY/FSCL) - (FCY/10.000.000) - 1)                    
        BIT 11:     I2C2BRG = ((16MHz/400KHz) - (16MHz/10.000.000) - 1)
        BIT 10:     I2C2BRG = (40 - 1,6 - 1)   
        BIT 9:      I2C2BRG = 37,40   
        BIT 8:      I2C2BRG = 37  
        BIT 7:                   
        BIT 6:                                                                                 
        BIT 5:                             
        BIT 4:                                                                  
        BIT 3:                              
        BIT 2:                  0x0025;     //400KHz
        BIT 1:                  0x004D;     //200KHz  
        BIT 0:                  0x009D;     //100KHz          
        */
        I2C2BRG = 0x009D;   

        /*
        * I2C2STAT: I2C2 STATUS REGISTER
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | BIT 15   | BIT 14   | BIT 13   | BIT 12   | BIT 11   | BIT 10   | BIT 9    | BIT 8    | BIT 7    | BIT 6    | BIT 5    | BIT 4    | BIT 3    | BIT 2    | BIT 1    | BIT 0    |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | R-0,HSC  | R-0,HSC  | R-0,HSC  | U-0      | U-0      | R/C-0,HS | R-0,HSC  | R-0,HSC  | R/C-0,HS | R/C-0,HS | R-0,HSC  | R-0,HSC  | R-0,HSC  | R-0,HSC  | R-0,HSC  | R-0,HSC  |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | ACKSTAT  | TRSTAT   | ACKTIM   | -        | -        | BCL      | GCSTAT   | ADD10    | IWCOL    | I2COV    | D/A      | P        | S        | R/W      | RBF      | TBF      |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        BIT 15:     ACKSTAT: Acknowledge Status bit                                 X (0 = ACK received from slave)   
        BIT 14:     TRSTAT (Transmit Status bit, Master mode transmit op.)          X (0 = Master transmit is not in progress)                                                                                                                           
        BIT 13:     ACKTIM (Acknowledge Time Status bit, slave mode only)           X (0 = Not an Acknowledge sequence, cleared on ninth rising edge of the SCLx clock)
        BIT 12:     -                                                               X 
        BIT 11:     -                                                               X 
        BIT 10:     BCL (Bus Collision Detect bit, Master and Slave modes)          X (0 = No collision) 
        BIT 9:      GCSTAT (General Call Status bit)                                X (0 = General call address was not received)
        BIT 8:      ADD10 (10-Bit Address Status bit)                               X (0 = 10-bit address was not matched)
        BIT 7:      IWCOL (I2Cx Write Collision Detect bit)                         X (0 = No collision)               
        BIT 6:      I2COV (I2Cx Receive Overflow Flag bit)                          X (0 = No overflow)                                                       
        BIT 5:      D/A (Data/Address bit, I2C Slave mode)                          X (0 = Indicates that the last byte received was a device address)                         
        BIT 4:      P (Stop bit)                                                    X (0 = Stop bit was not detected last)                                                          
        BIT 3:      S (Start bit)                                                   X (0 = Start bit was not detected last)                              
        BIT 2:      R/W (Read/Write Information bit, slave mode only)               X (0 = Write, data transfer is an input to the slave)
        BIT 1:      RBF (Receive Buffer Full Status bit)                            X (0 = Receive is not complete; the I2CxRCV register is empty)
        BIT 0:      TBF (Transmit Buffer Full Status bit)                           X (0 = Transmit completes; the I2CxTRN register is empty)                
        */  
        I2C2STAT = 0x0000;     
        
        /*
        * I2C2CON: I2C2 CONTROL REGISTER
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | BIT 15   | BIT 14   | BIT 13   | BIT 12   | BIT 11   | BIT 10   | BIT 9    | BIT 8    | BIT 7    | BIT 6    | BIT 5    | BIT 4    | BIT 3    | BIT 2    | BIT 1    | BIT 0    |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | R/W-0    | -        | R/W-0    | R/W-1,HC | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0    | R/W-0,HC | R/W-0,HC | R/W-0,HC | R/W-0,HC | R/W-0,HC |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        | I2CEN    | -        | I2CSIDL  | SCLREL   | IPMIEN   | A10M     | DISSLW   | SMEN     | GCEN     | STREN    | ACKDT    | ACKEN    | RCEN     | PEN      | RSEN     | SEN      |
         -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        BIT 15:     I2CEN (I2Cx Enable bit)                                         1 = Enables the I2Cx module and configures the SDAx and SCLx pins as serial port pins   
        BIT 14:     -                                                               X                                                                                                                           
        BIT 13:     I2CSIDL (I2Cx Stop in Idle Mode bit)                            X (0 = Continues the module operation in the Idle mode)
        BIT 12:     SCLREL (SCLx Release Control bit, when op. as I2C slave)        1 = Releases the SCLx clock
        BIT 11:     IPMIEN (IPMI Enable bit)                                        0 = IPMI Support mode is disabled
        BIT 10:     A10M (10-Bit Slave Address bit)                                 X (0 = I2CxADD register is a 7-bit slave address)          
        BIT 9:      DISSLW (Disable Slew Rate Control bit)                          X (0 = Slew rate control is enabled)
        BIT 8:      SMEN (SMBus Input Levels bit)                                   X (0 = Disables the SMBus input thresholds)
        BIT 7:      GCEN (General Call Enable bit, when op. as I2C slave)           X (0 = Disables the general call address)                 
        BIT 6:      STREN (SCLx Clock Stretch Enable bit, I2C Slave mode only)      X (0 = Disables the user software or the receive clock stretching)                                                                                    
        BIT 5:      ACKDT (Acknowledge Data bit, I2C Master mode, receive op. only) X (0 = Sends an ACK during an Acknowledge)                                    
        BIT 4:      ACKEN (Acknowledge Seq. Enable bit, I2C Master mode, rec. op.)  X (0 = Acknowledge sequence is not in progress)                                                              
        BIT 3:      RCEN (Receive Enable bit, I2C Master mode)                      X (0 = Receive sequence is not in progress)                                    
        BIT 2:      PEN (Stop Condition Enable bit, I2C Master mode)                X (0 = Stop condition is not in progress)
        BIT 1:      RSEN (Repeated Start Condition Enable bit, I2C Master mode)     X (0 = Repeated Start condition is not in progress)
        BIT 0:      SEN (Start Condition Enable bit, I2C Master mode)               X (0 = Start condition is not in progress)                
        */  
        I2C2CON = 0x9000; 

        /*
        IFS3: INTERRUPT FLAG STATUS REGISTER 3
            BIT 2:  MI2C2IF: Master I2C2 Event Interrupt Flag Status bit
                        1 = Interrupt request has occurred
                        0 = Interrupt request has not occurred 
        
        IEC3: INTERRUPT FLAG STATUS REGISTER 3
            BIT 2:  MI2C2IE: Master I2C2 Event Interrupt Enable bit
                        1 = Interrupt request is enabled
                        0 = Interrupt request is not enabled
        */       
        IFS3bits.MI2C2IF = 0;                                                      //0 = Interrupt request has not occurred    
        IEC3bits.MI2C2IE = 0;                                                      //disable the master interrupt              
}
// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="I2C2Stop">
bool I2C2Stop(void)
{      
        //The lower 5 bits of the I2C2CON register must be ?0? (master logic inactive) before attempting to set the PEN bit
        I2C2CON = I2C2CON & 0xFFE0;
        
        //Initiates Stop condition on the SDA2 and SCL2 pins. Hardware is clear at the end of the master Stop sequence
        I2C2_STOP_CONDITION_ENABLE_BIT = true;     

        //If the user software writes to the I2C2TRN register when a Start sequence is in progress, the IWCOL status bit (I2C2STAT<7>) is set
        return  !I2C2_WRITE_COLLISION_STATUS_BIT;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2Start">
bool I2C2Start(void)
{       
        //Initiates Start condition on SDA2 and SCL2 pins. Hardware is clear at the end of the master Start sequence. 
        I2C2_START_CONDITION_ENABLE_BIT = true;     
            
        //If the user software writes to the I2C2TRN register when a Start sequence is in progress, the BCL status bit (I2C2STAT<10>) is set
        return  !I2C2_BUS_COLLISION_STATUS_BIT;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2BusIsIdle">
bool I2C2BusIsIdle(void)
{
        // SEN = 0      (Start condition is not in progress)
        // RSEN = 0     (Repeated Start condition is not in progress)
        // PEN = 0      (Stop condition is not in progress)
        // RCEN = 0     (Receive sequence is not in progress)
        // ACKEN = 0    (Acknowledge sequence is not in progress)
        // TRSTAT = 0   (Master transmit is not in progress)
    
        return !(I2C2CONbits.SEN || I2C2CONbits.RSEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2RepeatStart">
bool I2C2RepeatStart(void)
{
        //The lower 5 bits of the I2C2CON register must be ?0? (master logic inactive) before attempting to set the RSEN bit
        I2C2CON = I2C2CON & 0xFFE0;
    
        //Initiates Repeated Start condition on the SDA2 and SCL2 pins. Hardware is clear at the end of the master Repeated Start sequence.
        I2C2_REPEAT_START_CONDITION_ENABLE_BIT = true; 
        
        //If the user software writes to the I2C2TRN register when a Start sequence is in progress, the IWCOL status bit (I2C2STAT<7>) is set.
        //If the user software writes to the I2C2TRN register when a Start sequence is in progress, the BCL status bit (I2C2STAT<10>) is set
        // Check for collisions
        /* I2C2STAT: I2C2 Status Register
            BIT 7 IWCOL: I2Cx Write Collision Detect bit
                1 = An attempt to write to the I2CxTRN register failed because the I2C module is busy
                0 = No collision
                Hardware sets at occurrence of a write to the I2CxTRN register while busy (cleared by software)
         
            BIT 10 BCL: Bus Collision Detect bit (Master and Slave modes)
                1 = A bus collision has been detected during a master or slave operation
                0 = No collision
                Hardware sets at detection of a bus collision; clears when I2C module is disabled, I2CEN = 0
        */
        if (I2C2_BUS_COLLISION_STATUS_BIT || I2C2_WRITE_COLLISION_STATUS_BIT)    
        {
            return(false); 
        } else return(true);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2StopTransfer">
bool I2C2StopTransfer(void)
{
    unsigned int Retries = I2C2_RETRIES_NUMBER;
    
    
        // Send the Stop signal
        if(!I2C2Stop())
        {
            ///Error: Bus collision during transfer Stop
            return false;
        }
               
        // Wait for the signal to complete. The slave logic detects a Stop. Module sets S = 0 and P = 1.           
        while (((I2C2_START_STATUS_BIT) && (!I2C2_STOP_STATUS_BIT)) && Retries) {ClrWdt(); Retries--;}
        
        // Wait module clears PEN.
        while ((I2C2_STOP_CONDITION_ENABLE_BIT) && Retries) {ClrWdt(); Retries--;} 
        
        return true;  
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2StartTransfer">
bool I2C2StartTransfer(bool flag_Restart)
{
    unsigned int Retries = I2C2_RETRIES_NUMBER;
        
        
        // Send the Start (or Restart) signal
        if(flag_Restart == true)
        {
            if(!I2C2RepeatStart())
            {
                //Error: Bus collision during transfer Start
                return false;             
            }                     
        }
        else
        {
            // Wait for the bus to be idle
            while ((!I2C2BusIsIdle()) && Retries) {ClrWdt(); Retries--;}
            
            // Initiate Start event
            if(!I2C2Start())
            {
                //Error: Bus collision during transfer Start           
                return false;
            }
        }
            
        // Wait for the signal to complete. The slave detects the Start and sets S = 1 and P = 0           
        while (((!I2C2_START_STATUS_BIT) && (I2C2_STOP_STATUS_BIT)) && Retries) {ClrWdt(); Retries--;}
          
        if(flag_Restart)
        {
            // Wait module clears RSEN
            while ((I2C2_REPEAT_START_CONDITION_ENABLE_BIT) && Retries) {ClrWdt(); Retries--;}        
        }
        else
        {
            // Wait module clears SEN
            while ((I2C2_START_CONDITION_ENABLE_BIT) && Retries) {ClrWdt(); Retries--;}   
        }
        
        return true;
}
// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="I2C2ReadOneByte">
unsigned char I2C2ReadOneByte(void)
{
    unsigned char i2cbyte;
    unsigned int Retries = I2C2_RETRIES_NUMBER; 

    
        if(I2C2ReceiverEnable(true))
        {                  
            while ((!I2C2ReceivedDataIsAvailable()) && Retries) {ClrWdt(); Retries--;}
            i2cbyte = I2C2GetByte();
            
            return i2cbyte;    
        }
        else
        {
            //Error: I2C Receive Overflow
            return 0xFF;
        }		
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2TransmitOneByte">
bool I2C2TransmitOneByte(unsigned char Data)
{ 
    unsigned int Retries = I2C2_RETRIES_NUMBER; 
    
    
        // Wait for the transmitter to be ready
        while ((!I2C2TransmitterIsReady()) && Retries) {ClrWdt(); Retries--;}

        // Transmit the byte
        if (!I2C2SendByte(Data))
        {
            //Error: I2C Master Bus Collision
            return false;
        }

        // Wait for the transmission to finish
        while ((!I2C2TransmissionHasCompleted()) && Retries) {ClrWdt(); Retries--;}


        return true;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2AcknowledgeByte">
bool I2C2AcknowledgeByte(bool ack_Type)                 
{
     unsigned int Retries = I2C2_RETRIES_NUMBER;
     
     
        //The lower 5 bits of the I2C2CON register must be ?0? (master logic inactive) before attempting to set the ACKEN bit
        I2C2CON = I2C2CON & 0xFFE0;
        
        //Acknowledge Data bit (when operating as I2C master; applicable during master receive), value that will be transmitted when the software initiates an Acknowledge sequence
        if (ack_Type == ACK)
        {//ACK
             I2C2_ACKNOWLEDGE_DATA_BIT = false;    
        }
        else
        {//NAK
            I2C2_ACKNOWLEDGE_DATA_BIT = true;    
        }
        
        //Initiates Acknowledge sequence on SDAx and SCLx pins and transmits the ACKDT data bit. Hardware is clear at the end of the master Acknowledge sequence
        I2C2_ACKNOWLEDGE_ENABLE_BIT  = true; 
                          
        // Wait module clears ACKEN
        while ((I2C2_ACKNOWLEDGE_ENABLE_BIT) && Retries) {ClrWdt(); Retries--;}
        
              
        //If the user software writes to the I2CxTRN register when a Start sequence is in progress, the IWCOL status bit (I2C2STAT<7>) is set.
        return  !I2C2_WRITE_COLLISION_STATUS_BIT;   
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2ByteWasAcknowledged">
bool I2C2ByteWasAcknowledged(void)                        
{
        /* I2C2STAT: I2C2 Status Register
      
            BIT 15 ACKSTAT: Acknowledge Status bit
                1 = NACK received from slave
                0 = ACK received from slave
                Hardware sets or clears at the end of slave or master Acknowledge
        */
        return(!I2C2_ACKNOWLEDGE_STATUS_BIT);
}
// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="I2C2GetByte">
unsigned char I2C2GetByte(void)
{
        return(I2C2_RECEIVE_REG);    //I2C2 Receive Buffer Register: This is the buffer register from which data bytes can be read. The I2C2RCV register is a read-only register
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2SendByte">
bool I2C2SendByte(unsigned char data)
{
        // Send the byte
        I2C2_TRANSMIT_REG = data;     //I2C2Transmit Register: The bytes are written to this register during a transmit operation. The I2C2TRN register is a read/write register

            
        // Check for collisions
        /* I2C2STAT: I2C2 Status Register
            BIT 7 IWCOL: I2Cx Write Collision Detect bit
                1 = An attempt to write to the I2CxTRN register failed because the I2C module is busy
                0 = No collision
                Hardware sets at occurrence of a write to the I2CxTRN register while busy (cleared by software)
         
            BIT 10 BCL: Bus Collision Detect bit (Master and Slave modes)
                1 = A bus collision has been detected during a master or slave operation
                0 = No collision
                Hardware sets at detection of a bus collision; clears when I2C module is disabled, I2CEN = 0
        */
        if (I2C2_BUS_COLLISION_STATUS_BIT || I2C2_WRITE_COLLISION_STATUS_BIT)    
        {
            return(false); 
        } else return(true);
}
// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="I2C2ReceiverEnable">
bool I2C2ReceiverEnable(bool state)
{
        // Enable/Disable the receiver
        I2C2_RECEIVE_ENABLE_BIT = state;
	
        // Check for an overflow condition
        return (!I2C2_RECEIVER_OVERFLOW_STATUS_BIT);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2TransmitterIsReady">
bool I2C2TransmitterIsReady(void)
{
        /* I2C2STAT: I2C2 STATUS REGISTER
        BIT 0 (TBF: Transmit Buffer Full Status bit)
            1 = Transmit is in progress, I2C2TRN is full
            0 = Transmit is complete, I2C2TRN is empty
            Hardware is set when software writes to I2C2TRN; hardware is clear at the completion of data transmission
        */  
        return(!I2C2_TRANSMIT_BUFFER_FULL_STATUS_BIT);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2ReceivedDataIsAvailable">
bool I2C2ReceivedDataIsAvailable(void)
{
        /* I2C2STAT: I2C2 STATUS REGISTER
        BIT 1 RBF: Receive Buffer Full Status bit
            1 = Receive is complete, I2C2RCV is full
            0 = Receive is not complete, I2C2RCV is empty
            Hardware is set when I2C2RCV is written with the received byte; hardware is clear when the software reads I2C2RCV.
        */
        return(I2C2_RECEIVER_BUFFER_FULL_STATUS_BIT);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C2TransmissionHasCompleted">
bool I2C2TransmissionHasCompleted(void)
{
        /* I2C2STAT: I2C2 Status Register    
            BIT 14 TRSTAT: Transmit Status bit (when operating as I2C master; applicable to master transmit operation)
                1 = Master transmit is in progress (8 bits + ACK)
                0 = Master transmit is not in progress
                Hardware is set at the beginning of master transmission; hardware is clear at the end of slave Acknowledge
        */  
        return(!I2C2_TRANSMIT_STATUS_BIT);
}
// </editor-fold>
// </editor-fold>
/******************************************************************************************************************/

/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/