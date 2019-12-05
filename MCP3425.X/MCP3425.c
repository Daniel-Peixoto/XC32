/***************************************************** Include ******************************************************/
// <editor-fold defaultstate="collapsed" desc="INCLUDE">
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "I2C1.h"
#include "I2C2.h"
#include "MCP3425.h"
#include "p24FJ128GC006.h"
// </editor-fold>
/********************************************************************************************************************/

/****************************************************** Define ******************************************************/

/********************************************************************************************************************/

/****************************************************** Struct ******************************************************/
// <editor-fold defaultstate="collapsed" desc="STRUCT">
MCP3425_type MCP3425;
// </editor-fold>
/********************************************************************************************************************/

/***************************************************** Constant *****************************************************/

/********************************************************************************************************************/

/*********************************************** Extern Global Variable *********************************************/

/********************************************************************************************************************/

/*************************************************** Global Variable ************************************************/

/********************************************************************************************************************/

/************************************************* Function Prototype ***********************************************/

/********************************************************************************************************************/
/********************************************************************************************************************/


/***************************************************** Function *****************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
// <editor-fold defaultstate="collapsed" desc="FUNCTION">
/******************************************************* ADC ********************************************************/
bool ADC_Initialize(unsigned char address, unsigned char I2CNumber, unsigned char deviceCode, bool conversionMode, unsigned char gain, unsigned char sampleRate, MCP3425_type *MCP3425_buffer)
{
        (*MCP3425_buffer).Address = address;
        (*MCP3425_buffer).I2C_Number = I2CNumber;
        (*MCP3425_buffer).DeviceCode = deviceCode;

        Update_ADC_REGISTER (conversionMode, gain, sampleRate, MCP3425_buffer);
        return Write_ADC_REGISTER (MCP3425_buffer);
}


//----------------------------------------------------- Write ------------------------------------------------------//
// <editor-fold defaultstate="collapsed" desc="Write">
bool Write_ADC_REGISTER (MCP3425_type *MCP3425_buffer)
{
    bool Next = false;
    unsigned int Retries = 0;
    
    
        (*MCP3425_buffer).Flag_Write = true;
      
        switch ((*MCP3425_buffer).I2C_Number)
        {
            // <editor-fold defaultstate="collapsed" desc="I2C1">
            case MCP3425_I2C1:
                Retries = I2C1_RETRIES_NUMBER;
                
                                                                                            
                do {Retries--; ClrWdt(); Next = I2C1StartTransfer (false);} while (!Next && Retries);               // Start the transfer
                              
                if (Next) {Next = I2C1TransmitOneByte((*MCP3425_buffer).WriteBuffer[0]);}                           // Transmit the byte[0]                                                                                                          
                if (Next) {do {Retries--; ClrWdt(); Next = I2C1ByteWasAcknowledged();} while (!Next && Retries);}   // Verify that the byte was acknowledged
                           
                if (Next) {Next = I2C1TransmitOneByte((*MCP3425_buffer).WriteBuffer[1]);}                           // Transmit the byte[1]                
                if (Next) {do {Retries--; ClrWdt(); Next = I2C1ByteWasAcknowledged();} while (!Next && Retries);}   // Verify that the byte was acknowledged
                
                do {Retries--; ClrWdt();} while (!I2C1BusIsIdle() && Retries);                                      // Wait for the bus to be idle
               
                do {Retries--; ClrWdt();} while (!I2C1StopTransfer() && Retries);                                   // Stop the transfer  
 
                break;
            // </editor-fold>
                
            // <editor-fold defaultstate="collapsed" desc="I2C2">
            case MCP3425_I2C2:
                Retries = I2C2_RETRIES_NUMBER;
                
                                                                                            
                do {Retries--; ClrWdt(); Next = I2C2StartTransfer (false);} while (!Next && Retries);               // Start the transfer
                              
                if (Next) {Next = I2C2TransmitOneByte((*MCP3425_buffer).WriteBuffer[0]);}                           // Transmit the byte[0]                                                                                        
                if (Next) {do {Retries--; ClrWdt(); Next = I2C2ByteWasAcknowledged();} while (!Next && Retries);}   // Verify that the byte was acknowledged
                           
                if (Next) {Next = I2C2TransmitOneByte((*MCP3425_buffer).WriteBuffer[1]);}                           // Transmit the byte[1]
                if (Next) {do {Retries--; ClrWdt(); Next = I2C2ByteWasAcknowledged();} while (!Next && Retries);}   // Verify that the byte was acknowledged
                
                do {Retries--; ClrWdt();} while (!I2C2BusIsIdle() && Retries);                                      // Wait for the bus to be idle
               
                do {Retries--; ClrWdt();} while (!I2C2StopTransfer() && Retries);                                   // Stop the transfer  
 
                break;
            // </editor-fold>
        }
        
        
        if (Next)
        {
            // <editor-fold defaultstate="collapsed" desc="ADC Write was performed">
            (*MCP3425_buffer).Flag_Write = false;
            
            return true;
            // </editor-fold> 
        }
        else
        {
            // <editor-fold defaultstate="collapsed" desc="ADC Write wasn't performed">
            (*MCP3425_buffer).Flag_Write = true;
            
            return false;
            // </editor-fold> 
        }     
}
// </editor-fold>
//------------------------------------------------------------------------------------------------------------------//

//------------------------------------------------------ Read ------------------------------------------------------//
// <editor-fold defaultstate="collapsed" desc="Read">
bool Read_ADC (MCP3425_type *MCP3425_buffer)
{
    bool Next = false; 
    unsigned char Data;
    unsigned int Retries = 0;
     
    
        (*MCP3425_buffer).Flag_Read = true;
   
        switch ((*MCP3425_buffer).I2C_Number)
        {
            // <editor-fold defaultstate="collapsed" desc="I2C1">
            case MCP3425_I2C1:
                Retries = I2C1_RETRIES_NUMBER;
                   
                Data = ((*MCP3425_buffer).DeviceCode << 4) | ((*MCP3425_buffer).Address << 1) | MCP3425_READ;              
                
                
                do {Retries--; ClrWdt(); Next = I2C1StartTransfer (false);} while (!Next && Retries);               // Start the transfer
                              
                if (Next) {Next = I2C1TransmitOneByte(Data);}                                                       // Transmit the byte[0]                                                                                                       
                if (Next) {do {Retries--; ClrWdt(); Next = I2C1ByteWasAcknowledged();} while (!Next && Retries);}   // Verify that the byte was acknowledged 
                
                if (Next)
                {
                    (*MCP3425_buffer).ReadBuffer[0] = I2C1ReadOneByte();                                            // Read the byte[0]
                    Next = I2C1AcknowledgeByte(ACK);                                                                // Generate ACK
                }
                
                if (Next)
                {
                    (*MCP3425_buffer).ReadBuffer[1] = I2C1ReadOneByte();                                            // Read the byte[1]
                     Next = I2C1AcknowledgeByte(NACK);                                                              // Generate NACK
                }
                
                do {Retries--; ClrWdt();} while (!I2C1BusIsIdle() && Retries);                                      // Wait for the bus to be idle
               
                do {Retries--; ClrWdt();} while (!I2C1StopTransfer() && Retries);                                   // Stop the transfer     

                break;
            // </editor-fold>   
         
            // <editor-fold defaultstate="collapsed" desc="I2C2">
            case MCP3425_I2C2:
                Retries = I2C2_RETRIES_NUMBER;
                   
                Data = ((*MCP3425_buffer).DeviceCode << 4) | ((*MCP3425_buffer).Address << 1) | MCP3425_READ;              
                
                
                do {Retries--; ClrWdt(); Next = I2C2StartTransfer (false);} while (!Next && Retries);               // Start the transfer
                              
                if (Next) {Next = I2C2TransmitOneByte(Data);}                                                       // Transmit the byte[0]                                                                                                       
                if (Next) {do {Retries--; ClrWdt(); Next = I2C2ByteWasAcknowledged();} while (!Next && Retries);}   // Verify that the byte was acknowledged 
                
                if (Next)
                {
                    (*MCP3425_buffer).ReadBuffer[0] = I2C2ReadOneByte();                                            // Read the byte[0]
                    Next = I2C2AcknowledgeByte(ACK);                                                                // Generate ACK
                }
                
                if (Next)
                {
                    (*MCP3425_buffer).ReadBuffer[1] = I2C2ReadOneByte();                                            // Read the byte[1]
                     Next = I2C2AcknowledgeByte(NACK);                                                              // Generate NACK
                }
                
                do {Retries--; ClrWdt();} while (!I2C2BusIsIdle() && Retries);                                      // Wait for the bus to be idle
               
                do {Retries--; ClrWdt();} while (!I2C2StopTransfer() && Retries);                                   // Stop the transfer     

                break;
            // </editor-fold>                          
        }
        
        
        if (Next)
        {
            // <editor-fold defaultstate="collapsed" desc="ADC Read was performed">
            (*MCP3425_buffer).Flag_Read = false;
            (*MCP3425_buffer).Read_ADCData = ((*MCP3425_buffer).ReadBuffer[0] << 8) | (*MCP3425_buffer).ReadBuffer[1];  
            
            return true;
            // </editor-fold> 
        }
        else
        {
            // <editor-fold defaultstate="collapsed" desc="ADC Read wasn't performed">
            (*MCP3425_buffer).Flag_Read = true;
            (*MCP3425_buffer).ReadBuffer[0] = 0xFF;
            (*MCP3425_buffer).ReadBuffer[1] = 0xFF;
            (*MCP3425_buffer).ReadBuffer[2] = 0xFF;
            
            return false;
            // </editor-fold> 
        }
}


bool Read_ADC_REGISTER (MCP3425_type *MCP3425_buffer)
{
    bool Next = false; 
    unsigned char i, Data;
    unsigned int Retries = 0;
    
    
        (*MCP3425_buffer).Flag_Read = true;
        
        switch ((*MCP3425_buffer).I2C_Number)
        {
            // <editor-fold defaultstate="collapsed" desc="I2C1">
            case MCP3425_I2C1:
                Retries = I2C1_RETRIES_NUMBER;
                   
                Data = ((*MCP3425_buffer).DeviceCode << 4) | ((*MCP3425_buffer).Address << 1) | MCP3425_READ;              

                
                do {Retries--; ClrWdt(); Next = I2C1StartTransfer (false);} while (!Next && Retries);               // Start the transfer
                              
                if (Next) {Next = I2C1TransmitOneByte(Data);}                                                       // Transmit the byte[0]                                                                                                       
                if (Next) {do {Retries--; ClrWdt(); Next = I2C1ByteWasAcknowledged();} while (!Next && Retries);}   // Verify that the byte was acknowledged 
                
                for (i = 0; i < 3; i++)
                {
                    if (Next)
                    {
                        (*MCP3425_buffer).ReadBuffer[i] = I2C1ReadOneByte();                                        // Read the byte[i]
                        if (i < 2) Next = I2C1AcknowledgeByte(ACK);                                                 // Generate ACK
                    }
                } 
                
                Next = I2C1AcknowledgeByte(NACK);                                                                   // Generate NACK
                
                do {Retries--; ClrWdt();} while (!I2C1BusIsIdle() && Retries);                                      // Wait for the bus to be idle
               
                do {Retries--; ClrWdt();} while (!I2C1StopTransfer() && Retries);                                   // Stop the transfer   
                
                break;
            // </editor-fold>  
                
            // <editor-fold defaultstate="collapsed" desc="I2C2">
            case MCP3425_I2C2:
                Retries = I2C2_RETRIES_NUMBER;
                   
                Data = ((*MCP3425_buffer).DeviceCode << 4) | ((*MCP3425_buffer).Address << 1) | MCP3425_READ;              

                
                do {Retries--; ClrWdt(); Next = I2C2StartTransfer (false);} while (!Next && Retries);               // Start the transfer
                              
                if (Next) {Next = I2C2TransmitOneByte(Data);}                                                       // Transmit the byte[0]                                                                                                       
                if (Next) {do {Retries--; ClrWdt(); Next = I2C2ByteWasAcknowledged();} while (!Next && Retries);}   // Verify that the byte was acknowledged 
                
                for (i = 0; i < 3; i++)
                {
                    if (Next)
                    {
                        (*MCP3425_buffer).ReadBuffer[i] = I2C2ReadOneByte();                                        // Read the byte[i]
                        if (i < 2) Next = I2C2AcknowledgeByte(ACK);                                                 // Generate ACK
                    }
                } 
                
                Next = I2C2AcknowledgeByte(NACK);                                                                   // Generate NACK
                
                do {Retries--; ClrWdt();} while (!I2C2BusIsIdle() && Retries);                                      // Wait for the bus to be idle
               
                do {Retries--; ClrWdt();} while (!I2C2StopTransfer() && Retries);                                   // Stop the transfer   
                
                break;    
            // </editor-fold>              
        }

        
        if (Next)
        {
            // <editor-fold defaultstate="collapsed" desc="ADC Register Read was performed">
            (*MCP3425_buffer).Flag_Read = false;           
            (*MCP3425_buffer).Gain = (*MCP3425_buffer).ReadBuffer[2] & GAIN_MASK;
            (*MCP3425_buffer).SampleRate = ((*MCP3425_buffer).ReadBuffer[2] & SAMPLERATE_MASK) >> 2;
            (*MCP3425_buffer).ConversionMode = ((*MCP3425_buffer).ReadBuffer[2] & CONVERSIONMODE_MASK) >> 4;
            (*MCP3425_buffer).ConversionReady = ((*MCP3425_buffer).ReadBuffer[2] & CONVERSIONREADY_MASK) >> 7;
            (*MCP3425_buffer).Read_ADCData = ((*MCP3425_buffer).ReadBuffer[0] << 8) | (*MCP3425_buffer).ReadBuffer[1]; 
            
            return true;
            // </editor-fold>    
        }
        else
        {
            // <editor-fold defaultstate="collapsed" desc="ADC Register Read wasn't performed">
            (*MCP3425_buffer).Flag_Read = true;
            (*MCP3425_buffer).ReadBuffer[0] = 0xFF;
            (*MCP3425_buffer).ReadBuffer[1] = 0xFF;
            (*MCP3425_buffer).ReadBuffer[2] = 0xFF;
            
            return false;
            // </editor-fold>
        }     
}
// </editor-fold>
//------------------------------------------------------------------------------------------------------------------//

//----------------------------------------------------- Update -----------------------------------------------------//
// <editor-fold defaultstate="collapsed" desc="Update">
void Update_ADC_REGISTER (bool conversionMode, unsigned char gain, unsigned char sampleRate, MCP3425_type *MCP3425_buffer)
{
        (*MCP3425_buffer).Gain = gain;
        (*MCP3425_buffer).SampleRate = sampleRate;
        (*MCP3425_buffer).ConversionMode = conversionMode;
        (*MCP3425_buffer).Write_ADCData = (0b00011111) & (((*MCP3425_buffer).ConversionMode << 4) | ((*MCP3425_buffer).SampleRate << 2) | ((*MCP3425_buffer).Gain)); 

        (*MCP3425_buffer).WriteBuffer[0] = ((*MCP3425_buffer).DeviceCode << 4) | ((*MCP3425_buffer).Address << 1) | MCP3425_WRITE;
        (*MCP3425_buffer).WriteBuffer[1] =  (*MCP3425_buffer).Write_ADCData;
}   
// </editor-fold>
//------------------------------------------------------------------------------------------------------------------//
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
