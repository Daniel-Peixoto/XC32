/********************************************************************************************************************/
// <editor-fold defaultstate="collapsed" desc="INTEGRATED CIRCUIT">
/*
* INTEGRATED CIRCUIT (MCP3425)

                                     ---------------
                                1---| VIN+     VIN- |---6
                                    |               |
                                2---| VSS       VDD |---5
                                    |               |
                                3---| SCL       SDA |---4
                                     ---------------


* REGISTER 5-1: CONFIGURATION REGISTER
-------------------------------------------------------------------------
| BIT 7  | BIT 6  | BIT 5  | BIT 4  | BIT 3  | BIT 2  | BIT 1  | BIT 0  |
-------------------------------------------------------------------------
| R/W-1  | R/W-0  | R/W-0  | R/W-1  | R/W-0  | R/W-0  | R/W-0  | R/W-0  |
-------------------------------------------------------------------------
|  RDY   |   C1   |   C0   |  O/C   |   S1   |   S0   |   G1   |   G0   |
-------------------------------------------------------------------------

BIT 7:  RDY: Ready Bit (This bit is the data ready flag. In read mode, this bit indicates if the output register has been updated with a new conversion. In One-Shot Conversion mode, writing this bit to ?1? initiates a new conversion)     
            Reading RDY bit with the read command:
                1 = Output register has not been updated.
                0 = Output register has been updated with the latest conversion data       
            Writing RDY bit with the write command: (Continuous Conversion mode: No effect) 
                One-Shot Conversion mode:
                1 = Initiate a new conversion.
                0 = No effect.   
 
BIT 6:  C1-C0: Channel Selection Bits (These are the Channel Selection bits, but not used in the MCP3425 device)                                                                                
BIT 5: 
                             
BIT 4:  O/C: Conversion Mode Bit      
            1 = Continuous Conversion Mode. Once this bit is selected, the device performs data conversions continuously
            0 = One-Shot Conversion Mode. The device performs a single conversion and enters a low power standby mode until it receives another write/read command 
                                              
BIT 3:  S1-S0: Sample Rate Selection Bit                            
BIT 2:      00 = 240 SPS (12 bits)
            01 = 60 SPS (14 bits)
            10 = 15 SPS (16 bits)
 
BIT 1:  G1-G0: PGA Gain Selector Bits    
BIT 0:      00 = 1 V/V
            01 = 2 V/V
            10 = 4 V/V
            11 = 8 V/V
  
  
 
* TABLE 5-1: WRITE CONFIGURATION BITS
---------------------------------------------------------------------------------------------------------------------
| R/W | O/C | RDY | OPERATION                                                                                       |  
---------------------------------------------------------------------------------------------------------------------
|  0  |  0  |  0  | NO EFFECT IF ALL OTHER BITS REMAIN THE SAME - OPERATION CONTINUES WITH THE PREVIOUS SETTINGS    |  
|  0  |  0  |  1  | INITIATE ONE-SHOT CONVERSION                                                                    |  
|  0  |  1  |  0  | INITIATE CONTINUOUS CONVERSION                                                                  |  
|  0  |  1  |  1  | INITIATE CONTINUOUS CONVERSION                                                                  |  
---------------------------------------------------------------------------------------------------------------------
  
  
* TABLE 5-2: READ CONFIGURATION BITS
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| R/W | O/C | RDY | OPERATION                                                                                                                                                                   |
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  1  |  0  |  0  | NEW CONVERSION RESULT IN ONE-SHOT CONVERSION MODE HAS JUST BEEN READ. THE RDY BITS REMAINS LOW UNTIL SET BY A NEW WRITE COMMAND                                             |  
|  1  |  0  |  1  | ONE-SHOT CONVERSION IS IN PROGRESS. THE CONVERSION RESULT IN NOT UPDATED YET. THE RDY BIT STAYS HIGH UNTIL THE CURRENT CONVERSION IS COMPLETED                              |                                                                   
|  1  |  1  |  0  | NEW CONVERSION RESULT IN CONTINUOUS CONVERSION MODE HAS JUST BEEN READ. THE RDY BIT CHANGES TO HIGH AFTER READING THE CONVERSION DATA                                       |                                                              
|  1  |  1  |  1  | THE CONVERSION RESULT IN CONTINUOUS CONVERSION MODE WAS ALREADY READ. THE NEXT NEW CONVERSION DATA IS NOT READY. THE RDY BIT STAYS HIGH UNTIL A NEW CONVERSION IS COMPLETED |
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   
  
* TABLE 5-3: OUTPUT CODES OF EACH RESOLUTION OPTION
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
| CONVERSION OPTION | DIGITAL OUTPUT CODES                                                                                                                      |
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
|       16 BITS     | D15 ~ D8 (1ST DATA BYTE) - D7 ~ D0 (2ND DATA BYTE) - CONFIGURATION BYTE (D15 IS MSB, = SIGN BIT)                                          |  
|       14 BITS     | MMD13 ~ D8 (1ST DATA BYTE) - D7 ~ D0 (2ND DATA BYTE) - CONFIGURATION BYTE (D13 IS MSB, = SIGN BIT, M IS REPEATED MSB OF THE DATA BYTE     |                                                                     
|       12 BITS     | MMMMD11 ~ D8 (1ST DATA BYTE) - D7 ~ D0 (2ND DATA BYTE) - CONFIGURATION BYTE (D11 IS MSB, = SIGN BIT, M IS REPEATED MSB OF THE DATA BYTE   |                                                                 
-----------------------------------------------------------------------------------------------------------------------------------------------------------------
   
  
  
* READ ADC: 
--------------------------------------------     -----------------------------------------------     -----------------------------------------    
|                   BYTE 1                 |     |                    BYTE 2                   |     |                 BYTE 3                |     
------------------------------------------------------------------------------------------------------------------------------------------------------------
| START | DEVICE CODE | ADDRESS BITS | R/W | ACK | D15 | D14 | D13 | D12 | D11 | D10 | D9 | D8 | ACK | D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 | NACK | STOP |
------------------------------------------------------------------------------------------------------------------------------------------------------------
|   M   |     1101    |   A2 A1 A0   |  1  |  S  |                    ADC DATA                 |  M  |                                       |   M  |   M  |
------------------------------------------------------------------------------------------------------------------------------------------------------------
M-MASTER TO SLAVE; S-SLAVE TO MASTER  


* READ ADC REGISTER: 
--------------------------------------------     -----------------------------------------------     -----------------------------------------     -------------------------------------------
|                   BYTE 1                 |     |                    BYTE 2                   |     |                 BYTE 3                |     |                  BYTE 4                 | 
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| START | DEVICE CODE | ADDRESS BITS | R/W | ACK | D15 | D14 | D13 | D12 | D11 | D10 | D9 | D8 | ACK | D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 | ACK | RDY | C1 | C0 | O/C | S1 | S0 | G1 | G0 | NACK | STOP |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   M   |     1101    |   A2 A1 A0   |  1  |  S  |                    ADC DATA                 |  M  |                                       |  M  |  X  |  X |  X |  X  |  X |  X |  X |  X |   M  |   M  |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
M-MASTER TO SLAVE; S-SLAVE TO MASTER  
 
  
 * WRITE ADC: 
--------------------------------------------     -------------------------------------------        
|                   BYTE 1                 |     |                 BYTE 2                  |    
---------------------------------------------------------------------------------------------------------
| START | DEVICE CODE | ADDRESS BITS | R/W | ACK | RDY | C1 | C0 | O/C | S1 | S0 | G1 | G0 | ACK | STOP |
---------------------------------------------------------------------------------------------------------
|   M   |     1101    |   A2 A1 A0   |  0  |  S  |  1  | X  | X  |  0  |  1 |  0 |  1 |  1 |  S  |   M  | 
---------------------------------------------------------------------------------------------------------
M-MASTER TO SLAVE; S-SLAVE TO MASTER 
*/ 
// </editor-fold>
/********************************************************************************************************************/

/****************************************************** Include *****************************************************/

/********************************************************************************************************************/

/******************************************************* Define *****************************************************/
// <editor-fold defaultstate="collapsed" desc="DEFINE">
#define MCP3425_A0                      0
#define MCP3425_A1                      1
#define MCP3425_A2                      2
#define MCP3425_A3                      3   

#define MCP3425_I2C1                    0
#define MCP3425_I2C2                    1

#define SAMPLE_RATE_240SPS_12BITS       0
#define SAMPLE_RATE_60SPS_14BITS        1
#define SAMPLE_RATE_15SPS_16BITS        2

#define GAIN_1_VV                       0
#define GAIN_2_VV                       1
#define GAIN_4_VV                       2
#define GAIN_8_VV                       3

#define ONE_SHOT                        0
#define CONTINUOUS                      1

#define MCP3425_READ                    0b1
#define MCP3425_WRITE                   0b0
#define MCP3425_DEVICE_CODE             0b1101

#define GAIN_MASK                       0x03
#define SAMPLERATE_MASK                 0x0C
#define CONVERSIONMODE_MASK             0x10
#define CONVERSIONREADY_MASK            0x80
// </editor-fold>
/********************************************************************************************************************/

/******************************************************* Struct *****************************************************/
// <editor-fold defaultstate="collapsed" desc="STRUCT">
typedef struct
{	
    unsigned char I2C_Number;                                                       // I2C channel number
    unsigned char Address;                                                          // I2C address A2,A1,A0
    unsigned char DeviceCode;
	unsigned char Gain;
    unsigned char SampleRate;
    unsigned int Read_ADCData;
    unsigned char Write_ADCData;
    unsigned char WriteBuffer[2];
    unsigned char ReadBuffer[4];
    bool ConversionMode;
    bool ConversionReady;
	bool Flag_Write;                                                                // I2C write is pendding
	bool Flag_Read;                                                                 // I2C read is pendding
}_MCP3425;
// </editor-fold>
/********************************************************************************************************************/

/******************************************************* Typedef ****************************************************/
// <editor-fold defaultstate="collapsed" desc="TYPEDEF">
#define MCP3425_type _MCP3425
// </editor-fold>
/********************************************************************************************************************/

/****************************************************** Constant ****************************************************/

/********************************************************************************************************************/

/*************************************************** Global Variable ************************************************/

/********************************************************************************************************************/

/************************************************ Extern Global Variable ********************************************/

/********************************************************************************************************************/

/************************************************* Function Prototype ***********************************************/
// <editor-fold defaultstate="collapsed" desc="FUNCTION PROTOTYPE">
/**
  @Function
    bool ADC_Initialize(unsigned char address, unsigned char I2CNumber, unsigned char deviceCode, bool conversionMode, unsigned char gain, unsigned char sampleRate, MCP3425_type *MCP3425_buffer)

  @Summary
    ADC initialize

  @Description
    None

  @Preconditions
    None

  @Param
    address [MCP3425_A0, MCP3425_A1, MCP3425_A2, MCP3425_A3]
    I2CNumber [MCP3425_I2C1, MCP3425_I2C2]
    deviceCode [MCP3425_DEVICE_CODE] 
    conversionMode [ONE_SHOT, CONTINUOUS]
    gain [GAIN_1_VV, GAIN_2_VV, GAIN_4_VV, GAIN_8_VV]
    sampleRate [SAMPLE_RATE_240SPS_12BITS, SAMPLE_RATE_60SPS_14BITS, SAMPLE_RATE_15SPS_16BITS]
    Pointer to MCP3425_type struct

  @Returns
    True, if ADC correctely initialized

  @Example
    <code>
    ADC_Initialize(MCP3425_A0, MCP3425_I2C1, MCP3425_DEVICE_CODE, CONTINUOUS, GAIN_1_VV, SAMPLE_RATE_15SPS_16BITS, &MCP3425); //HL - ADC register data 16bits
    </code>
*/
bool ADC_Initialize(unsigned char address, unsigned char I2CNumber, unsigned char deviceCode, bool conversionMode, unsigned char gain, unsigned char sampleRate, MCP3425_type *MCP3425_buffer);


/**
  @Function
    bool Read_ADC (MCP3425_type *MCP3425_buffer)

  @Summary
    Read ADC value

  @Description
    None

  @Preconditions
    None
 
  @Param
    Pointer to MCP3425_type struct

  @Returns
    True, if ADC Read was performed

  @Example
    <code>
    Read_ADC (&MCP3425); //HL - ADC register data 16bits
    </code>
*/
bool Read_ADC (MCP3425_type *MCP3425_buffer);


/**
  @Function
    bool Read_ADC_REGISTER (MCP3425_type *MCP3425_buffer)

  @Summary
    Read ADC Register

  @Description
    None
        
  @Preconditions
    None

  @Param
    Pointer to MCP3425_type struct

  @Returns
    True, if ADC Register Read was performed

  @Example
    <code>
    Read_ADC_REGISTER (&MCP3425); //HL- ADC register, configuration
    </code>
*/
bool Read_ADC_REGISTER (MCP3425_type *MCP3425_buffer);


/**
  @Function
    bool Write_ADC_REGISTER (MCP3425_type *MCP3425_buffer)
  
  @Summary
    This command is used to change the ADC register

  @Description
    None

  @Preconditions
    Call function: void Update_ADC_REGISTER (bool conversionMode, unsigned char gain, unsigned char sampleRate, MCP3425_type *MCP3425_buffer)

  @Param
     Pointer to MCP3425_type struct

  @Returns
    True, if ADC Register Write was performed

  @Example
    <code>
    Write_ADC_REGISTER (&MCP3425);
    </code>
*/
bool Write_ADC_REGISTER (MCP3425_type *MCP3425_buffer);


/**
  @Function
    void Update_ADC_REGISTER (bool conversionMode, unsigned char gain, unsigned char sampleRate, MCP3425_type *MCP3425_buffer)
  
  @Summary
    This command is used to change the DAC register. EEPROM is not affected.

  @Description
    * REGISTER 5-1: CONFIGURATION REGISTER
    -------------------------------------------------------------------------
    | BIT 7  | BIT 6  | BIT 5  | BIT 4  | BIT 3  | BIT 2  | BIT 1  | BIT 0  |
    -------------------------------------------------------------------------
    | R/W-1  | R/W-0  | R/W-0  | R/W-1  | R/W-0  | R/W-0  | R/W-0  | R/W-0  |
    -------------------------------------------------------------------------
    |  RDY   |   C1   |   C0   |  O/C   |   S1   |   S0   |   G1   |   G0   |
    -------------------------------------------------------------------------

    BIT 4:  O/C: Conversion Mode Bit      
                1 = Continuous Conversion Mode. Once this bit is selected, the device performs data conversions continuously
                0 = One-Shot Conversion Mode. The device performs a single conversion and enters a low power standby mode until it receives another write/read command 

    BIT 3:  S1-S0: Sample Rate Selection Bit                            
    BIT 2:      00 = 240 SPS (12 bits)
                01 = 60 SPS (14 bits)
                10 = 15 SPS (16 bits)

    BIT 1:  G1-G0: PGA Gain Selector Bits    
    BIT 0:      00 = 1 V/V
                01 = 2 V/V
                10 = 4 V/V
                11 = 8 V/V

  @Preconditions
    None

  @Param
    conversionMode [ONE_SHOT, CONTINUOUS]
    gain [GAIN_1_VV, GAIN_2_VV, GAIN_4_VV, GAIN_8_VV]
    sampleRate [SAMPLE_RATE_240SPS_12BITS, SAMPLE_RATE_60SPS_14BITS, SAMPLE_RATE_15SPS_16BITS]
    Pointer to MCP3425_type struct
 
  @Returns
    None

  @Example
    <code>
    Update_ADC_REGISTER (CONTINUOUS, GAIN_8_VV, SAMPLE_RATE_15SPS_16BITS, &MCP3425);
    </code>
*/
void Update_ADC_REGISTER (bool conversionMode, unsigned char gain, unsigned char sampleRate, MCP3425_type *MCP3425_buffer);
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