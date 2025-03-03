
/**************************************************************
*** Configuration File generated automatically by           ***
*** SILICON LABS's Wireless Development Suite               ***
*** WDS GUI Version: 3.2.11.0                                    ***
*** Device: Si4032 Rev.: B1                                 ***
***                                                         ***
***                                                         ***
*** For further details please consult with the device      ***
*** datasheet, available at <http://www.silabs.com>         ***
***************************************************************/

/**************** Frequency Control ********************

        Operating Frequency: 431.050 MHz
 *******************************************************/

#define Si4032_CRYSTAL_OSCILLATOR_LOAD_CAPACITANCE 0x76
/*
        Address:                0x09
        Crystal Oscillator Load Capacitance: 11.742 pF
*/

#define Si4032_FREQUENCY_OFFSET_1                  0x00
/*
        Address:                0x73
*/

#define Si4032_FREQUENCY_OFFSET_2                  0x00
/*
        Address:                0x74
*/

#define Si4032_FREQUENCY_BAND                      0x53
/*
        Address:                0x75
*/

#define Si4032_NOMINAL_CARRIER_FREQUENCY_1         0x1A
/*
        Address:                0x76
*/

#define Si4032_NOMINAL_CARRIER_FREQUENCY_0         0x40
/*
        Address:                0x77
*/

#define Si4032_FREQUENCY_HOPPING_STEP_SIZE         0x00
/*
        Address:                0x7a
        If frequency hopping is used then the step size should 
        be set first, and then the hopping channel, because the 
        frequency change occurs when the channel number is set.
*/

#define Si4032_FREQUENCY_HOPPING_CHANNEL           0x00
/*
        Address:                0x79
        If frequency hopping is used then the step size should 
        be set first, and then the hopping channel, because the 
        frequency change occurs when the channel number is set.
*/

/**************** TX Modulation Options ****************/

#define Si4032_TX_POWER                            0x18
/*
        Address:                0x6d
*/

#define Si4032_TX_DATA_RATE_1                      0x08
/*
        Address:                0x6e
*/

#define Si4032_TX_DATA_RATE_0                      0x31
/*
        Address:                0x6f
*/

#define Si4032_TX_DR_IN_BPS 1000L

#define Si4032_MODULATION_MODE_CONTROL_1           0x2C
/*
        Address:                0x70
*/

#define Si4032_MODULATION_MODE_CONTROL_2           0x12
/*
        Address:                0x71
Data Clock Config.: No TX Data Clock (only for OOK and FSK) 
Data Source:        Direct Mode using TX_Data via SDI pin
Modulation Type:    FSK

*/

#define Si4032_FREQUENCY_DEVIATION                 0x01
/*
        Address:                0x72
        Deviation: 0.625 kHz
*/

/**************** Operation mode ***********************/

#define Si4032_OPERATING_AND_FUNCTION_CONTROL_1    0x08
/*
        Address:                0x07
        Operation Mode: Tx
*/

/**************** End of Header file *******************/

