
/*!
LT3966: Functions to interface with LT3966 over Linduino I2C bus.

REVISION HISTORY
$Revision: 0001 $
$Date: 2017-04-18 8:34:50 -0700 (Tues, 18 April 2017) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

#ifndef LT3966_H
#define LT3966_H

#include <stdint.h>
//***************************************************************
// Chip Address Masks
#define CA_READ   0xA1      // Chip Address Read Mask
#define CA_WRITE  0xA0      // Chip Address Write Mask
#define BC_WRITE  0x18      // Broadcast Command Write Mask
#define BC_READ   0x19      // Broadcast Command Read Mask
//***************************************************************
// Global Registers
#define GLBCFG    0x00      // Global Config Reg
//***************************************************************
// Channel 1 Registers  
#define STAT1     0x10      // Status Reg CH1
#define CFG1      0x11      // Config Reg CH1
#define DIM1H     0x12      // PWM Dim Reg1 CH1
#define DIM1L     0x13      // PWM Dim Reg2 CH1
#define ADIM1     0x14      // Analog Dim Reg CH1
//***************************************************************
// Channel 2 Registers
#define STAT2     0x20      // Status Reg CH2
#define CFG2      0x21      // Config Reg CH2
#define DIM2H     0x22      // PWM Dim Reg1 CH2
#define DIM2L     0x23      // PWM Dim Reg2 CH2
#define ADIM2     0x24      // Analog Dim Reg CH2
//***************************************************************
// Channel 3 Registers
#define STAT3     0x30      // Status Reg CH3
#define CFG3      0x31      // Config Reg CH3
#define DIM3H     0x32      // PWM Dim Reg1 CH3
#define DIM3L     0x33      // PWM Dim Reg2 CH3
#define ADIM3     0x34      // Analog Dim Reg CH3
//***************************************************************
// Channel 4  Registers
#define STAT4     0x40      // Status Reg CH4
#define CFG4      0x41      // Config Reg CH4
#define DIM4H     0x42      // PWM Dim Reg1 CH4
#define DIM4L     0x43      // PWM Dim Reg2 CH4
#define ADIM4     0x44      // Analog Dim Reg CH4
//***************************************************************
// Analog to Digital Converter Registers
#define ADCCFG    0x50      // ADC Config Reg
#define VIN       0x51      // Input Voltage Reg
#define TEMP      0x52      // System Temp Reg
#define EXT1      0x53      // External Voltage 1 Reg
#define EXT2      0x54      // External Voltage 2 Reg
#define VFB1      0x55      // Feedback Voltage Reg CH1
#define ILED1     0x56      // LED Current Reg CH1
#define VFB2      0x57      // Feedback Voltage Reg CH2
#define ILED2     0x58      // LED Current Reg CH2
#define VFB3      0x59      // Feedback Voltage Reg CH3
#define ILED3     0x5A      // LED Current Reg CH3
#define VFB4      0x5B      // Feedback Voltage Reg CH4
#define ILED4     0x5C      // LED Current Reg CH4
//***************************************************************
// Part ID Register (Reads "03 96 60")
#define ID0       0xFD      // Part ID Reg High
#define ID1       0xFE      // Part ID Reg Mid
#define ID2       0xFF      // Part ID Reg Low
//***************************************************************
#define CRCPOLY   0x07      // X8+X2+X1+1 Polynomial for CRC


/** @brief Generates a CRC byte (8-bit) for LT3966 Package Error Correction
 *         using X8+X2+X1+1 for calculation
 *
 *  Algorithm for calculating CRC-8 is as follows:
 *    - Initialize shift register with all zeros as contents
 *    - Shift left the message until a 1 comes out
 *    - XOR the contents of the register with the low eight bits of the CRC Polynomial
 *    - Continue until the whole message has passed though the shift register
 *    - Last value out of the shift regeister is the PEC byte. 
 *
 *  @param byteIn   - Previous byte CRC value, or 0 for start of new CRC
 *  @param dataIn   - New byte to be added to CRC calculation
 *  @return     - The generated CRC used to send for PEC or continue calculating CRC
 */
uint8_t genCRC(uint8_t byteIn, uint8_t dataIn);


/** @brief Sends one byte of data to a specified Chip Address / SubAddress with a CRC byte
 *
 *  @param chipAdd  - Chip address specified by bits A3-A0 at pins ADR1 & ARD2
 *  @param subAdd   - Subaddress of the register to be written to
 *  @param data   - Byte of data to be written to the specified register
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_write(uint8_t chipAdd, uint8_t subAdd, uint8_t data);


/** @brief Sends two bytes of data to separate subaddresses with repeated start. Use this for updating DIMH and DIML registers
 *      
 *  @param chipAdd  - Chip address specified by bits A3-A0 at pins ADR1 & ARD2
 *  @param subAdd1  - Subaddress of the first register to be written to
 *  @param data1  - Byte of data to be written to the first specified register
 *  @param subAdd2  - Subaddress of the second register to be written to
 *  @param data2  - Byte of data to be written to the second specified register
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_write_rs(uint8_t chipAdd, uint8_t subAdd1, uint8_t data1, uint8_t subAdd2, uint8_t data2);


/** @brief Reads one byte of data from a specified Chip Address / SubAddress
 *
 *  @param chipAdd  - Chip address specified by bits A3-A0 at pins ADR1 & ARD2
 *  @param subAdd   - Subaddress of the register to be written to
 *  @param *value   - Byte of data read back from LT3966
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_read(uint8_t chipAdd, uint8_t subAdd, uint8_t *data);


/** @brief Sends out Broadcast Read message, and reads back faulted LT3966 address
 *
 *  @param *value   - Address of faulted LT3966
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_broadcast_read(uint8_t *value);


/** @brief Sends out Broadcast Write message, synchronizing PWM dimming of all connected LT3966 devices
 *
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_broadcast_write();

#endif     //LT3966_H 
