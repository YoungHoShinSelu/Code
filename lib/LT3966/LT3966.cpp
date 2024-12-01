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

#include <Arduino.h>
#include <stdint.h>
#include "LT_I2C.h"
#include "Linduino.h"
#include "LT3966.H"


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
uint8_t genCRC(uint8_t byteIn, uint8_t dataIn)
{
  uint8_t crc = 0;                                      // Initialize PEC variable to 0
  crc = byteIn ^ dataIn;                                // XOR existing PEC or 0 (byteIn) and new data (dataIn)
  for(uint8_t i=0; i<8; i++){                           // Step through each bit of the modified value 
    if(crc & 0x80){                                     // Check to see if MSB is HIGH 
      crc <<= 1;                                        // If MSB is HIGH, shift contents up
      crc ^= CRCPOLY;                                   // Then XOR with the crcPoly variable
    }         
    else{                   
      crc <<= 1;                                        // If MSB is LOW, shift contents up
    }         
  }         
return crc;                                             // Once all bits shifted through, return PEC
} 


/** @brief Sends one byte of data to a specified Chip Address / SubAddress with a CRC byte
 *
 *  @param chipAdd  - Chip address specified by bits A3-A0 at pins ADR1 & ARD2
 *  @param subAdd   - Subaddress of the register to be written to
 *  @param data   - Byte of data to be written to the specified register
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_write(uint8_t chipAdd, uint8_t subAdd, uint8_t data)
{
  int8_t ret= 0;
  uint8_t writeCA = 0;      
  uint8_t writeSA = 0;
  uint8_t writeD = 0;
  uint8_t writePEC = 0;
  writeCA = (CA_WRITE|(chipAdd << 1));
  writeSA = subAdd;
  writeD = data;
  writePEC = genCRC(writePEC, writeCA);
  writePEC = genCRC(writePEC, writeSA);
  writePEC = genCRC(writePEC, writeD);
  if (i2c_start()!=0)                                   //I2C START
    return(1);                                          //Stop and return 0 if START fail
  ret |= i2c_write(writeCA);                            //Write Chip Address
  ret |= i2c_write(writeSA);                            //Write SubAddress
  ret |= i2c_write(writeD);                             //Write Data
  ret |= i2c_write(writePEC);                           //Write PEC byte
  i2c_stop();                                           //I2C STOP
  if (ret!=0)                                           // Returns 1 if failed
    return(1);         
  return(0);                                            // Returns 0 if success
}


/** @brief Sends two bytes of data to separate subaddresses. Use this for updating DIMH and DIML registers
 *      
 *  @param chipAdd  - Chip address specified by bits A3-A0 at pins ADR1 & ARD2
 *  @param subAdd1  - Subaddress of the first register to be written to
 *  @param data1  - Byte of data to be written to the first specified register
 *  @param subAdd2  - Subaddress of the second register to be written to
 *  @param data2  - Byte of data to be written to the second specified register
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_write_rs(uint8_t chipAdd, uint8_t subAdd1, uint8_t data1, uint8_t subAdd2, uint8_t data2)
{
  int8_t ret= 0;

  uint8_t calcPEC1 = 0;
  uint8_t calcPEC2 = 0;

  uint8_t writeCA = (CA_WRITE | (chipAdd << 1));

  calcPEC1 = genCRC(calcPEC1, writeCA);
  calcPEC1 = genCRC(calcPEC1, subAdd1);
  calcPEC1 = genCRC(calcPEC1, data1);
  calcPEC2 = genCRC(calcPEC2, writeCA);
  calcPEC2 = genCRC(calcPEC2, subAdd2);
  calcPEC2 = genCRC(calcPEC2, data2);
  if (i2c_start()!=0)                                   //I2C START
    return(1);                                          //Stop and return 0 if START fail
  ret |= i2c_write(writeCA);                  
  ret |= i2c_write(subAdd1);                     
  ret |= i2c_write(data1);
  ret |= i2c_write(calcPEC1);
  if (i2c_repeated_start()!=0)
    return(1);
  ret |= i2c_write(writeCA);                  
  ret |= i2c_write(subAdd2);                     
  ret |= i2c_write(data2);
  ret |= i2c_write(calcPEC2);
  i2c_stop();                                           // I2C STOP
  if (ret!=0)                                           // Returns 1 if failed
    return(1);
  return(0);                                            // Returns 0 if success
}


/** @brief Reads one byte of data from a specified Chip Address / SubAddress
 *
 *  @param chipAdd  - Chip address specified by bits A3-A0 at pins ADR1 & ARD2
 *  @param subAdd   - Subaddress of the register to be written to
 *  @param *value   - Byte of data read back from LT3966
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_read(uint8_t chipAdd, uint8_t subAdd, uint8_t *data)
{
  int8_t ret= 0;                                              

  uint8_t calcPEC = 0;                                  // Variable to store calculated CRC           
                                                                      
  uint8_t writeCA = (CA_WRITE | (chipAdd << 1));        // Combine Chip Address Write Mask with 4-bit address shifted up by 1 bit
  uint8_t readCA = (CA_READ | (chipAdd << 1));          // Combine Chip Address Read Mask with 4-bit address shifted up by 1 bit             
                                                                  
  if (i2c_start() != 0)                                 // Initialize I2C Comunication
    return(1);                                          // Stop and return failure if i2c_start was unsuccesful
  ret |= i2c_write(writeCA);                            // Write Chip Address (Write)
  ret |= i2c_write(subAdd);                             // Write SubAddress
  i2c_stop();                                           // End I2C Communication
  if (i2c_start() != 0)                                 // Re-initialize I2C Comunication
    return(1);                                          // Stop and return failure if i2c_start was unsuccesful
  ret |= i2c_write(readCA);                             // Write Chip Address (Read)
  uint8_t readD = i2c_read(WITH_ACK);                   // Read back data from selected register
  uint8_t readPEC = i2c_read(WITH_ACK);                 // Read back CRC generated by LT3966
  i2c_stop();                                           // End I2C Communication
  *data = readD;                                        // Set pointer to location of data from register
  calcPEC = genCRC(calcPEC, readCA);                    // Calculate CRC based off what was sent (first byte)
  calcPEC = genCRC(calcPEC, readD);                     // Calculate CRC based off what was sent (second byte)
  if(calcPEC != readPEC)                                // Check to see if recieved CRC matches calculated CRC
    return(1);                                          // Returns 1 if CRC bytes don't match
  if (ret != 0)                                         // Check ret variable
    return(1);                                          // Returns 1 if failure
  return(0);                                            // Returns 0 if success
}





/** @brief Sends out Broadcast Read message, and reads back faulted LT3966 address
 *
 *  @param *value   - Address of faulted LT3966
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_broadcast_read(uint8_t *value)
{
    int8_t ret = 0;
    if(i2c_start() != 0)
        return(1);
    ret |= i2c_write(BC_READ);
    if(ret != 0) {
        i2c_stop();
        return(1);
    }
    *value = i2c_read(WITH_ACK);  // Store directly in output parameter
    i2c_stop();
    if(ret != 0)
        return(1);
    return(0);
}


/** @brief Sends out Broadcast Write message, synchronizing PWM dimming of all connected LT3966 devices
 *
 *  @return     - Returns 0 for success / 1 for failure
 */
int8_t lt3966_i2c_broadcast_write()
{
  int8_t ret = 0;
  if(i2c_start() != 0)
    return(1);
  ret |= i2c_write(BC_WRITE);
  i2c_stop();
  if(ret != 0)
    return(1);
  return(0);
}
