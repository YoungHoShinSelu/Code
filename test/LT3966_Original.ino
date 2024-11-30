#include "LT3966.h"
#include <Arduino.h>
#include <stdint.h>
#include <util/delay.h>
#include "Linduino.h"
#include "LT_I2C.h"
#include <Wire.h>

#define ALERT_PIN   12
#define LT3966_ADD  0b1111

#define B7      0x80
#define B6      0x40
#define B5      0x20
#define B4      0x10
#define B3      0x08
#define B2      0x04
#define B1      0x02
#define B0      0x01

// Function prototypes
long readTerminal();
uint8_t readTerminalDecimal();
float readfloatTerminal();
uint8_t readTerminalBinary();
uint16_t calcDuty (float percentage);
uint8_t calcAnalog( float percentage);
void printPaddedBinary(uint8_t value);
void cycleColor(void);

void setup()
{
  Wire.begin();
  pinMode(ALERT_PIN, INPUT);

  lt3966_i2c_write(LT3966_ADD, STAT1, 0b11110000);      // Set up CH1 with all fault reporting enabled
  lt3966_i2c_write(LT3966_ADD, STAT2, 0b11110000);      // Set up CH2 with all fault reporting enabled
  lt3966_i2c_write(LT3966_ADD, STAT3, 0b11110000);      // Set up CH3 with all fault reporting enabled
  lt3966_i2c_write(LT3966_ADD, STAT4, 0b11110000);      // Set up CH4 with all fault reporting enabled

  Serial.begin(9600);

  Serial.println(F("*** LT3966 Command Line Interface ***\n"));
}

void loop()
{
  int input1 = 0;
  char input2 = 0;
  uint8_t input3 = 0;
  uint8_t input32 = 0;
  uint8_t input4 = 0;
  char input5 = 0;
  int input6 = 0;
  int input62 = 0;
  int input6dim = 0;
  float n_period = 0;
  int DIM = 0;
  float duty_max = 0;
  uint8_t readpwm1 = 0;
  int readpwm2 = 0;
  uint8_t regVal = 0;
  uint8_t regVal2 = 0;
  uint8_t startFlag = 0;

  Serial.println(F("\nType associated # followed by enter to select a register to access:"));
  Serial.println(F("  (1) Global Configuration Register "));
  Serial.println(F("  (2) Status Register "));
  Serial.println(F("  (3) Channel Configuration Register "));
  Serial.println(F("  (4) PWM Dimming Register "));
  Serial.println(F("  (5) Analog Dimming Register "));
  Serial.println(F("  (6) Analog to Digital Converter Register Read"));
  Serial.println(F("  (7) Check ALERT"));
  Serial.println(F("  (8) Broadcast Mode "));
  Serial.println(F("  (9) Color Cycle \n\n"));

  input1 = readTerminal();
  Serial.print(F("You Entered: "));
  Serial.println((int)input1);

  if(((int)input1 != 7) && ((int)input1 != 6))
  {
    Serial.println(F("\nPerform (R)ead or (W)rite?\n"));  
    while (Serial.available() == 0);
    input2 = Serial.read();
    input2 = toupper(input2); // Convert input to uppercase
    
    // Clear any remaining characters in the buffer
    delay(10); // Short delay to allow any remaining characters to arrive
    while (Serial.available() > 0)
    {
      Serial.read();
    }
    
    Serial.print(F("You Entered: "));
    Serial.println(input2);
    Serial.print("\n");
  }

  Serial.print("input2 = '");
  Serial.print(input2);
  Serial.print("' (ASCII ");
  Serial.print((int)input2);
  Serial.println(")");

  switch((int)input1) 
  {
    case 1:     // Global Configuration Reg
      Serial.println(F("Global Configuration Register"));
      if(input2 == 'W')
      { 
        Serial.print(F("\nEnable Watchdog Timer (WDTEN)? (Y/N): "));
        while ( Serial.available() == 0);
        input5 = Serial.read();
        Serial.print(input5);
        if(input5 == 'Y')
          input3 = input3 | B6;
        else if(input5 == 'N')
          input3 = input3 & !B6;
        else 
          break;

        Serial.print(F("\nEnable Multiphase (MPHASE)? (Y/N): "));
        while ( Serial.available() == 0);
        input5 = Serial.read();
        Serial.print(input5);
        if(input5 == 'Y')
          input3 = input3 | B5;
        else if(input5 == 'N')
          input3 = input3 & ~B5;
        else 
          break;

        Serial.print(F("\nSet SYNC as output (CLKOUT)? (Y/N): "));
        while ( Serial.available() == 0);
        input5 = Serial.read();
        Serial.print(input5);
        if(input5 == 'Y')
          input3 = input3 | B4;
        else if(input5 == 'N')
          input3 = input3 & ~B4;
        else 
          break;

        Serial.print(F("\nEnable Channel 1 (OFF0)? (Y/N): "));
        while ( Serial.available() == 0);
        input5 = Serial.read();
        Serial.print(input5);
        if(input5 == 'Y')
          input3 = input3 & ~B0;
        else if(input5 == 'N')
          input3 = input3 | B0;
        else 
          break;

        Serial.print(F("\nEnable Channel 2 (OFF1)? (Y/N): "));
        while ( Serial.available() == 0);
        input5 = Serial.read();
        Serial.print(input5);
        if(input5 == 'Y')
          input3 = input3 & ~B1;
        else if(input5 == 'N')
          input3 = input3 | B1;
        else 
          break;

        Serial.print(F("\nEnable Channel 3 (OFF2)? (Y/N): "));
        while ( Serial.available() == 0);
        input5 = Serial.read();
        Serial.print(input5);
        if(input5 == 'Y')
          input3 = input3 & ~B2;
        else if(input5 == 'N')
          input3 = input3 | B2;
        else 
          break;

        Serial.print(F("\nEnable Channel 4 (OFF3)? (Y/N): "));
        while ( Serial.available() == 0);
        input5 = Serial.read();
        Serial.print(input5);
        if(input5 == 'Y')
          input3 = input3 & ~B3;
        else if(input5 == 'N')
          input3 = input3 | B3;
        else 
          break;
        
        lt3966_i2c_write(LT3966_ADD, GLBCFG, input3);
        Serial.println("\n");
        Serial.print(F("Global Configuration Register (GLBCFG) Updated: "));
        printPaddedBinary(input3);
        Serial.print("\n");
      }
      else if(input2 == 'R')
      {
        lt3966_i2c_read(LT3966_ADD, GLBCFG, &regVal);
        Serial.print(F("\nGlobal Configuration Register (GLBCFG) Contents: "));
        printPaddedBinary(regVal);

        Serial.print(F("\nWatchdog Timer Status (WDTFLAG): "));
        if((regVal&B7) == B7)
          Serial.println(F("1"));
        else
          Serial.println(F("0"));

        Serial.print(F("Watchdog Timer (WDTEN):"));
        if((regVal&B6) == B6)
          Serial.println(F("Enabled"));
        else
          Serial.println(F("Disabled"));

        Serial.print(F("Multiphase DC/DC Converter Clocking: "));
        if((regVal&B5) == B5)
          Serial.println(F("Enabled"));
        else
          Serial.println(F("Disabled"));

        Serial.print(F("SYNC Pin Direction: "));
        if((regVal&B4) == B4)
          Serial.println(F("Output"));
        else
          Serial.println(F("Input"));

        Serial.print(F("Channel 1: "));
        if((regVal&B0) == B0)
          Serial.println(F("Disabled"));
        else
          Serial.println(F("Enabled"));

        Serial.print(F("Channel 2: "));
        if((regVal&B1) == B1)
          Serial.println(F("Disabled"));
        else
          Serial.println(F("Enabled"));

        Serial.print(F("Channel 3: "));
        if((regVal&B2) == B2)
          Serial.println(F("Disabled"));
        else
          Serial.println(F("Enabled"));

        Serial.print(F("Channel 4: "));
        if((regVal&B3) == B3)
          Serial.println(F("Disabled"));
        else
          Serial.println(F("Enabled"));
      }
      else
      {
        Serial.println(F("\n***********Invalid input***********"));
      }
      break;

    case 2:     // Status Reg
      Serial.println(F("\nStatus Register"));
      Serial.println(F("\nWhich Channel Register would you like access? (1)(2)(3)(4)\n"));
      input4 = readTerminalDecimal();
      Serial.print(F("You Entered: "));
      Serial.println(input4);

      switch(input4)
      {
        case 1: //CH1
          if(input2 == 'W')
          {
            Serial.print(F("\nLED Overcurrent Reporting Enable (OC_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B7;
            else if(input5 == 'N')
              input3 = input3 & !B7;
            else 
              break;
            
            Serial.print(F("\nShorted LED Reporting Enable (SHORT_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B6;
            else if(input5 == 'N')
              input3 = input3 & !B6;
            else 
              break;
    
            Serial.print(F("\nOpen LED Reporting Enable (OPEN_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B5;
            else if(input5 == 'N')
              input3 = input3 & ~B5;
            else 
              break;
    
            Serial.print(F("\nFB Overvoltage Reporting Enable (OVFB_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B4;
            else if(input5 == 'N')
              input3 = input3 & ~B4;
            else 
              break;
    
            lt3966_i2c_write(LT3966_ADD, STAT1, input3);
            Serial.println(F("\n"));
            Serial.print(F("Channel 1 Status Register (STAT1) Updated: "));
            printPaddedBinary(input3);
            Serial.print("\n");
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, STAT1, &regVal);
            Serial.print(F("\nChannel 1 Status Register (STAT1) Contents: "));
            printPaddedBinary(regVal);

            Serial.print(F("\nLED Overcurrent Reporting (OC_EN): "));
            if((regVal&B7) == B7)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("Shorted LED Reporting (SHORT_EN): "));
            if((regVal&B6) == B6)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("Open LED Reporting (OPEN_EN): "));
            if((regVal&B5) == B5)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("FB Overvoltage Reporting (OVFB_EN): "));
            if((regVal&B4) == B4)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("LED Overcurrent Status: "));
            if((regVal&B7) != B7)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B3) == B3)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("Shorted LED Status: "));
            if((regVal&B6) != B6)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B2) == B2)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("Open LED Status: "));
            if((regVal&B5) != B5)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B1) == B1)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("FB Overvoltage Status: "));
            if((regVal&B4) != B4)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B0) == B0)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }         
          break;

        case 2: // CH2
          if(input2 == 'W')
          {
            Serial.print(F("\nLED Overcurrent Reporting Enable (OC_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B7;
            else if(input5 == 'N')
              input3 = input3 & !B7;
            else 
              break;
            
            Serial.print(F("\nShorted LED Reporting Enable (SHORT_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B6;
            else if(input5 == 'N')
              input3 = input3 & !B6;
            else 
              break;
    
            Serial.print(F("\nOpen LED Reporting Enable (OPEN_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B5;
            else if(input5 == 'N')
              input3 = input3 & ~B5;
            else 
              break;
    
            Serial.print(F("\nFB Overvoltage Reporting Enable (OVFB_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B4;
            else if(input5 == 'N')
              input3 = input3 & ~B4;
            else 
              break;
    
            lt3966_i2c_write(LT3966_ADD, STAT2, input3);
            Serial.println(F("\n"));
            Serial.print(F("Channel 2 Status Register (STAT2) Updated: "));
            printPaddedBinary(input3);
            Serial.print("\n");
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, STAT2, &regVal);
            Serial.print(F("\nChannel 2 Status Register (STAT2) Contents: "));
            printPaddedBinary(regVal);

            Serial.print(F("\nLED Overcurrent Reporting (OC_EN): "));
            if((regVal&B7) == B7)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("Shorted LED Reporting (SHORT_EN): "));
            if((regVal&B6) == B6)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("Open LED Reporting (OPEN_EN): "));
            if((regVal&B5) == B5)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("FB Overvoltage Reporting (OVFB_EN): "));
            if((regVal&B4) == B4)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("LED Overcurrent Status: "));
            if((regVal&B7) != B7)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B3) == B3)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("Shorted LED Status: "));
            if((regVal&B6) != B6)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B2) == B2)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("Open LED Status: "));
            if((regVal&B5) != B5)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B1) == B1)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("FB Overvoltage Status: "));
            if((regVal&B4) != B4)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B0) == B0)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        case 3: // CH3
          if(input2 == 'W')
          {
            Serial.print(F("\nLED Overcurrent Reporting Enable (OC_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B7;
            else if(input5 == 'N')
              input3 = input3 & !B7;
            else 
              break;
            
            Serial.print(F("\nShorted LED Reporting Enable (SHORT_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B6;
            else if(input5 == 'N')
              input3 = input3 & !B6;
            else 
              break;
    
            Serial.print(F("\nOpen LED Reporting Enable (OPEN_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B5;
            else if(input5 == 'N')
              input3 = input3 & ~B5;
            else 
              break;
    
            Serial.print(F("\nFB Overvoltage Reporting Enable (OVFB_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B4;
            else if(input5 == 'N')
              input3 = input3 & ~B4;
            else 
              break;
    
            lt3966_i2c_write(LT3966_ADD, STAT3, input3);
            Serial.println(F("\n"));
            Serial.print(F("Channel 3 Status Register (STAT3) Updated: "));
            printPaddedBinary(input3);
            Serial.print("\n");
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, STAT3, &regVal);
            Serial.print(F("\nChannel 3 Status Register (STAT3) Contents: "));
            printPaddedBinary(regVal);

            Serial.print(F("\nLED Overcurrent Reporting (OC_EN): "));
            if((regVal&B7) == B7)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("Shorted LED Reporting (SHORT_EN): "));
            if((regVal&B6) == B6)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("Open LED Reporting (OPEN_EN): "));
            if((regVal&B5) == B5)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("FB Overvoltage Reporting (OVFB_EN): "));
            if((regVal&B4) == B4)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("LED Overcurrent Status: "));
            if((regVal&B7) != B7)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B3) == B3)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("Shorted LED Status: "));
            if((regVal&B6) != B6)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B2) == B2)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("Open LED Status: "));
            if((regVal&B5) != B5)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B1) == B1)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("FB Overvoltage Status: "));
            if((regVal&B4) != B4)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B0) == B0)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        case 4: // CH4
          if(input2 == 'W')
          {
            Serial.print(F("\nLED Overcurrent Reporting Enable (OC_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B7;
            else if(input5 == 'N')
              input3 = input3 & !B7;
            else 
              break;
            
            Serial.print(F("\nShorted LED Reporting Enable (SHORT_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B6;
            else if(input5 == 'N')
              input3 = input3 & !B6;
            else 
              break;
    
            Serial.print(F("\nOpen LED Reporting Enable (OPEN_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B5;
            else if(input5 == 'N')
              input3 = input3 & ~B5;
            else 
              break;
    
            Serial.print(F("\nFB Overvoltage Reporting Enable (OVFB_EN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B4;
            else if(input5 == 'N')
              input3 = input3 & ~B4;
            else 
              break;
    
            lt3966_i2c_write(LT3966_ADD, STAT4, input3);
            Serial.println(F("\n"));
            Serial.print(F("Channel 4 Status Register (STAT4) Updated: "));
            printPaddedBinary(input3);
            Serial.print("\n");
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, STAT4, &regVal);
            Serial.print(F("\nChannel 4 Status Register (STAT4) Contents: "));
            printPaddedBinary(regVal);

            Serial.print(F("\nLED Overcurrent Reporting (OC_EN): "));
            if((regVal&B7) == B7)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("Shorted LED Reporting (SHORT_EN): "));
            if((regVal&B6) == B6)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("Open LED Reporting (OPEN_EN): "));
            if((regVal&B5) == B5)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("FB Overvoltage Reporting (OVFB_EN): "));
            if((regVal&B4) == B4)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
    
            Serial.print(F("LED Overcurrent Status: "));
            if((regVal&B7) != B7)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B3) == B3)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("Shorted LED Status: "));
            if((regVal&B6) != B6)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B2) == B2)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("Open LED Status: "));
            if((regVal&B5) != B5)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B1) == B1)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));

            Serial.print(F("FB Overvoltage Status: "));
            if((regVal&B4) != B4)
              Serial.println(F("Reporting Disabled"));
            else if((regVal&B0) == B0)
              Serial.println(F("HIGH"));
            else
              Serial.println(F("LOW"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        default:
          Serial.println(F("\n***********Invalid input***********"));
          break;
      }
      break;

    case 3:     // Channel Config Reg
      Serial.println(F("\nChannel Configuration Register"));
      Serial.println(F("\nWhich Channel Register would you like access? (1)(2)(3)(4)\n"));
      input4 = readTerminalDecimal();
      Serial.print(F("You Entered: "));
      Serial.println(input4);
      Serial.print("\n");
      switch(input4)
      {
        case 1: //CH1
          if(input2 == 'W')
          {
            Serial.print(F("\nIn Phase Mode Select (INPH)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B3;
            else if(input5 == 'N')
              input3 = input3 & !B3;
            else 
              break;
            
            Serial.print(F("\nLatchoff Mode (LATOFF)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B2;
            else if(input5 == 'N')
              input3 = input3 & !B2;
            else 
              break;
    
            Serial.print(F("\nBit Angle Modulation Select (BAM)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B1;
            else if(input5 == 'N')
              input3 = input3 & ~B1;
            else 
              break;
    
            Serial.print(F("\nDimming Generator Enable (DIMEN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B0;
            else if(input5 == 'N')
              input3 = input3 & ~B0;
            else 
              break;
              
            lt3966_i2c_write(LT3966_ADD, CFG1, input3);
            Serial.println(F("\n"));
            Serial.println(F("Channel 1 Configuration Register (CFG1) Updated: "));
            printPaddedBinary(input3);
            Serial.print(F("\n"));
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, CFG1, &regVal);
            Serial.print(F("Channel 1 Configuration Register (CFG1) Contents: "));
            printPaddedBinary(regVal);

            Serial.print(F("\nIn Phase Mode Select (INPH): "));
            if((regVal&B3) == B3)
              Serial.println(F("In Phase"));
            else
              Serial.println(F("Out of Phase"));
    
            Serial.print(F("Latchoff Mode (LATOFF): "));
            if((regVal&B2) == B2)
              Serial.println(F("Set"));
            else
              Serial.println(F("Clear"));
    
            Serial.print(F("Bit Angle Modulation Select (BAM): "));
            if((regVal&B1) == B1)
              Serial.println(F("On (Bit-Angle-Modulation)"));
            else
              Serial.println(F("Off (PWM)"));
    
            Serial.print(F("Dimming Generator (DIMEN): "));
            if((regVal&B0) == B0)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
            
            Serial.print(F("\n"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }         
          break;

        case 2: // CH2
          if(input2 == 'W')
          {

            Serial.print(F("\nIn Phase Mode Select (INPH)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B3;
            else if(input5 == 'N')
              input3 = input3 & !B3;
            else 
              break;
            
            Serial.print(F("\nLatchoff Mode (LATOFF)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B2;
            else if(input5 == 'N')
              input3 = input3 & !B2;
            else 
              break;
    
            Serial.print(F("\nBit Angle Modulation Select (BAM)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B1;
            else if(input5 == 'N')
              input3 = input3 & ~B1;
            else 
              break;
    
            Serial.print(F("\nDimming Generator Enable (DIMEN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B0;
            else if(input5 == 'N')
              input3 = input3 & ~B0;
            else 
              break;
              
            lt3966_i2c_write(LT3966_ADD, CFG2, input3);
            Serial.println(F("\n"));
            Serial.println(F("Channel 2 Configuration Register (CFG2) Updated: "));
            printPaddedBinary(input3);
            Serial.print(F("\n"));
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, CFG2, &regVal);
            Serial.print(F("Channel 2 Configuration Register (CFG2) Contents: "));
            printPaddedBinary(regVal);

            Serial.print(F("\nIn Phase Mode Select (INPH): "));
            if((regVal&B3) == B3)
              Serial.println(F("In Phase"));
            else
              Serial.println(F("Out of Phase"));
    
            Serial.print(F("Latchoff Mode (LATOFF): "));
            if((regVal&B2) == B2)
              Serial.println(F("Set"));
            else
              Serial.println(F("Clear"));
    
            Serial.print(F("Bit Angle Modulation Select (BAM): "));
            if((regVal&B1) == B1)
              Serial.println(F("On (Bit-Angle-Modulation)"));
            else
              Serial.println(F("Off (PWM)"));
    
            Serial.print(F("Dimming Generator (DIMEN): "));
            if((regVal&B0) == B0)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
            
            Serial.print(F("\n"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        case 3: // CH3
          if(input2 == 'W')
          {
            Serial.print(F("\nIn Phase Mode Select (INPH)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B3;
            else if(input5 == 'N')
              input3 = input3 & !B3;
            else 
              break;
            
            Serial.print(F("\nLatchoff Mode (LATOFF)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B2;
            else if(input5 == 'N')
              input3 = input3 & !B2;
            else 
              break;
    
            Serial.print(F("\nBit Angle Modulation Select (BAM)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B1;
            else if(input5 == 'N')
              input3 = input3 & ~B1;
            else 
              break;
    
            Serial.print(F("\nDimming Generator Enable (DIMEN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B0;
            else if(input5 == 'N')
              input3 = input3 & ~B0;
            else 
              break;
              
            lt3966_i2c_write(LT3966_ADD, CFG3, input3);
            Serial.println(F("\n"));
            Serial.println(F("Channel 3 Configuration Register (CFG3) Updated: "));
            printPaddedBinary(input3);
            Serial.print(F("\n"));
          }
          else if(input2 == 'R')
          {            
            lt3966_i2c_read(LT3966_ADD, CFG3, &regVal);
            Serial.print(F("Channel 3 Configuration Register (CFG3) Contents: "));
            printPaddedBinary(regVal);

            Serial.print(F("\nIn Phase Mode Select (INPH): "));
            if((regVal&B3) == B3)
              Serial.println(F("In Phase"));
            else
              Serial.println(F("Out of Phase"));
    
            Serial.print(F("Latchoff Mode (LATOFF): "));
            if((regVal&B2) == B2)
              Serial.println(F("Set"));
            else
              Serial.println(F("Clear"));
    
            Serial.print(F("Bit Angle Modulation Select (BAM): "));
            if((regVal&B1) == B1)
              Serial.println(F("On (Bit-Angle-Modulation)"));
            else
              Serial.println(F("Off (PWM)"));
    
            Serial.print(F("Dimming Generator (DIMEN): "));
            if((regVal&B0) == B0)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
            
            Serial.print(F("\n"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        case 4: // CH4
          if(input2 == 'W')
          {
            Serial.print(F("\nIn Phase Mode Select (INPH)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B3;
            else if(input5 == 'N')
              input3 = input3 & !B3;
            else 
              break;
            
            Serial.print(F("\nLatchoff Mode (LATOFF)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B2;
            else if(input5 == 'N')
              input3 = input3 & !B2;
            else 
              break;
    
            Serial.print(F("\nBit Angle Modulation Select (BAM)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B1;
            else if(input5 == 'N')
              input3 = input3 & ~B1;
            else 
              break;
    
            Serial.print(F("\nDimming Generator Enable (DIMEN)? (Y/N): "));
            while ( Serial.available() == 0);
            input5 = Serial.read();
            Serial.print(input5);
            if(input5 == 'Y')
              input3 = input3 | B0;
            else if(input5 == 'N')
              input3 = input3 & ~B0;
            else 
              break;
              
            lt3966_i2c_write(LT3966_ADD, CFG4, input3);
            Serial.println(F("\n"));
            Serial.println(F("Channel 4 Configuration Register (CFG4) Updated: "));
            printPaddedBinary(input3);
            Serial.print(F("\n"));
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, CFG4, &regVal);
            Serial.print(F("Channel 4 Configuration Register (CFG4) Contents: "));
            printPaddedBinary(regVal);

            Serial.print(F("\nIn Phase Mode Select (INPH): "));
            if((regVal&B3) == B3)
              Serial.println(F("In Phase"));
            else
              Serial.println(F("Out of Phase"));
    
            Serial.print(F("Latchoff Mode (LATOFF): "));
            if((regVal&B2) == B2)
              Serial.println(F("Set"));
            else
              Serial.println(F("Clear"));
    
            Serial.print(F("Bit Angle Modulation Select (BAM): "));
            if((regVal&B1) == B1)
              Serial.println(F("On (Bit-Angle-Modulation)"));
            else
              Serial.println(F("Off (PWM)"));
    
            Serial.print(F("Dimming Generator (DIMEN): "));
            if((regVal&B0) == B0)
              Serial.println(F("Enabled"));
            else
              Serial.println(F("Disabled"));
            
            Serial.print(F("\n"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        default:
          Serial.println(F("\n***********Invalid input***********"));
          break;
      }
      break;

    case 4:     // PWM Dim Reg
      Serial.println(F("\nPWM Dimming Register"));
      Serial.println(F("\nWhich Channel Register would you like access? (1)(2)(3)(4)\n"));
      input4 = readTerminalDecimal();
      Serial.print(F("You Entered: "));
      Serial.println(input4);
      Serial.print("\n");

      // Debug print to check input4
      Serial.print("input4 = ");
      Serial.println(input4);

      switch(input4)
      {
        case 1: //CH1
          if(input2 == 'W')
          {
            Serial.print(F("Choose PWM Cycle Length Select (SCL) (0-7): "));
            input6 = readTerminal();
            Serial.print(input6);
            n_period = pow(2,6 + input6);                                                       // n = 2^(6+SEL)
            duty_max = (n_period-1)/n_period;                                                   // duty_max = (n-1)/n
            Serial.print(F("\n"));
            Serial.print(F("Choose PWM Duty Cycle % (DIM) (0-"));
            Serial.print((duty_max*100.0));
            Serial.print("): ");
            input62 = readTerminal();
            Serial.print(input62);

            // LT3966 ignores LSBs depending on value of SEL. This shifts DIM bits left by 7 - SEL.
            input6dim = ((int)((((float)input62)/100.0) * n_period) << (7 - input6));             // DIM = Duty Cycle * n

            input3 = (input6 << 5)|((input6dim & 0x001F));
            input32 = (input6dim >> 5);
            Serial.print(F("\nDIMH: "));
            printPaddedBinary(input3);
            Serial.print(F("\nDIML: "));
            printPaddedBinary(input32);
            Serial.print(F("\n"));
            lt3966_i2c_write_rs(LT3966_ADD, DIM1H, input3, DIM1L, input32);
            Serial.println(F("\nChannel 1 PWM Dim Value Registers Updated!"));
          }
          else if(input2 == 'R')
          {
            Serial.print(F("Channel 1 PWM Dim Value Registers Contents: "));
            lt3966_i2c_read(LT3966_ADD, DIM1H, &regVal);
            lt3966_i2c_read(LT3966_ADD, DIM1L, &regVal2);
            printPaddedBinary(regVal);
            Serial.print(F(" "));
            printPaddedBinary(regVal2);
            Serial.print(F("\n"));

            readpwm1 = regVal >> 5;
            // This shifts DIM bits right by 7 - SEL to have the correct DIM value for reading.
            readpwm2 = (int)((regVal&0x1F)) + ((int)regVal2*32) >> (7 - int(readpwm1));
            Serial.print(F("PWM Cycle Length Select: "));
            Serial.println((int)readpwm1);
            n_period = pow(2,6 + (int)readpwm1);                                  // n = 2^(6+SEL)
            Serial.print(F("PWM Duty Cycle (%): "));
            Serial.println(((float)readpwm2)/n_period*100);                       // Duty Cycle = DIM/n * 100%
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }         
          break;

        case 2: // CH2
          if(input2 == 'W')
          {
            Serial.print(F("Choose PWM Cycle Length Select (SCL) (0-7): "));
            input6 = readTerminal();
            Serial.print(input6);
            n_period = pow(2,6 + input6);                                                       // n = 2^(6+SEL)
            duty_max = (n_period-1)/n_period;                                                   // duty_max = (n-1)/n
            Serial.print(F("\n"));
            Serial.print(F("Choose PWM Duty Cycle % (DIM) (0-"));
            Serial.print((duty_max*100.0));
            Serial.print("): ");
            input62 = readTerminal();
            Serial.print(input62);

            // LT3966 ignores LSBs depending on value of SEL. This shifts DIM bits left by 7 - SEL.
            input6dim = ((int)((((float)input62)/100.0) * n_period) << (7 - input6));             // DIM = Duty Cycle * n

            input3 = (input6 << 5)|((input6dim & 0x001F));
            input32 = (input6dim >> 5);
            Serial.print(F("\nDIMH: "));
            printPaddedBinary(input3);
            Serial.print(F("\nDIML: "));
            printPaddedBinary(input32);
            Serial.print(F("\n"));
            lt3966_i2c_write_rs(LT3966_ADD, DIM2H, input3, DIM2L, input32);
            Serial.println(F("\nChannel 2 PWM Dim Value Registers Updated!"));
          }
          else if(input2 == 'R')
          {
            Serial.print(F("Channel 2 PWM Dim Value Registers Contents: "));
            lt3966_i2c_read(LT3966_ADD, DIM2H, &regVal);
            lt3966_i2c_read(LT3966_ADD, DIM2L, &regVal2);
            printPaddedBinary(regVal);
            Serial.print(F(" "));
            printPaddedBinary(regVal2);
            Serial.print(F("\n"));

            readpwm1 = regVal >> 5;
            // This shifts DIM bits right by 7 - SEL to have the correct DIM value for reading.
            readpwm2 = (int)((regVal&0x1F)) + ((int)regVal2*32) >> (7 - int(readpwm1));
            Serial.print(F("PWM Cycle Length Select: "));
            Serial.println((int)readpwm1);
            n_period = pow(2,6 + (int)readpwm1);                                  // n = 2^(6+SEL)
            Serial.print(F("PWM Duty Cycle (%): "));
            Serial.println(((float)readpwm2)/n_period*100);                       // Duty Cycle = DIM/n * 100%
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        case 3: // CH3
          if(input2 == 'W')
          { 
            Serial.print(F("Choose PWM Cycle Length Select (SCL) (0-7): "));
            input6 = readTerminal();
            Serial.print(input6);
            n_period = pow(2,6 + input6);                                                       // n = 2^(6+SEL)
            duty_max = (n_period-1)/n_period;                                                   // duty_max = (n-1)/n
            Serial.print(F("\n"));
            Serial.print(F("Choose PWM Duty Cycle % (DIM) (0-"));
            Serial.print((duty_max*100.0));
            Serial.print("): ");
            input62 = readTerminal();
            Serial.print(input62);

            // LT3966 ignores LSBs depending on value of SEL. This shifts DIM bits left by 7 - SEL.
            input6dim = ((int)((((float)input62)/100.0) * n_period) << (7 - input6));             // DIM = Duty Cycle * n

            input3 = (input6 << 5)|((input6dim & 0x001F));
            input32 = (input6dim >> 5);
            Serial.print(F("\nDIMH: "));
            printPaddedBinary(input3);
            Serial.print(F("\nDIML: "));
            printPaddedBinary(input32);
            Serial.print(F("\n"));
            lt3966_i2c_write_rs(LT3966_ADD, DIM3H, input3, DIM3L, input32);
            Serial.println(F("\nChannel 3 PWM Dim Value Registers Updated!"));
          }
          else if(input2 == 'R')
          {
            Serial.print(F("Channel 3 PWM Dim Value Registers Contents: "));
            lt3966_i2c_read(LT3966_ADD, DIM3H, &regVal);
            lt3966_i2c_read(LT3966_ADD, DIM3L, &regVal2);
            printPaddedBinary(regVal);
            Serial.print(F(" "));
            printPaddedBinary(regVal2);
            Serial.print(F("\n"));

            readpwm1 = regVal >> 5;
            // This shifts DIM bits right by 7 - SEL to have the correct DIM value for reading.
            readpwm2 = (int)((regVal&0x1F)) + ((int)regVal2*32) >> (7 - int(readpwm1));
            Serial.print(F("PWM Cycle Length Select: "));
            Serial.println((int)readpwm1);
            n_period = pow(2,6 + (int)readpwm1);                                  // n = 2^(6+SEL)
            Serial.print(F("PWM Duty Cycle (%): "));
            Serial.println(((float)readpwm2)/n_period*100);                       // Duty Cycle = DIM/n * 100%
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        case 4: // CH4
          if(input2 == 'W')
          {
            Serial.print(F("Choose PWM Cycle Length Select (SCL) (0-7): "));
            input6 = readTerminal();
            Serial.print(input6);
            n_period = pow(2,6 + input6);                                                       // n = 2^(6+SEL)
            duty_max = (n_period-1)/n_period;                                                   // duty_max = (n-1)/n
            Serial.print(F("\n"));
            Serial.print(F("Choose PWM Duty Cycle % (DIM) (0-"));
            Serial.print((duty_max*100.0));
            Serial.print("): ");
            input62 = readTerminal();
            Serial.print(input62);

            // LT3966 ignores LSBs depending on value of SEL. This shifts DIM bits left by 7 - SEL.
            input6dim = ((int)((((float)input62)/100.0) * n_period) << (7 - input6));             // DIM = Duty Cycle * n

            input3 = (input6 << 5)|((input6dim & 0x001F));
            input32 = (input6dim >> 5);
            Serial.print(F("\nDIMH: "));
            printPaddedBinary(input3);
            Serial.print(F("\nDIML: "));
            printPaddedBinary(input32);
            Serial.print(F("\n"));
            lt3966_i2c_write_rs(LT3966_ADD, DIM4H, input3, DIM4L, input32);
            Serial.println(F("\nChannel 4 PWM Dim Value Registers Updated!"));
          }
          else if(input2 == 'R')
          {
            Serial.print(F("Channel 4 PWM Dim Value Registers Contents: "));
            lt3966_i2c_read(LT3966_ADD, DIM4H, &regVal);
            lt3966_i2c_read(LT3966_ADD, DIM4L, &regVal2);
            printPaddedBinary(regVal);
            Serial.print(F(" "));
            printPaddedBinary(regVal2);
            Serial.print(F("\n"));

            readpwm1 = regVal >> 5;
            // This shifts DIM bits right by 7 - SEL to have the correct DIM value for reading.
            readpwm2 = (int)((regVal&0x1F)) + ((int)regVal2*32) >> (7 - int(readpwm1));
            Serial.print(F("PWM Cycle Length Select: "));
            Serial.println((int)readpwm1);
            n_period = pow(2,6 + (int)readpwm1);                                  // n = 2^(6+SEL)
            Serial.print(F("PWM Duty Cycle (%): "));
            Serial.println(((float)readpwm2)/n_period*100);                       // Duty Cycle = DIM/n * 100%
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        default:
          Serial.println(F("\n***********Invalid input***********"));
          break;
      }

      break;

    case 5:     // Analog Dim Reg
      Serial.println(F("\nAnalog Dimming Register"));
      Serial.println(F("\nWhich Channel Register would you like access? (1)(2)(3)(4)\n"));
      input4 = readTerminalDecimal();
      Serial.print(F("You Entered: "));
      Serial.println(input4);
      Serial.print("\n");
      switch(input4)
      {
        case 1: //CH1
          if(input2 == 'W')
          {
            Serial.print(F("Enter an integer between 1-255: "));
            input3 = readTerminal();
            Serial.println(input3);
            lt3966_i2c_write(LT3966_ADD, ADIM1, input3);
            Serial.print(F("ADIM1: "));
            printPaddedBinary(input3);
            Serial.println(F("\nChannel 1 Analog Dimming Register (ADIM1) Updated!"));
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, ADIM1, &regVal);
            Serial.print(F("Channel 1 Analog Dimming Register (ADIM1) Contents: "));
            printPaddedBinary(regVal);
            Serial.print(F("\nAnalog Dimming Value: "));
            Serial.print((int)regVal);
            Serial.print(F("\n"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }         
          break;

        case 2: // CH2
          if(input2 == 'W')
          {
            Serial.print(F("Enter an integer between 1-255: "));
            input3 = readTerminal();
            Serial.println(input3);
            lt3966_i2c_write(LT3966_ADD, ADIM2, input3);
            Serial.print(F("ADIM2: "));
            printPaddedBinary(input3);
            Serial.println(F("\nChannel 2 Analog Dimming Register (ADIM2) Updated!"));
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, ADIM2, &regVal);
            Serial.print(F("Channel 2 Analog Dimming Register (ADIM2) Contents: "));
            printPaddedBinary(regVal);
            Serial.print(F("\nAnalog Dimming Value: "));
            Serial.print((int)regVal);
            Serial.print(F("\n"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        case 3: // CH3
          if(input2 == 'W')
          {
            Serial.print(F("Enter an integer between 1-255: "));
            input3 = readTerminal();
            Serial.println(input3);
            lt3966_i2c_write(LT3966_ADD, ADIM3, input3);
            Serial.print(F("ADIM3: "));
            printPaddedBinary(input3);
            Serial.println(F("\nChannel 3 Analog Dimming Register (ADIM3) Updated!"));
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, ADIM3, &regVal);
            Serial.print(F("Channel 3 Analog Dimming Register (ADIM3) Contents: "));
            printPaddedBinary(regVal);
            Serial.print(F("\nAnalog Dimming Value: "));
            Serial.print((int)regVal);
            Serial.print(F("\n"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        case 4: // CH4
          if(input2 == 'W')
          {
            Serial.print(F("Enter an integer between 1-255: "));
            input3 = readTerminal();
            Serial.println(input3);
            lt3966_i2c_write(LT3966_ADD, ADIM4, input3);
            Serial.print(F("ADIM4: "));
            printPaddedBinary(input3);
            Serial.println(F("\nChannel 4 Analog Dimming Register (ADIM4) Updated!"));
          }
          else if(input2 == 'R')
          {
            lt3966_i2c_read(LT3966_ADD, ADIM4, &regVal);
            Serial.print(F("Channel 4 Analog Dimming Register (ADIM4) Contents: "));
            printPaddedBinary(regVal);
            Serial.print(F("\nAnalog Dimming Value: "));
            Serial.print((int)regVal);
            Serial.print(F("\n"));
          }
          else
          {
            Serial.println(F("\n***********Invalid input***********"));
          }
          break;

        default:
          Serial.println(F("\n***********Invalid input***********"));
          break;
      } 
      break;

    case 6:     // ADC Reg
    {
      float vin = 0;
      float temp = 0;
      float ext1 = 0;
      float ext2 = 0;
      float vfb1 = 0;
      float iled1 = 0;
      float vfb2 = 0;
      float iled2 = 0;
      float vfb3 = 0;
      float iled3 = 0;
      float vfb4 = 0;
      float iled4 = 0;

      uint8_t vinReg = 0;
      uint8_t tempReg = 0;
      uint8_t ext1Reg = 0;
      uint8_t ext2Reg = 0;
      uint8_t vfb1Reg = 0;
      uint8_t iled1Reg = 0;
      uint8_t vfb2Reg = 0;
      uint8_t iled2Reg = 0;
      uint8_t vfb3Reg = 0;
      uint8_t iled3Reg = 0;
      uint8_t vfb4Reg = 0;
      uint8_t iled4Reg = 0;

      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA0);
      delayMicroseconds(4);
      lt3966_i2c_read(LT3966_ADD, VIN, &vinReg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA1);
      lt3966_i2c_read(LT3966_ADD, TEMP, &tempReg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA2);
      lt3966_i2c_read(LT3966_ADD, EXT1, &ext1Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA3);
      lt3966_i2c_read(LT3966_ADD, EXT2, &ext2Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA4);
      lt3966_i2c_read(LT3966_ADD, VFB1, &vfb1Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA5);
      lt3966_i2c_read(LT3966_ADD, ILED1, &iled1Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA6);        
      lt3966_i2c_read(LT3966_ADD, VFB2, &vfb2Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA7);
      lt3966_i2c_read(LT3966_ADD, ILED2, &iled2Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA8);
      lt3966_i2c_read(LT3966_ADD, VFB3, &vfb3Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xA9);
      lt3966_i2c_read(LT3966_ADD, ILED3, &iled3Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xAA);
      lt3966_i2c_read(LT3966_ADD, VFB4, &vfb4Reg);
      lt3966_i2c_write(LT3966_ADD, ADCCFG, 0xAB);
      lt3966_i2c_read(LT3966_ADD, ILED4, &iled4Reg);

      Serial.println(F("\nAnalog to Digital Converter Register"));

      Serial.print(F("VIN:    "));
      printPaddedBinary(vinReg);
      vin = (vinReg * 0.24);
      Serial.print(F("  "));
      Serial.print(vin);
      Serial.print(F("V"));
      Serial.print(F("\n"));

      Serial.print(F("EXT1:   "));
      printPaddedBinary(ext1Reg);
      ext1 = (ext1Reg * 0.005);
      Serial.print(F("  "));
      Serial.print(ext1,3);
      Serial.print(F("V"));
      Serial.print(F("\n"));

      Serial.print(F("EXT2:   "));
      printPaddedBinary(ext2Reg);
      ext2 = (ext2Reg * 0.005);
      Serial.print(F("  "));
      Serial.print(ext2,3);
      Serial.print(F("V"));
      Serial.print(F("\n"));

      Serial.print(F("VFB1:   "));
      printPaddedBinary(vfb1Reg);
      vfb1 = (vfb1Reg * 0.005);
      Serial.print(F("  "));
      Serial.print(vfb1,3);
      Serial.print(F("V"));
      Serial.print(F("\n"));

      Serial.print(F("ILED1:  "));
      printPaddedBinary(iled1Reg);
      iled1 = ((iled1Reg * 0.005)/4);
      Serial.print(F("  "));
      Serial.print(iled1,3);
      Serial.print(F("A"));
      Serial.print(F("\n"));

      Serial.print(F("VFB2:   "));
      printPaddedBinary(vfb2Reg);
      vfb2 = (vfb2Reg * 0.005);
      Serial.print(F("  "));
      Serial.print(vfb2,3);
      Serial.print(F("V"));
      Serial.print(F("\n"));

      Serial.print(F("ILED2:  "));
      printPaddedBinary(iled2Reg);
      iled2 = ((iled2Reg * 0.005)/4);
      Serial.print(F("  "));
      Serial.print(iled2,3);
      Serial.print(F("A"));
      Serial.print(F("\n"));

      Serial.print(F("VFB3:   "));
      printPaddedBinary(vfb3Reg);
      vfb3 = (vfb3Reg * 0.005);
      Serial.print(F("  "));
      Serial.print(vfb3,3);
      Serial.print(F("V"));
      Serial.print(F("\n"));

      Serial.print(F("ILED3:  "));
      printPaddedBinary(iled3Reg);
      iled3 = ((iled3Reg * 0.005)/4);
      Serial.print(F("  "));
      Serial.print(iled3,3);
      Serial.print(F("A"));
      Serial.print(F("\n"));

      Serial.print(F("VFB4:   "));
      printPaddedBinary(vfb4Reg);
      vfb4 = (vfb4Reg * 0.005);
      Serial.print(F("  "));
      Serial.print(vfb4,3);
      Serial.print(F("V"));
      Serial.print(F("\n"));

      Serial.print(F("ILED4:  "));
      printPaddedBinary(iled4Reg);
      iled4 = ((iled4Reg * 0.005)/4);
      Serial.print(F("  "));
      Serial.print(iled4,3);
      Serial.print(F("A"));
      Serial.print(F("\n"));

      break;
    }

    case 7:     // Check ALERT Pin
    {
      uint8_t alert = digitalRead(ALERT_PIN);
      if(alert)
      {
        Serial.println(F("\nNo Fault Detected / ALERT Pin Deasserted (HIGH)"));
      }
      else
      {
        Serial.println(F("\n***Fault Detected!*** / ALERT Pin Asserted (LOW)"));
      }
      break;
    }

    case 8:
    {
      if(input2 == 'W')
      {
        lt3966_i2c_broadcast_write();
        Serial.println(F("\nLT3966 PWM Dimming Cycles Synchronized!"));
      }

      if(input2 == 'R')
      {
        if(lt3966_i2c_broadcast_read(&regVal))
        {
          Serial.println(F("\nNo LT3966 Devices Reporting Fault"));
        }
        else
        {
          Serial.print(F("\nFaulty Device Address: "));
          printPaddedBinary(regVal); 
          Serial.print(F("\n\n"));         
        }
      }

      else
      {
        Serial.println(F("\n***********Invalid input***********"));
      }

      break;
    }

    case 9:
    {
    	lt3966_i2c_write(LT3966_ADD, ADIM1, 0b11111111);	//Reduce Analog Dim to prevent false trip of overcurrent comparitor 
    	lt3966_i2c_write(LT3966_ADD, ADIM2, 0b11111111);	//Reduce Analog Dim to prevent false trip of overcurrent comparitor 
    	lt3966_i2c_write(LT3966_ADD, ADIM3, 0b11111111);	//Reduce Analog Dim to prevent false trip of overcurrent comparitor 
    	lt3966_i2c_write(LT3966_ADD, ADIM4, 0b11111111);	//Reduce Analog Dim to prevent false trip of overcurrent comparitor 
    	while(Serial.available() == 0)
    		{cycleColor();}
    	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11100000, DIM1L, 0b00000000);		//Set Red PWM Register to OFF
  		lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//Set White PWM Register to OFF
  		lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11100000, DIM2L, 0b00000000);		//Set Green PWM Register to OFF
  		lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11100000, DIM3L, 0b00000000);		//Set Blue PWM Register to OFF
  		lt3966_i2c_write(LT3966_ADD, ADIM1, 0b11111111);	//Reset Analog Dim to 100%
    	lt3966_i2c_write(LT3966_ADD, ADIM2, 0b11111111);	//Reset Analog Dim to 100%
    	lt3966_i2c_write(LT3966_ADD, ADIM3, 0b11111111);	//Reset Analog Dim to 100%
    	lt3966_i2c_write(LT3966_ADD, ADIM4, 0b11111111);	//Reset Analog Dim to 100%
  		break;

    }

    default:
      Serial.println(F("\n***********Invalid input***********"));
      break;
  }
}


//************************************************************************************
// Functions for command line interface. NOT TO BE INCLUDED IN LT3966 HEADER FILE
float readfloatTerminal()
{
  float buf;
  while ( Serial.available() == 0);
  while ( Serial.available() > 0 )
  {
    buf = Serial.parseFloat();
  }
  return (buf);
}

long readTerminal()
{
  while (Serial.available() == 0);
  long buf = Serial.parseInt();
  delay(10); // Short delay to allow any remaining characters to arrive
  // Clear any remaining characters in the buffer, including newline
  while (Serial.available() > 0)
  {
    Serial.read();
  }
  return (buf);
}



uint8_t readTerminalDecimal()
{
  while (Serial.available() == 0);
  uint8_t buf = Serial.parseInt();
  delay(10); // Short delay to allow any remaining characters to arrive
  // Clear any remaining characters in the buffer, including newline
  while (Serial.available() > 0)
  {
    Serial.read();
  }
  return (buf);
}


uint8_t readTerminalBinary()
{
  char bufStr[] = "00000000";
  uint8_t value = 0;

  while ( Serial.available() == 0);
    while ( Serial.available() > 0 )
    {
      Serial.readBytes(bufStr, 8);
    }
          for (int x = 0; x < 8; x++)
      {
          if (bufStr[x] == '1')
          {
            double temp = pow(2, 7 - x) + 0.5;
            value += (uint8_t) temp;
          }
      }
    return(value);
}

uint16_t calcDuty (float percentage)
{
  float temp;
  temp = (((percentage / 100) * 8192) - 1);
  Serial.println("\n");
  Serial.print(temp);
  Serial.println("\n");
  return ((uint16_t) temp);
}

uint8_t calcAnalog(float percentage)
{
  float temp;
  temp = (((percentage / 100) * 256) - 1);
  return ((uint8_t)temp);
}

void printPaddedBinary(uint8_t value)
{
  uint8_t mask = 0b10000000;
  while(mask != 0)
  {
    if((mask & value) != mask)
    {
      Serial.print("0");
    }
    else
    {
      Serial.print("1");
    }
    mask = mask >> 1;
  }
  return;
}

void cycleColor(void)
{
 while(Serial.available() ==0)
 {
  uint8_t delayval = 1000;
  uint8_t pwm1bits = 0;
  uint8_t pwm2bits = 0;

  lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11100000, DIM1L, 0b00000000);		//Red
  lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White
  lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11100000, DIM2L, 0b00000000);		//Green
  lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11100000, DIM3L, 0b00000000);		//Blue

  for (uint16_t x = 0; x < 8192; x++) //Blue Fade Up
    {
      pwm2bits = (x & 0xff);
      pwm1bits = (x >> 8);
      pwm1bits &= 0b00011111;
      pwm1bits |= 0b11100000;
      lt3966_i2c_write_rs(LT3966_ADD, DIM3H, pwm1bits, DIM3L, pwm2bits);			//Blue
    }

  for (uint8_t y = 0; y < 3; y++)			//Color Cycle Starting with Blue
  {
  lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11111111, DIM3L, 0b11111111);		//Blue
	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11100000, DIM2L, 0b00000000);		//Green
  	for (uint16_t x = 0; x < 8192; x++) //Red Fade Up, Blue Constant ON, Green Constant OFF
  	{
  	  pwm2bits = (x & 0xff);
  	  pwm1bits = (x >> 8);
  	  pwm1bits &= 0b00011111;
  	  pwm1bits |= 0b11100000;
  	  lt3966_i2c_write_rs(LT3966_ADD, DIM1H, pwm1bits, DIM1L, pwm2bits);			//Red
  	}

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11111111, DIM1L, 0b11111111);		//Red
	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11100000, DIM2L, 0b00000000);		//Green
  	for (uint16_t x = 8191; x > 0; x--) //Blue Fade Down, Red Constant ON, Green Constant OFF
  	{
  	  pwm2bits = (x & 0xff);
  	  pwm1bits = (x >> 8);
  	  pwm1bits &= 0b00011111;
  	  pwm1bits |= 0b11100000;
	  lt3966_i2c_write_rs(LT3966_ADD, DIM3H, pwm1bits, DIM3L, pwm2bits);			//Blue
  	}

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11111111, DIM1L, 0b11111111);		//Red
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11100000, DIM3L, 0b00000000);		//Blue
  	for (uint16_t x = 0; x < 8192; x++) //Green Fade Up, Red Constant ON, Blue Constant OFF
  	{
  	  pwm2bits = (x & 0xff);
  	  pwm1bits = (x >> 8);
  	  pwm1bits &= 0b00011111;
  	  pwm1bits |= 0b11100000;
	  lt3966_i2c_write_rs(LT3966_ADD, DIM2H, pwm1bits, DIM2L, pwm2bits);			//Green
  	}

  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11111111, DIM2L, 0b11111111);		//Green
	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11100000, DIM3L, 0b00000000);		//Blue
  	for (uint16_t x = 8191; x > 0; x--) //Red Fade Down, Green Constant ON, Blue Constant OFF
  	{
  	  pwm2bits = (x & 0xff);
  	  pwm1bits = (x >> 8);
  	  pwm1bits &= 0b00011111;
  	  pwm1bits |= 0b11100000;
  	  lt3966_i2c_write_rs(LT3966_ADD, DIM1H, pwm1bits, DIM1L, pwm2bits);			//Red
  	}
  	
	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11100000, DIM1L, 0b00000000);		//Red
	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11111111, DIM2L, 0b11111111);		//Green
  	for (uint16_t x = 0; x < 8192; x++) //Blue Fade Up, Green Constant ON, Red Constant OFF
  	{
  	  pwm2bits = (x & 0xff);
  	  pwm1bits = (x >> 8);
  	  pwm1bits &= 0b00011111;
  	  pwm1bits |= 0b11100000;
	  lt3966_i2c_write_rs(LT3966_ADD, DIM3H, pwm1bits, DIM3L, pwm2bits);			//Blue
  	}

	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11100000, DIM1L, 0b00000000);		//Red
	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11111111, DIM3L, 0b11111111);		//Blue
  	for (uint16_t x = 8191; x > 0; x--) //Green Fade Down, Blue Constant ON, Red Constant OFF
  	{
  	  pwm2bits = (x & 0xff);
  	  pwm1bits = (x >> 8);
  	  pwm1bits &= 0b00011111;
  	  pwm1bits |= 0b11100000;
	  lt3966_i2c_write_rs(LT3966_ADD, DIM2H, pwm1bits, DIM2L, pwm2bits);			//Green
  	}
  }
  /*

  for(uint8_t z =0; z < 3; z++)
  {
   	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11100000, DIM1L, 0b00000000);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11100000, DIM2L, 0b00000000);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11100000, DIM3L, 0b00000000);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White

  	delay(delayval);

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11111111, DIM1L, 0b11111111);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11100000, DIM2L, 0b00000000);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11100000, DIM3L, 0b00000000);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White

  	delay(delayval);

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11100000, DIM1L, 0b00000000);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11111111, DIM2L, 0b11111111);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11100000, DIM3L, 0b00000000);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White

  	delay(delayval);

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11100000, DIM1L, 0b00000000);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11100000, DIM2L, 0b00000000);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11111111, DIM3L, 0b11111111);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White

  	delay(delayval);

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11111111, DIM1L, 0b11111111);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11111111, DIM2L, 0b11111111);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11100000, DIM3L, 0b00000000);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White

  	delay(delayval);

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11100000, DIM1L, 0b00000000);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11111111, DIM2L, 0b11111111);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11111111, DIM3L, 0b11111111);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White

  	delay(delayval);

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11111111, DIM1L, 0b11111111);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11100000, DIM2L, 0b00000000);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11111111, DIM3L, 0b11111111);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White

  	delay(delayval);

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b11111111, DIM1L, 0b11111111);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b11111111, DIM2L, 0b11111111);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b11111111, DIM3L, 0b11111111);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11100000, DIM4L, 0b00000000);		//White

  	delay(delayval);

  	lt3966_i2c_write_rs(LT3966_ADD, DIM1H, 0b00000000, DIM1L, 0b00000000);		//Red  
  	lt3966_i2c_write_rs(LT3966_ADD, DIM2H, 0b00000000, DIM2L, 0b00000000);		//Green
  	lt3966_i2c_write_rs(LT3966_ADD, DIM3H, 0b00000000, DIM3L, 0b00000000);		//Blue 
  	lt3966_i2c_write_rs(LT3966_ADD, DIM4H, 0b11111111, DIM4L, 0b11111111);		//White

  	delay(delayval);

  }
  */
  return;
 }
}
