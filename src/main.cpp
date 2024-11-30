#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "Linduino.h"
#include "LT3966.h"

// New LED module addresses - ADD THESE BEFORE setup()
#define LT3966_ADD1  0b1111   // ADD1: VCC, ADD2: VCC (0x5F)
#define LT3966_ADD2  0b0011   // ADD1: VCC, ADD2: FLOAT (0x57)  New module ADD1: Flaot, ADD2: GND (0x53;0011)
#define LT3966_ADD3  0b0001   // ADD1: FLOAT, ADD2: GND (0x51)
#define LT3966_ADD4  0b0101   // ADD1: FLOAT, ADD2: FLOAT (0x55)

// Add global variable for current LED
uint8_t current_led_address = LT3966_ADD1; // Default to first LED
bool debug_mode = true;  // Enable debugging output

// Original function declarations
float readfloatTerminal();
uint8_t readTerminalDecimal();
void printPaddedBinary(uint8_t value);
void verifyConfiguration(uint8_t address);
uint16_t calculatePWM(float duty_cycle, uint8_t scl);
bool verifyI2CWriteRS(uint8_t address, uint8_t reg1, uint8_t value1, uint8_t reg2, uint8_t value2);
bool verifyI2CWrite(uint8_t address, uint8_t reg, uint8_t value);

// Add this helper function at the top with other function declarations
String readSerialUntilEnter() {
    String input = "";
    bool newLine = false;
    
    while (!newLine) {
        if (Serial.available()) {
            char c = Serial.read();
            
            // Handle backspace
            if (c == '\b' || c == 127) {  // 127 is also backspace on some terminals
                if (input.length() > 0) {
                    input.remove(input.length() - 1);
                    Serial.print("\b \b");  // Erase character from terminal
                }
            }
            // Handle line endings
            else if (c == '\r' || c == '\n') {
                if (input.length() > 0) {
                    newLine = true;
                }
            }
            // Only add printable characters to input
            else if (c >= 32 && c <= 126) {
                input += c;
                Serial.print(c); // Echo the character back
            }
        }
    }
    Serial.println(); // Add a newline after input
    return input;
}

float readfloatTerminal()
{
    String input = readSerialUntilEnter();
    return input.toFloat();
}

uint8_t readTerminalDecimal()
{
    String input = readSerialUntilEnter();
    if(input.length() == 0) {
        return 255; // Return invalid value
    }
    
    // Convert string to integer
    uint8_t value = input.toInt();
    
    // Verify the conversion was successful
    if(value == 0 && input[0] != '0') {
        return 255; // Return invalid value if conversion failed
    }
    
    return value;
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
void setup()
{
    Wire.begin();
//    pinMode(ALERT_PIN, INPUT);
    Serial.begin(9600);

    // Initialize all LED modules
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    for (uint8_t i = 0; i < 4; i++) {

    // Pre-set Global Configuration Register (GLBCFG)
    uint8_t glbcfg = 0x00; // All bits initialized to 0
    // Bits 6-4 already set to 0 (WDTEN, MPHASE, CLKOUT disabled)
    // Bits 3-0: OFF[3:0], set to 0 to enable all channels
    lt3966_i2c_write(addresses[i], GLBCFG, glbcfg);

    // Disable Status Reporting in STAT1 to STAT4
        uint8_t stat_reg = 0x00; // All bits set to 0
        lt3966_i2c_write(addresses[i], STAT1, stat_reg);
        lt3966_i2c_write(addresses[i], STAT2, stat_reg);
        lt3966_i2c_write(addresses[i], STAT3, stat_reg);
        lt3966_i2c_write(addresses[i], STAT4, stat_reg);

        // Pre-set Channel Configuration Registers (CFG1 to CFG4)
        uint8_t cfg_reg = 0x03; // Set both DIMEN (bit 0) and ICTRL (bit 1)
        lt3966_i2c_write(addresses[i], CFG1, cfg_reg);
        lt3966_i2c_write(addresses[i], CFG2, cfg_reg);
        lt3966_i2c_write(addresses[i], CFG3, cfg_reg);
        lt3966_i2c_write(addresses[i], CFG4, cfg_reg);

        // Pre-set Analog Dimming Registers to maximum
        lt3966_i2c_write(addresses[i], ADIM1, 0xFF); // Set Analog Dimming to 100%
        lt3966_i2c_write(addresses[i], ADIM2, 0xFF);
        lt3966_i2c_write(addresses[i], ADIM3, 0xFF);
        lt3966_i2c_write(addresses[i], ADIM4, 0xFF);

        // Initialize PWM settings
		// PWM Configuration Notes:
		// DIMxH: bits 7-5 = SCL (frequency select)
		//        bits 4-0 = upper 5 bits of duty cycle
		// DIMxL: bits 7-0 = lower 8 bits of duty cycle
        lt3966_i2c_write_rs(addresses[i], DIM1H, 0b11100000, DIM1L, 0b00000001);
        lt3966_i2c_write_rs(addresses[i], DIM2H, 0b11100000, DIM2L, 0b00000001);
        lt3966_i2c_write_rs(addresses[i], DIM3H, 0b11100000, DIM3L, 0b00000001);
        lt3966_i2c_write_rs(addresses[i], DIM4H, 0b11100000, DIM4L, 0b00000001);

        if(debug_mode) {
            verifyConfiguration(addresses[i]);
        }
    }

    Serial.println(F("*** LT3966 Command Line Interface ***\n"));
}

// Verification function
void verifyConfiguration(uint8_t address) {
    uint8_t value;
    
    Serial.print(F("Verifying LED Module 0x"));
    Serial.println(address, HEX);
    
    // Check GLBCFG
    lt3966_i2c_read(address, GLBCFG, &value);
    Serial.print(F("GLBCFG: 0x"));
    Serial.println(value, HEX);
    
    // Add ADIM verification and reset if needed
    for(uint8_t ch = 0; ch < 4; ch++) {
        uint8_t adim_value;
        lt3966_i2c_read(address, ADIM1 + ch, &adim_value);
        if(adim_value != 0xFF) {
            lt3966_i2c_write(address, ADIM1 + ch, 0xFF);
            Serial.print(F("Reset ADIM"));
            Serial.println(ch + 1);
        }
    }
    
    // Check channel configurations
    for(uint8_t ch = 0; ch < 4; ch++) {
        lt3966_i2c_read(address, CFG1 + (ch * 0x10), &value);
        Serial.print(F("CFG"));
        Serial.print(ch + 1);
        Serial.print(F(": 0x"));
        Serial.println(value, HEX);
    }

    // Verify ADIM and PWM settings
    for(uint8_t ch = 0; ch < 4; ch++) {
        uint8_t adim_value, dimh_value, diml_value;
        lt3966_i2c_read(address, ADIM1 + ch, &adim_value);
        lt3966_i2c_read(address, DIM1H + (ch * 0x10), &dimh_value);
        lt3966_i2c_read(address, DIM1L + (ch * 0x10), &diml_value);
        
        if(debug_mode) {
            Serial.print(F("CH"));
            Serial.print(ch + 1);
            Serial.print(F(" ADIM: 0x"));
            Serial.print(adim_value, HEX);
            Serial.print(F(" PWM: 0x"));
            Serial.print(dimh_value, HEX);
            Serial.print(diml_value, HEX);
            Serial.println();
        }
    }
}

void loop() {
    Serial.println(F("\n=== LED Control System ==="));
    Serial.println(F("(1) Quick Controls"));
    Serial.println(F("(2) Individual LED Control"));
    Serial.println(F("(3) Status and Settings"));
    Serial.print(F("Enter choice: "));

    String top_choice = readSerialUntilEnter();
    Serial.println();

    if(top_choice == "1") {
        showQuickControls();
    }
    else if(top_choice == "2") {
        showIndividualControl();
    }
    else if(top_choice == "3") {
        showStatusAndSettings();
    }
    else {
        Serial.println(F("Invalid selection"));
    }
}

// Calculate PWM values based on percentage
uint16_t calculatePWM(float percentage, uint8_t scl) {
    if(percentage < 0.0f || percentage > 100.0f) return 0;
    
    if(percentage == 0.0f) {
        return 0;  // Will be combined with 0b11100000 in high byte
    } else if(percentage == 100.0f) {
        return 0xFFFF;  // Will result in maximum PWM value
    }
    
    uint16_t period = (1 << (6 + scl));
    uint16_t value = (uint16_t)((percentage / 100.0) * (period - 1));
    return value;
}

// Set PWM duty cycle for a channel
void setPWMDutyCycle(uint8_t address, uint8_t channel, float percentage) {
    uint8_t dimh_reg = DIM1H + (channel * 0x10);
    uint8_t diml_reg = dimh_reg + 1;
    
    if(percentage <= 0.0f) {
        lt3966_i2c_write_rs(address, dimh_reg, 0b11100000, diml_reg, 0x00);
    }
    else if(percentage >= 100.0f) {
        lt3966_i2c_write_rs(address, dimh_reg, 0b11111111, diml_reg, 0xFF);
    }
    else {
        uint8_t scl = 7;  // Maximum resolution preserved
        uint16_t pwm_value = calculatePWM(percentage, scl);
        uint8_t dim_high = 0b11100000 | ((pwm_value >> 8) & 0x1F);
        uint8_t dim_low = pwm_value & 0xFF;
        lt3966_i2c_write_rs(address, dimh_reg, dim_high, diml_reg, dim_low);
    }
}

// Add these helper functions
bool verifyI2CWrite(uint8_t address, uint8_t reg, uint8_t value) {
    uint8_t readback;
    if(lt3966_i2c_write(address, reg, value) != 0) return false;
    if(lt3966_i2c_read(address, reg, &readback) != 0) return false;
    return (readback == value);
}

bool verifyI2CWriteRS(uint8_t address, uint8_t reg1, uint8_t value1, uint8_t reg2, uint8_t value2) {
    uint8_t readback1, readback2;
    if(lt3966_i2c_write_rs(address, reg1, value1, reg2, value2) != 0) return false;
    if(lt3966_i2c_read(address, reg1, &readback1) != 0) return false;
    if(lt3966_i2c_read(address, reg2, &readback2) != 0) return false;
    return (readback1 == value1 && readback2 == value2);
}

void showQuickControls() {
    Serial.println(F("\n=== Quick Controls ==="));
    Serial.println(F("(1) Turn All LEDs On/Off"));
    Serial.println(F("(2) Set All LEDs to Same Brightness"));
    Serial.print(F("Enter choice: "));
    
    String choice = readSerialUntilEnter();
    
    if(choice == "1") {
        Serial.println(F("\nSelect state:"));
        Serial.println(F("(1) Turn All On"));
        Serial.println(F("(2) Turn All Off"));
        Serial.print(F("Enter choice: "));
        
        String state = readSerialUntilEnter();
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        
        for(uint8_t i = 0; i < 4; i++) {
            if(state == "1") {
                lt3966_i2c_write(addresses[i], GLBCFG, 0x00);  // Enable all channels
                setPWMDutyCycle(addresses[i], 0, 100.0f);  // Set to full brightness
                setPWMDutyCycle(addresses[i], 1, 100.0f);
                setPWMDutyCycle(addresses[i], 2, 100.0f);
                setPWMDutyCycle(addresses[i], 3, 100.0f);
            } else if(state == "2") {
                lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);  // Disable all channels
            }
        }
    }
    else if(choice == "2") {
        Serial.print(F("\nEnter brightness (0-120mA): "));
        float current_ma = readfloatTerminal();
        if(current_ma < 0 || current_ma > 120) {
            Serial.println(F("Invalid current value"));
            return;
        }
        
        float duty_cycle = (current_ma / 120.0f) * 100.0f;
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        
        for(uint8_t i = 0; i < 4; i++) {
            lt3966_i2c_write(addresses[i], GLBCFG, 0x00);  // Enable all channels
            for(uint8_t ch = 0; ch < 4; ch++) {
                setPWMDutyCycle(addresses[i], ch, duty_cycle);
            }
        }
    }
}

// Keep channel configuration function
void configureChannel(uint8_t address, uint8_t channel, bool inPhase) {
    uint8_t cfg_reg = CFG1 + (channel * 0x10);
    uint8_t cfg_value = 0x03;  // Keep DIMEN and ICTRL settings
    if(inPhase) cfg_value |= (1 << 3);  // Set INPH bit if needed
    lt3966_i2c_write(address, cfg_reg, cfg_value);
}

void showIndividualControl() {
    Serial.println(F("\n=== Individual LED Control ==="));
    Serial.println(F("Select LED (1-4): "));
    uint8_t led = readTerminalDecimal();
    if(led < 1 || led > 4) {
        Serial.println(F("Invalid LED selection"));
        return;
    }
    
    uint8_t address;
    switch(led) {
        case 1: address = LT3966_ADD1; break;
        case 2: address = LT3966_ADD2; break;
        case 3: address = LT3966_ADD3; break;
        case 4: address = LT3966_ADD4; break;
    }
    
    Serial.println(F("Select Channel:"));
    Serial.println(F("(1) White"));
    Serial.println(F("(2) Blue"));
    Serial.println(F("(3) Green"));
    Serial.println(F("(4) Red"));
    
    uint8_t channel = readTerminalDecimal();
    if(channel < 1 || channel > 4) {
        Serial.println(F("Invalid channel selection"));
        return;
    }
    channel--; // Convert to 0-based index
    
    Serial.print(F("Enter current (0-120mA): "));
    float current_ma = readfloatTerminal();
    if(current_ma < 0 || current_ma > 120) {
        Serial.println(F("Invalid current value"));
        return;
    }
    
    float duty_cycle = (current_ma / 120.0f) * 100.0f;
    lt3966_i2c_write(address, GLBCFG, 0x00);  // Ensure channel is enabled
    setPWMDutyCycle(address, channel, duty_cycle);
}

void showStatusAndSettings() {
    Serial.println(F("\n=== Status and Settings ==="));
    Serial.println(F("(1) View LED Status"));
    Serial.println(F("(2) Configure PWM Resolution"));
    Serial.print(F("Enter choice: "));
    
    String choice = readSerialUntilEnter();
    
    if(choice == "1") {
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        for(uint8_t i = 0; i < 4; i++) {
            verifyConfiguration(addresses[i]);  // Using existing verification function
        }
    }
    else if(choice == "2") {
        Serial.println(F("\nEnter PWM resolution (6-13 bits): "));
        uint8_t resolution = readTerminalDecimal();
        if(resolution < 6 || resolution > 13) {
            Serial.println(F("Invalid resolution"));
            return;
        }
        uint8_t scl = resolution - 6;  // Convert to SCL value
        // Apply to all channels
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        for(uint8_t i = 0; i < 4; i++) {
            for(uint8_t ch = 0; ch < 4; ch++) {
                uint8_t dimh_reg = DIM1H + (ch * 0x10);
                uint8_t current_value;
                lt3966_i2c_read(addresses[i], dimh_reg, &current_value);
                uint8_t new_value = (scl << 5) | (current_value & 0x1F);
                lt3966_i2c_write(addresses[i], dimh_reg, new_value);
            }
        }
    }
}
