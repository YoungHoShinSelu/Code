// Include necessary headers
#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "Linduino.h"
#include "LT3966.h"

// New LED module addresses - ADD THESE BEFORE setup()
#define LT3966_ADD1  0b1111   // ADD1: VCC, ADD2: VCC (0x5F)
#define LT3966_ADD2  0b0011   // ADD1: FLOAT, ADD2: GND (0x53)  
#define LT3966_ADD3  0b0001   // ADD1: FLOAT, ADD2: GND (0x51)
#define LT3966_ADD4  0b0101   // ADD1: FLOAT, ADD2: FLOAT (0x55)

// Add global variable for current LED
uint8_t current_led_address = LT3966_ADD1; // Default to first LED
bool debug_mode = true;  // Enable debugging output

// Original function declarations
float readfloatTerminal();
uint8_t readTerminalDecimal();
void printPaddedBinary(uint8_t value);

float readfloatTerminal()
{
    float buf;
    while (Serial.available() == 0);
    while (Serial.available() > 0)
    {
        buf = Serial.parseFloat();
    }
    return (buf);
}

uint8_t readTerminalDecimal()
{
    while (Serial.available() == 0);
    uint8_t buf = Serial.parseInt();
    while (Serial.available() > 0)
    {
        Serial.read();
    }
    return (buf);
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
        uint8_t cfg_reg = 0x01; // Initialize with DIMEN enabled (Bit 0 = 1)
        lt3966_i2c_write(addresses[i], CFG1, cfg_reg);
        lt3966_i2c_write(addresses[i], CFG2, cfg_reg);
        lt3966_i2c_write(addresses[i], CFG3, cfg_reg);
        lt3966_i2c_write(addresses[i], CFG4, cfg_reg);

        // Pre-set Analog Dimming Registers to maximum
        lt3966_i2c_write(addresses[i], ADIM1, 0x0F); // Set Analog Dimming to 100%
        lt3966_i2c_write(addresses[i], ADIM2, 0x0F);
        lt3966_i2c_write(addresses[i], ADIM3, 0x0F);
        lt3966_i2c_write(addresses[i], ADIM4, 0x0F);

        // Initialize PWM settings
		// PWM Configuration Notes:
		// DIMxH: bits 7-5 = SCL (frequency select)
		//        bits 4-0 = upper 5 bits of duty cycle
		// DIMxL: bits 7-0 = lower 8 bits of duty cycle
        lt3966_i2c_write_rs(addresses[i], DIM1H, 0b11100000, DIM1L, 0b11111111);
        lt3966_i2c_write_rs(addresses[i], DIM2H, 0b11100000, DIM2L, 0b11111111);
        lt3966_i2c_write_rs(addresses[i], DIM3H, 0b11100000, DIM3L, 0b11111111);
        lt3966_i2c_write_rs(addresses[i], DIM4H, 0b11100000, DIM4L, 0b11111111);

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
	
	// Add ADIM verification
    for(uint8_t ch = 0; ch < 4; ch++) {
        uint8_t adim_value;
        lt3966_i2c_read(address, ADIM1 + ch, &adim_value);
        if(adim_value != 0xFF) {
            Serial.println(F("ADIM configuration error"));
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
}

void loop()
{
    // Top level menu
    Serial.println(F("\n=== Main Menu ==="));
    Serial.println(F("(1) LED Select"));
    Serial.println(F("(2) Current Data")); 
    Serial.println(F("(3) Setup"));
    Serial.print(F("Enter choice: "));

    while (!Serial.available());
    char top_choice = Serial.read();
    Serial.println(top_choice);

    if(top_choice == '1') {
        Serial.println(F("\n=== Select LED Module ==="));
        Serial.println(F("(1) LED 1 - 0x5F"));
        Serial.println(F("(2) LED 2 - 0x57"));
        Serial.println(F("(3) LED 3 - 0x51")); 
        Serial.println(F("(4) LED 4 - 0x55"));
        Serial.print(F("Select LED (1-4): "));

        while (!Serial.available());
        char led_choice = Serial.read();
        Serial.println(led_choice);

        // Set current LED address and verify configuration
        switch(led_choice) {
            case '1': 
                current_led_address = LT3966_ADD1;
                break;
            case '2':
                current_led_address = LT3966_ADD2;
                break;
            case '3':
                current_led_address = LT3966_ADD3;
                break;
            case '4':
                current_led_address = LT3966_ADD4;
                break;
            default:
                Serial.println(F("Invalid LED selection"));
                return;
        }

        // Verify and reset configuration if needed
        uint8_t glbcfg_value;
        lt3966_i2c_read(current_led_address, GLBCFG, &glbcfg_value);
        if(glbcfg_value & 0x0F) {
            lt3966_i2c_write(current_led_address, GLBCFG, 0x00);
            if(debug_mode) {
                Serial.println(F("Reset GLBCFG"));
            }
        }

        // Verify channel configurations
        for(uint8_t ch = 0; ch < 4; ch++) {
            uint8_t cfg_value;
            lt3966_i2c_read(current_led_address, CFG1 + (ch * 0x10), &cfg_value);
            if(!(cfg_value & 0x01)) {
                lt3966_i2c_write(current_led_address, CFG1 + (ch * 0x10), 0x01);
                if(debug_mode) {
                    Serial.print(F("Reset CFG"));
                    Serial.println(ch + 1);
                }
            }
        }

        if(debug_mode) {
            verifyConfiguration(current_led_address);
        }

        // Sub-menu
        Serial.println(F("\nMain Menu:"));
        Serial.println(F("(1) Configure In-Phase Mode (INPH)"));
        Serial.println(F("(2) PWM Duty Cycle"));
        Serial.println(F("(4) PWM Frequency"));
        Serial.println(F("(5) ADC Register"));
        Serial.print(F("Enter your choice: "));
    
        while (Serial.available() == 0);
        char input = Serial.read();
        Serial.println(input);

        switch(input) 
        {
            case '1': // Configure INPH
            {
                Serial.println(F("\nConfigure In-Phase Mode (INPH)"));
                Serial.println(F("Apply to:\n1. All Channels\n2. Channel 1\n3. Channel 2\n4. Channel 3\n5. Channel 4"));
                Serial.print(F("Enter choice: "));
                uint8_t channel_choice = readTerminalDecimal();
                Serial.println(channel_choice);

                Serial.print(F("Set INPH to In-Phase? (Y/N): "));
                while (Serial.available() == 0);
                char inph_input = Serial.read();
                Serial.println(inph_input);
                bool set_inph = (inph_input == 'Y' || inph_input == 'y');

                uint8_t cfg_address_start = 0;
                uint8_t cfg_address_end = 0;

                switch (channel_choice)
                {
                    case 1: // All Channels
                        cfg_address_start = CFG1;
                        cfg_address_end = CFG4;
                        break;
                    case 2: // Channel 1
                        cfg_address_start = CFG1;
                        cfg_address_end = CFG1;
                        break;
                    case 3: // Channel 2
                        cfg_address_start = CFG2;
                        cfg_address_end = CFG2;
                        break;
                    case 4: // Channel 3
                        cfg_address_start = CFG3;
                        cfg_address_end = CFG3;
                        break;
                    case 5: // Channel 4
                        cfg_address_start = CFG4;
                        cfg_address_end = CFG4;
                        break;
                    default:
                        Serial.println(F("\n***********Invalid input***********"));
                        break;
                }

                if (cfg_address_start != 0)
                {
                    for (uint8_t addr = cfg_address_start; addr <= cfg_address_end; addr += 0x10)
                    {
                        uint8_t cfg_value = 0x01; // Start with pre-set value (DIMEN enabled)
                        if (set_inph)
                            cfg_value |= (1 << 3); // Set INPH bit
                        lt3966_i2c_write(current_led_address, addr, cfg_value);
                        Serial.print(F("Channel "));
                        Serial.print((addr - CFG1) / 0x10 + 1);
                        Serial.println(F(" Configuration Updated."));
                    }
                }
                break;
            }

            case '2': // PWM Duty Cycle
            {
                Serial.println(F("\nPWM Duty Cycle Configuration"));
                Serial.println(F("Apply to:\n1. All Channels\n2. Channel 1\n3. Channel 2\n4. Channel 3\n5. Channel 4"));
                Serial.print(F("Enter choice: "));
                uint8_t channel_choice = readTerminalDecimal();
                Serial.println(channel_choice);

                // Get PWM duty cycle percentage
                Serial.print(F("\nEnter PWM Duty Cycle (0-100%): "));
                float duty_cycle = readfloatTerminal();
                Serial.println(duty_cycle);

                // Keep track of which channels to update
                uint8_t start_channel = 0;
                uint8_t end_channel = 0;

                switch (channel_choice)
                {
                    case 1: // All Channels
                        start_channel = 0;
                        end_channel = 3;  // 0-3 represents channels 1-4
                        break;
                    case 2: // Channel 1
                        start_channel = 0;
                        end_channel = 0;
                        break;
                    case 3: // Channel 2
                        start_channel = 1;
                        end_channel = 1;
                        break;
                    case 4: // Channel 3
                        start_channel = 2;
                        end_channel = 2;
                        break;
                    case 5: // Channel 4
                        start_channel = 3;
                        end_channel = 3;
                        break;
                    default:
                        Serial.println(F("\n***********Invalid input***********"));
                        break;
                }

                // Update selected channels
                if (start_channel <= end_channel)  // Valid channel selection
                {
                    for (uint8_t ch = start_channel; ch <= end_channel; ch++)
                    {
                        // Read current settings
                        uint8_t dimh_reg = DIM1H + (ch * 0x10);
                        uint8_t current_dimh;
                        lt3966_i2c_read(current_led_address, dimh_reg, &current_dimh);
                        
                        // Calculate new PWM values using helper function
                        uint8_t scl = (current_dimh >> 5) & 0x07;
                        uint16_t pwm_value = calculatePWM(duty_cycle, scl);
                        
                        // Prepare high and low bytes
                        uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
                        uint8_t dim_low = pwm_value & 0xFF;

                        // Update registers
                        lt3966_i2c_write_rs(current_led_address, dimh_reg, dim_high, dimh_reg + 1, dim_low);

                        if(debug_mode) {
                            Serial.print(F("Channel "));
                            Serial.print(ch + 1);
                            Serial.print(F(" PWM Updated - SCL: "));
                            Serial.print(scl);
                            Serial.print(F(" Value: "));
                            Serial.println(pwm_value);
                        } else {
                            Serial.print(F("Channel "));
                            Serial.print(ch + 1);
                            Serial.println(F(" PWM Duty Cycle Updated."));
                        }
                    }
                }
                break;
            }

            case '4': // PWM Frequency
            {
                Serial.println(F("\nPWM Frequency Configuration"));
                Serial.println(F("Apply to:\n1. All Channels\n2. Channel 1\n3. Channel 2\n4. Channel 3\n5. Channel 4"));
                Serial.print(F("Enter choice: "));
                uint8_t channel_choice = readTerminalDecimal();
                Serial.println(channel_choice);

                Serial.println(F("\nSelect PWM Frequency:"));
                Serial.println(F("0: 31.25 kHz  (6-bit,  64 steps)"));
                Serial.println(F("1: 15.625 kHz (7-bit,  128 steps)"));
                Serial.println(F("2: 7.8125 kHz (8-bit,  256 steps)"));
                Serial.println(F("3: 3.906 kHz  (9-bit,  512 steps)"));
                Serial.println(F("4: 1.953 kHz  (10-bit, 1024 steps)"));
                Serial.println(F("5: 976.6 Hz   (11-bit, 2048 steps)"));
                Serial.println(F("6: 488.3 Hz   (12-bit, 4096 steps)"));
                Serial.println(F("7: 244.14 Hz  (13-bit, 8192 steps)"));
                Serial.print(F("Enter SCL value (0-7): "));
                uint8_t scl = readTerminalDecimal();
                Serial.println(scl);

                if (scl > 7) {
                    Serial.println(F("\n***********Invalid SCL value***********"));
                    break;
                }

                uint8_t dimh_start = 0;
                uint8_t dimh_end = 0;

                switch (channel_choice)
                {
                    case 1: // All Channels
                        dimh_start = DIM1H;
                        dimh_end = DIM4H;
                        break;
                    case 2: // Channel 1
                        dimh_start = DIM1H;
                        dimh_end = DIM1H;
                        break;
                    case 3: // Channel 2
                        dimh_start = DIM2H;
                        dimh_end = DIM2H;
                        break;
                    case 4: // Channel 3
                        dimh_start = DIM3H;
                        dimh_end = DIM3H;
                        break;
                    case 5: // Channel 4
                        dimh_start = DIM4H;
                        dimh_end = DIM4H;
                        break;
                    default:
                        Serial.println(F("\n***********Invalid input***********"));
                        break;
                }

                if (dimh_start != 0)
                {
                    for (uint8_t addr = dimh_start; addr <= dimh_end; addr += 0x10)
                    {
                        uint8_t current_dimh, current_diml;
                        lt3966_i2c_read(current_led_address, addr, &current_dimh);
                        lt3966_i2c_read(current_led_address, addr + 1, &current_diml);

                        uint8_t old_scl = (current_dimh >> 5) & 0x07;
                        uint16_t old_period = (1 << (6 + old_scl));
                        uint16_t old_dim_value = ((current_dimh & 0x1F) << 8) | current_diml;
                        float duty_cycle = (float)old_dim_value / old_period * 100.0;

                        uint16_t new_period = (1 << (6 + scl));
                        uint16_t new_dim_value = (uint16_t)((duty_cycle / 100.0) * new_period);
                        if (new_dim_value >= new_period) new_dim_value = new_period - 1;

                        uint8_t dim_high = ((scl & 0x07) << 5) | ((new_dim_value >> 8) & 0x1F);
                        uint8_t dim_low = new_dim_value & 0xFF;

                        lt3966_i2c_write_rs(current_led_address, addr, dim_high, addr + 1, dim_low);
                        Serial.print(F("Channel "));
                        Serial.print((addr - DIM1H) / 0x10 + 1);
                        Serial.println(F(" PWM Frequency Updated."));
                    }
                }
                break;
            }

            case '5': // ADC Register
            {
                struct {
                    bool error;
                    String message;  // Changed from const char* to String
                } status = {false, ""};

                // Constants for scaling and thresholds
                const float VIN_SCALE = 0.24;
                const float VFB_SCALE = 0.005;
                const float ILED_SCALE = 0.000625;
                const float VOUT_DIVIDER = 16.04;

                const float ILED_MAX = 125.0;
                const float ILED_MIN = 5.0;
                const float VFB_MIN = 0.1;
                const float VIN_MIN = 6.70;
                const float VIN_MAX = 60.0;

                // ADC configuration constants
                const uint8_t ADC_AUTO_CONFIG = 0xC0;
                const uint16_t ADC_WAIT_TIME = 100;
                const uint8_t CHANNELS = 4;
                
                // Initialize measurements arrays
                uint8_t vfbRegs[CHANNELS] = {0};
                uint8_t iledRegs[CHANNELS] = {0};
                float vfb_v[CHANNELS] = {0};
                float vout_v[CHANNELS] = {0};
                float iled_ma[CHANNELS] = {0};
                
                // Configure ADC
                if(lt3966_i2c_write(current_led_address, ADCCFG, ADC_AUTO_CONFIG) != 0) {
                    status.error = true;
                    status.message = F("Error: ADC configuration failed");
                    break;
                }
                delayMicroseconds(ADC_WAIT_TIME);

                // Read input voltage
                uint8_t vinReg;
                if(lt3966_i2c_read(current_led_address, VIN, &vinReg) != 0) {
                    status.error = true;
                    status.message = F("Error: Failed to read input voltage");
                    break;
                }
                float vin_v = vinReg * VIN_SCALE;

                // Read channel measurements
                for(uint8_t ch = 0; ch < CHANNELS; ch++) {
                    // Read VFB
                    if(lt3966_i2c_read(current_led_address, VFB1 + (ch * 2), &vfbRegs[ch]) != 0) {
                        status.error = true;
                        status.message = F("Error: Failed to read VFB");
                        break;
                    }
                    
                    // Read ILED
                    if(lt3966_i2c_read(current_led_address, ILED1 + (ch * 2), &iledRegs[ch]) != 0) {
                        status.error = true;
                        status.message = F("Error: Failed to read ILED");
                        break;
                    }
                    
// Convert readings
                    vfb_v[ch] = vfbRegs[ch] * VFB_SCALE;
                    vout_v[ch] = vfb_v[ch] * VOUT_DIVIDER;
                    iled_ma[ch] = iledRegs[ch] * ILED_SCALE * 1000;
                }

                // Check for errors before displaying
                if(status.error) {
                    Serial.println(status.message);
                    break;
                }

                // Display header
                Serial.println(F("\n=== ADC Measurements ==="));
                
                // Display input voltage with status
                Serial.print(F("Input Voltage: "));
                Serial.print(vin_v, 2);
                Serial.print(F(" V - "));
                if(vin_v < VIN_MIN || vin_v > VIN_MAX) {
                    Serial.println(F("WARNING: Out of range"));
                } else {
                    Serial.println(F("OK"));
                }

                // Display channel measurements
                Serial.println(F("\nChannel Measurements:"));
                Serial.println(F("CH  VOUT(V)  Status    ILED(mA)  Status "));
                Serial.println(F("-----------------------------------------"));
                
                for(uint8_t ch = 0; ch < CHANNELS; ch++) {
                    // Channel number
                    Serial.print(F(" "));
                    Serial.print(ch + 1);
                    Serial.print(F("  "));
                    
                    // VOUT value and status
                    Serial.print(vout_v[ch], 2);
                    Serial.print(F("    "));
                    Serial.print(vfb_v[ch] > VFB_MIN ? F("Active ") : F("Off    "));
                    
                    // ILED value and status
                    Serial.print(F("  "));
                    Serial.print(iled_ma[ch], 1);
                    Serial.print(F("     "));
                    
                    // Current status
                    if(iled_ma[ch] > ILED_MAX) {
                        Serial.println(F("OVER"));
                    } else if(iled_ma[ch] > ILED_MIN) {
                        Serial.println(F("OK"));
                    } else {
                        Serial.println(F("LOW"));
                    }
                }
                
                Serial.println(F("-----------------------------------------\n"));
                break;
            }

            default:
                Serial.println(F("\n***********Invalid input***********"));
                break;
        }
    }
    else if(top_choice == '2') {
        Serial.println(F("\n=== Current LED Status ==="));
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        const char* names[] = {"LED 1", "LED 2", "LED 3", "LED 4"};
        
        for(uint8_t i = 0; i < 4; i++) {
            Serial.print(F("\n")); Serial.println(names[i]);
            uint8_t cfg_value;
            lt3966_i2c_read(addresses[i], GLBCFG, &cfg_value);
            Serial.print(F("Status: "));
            Serial.println(cfg_value & 0x0F ? F("Disabled") : F("Enabled"));
        }
    }
    else if(top_choice == '3') {
        Serial.println(F("\n=== Setup Menu ==="));
        Serial.println(F("Setup options coming soon"));
    }
    else {
        Serial.println(F("Invalid selection"));
    }
}

// Calculate PWM values based on percentage
uint16_t calculatePWM(float percentage, uint8_t scl) {
    if(scl > 7) return 0;
    if(percentage < 0.0f || percentage > 100.0f) return 0;
    uint16_t period = (1 << (6 + scl));
    uint16_t value = (uint16_t)((percentage / 100.0) * period);
    return (value >= period) ? period - 1 : value;
}

// Set PWM duty cycle for a channel
void setPWMDutyCycle(uint8_t address, uint8_t channel, float percentage) {
    if(percentage < 0.0f) percentage = 0.0f;
    if(percentage > 100.0f) percentage = 100.0f;
    
    uint8_t dimh_reg = DIM1H + (channel * 0x10);
    uint8_t current_dimh;
    
    // Read current configuration
    if(lt3966_i2c_read(address, dimh_reg, &current_dimh) != 0) {
        if(debug_mode) Serial.println(F("Failed to read current PWM settings"));
        return;
    }
    
    uint8_t scl = (current_dimh >> 5) & 0x07;
    uint16_t pwm_value = calculatePWM(percentage, scl);
    
    uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
    uint8_t dim_low = pwm_value & 0xFF;
    
    if(!verifyI2CWriteRS(address, dimh_reg, dim_high, dimh_reg + 1, dim_low)) {
        if(debug_mode) Serial.println(F("Failed to update PWM settings"));
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