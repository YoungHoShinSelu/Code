// Include necessary headers
#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "Linduino.h"
#include "LT3966.h"

// New LED module addresses - ADD THESE BEFORE setup()
#define LT3966_ADD1  0b1111   // ADD1: VCC, ADD2: VCC (0x5F)
#define LT3966_ADD2  0b0111   // ADD1: VCC, ADD2: FLOAT (0x57)  
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

void loop()
{
    // Top level menu
    Serial.println(F("\n=== Main Menu ==="));
    Serial.println(F("(0) Shutdown All LEDs"));
    Serial.println(F("(1) LED Intensity Control"));  // Changed from "LED Select"
    Serial.println(F("(2) LED Status")); 
    Serial.println(F("(3) Setup"));
    Serial.print(F("Enter choice: "));

    String top_choice = readSerialUntilEnter();
    Serial.println();

    if(top_choice == "0") {
        // Shutdown all LEDs
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        for(uint8_t i = 0; i < 4; i++) {
            // Set GLBCFG to disable all channels
            lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);  // Set OFF[3:0] bits to 1
            Serial.print(F("LED "));
            Serial.print(i + 1);
            Serial.println(F(" shutdown complete"));
        }
        Serial.println(F("All LEDs have been shut down"));
    }
    else if(top_choice == "1") {
        Serial.println(F("\n=== LED Intensity Control ==="));
        Serial.println(F("(1) LED 1 - 0x5F"));
        Serial.println(F("(2) LED 2 - 0x57"));
        Serial.println(F("(3) LED 3 - 0x51")); 
        Serial.println(F("(4) LED 4 - 0x55"));
        Serial.print(F("Select LED (1-4): "));

        String led_choice = readSerialUntilEnter();
        Serial.println();

        // Set current LED address and verify configuration
        switch(led_choice[0]) {
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
            if((cfg_value & 0x03) != 0x03) {  // Check both DIMEN and ICTRL
                lt3966_i2c_write(current_led_address, CFG1 + (ch * 0x10), 0x03);
                if(debug_mode) {
                    Serial.print(F("Reset CFG"));
                    Serial.println(ch + 1);
                }
            }
        }

        if(debug_mode) {
            verifyConfiguration(current_led_address);
        }

        // Remove the redundant "Intensity Control" menu and go straight to channel selection
        Serial.println(F("\nPWM Duty Cycle Configuration"));
        Serial.println(F("Apply to:\n1. All Channels\n2. White\n3. Blue\n4. Green\n5. Red"));
        Serial.print(F("Enter choice: "));
        
        String channel_input = readSerialUntilEnter();
        Serial.println();

        // Keep track of which channels to update
        uint8_t start_channel = 0;
        uint8_t end_channel = 0;

        // Process channel selection
        switch(channel_input[0]) {
            case '1': // All Channels
                start_channel = 0;
                end_channel = 3;
                break;
            case '2': // White
                start_channel = 0;
                end_channel = 0;
                break;
            case '3': // Blue
                start_channel = 1;
                end_channel = 1;
                break;
            case '4': // Green
                start_channel = 2;
                end_channel = 2;
                break;
            case '5': // Red
                start_channel = 3;
                end_channel = 3;
                break;
            default:
                Serial.println(F("\n***********Invalid input***********"));
                return;
        }

        // Get PWM duty cycle percentage
        Serial.print(F("\nEnter PWM Duty Cycle (0-100%): "));
        float duty_cycle = readfloatTerminal();
        Serial.println(duty_cycle);

        // Update selected channels
        for (uint8_t ch = start_channel; ch <= end_channel; ch++) {
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
            }
        }
    }
    else if(top_choice == "2") {
        Serial.println(F("\n=== LED Module Status ==="));
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        const char* names[] = {"LED 1", "LED 2", "LED 3", "LED 4"};
        
        for(uint8_t i = 0; i < 4; i++) {
            Serial.print(F("\n")); 
            Serial.println(names[i]);
            
            // Read Global Configuration
            uint8_t glbcfg_value;
            lt3966_i2c_read(addresses[i], GLBCFG, &glbcfg_value);
            Serial.print(F("Module Status: "));
            Serial.println(glbcfg_value & 0x0F ? F("Disabled") : F("Enabled"));
            
            if(!(glbcfg_value & 0x0F)) {  // If module is enabled
                // For each channel
                const char* channel_names[] = {"White", "Blue", "Green", "Red"};
                for(uint8_t ch = 0; ch < 4; ch++) {
                    Serial.print(channel_names[ch]); 
                    Serial.print(F(": "));
                    
                    // Read Channel Configuration
                    uint8_t cfg_value;
                    lt3966_i2c_read(addresses[i], CFG1 + (ch * 0x10), &cfg_value);
                    
                    // Read PWM Settings
                    uint8_t dimh_value, diml_value;
                    lt3966_i2c_read(addresses[i], DIM1H + (ch * 0x10), &dimh_value);
                    lt3966_i2c_read(addresses[i], DIM1L + (ch * 0x10), &diml_value);
                    
                    // Calculate PWM percentage correctly
                    uint8_t scl = (dimh_value >> 5) & 0x07;
                    uint16_t pwm_value = ((dimh_value & 0x1F) << 8) | diml_value;
                    uint16_t period = (1 << (6 + scl));
                    float duty_cycle = (float)pwm_value / (period - 1) * 100.0;
                    
                    // Display status
                    Serial.print(F("PWM "));
                    Serial.print(duty_cycle, 1);
                    Serial.print(F("%, "));
                    Serial.print(F("INPH "));
                    Serial.println(cfg_value & (1 << 3) ? F("On") : F("Off"));
                }
            }
        }
    }
    else if(top_choice == "3") {
        Serial.println(F("\n=== Setup Menu ==="));
        Serial.println(F("(1) PWM Frequency Configuration"));
        Serial.println(F("(2) Power Monitoring"));
        Serial.println(F("(3) Configure In-Phase Mode (INPH)"));
        Serial.print(F("Enter choice: "));
        
        String setup_choice = readSerialUntilEnter();
        Serial.println();

        switch(setup_choice[0]) {
            case '1': // PWM Frequency
            {
                Serial.println(F("\nPWM Frequency Configuration"));
                Serial.println(F("Apply to:\n1. All Channels\n2. White\n3. Blue\n4. Green\n5. Red"));
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
                    case 2: // White
                        dimh_start = DIM1H;
                        dimh_end = DIM1H;
                        break;
                    case 3: // Blue
                        dimh_start = DIM2H;
                        dimh_end = DIM2H;
                        break;
                    case 4: // Green
                        dimh_start = DIM3H;
                        dimh_end = DIM3H;
                        break;
                    case 5: // Red
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
            
            case '2': // Power Monitoring
            {
                struct {
                    bool error;
                    String message;
                } status = {false, ""};
                
                // LED addresses array
                const uint8_t led_addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
                const char* led_names[] = {"LED 1", "LED 2", "LED 3", "LED 4"};
                const char* channel_names[] = {"White", "Blue", "Green", "Red"};
                
                Serial.println(F("\n=== Power Monitoring ==="));
                Serial.println(F("\nChannel Status:"));
                Serial.println(F("Module  Channel    VOUT(V)  ILED(mA)  Status"));
                Serial.println(F("------------------------------------------------"));
                
                // Iterate through each LED module
                for(uint8_t led = 0; led < 4; led++) {
                    uint8_t current_address = led_addresses[led];
                    
                    // Check if module is enabled
                    uint8_t glbcfg_value;
                    lt3966_i2c_read(current_address, GLBCFG, &glbcfg_value);
                    if(glbcfg_value & 0x0F) {  // Module is disabled
                        Serial.print(led_names[led]);
                        Serial.println(F(" - Disabled"));
                        continue;
                    }

                    // Variables for ADC readings
                    uint8_t vfbRegs[4] = {0};
                    uint8_t iledRegs[4] = {0};
                    
                    // Configure ADC and read all channels in sequence
                    // Exactly matching original code sequence
                    lt3966_i2c_write(current_address, ADCCFG, 0xA0);
                    delayMicroseconds(4);
                    lt3966_i2c_read(current_address, VFB1, &vfbRegs[0]);
                    lt3966_i2c_write(current_address, ADCCFG, 0xA1);
                    lt3966_i2c_read(current_address, ILED1, &iledRegs[0]);
                    lt3966_i2c_write(current_address, ADCCFG, 0xA2);
                    lt3966_i2c_read(current_address, VFB2, &vfbRegs[1]);
                    lt3966_i2c_write(current_address, ADCCFG, 0xA3);
                    lt3966_i2c_read(current_address, ILED2, &iledRegs[1]);
                    lt3966_i2c_write(current_address, ADCCFG, 0xA4);
                    lt3966_i2c_read(current_address, VFB3, &vfbRegs[2]);
                    lt3966_i2c_write(current_address, ADCCFG, 0xA5);
                    lt3966_i2c_read(current_address, ILED3, &iledRegs[2]);
                    lt3966_i2c_write(current_address, ADCCFG, 0xA6);
                    lt3966_i2c_read(current_address, VFB4, &vfbRegs[3]);
                    lt3966_i2c_write(current_address, ADCCFG, 0xA7);
                    lt3966_i2c_read(current_address, ILED4, &iledRegs[3]);

                    // Process and display readings for each channel
                    for(uint8_t ch = 0; ch < 4; ch++) {
                        Serial.print(led_names[led]);
                        Serial.print(F("   "));
                        Serial.print(channel_names[ch]);
                        for(uint8_t i = 0; i < (10 - strlen(channel_names[ch])); i++) Serial.print(" ");

                        // Read PWM settings to check if channel is active
                        uint8_t dimh_value;
                        lt3966_i2c_read(current_address, DIM1H + (ch * 0x10), &dimh_value);
                        
                        if((dimh_value & 0x1F) > 0) {  // Channel is active
                            float vfb = vfbRegs[ch] * 0.005;  // 5mV per LSB
                            float vout = vfb * 16.04;         // Output voltage divider ratio
                            float iled = (iledRegs[ch] * 0.005) / 4.0;  // Exactly as in original code
                            float iled_ma = iled * 1000.0;    // Convert to mA
                            
                            Serial.print(vout, 1);
                            Serial.print(F("     "));
                            Serial.print(iled_ma, 1);
                            Serial.print(F("     "));
                            
                            if(iled_ma > 125.0) Serial.println(F("OVER"));
                            else if(iled_ma > 5.0) Serial.println(F("OK"));
                            else Serial.println(F("LOW"));
                        } else {
                            Serial.println(F("    --       --      OFF"));
                        }
                    }
                }
                Serial.println(F("------------------------------------------------"));
                break;
            }

            case '3': // INPH Configuration (moved from LED menu)
            {
                Serial.println(F("\nConfigure In-Phase Mode (INPH)"));
                Serial.println(F("Apply to:\n1. All Channels\n2. White\n3. Blue\n4. Green\n5. Red"));
                Serial.print(F("Enter choice: "));
                uint8_t channel_choice = readTerminalDecimal();
                if(channel_choice == 255 || channel_choice < 1 || channel_choice > 5) {
                    Serial.println(F("\n***********Invalid input***********"));
                    break;
                }
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
                    case 2: // White
                        cfg_address_start = CFG1;
                        cfg_address_end = CFG1;
                        break;
                    case 3: // Blue
                        cfg_address_start = CFG2;
                        cfg_address_end = CFG2;
                        break;
                    case 4: // Green
                        cfg_address_start = CFG3;
                        cfg_address_end = CFG3;
                        break;
                    case 5: // Red
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
                        uint8_t cfg_value = 0x03; // Start with both DIMEN and ICTRL enabled
                        if (set_inph)
                            cfg_value |= (1 << 3); // Add INPH bit if requested
                        lt3966_i2c_write(current_led_address, addr, cfg_value);
                        Serial.print(F("Channel "));
                        Serial.print((addr - CFG1) / 0x10 + 1);
                        Serial.println(F(" Configuration Updated."));
                    }
                }
                break;
            }
            
            default:
                Serial.println(F("Invalid selection"));
                break;
        }
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
        // Turn off - use pattern from original code
        lt3966_i2c_write_rs(address, dimh_reg, 0b11100000, diml_reg, 0x00);
    }
    else if(percentage >= 100.0f) {
        // Full on - use pattern from original code
        lt3966_i2c_write_rs(address, dimh_reg, 0b11111111, diml_reg, 0xFF);
    }
    else {
        uint8_t scl = 7;  // Use maximum resolution like original code
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

//update and see
//working now?