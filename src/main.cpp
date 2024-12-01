#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include "Linduino.h"
#include "LT3966.h"
#include <EEPROM.h>
#include "RTClib.h"

// New LED module addresses - ADD THESE BEFORE setup()
#define LT3966_ADD1  0b1111   // ADD1: VCC, ADD2: VCC (0x5F)
#define LT3966_ADD2  0b0011   // ADD1: VCC, ADD2: FLOAT (0x57)  New module ADD1: Flaot, ADD2: GND (0x53;0011)
#define LT3966_ADD3  0b0001   // ADD1: FLOAT, ADD2: GND (0x51)
#define LT3966_ADD4  0b0101   // ADD1: FLOAT, ADD2: FLOAT (0x55)

// Add global variable for current LED
uint8_t current_led_address = LT3966_ADD1; // Default to first LED
bool debug_mode = true;  // Enable debugging output

// Add with other function declarations (around line 27)
void initializeLogging();
void loadSettings();
void handleMainMenu(uint8_t buttons);
void handleQuickControls(uint8_t buttons);
void handleIndividualControl(uint8_t buttons);
void handleStatusSettings(uint8_t buttons);
void updateDisplay();
void displayErrorStatus();
void displayVoltageLevels();
void displayTemperature();
void displayCurrentSettings();

// Original function declarations
float readfloatTerminal();
uint8_t readTerminalDecimal();
void printPaddedBinary(uint8_t value);
void verifyConfiguration(uint8_t address);
uint16_t calculatePWM(float duty_cycle, uint8_t scl);
bool verifyI2CWriteRS(uint8_t address, uint8_t reg1, uint8_t value1, uint8_t reg2, uint8_t value2);
bool verifyI2CWrite(uint8_t address, uint8_t reg, uint8_t value);

// 1. Improved Serial Input Handling
#define SERIAL_TIMEOUT 10000  // 10 second timeout
#define INPUT_BUFFER_SIZE 32

char* readSerialUntilEnter() {
    static char buffer[INPUT_BUFFER_SIZE];
    unsigned long startTime = millis();
    uint8_t index = 0;
    
    // Clear buffer
    memset(buffer, 0, INPUT_BUFFER_SIZE);
    
    while ((millis() - startTime) < SERIAL_TIMEOUT) {
        if (Serial.available()) {
            char c = Serial.read();
            
            if (c == '\r' || c == '\n') {
                if (index > 0) {
                    buffer[index] = '\0';
                    return buffer;
                }
            }
            else if (c == '\b' && index > 0) {  // Backspace
                index--;
                Serial.print("\b \b");
            }
            else if (index < (INPUT_BUFFER_SIZE - 1) && isprint(c)) {
                buffer[index++] = c;
                Serial.print(c);
            }
        }
    }
    
    // Timeout occurred
    return NULL;
}

// Input validation for numerical values
bool parseFloat(const char* input, float& result) {
    result = atof(input);
    return (input[0] != '\0' && isdigit(input[0]));  // Basic validation
}

// LCD instance
Adafruit_RGBLCDShield lcd;

// LCD backlight color definitions
#define WHITE 0x7
#define RED   0x1
#define GREEN 0x2
#define BLUE  0x4
#define YELLOW (RED | GREEN)

// Time structure for better readability
struct TimeStamp {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
};


// Running statistics structure
struct RunningStats {
    struct ChannelStats {
        bool is_active;
        uint8_t current_level;
        uint8_t config;
        TimeStamp total_runtime;
        unsigned long start_time;
    } channels[4][4];
};

// Forward declare checksum calculation
struct SavedSettings;
uint16_t calculateChecksum(SavedSettings* settings);

// Settings structure
struct SavedSettings {
    float brightness;
    uint8_t pwmResolution;
    uint16_t checksum;
} settings;

// Menu state definitions
enum MenuState {
    MAIN_MENU,
    QUICK_CONTROLS,
    INDIVIDUAL_CONTROL,
    STATUS_SETTINGS
};

// Global variables
MenuState currentMenu = MenuState::MAIN_MENU;
uint8_t menuPosition = 0;
float currentBrightness = 100.0;
uint8_t currentPWMResolution = 13;
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_REFRESH_RATE = 100;
uint8_t individualLED = 0;
uint8_t individualChannel = 0;
bool inChannelSelect = false;
float individualBrightness = 100.0;
uint8_t quickControlPosition = 0;
uint8_t statusPosition = 0;

// Forward declare classes
class LogManager;

// Global instances
extern LogManager logManager;
extern RunningStats stats;

RTC_DS3231 rtc;  // Create RTC object

void setup() {
    Wire.begin();
    Serial.begin(9600);
    lcd.begin(16, 2);
    lcd.setBacklight(WHITE);
    
    if (!rtc.begin()) {
        if (debug_mode) Serial.println("Couldn't find RTC");
    }
    
    if (rtc.lostPower()) {
        // RTC lost power, set to compile time
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    // Initialize all LED modules
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    for(uint8_t i = 0; i < 4; i++) {
        lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);  // All channels off initially
    }
    
    initializeLogging();
    loadSettings();
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
    static unsigned long lastButtonCheck = 0;
    const unsigned long buttonDelay = 150;  // Debounce delay
    
    uint8_t buttons = lcd.readButtons();
    
    if (buttons && (millis() - lastButtonCheck > buttonDelay)) {
        lastButtonCheck = millis();
        
        switch(currentMenu) {
            case MAIN_MENU:
                handleMainMenu(buttons);
                break;
            case QUICK_CONTROLS:
                handleQuickControls(buttons);
                break;
            case INDIVIDUAL_CONTROL:
                handleIndividualControl(buttons);
                break;
            case STATUS_SETTINGS:
                handleStatusSettings(buttons);
                break;
        }
    }
    
    // Update display if needed
    updateDisplay();
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
void setPWMDutyCycle(uint8_t address, uint8_t channel, float duty_cycle) {
    uint8_t scl = currentPWMResolution - 6;  // Convert resolution to SCL value
    uint16_t pwm_value = calculatePWM(duty_cycle, scl);
    
    uint8_t dimh_reg = DIM1H + (channel * 0x10);
    uint8_t diml_reg = DIM1L + (channel * 0x10);
    
    uint8_t dimh = (scl << 5) | ((pwm_value >> 8) & 0x1F);
    uint8_t diml = pwm_value & 0xFF;
    
    verifyI2CWriteRS(address, dimh_reg, dimh, diml_reg, diml);
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

void handleIndividualControl(uint8_t buttons) {
    const char* led_names[] = {"LED 1", "LED 2", "LED 3", "LED 4"};
    const char* channel_names[] = {"White", "Blue", "Green", "Red"};
    
    if (buttons & BUTTON_LEFT) {
        if (inChannelSelect) {
            inChannelSelect = false;
        } else {
            currentMenu = MAIN_MENU;
            menuPosition = 0;
            return;
        }
    }
    
    if (!inChannelSelect) {
        // LED module selection
        if (buttons & BUTTON_UP) {
            if (individualLED > 0) individualLED--;
        }
        if (buttons & BUTTON_DOWN) {
            if (individualLED < 3) individualLED++;
        }
        if (buttons & BUTTON_SELECT) {
            inChannelSelect = true;
        }
    } else {
        // Channel control
        if (buttons & BUTTON_UP) {
            if (individualChannel > 0) individualChannel--;
        }
        if (buttons & BUTTON_DOWN) {
            if (individualChannel < 3) individualChannel++;
        }
        if (buttons & BUTTON_RIGHT) {
            individualBrightness = min(100.0f, individualBrightness + 5.0f);
            uint8_t address = (individualLED == 0) ? LT3966_ADD1 : 
                             (individualLED == 1) ? LT3966_ADD2 :
                             (individualLED == 2) ? LT3966_ADD3 : LT3966_ADD4;
            setPWMDutyCycle(address, individualChannel, individualBrightness);
        }
        if (buttons & BUTTON_LEFT) {
            individualBrightness = max(0.0f, individualBrightness - 5.0f);
            uint8_t address = (individualLED == 0) ? LT3966_ADD1 : 
                             (individualLED == 1) ? LT3966_ADD2 :
                             (individualLED == 2) ? LT3966_ADD3 : LT3966_ADD4;
            setPWMDutyCycle(address, individualChannel, individualBrightness);
        }
    }
    
    // Update display
    lcd.clear();
    if (!inChannelSelect) {
        lcd.print("> " + String(led_names[individualLED]));
        if (individualLED < 3) {
            lcd.setCursor(0, 1);
            lcd.print("  " + String(led_names[individualLED + 1]));
        }
    } else {
        lcd.print(String(led_names[individualLED]) + ":");
        lcd.print(String(channel_names[individualChannel]));
        lcd.setCursor(0, 1);
        lcd.print("Bright: " + String(individualBrightness, 1) + "%");
    }
}

void handleStatusSettings(uint8_t buttons) {
    static uint8_t statusPage = 0;
    static uint8_t currentLED = 0;
    const char* status_items[] = {"View Status", "PWM Resolution", "Monitor Current"};
    const uint8_t status_size = 3;
    
    if (buttons & BUTTON_LEFT) {
        currentMenu = MAIN_MENU;
        menuPosition = 0;
        return;
    }
    
    if (buttons & BUTTON_UP) {
        if (statusPosition > 0) statusPosition--;
    }
    if (buttons & BUTTON_DOWN) {
        if (statusPosition < status_size - 1) statusPosition++;
    }
    
    if (buttons & BUTTON_SELECT) {
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        uint8_t led_current;
        
        switch(statusPosition) {
            case 0: // View Status
                if (buttons & BUTTON_RIGHT) statusPage = (statusPage + 1) % 4;
                
                lcd.clear();
                switch(statusPage) {
                    case 0: // Error Status
                        displayErrorStatus();
                        break;
                    case 1: // Voltage Levels
                        displayVoltageLevels();
                        break;
                    case 2: // Temperature
                        displayTemperature();
                        break;
                    case 3: // Current Settings
                        displayCurrentSettings();
                        break;
                }
                break;
                
            case 1: // PWM Resolution
                if (buttons & BUTTON_RIGHT) {
                    currentPWMResolution = min(13, currentPWMResolution + 1);
                    for(uint8_t i = 0; i < 4; i++) {
                        setPWMDutyCycle(addresses[i], 0, currentBrightness);
                    }
                }
                break;
                
            case 2: // Monitor Current
                if (buttons & BUTTON_RIGHT) {
                    lt3966_i2c_read(addresses[currentLED], ILED1, &led_current);
                    float current_ma = (led_current / 255.0) * 120.0;
                    lcd.print("LED" + String(currentLED+1) + " Current:");
                    lcd.setCursor(0, 1);
                    lcd.print(String(current_ma, 1) + "mA");
                }
                break;
        }
    }
    
    // Update display
    lcd.clear();
    lcd.print("> " + String(status_items[statusPosition]));
    if(statusPosition < status_size - 1) {
        lcd.setCursor(0, 1);
        lcd.print("  " + String(status_items[statusPosition + 1]));
    }
}

// Add before loop()
void handleMainMenu(uint8_t buttons) {
    const char* menu_items[] = {"Quick Controls", "Individual Ctrl", "Status/Settings"};
    const uint8_t menu_size = 3;
    
    if (buttons & BUTTON_UP) {
        if (menuPosition > 0) menuPosition--;
    }
    if (buttons & BUTTON_DOWN) {
        if (menuPosition < menu_size - 1) menuPosition++;
    }
    if (buttons & BUTTON_SELECT) {
        switch(menuPosition) {
            case 0: currentMenu = QUICK_CONTROLS; break;
            case 1: currentMenu = INDIVIDUAL_CONTROL; break;
            case 2: currentMenu = STATUS_SETTINGS; break;
        }
        menuPosition = 0;
        return;
    }
    
    lcd.clear();
    lcd.print("> " + String(menu_items[menuPosition]));
    if (menuPosition < menu_size - 1) {
        lcd.setCursor(0, 1);
        lcd.print("  " + String(menu_items[menuPosition + 1]));
    }
}

void handleQuickControls(uint8_t buttons) {
    if (buttons & BUTTON_LEFT) {
        currentMenu = MAIN_MENU;
        menuPosition = 0;
        return;
    }
    
    if (buttons & BUTTON_UP) {
        if (quickControlPosition > 0) quickControlPosition--;
    }
    if (buttons & BUTTON_DOWN) {
        if (quickControlPosition < 1) quickControlPosition++;
    }
    
    if (buttons & BUTTON_SELECT) {
        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
        switch(quickControlPosition) {
            case 0: // All On/Off
                for(uint8_t i = 0; i < 4; i++) {
                    lt3966_i2c_write(addresses[i], GLBCFG, (buttons & BUTTON_RIGHT) ? 0x00 : 0x0F);
                }
                break;
            case 1: // Global Brightness
                currentBrightness = min(100.0f, currentBrightness + ((buttons & BUTTON_RIGHT) ? 5.0f : -5.0f));
                for(uint8_t i = 0; i < 4; i++) {
                    for(uint8_t ch = 0; ch < 4; ch++) {
                        setPWMDutyCycle(addresses[i], ch, currentBrightness);
                    }
                }
                break;
        }
    }
    
    lcd.clear();
    lcd.print(quickControlPosition == 0 ? "> All On/Off" : "> Global Bright");
    lcd.setCursor(0, 1);
    lcd.print(quickControlPosition == 0 ? "  Global Bright" : String(currentBrightness, 1) + "%");
}

void updateDisplay() {
    if (millis() - lastDisplayUpdate < DISPLAY_REFRESH_RATE) {
        return;  // Skip if not enough time has passed
    }
    lastDisplayUpdate = millis();
    
    // Update based on current menu state
    switch(currentMenu) {
        case MAIN_MENU:
            // Main menu is already handled in handleMainMenu
            break;
        case QUICK_CONTROLS:
        case INDIVIDUAL_CONTROL:
        case STATUS_SETTINGS:
            // These will be implemented in Phase 2
            break;
    }
}

// Add to all I2C operations
bool performI2COperation(uint8_t address, const char* operation) {
    if (Wire.endTransmission() != 0) {
        lcd.setBacklight(RED);
        lcd.clear();
        lcd.print("I2C Error:");
        lcd.setCursor(0, 1);
        lcd.print(operation);
        delay(1000);
        lcd.setBacklight(WHITE);
        return false;
    }
    return true;
}

// Add these function implementations
uint16_t calculateChecksum(SavedSettings* settings) {
    uint16_t checksum = 0;
    uint8_t* ptr = (uint8_t*)settings;
    for(size_t i = 0; i < sizeof(SavedSettings) - sizeof(uint16_t); i++) {
        checksum ^= ptr[i];
    }
    return checksum;
}

void displayErrorStatus() {
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    bool found_error = false;
    
    for(uint8_t i = 0; i < 4; i++) {
        uint8_t status;
        if(!lt3966_i2c_read(addresses[i], STAT1 + (i * 0x10), &status)) {
            lcd.print("I2C ERR LED" + String(i+1));
            found_error = true;
            break;
        }
        if(status & 0x0F) {
            lcd.print("ERR LED" + String(i+1) + ":");
            lcd.setCursor(0, 1);
            if(status & 0x01) lcd.print("OVFB ");
            if(status & 0x02) lcd.print("OPEN ");
            if(status & 0x04) lcd.print("SHORT ");
            if(status & 0x08) lcd.print("OC");
            found_error = true;
            break;
        }
    }
    
    if(!found_error) {
        lcd.print("All Systems OK");
    }
}

void displayVoltageLevels() {
    static uint8_t led_idx = 0;
    lcd.print("LED" + String(led_idx+1));
    lcd.setCursor(0, 1);
    lcd.print("PWM: " + String(currentPWMResolution));
}

void displayTemperature() {
    // Note: This is a placeholder as temperature monitoring 
    // requires additional hardware setup
    lcd.print("Temp Monitor");
    lcd.setCursor(0, 1);
    lcd.print("Not Available");
}

void displayCurrentSettings() {
    lcd.print("PWM Res: " + String(currentPWMResolution));
    lcd.setCursor(0, 1);
    lcd.print("Bright: " + String(currentBrightness) + "%");
}

// Simplify ChannelLog
struct ChannelLog {
    TimeStamp runtime;          // 3 bytes
    uint8_t current_level;      // 1 byte
    uint8_t config;            // 1 byte (PWM resolution)
}; // Total: 5 bytes per entry

class LogManager {
private:
    static const uint16_t EEPROM_START_ADDR = 0;
    static const uint16_t LOGS_PER_CHANNEL = 5;
    ChannelLog current_logs[4][4];

public:
    LogManager() {} // Add constructor
    
    uint16_t getChannelBaseAddr(uint8_t led, uint8_t channel);
    void saveLogToEEPROM(uint16_t addr, ChannelLog& log);
    void logChannelEvent(uint8_t led, uint8_t channel, bool is_start);
};

// Global instances
LogManager logManager;
RunningStats stats;

// Helper functions for time management
void updateChannelRuntime(uint8_t led, uint8_t channel, RunningStats& stats) {
    if(stats.channels[led][channel].is_active) {
        uint32_t current = millis();
        uint32_t elapsed = (current - stats.channels[led][channel].start_time) / 1000;
        
        stats.channels[led][channel].total_runtime.hours = elapsed / 3600;
        stats.channels[led][channel].total_runtime.minutes = (elapsed % 3600) / 60;
        stats.channels[led][channel].total_runtime.seconds = elapsed % 60;
    }
}

// Simplify monitoring function
void monitorChannelStatus() {
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 1000) return;  // Check every second
    lastCheck = millis();
    
    for (uint8_t led = 0; led < 4; led++) {
        for (uint8_t channel = 0; channel < 4; channel++) {
            if (stats.channels[led][channel].is_active) {
                // Only update runtime
                updateChannelRuntime(led, channel, stats);
            }
        }
    }
}

// Add these missing function declarations at the top
TimeStamp getCurrentTime();
String formatTime(TimeStamp time);
uint8_t getAddressForLED(uint8_t led);
void readLogFromEEPROM(uint16_t addr, ChannelLog* log);
uint16_t calculateChecksum(ChannelLog* log);

// Add implementations
TimeStamp getCurrentTime() {
    uint32_t current = millis() / 1000; // Convert to seconds
    TimeStamp time;
    time.hours = current / 3600;
    time.minutes = (current % 3600) / 60;
    time.seconds = current % 60;
    return time;
}

String formatTime(TimeStamp time) {
    char buffer[9];
    sprintf(buffer, "%02d:%02d:%02d", time.hours, time.minutes, time.seconds);
    return String(buffer);
}

uint8_t getAddressForLED(uint8_t led) {
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    return addresses[led];
}

void readLogFromEEPROM(uint16_t addr, ChannelLog* log) {
    EEPROM.get(addr, *log);
}

uint16_t calculateChecksum(ChannelLog* log) {
    uint16_t checksum = 0;
    uint8_t* ptr = (uint8_t*)log;
    for(size_t i = 0; i < sizeof(ChannelLog) - sizeof(uint16_t); i++) {
        checksum ^= ptr[i];
    }
    return checksum;
}

void initializeLogging() {
    // Initialize all channels as inactive
    for(uint8_t led = 0; led < 4; led++) {
        for(uint8_t channel = 0; channel < 4; channel++) {
            stats.channels[led][channel].is_active = false;
            stats.channels[led][channel].current_level = 0;
            stats.channels[led][channel].config = 0;
            memset(&stats.channels[led][channel].total_runtime, 0, sizeof(TimeStamp));
        }
    }
    
    // Verify EEPROM integrity
    if(EEPROM.read(0) == 0xFF) {
        // Initialize EEPROM if never used
        for(uint8_t led = 0; led < 4; led++) {
            for(uint8_t channel = 0; channel < 4; channel++) {
                uint16_t base_addr = logManager.getChannelBaseAddr(led, channel);
                EEPROM.write(base_addr, 0);  // Initialize position counter
            }
        }
    }
}


uint16_t LogManager::getChannelBaseAddr(uint8_t led, uint8_t channel) {
    return EEPROM_START_ADDR + (led * 4 + channel) * LOGS_PER_CHANNEL * sizeof(ChannelLog);
}

void LogManager::saveLogToEEPROM(uint16_t addr, ChannelLog& log) {
    EEPROM.put(addr, log);
}

void LogManager::logChannelEvent(uint8_t led, uint8_t channel, bool is_start) {
    if (is_start) {
        stats.channels[led][channel].start_time = millis();
        stats.channels[led][channel].is_active = true;
    } else {
        stats.channels[led][channel].is_active = false;
        updateChannelRuntime(led, channel, stats);
    }
}

// Add after initializeLogging() function (around line 270)
void loadSettings() {
    EEPROM.get(EEPROM.length() - sizeof(SavedSettings), settings);
    if (settings.checksum == calculateChecksum(&settings)) {
        currentBrightness = settings.brightness;
        currentPWMResolution = settings.pwmResolution;
    } else {
        // Default values if checksum fails
        settings.brightness = 100.0;
        settings.pwmResolution = 13;
        settings.checksum = calculateChecksum(&settings);
        EEPROM.put(EEPROM.length() - sizeof(SavedSettings), settings);
    }
}

// 2. Improved EEPROM Management
class EEPROMManager {
private:
    static const uint16_t EEPROM_SIZE = 1024;  // Arduino UNO
    static const uint16_t WEAR_COUNT_ADDR = 0;
    static const uint16_t DATA_START_ADDR = 16;
    static const uint8_t MAX_WEAR_COUNT = 255;  // Limit to uint8_t max value
    
    struct BlockHeader {
        uint16_t checksum;
        uint8_t version;
        uint8_t status;  // 0xFF = empty, 0x00 = active, 0x01 = obsolete
    };
    
    // Calculate CRC16 checksum
    uint16_t calculateCRC(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        return crc;
    }
    
    // Find next available block
    uint16_t findNextBlock() {
        uint16_t addr = DATA_START_ADDR;
        while (addr < EEPROM_SIZE) {
            BlockHeader header;
            EEPROM.get(addr, header);
            if (header.status == 0xFF) {
                return addr;
            }
            addr += sizeof(BlockHeader) + sizeof(ChannelLog);
        }
        return 0;  // No space available
    }

public:
    bool writeLog(ChannelLog& log, uint8_t led, uint8_t channel) {
        noInterrupts();  // Ensure atomic operation
        
        uint16_t addr = findNextBlock();
        if (addr == 0) {
            // No space - need to compact
            compactStorage();
            addr = findNextBlock();
            if (addr == 0) {
                interrupts();
                return false;
            }
        }
        
        BlockHeader header;
        header.version = 1;
        header.status = 0x00;
        header.checksum = calculateCRC((uint8_t*)&log, sizeof(ChannelLog));
        
        // Write header and data
        EEPROM.put(addr, header);
        EEPROM.put(addr + sizeof(BlockHeader), log);
        
        // Update wear count
        uint32_t wearCount;
        EEPROM.get(WEAR_COUNT_ADDR, wearCount);
        if (++wearCount >= MAX_WEAR_COUNT) {
            // Handle wear limit reached
            handleWearLimitReached();
        }
        EEPROM.put(WEAR_COUNT_ADDR, wearCount);
        
        interrupts();
        return true;
    }
    
    bool readLog(ChannelLog& log, uint8_t led, uint8_t channel) {
        uint16_t addr = findLatestLog(led, channel);
        if (addr == 0) return false;
        
        BlockHeader header;
        EEPROM.get(addr, header);
        EEPROM.get(addr + sizeof(BlockHeader), log);
        
        // Verify checksum
        uint16_t calculatedCRC = calculateCRC((uint8_t*)&log, sizeof(ChannelLog));
        return (calculatedCRC == header.checksum);
    }
    
private:
    void compactStorage() {
        // Implement storage compaction
        // Move valid blocks to beginning, erase old blocks
    }
    
    void handleWearLimitReached() {
        // Implement wear limit handling
        // Could rotate blocks, alert user, etc.
    }
    
    uint16_t findLatestLog(uint8_t led, uint8_t channel) {
        // Find most recent valid log for given LED/channel
        return 0;  // Implement actual search
    }
};
