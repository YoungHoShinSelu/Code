#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include "Linduino.h"
#include "LT3966.h"
#include <EEPROM.h>
#include "RTClib.h"
#include <util/crc16.h>  // For CRC calculation
#include <Adafruit_PCT2075.h>


// New LED module addresses - ADD THESE BEFORE setup()
#define LT3966_ADD1  0b1111   // ADD1: VCC, ADD2: VCC (0x5F)
#define LT3966_ADD2  0b0111   // ADD1: VCC, ADD2: FLOAT (0x57)  **LSU module address ADD1: Flaot, ADD2: GND (0x53;0011)
#define LT3966_ADD3  0b0001   // ADD1: FLOAT, ADD2: GND (0x51)
#define LT3966_ADD4  0b0101   // ADD1: FLOAT, ADD2: FLOAT (0x55)

// Add global variable for current LED
uint8_t current_led_address = LT3966_ADD1; // Default to first LED
bool debug_mode = true;  // Enable debugging output

// Add these near the top of the file, after the #includes but before any function declarations

// Forward declare the PWM configuration structures
struct PWMFreqConfig {
    uint8_t scl;
    uint16_t freq;    // Frequency in Hz
    uint8_t bits;     // Resolution in bits
    const char* desc; // Description
};

// Forward declare the PWM functions
void handlePWMFrequencyConfig(uint8_t buttons);
void applyPWMFrequency(uint8_t ledSelect, uint8_t channelSelect, uint8_t freqSelect);
void initializeLED(uint8_t address);

// Then with other function declarations
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
// Add these with other function declarations
void displaySystemStatus();
void displayPWMStatus();
void displayCurrentStatus();

// Add these with other function declarations (around line 50-60)
void displaySystemStatus();
void displayPWMStatus();
void displayCurrentStatus();

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
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// LCD backlight color definitions
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

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

// Add these with other global variables (around line 50-60)
uint8_t statusPage = 0;     // For tracking status display pages
uint8_t currentLED = 0;     // For tracking current LED selection

// Forward declare classes
class LogManager;

// Global instances
extern LogManager logManager;
extern RunningStats stats;

// Define RTC address
#define DS3231_ADDRESS 0x68

// Modify RTC instance declaration
RTC_DS3231 rtc;  // The address is handled internally by the library

// Add before BlockHeader
struct __attribute__((packed)) MetaData {
    uint32_t wearCount;
    uint16_t lastSequence;
    uint16_t checksum;
} __attribute__((aligned(4)));

// Modify existing BlockHeader structure
struct __attribute__((packed)) BlockHeader {
    static const uint32_t MAGIC = 0xEEDB1000;
    static const uint8_t CURRENT_VERSION = 1;
    
    uint32_t magicNumber;
    uint16_t blockSize;
    uint16_t version;
    uint8_t status;
    uint8_t wearCount;
    uint8_t led;
    uint8_t channel;
    uint32_t crc32;
    uint16_t sequence;
    uint16_t reserved;  // For future expansion
    
    bool isValid() const {
        return magicNumber == MAGIC && 
               version <= CURRENT_VERSION &&
               blockSize <= 1024 &&
               blockSize % 32 == 0;
    }
} __attribute__((aligned(4)));

struct __attribute__((packed)) RecoveryBlock {
    uint16_t targetAddr;
    uint8_t operationType;
    uint32_t timestamp;
    uint32_t crc32;
    uint8_t data[16];
} __attribute__((aligned(4)));

// Add after BlockHeader structure
struct __attribute__((packed)) ErrorInfo {
    enum class ErrorType {
        WRITE_FAILURE,
        CRC_MISMATCH,
        ALIGNMENT_ERROR,
        TIMEOUT,
        WEAR_LIMIT,
        POWER_LOSS,
        INIT_FAILURE,
        TEMP_WARNING,
        WEAR_WARNING
    };
    
    ErrorType type;
    uint16_t address;
    uint32_t timestamp;
    uint8_t retryCount;
};

// Add after other structures
struct RuntimeLog {
    DateTime startTime;
    DateTime endTime;
    uint8_t led;
    uint8_t channel;
    float current_mA;
    uint8_t pwmResolution;
    bool inPhaseMode;
    float temperature;
};

class RuntimeTracker {
private:
    static const uint16_t LOG_START_ADDR = 512;  // Use second half of EEPROM
    static const uint8_t LOGS_PER_CHANNEL = 4;   // Store last 4 entries per channel
    
public:
    void logChannelStart(uint8_t led, uint8_t channel, float current) {
        RuntimeLog log;
        log.startTime = rtc.now();
        log.led = led;
        log.channel = channel;
        log.current_mA = current;
        log.pwmResolution = currentPWMResolution;
        log.temperature = rtc.getTemperature();
        
        uint16_t addr = getLogAddress(led, channel);
        EEPROM.put(addr, log);
    }
    
    void logChannelStop(uint8_t led, uint8_t channel) {
        uint16_t addr = getLogAddress(led, channel);
        RuntimeLog log;
        EEPROM.get(addr, log);
        log.endTime = rtc.now();
        EEPROM.put(addr, log);
    }

private:
    uint16_t getLogAddress(uint8_t led, uint8_t channel) {
        return LOG_START_ADDR + 
               (led * 4 + channel) * sizeof(RuntimeLog) * LOGS_PER_CHANNEL;
    }
};

// Add these declarations right after the includes, before any functions
#include <Adafruit_PCT2075.h>

// Global variables and constants (add with other globals)
Adafruit_PCT2075 tempsensor;
static const uint16_t TEMP_LOG_ADDR = 960;
static const float TEMP_WARNING_THRESHOLD = 40.0;  // Move this here too

// Remove these declarations from their current locations:
// #define TEMP_WARNING_THRESHOLD 40.0  // Remove this
// static const uint16_t TEMP_LOG_ADDR = 960;  // Remove this
// Adafruit_PCT2075 tempsensor;  // Remove this

// Forward declare all classes
class SafeEEPROMManager;
class WearLevelingManager;
class ErrorTracker;
class BoundaryManager;
class PowerLossRecovery;
class SystemManager;

// Declare global instance after all class definitions
extern SystemManager systemManager;

// Then define classes in correct order
class SafeEEPROMManager {
public:
    bool verifyWrite(uint16_t addr, const void* data, size_t len);
    bool writeWithVerification(uint16_t addr, const void* data, size_t len);
    
private:
    // Constants
    static const uint16_t BLOCK_SIZE = 32;
    static const uint16_t META_ADDR = 0;
    static const uint16_t DATA_START = 64;
    static const uint16_t RECOVERY_ADDR = 32;
    static const uint32_t WEAR_WARNING_THRESHOLD = 90000;
    static const unsigned long TIMEOUT_MS = 1000;
    
    bool operationInProgress;

    void handleInterruptedOperation();
    void clearRecoveryPoint();
    uint32_t calculateCRC32(const void* data, size_t len);
    void compactStorage();
    void handleWearLimitReached();
    uint16_t findLatestLog(uint8_t led, uint8_t channel);
    uint16_t findNextBlock();
};

class WearLevelingManager {
public:
    bool write(uint16_t addr, const void* data, size_t len);
    void updateWearCount(uint16_t addr);
private:
    uint32_t wearCounts[512];  // Track wear for each block
};

class ErrorTracker {
public:
    void logError(ErrorInfo::ErrorType type, uint16_t address);
    void clearErrors();
    bool hasErrors() const;
private:
    static const uint8_t MAX_ERRORS = 10;
    ErrorInfo errors[MAX_ERRORS];
    uint8_t errorCount;
};

class BoundaryManager {
public:
    bool checkBoundary(uint16_t addr, size_t len);
    void setProtectedRegion(uint16_t start, uint16_t end);
private:
    uint16_t protectedStart;
    uint16_t protectedEnd;
};

class PowerLossRecovery {
public:
    void saveRecoveryPoint(uint16_t addr, const void* data, size_t len);
    bool needsRecovery();
    void performRecovery();
private:
    RecoveryBlock recoveryData;
    bool writeWithVerification(uint16_t addr, const void* data, size_t len);
    uint32_t calculateCRC32(const void* data, size_t len);
};

class SystemManager {
public:
    bool initialize() {
        Wire.begin();
        // Initialize all subsystems
        return true;
    }
    
    void update() {
        // Don't try to read temperature if sensor isn't initialized
        if (tempsensor.begin(0x48)) {  // Only read if sensor exists
            handleTemperature();
        }
        checkErrors();
        updateDisplay();
    }
    
private:
    void handleTemperature();
    void checkErrors();
    void updateDisplay();
    
    SafeEEPROMManager eepromManager;
    WearLevelingManager wearManager;
    ErrorTracker errorTracker;
    BoundaryManager boundaryManager;
    PowerLossRecovery powerLossRecovery;
};

// Define the global instance
SystemManager systemManager;

// Add these constants for PWM configuration
const PWMFreqConfig PWM_CONFIGS[] = {
    {0, 31250,  6, "31.25 kHz  (64 steps)"},
    {1, 15625,  7, "15.625 kHz (128 steps)"},
    {2, 7812,   8, "7.8125 kHz (256 steps)"},
    {3, 3906,   9, "3.906 kHz  (512 steps)"},
    {4, 1953,   10, "1.953 kHz  (1024 steps)"},
    {5, 977,    11, "976.6 Hz   (2048 steps)"},
    {6, 488,    12, "488.3 Hz   (4096 steps)"},
    {7, 244,    13, "244.14 Hz  (8192 steps)"}
};

// Add this function to handle PWM frequency configuration
void handlePWMFrequencyConfig(uint8_t buttons) {
    static uint8_t menuState = 0;  // 0: LED select, 1: Channel select, 2: Frequency select
    static uint8_t selectedLED = 0;
    static uint8_t selectedChannel = 0;
    static uint8_t selectedFreq = 0;

    switch(menuState) {
        case 0:  // LED Selection
            lcd.clear();
            lcd.print("Select LED:");
            lcd.setCursor(0, 1);
            if(selectedLED == 0) {
                lcd.print("All LEDs");
            } else {
                lcd.print("LED ");
                lcd.print(selectedLED);
            }
            
            if(buttons & BUTTON_UP) {
                if(selectedLED > 0) selectedLED--;
            }
            if(buttons & BUTTON_DOWN) {
                if(selectedLED < 4) selectedLED++;
            }
            if(buttons & BUTTON_SELECT) {
                menuState = 1;
            }
            break;
            
        case 1:  // Channel Selection
            lcd.clear();
            lcd.print("Select Channel:");
            lcd.setCursor(0, 1);
            switch(selectedChannel) {
                case 0: lcd.print("All Channels"); break;
                case 1: lcd.print("White"); break;
                case 2: lcd.print("Blue"); break;
                case 3: lcd.print("Green"); break;
                case 4: lcd.print("Red"); break;
            }
            
            if(buttons & BUTTON_UP) {
                if(selectedChannel > 0) selectedChannel--;
            }
            if(buttons & BUTTON_DOWN) {
                if(selectedChannel < 4) selectedChannel++;
            }
            if(buttons & BUTTON_SELECT) {
                menuState = 2;
            }
            break;
            
        case 2:  // Frequency Selection
            lcd.clear();
            lcd.print("PWM Frequency:");
            lcd.setCursor(0, 1);
            lcd.print(PWM_CONFIGS[selectedFreq].freq);
            lcd.print("Hz ");
            lcd.print(PWM_CONFIGS[selectedFreq].bits);
            lcd.print("bit");
            
            if(buttons & BUTTON_UP) {
                if(selectedFreq > 0) selectedFreq--;
            }
            if(buttons & BUTTON_DOWN) {
                if(selectedFreq < 7) selectedFreq++;
            }
            if(buttons & BUTTON_SELECT) {
                // Apply the frequency settings
                applyPWMFrequency(selectedLED, selectedChannel, selectedFreq);
                menuState = 0;  // Return to LED selection
            }
            break;
    }
    
    if(buttons & BUTTON_LEFT && menuState > 0) {
        menuState--;
    }
}

// Function to apply PWM frequency settings
void applyPWMFrequency(uint8_t ledSelect, uint8_t channelSelect, uint8_t freqSelect) {
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    uint8_t scl = PWM_CONFIGS[freqSelect].scl;
    
    // Determine which LEDs to configure
    uint8_t startLED = (ledSelect == 0) ? 0 : ledSelect - 1;
    uint8_t endLED = (ledSelect == 0) ? 3 : ledSelect - 1;
    
    // Determine which channels to configure
    uint8_t startCh = (channelSelect == 0) ? 0 : channelSelect - 1;
    uint8_t endCh = (channelSelect == 0) ? 3 : channelSelect - 1;
    
    // Configure selected LEDs and channels
    for(uint8_t led = startLED; led <= endLED; led++) {
        for(uint8_t ch = startCh; ch <= endCh; ch++) {
            uint8_t dimh_reg = DIM1H + (ch * 0x10);
            uint8_t current_dimh, current_diml;
            
            // Read current settings
            lt3966_i2c_read(addresses[led], dimh_reg, &current_dimh);
            lt3966_i2c_read(addresses[led], dimh_reg + 1, &current_diml);
            
            // Preserve duty cycle while changing frequency
            uint8_t old_scl = (current_dimh >> 5) & 0x07;
            uint16_t old_period = (1 << (6 + old_scl));
            uint16_t old_value = ((current_dimh & 0x1F) << 8) | current_diml;
            float duty_cycle = (float)old_value / (old_period - 1) * 100.0;
            
            // Calculate new PWM values
            uint16_t new_period = (1 << (6 + scl));
            uint16_t new_value = (uint16_t)((duty_cycle / 100.0) * (new_period - 1));
            
            uint8_t dim_high = ((scl & 0x07) << 5) | ((new_value >> 8) & 0x1F);
            uint8_t dim_low = new_value & 0xFF;
            
            // Update with verification
            verifyI2CWriteRS(addresses[led], dimh_reg, dim_high, dimh_reg + 1, dim_low);
        }
    }
}

// First, let's add debug flags and I2C address check
#define LCD_I2C_ADDR 0x20  // Default I2C address for LCD shield
#define DEBUG_LCD true

void initializeLED(uint8_t address) {
    // Initialize all channels
    for(uint8_t channel = 0; channel < 4; channel++) {
        uint8_t cfg_reg = CFG1 + (channel * 0x10);
        uint8_t adim_reg = ADIM1 + channel;
        uint8_t dimh_reg = DIM1H + (channel * 0x10);
        uint8_t diml_reg = dimh_reg + 1;
        
        // Enable PWM dimming
        lt3966_i2c_write(address, cfg_reg, 0x01);
        
        // Set analog dimming to maximum
        lt3966_i2c_write(address, adim_reg, 0xFF);
        
        // Initialize PWM to 0
        lt3966_i2c_write_rs(address, dimh_reg, 0xA0, diml_reg, 0x00);  // SCL=5, PWM=0
    }
}

void setup() {
    // Start Serial first for debugging
    Serial.begin(9600);
    while (!Serial && millis() < 3000); // Wait for serial to initialize (with timeout)
    Serial.println(F("Starting initialization..."));
    
    // Initialize I2C first
    Wire.begin();
    delay(100);  // Longer delay after I2C init
    
    // Initialize all LED modules
    initializeLED(LT3966_ADD1);
    initializeLED(LT3966_ADD2);
    initializeLED(LT3966_ADD3);
    initializeLED(LT3966_ADD4);
    
    // Check if LCD is responding - LCD is critical
    Wire.beginTransmission(LCD_I2C_ADDR);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.println(F("LCD found at 0x20"));
    } else {
        Serial.println(F("LCD not found! Error: "));
        Serial.println(error);
        while(1) {  // Only halt if LCD fails since it's critical
            delay(1000);
        }
    }
    
    // Initialize LCD
    Serial.println(F("Initializing LCD..."));
    lcd.begin(16, 2);
    lcd.setBacklight(WHITE);
    lcd.clear();
    lcd.print(F("Initializing..."));
    
    // Initialize temperature sensor - non-critical
    if (!tempsensor.begin(0x48)) {
        Serial.println(F("Couldn't find PCT2075!"));
        lcd.clear();
        lcd.print(F("Temp Sens Error"));
        delay(1000);  // Show error briefly but continue
    } else {
        Serial.println(F("PCT2075 initialized"));
    }
    
    // Check RTC - non-critical
    Wire.beginTransmission(DS3231_ADDRESS);
    if (Wire.endTransmission() == 0) {
        Serial.println(F("RTC found at 0x68"));
        if (!rtc.begin()) {
            Serial.println(F("RTC init failed"));
            lcd.clear();
            lcd.print(F("RTC Error"));
            delay(1000);  // Show error briefly but continue
        }
    } else {
        Serial.println(F("RTC not found"));
        lcd.clear();
        lcd.print(F("No RTC Found"));
        delay(1000);  // Show error briefly but continue
    }
    
    // Initialize system manager
    if (!systemManager.initialize()) {
        Serial.println(F("System init failed"));
        lcd.clear();
        lcd.print(F("System Error"));
        delay(1000);  // Show error briefly but continue
    }
    
    // Show ready status
    lcd.clear();
    lcd.print(F("System Ready"));
    delay(1000);
    
    // Return to main menu
    currentMenu = MAIN_MENU;
    menuPosition = 0;
    
    // Initialize all LED drivers with default PWM frequency
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    for(uint8_t i = 0; i < 4; i++) {
        // Set default PWM frequency (976.6 Hz, SCL=5) for all channels
        for(uint8_t ch = 0; ch < 4; ch++) {
            uint8_t dimh_reg = DIM1H + (ch * 0x10);
            uint8_t diml_reg = dimh_reg + 1;
            uint8_t scl = 5;  // 976.6 Hz with 2048 steps
            
            // Initialize with 0% duty cycle but correct frequency
            uint8_t dim_high = ((scl & 0x07) << 5);  // Set SCL bits, PWM duty cycle bits = 0
            uint8_t dim_low = 0x00;
            lt3966_i2c_write_rs(addresses[i], dimh_reg, dim_high, diml_reg, dim_low);
            
            // Configure channel
            uint8_t cfg_reg = CFG1 + (ch * 0x10);
            lt3966_i2c_write(addresses[i], cfg_reg, 0x03);  // Enable DIMEN and ICTRL
            
            // Set ADIM to maximum
            lt3966_i2c_write(addresses[i], ADIM1 + ch, 0xFF);
        }
    }
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
    static unsigned long lastUpdate = 0;
    static unsigned long lastButtonCheck = 0;
    static uint8_t lastButtons = 0;  // Add this to track previous button state
    const unsigned long buttonDelay = 150;  // Debounce delay
    
    uint8_t buttons = lcd.readButtons();
    uint8_t newButtons = buttons & ~lastButtons;  // Only detect new button presses
    
    // Handle button inputs with debounce
    if (newButtons && (millis() - lastButtonCheck > buttonDelay)) {  // Changed from buttons to newButtons
        lastButtonCheck = millis();
        
        switch(currentMenu) {
            case MAIN_MENU:
                handleMainMenu(newButtons);  // Pass only new button presses
                break;
            case QUICK_CONTROLS:
                handleQuickControls(newButtons);
                break;
            case INDIVIDUAL_CONTROL:
                handleIndividualControl(newButtons);
                break;
            case STATUS_SETTINGS:
                handleStatusSettings(newButtons);
                break;
        }
        
        // Reset display update timer after button press
        lastUpdate = millis();
    }
    
    lastButtons = buttons;  // Update last button state

    // Only update display if no button was pressed recently
    if (millis() - lastUpdate > 5000) {
        updateDisplay();
        lastUpdate = millis();
    }
}

// Calculate PWM values based on percentage
uint16_t calculatePWM(float percentage, uint8_t scl) {
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;

    uint16_t period = 1 << (6 + scl);
    uint16_t max_pwm_value = period - 1;

    uint32_t value = (percentage / 100.0f) * max_pwm_value + 0.5f;  // Add 0.5 for rounding

    if (value > max_pwm_value) {
        value = max_pwm_value;
    }

    return (uint16_t)value;
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
    static uint8_t selectedLED = 0;      // 0-3 for LED1-4
    static uint8_t selectedChannel = 0;   // 0-3 for WHITE/BLUE/GREEN/RED
    static bool inChannelMode = false;    // false = LED select, true = Channel control
    static float channelBrightness[4][4] = {{0}};  // Store brightness for each LED/channel
    const float MAX_ACTUAL_BRIGHTNESS = 12.0;  // Maximum actual brightness percentage due to hardware limitations
    const float BRIGHTNESS_STEP = 1.0;         // 1% per step
    const char* channel_names[] = {"WHITE", "BLUE", "GREEN", "RED"};
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};

    // Handle back button
    if (buttons & BUTTON_LEFT) {
        if (inChannelMode) {
            // Return to LED selection
            inChannelMode = false;
            lcd.clear();
            lcd.print(F("Select LED:"));
            lcd.setCursor(0, 1);
            lcd.print(F("LED "));
            lcd.print(selectedLED + 1);
        } else {
            // Return to main menu
            currentMenu = MAIN_MENU;
            menuPosition = 1;  // Position at "Indiv. Control"
            lcd.clear();
            lcd.print(F("== Main Menu =="));
            lcd.setCursor(0, 1);
            lcd.print(F(">Indiv. Control"));
        }
        return;
    }

    if (!inChannelMode) {
        // LED Selection Mode
        if (buttons & BUTTON_UP) {
            if (selectedLED > 0) selectedLED--;
            lcd.clear();
            lcd.print(F("Select LED:"));
            lcd.setCursor(0, 1);
            lcd.print(F("LED "));
            lcd.print(selectedLED + 1);
        }
        if (buttons & BUTTON_DOWN) {
            if (selectedLED < 3) selectedLED++;
            lcd.clear();
            lcd.print(F("Select LED:"));
            lcd.setCursor(0, 1);
            lcd.print(F("LED "));
            lcd.print(selectedLED + 1);
        }
        if (buttons & BUTTON_SELECT) {
            inChannelMode = true;
            selectedChannel = 0;  // Start with WHITE channel
            // Immediately show channel control
            lcd.clear();
            lcd.print(F("LED"));
            lcd.print(selectedLED + 1);
            lcd.print(F(" "));
            lcd.print(channel_names[selectedChannel]);
            lcd.setCursor(0, 1);
            lcd.print(F("Bright: "));
            lcd.print(channelBrightness[selectedLED][selectedChannel], 1);
            lcd.print(F("%"));
        }
    } else {
        // Channel Control Mode
        if (buttons & BUTTON_UP || buttons & BUTTON_DOWN) {
            // Change channel
            if (buttons & BUTTON_UP) {
                if (selectedChannel > 0) selectedChannel--;
            }
            if (buttons & BUTTON_DOWN) {
                if (selectedChannel < 3) selectedChannel++;
            }
        }

        // Increase brightness
        if (buttons & BUTTON_RIGHT) {
            if (channelBrightness[selectedLED][selectedChannel] < 100.0) {
                channelBrightness[selectedLED][selectedChannel] += BRIGHTNESS_STEP;
                if (channelBrightness[selectedLED][selectedChannel] > 100.0) {
                    channelBrightness[selectedLED][selectedChannel] = 100.0;
                }

                // Note: Due to hardware limitations, the actual brightness range is 0% to 12%.
                // We scale the user interface brightness (0% to 100%) to this range.
                // In the future, if the hardware supports the full range, remove or adjust this scaling.
                float actualBrightness = (channelBrightness[selectedLED][selectedChannel] / 100.0f) * MAX_ACTUAL_BRIGHTNESS;

                // Use actualBrightness in calculatePWM
                uint8_t dimh_reg = DIM1H + (selectedChannel * 0x10);
                uint8_t diml_reg = dimh_reg + 1;
                uint8_t scl = 5;  // PWM scaling factor

                uint16_t pwm_value = calculatePWM(actualBrightness, scl);
                uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
                uint8_t dim_low = pwm_value & 0xFF;

                // Enable channel if brightness > 0
                if (channelBrightness[selectedLED][selectedChannel] > 0) {
                    lt3966_i2c_write(addresses[selectedLED], GLBCFG, 0x00);
                    lt3966_i2c_write(addresses[selectedLED], ADIM1 + selectedChannel, 0xFF);
                }

                lt3966_i2c_write_rs(addresses[selectedLED], dimh_reg, dim_high, diml_reg, dim_low);
            }
        }

        // Decrease brightness
        if (buttons & BUTTON_LEFT) {
            if (channelBrightness[selectedLED][selectedChannel] > 0.0) {
                channelBrightness[selectedLED][selectedChannel] -= BRIGHTNESS_STEP;
                if (channelBrightness[selectedLED][selectedChannel] < 0.0) {
                    channelBrightness[selectedLED][selectedChannel] = 0.0;
                }

                // Note: Due to hardware limitations, the actual brightness range is 0% to 12%.
                // We scale the user interface brightness (0% to 100%) to this range.
                // In the future, if the hardware supports the full range, remove or adjust this scaling.
                float actualBrightness = (channelBrightness[selectedLED][selectedChannel] / 100.0f) * MAX_ACTUAL_BRIGHTNESS;

                // Use actualBrightness in calculatePWM
                uint8_t dimh_reg = DIM1H + (selectedChannel * 0x10);
                uint8_t diml_reg = dimh_reg + 1;
                uint8_t scl = 5;  // PWM scaling factor

                uint16_t pwm_value = calculatePWM(actualBrightness, scl);
                uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
                uint8_t dim_low = pwm_value & 0xFF;

                // Disable channel if brightness is 0
                if (channelBrightness[selectedLED][selectedChannel] == 0) {
                    lt3966_i2c_write(addresses[selectedLED], GLBCFG, 0x0F);
                }

                lt3966_i2c_write_rs(addresses[selectedLED], dimh_reg, dim_high, diml_reg, dim_low);
            }
        }

        if (buttons & BUTTON_SELECT) {
            // Turn channel completely off
            channelBrightness[selectedLED][selectedChannel] = 0;
            lt3966_i2c_write(addresses[selectedLED], GLBCFG, 0x0F);
        }

        // Update channel control display
            lcd.clear();
            lcd.print(F("LED"));
            lcd.print(selectedLED + 1);
            lcd.print(F(" "));
            lcd.print(channel_names[selectedChannel]);
            lcd.setCursor(0, 1);
            lcd.print(F("Bright: "));
            lcd.print(channelBrightness[selectedLED][selectedChannel], 1);
            lcd.print(F("%"));
        }
}

void handleStatusSettings(uint8_t buttons) {
    static uint8_t statusPage = 0;
    static uint8_t currentLED = 0;
    const char* status_items[] = {"View Status", "PWM Resolution", "Monitor Current"};
    const uint8_t status_size = 3;
    static bool inStatusView = false;

    if (buttons & BUTTON_LEFT) {
        if (inStatusView) {
            // Return to status menu
            inStatusView = false;
            lcd.clear();
            lcd.print(F("> View Status"));
            lcd.setCursor(0, 1);
            lcd.print(F("  PWM Resolution"));
        } else {
            // Return to main menu
            currentMenu = MAIN_MENU;
            menuPosition = 2;
            lcd.clear();
            lcd.print(F("=== Main Menu ==="));
            lcd.setCursor(0, 1);
            lcd.print(F(">Status/Settings"));
        }
        return;
    }

    if (!inStatusView) {
        // Main status menu navigation
        if (buttons & BUTTON_UP) {
            if (statusPosition > 0) {
                statusPosition--;
                lcd.clear();
                lcd.print(F("> "));
                lcd.print(status_items[statusPosition]);
                if(statusPosition < status_size - 1) {
                    lcd.setCursor(0, 1);
                    lcd.print(F("  "));
                    lcd.print(status_items[statusPosition + 1]);
                }
            }
        }
        if (buttons & BUTTON_DOWN) {
            if (statusPosition < status_size - 1) {
                statusPosition++;
                lcd.clear();
                lcd.print(F("> "));
                lcd.print(status_items[statusPosition]);
                if(statusPosition < status_size - 1) {
                    lcd.setCursor(0, 1);
                    lcd.print(F("  "));
                    lcd.print(status_items[statusPosition + 1]);
                }
            }
        }
        if (buttons & BUTTON_SELECT) {
            inStatusView = true;
            statusPage = 0;
            switch(statusPosition) {
                case 0: // View Status
                    displaySystemStatus();
                    break;
                case 1: // PWM Resolution
                    displayPWMStatus();
                    break;
                case 2: // Monitor Current
                    displayCurrentStatus();
                    break;
            }
        }
    } else {
        // Status view navigation
        if (buttons & BUTTON_RIGHT) {
            switch(statusPosition) {
                case 0: // System Status Pages
                    statusPage = (statusPage + 1) % 3;
                    displaySystemStatus();
                    break;
                case 2: // Current Monitor
                    currentLED = (currentLED + 1) % 4;
                    displayCurrentStatus();
                    break;
            }
        }
    }
}

// New status display functions
void displaySystemStatus() {
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    lcd.clear();
    
    switch(statusPage) {
        case 0: // LED Status
            lcd.print(F("LED Status >"));
            lcd.setCursor(0, 1);
            bool anyError = false;
            for(uint8_t i = 0; i < 4; i++) {
                uint8_t status;
                if(lt3966_i2c_read(addresses[i], STAT1 + (i * 0x10), &status)) {
                    if(status & 0x0F) {
                        lcd.print(i+1);
                        lcd.print(F("!"));
                        anyError = true;
                    } else {
                        lcd.print(i+1);
                        lcd.print(F("√"));
                    }
                } else {
                    lcd.print(i+1);
                    lcd.print(F("X"));
                    anyError = true;
                }
                lcd.print(" ");
            }
            if(anyError) {
                lcd.setBacklight(RED);
            } else {
                lcd.setBacklight(GREEN);
            }
            break;

        case 1: // Temperature
            lcd.print(F("Temp Status >"));
            lcd.setCursor(0, 1);
            if(tempsensor.begin()) {
                float temp = tempsensor.getTemperature();
                lcd.print(temp, 1);
                lcd.print(F("C "));
                if(temp > TEMP_WARNING_THRESHOLD) {
                    lcd.print(F("HIGH!"));
                    lcd.setBacklight(RED);
                } else {
                    lcd.print(F("OK"));
                    lcd.setBacklight(GREEN);
                }
            } else {
                lcd.print(F("Sensor Error"));
                lcd.setBacklight(RED);
            }
            break;

        case 2: // System Info
            lcd.print(F("System Info >"));
            lcd.setCursor(0, 1);
            lcd.print(F("PWM:"));
            lcd.print(currentPWMResolution);
            lcd.print(F("b "));
            lcd.print(PWM_CONFIGS[5].freq);
            lcd.print(F("Hz"));
            lcd.setBacklight(WHITE);
            break;
    }
}

void displayPWMStatus() {
    lcd.clear();
    lcd.print(F("PWM Settings"));
    lcd.setCursor(0, 1);
    lcd.print(F("Res:"));
    lcd.print(currentPWMResolution);
    lcd.print(F("-bit "));
    lcd.print(PWM_CONFIGS[5].freq);
    lcd.print(F("Hz"));
}

void displayCurrentStatus() {
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    lcd.clear();
    lcd.print(F("LED"));
    lcd.print(currentLED + 1);
    lcd.print(F(" Current >"));
    
    uint8_t led_current;
    if(lt3966_i2c_read(addresses[currentLED], ILED1, &led_current)) {
        lcd.setCursor(0, 1);
        float current_ma = (led_current / 255.0) * 120.0;
        lcd.print(current_ma, 1);
        lcd.print(F("mA"));
        if(current_ma > 100.0) {
            lcd.setBacklight(YELLOW);
        } else {
            lcd.setBacklight(GREEN);
        }
    } else {
        lcd.setCursor(0, 1);
        lcd.print(F("Read Error"));
        lcd.setBacklight(RED);
    }
}

// Add before loop()
void handleMainMenu(uint8_t buttons) {
    static const char* menu_items[] = {
        "Quick Controls",
        "Indiv. Control",
        "Status/Settings"
    };
    static const uint8_t num_items = 3;
    
    if (buttons & BUTTON_UP) {
        if (menuPosition > 0) menuPosition--;
    }
    if (buttons & BUTTON_DOWN) {
        if (menuPosition < num_items - 1) menuPosition++;
    }
    if (buttons & BUTTON_SELECT) {
        switch(menuPosition) {
            case 0: 
                currentMenu = QUICK_CONTROLS;
                // Immediately show Quick Controls menu
                lcd.clear();
                lcd.print(F("Quick Control:"));
                lcd.setCursor(0, 1);
                lcd.print(F("All LEDs: OFF"));
                break;
            case 1: 
                currentMenu = INDIVIDUAL_CONTROL;
                // Immediately show Individual Control menu
                lcd.clear();
                lcd.print(F("Select LED:"));
                lcd.setCursor(0, 1);
                lcd.print(F("LED 1")); 
                break;
            case 2: 
                currentMenu = STATUS_SETTINGS;
                // Immediately show Status menu
                lcd.clear();
                lcd.print(F("> View Status"));
                lcd.setCursor(0, 1);
                lcd.print(F("  PWM Resolution"));
                break;
        }
        menuPosition = 0;  // Reset position for submenu
        return;
    }
    
    // Update main menu display
    lcd.clear();
    lcd.print(F("=== Main Menu ==="));  // Added F() macro
    lcd.setCursor(0, 1);
    lcd.print(F(">"));  // Added F() macro
    lcd.print(menu_items[menuPosition]);
}  // Added missing closing brace

void handleQuickControls(uint8_t buttons) {
    static bool isOn = false;
    static bool inSequenceMode = false;
    static uint8_t currentLED = 0;
    static uint8_t currentChannel = 0;
    static unsigned long lastSequenceUpdate = 0;
    static uint8_t menuOption = 0;  // 0 = All LEDs, 1 = Sequence
    const float DEFAULT_BRIGHTNESS = 0.1;
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    const unsigned long SEQUENCE_DELAY = 1000;
    const char* channel_names[] = {"WHITE", "BLUE", "GREEN", "RED"};

    // Handle back button (LEFT)
    if (buttons & BUTTON_LEFT) {
        if (inSequenceMode) {
            // Turn off all LEDs when exiting sequence mode
            for(uint8_t i = 0; i < 4; i++) {
                lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);
            }
            inSequenceMode = false;
            // Show Quick Controls submenu immediately
            lcd.clear();
            lcd.print(F("Quick Control:"));
            lcd.setCursor(0, 1);
            lcd.print(F(">Start Sequence"));
            return;
        } else {
            currentMenu = MAIN_MENU;
            menuPosition = 0;
            // Show main menu immediately
            lcd.clear();
            lcd.print(F("=== Main Menu ==="));
            lcd.setCursor(0, 1);
            lcd.print(F(">Quick Controls"));
            return;
        }
    }

    // Handle menu navigation
    if (buttons & BUTTON_UP || buttons & BUTTON_DOWN) {
        menuOption = !menuOption;  // Toggle between All LEDs and Sequence
    }

    if (buttons & BUTTON_SELECT) {
        if (menuOption == 0) {  // All LEDs mode
            isOn = !isOn;  // Toggle state
            
            for(uint8_t i = 0; i < 4; i++) {
                if(isOn) {
                    // Enable all channels
                    lt3966_i2c_write(addresses[i], GLBCFG, 0x00);
                    
                    // Set ADIM registers to maximum
                    lt3966_i2c_write(addresses[i], ADIM1, 0xFF);
                    lt3966_i2c_write(addresses[i], ADIM2, 0xFF);
                    lt3966_i2c_write(addresses[i], ADIM3, 0xFF);
                    lt3966_i2c_write(addresses[i], ADIM4, 0xFF);
                    
                    // Set all channels to 10% brightness
                    for(uint8_t ch = 0; ch < 4; ch++) {
                        uint8_t dimh_reg = DIM1H + (ch * 0x10);
                        uint8_t diml_reg = dimh_reg + 1;
                        uint8_t scl = 5;  // 976.6 Hz
                        uint16_t pwm_value = calculatePWM(DEFAULT_BRIGHTNESS, scl);
                        uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
                        uint8_t dim_low = pwm_value & 0xFF;
                        lt3966_i2c_write_rs(addresses[i], dimh_reg, dim_high, diml_reg, dim_low);
                    }
                } else {
                    lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);  // Turn off all channels
                }
            }
        } else {  // Sequence mode
            inSequenceMode = !inSequenceMode;
            currentLED = 0;
            currentChannel = 0;
            lastSequenceUpdate = millis();
            
            // Turn off all LEDs when starting sequence
            for(uint8_t i = 0; i < 4; i++) {
                lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);
            }
        }
    }

    // Handle sequence mode if active
    if (inSequenceMode && menuOption == 1 && (millis() - lastSequenceUpdate >= SEQUENCE_DELAY)) {
        lastSequenceUpdate = millis();
        
        // Turn off previous LED
        if (currentLED < 4) {
            lt3966_i2c_write(addresses[currentLED], GLBCFG, 0x0F);
        }
        
        // Check if sequence is complete
        if (currentLED >= 3 && currentChannel >= 3) {  // At LED4 RED
            // Turn off all LEDs
            for(uint8_t i = 0; i < 4; i++) {
                lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);
            }
            // Reset sequence mode
            inSequenceMode = false;
            currentLED = 0;
            currentChannel = 0;
            // Return to Quick Controls submenu
            lcd.clear();
            lcd.print(F("Quick Control:"));
            lcd.setCursor(0, 1);
            lcd.print(F(">Start Sequence"));
            return;
        }

        // Move to next LED/channel
        if (currentChannel >= 4) {
            currentChannel = 0;
            currentLED++;
        }

        // Turn on current LED/channel BEFORE updating display
        // This ensures the display matches the actual LED state
        lt3966_i2c_write(addresses[currentLED], GLBCFG, 0x00);
        lt3966_i2c_write(addresses[currentLED], ADIM1 + currentChannel, 0xFF);
        
        uint8_t dimh_reg = DIM1H + (currentChannel * 0x10);
        uint8_t diml_reg = dimh_reg + 1;
        uint8_t scl = 5;  // 976.6 Hz
        uint16_t pwm_value = calculatePWM(DEFAULT_BRIGHTNESS, scl);
        uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
        uint8_t dim_low = pwm_value & 0xFF;
        lt3966_i2c_write_rs(addresses[currentLED], dimh_reg, dim_high, diml_reg, dim_low);
        
        // Update display AFTER LED is turned on
        lcd.clear();
        lcd.print(F("Quick Control:"));
        lcd.setCursor(0, 1);
        lcd.print(F("LED"));
        lcd.print(currentLED + 1);
        lcd.print(F(" "));
        lcd.print(channel_names[currentChannel]);
        
        currentChannel++;  // Move to next channel after everything is done
    } else {
        // Regular display update for non-sequence modes
        lcd.clear();
        lcd.print(F("Quick Control:"));
        lcd.setCursor(0, 1);
        
        if (menuOption == 0) {
            lcd.print(F("All LEDs: "));
            lcd.print(isOn ? F("ON") : F("OFF"));
        } else if (!inSequenceMode) {
            lcd.print(F(">Start Sequence"));
        }
    }
}

void updateDisplay() {
    switch(currentMenu) {
        case MAIN_MENU:
            handleMainMenu(0);
            break;
        case QUICK_CONTROLS:
            // Display current quick controls state
            break;
        case INDIVIDUAL_CONTROL:
            // Display current individual control state
            break;
        case STATUS_SETTINGS:
            handleStatusSettings(0);
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

// Add these key safety functions
bool writeWithVerification(uint16_t addr, const void* data, size_t len) {
    const uint8_t MAX_RETRIES = 3;
    for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
        EEPROM.put(addr, data);
        // Verify write
        bool verified = true;
        for (size_t i = 0; i < len; i++) {
            if (EEPROM.read(addr + i) != ((const uint8_t*)data)[i]) {
                verified = false;
                break;
            }
        }
        if (verified) return true;
    }
    return false;
}

// Add after the existing EEPROMManager class


// Implementation of SafeEEPROMManager member functions
void SafeEEPROMManager::handleInterruptedOperation() {
    if (operationInProgress) {
        clearRecoveryPoint();
    }
}

void SafeEEPROMManager::clearRecoveryPoint() {
    EEPROM.write(RECOVERY_ADDR, 0);
    operationInProgress = false;
}

uint32_t SafeEEPROMManager::calculateCRC32(const void* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t* ptr = (const uint8_t*)data;
    while (len--) {
        crc ^= *ptr++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

bool SafeEEPROMManager::verifyWrite(uint16_t addr, const void* data, size_t len) {
    const uint8_t MAX_RETRIES = 3;
    const uint8_t* src = (const uint8_t*)data;
    
    for (uint8_t retry = 0; retry < MAX_RETRIES; retry++) {
        for (size_t i = 0; i < len; i++) {
            EEPROM.write(addr + i, src[i]);
        }
        
        bool verified = true;
        for (size_t i = 0; i < len; i++) {
            if (EEPROM.read(addr + i) != src[i]) {
                verified = false;
                break;
            }
        }
        
        if (verified) return true;
        delay(5);
    }
    return false;
}

bool SafeEEPROMManager::writeWithVerification(uint16_t addr, const void* data, size_t len) {
    if (!this->verifyWrite(addr, data, len)) {
        return false;
    }
    return true;
}

void SafeEEPROMManager::compactStorage() {
    // Move valid blocks to eliminate gaps
}

void SafeEEPROMManager::handleWearLimitReached() {
    // Handle when a block reaches wear limit
}

uint16_t SafeEEPROMManager::findLatestLog(uint8_t led, uint8_t channel) {
    // Find the most recent log entry for given LED and channel
    return 0;
}

uint16_t SafeEEPROMManager::findNextBlock() {
    // Find next available block for writing
    return DATA_START;
}

class DataLogger {
private:
    static const uint16_t LOG_START_ADDR = 512;  // Use second half of EEPROM
    static const uint8_t LOGS_PER_CHANNEL = 4;   // Store last 4 entries per channel

    uint16_t getLogAddress(uint8_t led, uint8_t channel) {
        return LOG_START_ADDR + 
               (led * 4 + channel) * sizeof(RuntimeLog) * LOGS_PER_CHANNEL;
    }

public:
    void exportData() {
        Serial.println(F("Start Time,End Time,LED,Channel,Current(mA),PWM,Temp(C)"));
        
        for(uint8_t led = 0; led < 4; led++) {
            for(uint8_t ch = 0; ch < 4; ch++) {
                RuntimeLog log;
                uint16_t addr = getLogAddress(led, ch);
                EEPROM.get(addr, log);
                
                // Print in CSV format
                printDateTime(log.startTime);
                Serial.print(',');
                printDateTime(log.endTime);
                Serial.print(',');
                Serial.print(log.led + 1);
                Serial.print(',');
                Serial.print(log.channel + 1);
                Serial.print(',');
                Serial.print(log.current_mA);
                Serial.print(',');
                Serial.print(log.pwmResolution);
                Serial.print(',');
                Serial.println(log.temperature);
            }
        }
    }

private:
    void printDateTime(const DateTime& dt) {
        Serial.print(dt.year(), DEC);
        Serial.print('/');
        Serial.print(dt.month(), DEC);
        Serial.print('/');
        Serial.print(dt.day(), DEC);
        Serial.print(' ');
        Serial.print(dt.hour(), DEC);
        Serial.print(':');
        Serial.print(dt.minute(), DEC);
        Serial.print(':');
        Serial.print(dt.second(), DEC);
    }
};


void SystemManager::handleTemperature() {
    float temp = tempsensor.getTemperature();
    if (temp > 85.0) {
        errorTracker.logError(ErrorInfo::ErrorType::TEMP_WARNING, 0);
    }
}

void SystemManager::checkErrors() {
    if (errorTracker.hasErrors()) {
        lcd.setBacklight(RED);
    } else {
        lcd.setBacklight(WHITE);
    }
}

void SystemManager::updateDisplay() {
    // Update LCD display with current status
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(tempsensor.getTemperature());
    lcd.print("C");
}

void ErrorTracker::logError(ErrorInfo::ErrorType type, uint16_t address) {
    if (errorCount < MAX_ERRORS) {
        errors[errorCount].type = type;
        errors[errorCount].address = address;
        errors[errorCount].timestamp = millis();
        errorCount++;
    }
}

void ErrorTracker::clearErrors() {
    errorCount = 0;
}

bool ErrorTracker::hasErrors() const {
    return errorCount > 0;
}

