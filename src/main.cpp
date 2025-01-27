#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include "Linduino.h"
#include "LT3966.h"
#include <EEPROM.h>
#include <I2C_RTC.h>  // Replace RTClib.h
#include <util/crc16.h>  // For CRC calculation
#include <Adafruit_PCT2075.h>
#include <time.h>  // For time_t

// New LED module addresses - ADD THESE BEFORE setup()
#define LT3966_ADD1  0b1111   // ADD1: VCC, ADD2: VCC (0x5F)
#define LT3966_ADD2  0b0011   // ADD1: VCC, ADD2: FLOAT (0x57)  **LSU module address ADD1: Flaot, ADD2: GND (0x53;0011)
#define LT3966_ADD3  0b0001   // ADD1: FLOAT, ADD2: GND (0x51)
#define LT3966_ADD4  0b0101   // ADD1: FLOAT, ADD2: FLOAT (0x55)

// Add global variable for current LED
uint8_t current_led_address = LT3966_ADD1; // Default to first LED
bool debug_mode = true;  // Enable debugging output

// Improved experiment timing structure using millis()
struct ExperimentTiming {
    bool isRunning;
    unsigned long startMillis;  // When experiment was started
    unsigned long elapsedSeconds;  // Current elapsed time in seconds
    unsigned long lastUpdate;  // Last time the elapsed time was updated
    bool firstDisplay;  // For initial display
    int dailyErrorFast;  // Milliseconds fast per day
    int dailyErrorBehind;  // Milliseconds behind per day
    bool correctedToday;  // Track daily corrections
} experimentTiming;

// Menu state structure for managing menu navigation
struct MenuStateVars {
    bool showContent;    // Flag to track if we're showing content
    bool isFirstEntry;   // Track if this is first entry into menu
    uint8_t currentItem; // Current selected menu item

    MenuStateVars() : showContent(false), isFirstEntry(true), currentItem(0) {}
};

// Forward declare the PWM configuration structures
struct PWMFreqConfig {
    uint8_t scl;
    uint16_t freq;    // Frequency in Hz
    uint8_t bits;     // Resolution in bits
    const char* desc; // Description
};

struct TempTracking {
    float currentTemp;
    float maxTemp;
    float minuteReadings[60];  // Store 60 readings for 1-minute average
    uint8_t readingIndex;
    unsigned long lastUpdateTime;
    bool isFirstReading;
};

// Any function declarations can go here
void handlePWMFrequencyConfig(uint8_t buttons);
void applyPWMFrequency(uint8_t ledSelect, uint8_t channelSelect, uint8_t freqSelect);
void initializeLED(uint8_t address);
void initializeLogging();
void loadSettings();
void handleMainMenu(uint8_t buttons);
void handleQuickControls(uint8_t buttons);
void handleIndividualControl(uint8_t buttons);
void handleStatusSettings(uint8_t buttons);
void updateDisplay();
void displayErrorStatus();
void displayVoltageLevels();
void displayCurrentSettings();
void displayPWMStatus();
void displayCurrentStatus();
float readfloatTerminal();
uint8_t readTerminalDecimal();
void printPaddedBinary(uint8_t value);
void verifyConfiguration(uint8_t address);
uint16_t calculatePWM(float duty_cycle, uint8_t scl);
bool verifyI2CWriteRS(uint8_t address, uint8_t reg1, uint8_t value1, uint8_t reg2, uint8_t value2);
bool verifyI2CWrite(uint8_t address, uint8_t reg, uint8_t value);
void handleStatusMenu(uint8_t buttons);
void displayLEDStateScreen(uint8_t currentLED, uint8_t buttons);
void displayTemperatureScreen();
void displayElapsedTimeScreen();
void toggleExperiment(bool start);
void updateTemperatureStats();
void formatElapsedTime(char* buffer, unsigned long seconds);
void storeTemperatureReading();
float calculateTemperatureAverage();
void handleStatusDisplay(uint8_t buttons, uint8_t& displayMenuItem);
void handlePWMSettings(uint8_t buttons);
void handlePhaseSettings(uint8_t buttons); // Add this declaration
void updateLEDCache();
void displayCurrentTime();
bool isChannelAvailable(uint8_t led, uint8_t channel);
void displayAvailableChannels(uint8_t led);
const char* getChannelName(uint8_t channel);
void updateChannelDisplay(uint8_t led, uint8_t channel, float brightness);
uint8_t findNextAvailableChannel(uint8_t led, uint8_t currentChannel, bool forward);
void initializeExperimentState();
void updateExperimentTime();
void initializeExperimentTiming();
void updateTempHistory(float* history, float newTemp, float& maxTemp);
float calculateTempAverage(float* history);
void updateTempTracking(TempTracking& tracker, float newTemp);
float calculateMinuteAverage(TempTracking& tracker);
void handleMainMenuNavigation(uint8_t buttons);
void handleQuickControlsNavigation(uint8_t buttons);
void handleIndividualControlNavigation(uint8_t buttons);
void handleStatusSettingsNavigation(uint8_t buttons);
void displayMainMenu();
void displayQuickControls();
void displayIndividualControl();
void displayStatusSettings();

// 1. Improved Serial Input Handling
#define SERIAL_TIMEOUT 10000  // 10 second timeout
#define INPUT_BUFFER_SIZE 32
float roomTempHistory[60] = {0};  // Last 60 seconds of room temperature readings
float pcbTempHistory[60] = {0};   // Last 60 seconds of PCB temperature readings
uint8_t tempHistoryIndex = 0;     // Current index in the circular buffer
float maxRoomTemp = 0;            // Maximum room temperature
float maxPCBTemp = 0;             // Maximum PCB temperature
unsigned long lastTempUpdate = 0;  // Last temperature update time

TempTracking roomTempTrack;
TempTracking pcbTempTrack;

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

// LED state cache structure
struct LEDStateCache {
    unsigned long lastUpdate;
    float channelBrightness[4][4];
    bool isValid;
};

// Cache update interval (1 second)
static const unsigned long UPDATE_INTERVAL = 5000;  // Increase to 5 seconds

// Global instance of LED state cache
LEDStateCache ledStateCache = {0, {{0}}, false};
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
        uint32_t total_seconds;  // Instead of TimeStamp
        float max_current;
        float min_current;
        float avg_current;
        uint32_t last_update;
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

enum IndividualControlState {
    LED_SELECT,
    CHANNEL_SELECT,
    EXPERIMENT_CONTROL
};

enum StatusDisplayState {
    LED_STATUS,
    ELAPSED_TIME,
    TEMPERATURE_DATA,
    STOP_EXPERIMENT
};

enum StatusMenuState {
    STATUS_SUBMENU_SELECT,  // Choosing between Status Display and PWM Settings
    STATUS_DISPLAY,         // In Status Display sub-menu
    PWM_SETTINGS,          // In PWM Settings sub-menu
    PHASE_SETTINGS         // In Phase Settings sub-menu
};

enum PWMMenuState {
    PWM_LED_SELECT,        // Select LED (1-4 or All)
    PWM_CHANNEL_SELECT,    // Select Channel (default All)
    PWM_FREQUENCY_SELECT   // Select frequency
};

enum PhaseMenuState {
    PHASE_MODE_SELECT,     // Select In-phase or Out-of-phase
    PHASE_LED_SELECT,      // Select LED (1-4 or All)
    PHASE_CHANNEL_SELECT   // Select Channel
};

// Global variables
MenuState currentMenu = MenuState::MAIN_MENU;
uint8_t menuPosition = 0;
// Global menu state instance for status display
MenuStateVars statusDisplayState;
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

// Replace RTC_DS3231 rtc with:
static DS3231 RTC;

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
    time_t startTime;
    time_t endTime;
    uint8_t led;
    uint8_t channel;
    float current_mA;
    uint8_t pwmResolution;
    float temperature;
};

class RuntimeTracker {
private:
    static const uint16_t LOG_START_ADDR = 512;  // Use second half of EEPROM
    static const uint8_t LOGS_PER_CHANNEL = 4;   // Store last 4 entries per channel
    
public:
    void logChannelStart(uint8_t led, uint8_t channel, float current) {
        RuntimeLog log;
        log.startTime = RTC.getEpoch(true);  // Get Unix timestamp directly
        log.led = led;
        log.channel = channel;
        log.current_mA = current;
        log.pwmResolution = currentPWMResolution;
        log.temperature = RTC.getTemp();
        
        uint16_t addr = getLogAddress(led, channel);
        EEPROM.put(addr, log);
    }
    
    void logChannelStop(uint8_t led, uint8_t channel) {
        uint16_t addr = getLogAddress(led, channel);
        RuntimeLog log;
        EEPROM.get(addr, log);
        log.endTime = RTC.getEpoch(true);  // Get Unix timestamp directly
        EEPROM.put(addr, log);
    }

private:
    uint16_t getLogAddress(uint8_t led, uint8_t channel) {
        return LOG_START_ADDR + 
               (led * 4 + channel) * sizeof(RuntimeLog) * LOGS_PER_CHANNEL;
    }
};

// Add these declarations right after the includes, before any functions


// Global variables and constants (add with other globals)
Adafruit_PCT2075 tempsensor;
static const uint16_t TEMP_LOG_ADDR = 960;
static const float TEMP_WARNING_THRESHOLD = 60.0;  // Move this here too

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
    static PWMMenuState menuState = PWM_LED_SELECT;
    static uint8_t selectedLED = 0;
    static uint8_t selectedChannel = 0;
    static uint8_t selectedFreq = 0;
    static bool firstEntry = true;
    static bool configApplied = false;

    // If this is first entry or config was just applied, reset the menu
    if (firstEntry || configApplied) {
        lcd.clear();
        switch (menuState) {
            case PWM_LED_SELECT:
                lcd.print(F("Select LED:"));
                lcd.setCursor(0, 1);
                if (selectedLED == 0) {
                    lcd.print(F("All LEDs"));
                } else {
                    lcd.print(F("LED "));
                    lcd.print(selectedLED);
                }
                break;
                
            case PWM_CHANNEL_SELECT:
                lcd.print(F("Select Channel:"));
                lcd.setCursor(0, 1);
                switch (selectedChannel) {
                    case 0: lcd.print(F("All Channels")); break;
                    case 1: lcd.print(F("White")); break;
                    case 2: lcd.print(F("Blue")); break;
                    case 3: lcd.print(F("Green")); break;
                    case 4: lcd.print(F("Red")); break;
                }
                break;
                
            case PWM_FREQUENCY_SELECT:
                lcd.print(F("PWM Frequency:"));
                lcd.setCursor(0, 1);
                lcd.print(PWM_CONFIGS[selectedFreq].desc);
                break;
        }
        firstEntry = false;
        configApplied = false;
    }

    // Handle back button
    if (buttons & BUTTON_LEFT) {
        if (menuState > PWM_LED_SELECT) {
            menuState = (PWMMenuState)(menuState - 1);
            firstEntry = true;
        }
        return;
    }

    // Handle navigation
    if (buttons & (BUTTON_UP | BUTTON_DOWN)) {
        switch (menuState) {
            case PWM_LED_SELECT:
                if (buttons & BUTTON_UP) {
                    if (selectedLED > 0) selectedLED--;
                } else {
                    if (selectedLED < 4) selectedLED++;
                }
                lcd.setCursor(0, 1);
                lcd.print(F("                "));  // Clear line
                lcd.setCursor(0, 1);
                if (selectedLED == 0) {
                    lcd.print(F("All LEDs"));
                } else {
                    lcd.print(F("LED "));
                    lcd.print(selectedLED);
                }
                break;
                
            case PWM_CHANNEL_SELECT:
                if (buttons & BUTTON_UP) {
                    if (selectedChannel > 0) selectedChannel--;
                } else {
                    if (selectedChannel < 4) selectedChannel++;
                }
                lcd.setCursor(0, 1);
                lcd.print(F("                "));  // Clear line
                lcd.setCursor(0, 1);
                switch (selectedChannel) {
                    case 0: lcd.print(F("All Channels")); break;
                    case 1: lcd.print(F("White")); break;
                    case 2: lcd.print(F("Blue")); break;
                    case 3: lcd.print(F("Green")); break;
                    case 4: lcd.print(F("Red")); break;
                }
                break;
                
            case PWM_FREQUENCY_SELECT:
                if (buttons & BUTTON_UP) {
                    if (selectedFreq > 0) selectedFreq--;
                } else {
                    if (selectedFreq < 7) selectedFreq++;
                }
                lcd.setCursor(0, 1);
                lcd.print(F("                "));  // Clear line
                lcd.setCursor(0, 1);
                lcd.print(PWM_CONFIGS[selectedFreq].desc);
                break;
        }
    }

    // Handle selection
    if (buttons & BUTTON_SELECT) {
        switch (menuState) {
            case PWM_LED_SELECT:
                menuState = PWM_CHANNEL_SELECT;
                firstEntry = true;
                break;
                
            case PWM_CHANNEL_SELECT:
                menuState = PWM_FREQUENCY_SELECT;
                firstEntry = true;
                break;
                
            case PWM_FREQUENCY_SELECT:
                // Apply the frequency settings
                applyPWMFrequency(selectedLED, selectedChannel, selectedFreq);
                
                // Show confirmation
                lcd.clear();
                lcd.print(F("PWM Updated!"));
                delay(1000);
                
                // Reset menu state for next time
                menuState = PWM_LED_SELECT;
                selectedLED = 0;
                selectedChannel = 0;
                selectedFreq = 0;
                firstEntry = true;
                configApplied = true;
                break;
        }
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
    
    // Initialize experiment state
    initializeExperimentTiming();
    
    // Check if LCD is responding - LCD is critical
    Wire.beginTransmission(LCD_I2C_ADDR);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.println(F("LCD foundvoid displayLEDStateScreen(uint8_t currentLED); at 0x20"));
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
        if (!RTC.begin()) {
            Serial.println(F("RTC init failed"));
            lcd.clear();
            lcd.print(F("RTC Error"));
            delay(1000);  // Show error briefly but continue
        } else {
            // Verify RTC is running and has valid time
            time_t currentTime = RTC.getEpoch(true);
            if (currentTime < 1000000000) { // Basic sanity check for RTC time
                Serial.println(F("RTC time invalid"));
                lcd.clear();
                lcd.print(F("RTC Time Error"));
                lcd.setCursor(0, 1);
                lcd.print(F("Set Clock"));
                delay(2000);
            } else {
                Serial.println(F("RTC OK"));
            }
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

// Add new array declaration
static float channelBrightness[4][4] = {{0}};  // [LED][channel] brightness values

// Add new function declarations
void updateLEDStateDisplay();
void displayLEDChannelStatus(uint8_t led, uint8_t channel, float brightness);
void formatChannelDisplay(char* buffer, uint8_t led, uint8_t channel);
bool isChannelActive(uint8_t led, uint8_t channel);

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

    // Get LED index from address
    uint8_t led_idx = 0;
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    for(uint8_t i = 0; i < 4; i++) if(addresses[i] == address) led_idx = i;

    // Update cache
    ledStateCache.channelBrightness[led_idx][channel] = duty_cycle;
    ledStateCache.isValid = true;

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

// Helper functions for Individual Control Enhancement
bool isChannelAvailable(uint8_t led, uint8_t channel) {
    // LED2 and LED4 (index 1 and 3) cannot use RED channel (index 3)
    if ((led == 1 || led == 3) && channel == 3) {
        return false;
    }
    return true;
}

const char* getChannelName(uint8_t channel) {
    static const char* channel_names[] = {"WHITE", "BLUE", "GREEN", "RED"};
    return channel_names[channel];
}

uint8_t findNextAvailableChannel(uint8_t led, uint8_t currentChannel, bool forward) {
    uint8_t nextChannel = currentChannel;
    for(uint8_t i = 0; i < 4; i++) {  // Maximum 4 iterations
        if(forward) {
            nextChannel = (nextChannel + 1) % 4;
        } else {
            nextChannel = (nextChannel + 3) % 4;  // +3 is same as -1 with wraparound
        }
        if(isChannelAvailable(led, nextChannel)) {
            return nextChannel;
        }
    }
    return currentChannel;  // If no available channel found, stay on current
}

void updateChannelDisplay(uint8_t led, uint8_t channel, float brightness) {
    lcd.clear();
    lcd.print(F("LED"));
    lcd.print(led + 1);
    lcd.print(F(" "));
    lcd.print(getChannelName(channel));
    if(!isChannelAvailable(led, channel)) {
        lcd.print(F(" N/A"));
    }
    lcd.setCursor(0, 1);
    lcd.print(F("Bright: "));
    lcd.print((int)(brightness + 0.5f));  // Round to nearest integer
    lcd.print(F("%"));
}

void displayAvailableChannels(uint8_t led) {
    // Add bounds check to prevent showing LED5
    if (led >= 4) return;  // Only show channels for LED1-4
    
    lcd.clear();
    lcd.print(F("LED"));
    lcd.print(led + 1);
    lcd.print(F(" Channels:"));
    lcd.setCursor(0, 1);
    
    String available = "";
    for(uint8_t ch = 0; ch < 4; ch++) {
        if(isChannelAvailable(led, ch)) {
            if(available.length() > 0) {
                available += ",";
            }
            available += getChannelName(ch)[0];  // First letter of channel name
        }
    }
    lcd.print(available);
}

void handleIndividualControl(uint8_t buttons) {
    static uint8_t selectedLED = 0;      // 0-3 for LED1-4
    static uint8_t selectedChannel = 0;   // 0-3 for WHITE/BLUE/GREEN/RED
    static bool inChannelMode = false;    // false = LED select, true = Channel control
    static bool inRunMode = false;        // true when in Run Experiment menu
    const float MAX_ACTUAL_BRIGHTNESS = 12.0;  // Maximum actual brightness percentage due to hardware limitations
    const float BRIGHTNESS_STEP = 1.0;         // 1% per step
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};

    // Handle back button
    if (buttons & BUTTON_LEFT) {
        if (inRunMode) {
            // Return to LED selection
            inRunMode = false;
            inChannelMode = false;
            selectedLED = 0;  // Reset to first LED when exiting Run Experiment
            lcd.clear();
            lcd.print(F(">LED "));
            lcd.print(selectedLED + 1);
            displayAvailableChannels(selectedLED);
        } else if (inChannelMode) {
            // Return to LED selection
            inChannelMode = false;
            lcd.clear();
            lcd.print(F(">LED "));
            lcd.print(selectedLED + 1);
            displayAvailableChannels(selectedLED);
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

    if (inRunMode) {
        // Run Experiment mode
        if (buttons & BUTTON_SELECT) {
            // Start the experiment
            toggleExperiment(true);  // This will handle elapsed time initialization

            // Turn on all configured LEDs
            for(uint8_t led = 0; led < 4; led++) {
                bool anyChannelActive = false;
                for(uint8_t ch = 0; ch < 4; ch++) {
                    if (channelBrightness[led][ch] > 0) {
                        anyChannelActive = true;
                        break;
                    }
                }
                
                if (anyChannelActive) {
                    lt3966_i2c_write(addresses[led], GLBCFG, 0x00);  // Enable channels
                    for(uint8_t ch = 0; ch < 4; ch++) {
                        if (channelBrightness[led][ch] > 0 && isChannelAvailable(led, ch)) {
                            float actualBrightness = (channelBrightness[led][ch] / 100.0f) * MAX_ACTUAL_BRIGHTNESS;
                            uint8_t dimh_reg = DIM1H + (ch * 0x10);
                            uint8_t diml_reg = dimh_reg + 1;
                            uint8_t scl = 5;  // PWM scaling factor
                            uint16_t pwm_value = calculatePWM(actualBrightness, scl);
                            uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
                            uint8_t dim_low = pwm_value & 0xFF;
                            lt3966_i2c_write(addresses[led], ADIM1 + ch, 0xFF);
                            lt3966_i2c_write_rs(addresses[led], dimh_reg, dim_high, diml_reg, dim_low);
                        }
                    }
                }
            }
            
            // Update display
            lcd.clear();
            lcd.print(F("Experiment"));
            lcd.setCursor(0, 1);
            lcd.print(F("Started!"));
            delay(1000);
            
            // Return to main menu
            currentMenu = MAIN_MENU;
            menuPosition = 1;
            return;
        }
        
        // Display Run Experiment screen
        lcd.clear();
        lcd.print(F("Run Experiment?"));
        lcd.setCursor(0, 1);
        lcd.print(F("Select to Start"));
        return;
    }

    if (!inChannelMode) {
        // LED Selection Mode
        if (buttons & BUTTON_UP) {
            if (selectedLED > 0) selectedLED--;
            lcd.clear();
            lcd.print(F(">LED "));
            lcd.print(selectedLED + 1);
            displayAvailableChannels(selectedLED);
        }
        if (buttons & BUTTON_DOWN) {
            if (selectedLED < 4) selectedLED++;  // Allow one more position for Run Experiment
            if (selectedLED == 4) {
                lcd.clear();
                lcd.print(F(">Run Experiment"));
                lcd.setCursor(0, 1);
                lcd.print(F("Select to Start"));
            } else {
                lcd.clear();
                lcd.print(F(">LED "));
                lcd.print(selectedLED + 1);
                displayAvailableChannels(selectedLED);
            }
        }
        if (buttons & BUTTON_SELECT) {
            if (selectedLED == 4) {
                // Enter Run Experiment mode
                inRunMode = true;
                lcd.clear();
                lcd.print(F("Run Experiment?"));
                lcd.setCursor(0, 1);
                lcd.print(F("Select to Start"));
            } else {
                inChannelMode = true;
                // Find first available channel for this LED
                selectedChannel = 0;
                while(!isChannelAvailable(selectedLED, selectedChannel) && selectedChannel < 4) {
                    selectedChannel++;
                }
                // Update display with channel info
                updateChannelDisplay(selectedLED, selectedChannel, channelBrightness[selectedLED][selectedChannel]);
            }
        }
    } else {
        // Channel Control Mode - Store brightness values but don't turn on LEDs yet
        if (buttons & BUTTON_UP || buttons & BUTTON_DOWN) {
            // Change channel
            if (buttons & BUTTON_UP) {
                selectedChannel = findNextAvailableChannel(selectedLED, selectedChannel, false);
            }
            if (buttons & BUTTON_DOWN) {
                selectedChannel = findNextAvailableChannel(selectedLED, selectedChannel, true);
            }
            updateChannelDisplay(selectedLED, selectedChannel, channelBrightness[selectedLED][selectedChannel]);
        }

        // Increase brightness
        if (buttons & BUTTON_RIGHT) {
            if (isChannelAvailable(selectedLED, selectedChannel) && 
                channelBrightness[selectedLED][selectedChannel] < 100.0) {
                channelBrightness[selectedLED][selectedChannel] += BRIGHTNESS_STEP;
                if (channelBrightness[selectedLED][selectedChannel] > 100.0) {
                    channelBrightness[selectedLED][selectedChannel] = 100.0;
                }
                // Store the brightness value but don't turn on LED yet
                ledStateCache.channelBrightness[selectedLED][selectedChannel] = channelBrightness[selectedLED][selectedChannel];
                ledStateCache.isValid = true;
                ledStateCache.lastUpdate = millis();
                updateChannelDisplay(selectedLED, selectedChannel, channelBrightness[selectedLED][selectedChannel]);
            }
        }

        // Decrease brightness
        if (buttons & BUTTON_LEFT) {
            if (isChannelAvailable(selectedLED, selectedChannel) && 
                channelBrightness[selectedLED][selectedChannel] > 0.0) {
                channelBrightness[selectedLED][selectedChannel] -= BRIGHTNESS_STEP;
                if (channelBrightness[selectedLED][selectedChannel] < 0.0) {
                    channelBrightness[selectedLED][selectedChannel] = 0.0;
                }
                // Store the brightness value but don't turn on LED yet
                ledStateCache.channelBrightness[selectedLED][selectedChannel] = channelBrightness[selectedLED][selectedChannel];
                ledStateCache.isValid = true;
                ledStateCache.lastUpdate = millis();
                updateChannelDisplay(selectedLED, selectedChannel, channelBrightness[selectedLED][selectedChannel]);
            }
        }

        if (buttons & BUTTON_SELECT) {
            // Set channel to 0% if it's available
            if (isChannelAvailable(selectedLED, selectedChannel)) {
                channelBrightness[selectedLED][selectedChannel] = 0;
                ledStateCache.channelBrightness[selectedLED][selectedChannel] = 0;
                ledStateCache.isValid = true;
                ledStateCache.lastUpdate = millis();
                updateChannelDisplay(selectedLED, selectedChannel, 0);
            }
        }
    }
}

void handleStatusSettings(uint8_t buttons) {
    static uint8_t subMenuSelection = 0;  // 0 = Status Display, 1 = PWM Settings, 2 = Phase Settings
    static MenuStateVars statusDisplayState;
    static uint8_t displayMenuItem = 0;
    static StatusMenuState menuState = STATUS_SUBMENU_SELECT;
    const uint8_t NUM_SUBMENU_ITEMS = 3;  // Updated to include Phase Settings
    
    // Handle back button
    if (buttons & BUTTON_LEFT) {
        statusDisplayState = MenuStateVars();
        if (menuState == STATUS_SUBMENU_SELECT) {
            currentMenu = MAIN_MENU;
            // Immediately show main menu
            lcd.clear();
            lcd.print(F("=== Main Menu ==="));
            lcd.setCursor(0, 1);
            lcd.print(F(">Status/Settings"));  // Return to where we were
            return;
        } else {
            menuState = STATUS_SUBMENU_SELECT;
            // Immediately show Status/Settings submenu
            lcd.clear();
            lcd.print(subMenuSelection == 0 ? ">" : " ");
            lcd.print(F("Status Display"));
            lcd.setCursor(0, 1);
            lcd.print(subMenuSelection == 1 ? ">" : " ");
            lcd.print(F("PWM Settings"));
            return;
        }
    }
    
    switch(menuState) {
        case STATUS_SUBMENU_SELECT:
            // Handle navigation first
            if (buttons & BUTTON_UP) {
                if (subMenuSelection > 0) subMenuSelection--;
            }
            if (buttons & BUTTON_DOWN) {
                if (subMenuSelection < NUM_SUBMENU_ITEMS - 1) subMenuSelection++;
            }
            
            // Then update display
            lcd.clear();
            // First line
            if (subMenuSelection <= 1) {
                lcd.print(subMenuSelection == 0 ? ">" : " ");
                lcd.print(F("Status Display"));
                lcd.setCursor(0, 1);
                lcd.print(subMenuSelection == 1 ? ">" : " ");
                lcd.print(F("PWM Settings"));
            } else {
                lcd.print(subMenuSelection == 1 ? ">" : " ");
                lcd.print(F("PWM Settings"));
                lcd.setCursor(0, 1);
                lcd.print(subMenuSelection == 2 ? ">" : " ");
                lcd.print(F("Phase Settings"));
            }
            
            if (buttons & BUTTON_SELECT) {
                switch (subMenuSelection) {
                    case 0:
                        menuState = STATUS_DISPLAY;
                        break;
                    case 1:
                        menuState = PWM_SETTINGS;
                        break;
                    case 2:
                        menuState = PHASE_SETTINGS;
                        break;
                }
                displayMenuItem = 0;  // Reset sub-menu position
                
                // Immediately show the selected submenu
                switch (menuState) {
                    case STATUS_DISPLAY:
                        handleStatusDisplay(0, displayMenuItem);
                        break;
                    case PWM_SETTINGS:
                        handlePWMSettings(0);
                        break;
                    case PHASE_SETTINGS:
                        handlePhaseSettings(0);
                        break;
                }
            }
            break;
            
        case STATUS_DISPLAY:
            handleStatusDisplay(buttons, displayMenuItem);
            break;
            
        case PWM_SETTINGS:
            handlePWMSettings(buttons);
            break;
            
        case PHASE_SETTINGS:
            handlePhaseSettings(buttons);
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
    lcd.print(PWM_CONFIGS[currentPWMResolution-6].freq);
    lcd.print(F("Hz"));
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
                // Directly show LED1's channels
                lcd.clear();
                lcd.print(F("LED  Channel"));
                lcd.setCursor(0, 1);
                lcd.print(F("Selection"));
                break;
            case 2: 
                currentMenu = STATUS_SETTINGS;
                // Show Status/Settings submenu options
                lcd.clear();
                lcd.print(F(">Status Display"));
                lcd.setCursor(0, 1);
                lcd.print(F(" PWM Settings"));
                break;
        }
        menuPosition = 0;  // Reset position for submenu
        return;
    }
    
    // Update main menu display
    lcd.clear();
    lcd.print(F("=== Main Menu ==="));
    lcd.setCursor(0, 1);
    lcd.print(F(">"));
    lcd.print(menu_items[menuPosition]);
}

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
                // Clear brightness values
                for(uint8_t ch = 0; ch < 4; ch++) {
                    channelBrightness[i][ch] = 0;
                    ledStateCache.channelBrightness[i][ch] = 0;
                }
            }
            ledStateCache.isValid = true;
            ledStateCache.lastUpdate = millis();
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
                    
                    // Set all channels to default brightness
                    for(uint8_t ch = 0; ch < 4; ch++) {
                        // Skip RED channel for LED2 and LED4
                        if (ch == 3 && (i == 1 || i == 3)) continue;
                        
                        uint8_t dimh_reg = DIM1H + (ch * 0x10);
                        uint8_t diml_reg = dimh_reg + 1;
                        uint8_t scl = 5;  // 976.6 Hz
                        uint16_t pwm_value = calculatePWM(DEFAULT_BRIGHTNESS, scl);
                        uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
                        uint8_t dim_low = pwm_value & 0xFF;
                        lt3966_i2c_write_rs(addresses[i], dimh_reg, dim_high, diml_reg, dim_low);
                        
                        // Update both arrays with the new brightness
                        channelBrightness[i][ch] = DEFAULT_BRIGHTNESS;
                        ledStateCache.channelBrightness[i][ch] = DEFAULT_BRIGHTNESS;
                    }
                } else {
                    lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);  // Turn off all channels
                    // Clear all brightness values
                    for(uint8_t ch = 0; ch < 4; ch++) {
                        channelBrightness[i][ch] = 0;
                        ledStateCache.channelBrightness[i][ch] = 0;
                    }
                }
            }
            // Update cache status
            ledStateCache.isValid = true;
            ledStateCache.lastUpdate = millis();
        } else {  // Sequence mode
            inSequenceMode = !inSequenceMode;
            currentLED = 0;
            currentChannel = 0;
            lastSequenceUpdate = millis();
            
            // Turn off all LEDs when starting sequence
            for(uint8_t i = 0; i < 4; i++) {
                lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);
                // Clear all brightness values
                for(uint8_t ch = 0; ch < 4; ch++) {
                    channelBrightness[i][ch] = 0;
                    ledStateCache.channelBrightness[i][ch] = 0;
                }
            }
            ledStateCache.isValid = true;
            ledStateCache.lastUpdate = millis();
        }
    }

    // Handle sequence mode if active
    if (inSequenceMode && menuOption == 1 && (millis() - lastSequenceUpdate >= SEQUENCE_DELAY)) {
        lastSequenceUpdate = millis();
        
        // Turn off previous LED and clear its brightness values
        if (currentLED < 4) {
            lt3966_i2c_write(addresses[currentLED], GLBCFG, 0x0F);
            for(uint8_t ch = 0; ch < 4; ch++) {
                channelBrightness[currentLED][ch] = 0;
                ledStateCache.channelBrightness[currentLED][ch] = 0;
            }
        }
        
        // Check if sequence is complete
        if (currentLED >= 3 && currentChannel >= 3) {
            // Turn off all LEDs
            for(uint8_t i = 0; i < 4; i++) {
                lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);
                for(uint8_t ch = 0; ch < 4; ch++) {
                    channelBrightness[i][ch] = 0;
                    ledStateCache.channelBrightness[i][ch] = 0;
                }
            }
            // Reset sequence mode
            inSequenceMode = false;
            currentLED = 0;
            currentChannel = 0;
            ledStateCache.isValid = true;
            ledStateCache.lastUpdate = millis();
            // Return to Quick Controls submenu
            lcd.clear();
            lcd.print(F("Quick Control:"));
            lcd.setCursor(0, 1);
            lcd.print(F(">Start Sequence"));
            return;
        }

        // Skip RED channel for LED2 and LED4
        if (currentChannel == 3 && (currentLED == 1 || currentLED == 3)) {
            currentChannel = 0;
            currentLED++;
        }
        
        // Turn on current LED/channel
        lt3966_i2c_write(addresses[currentLED], GLBCFG, 0x00);
        lt3966_i2c_write(addresses[currentLED], ADIM1 + currentChannel, 0xFF);
        
        uint8_t dimh_reg = DIM1H + (currentChannel * 0x10);
        uint8_t diml_reg = dimh_reg + 1;
        uint8_t scl = 5;  // 976.6 Hz
        uint16_t pwm_value = calculatePWM(DEFAULT_BRIGHTNESS, scl);
        uint8_t dim_high = ((scl & 0x07) << 5) | ((pwm_value >> 8) & 0x1F);
        uint8_t dim_low = pwm_value & 0xFF;
        lt3966_i2c_write_rs(addresses[currentLED], dimh_reg, dim_high, diml_reg, dim_low);
        
        // Update brightness values for current channel
        channelBrightness[currentLED][currentChannel] = DEFAULT_BRIGHTNESS;
        ledStateCache.channelBrightness[currentLED][currentChannel] = DEFAULT_BRIGHTNESS;
        ledStateCache.isValid = true;
        ledStateCache.lastUpdate = millis();
        
        // Update display
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

void displayCurrentSettings() {
    lcd.print("PWM Res: " + String(currentPWMResolution));
    lcd.setCursor(0, 1);
    lcd.print("Bright: " + String(currentBrightness) + "%");
}

// Simplify ChannelLog
struct ChannelLog {
    uint32_t total_seconds;  // Instead of TimeStamp
    float max_current;
    float min_current;
    float avg_current;
    uint32_t last_update;
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
        uint32_t elapsed = (current - stats.channels[led][channel].last_update) / 1000;
        
        stats.channels[led][channel].total_seconds += elapsed;
        stats.channels[led][channel].max_current = max(stats.channels[led][channel].max_current, stats.channels[led][channel].avg_current);
        stats.channels[led][channel].min_current = min(stats.channels[led][channel].min_current, stats.channels[led][channel].avg_current);
        stats.channels[led][channel].avg_current = (stats.channels[led][channel].total_seconds / stats.channels[led][channel].is_active) / 1000.0f;
        stats.channels[led][channel].last_update = current;
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
uint8_t getAddressForLED(uint8_t led);
void readLogFromEEPROM(uint16_t addr, ChannelLog* log);
uint16_t calculateChecksum(ChannelLog* log);

// Add implementations
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
            stats.channels[led][channel].total_seconds = 0;
            stats.channels[led][channel].max_current = 0;
            stats.channels[led][channel].min_current = 0;
            stats.channels[led][channel].avg_current = 0;
            stats.channels[led][channel].last_update = 0;
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
        stats.channels[led][channel].last_update = millis();
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
                Serial.print(log.startTime);
                Serial.print(',');
                Serial.print(log.endTime);
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
    // Removed printDateTime function
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

void displayMainMenu() {
    lcd.clear();
    lcd.print(F("=== Main Menu ==="));
    lcd.setCursor(0, 1);
    lcd.print(F(">"));
    lcd.print(F("Quick Controls"));
}

void displayQuickControls() {
    lcd.clear();
    lcd.print(F("Quick Controls"));
    lcd.setCursor(0, 1);
    lcd.print(F("(Navigation only)"));
}

void displayIndividualControl() {
    lcd.clear();
    lcd.print(F("Individual Control"));
    lcd.setCursor(0, 1);
    lcd.print(F("LED Select"));
}

void displayStatusSettings() {
    lcd.clear();
    lcd.print(F("Status/Settings"));
    lcd.setCursor(0, 1);
    lcd.print(F("Status Display"));
}

void handleMainMenuNavigation(uint8_t buttons) {
    static const char* menu_items[] = {
        "Quick Controls",
        "Indiv. Control",
        "Status/Settings"
    };
    static uint8_t num_items = 3;
    static uint8_t menuPosition = 0;

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
                displayQuickControls();
                break;
            case 1:
                currentMenu = INDIVIDUAL_CONTROL;
                displayIndividualControl();
                break;
            case 2:
                currentMenu = STATUS_SETTINGS;
                displayStatusSettings();
                break;
        }
    }

    // Update main menu display
    lcd.clear();
    lcd.print(F("=== Main Menu ==="));
    lcd.setCursor(0, 1);
    lcd.print(F(">"));
    lcd.print(menu_items[menuPosition]);
}

void handleQuickControlsNavigation(uint8_t buttons) {
    // Navigation logic only
    displayQuickControls();
}

void handleIndividualControlNavigation(uint8_t buttons) {
    // Navigation logic only
    displayIndividualControl();
}

void handleStatusSettingsNavigation(uint8_t buttons) {
    // Navigation logic only
    displayStatusSettings();
}

// LED state tracking
void updateLEDStateDisplay();
void formatLEDPercentage(char* buffer, float percentage);
void displayLEDChannelStatus(uint8_t led, uint8_t channel);
bool isLEDActive(uint8_t led);
float getLEDChannelPercentage(uint8_t led, uint8_t channel);

// Global variables for LED state tracking
struct LEDState {
    float channelPercentages[4];  // Percentages for WHITE, BLUE, GREEN, RED
    bool isActive;
};
LEDState ledStates[4];  // One for each LED

void updateLEDStateDisplay() {
    lcd.clear();
    for (uint8_t led = 0; led < 4; led++) {
        lcd.setCursor((led % 2) * 8, led / 2);
        lcd.print(F("LED"));
        lcd.print(led + 1);
        lcd.print(F(": "));
        
        for (uint8_t channel = 0; channel < 4; channel++) {
            displayLEDChannelStatus(led, channel, channelBrightness[led][channel]);
        }
    }
}

void formatLEDPercentage(char* buffer, float percentage) {
    if (percentage < 0) {
        strcpy(buffer, "--");
    } else {
        snprintf(buffer, 4, "%2.0f", percentage);
    }
}

void displayLEDChannelStatus(uint8_t led, uint8_t channel, float brightness) {
    char buffer[5];
    formatChannelDisplay(buffer, led, channel);
    lcd.print(buffer);
    lcd.print(F(" "));
}

void formatChannelDisplay(char* buffer, uint8_t led, uint8_t channel) {
    if (!isChannelAvailable(led, channel)) {
        strcpy(buffer, "---");
    } else {
        snprintf(buffer, 5, "%c%2.0f", getChannelName(channel)[0], channelBrightness[led][channel]);
    }
}

bool isChannelActive(uint8_t led, uint8_t channel) {
    return isChannelAvailable(led, channel) && channelBrightness[led][channel] > 0;
}

// Add after other struct definitions
struct TemperatureStats {
    float minTemp;
    float maxTemp;
    float hourlyReadings[24];
    uint8_t currentHourIndex;
    unsigned long lastUpdateTime;
    float lastAmbientTemp;
    float lastPCBTemp;
} tempStats;

void displayLEDStateScreen(uint8_t currentLED, uint8_t buttons) {
    static uint8_t selectedLED = 0;  // Track which LED is selected

    // Handle navigation if buttons are pressed
    if (buttons & BUTTON_UP) {
        if (selectedLED > 0) selectedLED--;
    }
    if (buttons & BUTTON_DOWN) {
        if (selectedLED < 3) selectedLED++;
    }

    // Display LED state
    lcd.clear();
    lcd.print(F("LED"));
    lcd.print(selectedLED + 1);
    lcd.print(F(" (%): "));
    
    // First line shows White channel
    lcd.print(F("W"));
    int w_val = (int)channelBrightness[selectedLED][0];
    if (w_val < 10) lcd.print(" ");  // Add space for single digit
    if (w_val < 100) lcd.print(" "); // Add space for double digit
    lcd.print(w_val);

    // Second line shows Blue, Green, Red
    lcd.setCursor(0, 1);
    
    // Blue
    lcd.print(F("B"));
    int b_val = (int)channelBrightness[selectedLED][1];
    if (b_val < 10) lcd.print(" ");
    if (b_val < 100) lcd.print(" ");
    lcd.print(b_val);
    lcd.print(F(" "));
    
    // Green
    lcd.print(F("G"));
    int g_val = (int)channelBrightness[selectedLED][2];
    if (g_val < 10) lcd.print(" ");
    if (g_val < 100) lcd.print(" ");
    lcd.print(g_val);
    lcd.print(F(" "));
    
    // Red (only if available)
    if (isChannelAvailable(selectedLED, 3)) {  // Check if RED channel is available
        lcd.print(F("R"));
        int r_val = (int)channelBrightness[selectedLED][3];
        if (r_val < 10) lcd.print(" ");
        if (r_val < 100) lcd.print(" ");
        lcd.print(r_val);
    }
}

void displayTemperatureScreen() {
    static unsigned long lastUpdate = 0;
    static bool firstDisplay = true;
    
    // Update temperature stats if needed
    if (firstDisplay || (millis() - lastUpdate >= 30000)) {
        lastUpdate = millis();
        firstDisplay = false;
        updateTemperatureStats();
        
        lcd.clear();
        // First line: Current temperatures
        lcd.print(F("A:"));
        lcd.print(tempStats.lastAmbientTemp, 1);
        lcd.print(F(" P:"));
        lcd.print(tempStats.lastPCBTemp, 1);
        
        // Second line: Min/Max
        lcd.setCursor(0, 1);
        lcd.print(F("L"));
        lcd.print(tempStats.minTemp, 1);
        lcd.print(F(" H"));
        lcd.print(tempStats.maxTemp, 1);
    }
}

void updateTemperatureStats() {
    float ambientTemp = RTC.getTemp();
    float pcbTemp = tempsensor.getTemperature();
    
    tempStats.lastAmbientTemp = ambientTemp;
    tempStats.lastPCBTemp = pcbTemp;
    
    // Update min/max
    if (pcbTemp < tempStats.minTemp || tempStats.minTemp == 0) {
        tempStats.minTemp = pcbTemp;
    }
    if (pcbTemp > tempStats.maxTemp) {
        tempStats.maxTemp = pcbTemp;
    }
    
    // Store hourly reading
    storeTemperatureReading();
    
    tempStats.lastUpdateTime = millis();
}

void displayElapsedTimeScreen() {
    static unsigned long displayUpdateTime = 0;
    unsigned long currentMillis = millis();
    
    // Update timing calculations
    updateExperimentTime();
    
    // Update display every second or on first entry
    if (experimentTiming.firstDisplay || currentMillis - displayUpdateTime >= 1000) {
        displayUpdateTime = currentMillis;
        experimentTiming.firstDisplay = false;
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Elapsed Time:"));
        
        lcd.setCursor(0, 1);
        if (experimentTiming.isRunning) {
            char timeStr[16];
            formatElapsedTime(timeStr, experimentTiming.elapsedSeconds);
            lcd.print(timeStr);
        } else {
            lcd.print(F("000d 00:00:00"));
        }
    }
}

void formatElapsedTime(char* buffer, unsigned long seconds) {
    unsigned long days = seconds / 86400;
    seconds %= 86400;
    unsigned long hours = seconds / 3600;
    seconds %= 3600;
    unsigned long minutes = seconds / 60;
    seconds %= 60;
    
    sprintf(buffer, "%03ldd %02lu:%02lu:%02lu", days, hours, minutes, seconds);
}

void toggleExperiment(bool start) {
    if (start) {
        experimentTiming.isRunning = true;
        experimentTiming.startMillis = millis();
        experimentTiming.correctedToday = false;
        experimentTiming.dailyErrorFast = 0;
        experimentTiming.dailyErrorBehind = 0;
    } else {
        experimentTiming.isRunning = false;
        experimentTiming.startMillis = 0;
    }
}

void storeTemperatureReading() {
    // Store in hourly readings array
    tempStats.hourlyReadings[tempStats.currentHourIndex] = tempStats.lastPCBTemp;
    tempStats.currentHourIndex = (tempStats.currentHourIndex + 1) % 24;
}

float calculateTemperatureAverage() {
    float sum = 0;
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < 24; i++) {
        if (tempStats.hourlyReadings[i] != 0) {
            sum += tempStats.hourlyReadings[i];
            count++;
        }
    }
    
    return count > 0 ? sum / count : 0;
}

void handleStatusDisplay(uint8_t buttons, uint8_t& displayMenuItem) {
    const uint8_t NUM_ITEMS = 4;  // Reduced from 5 (removed Current Time)
    static uint8_t tempSubMenu = 0; // 0 = Room Temp, 1 = PCB Temp
    static bool inTempSubMenu = false;
    
    // Reset state when first entering the menu
    if (statusDisplayState.isFirstEntry) {
        statusDisplayState.showContent = false;
        statusDisplayState.currentItem = 0;
        statusDisplayState.isFirstEntry = false;
        inTempSubMenu = false;
    }
    
    // Handle back button
    if (buttons & BUTTON_LEFT) {
        if (inTempSubMenu) {
            inTempSubMenu = false;
            statusDisplayState.showContent = true;
            return;
        }
        if (statusDisplayState.showContent) {
            statusDisplayState.showContent = false;
        } else {
            statusDisplayState.isFirstEntry = true;
            return;
        }
    }
    
    // If we're in temperature submenu
    if (inTempSubMenu) {
        if (buttons & BUTTON_UP && tempSubMenu > 0) tempSubMenu--;
        if (buttons & BUTTON_DOWN && tempSubMenu < 1) tempSubMenu++;
        
        lcd.clear();
        lcd.print(tempSubMenu == 0 ? ">" : " ");
        lcd.print(F("Room Temp (C\xDF)"));  // \xDF is the degree symbol
        lcd.setCursor(0, 1);
        lcd.print(tempSubMenu == 1 ? ">" : " ");
        lcd.print(F("PCB Temp (C\xDF)"));
        
        if (buttons & BUTTON_SELECT) {
            bool exitTemp = false;
            
            while(!exitTemp) {
                buttons = lcd.readButtons();
                if (buttons & BUTTON_LEFT) {
                    exitTemp = true;
                    break;
                }
                
                unsigned long currentMillis = millis();
                if (currentMillis - lastTempUpdate >= 1000) {  // Update every second
                    lastTempUpdate = currentMillis;
                    
                    if (tempSubMenu == 0) {
                        // Room Temperature
                        float roomTemp = RTC.getTemp();
                        updateTempHistory(roomTempHistory, roomTemp, maxRoomTemp);
                        
                        lcd.clear();
                        lcd.print(F("C:"));
                        lcd.print(roomTemp, 1);
                        lcd.print(F(",M:"));
                        lcd.print(maxRoomTemp, 1);
                        
                        lcd.setCursor(0, 1);
                        lcd.print(F("Avg:"));
                        lcd.print(calculateTempAverage(roomTempHistory), 1);
                    } else {
                        // PCB Temperature
                        float pcbTemp = tempsensor.getTemperature();
                        updateTempHistory(pcbTempHistory, pcbTemp, maxPCBTemp);
                        
                        lcd.clear();
                        lcd.print(F("C:"));
                        lcd.print(pcbTemp, 1);
                        lcd.print(F(",M:"));
                        lcd.print(maxPCBTemp, 1);
                        
                        lcd.setCursor(0, 1);
                        lcd.print(F("Avg:"));
                        lcd.print(calculateTempAverage(pcbTempHistory), 1);
                    }
                    
                    // Update circular buffer index
                    tempHistoryIndex = (tempHistoryIndex + 1) % 60;
                }
                
                delay(50);  // Small delay to prevent button bounce
            }
            
            statusDisplayState.showContent = false;
            return;
        }
        return;
    }
    
    // If we're showing content for main menu items
    if (statusDisplayState.showContent) {
        switch(displayMenuItem) {
            case 0:  // LED States
                displayLEDStateScreen(0, buttons);
                break;
            case 1:  // Elapsed Time
                displayElapsedTimeScreen();
                break;
            case 2:  // Temperature Menu
                inTempSubMenu = true;
                tempSubMenu = 0;
                statusDisplayState.showContent = false;
                return;
            case 3:  // Stop Experiment
                lcd.clear();
                lcd.print(F("Stop Experiment?"));
                lcd.setCursor(0, 1);
                if (experimentTiming.isRunning) {
                    lcd.print(F("Select to Stop"));
                    if (buttons & BUTTON_SELECT) {
                        toggleExperiment(false);  // Stop experiment and reset elapsed time
                        // Turn off all LEDs
                        const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
                        for(uint8_t i = 0; i < 4; i++) {
                            lt3966_i2c_write(addresses[i], GLBCFG, 0x0F);
                        }
                        lcd.clear();
                        lcd.print(F("Experiment"));
                        lcd.setCursor(0, 1);
                        lcd.print(F("Stopped!"));
                        delay(1000);
                        statusDisplayState.showContent = false;
                    }
                } else {
                    lcd.print(F("Not Running"));
                }
                break;
        }
        return;
    }
    
    // Handle menu navigation
    if (buttons & BUTTON_UP) {
        if (displayMenuItem > 0) displayMenuItem--;
    }
    if (buttons & BUTTON_DOWN) {
        if (displayMenuItem < NUM_ITEMS - 1) displayMenuItem++;
    }
    if (buttons & BUTTON_SELECT) {
        statusDisplayState.showContent = true;
        return;
    }
    
    // Display menu items
    lcd.clear();
    uint8_t firstItem = (displayMenuItem / 2) * 2;
    
    // First line
    if (firstItem < NUM_ITEMS) {
        lcd.print(displayMenuItem == firstItem ? ">" : " ");
        switch(firstItem) {
            case 0: lcd.print(F("LED States")); break;
            case 1: lcd.print(F("Elapsed Time")); break;
            case 2: lcd.print(F("Temperature")); break;
            case 3: lcd.print(F("Stop Experiment")); break;
        }
    }
    
    // Second line
    if (firstItem + 1 < NUM_ITEMS) {
        lcd.setCursor(0, 1);
        lcd.print(displayMenuItem == firstItem + 1 ? ">" : " ");
        switch(firstItem + 1) {
            case 1: lcd.print(F("Elapsed Time")); break;
            case 2: lcd.print(F("Temperature")); break;
            case 3: lcd.print(F("Stop Experiment")); break;
        }
    }
}

void handlePWMSettings(uint8_t buttons) {
    static uint8_t pwmMenuItem = 0;  // 0 = View Current PWM, 1 = Change PWM Config
    static bool inSubmenu = false;    // Track if we're in a submenu
    const uint8_t NUM_ITEMS = 2;
    
    // Handle back button
    if (buttons & BUTTON_LEFT) {
        if (inSubmenu) {
            inSubmenu = false;
            // Redraw the PWM settings menu
            lcd.clear();
            lcd.print(pwmMenuItem == 0 ? ">" : " ");
            lcd.print(F("View PWM"));
            lcd.setCursor(0, 1);
            lcd.print(pwmMenuItem == 1 ? ">" : " ");
            lcd.print(F("Change PWM"));
        } else {
            // Return to Status/Settings menu
            return;
        }
        return;
    }
    
    if (!inSubmenu) {
        // Handle main PWM settings menu navigation
        if (buttons & BUTTON_UP && pwmMenuItem > 0) {
            pwmMenuItem--;
        }
        if (buttons & BUTTON_DOWN && pwmMenuItem < NUM_ITEMS - 1) {
            pwmMenuItem++;
        }
        
        // Display menu
        lcd.clear();
        lcd.print(pwmMenuItem == 0 ? ">" : " ");
        lcd.print(F("View PWM"));
        lcd.setCursor(0, 1);
        lcd.print(pwmMenuItem == 1 ? ">" : " ");
        lcd.print(F("Change PWM"));
        
        // Handle selection
        if (buttons & BUTTON_SELECT) {
            inSubmenu = true;
            switch(pwmMenuItem) {
                case 0:  // View Current PWM
                    displayPWMStatus();
                    break;
                case 1:  // Change PWM Config
                    handlePWMFrequencyConfig(0);  // Initialize with no buttons pressed
                    break;
            }
        }
    } else {
        // Handle submenu
        if (pwmMenuItem == 0) {
            displayPWMStatus();  // Keep showing the PWM status
        } else {
            handlePWMFrequencyConfig(buttons);  // Pass through button presses
        }
    }
}

// New function to update LED cache in background
void updateLEDCache() {
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    static uint8_t currentLED = 0;
    static uint8_t currentChannel = 0;
    
    // Read one channel at a time
    uint8_t dimh_reg = DIM1H + (currentChannel * 0x10);
    uint8_t dimh_value, diml_value;
    
    if (lt3966_i2c_read(addresses[currentLED], dimh_reg, &dimh_value) &&
        lt3966_i2c_read(addresses[currentLED], dimh_reg + 1, &diml_value)) {
        
        uint8_t scl = (dimh_value >> 5) & 0x07;
        uint16_t pwm_value = ((dimh_value & 0x1F) << 8) | diml_value;
        uint16_t period = 1 << (6 + scl);
        ledStateCache.channelBrightness[currentLED][currentChannel] = (float)pwm_value / (period - 1) * 100.0f;
        
        // Move to next channel/LED
        currentChannel++;
        if (currentChannel >= 4) {
            currentChannel = 0;
            currentLED++;
            if (currentLED >= 4) {
                currentLED = 0;
                ledStateCache.lastUpdate = millis();
                ledStateCache.isValid = true;
            }
        }
    }
}

// Add time structure for RTC data
struct RTCDateTime {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    time_t timestamp;
};

// Function to get current time from RTC
RTCDateTime getCurrentTime() {
    RTCDateTime dt;
    dt.hour = RTC.getHours();
    dt.minute = RTC.getMinutes();
    dt.second = RTC.getSeconds();
    dt.day = RTC.getDay();
    dt.month = RTC.getMonth();
    dt.year = RTC.getYear();
    dt.timestamp = RTC.getEpoch(true);  // Get Unix timestamp
    return dt;
}

void displayCurrentTime() {
    char timeStr[20];
    sprintf(timeStr, "%02d:%02d:%02d", RTC.getHours(), RTC.getMinutes(), RTC.getSeconds());
    lcd.setCursor(0, 0);
    lcd.print("Time: ");
    lcd.print(timeStr);
    
    char dateStr[20];
    sprintf(dateStr, "%02d/%02d/%04d", RTC.getDay(), RTC.getMonth(), RTC.getYear());
    lcd.setCursor(0, 1);
    lcd.print("Date: ");
    lcd.print(dateStr);
}

// Add RTC time functions
time_t getRTCEpoch() {
    return RTC.getEpoch(true);
}

void logChannelStart(uint8_t led, uint8_t channel, float current) {
    RuntimeTracker runtimeTracker;
    runtimeTracker.logChannelStart(led, channel, current);
}

void logChannelEnd(uint8_t led, uint8_t channel) {
    RuntimeTracker runtimeTracker;
    runtimeTracker.logChannelStop(led, channel);
}

// Replace printDateTime function with printLogTime
void printLogTime(time_t epoch_time) {
    char buffer[32];
    time_t t = epoch_time;
    struct tm* timeinfo = localtime(&t);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    Serial.print(buffer);
}

void printLogEntry(const RuntimeLog& log) {
    Serial.print(F("Start: "));
    printLogTime(log.startTime);
    Serial.print(F(" End: "));
    printLogTime(log.endTime);
    Serial.print(F(" LED: "));
    Serial.print(log.led);
    Serial.print(F(" Channel: "));
    Serial.print(log.channel);
    Serial.print(F(" Current: "));
    Serial.print(log.current_mA);
    Serial.print(F("mA Temp: "));
    Serial.print(log.temperature);
    Serial.println(F("°C"));
}

// Add with other function declarations at the top
uint16_t getLogAddress(uint8_t led, uint8_t channel);

// Add the implementation after RuntimeLog structure definition
uint16_t getLogAddress(uint8_t led, uint8_t channel) {
    return sizeof(RuntimeLog) * (led * 4 + channel);
}

// Add this to setup() function after RTC initialization
void initializeExperimentState() {
    experimentTiming.isRunning = false;
    experimentTiming.startMillis = 0;
}

// Add new timing functions
void updateExperimentTime() {
    if (!experimentTiming.isRunning) return;
    
    unsigned long currentMillis = millis();
    // Handle millis() overflow
    if (currentMillis < experimentTiming.lastUpdate) {
        // Overflow occurred, adjust start time
        experimentTiming.startMillis = currentMillis;
        experimentTiming.lastUpdate = currentMillis;
        return;
    }
    
    // Calculate elapsed time
    unsigned long elapsedMillis = currentMillis - experimentTiming.startMillis;
    experimentTiming.elapsedSeconds = elapsedMillis / 1000;
    
    // Apply daily error correction if needed
    unsigned long daysSinceStart = experimentTiming.elapsedSeconds / 86400;
    if (daysSinceStart > 0 && !experimentTiming.correctedToday) {
        // Apply corrections for timing drift
        int totalCorrection = (experimentTiming.dailyErrorFast - experimentTiming.dailyErrorBehind) * daysSinceStart;
        experimentTiming.elapsedSeconds += totalCorrection / 1000;
        experimentTiming.correctedToday = true;
    }
    
    experimentTiming.lastUpdate = currentMillis;
}

// Initialize experiment timing
void initializeExperimentTiming() {
    experimentTiming.isRunning = false;
    experimentTiming.startMillis = 0;
    experimentTiming.elapsedSeconds = 0;
    experimentTiming.lastUpdate = 0;
    experimentTiming.firstDisplay = true;
    experimentTiming.dailyErrorFast = 0;
    experimentTiming.dailyErrorBehind = 0;
    experimentTiming.correctedToday = false;

    // Initialize temperature tracking
    roomTempTrack = {0};
    pcbTempTrack = {0};
    roomTempTrack.isFirstReading = true;
    pcbTempTrack.isFirstReading = true;
}

// Calculate 1-minute average temperature
float calculateMinuteAverage(TempTracking& tracker) {
    float sum = 0;
    int count = 0;
    
    for(int i = 0; i < 60; i++) {
        if(tracker.minuteReadings[i] != 0) {
            sum += tracker.minuteReadings[i];
            count++;
        }
    }
    
    return count > 0 ? sum / count : 0;
}

// Update temperature tracking
void updateTempTracking(TempTracking& tracker, float newTemp) {
    tracker.currentTemp = newTemp;
    
    // Update max temperature if this is first reading or new temp is higher
    if(tracker.isFirstReading || newTemp > tracker.maxTemp) {
        tracker.maxTemp = newTemp;
        tracker.isFirstReading = false;
    }
    
    // Store reading in circular buffer
    tracker.minuteReadings[tracker.readingIndex] = newTemp;
    tracker.readingIndex = (tracker.readingIndex + 1) % 60;
    tracker.lastUpdateTime = millis();
}

// Calculate average of temperature history
float calculateTempAverage(float* history) {
    float sum = 0;
    int count = 0;
    for(int i = 0; i < 60; i++) {
        if(history[i] != 0) {
            sum += history[i];
            count++;
        }
    }
    return count > 0 ? sum / count : 0;
}

// Update temperature history
void updateTempHistory(float* history, float newTemp, float& maxTemp) {
    history[tempHistoryIndex] = newTemp;
    if(maxTemp == 0 || newTemp > maxTemp) {
        maxTemp = newTemp;
    }
}

// Function declarations
void handlePhaseSettings(uint8_t buttons);
void applyPhaseConfig(uint8_t ledSelect, uint8_t channelSelect, bool inPhase);

void handlePhaseSettings(uint8_t buttons) {
    static PhaseMenuState menuState = PHASE_MODE_SELECT;
    static bool inPhaseMode = true;  // true = In-phase, false = Out-of-phase
    static uint8_t selectedLED = 0;   // 0 = All LEDs, 1-4 = Specific LED
    static uint8_t selectedChannel = 0; // 0 = All channels, 1-4 = Specific channel
    static bool firstEntry = true;
    static bool configApplied = false;

    // If this is first entry or config was just applied, reset the menu
    if (firstEntry || configApplied) {
        lcd.clear();
        switch (menuState) {
            case PHASE_MODE_SELECT:
                lcd.print(F("Phase Mode:"));
                lcd.setCursor(0, 1);
                lcd.print(inPhaseMode ? F("In-Phase") : F("Out-of-Phase"));
                break;
                
            case PHASE_LED_SELECT:
                lcd.print(F("Select LED:"));
                lcd.setCursor(0, 1);
                if (selectedLED == 0) {
                    lcd.print(F("All LEDs"));
                } else {
                    lcd.print(F("LED "));
                    lcd.print(selectedLED);
                }
                break;
                
            case PHASE_CHANNEL_SELECT:
                lcd.print(F("Select Channel:"));
                lcd.setCursor(0, 1);
                switch (selectedChannel) {
                    case 0: lcd.print(F("All Channels")); break;
                    case 1: lcd.print(F("White")); break;
                    case 2: lcd.print(F("Blue")); break;
                    case 3: lcd.print(F("Green")); break;
                    case 4: lcd.print(F("Red")); break;
                }
                break;
        }
        firstEntry = false;
        configApplied = false;
    }

    // Handle back button
    if (buttons & BUTTON_LEFT) {
        if (menuState > PHASE_MODE_SELECT) {
            menuState = (PhaseMenuState)(menuState - 1);
            firstEntry = true;
        } else {
            return;  // Exit to main menu
        }
        return;
    }

    // Handle navigation
    if (buttons & (BUTTON_UP | BUTTON_DOWN)) {
        switch (menuState) {
            case PHASE_MODE_SELECT:
                inPhaseMode = !inPhaseMode;  // Toggle between In-phase and Out-of-phase
                lcd.setCursor(0, 1);
                lcd.print(F("                "));  // Clear line
                lcd.setCursor(0, 1);
                lcd.print(inPhaseMode ? F("In-Phase") : F("Out-of-Phase"));
                break;
                
            case PHASE_LED_SELECT:
                if (buttons & BUTTON_UP) {
                    if (selectedLED > 0) selectedLED--;
                } else {
                    if (selectedLED < 4) selectedLED++;
                }
                lcd.setCursor(0, 1);
                lcd.print(F("                "));  // Clear line
                lcd.setCursor(0, 1);
                if (selectedLED == 0) {
                    lcd.print(F("All LEDs"));
                } else {
                    lcd.print(F("LED "));
                    lcd.print(selectedLED);
                }
                break;
                
            case PHASE_CHANNEL_SELECT:
                if (buttons & BUTTON_UP) {
                    if (selectedChannel > 0) selectedChannel--;
                } else {
                    if (selectedChannel < 4) selectedChannel++;
                }
                lcd.setCursor(0, 1);
                lcd.print(F("                "));  // Clear line
                lcd.setCursor(0, 1);
                switch (selectedChannel) {
                    case 0: lcd.print(F("All Channels")); break;
                    case 1: lcd.print(F("White")); break;
                    case 2: lcd.print(F("Blue")); break;
                    case 3: lcd.print(F("Green")); break;
                    case 4: lcd.print(F("Red")); break;
                }
                break;
        }
    }

    // Handle selection
    if (buttons & BUTTON_SELECT) {
        switch (menuState) {
            case PHASE_MODE_SELECT:
                menuState = PHASE_LED_SELECT;
                firstEntry = true;
                break;
                
            case PHASE_LED_SELECT:
                menuState = PHASE_CHANNEL_SELECT;
                firstEntry = true;
                break;
                
            case PHASE_CHANNEL_SELECT:
                // Apply the phase settings
                applyPhaseConfig(selectedLED, selectedChannel, inPhaseMode);
                
                // Show confirmation
                lcd.clear();
                lcd.print(F("Phase Updated!"));
                delay(1000);
                
                // Reset menu state for next time
                menuState = PHASE_MODE_SELECT;
                selectedLED = 0;
                selectedChannel = 0;
                firstEntry = true;
                configApplied = true;
                break;
        }
    }
}

void applyPhaseConfig(uint8_t ledSelect, uint8_t channelSelect, bool inPhase) {
    const uint8_t addresses[] = {LT3966_ADD1, LT3966_ADD2, LT3966_ADD3, LT3966_ADD4};
    
    // Determine which LEDs to configure
    uint8_t startLED = (ledSelect == 0) ? 0 : ledSelect - 1;
    uint8_t endLED = (ledSelect == 0) ? 3 : ledSelect - 1;
    
    // Determine which channels to configure
    uint8_t startCh = (channelSelect == 0) ? 0 : channelSelect - 1;
    uint8_t endCh = (channelSelect == 0) ? 3 : channelSelect - 1;
    
    // Configure selected LEDs and channels
    for(uint8_t led = startLED; led <= endLED; led++) {
        for(uint8_t ch = startCh; ch <= endCh; ch++) {
            configureChannel(addresses[led], ch, inPhase);
        }
    }
}

