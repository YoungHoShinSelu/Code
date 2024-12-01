
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <RTClib.h>
#include <LT_SPI.h>
#include <Linduino.h>
#include <QuikEval_EEPROM.h>
#include <UserInterface.h>
#include <LT3966.h>
#include <stdint.h>
#include <util/delay.h>
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
RTC_DS3231 rtc;

#define BACKLIGHT_PWM_PIN 10  // Define the PWM pin for channel 1 brightness control
#define LED_PWM_PIN_2 6   // Define the PWM pin for channel 2 brightness control
#define LED_PWM_PIN_3 5   // Define the PWM pin for channel 3 brightness control
#define LED_PWM_PIN_4 3   // Define the PWM pin for channel 4 brightness control
#define PCT2075_ADDR 0x48 // Set address for temp sensor 

#define ALERT_PIN   12

long readTerminal();
uint8_t readTerminalDecimal();
float readfloatTerminal();
uint8_t readTerminalBinary();
uint16_t calcDuty (float percentage);
uint8_t calcAnalog( float percentage);
void printPaddedBinary(uint8_t value);
void cycleColor(void);

unsigned long startTime = 0; 
boolean timerActive = false; // Flag to indicate if the timer is active
boolean inSubItem5 = false;

// These #defines make it easy to set the backlight color
#define RED 0x1 
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
#define DISABLE 0x8 

const int BaudRate = 115200;


int MainArray [0][4];  // 1 Main Menu Column with 6 Rows.  These numbers must match your string array items.
String menuItems[] = {"Select Module", "Check Temperture", "Elapsed Time","Info" };
String colors[] = {"RED", "YELLOW", "GREEN","TEAL"};

// Item array auto counter
#define NUMITEMS(menuItems) ((unsigned int) (sizeof (menuItems) / sizeof (menuItems [0])))
#define BGCOLORS(colors) ((unsigned int) (sizeof (colors) / sizeof (colors [0]))) 
int parameters = (NUMITEMS(menuItems));  // Number of menuItems
int maxMenuPages = (NUMITEMS (menuItems)-2); // Number of pages
 
int readKey; 
int menuPage = 0;
int button = 0;
int cursorPosition = 0;

//Creates custom arrow characters for the menu display
byte rightArrow[8] = {0x10,0x08,0x04,0x02,0x04,0x08,0x10,0x00};
byte upArrow[8] = {0x04,0x0E,0x1F,0x04,0x04,0x04,0x04,0x00};
byte downArrow[8] = {0x04,0x04,0x04,0x04,0x1F,0x0E,0x04,0x00};
byte leftArrow[8] = {0x01,0x02,0x04,0x08,0x04,0x02,0x01,0x00};
byte cancelMark[8] = {0x0,0x1b,0xe,0x4,0xe,0x1b,0x0}; 
byte checkMark[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0}; 

void NavRight(){ // Creates custom character display as functions for easy use within other functions.
  lcd.write(byte(0));
}
void NavUp(){ // menu navigation up arrow
  lcd.write(byte(1));
}
void NavDown(){ // menu navigation down arrow
  lcd.write(byte(2));
}
void NavLeft(){ // menu navigation left arrow
  lcd.write(byte(3));
}
void NavCancel(){ // menu navigation Cancel
  lcd.write(byte(4));
}
void NavConfirm(){ // menu navigation confirm
  lcd.write(byte(5));
}

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial.begin(BaudRate);
  lcd.begin(16, 2); 
  lcd.createChar(0, rightArrow); // Tranlates custom arrow characters from hex to lcd. Now initialized for global use after lcd.begin
  lcd.createChar(1, upArrow);
  lcd.createChar(2, downArrow);
  lcd.createChar(3, leftArrow);
  lcd.createChar(4, cancelMark);
  lcd.createChar(5, checkMark);
  lcd.setBacklight(RED);
  lcd.print("Welcome!!!");
  lcd.setCursor(0, 1);
  lcd.print("Main Menu");
  delay(500);
  lcd.setBacklight(GREEN);
  delay(500);
  lcd.setBacklight(BLUE);
  delay(500);

  rtc.begin();

 pinMode(LED_PWM_PIN_2, OUTPUT); // Set the PWM pin for subItem2 as an output
 pinMode(BACKLIGHT_PWM_PIN, OUTPUT); // Set the PWM pin for subItem1 as an output
 pinMode(LED_PWM_PIN_3, OUTPUT); // Set the PWM pin for subItem3 as an output
 pinMode(LED_PWM_PIN_4, OUTPUT); // Set the PWM pin for subItem4 as an output


  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting the time!");
    // This will reflect the time your code was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
    //TWSR = (HARDWARE_I2C_PRESCALER_1 & 0x03);       //! 1) set the TWI prescaler  (400kHz f_scl)
  TWBR = 12;                                      //! 2) set the TWI bit rate   (400kHz f_scl)

  //quikeval_I2C_connect();
  pinMode(ALERT_PIN, INPUT);

}



void mainMenuDraw() {
  lcd.setBacklight(WHITE);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(menuItems[menuPage]);
  lcd.setCursor(1, 1);
  lcd.print(menuItems[menuPage + 1]);
  if (menuPage == 0) {
    lcd.setCursor(15, 1);
    NavDown();
  } else if (menuPage > 0 and menuPage < maxMenuPages) {
    lcd.setCursor(15, 1);
    NavDown();
    lcd.setCursor(15, 0);
    NavUp();
  } else if (menuPage == maxMenuPages) {
    lcd.setCursor(15, 0);
    NavUp();
  }
}

// Cursor Navigation:
// When called, this function will erase the current cursor and redraw it based on the cursorPosition and menuPage variables.
void drawCursor() {
  for (int x = 0; x < 2; x++) { 
    lcd.setCursor(0, x);
    lcd.print(" ");
  }

  if (menuPage % 2 == 0) {
    if (cursorPosition % 2 == 0) {  // If the menu page is even and the cursor position is even that means the cursor should be on line 1
      lcd.setCursor(0, 0);
      NavRight();
    }
    if (cursorPosition % 2 != 0) {  // If the menu page is even and the cursor position is odd that means the cursor should be on line 2
      lcd.setCursor(0, 1);
      NavRight();
    }
  }
  if (menuPage % 2 != 0) {
    if (cursorPosition % 2 == 0) {  // If the menu page is odd and the cursor position is even that means the cursor should be on line 2
      lcd.setCursor(0, 1);
      NavRight();
    }
    if (cursorPosition % 2 != 0) {  // If the menu page is odd and the cursor position is odd that means the cursor should be on line 1
      lcd.setCursor(0, 0);
      NavRight();
    }
  }
}

void operateMainMenu() {
  int activeButton = 0;
  while (activeButton == 0) {
    readKey = lcd.readButtons();
    if (readKey == 0) {
      delay(100);
      readKey = lcd.readButtons();
    }
    button = evaluateButton(readKey);
    switch (button) {
      case 0: 
        break;
      case 1:   
      case 2:  
        button = 0;
        switch (cursorPosition) { 
          case 0:
            subItem1();
            break;
          case 1:
            subItem2();
            break;
          case 2:
            subItem3();
            break;
          case 3:
            subItem4();
            break;
//          case 4:       
//            break;
//          case 5:           
//            break;
//          case 6:
//            subItem7();
//            break;
//          case 7:
//            subItem8();
//            break;
//          case 8:
//            subItem9();
//            break;
//          case 9:
//            subItem10();
//            break;
//          case 10:
//            subItem11();
//            break;
//           case 11:
//            subItem12();
//            break;
        }
        activeButton = 1;
        mainMenuDraw();
        drawCursor();
        break;
      case 8: 
        button = 0;
        if (menuPage == 0) {
          cursorPosition = cursorPosition - 1;
          cursorPosition = constrain(cursorPosition, 0, ((sizeof(menuItems) / sizeof(String)) - 1));
        }
        if (menuPage % 2 == 0 and cursorPosition % 2 == 0) {
          menuPage = menuPage - 1;
          menuPage = constrain(menuPage, 0, maxMenuPages);
        }

        if (menuPage % 2 != 0 and cursorPosition % 2 != 0) {
          menuPage = menuPage - 1;
          menuPage = constrain(menuPage, 0, maxMenuPages);
        }

        cursorPosition = cursorPosition - 1;
        cursorPosition = constrain(cursorPosition, 0, ((sizeof(menuItems) / sizeof(String)) - 1));

        mainMenuDraw();
        drawCursor();
        activeButton = 1;
        break;
      case 4: 
        button = 0;
        if (menuPage % 2 == 0 and cursorPosition % 2 != 0) {
          menuPage = menuPage + 1;
          menuPage = constrain(menuPage, 0, maxMenuPages);
        }

        if (menuPage % 2 != 0 and cursorPosition % 2 == 0) {
          menuPage = menuPage + 1;
          menuPage = constrain(menuPage, 0, maxMenuPages);
        }

        cursorPosition = cursorPosition + 1;
        cursorPosition = constrain(cursorPosition, 0, ((sizeof(menuItems) / sizeof(String)) - 1));
        mainMenuDraw();
        drawCursor();
        activeButton = 1;
        break;
       default: break;
    }
  }
}


int evaluateButton(int x) {
  uint8_t buttons = lcd.readButtons();
  int result = 0;
  if (buttons & BUTTON_RIGHT) {
    result = 2; 
    Serial.print("Button Right ");
    Serial.println (BUTTON_RIGHT); 
  } else if (buttons & BUTTON_UP) {
    result = 8; 
    Serial.print("Button Up ");
    Serial.println (BUTTON_UP); 
  } else if (buttons & BUTTON_DOWN) {
    result = 4; 
    Serial.print("Button Down ");
    Serial.println (BUTTON_DOWN); 
  } else if (buttons & BUTTON_LEFT) {
    result = 16; 
    Serial.print("Button Left ");
    Serial.println (BUTTON_LEFT); 
  } else if (buttons & BUTTON_SELECT) { 
    result = 1; 
    Serial.print("Button Select ");
    Serial.println (BUTTON_SELECT); 
  }
  delay (100);
  return result;
}

void backbutton (){ // Left button navigates back until mainmenu
   int activeButton = 0;
     while (activeButton == 0) {
     uint8_t button = lcd.readButtons();
    readKey = lcd.readButtons();
    if (button & BUTTON_LEFT) {
      // delay(50);
    }
    button = evaluateButton(readKey);
    switch (button) {
      case 16:  // This case will execute if the "back" button is pressed
        button = 0;
        activeButton = 1;
        lcd.setBacklight(WHITE);
        break;
    }
  } 
}


 void subCursor() {   // clears screen adds back cursor to each subItem screen
  lcd.clear();
  lcd.setCursor(0,0);
  NavLeft();
  lcd.setCursor(2, 0);
}
  
  
#define LT3966_ADD  0x5F  // Changed address to 0x5F

uint16_t calcDuty(int percentage) {
    // Mapping the percentage to an 8-bit range (0-255)
    return static_cast<uint16_t>((percentage / 100.0) * 255);
}


void subItem1() {
  int optionSelection = 0;
  const String optionMenu[] = {"Module 1", "Module 2", "Module 3", "Module 4"};
  const int numOptions = sizeof(optionMenu) / sizeof(optionMenu[0]);

  while (true) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select Module:");
    lcd.setCursor(0, 1);
    lcd.print(optionMenu[optionSelection]);

    uint8_t buttons = lcd.readButtons();
    if (buttons & BUTTON_UP) {
      optionSelection = (optionSelection + 1) % numOptions;
      delay(200);
    } else if (buttons & BUTTON_DOWN) {
      optionSelection = (optionSelection - 1 + numOptions) % numOptions;
      delay(200);
    } else if (buttons & BUTTON_SELECT) {
      executeOption1(optionSelection);
      break;
    } else if (buttons & BUTTON_LEFT) {
      break;
    }
    delay(100); 
  }
}

void executeOption1(int option) {
  switch (option) {
    case 0:
      subItem7(); // Add your desired action for Module 1 here
      break;
    case 1:
     
      break;
    case 2:
      // Implement the action for Module 3
      // You can add your code here
      break;
    case 3:
      // Implement the action for Module 4
      // You can add your code here
      break;
    default:
      break;
  }
}



void subItem2() {  // Function executes when you select the 2nd item from main menu
  int optionSelection = 0;
  const String optionMenu[] = {"Module 1", "Module 2", "Module 3", "Module 4"};
  const int numOptions = sizeof(optionMenu) / sizeof(optionMenu[0]);

  while (true) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select Module:");
    lcd.setCursor(0, 1);
    lcd.print(optionMenu[optionSelection]);

    uint8_t buttons = lcd.readButtons();
    if (buttons & BUTTON_UP) {
      optionSelection = (optionSelection + 1) % numOptions;
      delay(200);
    } else if (buttons & BUTTON_DOWN) {
      optionSelection = (optionSelection - 1 + numOptions) % numOptions;
      delay(200);
    } else if (buttons & BUTTON_SELECT) {
      executeOption2(optionSelection);
      break;
    } else if (buttons & BUTTON_LEFT) {
      break;
    }
    delay(100);
  }
}

void executeOption2(int option) {
  switch (option) {
    case 0:
    subItem6();
      break;
    case 1:
      // Implement the action for Option 2
      // You can add your code here
      break;
    case 2:
      // Implement the action for Option 3
      // You can add your code here
      break;
    case 3:
      // Implement the action for Option 4
      // You can add your code here
      break;
    default:
      break;
  }
}
void subItem3() { // Function executes when you select the 3rd item from main menu
  int optionSelection = 0;
  const String optionMenu[] = {"Module 1", "Module 2", "Module 3", "Module 4"};
  const int numOptions = sizeof(optionMenu) / sizeof(optionMenu[0]);

  while (true) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select Module:");
    lcd.setCursor(0, 1);
    lcd.print(optionMenu[optionSelection]);

    uint8_t buttons = lcd.readButtons();
    if (buttons & BUTTON_UP) {
      optionSelection = (optionSelection + 1) % numOptions;
      delay(200);
    } else if (buttons & BUTTON_DOWN) {
      optionSelection = (optionSelection - 1 + numOptions) % numOptions;
      delay(200);
    } else if (buttons & BUTTON_SELECT) {
      executeOption3(optionSelection);
      break;
    } else if (buttons & BUTTON_LEFT) {
      break;
    }
    delay(100);
  }
}

void executeOption3(int option) {
  switch (option) {
    case 0:
    subItem5();
      break;
    case 1:
      // Implement the action for Option 2
      // You can add your code here
      break;
    case 2:
      // Implement the action for Option 3
      // You can add your code here
      break;
    case 3:
      // Implement the action for Option 4
      // You can add your code here
      break;
    default:
      break;
  }
}
void subItem4() { // Function executes when you select the 3rd item from main menu
  int optionSelection = 0;
  const String optionMenu[] = {"Module 1", "Module 2", "Module 3", "Module 4"};
  const int numOptions = sizeof(optionMenu) / sizeof(optionMenu[0]);

  while (true) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select Module:");
    lcd.setCursor(0, 1);
    lcd.print(optionMenu[optionSelection]);

    uint8_t buttons = lcd.readButtons();
    if (buttons & BUTTON_UP) {
      optionSelection = (optionSelection + 1) % numOptions;
      delay(200);
    } else if (buttons & BUTTON_DOWN) {
      optionSelection = (optionSelection - 1 + numOptions) % numOptions;
      delay(200);
    } else if (buttons & BUTTON_SELECT) {
      executeOption4(optionSelection);
      break;
    } else if (buttons & BUTTON_LEFT) {
      break;
    }
    delay(100);
  }
}

void executeOption4(int option) {
  switch (option) {
    case 0:
    subItem8();
      break;
    case 1:
      // Implement the action for Option 2
      // You can add your code here
      break;
    case 2:
      // Implement the action for Option 3
      // You can add your code here
      break;
    case 3:
      // Implement the action for Option 4
      // You can add your code here
      break;
    default:
      break;
  }
}

void setBrightness1(int percentage) {
  uint8_t duty = static_cast<uint8_t>((percentage / 100.0) * 255); // Calculate 8-bit duty cycle
  analogWrite(BACKLIGHT_PWM_PIN , duty); // Set PWM duty cycle for channel 1
}

void subItem7() { // Function executes when you select the 4th item from main menu
  int brightnessSelection = 0;
  const int brightnessLevels[] = {0, 25, 50, 75, 100}; 
  const int numBrightnessLevels = sizeof(brightnessLevels) / sizeof(brightnessLevels[0]);

  while (true) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Brightness:");
    lcd.setCursor(0, 1);
    lcd.print(brightnessLevels[brightnessSelection]);
    lcd.print("%");

    uint8_t buttons = lcd.readButtons();
    if (buttons & BUTTON_UP) {
      brightnessSelection = (brightnessSelection + 1) % numBrightnessLevels;
      delay(200);
    } else if (buttons & BUTTON_DOWN) {
      brightnessSelection = (brightnessSelection - 1 + numBrightnessLevels) % numBrightnessLevels;
      delay(200);
    } else if (buttons & BUTTON_SELECT) {
      setBrightness1(brightnessLevels[brightnessSelection]);
      break;
    } else if (buttons & BUTTON_LEFT) {
      break;
    }
    delay(100);
  }
}

void subItem5() {
  inSubItem5 = true; // Set the flag when entering subItem5
  subCursor();
  lcd.setCursor(3, 0);
  lcd.print("Sub Menu 5");
  lcd.setBacklight(BLUE);
  lcd.setCursor(0, 1);
  lcd.print("Start Timer");
  NavConfirm(); 
  // Get the current time from the RTC
  DateTime now = rtc.now();

  // Record the start time only if the timer is not active
  if (!timerActive) {
    startTime = now.unixtime();
    timerActive = true;
  }

  while (inSubItem5) { // Loop until the user backs out
    if (timerActive) {
      updateElapsedTime(); // Display elapsed time only when in subItem5
    }
    
    // Check for button press to exit
    uint8_t buttons = lcd.readButtons();
    if (buttons & BUTTON_RIGHT) {
      timerActive = false; // Stop the timer
    } else if (buttons & BUTTON_LEFT) {
      // Back out of subItem5 if left button is pressed
      inSubItem5 = false;
      break;
    }
    delay(1000); // Update every second
  }

  inSubItem5 = false; // Reset the flag when exiting subItem5

  // If the timer was stopped, display final elapsed time
  if (!timerActive) {
    updateElapsedTime();
    delay(2000); // Display for 2 seconds
  }
}


float readTemperature() {
  Wire.beginTransmission(PCT2075_ADDR);
  Wire.write(0x00); // Temperature register
  Wire.endTransmission();
  
  Wire.requestFrom(PCT2075_ADDR, 2); // Request 2 bytes for temperature
  if (Wire.available() == 2) {
    int tempRaw = (Wire.read() << 8) | Wire.read(); // Read the two bytes
    return tempRaw / 256.0; // Convert to Celsius
  }
  return -999; // Error value if data not received
}

void subItem6() { // Function executes when you select the 6th item from main menu
  unsigned long lastUpdateTime = 0; // Variable to track the last update time
  float temperature = 0; // Initialize temperature variable

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temperature:");
  lcd.setCursor(0, 1);

  while (true) {
    // Check if it's time to update the temperature (every 500 milliseconds)
    if (millis() - lastUpdateTime >= 500) {
      temperature = readTemperature(); // Read the temperature
      lcd.setCursor(0, 1);
      if (temperature != -999) {
        lcd.print(temperature);
        lcd.print(" C");
      } else {
        lcd.print("Error");
      }
      lastUpdateTime = millis(); // Update the last update time
    }

    // Check for button press to exit
    if (lcd.readButtons() & BUTTON_LEFT) {
      break; // Exit the loop if the left button is pressed
    }

    delay(100); // Delay to avoid continuous polling
  }

  lcd.setBacklight(VIOLET);
  backbutton ();
}

void updateElapsedTime() {
  // Get the current time
  DateTime currentTime = rtc.now();

  // Calculate elapsed time in seconds
  unsigned long elapsedTime = currentTime.unixtime() - startTime;

  // Calculate hours, minutes, and seconds
  unsigned long hours = elapsedTime / 3600;
  unsigned long minutes = (elapsedTime % 3600) / 60;
  unsigned long seconds = elapsedTime % 60;

  // Display elapsed time
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Elapsed Time:");
  lcd.setCursor(0, 1);
  lcd.print(String(hours) + "h " + String(minutes) + "m " + String(seconds) + "s");
}

void loop() {
  mainMenuDraw();
  drawCursor();
// this keeps updating the elapsed time even if your not displaying it
  if (timerActive && inSubItem5) {
    updateElapsedTime();
  }

  operateMainMenu();
}
void subItem8() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sub Item 5:");
  lcd.print("Elapsed Time: ");
  
  // Call subItem5() to display elapsed time
  subItem5();

  // Call subItem6() to display temperature
  lcd.print("Temperature: ");
  lcd.setCursor(0, 1);
  lcd.print(readTemperature());
  lcd.print(" C");

  // Call subItem7() to display brightness
  
  lcd.print("Brightness: ");
  lcd.print(analogRead(LED_PWM_PIN_4));
  lcd.print("%");
}