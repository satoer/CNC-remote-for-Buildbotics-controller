/*
  CNC Remote Panel Button-LED Test Program
  
  This test program lights up LEDs when corresponding buttons are pressed.
  Special behavior for router and dust collector buttons:
  - When pressed, they cycle through AUTO -> ON -> OFF states
  - The corresponding mode LED stays on until next button press
  
  by Michel Satoer, Satoer Creations.
  
  Required Library:
  - ShiftRegister74HC595 Arduino Library by Timo Denk:
    https://timodenk.com/blog/shift-register-arduino-library/
*/

#include <ShiftRegister74HC595.h>

// ======= CONFIGURATION OPTIONS =======
// Debug settings
#define DEBUG_MODE         true    // Enable serial debug messages
#define SERIAL_BAUD_RATE   9600    // Serial baud rate

// Button settings
#define BUTTON_DEBOUNCE_TIME 20    // Debounce time for buttons in milliseconds

// Pin configuration for shift register
#define SERIAL_DATA_PIN   A3      // DS pin
#define CLOCK_PIN         A5      // SHCP pin
#define LATCH_PIN         A4      // STCP pin

// LED and button definitions from original code
#define LED_X_AXIS        0       // X-Axis Status
#define LED_Y_AXIS        7       // Y-Axis Status
#define LED_SPEED_0       8       // Speed Setting 0
#define LED_SPEED_1       9       // Speed Setting 1
#define LED_SPEED_2       10      // Speed Setting 2
#define LED_SPEED_3       11      // Speed Setting 3
#define LED_ROUTER        12      // Router Status
#define LED_DUST_COLLECTOR 13     // Dust Collector Status
#define LED_ACC_1         14      // Accessory 1 Status
#define LED_ACC_2         15      // Accessory 2 Status
#define LED_ROUTER_ON     3       // Router ON Mode
#define LED_ROUTER_OFF    2       // Router OFF Mode
#define LED_ROUTER_AUTO   1       // Router AUTO Mode
#define LED_DUST_ON       6       // Dust Collector ON Mode
#define LED_DUST_OFF      5       // Dust Collector OFF Mode
#define LED_DUST_AUTO     4       // Dust Collector AUTO Mode

// Mode states
#define MODE_AUTO         0
#define MODE_ON           1
#define MODE_OFF          2

// Create shift register object
ShiftRegister74HC595<2> led(SERIAL_DATA_PIN, CLOCK_PIN, LATCH_PIN);

// Button matrix configuration
byte buttons[2][5] = {  // The symbols of the keys
  { 1, 2, 3, 4, 5 },
  { 6, 7, 8, 9, 10 }
};

// State variables
byte previousButton = 0;
byte routerMode = MODE_OFF;
byte dustCollectorMode = MODE_OFF;

void setup() {
  // Initialize serial communication
  if (DEBUG_MODE) {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println(F("CNC Remote Panel Button-LED Test Program"));
    Serial.println(F("Press buttons to test LED functionality..."));
  }
  
  // Button matrix pins setup
  // Rows as inputs
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  
  // Columns as inputs initially
  for (int col = 9; col <= 13; col++) {
    pinMode(col, INPUT);
  }
  
  // Flash all LEDs at startup
  flashAllLEDs(3);
  
  // Reset all LEDs
  led.setAllLow();
  
  // Set initial router and dust collector mode indicators
  updateRouterLEDs();
  updateDustCollectorLEDs();
}

void loop() {
  // Read button state
  byte keypress = readButtonMatrix();
  
  // Process button press or release
  if (keypress != 0) {
    if (keypress <= 100) { // Button press
      handleButtonPress(keypress);
    } else { // Button release (keypress > 100)
      handleButtonRelease(keypress - 100);
    }
  }
}

// Function to detect button presses from the matrix
byte readButtonMatrix() {
  byte buttonRead = 100;
  
  for (int col = 9; col <= 13; col++) {
    pinMode(col, OUTPUT);
    digitalWrite(col, HIGH);
    
    for (int row = 7; row <= 8; row++) {
      if (digitalRead(row) == 1) {
        buttonRead = buttons[row - 7][col - 9];
      }
    }
    
    pinMode(col, INPUT);
  }
  
  if (previousButton != buttonRead) {
    byte returnButton = buttonRead;
    if (buttonRead == 100) {  // Add 100 if it's a release of button
      returnButton = 100 + previousButton;
    }
    
    previousButton = buttonRead;
    delay(BUTTON_DEBOUNCE_TIME);  // Debounce button
    return returnButton;
  } else {
    return 0;  // No change in button state
  }
}

// Handle button press events
void handleButtonPress(byte buttonNumber) {
  if (DEBUG_MODE) {
    Serial.print(F("Button "));
    Serial.print(buttonNumber);
    Serial.print(F(" ("));
    printButtonName(buttonNumber);
    Serial.println(F(") pressed"));
  }
  
  // Light up the corresponding LED based on button number
  switch (buttonNumber) {
    case 1:  // X-Axis button
      led.set(LED_X_AXIS, HIGH);
      break;
      
    case 2:  // Y-Axis button
      led.set(LED_Y_AXIS, HIGH);
      break;
      
    case 3:  // Speed 0 button
      led.set(LED_SPEED_0, HIGH);
      break;
      
    case 4:  // Speed 1 button
      led.set(LED_SPEED_1, HIGH);
      break;
      
    case 5:  // Speed 2 button
      led.set(LED_SPEED_2, HIGH);
      break;
      
    case 6:  // Speed 3 button
      led.set(LED_SPEED_3, HIGH);
      break;
      
    case 7:  // Router button - cycle through modes
      led.set(LED_ROUTER, HIGH);  // Light up the router button LED
      // Cycle router mode: AUTO -> ON -> OFF -> AUTO...
      routerMode = (routerMode + 1) % 3;
      updateRouterLEDs();
      
      if (DEBUG_MODE) {
        Serial.print(F("Router mode changed to: "));
        printModeName(routerMode);
        Serial.println();
      }
      break;
      
    case 8:  // Dust collector button - cycle through modes
      led.set(LED_DUST_COLLECTOR, HIGH);  // Light up the dust collector button LED
      // Cycle dust collector mode: AUTO -> ON -> OFF -> AUTO...
      dustCollectorMode = (dustCollectorMode + 1) % 3;
      updateDustCollectorLEDs();
      
      if (DEBUG_MODE) {
        Serial.print(F("Dust collector mode changed to: "));
        printModeName(dustCollectorMode);
        Serial.println();
      }
      break;
      
    case 9:  // Accessory 1 button
      led.set(LED_ACC_1, HIGH);
      break;
      
    case 10:  // Accessory 2 button
      led.set(LED_ACC_2, HIGH);
      break;
  }
}

// Handle button release events
void handleButtonRelease(byte buttonNumber) {
  if (DEBUG_MODE) {
    Serial.print(F("Button "));
    Serial.print(buttonNumber);
    Serial.print(F(" ("));
    printButtonName(buttonNumber);
    Serial.println(F(") released"));
  }
  
  // Turn off the corresponding LED based on button number
  // (except for router and dust collector mode LEDs which stay on)
  switch (buttonNumber) {
    case 1:  // X-Axis button
      led.set(LED_X_AXIS, LOW);
      break;
      
    case 2:  // Y-Axis button
      led.set(LED_Y_AXIS, LOW);
      break;
      
    case 3:  // Speed 0 button
      led.set(LED_SPEED_0, LOW);
      break;
      
    case 4:  // Speed 1 button
      led.set(LED_SPEED_1, LOW);
      break;
      
    case 5:  // Speed 2 button
      led.set(LED_SPEED_2, LOW);
      break;
      
    case 6:  // Speed 3 button
      led.set(LED_SPEED_3, LOW);
      break;
      
    case 7:  // Router button
      led.set(LED_ROUTER, LOW);  // Turn off just the router button LED
      break;
      
    case 8:  // Dust collector button
      led.set(LED_DUST_COLLECTOR, LOW);  // Turn off just the dust collector button LED
      break;
      
    case 9:  // Accessory 1 button
      led.set(LED_ACC_1, LOW);
      break;
      
    case 10:  // Accessory 2 button
      led.set(LED_ACC_2, LOW);
      break;
  }
}

// Update router mode LEDs based on current mode
void updateRouterLEDs() {
  // Turn off all router mode LEDs first
  led.set(LED_ROUTER_AUTO, LOW);
  led.set(LED_ROUTER_ON, LOW);
  led.set(LED_ROUTER_OFF, LOW);
  
  // Turn on the appropriate LED based on current mode
  switch (routerMode) {
    case MODE_AUTO:
      led.set(LED_ROUTER_AUTO, HIGH);
      break;
    case MODE_ON:
      led.set(LED_ROUTER_ON, HIGH);
      break;
    case MODE_OFF:
      led.set(LED_ROUTER_OFF, HIGH);
      break;
  }
}

// Update dust collector mode LEDs based on current mode
void updateDustCollectorLEDs() {
  // Turn off all dust collector mode LEDs first
  led.set(LED_DUST_AUTO, LOW);
  led.set(LED_DUST_ON, LOW);
  led.set(LED_DUST_OFF, LOW);
  
  // Turn on the appropriate LED based on current mode
  switch (dustCollectorMode) {
    case MODE_AUTO:
      led.set(LED_DUST_AUTO, HIGH);
      break;
    case MODE_ON:
      led.set(LED_DUST_ON, HIGH);
      break;
    case MODE_OFF:
      led.set(LED_DUST_OFF, HIGH);
      break;
  }
}

// Function to flash all LEDs a specified number of times
void flashAllLEDs(int times) {
  if (DEBUG_MODE) {
    Serial.println(F("Flashing all LEDs..."));
  }
  
  for (int i = 0; i < times; i++) {
    led.setAllHigh();
    delay(300);
    led.setAllLow();
    delay(300);
  }
}

// Print the name of a button for debugging
void printButtonName(byte buttonNumber) {
  switch (buttonNumber) {
    case 1:
      Serial.print(F("X-Axis"));
      break;
    case 2:
      Serial.print(F("Y-Axis"));
      break;
    case 3:
      Serial.print(F("Speed 0"));
      break;
    case 4:
      Serial.print(F("Speed 1"));
      break;
    case 5:
      Serial.print(F("Speed 2"));
      break;
    case 6:
      Serial.print(F("Speed 3"));
      break;
    case 7:
      Serial.print(F("Router"));
      break;
    case 8:
      Serial.print(F("Dust Collector"));
      break;
    case 9:
      Serial.print(F("Accessory 1"));
      break;
    case 10:
      Serial.print(F("Accessory 2"));
      break;
    default:
      Serial.print(F("Unknown"));
      break;
  }
}

// Print the name of a mode for debugging
void printModeName(byte mode) {
  switch (mode) {
    case MODE_AUTO:
      Serial.print(F("AUTO"));
      break;
    case MODE_ON:
      Serial.print(F("ON"));
      break;
    case MODE_OFF:
      Serial.print(F("OFF"));
      break;
    default:
      Serial.print(F("Unknown"));
      break;
  }
}