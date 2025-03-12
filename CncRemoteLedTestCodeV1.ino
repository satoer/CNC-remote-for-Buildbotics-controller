/*
  CNC Remote Panel LED Test Program
  
  This program sequentially turns each LED on and off with configurable timing.
  Designed for testing the LED functionality on the Buildbotics / Onefinity CNC controller remote.
  
  by Michel Satoer, Satoer Creations.
  
  Required Library:
  - ShiftRegister74HC595 Arduino Library by Timo Denk:
    https://timodenk.com/blog/shift-register-arduino-library/
*/

#include <ShiftRegister74HC595.h>

// ======= CONFIGURATION OPTIONS =======
// Time settings (milliseconds)
#define LED_ON_TIME       1000    // How long each LED stays on (1 second default)
#define LED_OFF_TIME      500     // How long each LED stays off before next LED (0.5 second default)
#define CYCLE_PAUSE_TIME  2000    // Pause time between complete cycles (2 seconds default)

// Display mode options
#define SEQUENTIAL_MODE   true    // If true, lights LEDs one by one; if false, all at once
#define SHOW_LED_NUMBER   true    // If true, prints LED number to serial monitor
#define RUN_FOREVER       true    // If true, runs continuously; if false, runs only once

// Pin configuration
#define SERIAL_DATA_PIN   A3      // DS pin
#define CLOCK_PIN         A5      // SHCP pin
#define LATCH_PIN         A4      // STCP pin

// Total number of LEDs (using 2 shift registers, 8 bits each)
#define TOTAL_LEDS        16

// Optional: Named LED positions for clarity
// These match the original code's LED definitions
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

// Create shift register object
ShiftRegister74HC595<2> led(SERIAL_DATA_PIN, CLOCK_PIN, LATCH_PIN);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println(F("CNC Remote Panel LED Test Program"));
  Serial.println(F("Testing all LEDs in sequence..."));
  
  // Flash all LEDs at startup to confirm they're working
  flashAllLEDs(3);
}

void loop() {
  // Test all LEDs in sequence
  if (SEQUENTIAL_MODE) {
    testLEDsSequentially();
  } else {
    testAllLEDsTogether();
  }
  
  // Pause between complete cycles
  led.setAllLow();
  Serial.println(F("Cycle complete. Pausing..."));
  delay(CYCLE_PAUSE_TIME);
  
  // Stop after one cycle if RUN_FOREVER is false
  if (!RUN_FOREVER) {
    Serial.println(F("Test complete. Halting program."));
    while(1) {
      // Infinite loop to stop program
      delay(1000);
    }
  }
}

// Function to test each LED individually in sequence
void testLEDsSequentially() {
  for (int i = 0; i < TOTAL_LEDS; i++) {
    // Turn off all LEDs first
    led.setAllLow();
    
    // Print LED number if enabled
    if (SHOW_LED_NUMBER) {
      Serial.print(F("Testing LED #"));
      Serial.print(i);
      
      // Print LED name if it has a specific function
      printLEDName(i);
      
      Serial.println(F("..."));
    }
    
    // Turn on current LED
    led.set(i, HIGH);
    delay(LED_ON_TIME);
    
    // Turn off current LED
    led.set(i, LOW);
    delay(LED_OFF_TIME);
  }
}

// Function to test all LEDs at the same time
void testAllLEDsTogether() {
  // Turn all LEDs on
  Serial.println(F("All LEDs ON"));
  led.setAllHigh();
  delay(LED_ON_TIME);
  
  // Turn all LEDs off
  Serial.println(F("All LEDs OFF"));
  led.setAllLow();
  delay(LED_OFF_TIME);
}

// Function to flash all LEDs a specified number of times
void flashAllLEDs(int times) {
  Serial.println(F("Flashing all LEDs..."));
  for (int i = 0; i < times; i++) {
    led.setAllHigh();
    delay(300);
    led.setAllLow();
    delay(300);
  }
}

// Function to print the name of each LED based on its number
void printLEDName(int ledNumber) {
  Serial.print(F(" ("));
  
  switch (ledNumber) {
    case LED_X_AXIS:
      Serial.print(F("X-Axis Status"));
      break;
    case LED_Y_AXIS:
      Serial.print(F("Y-Axis Status"));
      break;
    case LED_SPEED_0:
      Serial.print(F("Speed 0"));
      break;
    case LED_SPEED_1:
      Serial.print(F("Speed 1"));
      break;
    case LED_SPEED_2:
      Serial.print(F("Speed 2"));
      break;
    case LED_SPEED_3:
      Serial.print(F("Speed 3"));
      break;
    case LED_ROUTER:
      Serial.print(F("Router Status"));
      break;
    case LED_DUST_COLLECTOR:
      Serial.print(F("Dust Collector Status"));
      break;
    case LED_ACC_1:
      Serial.print(F("Accessory 1"));
      break;
    case LED_ACC_2:
      Serial.print(F("Accessory 2"));
      break;
    case LED_ROUTER_ON:
      Serial.print(F("Router ON Mode"));
      break;
    case LED_ROUTER_OFF:
      Serial.print(F("Router OFF Mode"));
      break;
    case LED_ROUTER_AUTO:
      Serial.print(F("Router AUTO Mode"));
      break;
    case LED_DUST_ON:
      Serial.print(F("Dust Collector ON Mode"));
      break;
    case LED_DUST_OFF:
      Serial.print(F("Dust Collector OFF Mode"));
      break;
    case LED_DUST_AUTO:
      Serial.print(F("Dust Collector AUTO Mode"));
      break;
    default:
      Serial.print(F("Undefined"));
      break;
  }
  
  Serial.print(F(")"));
}