/*
  Buildbotics / Onefinity CNC controller remote source code. V1.5 (C) 2021 Michel Satoer, Satoer Creations.
  info: https://satoer.com/cnc-remote-panel-v3/
  
  Used Libraries:
  ShiftRegister74HC595 Arduino Library by Timo Denk:
  https://timodenk.com/blog/shift-register-arduino-library/

  Multimap by Robin Tillaart:
  https://github.com/RobTillaart/MultiMap

  Arduino Joystick Library by Matthew Heironimus:
  https://github.com/MHeironimus/ArduinoJoystickLibrary

*/

#include <ShiftRegister74HC595.h>
#include <MultiMap.h>
#include <EEPROM.h>
#include <Joystick.h>

#define epromChecksum 31415  // change this to a different random number beween 0 and 65535 if you want to erase the eeprom and load default values.
#define dustCollectorDelay 1000 // This is the delay time in ms for the dust collector in AUTO mode. This delay prevents a big power surge by not starting 2 devices at once. Change to 0 if you don't want this behaviour
#define debugMode false
#define buttonDebounceTime 20
#define sToggleSwitch 0
#define sMomentarySwitch 1
#define sOn 0
#define sOff 1
#define sAuto 2

//Onefinity gamepad mapping (linux):
#define jBtnMapSpeed0 3
#define jBtnMapSpeed1 0
#define jBtnMapSpeed2 1
#define jBtnMapSpeed3 4
#define jBtnMapLockX 7
#define jBtnMapLockY 6

#define xAxis 0
#define yAxis 1

#define ledXAxis 0
#define ledYAxis 7
#define ledSpeed0 8
#define ledSpeed1 9
#define ledSpeed2 10
#define ledSpeed3 11
#define ledRouter 12
#define ledDustCollector 13
#define ledAcc1 14
#define ledAcc2 15
#define ledRouterOn 3
#define ledRouterOff 2
#define ledRouterAuto 1
#define ledDustCollectorOn 6
#define ledDustCollectorOff 5
#define ledDustCollectorAuto 4

#define xAxisControl A0
#define yAxisControl A1
#define zAxisControl A2

#define relayRouter 5
#define relayDustCollector 6
#define relayAcc1 3
#define relayAcc2 4
#define cncInputIO 2

#define xYAxis0DeadMargin 200
#define zAxis0DeadMargin 200

struct settingsStruct {
  unsigned int checksum;
  bool reverseXAxis;
  bool reverseYAxis;
  bool reverseZAxis;
  byte acc1Type;
  byte acc2Type;
  byte dustCollectorDefault;
  byte routerDefault;
  byte speedDefault;
  bool XAxisReversed;
  bool YAxisReversed;
  bool ZAxisReversed;  
};

// default setings. Note: these are only programmed in eprom at first use. After that these are always loaded from eprom unless you change the checksum value (see line 32 at top of page).
// this is a random numeber beween 0 and 65535. use long button press at power-on to change behavour.

settingsStruct settings = {
  epromChecksum,  // checksum;
  false,          // reverseXAxis;
  false,          // reverseYAxis;
  false,          // reverseZAxis;
  sToggleSwitch,  // acc1Type; sToggleSwitch or sMomentarySwitch
  sToggleSwitch,  // acc2Type; sToggleSwitch or sMomentarySwitch
  sOff,           // dustCollectorDefault;
  sOff,           // routerDefault;
  2,              // speedDefault;
  false,          // XAxisReversed;
  false,          // YAxisReversed;
  false           // ZAxisReversed;
};

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   8, 0,                  // Button Count, Hat Switch Count
                   true, true, false,     // X and Y, but no Z Axis
                   false, true, false,    // No Rx, Ry, or Rz
                   false, false,          // No rudder or throttle
                   false, false, false);  // No accelerator, brake, or steering


const byte serialDataPin = A3;  // DS
const byte clockPin = A5;       // SHCP
const byte latchPin = A4;       // STCP

ShiftRegister74HC595<2> led(serialDataPin, clockPin, latchPin);

unsigned long previousMillis = 0;  
unsigned long lastRouterTrigger = 0;  
unsigned long lastDustCollectorTrigger = 0;


int joystickXZeropoint = 512;
int joystickYZeropoint = 512;
int joystickZZeropoint = 512;
long joysticCalibrationXIn[] = { 0, 512 - xYAxis0DeadMargin, 512, 512 + xYAxis0DeadMargin, 1023 };
long joysticCalibrationYIn[] = { 0, 512 - xYAxis0DeadMargin, 512, 512 + xYAxis0DeadMargin, 1023 };
long joysticCalibrationZIn[] = { 0, 512 - xYAxis0DeadMargin, 512, 512 + xYAxis0DeadMargin, 1023 };
long joysticCalibrationXOut[] = { -127, 0, 0, 0, 127 };
long joysticCalibrationYOut[] = { -127, 0, 0, 0, 127 };
long joysticCalibrationZOut[] = { -127, 0, 0, 0, 127 };

//                                   X  Y  Z
int joystickXPosition[3] =         { 0, 0, 0 };
int previousJoystickXPosition[3] = { 0, 0, 0 };

byte previousButton = 0;
bool enableXAxis = true;
bool enableYAxis = true;
bool enableRouter = false;
bool acc1Enabled = false;
bool acc2Enabled = false;
bool enableDustCollector = false;
byte speedSetting = 0;
byte routerSetting = 0;
byte dustCollectorSetting = 0;

byte buttons[2][5] = {  // The symbols of the keys
  { 1, 2, 3, 4, 5 },
  { 6, 7, 8, 9, 10 }
};


//Decode button matrix
//Outputs every number of button when pressed and same number +100 when released
byte lastButton() {
  byte buttonRead = 100;
  for (int row = 7; row <= 8; row++) {
    for (int col = 9; col <= 13; col++) {
      pinMode(col, OUTPUT);
      digitalWrite(col, HIGH);
      if (digitalRead(row) == 1) {
        buttonRead = buttons[row - 7][col - 9];
      }
      pinMode(col, INPUT);
    }
  }
  if (previousButton != buttonRead) {
    byte returnButton = buttonRead;
    if (buttonRead == 100) {  // add 100 if its a release of button
      returnButton = 100 + previousButton;
    }
    previousButton = buttonRead;
    delay(buttonDebounceTime);  //debounce button;
    return returnButton;
  } else {
    return 0;
  }
}


void setRouterLed() {
  led.set(ledRouterOn,         (routerSetting == 0));
  led.set(ledRouterOff,        (routerSetting == 1));
  led.set(ledRouterAuto,       (routerSetting == 2));  
}

void setDustCollectorLed() {
  led.set(ledDustCollectorOn,   (dustCollectorSetting == 0));
  led.set(ledDustCollectorOff,  (dustCollectorSetting == 1));
  led.set(ledDustCollectorAuto, (dustCollectorSetting == 2)); 
}

void flashall(int times){
  for (int i = 0; i <= times; i++) {
    led.setAllHigh(); // Turn on all LEDS
    delay(200);
    led.setAllLow();
    delay(200);
  }
}


void toggleAxis(byte axis){
  if (axis == xAxis) {
    enableXAxis = !enableXAxis;
    if (!enableYAxis){
      enableYAxis=true;
      Joystick.releaseButton(jBtnMapLockY); //Buttons need to be released first, otherwise the cnc won't act on the other axis buttonpress.
    }
  } else
  {
    enableYAxis = !enableYAxis;
    if (!enableXAxis){
      enableXAxis=true;
      Joystick.releaseButton(jBtnMapLockX);
    }
  }
  led.set(ledXAxis, enableXAxis);
  led.set(ledYAxis, enableYAxis);
  
  Joystick.setButton(jBtnMapLockX, !enableXAxis);
  Joystick.setButton(jBtnMapLockY, !enableYAxis);
}

void setSpeed (byte speed){  
  speedSetting = speed;
  led.set(ledSpeed0, (speed==0));
  led.set(ledSpeed1, (speed==1));
  led.set(ledSpeed2, (speed==2));
  led.set(ledSpeed3, (speed==3));  
  Joystick.setButton(jBtnMapSpeed0, (speed==0));
  Joystick.setButton(jBtnMapSpeed1, (speed==1));
  Joystick.setButton(jBtnMapSpeed2, (speed==2));
  Joystick.setButton(jBtnMapSpeed3, (speed==3));
  delay (100);
  Joystick.releaseButton(jBtnMapSpeed0);
  Joystick.releaseButton(jBtnMapSpeed1);
  Joystick.releaseButton(jBtnMapSpeed2);
  Joystick.releaseButton(jBtnMapSpeed3);
}

void setup() {
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  if (debugMode) {Serial.begin(9600);};

  settingsStruct epromSettings;
  //load settings
  EEPROM.get(0, epromSettings);
  if (epromSettings.checksum == epromChecksum) {
    settings = epromSettings;
  }



  //configuration startup configuration change moment. If a button is pushed down for 3 seconds (some buttons 10) during startup, the configuration of that button changes.
  byte keypress = lastButton();
  if (keypress != 0) {
    int timeCounter=0;
    switch (keypress) {
      case 1:  //Button XAxis default enabled setting       
        do {
          timeCounter++;
          led.set(ledXAxis, true);
          delay(50);
          led.set(ledXAxis, false);
          delay(50);
        }
        while ((lastButton()!=101)&&(timeCounter<100));
        
        if (timeCounter>=100){ //Button longer than 10 second hold adjust and save new settings
          settings.ZAxisReversed=!settings.ZAxisReversed;
          EEPROM.put(0, settings);         
          flashall(6);
        } else{

          if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings
            settings.XAxisReversed=!settings.XAxisReversed;
            EEPROM.put(0, settings);         
            flashall(6);
          }
        }
      break;

      case 2:  //YAxis XAxis default enabled setting
        do {
          timeCounter++;
          led.set(ledYAxis, true);
          delay(50);
          led.set(ledYAxis, false);
          delay(50);
        }
        while ((lastButton()!=102)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings
          settings.YAxisReversed=!settings.YAxisReversed;;
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;

      case 3:  //default Speed1
        do {
          timeCounter++;
          led.set(ledSpeed0, true);
          delay(50);
          led.set(ledSpeed0, false);
          delay(50);
        }
        while ((lastButton()!=103)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings
          settings.speedDefault=0;
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;

      case 4:  //default Speed1
        do {
          timeCounter++;
          led.set(ledSpeed1, true);
          delay(50);
          led.set(ledSpeed1, false);
          delay(50);
        }
        while ((lastButton()!=104)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings
          settings.speedDefault=1;
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;

      case 5:  //default Speed1
        do {
          timeCounter++;
          led.set(ledSpeed2, true);
          delay(50);
          led.set(ledSpeed2, false);
          delay(50);
        }
        while ((lastButton()!=105)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings
          settings.speedDefault=2;
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;

      case 6:  //default Speed1
        do {
          timeCounter++;
          led.set(ledSpeed3, true);
          delay(50);
          led.set(ledSpeed3, false);
          delay(50);
        }
        while ((lastButton()!=106)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings
          settings.speedDefault=3;
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;

      case 7:  //default Router Mode
        do {
          timeCounter++;
          led.set(ledRouter, true);
          delay(50);
          led.set(ledRouter, false);
          delay(50);
        }
        while ((lastButton()!=107)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings
          settings.routerDefault++;
          if (settings.routerDefault > 2) {
          settings.routerDefault = 0;
          }
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;

      case 8:  //default Dust collector mode
        do {
          timeCounter++;
          led.set(ledDustCollector, true);
          delay(50);
          led.set(ledDustCollector, false);
          delay(50);
        }
        while ((lastButton()!=108)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings
          settings.dustCollectorDefault++;
          if (settings.dustCollectorDefault > 2) {
          settings.dustCollectorDefault = 0;
          }
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;

      case 9:  //default Acc1 button mode
        do {
          timeCounter++;
          led.set(ledAcc1, true);
          delay(50);
          led.set(ledAcc1, false);
          delay(50);
        }
        while ((lastButton()!=109)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings

          if (settings.acc1Type==sToggleSwitch){
            settings.acc1Type=sMomentarySwitch;

          }else{
            settings.acc1Type=sToggleSwitch;
          }
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;
      
      case 10:  //default Acc2 button mode
        do {
          timeCounter++;
          led.set(ledAcc2, true);
          delay(50);
          led.set(ledAcc2, false);
          delay(50);
        }
        while ((lastButton()!=110)&&(timeCounter<30));
        
        if (timeCounter>=30){ //Button longer than 3 second hold adjust and save new settings

          if (settings.acc2Type==sToggleSwitch){
            settings.acc2Type=sMomentarySwitch;

          }else{
            settings.acc2Type=sToggleSwitch;
          }
          EEPROM.put(0, settings);         
          flashall(6);
        }
      break;      


    }
  }



  //Parse configuration:

  if (settings.reverseXAxis) {
    Joystick.setXAxisRange(127, -127);
  }else
  { 
    Joystick.setXAxisRange(-127, 127);
  }
  
  if (settings.reverseYAxis) {
    Joystick.setYAxisRange(127, -127);
  }else
  { 
    Joystick.setYAxisRange(-127, 127);
  }

  if (settings.reverseZAxis) {
    Joystick.setRyAxisRange(127, -127);
  }else
  { 
    Joystick.setRyAxisRange(-127, 127);
  }

  speedSetting         = settings.speedDefault;
  routerSetting       = settings.routerDefault;
  dustCollectorSetting = settings.dustCollectorDefault;

  pinMode(relayRouter, OUTPUT);
  pinMode(relayDustCollector, OUTPUT);
  pinMode(relayAcc1, OUTPUT);
  pinMode(relayAcc2, OUTPUT);

  led.setAllHigh(); // Turn on all LEDS
  delay(1000);
  led.setAllLow();  // Turn off all LEDS
  setRouterLed();
  setDustCollectorLed();
  led.set(ledXAxis, enableXAxis);
  led.set(ledYAxis, enableYAxis);
  led.set(ledAcc1, acc1Enabled);
  led.set(ledAcc2, acc2Enabled);

  //Joystick strartup 0 point calibbration:

  joystickXZeropoint = analogRead(xAxisControl);
  joystickYZeropoint = analogRead(yAxisControl);
  joystickZZeropoint = analogRead(zAxisControl);

  joysticCalibrationXIn[2] = joystickXZeropoint;
  joysticCalibrationYIn[2] = joystickYZeropoint;
  joysticCalibrationZIn[2] = joystickZZeropoint;

  joysticCalibrationXIn[1] = joystickXZeropoint - xYAxis0DeadMargin;
  joysticCalibrationYIn[1] = joystickYZeropoint - xYAxis0DeadMargin;
  joysticCalibrationZIn[1] = joystickZZeropoint - zAxis0DeadMargin;

  joysticCalibrationXIn[3] = joystickXZeropoint + xYAxis0DeadMargin;
  joysticCalibrationYIn[3] = joystickYZeropoint + xYAxis0DeadMargin;
  joysticCalibrationZIn[3] = joystickZZeropoint + zAxis0DeadMargin;

  //reverse joystick output if configured:
  if (settings.XAxisReversed) {
      joysticCalibrationXOut[0] = 127;
      joysticCalibrationXOut[4] = -127;
    }
  if (settings.YAxisReversed) {
      joysticCalibrationYOut[0] = 127;
      joysticCalibrationYOut[4] = -127;
    }
  if (settings.ZAxisReversed) {
      joysticCalibrationZOut[0] = 127;
      joysticCalibrationZOut[4] = -127;
    }

  Joystick.begin();

}

// Start runtime:
void loop() {

  
  //rebroadcast every remote position every second:
  if (millis() - 1000 > previousMillis) {   
    previousMillis = millis(); 
    setSpeed (speedSetting);
    Joystick.setButton(jBtnMapLockX, !enableXAxis);
    Joystick.setButton(jBtnMapLockY, !enableYAxis);
    led.set(ledXAxis, enableXAxis);
    led.set(ledYAxis, enableYAxis);
  }
 

  int xAnalogStickValue = analogRead(xAxisControl);
  int yAnalogStickValue = analogRead(yAxisControl);
  int zAnalogStickValue = analogRead(zAxisControl);
  joystickXPosition[0] = multiMap<long>(xAnalogStickValue, joysticCalibrationXIn, joysticCalibrationXOut, 5);
  joystickXPosition[1] = multiMap<long>(yAnalogStickValue, joysticCalibrationYIn, joysticCalibrationYOut, 5);
  joystickXPosition[2] = multiMap<long>(zAnalogStickValue, joysticCalibrationZIn, joysticCalibrationZOut, 5);

  if (memcmp(joystickXPosition, previousJoystickXPosition, sizeof(joystickXPosition)) != 0) {
    memcpy(previousJoystickXPosition, joystickXPosition, sizeof(joystickXPosition));

    Joystick.setXAxis(joystickXPosition[0]);
    Joystick.setYAxis(joystickXPosition[1]);
    Joystick.setRyAxis(joystickXPosition[2]);
  }


  // Handle all buttonpresses:
  byte keypress = lastButton();
  if (keypress != 0) {
    switch (keypress) {
      case 1:  //Button XAxis enabled
        toggleAxis(xAxis);
        break;
      case 2:  //Button YAxis enabled
        toggleAxis(yAxis);
        break;
      case 3:  //Button Speed0
        setSpeed (0);
        break;
      case 4:  //Button Speed1
        setSpeed (1);
        break;
      case 5:  //Button Speed2
        setSpeed (2);
        break;
      case 6:  //Button Speed3
        setSpeed (3);
        break;
      case 7:  //Button Router
        routerSetting++;
        if (routerSetting > 2) {
          routerSetting = 0;
        }             
        break;
      case 8:  //Button Dust Collection
        dustCollectorSetting++;
        if (dustCollectorSetting > 2) {
          dustCollectorSetting = 0;
        }
        break;
      case 9:  //Button Acc1
        if (settings.acc1Type == sToggleSwitch) {
          acc1Enabled = !acc1Enabled;
        } else {
          acc1Enabled = true;
        }
        led.set(ledAcc1, acc1Enabled);
        digitalWrite(relayAcc1, acc1Enabled);
        break;
      case 109: //Button Acc1 release

        if (settings.acc1Type == sMomentarySwitch) {
          acc1Enabled = false;
          led.set(ledAcc1, acc1Enabled);
          digitalWrite(relayAcc1, acc1Enabled);
        }
        break;
      case 10:  //Button Acc2
        if (settings.acc2Type == sToggleSwitch) {
          acc2Enabled = !acc2Enabled;
        } else {
          acc2Enabled = true;
        }

        led.set(ledAcc2, acc2Enabled);
        digitalWrite(relayAcc2, acc2Enabled);
        break;
      case 110: //Button Acc2 release
        if (settings.acc2Type == sMomentarySwitch) {
          acc2Enabled = false;
          led.set(ledAcc2, acc2Enabled);
          digitalWrite(relayAcc2, acc2Enabled);
        }
        break;
    }
    //Debug keypresses:
     if (debugMode) {
        if (keypress <= 100){ 
          Serial.println("Button" + String(keypress) + " pressed");
        } else
        {
          Serial.println("Button" + String(keypress-100) + " released");
        }
     }
    
  }

  //Switch on/off the router:
  //It checks if there's a signal the past 15ms (to ignore PWM triggering)
  bool routerActivate = false;
  if (!digitalRead(cncInputIO)){ //(active low)
    lastRouterTrigger=millis();
  }  
  
  if (millis() -15 < lastRouterTrigger) {
    routerActivate = true;
  }


  //Switch on the dust collector with a delay of "dustCollectorDelay" to prevent a big power surge (only if Router is in auto mode) :
  bool dustCollectorActivate = false; 
 
  if (!routerActivate){
    lastDustCollectorTrigger=millis();
  } else
  { 
    if (routerSetting!=sAuto) {
      dustCollectorActivate = true;
    } else
    {
      if (millis() -dustCollectorDelay > lastDustCollectorTrigger) {
        dustCollectorActivate = true;
      }
    }
  }
  
  switch (routerSetting) {
      case sOn:  //Button XAxis enabled
        digitalWrite(relayRouter,HIGH);
        led.set(ledRouter, true);
        break;
      case sOff:  //Button XAxis enabled
        digitalWrite(relayRouter,LOW);
        led.set(ledRouter, false);
        break;
      case sAuto:  //Button XAxis enabled
        digitalWrite(relayRouter,routerActivate);
        led.set(ledRouter, routerActivate);
        break;
  }
  setRouterLed();
  
  switch (dustCollectorSetting) {
      case sOn:  //Button XAxis enabled
        digitalWrite(relayDustCollector,HIGH);
        led.set(ledDustCollector, true);
        break;
      case sOff:  //Button XAxis enabled
        digitalWrite(relayDustCollector,LOW);
        led.set(ledDustCollector, false);
        break;
      case sAuto:  //Button XAxis enabled
        digitalWrite(relayDustCollector,dustCollectorActivate);
        led.set(ledDustCollector, dustCollectorActivate);
        break;
  }
  setDustCollectorLed();
}
