/*
 * PS2 to PSP Arduino Firmware
 * 
 * IDEAS
 * Reconsider caching button presses each loop
 */

#include <Wire.h>
#include <PS2X_lib.h>

#define JUMPER_PIN 25

// PS2 Controller //////////////////
#define PS2_DAT 23
#define PS2_CMD 4
#define PS2_SEL 3
#define PS2_CLK 2

PS2X ps2x;
byte ps2xError = 1; // starts as 1 to trigger the initial connection attempt


// PSP Button Pins //////////////////
#define PORT_B 0
#define PORT_C 1
#define PORT_D 2
#define PORT_E 3

typedef struct {
  const byte port;
  const byte num;
  bool pressed;
} pinDef;

pinDef upPin = { PORT_B, 3 };
pinDef downPin = { PORT_B, 1 };
pinDef leftPin = { PORT_B, 2 };
pinDef rightPin = { PORT_B, 4 };
pinDef leftShoulderPin = { PORT_B, 5 };

pinDef crossPin = { PORT_B, 7 };
pinDef squarePin = { PORT_B, 0 };
pinDef circlePin = { PORT_D, 5 };
pinDef trianglePin = { PORT_D, 6};
pinDef rightShoulderPin = { PORT_D, 7 };
pinDef powerPin = { PORT_B, 6 };

pinDef homePin = { PORT_E, 3 };
// pinDef volumeDownPin = { PORT_D, 0 }; // serial
// pinDef volumeUpPin = { PORT_D, 1 }; // serial
pinDef videoPin = { PORT_C, 0 };
pinDef eqPin = { PORT_C, 1 };
pinDef selectPin = { PORT_C, 2 };
pinDef startPin = { PORT_C, 3 };


// Button Mappings ///////////////////
#define SECONDARY_MAPPING_COMBO_HOLD_MILLIS 250

typedef struct {
  pinDef* pin;
  unsigned int buttonA;
  unsigned int buttonB;
} pinButtonMapping;

typedef struct {
  unsigned int buttonA;
  unsigned int buttonB;
  unsigned long pressedMillis;
} buttonCombo;

pinButtonMapping primaryMappings[] = {
  { &upPin, PSB_PAD_UP },
  { &downPin, PSB_PAD_DOWN },
  { &leftPin, PSB_PAD_LEFT },
  { &rightPin, PSB_PAD_RIGHT },
  { &leftShoulderPin, PSB_L1, PSB_L2 },

  { &crossPin, PSB_CROSS },
  { &squarePin, PSB_SQUARE },
  { &circlePin, PSB_CIRCLE },
  { &trianglePin, PSB_TRIANGLE },
  { &rightShoulderPin, PSB_R1, PSB_R2 },

  { &startPin, PSB_START },
  { &selectPin, PSB_SELECT }
};
const byte primaryMappingsSize = sizeof(primaryMappings) / sizeof(pinButtonMapping);

pinButtonMapping secondaryMappings[] = {
  { &homePin, PSB_START },
  { &powerPin, PSB_SELECT },
  { &videoPin, PSB_SQUARE },
  { &eqPin, PSB_CIRCLE },
  //  { &volumeUpPin, PSB_PAD_UP },
  //  { &volumeDownPin, PSB_PAD_DOWN },
};
const byte secondaryMappingsSize = sizeof(secondaryMappings) / sizeof(pinButtonMapping);

buttonCombo secondaryMappingCombo = { PSB_L2, PSB_R2 };
pinButtonMapping *currentMapping = primaryMappings;
byte currentMappingSize = primaryMappingsSize;


// Analog Sticks ////////////////////
#define LEFT_STICK_DEADZONE 30
#define LEFT_ANALOG_MODE_COUNT 2
#define LEFT_ANALOG_MODE_STICK 0
#define LEFT_ANALOG_MODE_PSX 1

#define THRESHOLD_STICK_DEADZONE 50
#define RIGHT_ANALOG_MODE_COUNT 2
#define RIGHT_ANALOG_MODE_DISABLED 0
#define RIGHT_ANALOG_MODE_THRESHOLD 1

byte leftAnalogMode = 0;
byte rightAnalogMode = 0;
byte thresholdMagnitude;
double thresholdAngle;


// LED /////////////////////////////
#define LED_PIN 24

byte ledTotalPulses = 0;
byte ledCurrentPulse = 0;
byte ledPulseLengthMillis = 50;
unsigned long ledLastMillis = 0;


// Setup & Loop //////////////////////////

void setup() {
  Serial.begin(57600);

  Wire.begin();
  centerPot();
  pinMode(JUMPER_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(500); // wait for psp to accept power signal
  if (digitalRead(JUMPER_PIN)) { // if jumper open
    powerUp();  
    delay(14000); // wait for date/time screen
    digitalWrite(LED_PIN, LOW);
    startVideoOut();
  }
  
  while (ps2xError) { // wait for controller and connect
    ps2xError = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  }
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  delay(2);
  ps2x.read_gamepad();
  updateLedFlash();

  updateCurrentMapping();
  resetPinPressed();

  updateButtons(currentMapping, currentMappingSize);
  updateLeftAnalog();
  updateRightAnalog();

  releaseUnusedPins();
}

// Functions ////////////////////////////

void updateButtons(pinButtonMapping mapping[], byte mappingSize){
  for (byte i = 0; i < mappingSize; i++){
    if (ps2x.Button(mapping[i].buttonA) || (!psxModeActive() && mapping[i].buttonB && ps2x.Button(mapping[i].buttonB))) {
      pressPin(mapping[i].pin);
    }

    if (psxModeActive()) { handleL2R2PSX(); }
  }
}

void handleL2R2PSX() {
  if (ps2x.Button(PSB_L2) && ps2x.Button(PSB_R2)) {
    setPot(127, 64); // up
  } else if (ps2x.Button(PSB_L2)) {
    setPot(64, 127); // left
  } else if (ps2x.Button(PSB_R2)) {
    setPot(191, 127); // right
  } else {
    centerPot();
  }
}

void updateCurrentMapping() {
  if (ps2x.Button(secondaryMappingCombo.buttonA) && ps2x.Button(secondaryMappingCombo.buttonB)){
    if (!secondaryMappingCombo.pressedMillis) { secondaryMappingCombo.pressedMillis = millis(); }
  } else {
    secondaryMappingCombo.pressedMillis = 0;
  }

  if (secondaryMappingCombo.pressedMillis && (millis() - secondaryMappingCombo.pressedMillis > SECONDARY_MAPPING_COMBO_HOLD_MILLIS)){
    currentMapping = secondaryMappings;
    currentMappingSize = secondaryMappingsSize;
  } else {
    currentMapping = primaryMappings;
    currentMappingSize = primaryMappingsSize;
  }
}

void pressPin(pinDef* pin){
  pin->pressed = true;

  switch(pin->port){
    case PORT_B:
      DDRB = DDRB | (1 << pin->num);
      PORTB = PORTB & ~(1 << pin->num);
      break;
    case PORT_C:
      DDRC = DDRC | (1 << pin->num);
      PORTC = PORTC & ~(1 << pin->num);
      break;
    case PORT_D:
      DDRD = DDRD | (1 << pin->num);
      PORTD = PORTD & ~(1 << pin->num);
      break;
    case PORT_E:
      DDRE = DDRE | (1 << pin->num);
      PORTE = PORTE & ~(1 << pin->num);
      break;
  }
}

void releasePin(pinDef* pin){
  switch(pin->port){
    case PORT_B:
      DDRB = DDRB & ~(1 << pin->num);
      break;
    case PORT_C:
      DDRC = DDRC & ~(1 << pin->num);
      break;
    case PORT_D:
      DDRD = DDRD & ~(1 << pin->num);
      break;
    case PORT_E:
      DDRE = DDRE & ~(1 << pin->num);
      break;
  }
}

void resetPinPressed() {
  for(byte i = 0; i < currentMappingSize; i++){
    currentMapping[i].pin->pressed = false;
  }
}

void releaseUnusedPins(){
  for(byte i = 0; i < currentMappingSize; i++){
    if (!(currentMapping[i].pin->pressed)){
      releasePin(currentMapping[i].pin);
    }
  }
}

int magnitude(int x, int y) {
  return sqrt((pow(x, 2)+pow(y, 2)));
}

byte currentTick = 0;
byte tickLength = 16; // ms ~60fps
unsigned long tickTime = 0;
void updateLeftAnalog(){
  if (ps2x.ButtonReleased(PSB_L3)){
    leftAnalogMode = (leftAnalogMode + 1) % LEFT_ANALOG_MODE_COUNT;
    centerPot();
    startLedFlash(2, 100);
  }

  switch(leftAnalogMode){
    case LEFT_ANALOG_MODE_STICK:
      if (magnitude(ps2x.Analog(PSS_LX) - 127, ps2x.Analog(PSS_LY) - 127) > LEFT_STICK_DEADZONE){
        setPot(127 + ((ps2x.Analog(PSS_LX) - 127) / 1.5), 127 + ((ps2x.Analog(PSS_LY) - 127) / 1.5));
      } else {
        centerPot();
      }
      break;
    case LEFT_ANALOG_MODE_PSX:
      thresholdStick(PSS_LX, PSS_LY, &downPin, &upPin, &leftPin, &rightPin);
      break;
  }
}

void updateRightAnalog(){
  if (ps2x.ButtonReleased(PSB_R3)){
    rightAnalogMode = (rightAnalogMode + 1) % RIGHT_ANALOG_MODE_COUNT;
    startLedFlash(2, 100);
  }
  
  switch(rightAnalogMode){
    case RIGHT_ANALOG_MODE_THRESHOLD:
      thresholdStick(PSS_RX, PSS_RY, &crossPin, &trianglePin, &squarePin, &circlePin);
      break;
  }
}

void thresholdStick(unsigned int xAxis, unsigned int yAxis, pinDef* up, pinDef* down, pinDef* left, pinDef* right){
  thresholdMagnitude = magnitude(ps2x.Analog(xAxis) - 127, ps2x.Analog(yAxis) - 127);
  if (thresholdMagnitude > THRESHOLD_STICK_DEADZONE){
    thresholdAngle = atan2(ps2x.Analog(yAxis) - 127, ps2x.Analog(xAxis) - 127);
    if (thresholdAngle < 0) { thresholdAngle += 6.2832; }

    if (thresholdAngle >= 0.3972 && thresholdAngle <= 1.1781){ pressPin(up); pressPin(right); } // up right
    else if (thresholdAngle >= 1.1781 && thresholdAngle <= 1.9635){ pressPin(up); } // up
    else if (thresholdAngle >= 1.9635 && thresholdAngle <= 2.7489){ pressPin(up); pressPin(left); } // up left
    else if (thresholdAngle >= 2.7489 && thresholdAngle <= 3.5343){ pressPin(left); } // left
    else if (thresholdAngle >= 3.5343 && thresholdAngle <= 4.3197){ pressPin(down); pressPin(left); } // down left
    else if (thresholdAngle >= 4.3197 && thresholdAngle <= 5.1051){ pressPin(down); } // down
    else if (thresholdAngle >= 5.1051 && thresholdAngle <= 5.8905){ pressPin(down); pressPin(right); } // down right
    else if (thresholdAngle >= 5.8905 || thresholdAngle <= 0.3972){ pressPin(right); } // right
  }
}

void setPot(byte x, byte y){
  Wire.beginTransmission(0);
  Wire.write(B10000000);
  Wire.write(x);
  Wire.write(0);
  Wire.endTransmission();

  Wire.beginTransmission(0);
  Wire.write(B10010000);
  Wire.write(y);
  Wire.write(0);
  Wire.endTransmission();
}

void centerPot(){
  setPot(127, 127);
}

bool psxModeActive(){
  return (leftAnalogMode == LEFT_ANALOG_MODE_PSX);
}

void powerUp(){
  pressPin(&powerPin);
  delay(500);
  releasePin(&powerPin);
}

void startVideoOut(){
  pressPin(&videoPin);
  delay(5000);
  releasePin(&videoPin);
}

void startLedFlash(byte pulses, byte pulseLength) {
  ledCurrentPulse = 0;
  ledTotalPulses = pulses * 2;
  ledPulseLengthMillis = pulseLength;
}

void updateLedFlash() {
  if (ledCurrentPulse >= ledTotalPulses) { return; }

  if (millis() - ledLastMillis >= ledPulseLengthMillis){
    ledLastMillis = millis();
    ledCurrentPulse++;
    if (ledCurrentPulse == ledTotalPulses) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}
