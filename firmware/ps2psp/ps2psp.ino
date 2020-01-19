/*
 * PS2 to PSP Arduino Firmware
 * 
 * TODO
 * Add switchable modes to right stick (off, threshold, smoothed)
 * Try scaling left analog stick
 * Reconsider key combos & mapping switching
 * Reconsider caching button presses each loop
 */

#include <Wire.h>
#include <PS2X_lib.h>


// PS2 Controller //////////////////
#define PS2_DAT A2
#define PS2_CMD A1
#define PS2_SEL A0
#define PS2_CLK A3

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
} pin;

pin upPin = { PORT_D, 1 };
pin downPin = { PORT_D, 2 };
pin leftPin = { PORT_D, 3 };
pin rightPin = { PORT_D, 4 };
pin leftShoulderPin = { PORT_D, 0 };

pin crossPin = { PORT_B, 1 };
pin squarePin = { PORT_B, 5 };
pin circlePin = { PORT_B, 2 };
pin trianglePin = { PORT_B, 3};
pin rightShoulderPin = { PORT_B, 4 };
pin powerPin = { PORT_B, 0 };

pin homePin = { PORT_D, 7 };
pin videoPin = { PORT_D, 6 };
pin startPin = { PORT_B, 7 };
pin selectPin = { PORT_D, 5 };

void pressPin(pin* pin){
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
//      DDRE = DDRE | (1 << pin->num);
//      PORTE = PORTE & ~(1 << pin->num);
      break;
  }
}

void releasePin(pin* pin){
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
//      DDRE = DDRE & ~(1 << pin->num);
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


// Button Mappings ///////////////////
#define SECONDARY_MAPPING_COMBO_HOLD_MILLIS 250

pinButtonMapping *currentMapping = primaryMappings;
byte currentMappingSize = primaryMappingsSize;

typedef struct {
  pin* pin;
  unsigned int buttonA;
  unsigned int buttonB;
} pinButtonMapping;

typedef struct {
  unsigned int buttonA;
  unsigned int buttonB;
  unsigned long pressedMillis;
} buttonCombo;

const pinButtonMapping primaryMappings[] = {
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

const pinButtonMapping secondaryMappings[] = {
  { &homePin, PSB_START },
  { &powerPin, PSB_SELECT },
  { &videoPin, PSB_TRIANGLE }
};
const byte secondaryMappingsSize = sizeof(secondaryMappings) / sizeof(pinButtonMapping);

buttonCombo secondaryMappingCombo = { PSB_L2, PSB_R2 };

void updateButtons(pinButtonMapping mapping[], byte mappingSize){
  for (byte i = 0; i < mappingSize; i++){
    if (ps2x.Button(mapping[i].buttonA) || (mapping[i].buttonB && ps2x.Button(mapping[i].buttonB))) {
      pressPin(mapping[i].pin);
    }
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


// Analog Sticks ////////////////////
#define STICK_DEADZONE 45

double rightAnalogAngle;
byte rightAnalogMode = 0;

int magnitude(int x, int y) {
  return sqrt((pow(x, 2)+pow(y, 2)));
}

void updateLeftAnalog(){
  if (magnitude(ps2x.Analog(PSS_LX) - 127, ps2x.Analog(PSS_LY) - 127) > STICK_DEADZONE){
    setPot(ps2x.Analog(PSS_LX), ps2x.Analog(PSS_LY));
  } else {
    centerPot();
  }
}

void updateRightAnalog(){
  if (magnitude(ps2x.Analog(PSS_RX) - 127, ps2x.Analog(PSS_RY) - 127) > STICK_DEADZONE){
    rightAnalogAngle = atan2(ps2x.Analog(PSS_RY) - 127, ps2x.Analog(PSS_RX) - 127);
    if (rightAnalogAngle < 0) { rightAnalogAngle += 6.2832; }

    if (rightAnalogAngle >= 0.3972 && rightAnalogAngle <= 1.1781){ pressPin(&crossPin); pressPin(&circlePin); } // up right
    else if (rightAnalogAngle >= 1.1781 && rightAnalogAngle <= 1.9635){ pressPin(&crossPin); } // up
    else if (rightAnalogAngle >= 1.9635 && rightAnalogAngle <= 2.7489){ pressPin(&crossPin); pressPin(&squarePin); } // up left
    else if (rightAnalogAngle >= 2.7489 && rightAnalogAngle <= 3.5343){ pressPin(&squarePin); } // left
    else if (rightAnalogAngle >= 3.5343 && rightAnalogAngle <= 4.3197){ pressPin(&trianglePin); pressPin(&squarePin); } // down left
    else if (rightAnalogAngle >= 4.3197 && rightAnalogAngle <= 5.1051){ pressPin(&trianglePin); } // down
    else if (rightAnalogAngle >= 5.1051 && rightAnalogAngle <= 5.8905){ pressPin(&trianglePin); pressPin(&circlePin); } // down right
    else if (rightAnalogAngle >= 5.8905 || rightAnalogAngle <= 0.3972){ pressPin(&circlePin); } // right
  }
}

void setPot(byte x, byte y){
  Wire.beginTransmission(B0101000);
  Wire.write(B10101001);
  Wire.write(0xFF - y);
  Wire.write(0xFF - x);
  Wire.endTransmission();
}

void centerPot(){
  setPot(127, 127);
}


// Setup & Loop //////////////////////////

void setup() {
//  Serial.begin(57600);
  Wire.begin();
  centerPot();

  while (ps2xError) {
    delay(300);
    ps2xError = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  }
}

void loop() {
  delay(2);
  ps2x.read_gamepad();

  updateCurrentMapping();
  resetPinPressed();

  updateButtons(currentMapping, currentMappingSize);
  updateLeftAnalog();
  updateRightAnalog();

  releaseUnusedPins();
}
