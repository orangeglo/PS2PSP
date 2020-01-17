/*
 * PS2 to PSP Arduino Firmware
 * 
 * TODO
 * - Add hold mapping
 * - Add stick deadzones
 * - Revisit "pressed" optimaztion so I'm not writing button state every loop
 * - Add right stick modes
 * - save stick mode to eeprom
 */

#include <PS2X_lib.h>
#include <Wire.h>

#define PS2_DAT A2
#define PS2_CMD A1
#define PS2_SEL A0
#define PS2_CLK A3

#define PORT_B 0
#define PORT_C 1
#define PORT_D 2
#define PORT_E 3

#define SECONDARY_MAPPING_COMBO_HOLD_MILLIS 300

typedef struct {
  byte port;
  byte num;
} pin;

typedef struct {
  pin pin;
  unsigned int buttonA;
  unsigned int buttonB;
} pinButtonMapping;

typedef struct {
  unsigned int buttonA;
  unsigned int buttonB;
  unsigned long pressedMillis;
} buttonCombo;

const pin upPin = { PORT_D, 1 };
const pin downPin = { PORT_D, 2 };
const pin leftPin = { PORT_D, 3 };
const pin rightPin = { PORT_D, 4 };
const pin leftShoulderPin = { PORT_D, 0 };

const pin crossPin = { PORT_B, 1 };
const pin squarePin = { PORT_B, 5 };
const pin circlePin = { PORT_B, 2 };
const pin trianglePin = { PORT_B, 3};
const pin rightShoulderPin = { PORT_B, 4 };
const pin powerPin = { PORT_B, 0 };

const pin homePin = { PORT_D, 7 };
const pin videoPin = { PORT_D, 6 };
const pin startPin = { PORT_B, 7 };
const pin selectPin = { PORT_D, 5 };

const pinButtonMapping primaryMappings[] = {
  { upPin, PSB_PAD_UP },
  { downPin, PSB_PAD_DOWN },
  { leftPin, PSB_PAD_LEFT },
  { rightPin, PSB_PAD_RIGHT },
  { leftShoulderPin, PSB_L1, PSB_L2 },

  { crossPin, PSB_CROSS },
  { squarePin, PSB_SQUARE },
  { circlePin, PSB_CIRCLE },
  { trianglePin, PSB_TRIANGLE },
  { rightShoulderPin, PSB_R1, PSB_R2 },

  { startPin, PSB_START },
  { selectPin, PSB_SELECT }
};
const byte primaryMappingsSize = sizeof(primaryMappings) / sizeof(pinButtonMapping);

const pinButtonMapping secondaryMappings[] = {
  { homePin, PSB_START },
  { powerPin, PSB_SELECT },
  { videoPin, PSB_TRIANGLE }
};
const byte secondaryMappingsSize = sizeof(secondaryMappings) / sizeof(pinButtonMapping);

const buttonCombo secondaryMappingCombo = { PSB_L2, PSB_R2 };


PS2X ps2x;
byte ps2xError = 1; // starts as 1 to trigger the initial connection attempt
pinButtonMapping *currentMapping = &primaryMappings;
byte currentMappingSize = primaryMappingsSize;

byte potX = 0;
byte potY = 0;
byte lastPotX = 0;
byte lastPotY = 0;


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
  ps2x.read_gamepad();

  updateButtonMapping();

  updateButtons(currentMapping, currentMappingSize);
  updateLeftAnalog();

  delay(2);
}

void updateButtonMapping() {
  if (ps2x.Button(secondaryMappingCombo.buttonA) && ps2x.Button(secondaryMappingCombo.buttonB)){
    if (!secondaryMappingCombo.pressedMillis) { secondaryMappingCombo.pressedMillis = millis(); }
  } else {
    secondaryMappingCombo.pressedMillis = 0;
  }

  if (secondaryMappingCombo.pressedMillis && (millis() - secondaryMappingCombo.pressedMillis > SECONDARY_MAPPING_COMBO_HOLD_MILLIS)){
    currentMapping = &secondaryMappings;
    currentMappingSize = secondaryMappingsSize;
  } else {
    currentMapping = &primaryMappings;
    currentMappingSize = primaryMappingsSize;
  }
}

void updateButtons(pinButtonMapping mapping[], byte mappingSize){
  for (byte i = 0; i < mappingSize; i++){
    if (ps2x.Button(mapping[i].buttonA) || (mapping[i].buttonB && ps2x.Button(mapping[i].buttonB))) {
      pressPin(&(mapping[i].pin));
    } else {
      releasePin(&(mapping[i].pin));
    }
  }
}

void pressPin(pin* pin){
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

void updateLeftAnalog(){
  potX = ps2x.Analog(PSS_LX);
  potY = ps2x.Analog(PSS_LY);
  if ((potX != lastPotX) || (potY != lastPotY)) {
    setPot(potX, potY);
    lastPotX = potX;
    lastPotY = potY;
  }
}

void centerPot(){
  setPot(128, 128); 
}

void setPot(byte x, byte y){
  Wire.beginTransmission(B0101000);
  Wire.write(B10101001);
  Wire.write(0xFF - y);
  Wire.write(0xFF - x);
  Wire.endTransmission();
}
