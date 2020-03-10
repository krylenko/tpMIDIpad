// Arduino Uno MIDI touch pad / control surface

#include <avr/pgmspace.h>
#include "waves.h"

#define CC                  0xB0
#define MIDI_CHANNEL        1

#define LED_OUT             5
#define LFO_OUT             3
#define LFO_SPD_DIVISIONS   5
#define LFO_CC              116
double lfoMix = 0.0;
uint8_t lfoPhase = 0;
uint16_t lfoSpeed = 5;
double lfoSpdDivisions[LFO_SPD_DIVISIONS] = {40.0, 20.0, 12.0, 3.0, 1.0};
uint8_t lfoSpdIdx = 1;
double lfoDepth = 1.0;
uint8_t lfoDownsampleFactor = 10;   // ramp this up for noise waveforms
uint16_t lastLFOvalue = 1000;

// define switch and button (momentary) inputs
#define NUM_SWITCHES        3
enum {
  TOGGLE_LOWER = 4,
  TOGGLE_TOP_LEFT = 9,
  TOGGLE_TOP_RIGHT = 10
};
int switches[NUM_SWITCHES] = {
  TOGGLE_LOWER,
  TOGGLE_TOP_LEFT,
  TOGGLE_TOP_RIGHT
};
uint16_t switchVal[NUM_SWITCHES];
uint16_t switchOld[NUM_SWITCHES];

#define NUM_BUTTONS         5
#define CHANGE_VALID_CT     5
enum {
  PUSH_ROUND = 6,
  BUTTON_RIGHT_TOP = 7,
  BUTTON_RIGHT_BOTTOM = 8,
  BUTTON_LEFT_BOTTOM = 11,
  BUTTON_LEFT_TOP = 12
};
int buttons[NUM_BUTTONS] = {
  PUSH_ROUND,
  BUTTON_RIGHT_TOP,
  BUTTON_RIGHT_BOTTOM,
  BUTTON_LEFT_BOTTOM,
  BUTTON_LEFT_TOP
};
uint16_t highCt[NUM_BUTTONS];
uint16_t lowCt[NUM_BUTTONS];
bool sendHigh[NUM_BUTTONS];
bool sendLow[NUM_BUTTONS];

// define analog inputs
#define NUM_ANALOG_INPUTS   6
enum {
  PAD_X,
  PAD_Y,
  EXP_PEDAL,
  POT_TOP_RIGHT,
  POT_BROWN_BOTTOM,
  POT_BROWN_TOP
};

// reference
// MIDI message structure:  http://midi.org/techspecs/midimessages.php#3
// controller assignments: http://nickfever.com/music/midi-cc-list

int controllers[NUM_ANALOG_INPUTS + NUM_SWITCHES + NUM_BUTTONS] = {
  102,      // A0     pad X               CC 102
  103,      // A1     pad Y               CC 103
  104,      // A2     exp pedal           CC 104
  105,      // A3     top knob            CC 105
  106,      // A4     left knob top       CC 106
  107,      // A5     left knob bot       CC 107
  108,      // D4     lower toggle        CC 108
  109,      // D6     round push          CC 109
  110,      // D7     right gray but top  CC 110
  111,      // D8     right gray but bot  CC 111
  112,      // D9     toggle top left     CC 112
  113,      // D10    toggle top right    CC 113
  114,      // D11    left gray but bot   CC 114
  115       // D12    left gray but top   CC 115
};
uint16_t analogVal[NUM_ANALOG_INPUTS];
uint16_t analogOld[NUM_ANALOG_INPUTS];

/* The format of the message to send via serial. We create a new data type, that can store 4 values at once.  This will be easier to send as MIDI. */
typedef struct {
  uint8_t commChannel;  // first  4 bits : channel message (NOTE_ON, NOTE_OFF or CC (controlchange)
  // second 4 bits : midi channel (0-15)
  uint8_t data1;        // second byte   : first value (0-127), controller number or note number
  uint8_t data2;        // third  byte   : second value (0-127), controller value or velocity
} t_midiMsg;

uint8_t ctIdx = 0;
t_midiMsg msg;

uint16_t minPadInt = 30;
uint16_t maxPadInt_x = 585;
uint16_t maxPadInt_y = 690;

void setup()
{

  pinMode(LED_OUT, OUTPUT);
  digitalWrite(LED_OUT, 1);
  pinMode(LFO_OUT, OUTPUT);
  digitalWrite(LFO_OUT, 1);

  // initialize user wave array
  for (int i = 0; i < PHASE_SZ; ++i) {
    userWave[i] = pgm_read_byte_near(sawDown + i);
  }

  for (int i = 0; i < NUM_ANALOG_INPUTS; i++) {
    analogVal[i] = 0;
    analogOld[i] = -1;
  }
  for (int j = 0; j < NUM_SWITCHES; ++j) {
    pinMode(switches[j], INPUT);
    switchVal[j] = 0;
    switchOld[j] = -1;
  }
  for (int k = 0; k < NUM_BUTTONS; ++k) {
    pinMode(buttons[k], INPUT);
    highCt[k] = 0;
    lowCt[k] = 0;
    sendHigh[k] = false;
    sendLow[k] = false;
  }

  Serial.begin(31250);
  delay(2000);

}

void loop()
{

  static uint8_t lfoDownsampleCt = 0;

  /************** WRITE MIDI CONTROLLER DATA *********************/

  // read and write states of switches
  uint8_t enablePadX = 1, enablePadY = 1, enableLFO = 1;
  for (int m = 0; m < NUM_SWITCHES; ++m) {
    int tempVal = digitalRead(switches[m]);
    tempVal == HIGH ? tempVal = 127 : tempVal = 0;
    switchVal[m] = tempVal;
    if ( switchVal[m] != switchOld[m] ) {
      msg.data1 = controllers[m + NUM_ANALOG_INPUTS];
      msg.data2 = switchVal[m];
      Serial.write(msg.commChannel);
      Serial.write(msg.data1);
      Serial.write(msg.data2);
      switchOld[m] = switchVal[m];
    }
    if (switches[m] == TOGGLE_TOP_LEFT) {
      switchVal[m] == 127 ? enablePadY = 1 : enablePadY = 0;
    }
    if (switches[m] == TOGGLE_TOP_RIGHT) {
      switchVal[m] == 127 ? enableLFO = 1 : enableLFO = 0;
    }
    if (switches[m] == TOGGLE_LOWER) {
      switchVal[m] == 127 ? enablePadX = 1 : enablePadX = 0;
    }
  }

  // read and output analog MIDI controllers
  uint16_t currVal = 0;
  currVal = analogRead(ctIdx);

  msg.commChannel = CC;
  msg.commChannel += MIDI_CHANNEL - 1;

  switch (ctIdx)
  {
    case PAD_X:
      {
        delay(7);
        currVal = analogRead(ctIdx);
        if ( currVal >= minPadInt && currVal <= maxPadInt_x - 20 ) {
          analogVal[ctIdx] = map(currVal, minPadInt, maxPadInt_x - 20, 0, 127);
          analogVal[ctIdx] = constrain(analogVal[ctIdx], 0, 127);
        }
        break;
      }
    case PAD_Y:
      {
        delay(7);
        currVal = analogRead(ctIdx);
        if ((currVal >= minPadInt) && (currVal <= maxPadInt_y)) {
          analogVal[ctIdx] = map(currVal, maxPadInt_y, minPadInt, 0, 127);
          analogVal[ctIdx] = constrain(analogVal[ctIdx], 0, 127);
        }
        break;
      }
    case POT_BROWN_BOTTOM:
      {
        analogVal[ctIdx] = currVal >> 3;
        lfoDepth = double(currVal / 1023.0);
        break;
      }
    case POT_BROWN_TOP:
      {
        analogVal[ctIdx] = currVal >> 3;
        lfoSpeed = int(currVal / lfoSpdDivisions[lfoSpdIdx]) + 1;
        break;
      }
    case POT_TOP_RIGHT:
      {
        analogVal[ctIdx] = currVal >> 3;
        lfoMix = double(4.0 * currVal / 1023.0);
        break;
      }
    default:
      {
        analogVal[ctIdx] = currVal >> 3;
        break;
      }
  }

  if (analogVal[ctIdx] != analogOld[ctIdx]) {
    if ( (ctIdx == PAD_X && enablePadX) || (ctIdx == PAD_Y && enablePadY) || (ctIdx != PAD_X && ctIdx != PAD_Y)) {
      msg.data1 = controllers[ctIdx];
      msg.data2 = analogVal[ctIdx];
      Serial.write(msg.commChannel);
      Serial.write(msg.data1);
      Serial.write(msg.data2);
    }
    analogOld[ctIdx] = analogVal[ctIdx];
  }

  ++ctIdx;
  if ( ctIdx >= NUM_ANALOG_INPUTS ) {
    ctIdx = 0;
  }

  // read and write states of buttons
  for (int p = 0; p < NUM_BUTTONS; ++p) {
    if ( digitalRead(buttons[p]) ) {
      ++highCt[p];
    } else {
      if ( highCt > 0 ) {
        ++lowCt[p];
      }
    }
    if (highCt[p] == CHANGE_VALID_CT) {
      sendHigh[p] = true;
      if (buttons[p] == BUTTON_LEFT_TOP) {
        // raise LFO speed
        ++lfoSpdIdx;
        if (lfoSpdIdx >= LFO_SPD_DIVISIONS) {
          lfoSpdIdx = LFO_SPD_DIVISIONS - 1;
        }
      }
      else if (buttons[p] == BUTTON_LEFT_BOTTOM) {
        // lower LFO speed
        --lfoSpdIdx;
        if (lfoSpdIdx <= 0) {
          lfoSpdIdx = 0;
        }
      }
    }
    if (lowCt[p] == CHANGE_VALID_CT) {
      sendLow[p] = true;
    }

    if (sendHigh[p]) {
      msg.data1 = controllers[p + NUM_ANALOG_INPUTS + NUM_SWITCHES];
      msg.data2 = 127;
      Serial.write(msg.commChannel);
      Serial.write(msg.data1);
      Serial.write(msg.data2);
      sendHigh[p] = false;
      lowCt[p] = 0;
    }
    if (sendLow[p]) {
      msg.data1 = controllers[p + NUM_ANALOG_INPUTS + NUM_SWITCHES];
      msg.data2 = 0;
      Serial.write(msg.commChannel);
      Serial.write(msg.data1);
      Serial.write(msg.data2);
      sendLow[p] = false;
      highCt[p] = 0;
    }
  }

  // blend LFO waveforms
  double tmpMix = 0.0;
  double alpha = 1.0;
  if (lfoMix >= 0.0 && lfoMix <= 1.0) {
    alpha = lfoMix;
    tmpMix = (1.0 - alpha) * pgm_read_byte_near(tri + lfoPhase) + alpha * pgm_read_byte_near(noise + lfoPhase);
  }
  if (lfoMix > 1.0 && lfoMix <= 2.0) {
    alpha = lfoMix - 1.0;
    tmpMix = (1.0 - alpha) * pgm_read_byte_near(noise + lfoPhase) + alpha * pgm_read_byte_near(sawUp + lfoPhase);
  }
  if (lfoMix > 2.0 && lfoMix <= 3.0) {
    alpha = lfoMix - 2.0;
    tmpMix = (1.0 - alpha) * pgm_read_byte_near(sawUp + lfoPhase) + alpha * pgm_read_byte_near(squ50 + lfoPhase);
  }
  if (lfoMix > 3.0 && lfoMix <= 4.0) {
    alpha = lfoMix - 3.0;
    tmpMix = (1.0 - alpha) * pgm_read_byte_near(squ50 + lfoPhase) + alpha * userWave[lfoPhase];
  }

  // send LFO output to LED, CV, and MIDI
  volatile uint8_t ledValue = uint8_t(tmpMix);
  volatile uint8_t lfoValue = uint8_t(tmpMix * lfoDepth / 1.25);
  uint16_t midiLFOValue = uint16_t(lfoValue * 1.25 / 2.0);
  if (midiLFOValue < 0) {
    midiLFOValue = 0;
  }
  if (midiLFOValue > 127) {
    midiLFOValue = 127;
  }
  analogWrite(LED_OUT, ledValue);
  analogWrite(LFO_OUT, lfoValue);
  if (enableLFO && (midiLFOValue != lastLFOvalue)) {
    msg.data1 = LFO_CC;
    msg.data2 = midiLFOValue;
    Serial.write(msg.commChannel);
    Serial.write(msg.data1);
    Serial.write(msg.data2);
    lastLFOvalue = midiLFOValue;
  }

  if (lfoDownsampleCt == lfoDownsampleFactor) {
    lfoPhase += lfoSpeed;
    if (lfoPhase > PHASE_SZ) {
      lfoPhase -= PHASE_SZ;
    }
    lfoDownsampleCt = 0;
  }
  ++lfoDownsampleCt;
}
