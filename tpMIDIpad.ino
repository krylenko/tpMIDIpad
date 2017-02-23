// Arduino Uno MIDI touch pad / control surface

#define NOTE_OFF       0x80
#define NOTE_ON        0x90
#define CC             0xB0
#define MIDI_CHANNEL        1

// define switch and button (momentary) inputs
#define NUM_SWITCHES        3 
enum{
  TOGGLE_LOWER = 5,
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
enum{
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
uint16_t buttonVal[NUM_BUTTONS];
uint16_t buttonOld[NUM_BUTTONS];

// define analog inputs
#define NUM_ANALOG_INPUTS   6
enum{
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
  108,      // D5     lower toggle        CC 108
  109,      // D9     toggle top left     CC 109
  110       // D10    toggle top right    CC 110
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

int statusByte = 0;
int noteByte = 0;
uint8_t playing = 0;
int currNote = 0;

int first = 0, second = 0;

uint8_t ctIdx = 0;
t_midiMsg msg;

uint16_t maxPadInt_x = 585;  
uint16_t maxPadInt_y = 678;   

void setup()
{

  for (int i = 0; i < NUM_ANALOG_INPUTS; i++) {
    analogOld[i] = -1;
  }
  for (int j = 0; j < NUM_SWITCHES; ++j) {
    pinMode(switches[j], INPUT);
    switchOld[j] = -1;
  }
  for (int k = 0; k < NUM_BUTTONS; ++k) {
    pinMode(buttons[k], INPUT);
    switchOld[k] = -1;
  }  

  Serial.begin(31250);   
  delay(2000);           
  
}

void loop()
{
  
  /************** WRITE MIDI CONTROLLER DATA *********************/ 

  // read and output analog MIDI controllers
  uint16_t currVal = 0;
  uint16_t minPadInt = 30;
  currVal = analogRead(ctIdx);

  msg.commChannel = CC;
  msg.commChannel += MIDI_CHANNEL - 1; 

  switch(ctIdx)
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
      if ( currVal >= minPadInt - 5 && currVal <= maxPadInt_y - 5 ) {
        analogVal[ctIdx] = map(currVal, maxPadInt_y - 5, minPadInt - 5, 0, 127);
      }   
      break;
    }
    default:
    {
      analogVal[ctIdx] = currVal >> 3;
      break;
    }
  }
  
  if(analogVal[ctIdx] != analogOld[ctIdx]){                                  
    msg.data1 = controllers[ctIdx];               
    msg.data2 = analogVal[ctIdx];                  
    Serial.write(msg.commChannel);
    Serial.write(msg.data1);
    Serial.write(msg.data2);
    analogOld[ctIdx] = analogVal[ctIdx];                   
  }
  
  ++ctIdx;
  if( ctIdx >= NUM_ANALOG_INPUTS ) {
    ctIdx = 0;
  }

  // read and write states of switches
  for (int m = NUM_ANALOG_INPUTS; m < NUM_ANALOG_INPUTS + NUM_SWITCHES; ++m) {                    
    int tempVal = digitalRead(switches[m - NUM_ANALOG_INPUTS]);
    if (tempVal == HIGH) {
      tempVal = 127;
    } else {
      tempVal = 0;                
    }
    switchVal[m - NUM_ANALOG_INPUTS] = tempVal;        
    if( switchVal[m - NUM_ANALOG_INPUTS] != switchOld[m - NUM_ANALOG_INPUTS] ) {           
      msg.data1 = controllers[m];
      msg.data2 = switchVal[m - NUM_ANALOG_INPUTS];
      Serial.write(msg.commChannel);
      Serial.write(msg.data1);
      Serial.write(msg.data2);
      switchOld[m - NUM_ANALOG_INPUTS] = switchVal[m - NUM_ANALOG_INPUTS];
    }    
  }

}
