/* This is the code for the Arduino Uno, it will not work on the Leonardo. */

/* These are constants: whenever we put one of these 3 words in capitals in our code, the precompiler will replace it by the 0xsomething after the word. 
 This makes our code more readable, instead of typing the meaningless 0x90, we can now just type NOTE_ON, if we want a note to be on. */
#define NOTE_OFF       0x80
#define NOTE_ON        0x90
#define CC             0xB0

#define LOWEST_NOTE    24
#define HIGHEST_NOTE   76
#define OCTAVE_OFFSET  12    // we want note 48 (C3) to be at 0.5v
#define GATE           2
#define CV_OUT         5

/* This is also a constant. If you want to change the number of analog inputs, you can simply do it once on this line, instead of changing it everywhere in your code.*/ 
#define NUMBER_OF_ANALOG_INPUTS  3 // The Uno has 6 analog inputs, we'll use all of them in this example. If you only need 4, change this to 4, and you'll be able to use A4 & A5 as normal I/O pins. 
//NOTE: if you change this value, also change the controllers array, to match the number of inputs and the number of controllers.

#define CHANNEL  1 // Send all messages on channel 1.

#define FILTLEN 3

enum{
  PAD_X,
  PAD_Y,
  EXP_PEDAL
};

/* The list with the corresponding controller numbers: for example, the values of the potentiometer on A0 will be sent as the first controller number in this list, A1 as the second, etc... 
 Here's the list with all controller numbers:  http://midi.org/techspecs/midimessages.php#3  You can change them if you want.*/
// http://nickfever.com/music/midi-cc-list
int controllers[NUMBER_OF_ANALOG_INPUTS] = { 
  102,    // A0 = pad X --> CC 102 
  103,    // A1 = pad Y --> CC 103 
  104    // A2 = exp pedal --> CC 104 
  //105,    // A3 
  //106,    // A4
  //107     // A5
};  

const int lenLUT = 53;
const int tuneLUT[lenLUT] = {
  13, 14, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24,
  26, 27, 29, 30, 32, 34, 36, 38, 41, 43, 46, 48, 51, 54, 57, 61, 65, 68, 72,
  77, 81, 86, 91, 97, 102, 108, 115, 122, 129, 137, 145, 153, 163, 172, 182, 193,
  205, 217, 230, 244, 258
};

const int notesTracked = 15;
int noteIdx = 0;
int noteStack[notesTracked];

uint16_t analogVal[NUMBER_OF_ANALOG_INPUTS];  // We declare an array for the values from the analog inputs

uint16_t analogOld[NUMBER_OF_ANALOG_INPUTS]; // We declare an array for the previous analog values. 

/* The format of the message to send via serial. We create a new data type, that can store 4 values at once.  This will be easier to send as MIDI. */
typedef struct {
  uint8_t commChannel;  // first  4 bits : channel message (NOTE_ON, NOTE_OFF or CC (controlchange)
                        // second 4 bits : midi channel (0-15)
  uint8_t data1;        // second byte   : first value (0-127), controller number or note number
  uint8_t data2;        // third  byte   : second value (0-127), controller value or velocity
} 
t_midiMsg;          // We call this data type 't_midiMsg

int statusByte = 0;
int noteByte = 0;
uint8_t playing = 0;
int currNote = 0;

int first = 0, second = 0;

uint8_t ctIdx = 0;
t_midiMsg msg;

uint16_t maxPadInt = 580;    // 583 corresponds to max pad values of ~2.85 v 
uint16_t minPadInt = 5;   

uint8_t filtIdx = 0;
uint16_t LPF[FILTLEN];

void setup() // The setup runs only once, at startup.
{
  pinMode(GATE, OUTPUT);
  pinMode(CV_OUT, OUTPUT);
  pinMode(13, OUTPUT);   // Set pin 13 (the one with the LED) to output
  digitalWrite(13, LOW); // Turn off the LED
  for(int i = 0; i < NUMBER_OF_ANALOG_INPUTS; i++){  // We make all values of analogOld -1, so it will always be different from any possible analog reading.
    analogOld[i]=-1;
  }
  for(int i = 0; i < notesTracked; ++i){
    noteStack[i] = 0;
  }
  Serial.begin(31250);  // Start a serial connection @115200 baud or bits per second on digital pin 0 and 1, this is the connection to the ATmega16U2, which runs the MIDI firmware.
  delay(2000);           // Wait 2 seconds before sending messages, to be sure everything is set up, and to make uploading new sketches easier.
  digitalWrite(13, HIGH);// Turn on the LED, when the loop is about to start.

  for(uint8_t i=0; i<FILTLEN; ++i){
    LPF[i] = 0;
  }
  
}

int lookUpTuning(int & midiNote){
  
  int mapNote = 0;
  int shiftNote = midiNote - OCTAVE_OFFSET;
  
  if( shiftNote > HIGHEST_NOTE ){
    mapNote = tuneLUT[HIGHEST_NOTE-LOWEST_NOTE];
  }
  else if( shiftNote < LOWEST_NOTE ){
    mapNote = tuneLUT[0];
  }
  else{
    mapNote = tuneLUT[shiftNote-LOWEST_NOTE];
  }
  
  return mapNote;
  
}

void noteOn(int& note_){
  noteStack[noteIdx] = note_;
  noteIdx++;
  if( noteIdx > notesTracked ){
    noteIdx = 0;
  }
  analogWrite(CV_OUT, lookUpTuning(note_));
  digitalWrite(GATE, HIGH); 
}

void noteOff(int& note_){
  
  // "turn off" this particular note
  int lowestNote = HIGHEST_NOTE;
  for(int i=notesTracked-1; i>=0; --i){
    if( noteStack[i] == note_ ){
      noteStack[i] = 0;
    }
    else{
      if( noteStack[i] <= lowestNote && noteStack[i] != 0 ){
        lowestNote = noteStack[i];
      }
    }
  } 
  if( lowestNote != HIGHEST_NOTE ){
    noteOn(lowestNote);
  }
  
  // check the stack to see if other notes are active
  int flag = 0;
  for(int i=0; i<notesTracked; ++i){
    flag |= noteStack[i];
  } 
  if( !flag ){
    digitalWrite(GATE,LOW);
  }
}

uint16_t mean(uint16_t * filt_)
{
  uint16_t mean=0;
  for(int i=0; i<FILTLEN; ++i){
    mean += filt_[i];
  }

  return mean/FILTLEN;
}

void loop() // The loop keeps on repeating forever.
{
  
  /************** READ MIDI NOTE DATA *********************/
  /*
  // good code in this block, in case I need it again
  if( Serial.available() > 0 ){
    statusByte = Serial.read();
  
    if( statusByte == NOTE_ON ){
      while( Serial.available() == 0 );
      noteByte = Serial.read();
      noteOn(noteByte);  
    }
    
    if( statusByte == NOTE_OFF ){
      while( Serial.available() == 0 );      
      noteByte = Serial.read();
      noteOff(noteByte);
    }
    
  }
  */
  /*
  if( Serial.available() > 0 ){
    statusByte = Serial.read();
    
    if( statusByte == NOTE_ON ){
      while( Serial.available() == 0 );
      noteByte = Serial.read();
      analogWrite(CV_OUT, lookUpTuning(noteByte) );
      currNote = noteByte;
      if( !playing ){
        digitalWrite(GATE, HIGH);
        playing = 1;
      }      
    }
    
    if( statusByte == NOTE_OFF ){
      while( Serial.available() == 0 );      
      noteByte = Serial.read();      
      if( playing && noteByte == currNote ){
        digitalWrite(GATE, LOW);
        analogWrite(CV_OUT, LOWEST_NOTE);
        currNote = 0;
        playing = 0;
      }
    }
    
  }
  */
  
  
  /************** WRITE MIDI CONTROLLER DATA *********************/ 
  
  uint16_t currVal=0;
  currVal = analogRead(ctIdx);
  if( ctIdx == PAD_X || ctIdx == PAD_Y ){
    if( currVal >= minPadInt && currVal <= maxPadInt ){
      if( ctIdx == PAD_Y ){
        currVal = maxPadInt - currVal;
        if( currVal < minPadInt ) { currVal = minPadInt; }
      }
      analogVal[ctIdx] = map(currVal, minPadInt, maxPadInt, 0, 127);
      //analogVal[ctIdx] = mean(LPF);     
      //analogVal[ctIdx] = currVal >> 2;
    }
  }
  else{
    analogVal[ctIdx] = currVal >> 3;                // The resolution of the Arduino's ADC is 10 bit, and the MIDI message has only 7 bits, 10 - 7 = 3, so we divide by 2^3, or 8.
  }
  if(analogVal[ctIdx] != analogOld[ctIdx]){              // Only send the value, if it is a different value than last time.
    msg.commChannel = CC;
    msg.commChannel += CHANNEL-1;                     // Channels are zero based (0 = ch1, and F = ch16)
    msg.data1   = controllers[ctIdx];                // Get the controller number from the array above.
    msg.data2   = analogVal[ctIdx];                  // Get the value of the analog input from the analogVal array.
    Serial.write(msg.commChannel);
    Serial.write(msg.data1);
    Serial.write(msg.data2);
    
    /*
    Serial.print("\ncommChannel\n");
    Serial.print(msg.commChannel, BIN);
    Serial.print("\nctrl num\n");      
    Serial.print(msg.data1, DEC);
    Serial.print("\ndata\n");      
    Serial.print(msg.data2, DEC);
    
    Serial.print("\n");
    Serial.print(sizeof(msg.commChannel),DEC);
    Serial.print("\n");
    Serial.print(sizeof(msg.data1),DEC);
    Serial.print("\n");
    Serial.print(sizeof(msg.data2),DEC);
    Serial.print("\n");
    Serial.print(sizeof(msg),DEC);      
    */
    analogOld[ctIdx] = analogVal[ctIdx];                   // Put the analog values in the array for old analog values, so we can compare the new values with the previous ones.
  }
  
  ++ctIdx;
  if( ctIdx >= NUMBER_OF_ANALOG_INPUTS ){
    ctIdx = 0;
  }
  ++filtIdx;
  if( filtIdx >= FILTLEN ){
    filtIdx = 0;
  }

}
