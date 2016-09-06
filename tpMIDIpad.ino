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

uint16_t maxPadInt_x = 585;  
uint16_t maxPadInt_y = 678;   

void setup() // The setup runs only once, at startup.
{
  pinMode(GATE, OUTPUT);
  pinMode(CV_OUT, OUTPUT);
  pinMode(13, OUTPUT);   // Set pin 13 (the one with the LED) to output
  digitalWrite(13, LOW); // Turn off the LED
  for(int i = 0; i < NUMBER_OF_ANALOG_INPUTS; i++){  // We make all values of analogOld -1, so it will always be different from any possible analog reading.
    analogOld[i]=-1;
  }

  Serial.begin(31250);  // Start a serial connection @115200 baud or bits per second on digital pin 0 and 1, this is the connection to the ATmega16U2, which runs the MIDI firmware.
  delay(2000);           // Wait 2 seconds before sending messages, to be sure everything is set up, and to make uploading new sketches easier.
  digitalWrite(13, HIGH);// Turn on the LED, when the loop is about to start.
  
}

void loop() // The loop keeps on repeating forever.
{
  
  /************** WRITE MIDI CONTROLLER DATA *********************/ 
  
  uint16_t currVal=0;
  uint16_t minPadInt = 30;
  currVal = analogRead(ctIdx);

  switch(ctIdx){
    case PAD_X:
    delay(7);
    currVal = analogRead(ctIdx);    
    if( currVal >= minPadInt && currVal <= maxPadInt_x-20 ){
        analogVal[ctIdx] = map(currVal, minPadInt, maxPadInt_x-20, 0, 127);
        analogVal[ctIdx] = constrain(analogVal[ctIdx], 0, 127);
    }    
    break;
    case PAD_Y:
    delay(7);
    currVal = analogRead(ctIdx);    
    if( currVal >= minPadInt-5 && currVal <= maxPadInt_y-5 ){
      analogVal[ctIdx] = map(currVal, maxPadInt_y-5, minPadInt-5, 0, 127);
    }    
    break;
    default:
    analogVal[ctIdx] = currVal >> 3;
    break;
  }
  
  if(analogVal[ctIdx] != analogOld[ctIdx]){              // Only send the value, if it is a different value than last time.
    msg.commChannel = CC;
    msg.commChannel += CHANNEL-1;                     // Channels are zero based (0 = ch1, and F = ch16)
    msg.data1   = controllers[ctIdx];                // Get the controller number from the array above.
    msg.data2   = analogVal[ctIdx];                  // Get the value of the analog input from the analogVal array.
    Serial.write(msg.commChannel);
    Serial.write(msg.data1);
    Serial.write(msg.data2);
    
    analogOld[ctIdx] = analogVal[ctIdx];                   // Put the analog values in the array for old analog values, so we can compare the new values with the previous ones.
  }
  
  ++ctIdx;
  if( ctIdx >= NUMBER_OF_ANALOG_INPUTS ){
    ctIdx = 0;
  }

}
