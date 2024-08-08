/* Compile for Nano Every
 * Nano Every is found in Mega2560 boards
 */
 
#include <avr/sleep.h>

// For Nano Every port definitions, see C:\Program Files (x86)\Arduino\hardware\tools\avr\avr\include\avr\iom4809.h

/*  Attempt to get it into low power state most of the time to reduce cooling requirements in vacuum
 *  - It didn't work
 *  - when trying to compile it later, the register accesses in the set_low_power routine failed 
 *  to compile in spite of setting AtMEGA328 emulation (although they had compiled previously)
 *  
 *  ... so the code is #defined out
*/
#define LOWPOWER_SLEEP 0
#define CTRL_C 3

// Macro to return number of elements in an array
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// relay setup parameters
const int boardCount = 6;           // how many boards are supported
const int channelsPerBoard = 15;    // how many channels per board
const int maxChan = boardCount * channelsPerBoard; // number of channels
const int senseRelayBit = 15;       // position of sense relay bit

// shift register (74HC595) control pinout 
int serPins[boardCount] = {2, 3, 4, 5, 6, 7};
const int SRCLK = 9;                // serial clock 
const int RCLK = 10;                // strobe to active register 
const int SRCLR = 11;               // asynchronous clear (active low)

// For synchronization with IV curves using falling voltage transition
// Note: not on rev 1.0 PCB
const int SYNC_PULSE = 8;  // for sensing sync pulse - stealing last board select pin, for now

// Set to Serial to debug over USB
// Set to Serial1 to connect via TTL
HardwareSerial &control = Serial1;
HardwareSerial &debug = Serial1;

const int inbuflen = 80;
char inbuf[inbuflen];
int charsInBuf;

void setup() {
  int i;

  control.begin(9600);
//  debug.begin(9600);
  
  for (i = 0; i < boardCount; i++)
    pinMode(serPins[i], OUTPUT);

  pinMode(SRCLK, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SRCLR, OUTPUT);
  
  digitalWrite(SRCLK, LOW);
  digitalWrite(RCLK, LOW);
  digitalWrite(SRCLR, HIGH);

  pinMode(SYNC_PULSE, INPUT_PULLUP);
  
  charsInBuf = 0;
  zeroState();
  writeState();
#if LOWPOWER_SLEEP
  set_low_power();
#endif
}

void loop() {
  int c;
#if LOWPOWER_SLEEP
  sleep();  // wake on UART interrupt
#endif  
  // read a character
  c = control.read();
  if (-1 == c) return;

  // add it to the buffer, if there's room
  if (charsInBuf < COUNT_OF(inbuf))
    inbuf[charsInBuf++] = c;
  
  #define DELIMS " \n\r"
  // separate into command and arguments
  char *command, *arg1, *arg2, *arg3;
  if (c == '\n' || c == '\r') {
      if (inbuf[charsInBuf-1] == '\n') inbuf[--charsInBuf] = 0;
      if (inbuf[charsInBuf-1] == '\r') inbuf[--charsInBuf] = 0;      
      debug.print("Command: <"); debug.print(inbuf); debug.println(">");
      command = strtok(inbuf, DELIMS);
      arg1 = strtok(NULL, DELIMS);
      arg2 = strtok(NULL, DELIMS);
      arg3 = strtok(NULL, DELIMS);
  }
  else
    return;

  int status = -1;

  // "a" activate command
  // activate just one channel, with its bias and sense relays
  if (0 == strcasecmp(command, "a")) {
    debug.println("Activating");
    int chan = atoi(arg1);
    status = channelActive(chan);
  }


  // "t" temperature read command
  else if (0 == strcasecmp(command, "t")) {
    tempRead();
    status = 0;
  }

  // "s" sense relay command
  // set state of sense relay on board of specified channel: s c 1/0
  else if (0 == strcasecmp(command, "s")) {
    debug.println("Sense");
    int chan  = atoi(arg1);
    int state = atoi(arg2);
    status = setSenseRelayState(chan, state);
    writeState();
  }
  
  // "b" bias relay command
  // set state of bias  relay on specified board and channel: s c 1/0
  else if (0 == strcasecmp(command, "b")) {
    debug.println("Bias");    
    int chan =  atoi(arg1);
    int state = atoi(arg2);
    status = setBiasRelayState(chan, state);
    writeState();
  }

  // "q" seQuence command
  // run through a range of channels, activating one at a time, for specified time (mS)
  else if (0 == strcasecmp(command, "q")) {
    debug.println("Sequence");    
    int from = atoi(arg1);
    int to = atoi(arg2);
    unsigned long  ms = atol(arg3);
    status = sequence(from, to, ms);
  }

  // "d" dump command
  // dump state
  else if (0 == strcasecmp(command, "d")) {
    debug.println("Dump");
    dumpState();
    status = 0;
  }
  
  // "w" write command
  // write state to relays
  else if (0 == strcasecmp(command, "w")) {
    debug.println("Write");
    writeState();
    status = 0;
  }
  
  // "z" zero state command
  // zero state
  else if (0 == strcasecmp(command, "z")) {
    debug.println("Zero");
    zeroState();
    writeState();
    status = 0;
  }

  else if (*inbuf == 0) {
    printHelp();
    status = 0; // no error on null command
  }
  else {
    printHelp();    
  }
  
  if (status)
    debug.println("Error in command");
  else
    debug.println("Command complete");
  
  charsInBuf = 0;
}

#if LOWPOWER_SLEEP
// disable some unused peripherals
void set_low_power() {
  // Disable the ADC by setting the ADEN bit (bit 7)  of the
  // ADCSRA register to zero.
  ADCSRA &= ~0x80; // B01111111;
  
  // Disable the analog comparator by setting the ACD bit
  // (bit 7) of the ACSR register to one.
  ACSR |= 0x80; // B10000000;
  
  // Disable digital input buffers on all analog input pins
  // by setting bits 0-5 of the DIDR0 register to one.
  // Of course, only do this if you are not using the analog 
  // inputs for your project.
  // DIDR0 = DIDR0 | B00111111; // not sure about this one
}

void sleep()
{
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_cpu();
}
#endif

// An array of 16-bit words, one for each board (SER pin)
// active high
// bit 0 = ON0 .. bit 14 = ON14 .. bit 15 = SENSE_ON
uint16_t outState[boardCount];

int channelActive(int chan) {
    zeroState();
    if (setBiasRelayState(chan, true)) return -1;
    if (setSenseRelayState(chan, true)) return -1;
    writeState();
    return 0;
}

const unsigned long MIN_PULSE = 500;    // 500uS

int sequence(int from, int to, long mSdelay) {
  int i;
  bool abort = false;
  if (from < 1 || to < 1 || from > to)
    return -1;
  for (i=from; i<=to && !abort; i++) {
    
    debug.print("Activating "); debug.print(i); 
    debug.print(", at "); debug.println(millis());
    
    if (channelActive(i)) return -1;  // activate channel; abort on error exit
    
    // for positive delay parameter, just delay
    if (mSdelay >= 0)
      delay(mSdelay);
    else {
      // for negative delay parameter, first find a (long enough) negative-going pulse on the sync line
      bool done = false;
      while (!done && !abort) {
        while (HIGH == digitalRead(SYNC_PULSE))
          if (abort |= control.read() == CTRL_C) break;
        unsigned long start = micros();
        while (LOW == digitalRead(SYNC_PULSE))
          if (abort |= control.read() == CTRL_C) break;
        unsigned long width = micros() - start;
        done = width >= MIN_PULSE;
        debug.print("Pulse width: "); debug.println(width);
      }

      delay(-mSdelay);
    }
  }
  zeroState();
  writeState();
  return 0;
}

void writeState() {
  int i, j;
  uint16_t mask;
  for (i = 0, mask = 0x8000; mask; i++, mask >>=1) {
    // set SER pins
    for (j = 0; j < boardCount; j++)
      digitalWrite(serPins[j], 0 != (outState[j] & mask)? HIGH : LOW);

    // clock '595 shift register
    digitalWrite(SRCLK, HIGH);
    digitalWrite(SRCLK, LOW);
  }

  // update '595 output register
  digitalWrite(RCLK, HIGH);
  digitalWrite(RCLK, LOW);
}

void zeroState() {
  int i;
  for (i=0; i<boardCount; i++)
    outState[i] = 0;
}

void dumpState() {
  int i;
  for (i=0; i<boardCount; i++)
    debug.println(outState[i], HEX);
}

int setBiasRelayState(uint16_t channel, bool enable) {
  if (channel < 1 || channel > maxChan) return -1;

  int board = (channel-1) / channelsPerBoard;
  int chanOnBoard = (channel-1) % channelsPerBoard;

  if (enable)
    bitSet(outState[board], chanOnBoard);
  else
    bitClear(outState[board], chanOnBoard);

  return 0;
}

int setSenseRelayState(uint16_t channel, bool enable) {
  if (channel < 1 || channel > maxChan) return -1;

  int board = (channel-1) / channelsPerBoard;

  if (enable)
    bitSet(outState[board], senseRelayBit);
  else
    bitClear(outState[board], senseRelayBit);
  return 0;
}

void printHelp() {
    debug.println("a <channel>           : activate <channel>, clearing everything else and setting bias and sense");
    debug.println("s <channel> <state>   : set sense relay of <channel> to <state>");
    debug.println("t                     : print temperature (Kelvin) once per second until another character is received");
    debug.println("b <channel> <state>   : set bias relay of <channel> to <state>");
    debug.println("q <from> <to> <delay> : activate channel from <from> to <to> in seQuence, with <delay> (mS) per step");
    debug.println("      (if <delay> < 0 wait for a trigger after each channel activation, and then wait - <delay> ");
    debug.println("d                     : dump state");
    debug.println("w                     : write state to relays");
    debug.println("z                     : zero state (and write to relays)");  
}

void tempRead() {

  // read internal temperature sensor
  while (control.read() == -1) {
    // Set VREF to 1.1V
    VREF.CTRLA = 0x11;    // 1V1 for ADC0, don't care for AC0
    VREF.CTRLB = 0x02;    // force enable of VREF for ADC0 (probably not needed)
  
    //ADC0.CTRLA = 0;       // disable until we're ready
    ADC0.CTRLC = 0x46;    // 128 prescaler, internal voltage reference, SAMPCAP = 1
  
    ADC0.MUXPOS = 0x1E;   // TEMPSENSE
  
    ADC0.CTRLD = 0xE0;    // maximum INITDLY
    ADC0.SAMPCTRL = 0x1f; // maximum SAMPLEN
    
    ADC0.CTRLA = 1;       // enable ADC
  
    ADC0.COMMAND = 1;     // start conversion

    while (ADC0.COMMAND & 0x01)
      ;

    debug.println((ADC0.RES - int8_t(SIGROW_TEMPSENSE1))*(SIGROW_TEMPSENSE0/256.0));

    delay(1000);
  }
}
