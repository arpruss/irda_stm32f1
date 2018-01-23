#include <USBHID.h>
#include "dwt.h"

const unsigned ledPin = PB12;
uint8 capslock = 0;
const uint32_t cyclesPerBit = SystemCoreClock / 9756;
const unsigned irdaPort = PA0;
const uint16_t portLevel = 1700;
unsigned position = 0;
uint8 keys[0x80];
uint8 fnMode = 0;

#define BUFFER_SIZE 0x100 // power of 2
#define BUFFER_SIZE_MASK (BUFFER_SIZE-1)
volatile unsigned int bufferHead = 0;
unsigned int bufferTail = 0;
volatile uint8 buffer[BUFFER_SIZE];
uint8 currentKey[3];

void irdaInit(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 1;
  pinMode(irdaPort, INPUT_ANALOG);
}


void setupMap() {
  for(int i=0; i<0x80; i++) keys[i] = 0xFF;
    keys[0x4e] = '-'; 
    keys[0x0e] = '`'; 
    keys[0x55] = '='; 

    keys[0x66] = KEY_BACKSPACE; 
    keys[0x0d] = KEY_TAB; 
    keys[0x54] = '['; 
    keys[0x5b] = ']'; 
    keys[0x5d] = '\\';

    keys[0x4c] = ';'; 
    keys[0x52] = '\''; 
    keys[0x5a] = KEY_RETURN; 
    keys[0x48] = ',';
    keys[0x49] = '.';
    keys[0x4a] = '/';

    //keys[0x11] = KeyEvent.KEYCODE_HOME;

    //keys[0x12] = KeyEvent.KEYCODE_SHIFT_LEFT;
    //keys[0x59] = KeyEvent.KEYCODE_SHIFT_RIGHT;
    keys[0x29] = ' ';

    keys[0x28] = KEY_UP_ARROW;
    keys[0x5e] = KEY_LEFT_ARROW;
    keys[0x60] = KEY_DOWN_ARROW;
    keys[0x2f] = KEY_RIGHT_ARROW;

    keys[0x16] = '1';
    keys[0x1e] = '2';
    keys[0x26] = '3';
    keys[0x25] = '4';
    keys[0x2e] = '5';
    keys[0x36] = '6';
    keys[0x3d] = '7';
    keys[0x3e] = '8';
    keys[0x46] = '9';
    keys[0x45] = '0';

    keys[0x15] = 'q';
    keys[0x1d] = 'w';
    keys[0x24] = 'e';
    keys[0x2d] = 'r';
    keys[0x2c] = 't';
    keys[0x35] = 'y';
    keys[0x3c] = 'u';
    keys[0x43] = 'i';
    keys[0x44] = 'o';
    keys[0x4d] = 'p';

    keys[0x1c] = 'a';
    keys[0x1b] = 's'; 
    keys[0x23] = 'd';
    keys[0x2b] = 'f';
    keys[0x34] = 'g';
    keys[0x33] = 'h';
    keys[0x3b] = 'j';
    keys[0x42] = 'k';
    keys[0x4b] = 'l';

    keys[0x1a] = 'z';
    keys[0x22] = 'x';
    keys[0x21] = 'c';
    keys[0x2a] = 'v';
    keys[0x32] = 'b';
    keys[0x31] = 'n';
    keys[0x3a] = 'm';
    keys[0x58] = KEY_CAPS_LOCK;
    keys[0x1f] = KEY_DELETE;

    //keys[0x30] = KeyEvent.KEYCODE_ALT_RIGHT;

    //keys[0x03] = KeyEvent.KEYCODE_MENU; // Windows key

    //keys[0x02] = KeyEvent.KEYCODE_FUNCTION;
    //keys[0x14] = KeyEvent.KEYCODE_CTRL_LEFT; 
}

inline bool plus(uint32 length) {
  uint32 next = DWT->CYCCNT + length;
  while(DWT->CYCCNT < next) {
    if (analogRead(irdaPort) >= portLevel)
      return true;
  }
  return false;
}

void readData() {
  DWT->CYCCNT = 0;
  while(plus(cyclesPerBit*2)) {
    DWT->CYCCNT = 0;
    uint8 datum = 0;
    uint32 next = cyclesPerBit;
    for (int i=0; i<9; i++) {
      while(DWT->CYCCNT < next);
      next += cyclesPerBit;
      if (i<8 && plus(cyclesPerBit/2)) 
         datum |= (1 << i);
    }
    while(DWT->CYCCNT < next);
    DWT->CYCCNT = 0;
    buffer[bufferHead] = datum^0xFF;
    bufferHead = (bufferHead+1)&BUFFER_SIZE_MASK;
  }
}

unsigned inBuffer() {
  return (bufferHead-bufferTail)&BUFFER_SIZE_MASK;
}

void watchdogInterrupt(void) {
  readData();
}

static inline void waitForThreshold() {
  ADC1->regs->SQR3 = PIN_MAP[irdaPort].adc_channel;
  ADC1->regs->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART;
  ADC1->regs->HTR = portLevel;
  ADC1->regs->LTR = 0;
  ADC1->regs->CR1 = ADC_CR1_AWDEN; // | ADC_CR1_AWDSGL; | ADC_CR1_AWDIE;
  adc_attach_interrupt(ADC1, ADC_AWD, readData);
}

void setup() {
  USBHID.setSerial(0);
  USBHID.begin(HID_KEYBOARD);
  Keyboard.begin();
  irdaInit();
  setupMap();
  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,1);
  delay(1000);
  waitForThreshold();
}

inline void processKey(uint8 key) {
//  CompositeSerial.println(key,HEX);
  uint8 rel = (key&0x80) != 0;
  key &= 0x7F;
  uint8 out = keys[key];
  if (fnMode) {
    if (out == '`') 
      out = KEY_ESC;
    else if (out == KEY_LEFT_ARROW) 
      out = KEY_HOME;
    else if (out == KEY_RIGHT_ARROW)
      out = KEY_END;
    else if (out == KEY_UP_ARROW)
      out = KEY_PAGE_UP;
    else if (out == KEY_DOWN_ARROW)
      out = KEY_PAGE_DOWN;
  }
  
  if (out == 0xFF) {
    uint8 mask = 0;
    if (key == 0x12) // left shift
      mask = 0x02;
    else if (key == 0x59)  // right shift
      mask = 0x20;
    else if (key == 0x14) // left ctrl
      mask = 0x01;
    else if (key == 0x30) // right alt
      mask = 0x40;
    else if (key == 0x03) // windows
      mask = 0x08;
    else if (key == 0x11) // home->left alt
      mask = 0x04;
    else if (key == 0x02) {// Fn 
      fnMode = !rel;
    }

    if (mask) {
      if (rel)
        Keyboard.keyReport.modifiers &= ~mask; 
      else
        Keyboard.keyReport.modifiers |= mask; 
      Keyboard.sendReport();
    }
    return;
  }
  if (rel)
    Keyboard.release(out);
  else
    Keyboard.press(out);
}

inline void processChar() {
  uint8 datum = buffer[bufferTail];
  bufferTail = (bufferTail + 1) & BUFFER_SIZE_MASK; 
  if (datum == 0xFF) {
    position = 1;
  }
  else if (position == 1) {
    if (datum == 0xC0) {
      position++;
    }
    else {
      position = 0;
    }    
  }
  else if (position == 2) {
    currentKey[0] = datum;
    position++;
  }
  else if (position == 3) {
    currentKey[1] = datum^0xFF;
    position++;
  }
  else if (position == 4) {
    currentKey[2] = datum^0xFF;
    position++;
  }
  else if (position == 5) {
    if (datum == 0xC1) {
      uint8 key;
      if (currentKey[0] == currentKey[1] || currentKey[0] == currentKey[2]) {
        key = currentKey[0];
      }
      else if (currentKey[1] == currentKey[2]) {
        key = currentKey[1];
      }
      processKey(key);
      position = 0;
    }
  }
}

uint32 lastKeyTime = 0;

void xloop() {
  if (analogRead(irdaPort) >= portLevel) {
    lastKeyTime = millis();
    readData();
  }
  else {
    if (millis() >= lastKeyTime+10 && bufferHead != bufferTail) 
      processChar();
  }
}

void yloop() {
  if (analogRead(irdaPort) >= portLevel) {
    lastKeyTime = millis();
    readData();
  }
  else {
    if (millis() >= lastKeyTime+10 && bufferHead != bufferTail) 
      processChar();
  }
}

void loop() {
  //CompositeSerial.println(analogRead(irdaPort));
  if (bufferHead != bufferTail) 
    processChar();
  uint8 curCapslock = (Keyboard.getLEDs() & 0x02) != 0;
  if (curCapslock != capslock) {
    capslock = curCapslock;
    digitalWrite(ledPin, !capslock);
  }
}


