#include "dwt.h"

const uint32_t cyclesPerBit = SystemCoreClock / 9756;
const unsigned irdaPort = PA0;
const uint16_t portLevel = 1700;
unsigned position = 0;

#define BUFFER_SIZE 0x100 // power of 2
#define BUFFER_SIZE_MASK (BUFFER_SIZE-1)
unsigned int bufferHead = 0;
unsigned int bufferTail = 0;
uint8 buffer[BUFFER_SIZE];
uint8 currentKey[3];

void irdaInit(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 1;
  pinMode(irdaPort, INPUT_ANALOG);
}

inline bool plus(uint32 length) {
  uint32 next = DWT->CYCCNT + length;
  while(DWT->CYCCNT < next) {
    if (analogRead(irdaPort) >= portLevel)
      return true;
  }
  return false;
}

inline unsigned readData() {
  unsigned bytesRead = 0;
  DWT->CYCCNT = 0;
  nvic_globalirq_disable();
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
    bytesRead++;
  }
  nvic_globalirq_enable();
  return bytesRead;
}

unsigned inBuffer() {
  return (bufferHead-bufferTail)&BUFFER_SIZE_MASK;
}

void setup() {
  irdaInit();
  pinMode(PB12,OUTPUT);
  digitalWrite(PB12,1);
}

inline void processKey(uint8 key) {
  Serial.println(key,HEX);
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

void loop() {
  if (analogRead(irdaPort) >= portLevel) {
    lastKeyTime = millis();
    readData();
  }
  else {
    if (millis() >= lastKeyTime+10 && bufferHead != bufferTail) 
      processChar();
  }
}


