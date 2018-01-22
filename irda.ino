#include "dwt.h"

const uint32_t cyclesPerBit = SystemCoreClock / 9756;
const unsigned irdaPort = PA0;
const uint16_t portLevel = 1700;

#define BUFFER_SIZE 0x100 // power of 2
#define BUFFER_SIZE_MASK (BUFFER_SIZE-1)
unsigned int bufferHead = 0;
unsigned int bufferTail = 0;
uint8 buffer[256];

void irdaInit(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 1;
  pinMode(irdaPort, INPUT_ANALOG);
}

inline uint8 readByte() {
  DWT->CYCCNT = 0;
  nvic_globalirq_disable();
  uint8 datum = 0;
  uint32 next = cyclesPerBit + cyclesPerBit/16;
  for (int i=0; i<9; i++) {
    while(DWT->CYCCNT < next);
    uint32 next1 = next + cyclesPerBit/16;
    if (i<8)
      while(DWT->CYCCNT < next1) {
        if (analogRead(irdaPort) >= portLevel) {
          datum |= (1 << i);
          break;
        }
      }
    next += cyclesPerBit;
  }
  next += cyclesPerBit / 32;
  while(DWT->CYCCNT < next);
  while(analogRead(irdaPort) >= portLevel);
  nvic_globalirq_enable();
  return datum^0xFF;
}

inline bool plus() {
  uint32 next = DWT->CYCCNT + cyclesPerBit/2;
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
  while(plus()) {
    uint8 datum = 0;
    uint32 next = cyclesPerBit;
    for (int i=0; i<9; i++) {
      while(DWT->CYCCNT < next);
      next += cyclesPerBit;
      if (i<8 && plus()) 
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

void loop() {
  if (analogRead(irdaPort) >= portLevel) {
    unsigned n = readData();
    if(1<n) Serial.println("hurrah"+String(n));
    if (inBuffer()>=20) {
      while(bufferHead != bufferTail) {
        Serial.println(buffer[bufferTail],HEX);
        bufferTail = (bufferTail+1)&BUFFER_SIZE_MASK;
      }
    }
  } 
//  Serial.println(String(DWT->CYCCNT)+","+String(analogRead(irdaPort)));
}

