/*
   Arduino LFO
     DDSの波形テーブルの検証

   2018.01.26

*/
#include <SPI.h>
#include "avr/pgmspace.h"

#include "wavetable_12bit_8k.h"

#define PIN_CHECK  (0)
#define BIT_LENGTH_8  (0)

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Pin Assign
const int MCP4922Ldac = 9;
const int MCP4922Cs = 10;

#if (PIN_CHECK)
const int CheckPin1 = 18;      // A4
const int CheckPin2 = 19;      // A5
#endif

// MCP4922
SPISettings MCP4922_SPISetting(8000000, MSBFIRST, SPI_MODE0);

// Parameter
double drate = 50.0;                 // initial output rate (Hz)
const double refclk = 15625.0;       // = 16MHz / 8 / 128

// DDS
volatile uint32_t phaccu;
volatile uint32_t tword_m;

//-------------------------------------------------------------------------------------------------
// Interrupt Service Routine
//

// param
//   channel: 0, 1
//   val: 0 .. 4095
void MCP4922Write(bool channel, uint16_t val)
{
  uint16_t cmd = channel << 15 | 0x3000;
  cmd |= (val & 0x0fff);

  digitalWrite(MCP4922Ldac, HIGH);
  digitalWrite(MCP4922Cs, LOW);
  SPI.transfer(highByte(cmd));
  SPI.transfer(lowByte(cmd));
  digitalWrite(MCP4922Cs, HIGH);
  digitalWrite(MCP4922Ldac, LOW);
}

ISR(TIMER2_OVF_vect)
{
#if (PIN_CHECK)
  digitalWrite(CheckPin1, HIGH);
#endif

  // synthesize
  phaccu = phaccu + tword_m;

  // テーブルサイズに合わせてシフトするビットを変更
  int idx = phaccu >> 19;  // use upper n bits (table size)

#if (BIT_LENGTH_8)
  MCP4922Write(0, pgm_read_word_near(sin_table + idx) << 4);
#else
  MCP4922Write(0, pgm_read_word_near(sin_table + idx));
#endif

#if (PIN_CHECK)
  digitalWrite(CheckPin1, LOW);
#endif
}

//-------------------------------------------------------------------------------------------------
// Setup
//

// TIMER2 setup
void Setup_timer2()
{
  // non-PWM / Normal port operation, OC0A disconnected.
  cbi (TCCR2A, COM2A0);
  cbi (TCCR2A, COM2A1);

  // Mode 7 / Fast PWM
  sbi (TCCR2A, WGM20);
  sbi (TCCR2A, WGM21);
  sbi (TCCR2B, WGM22);

  // 16000000 / 8 / 128 = 15625 Hz clock
  OCR2A = 127;

  // Timer2 Clock Prescaler to : 8
  cbi (TCCR2B, CS20);
  sbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);
}

void setup()
{
  tword_m = pow(2, 32) * drate / refclk;  // calculate DDS tuning word;

#if PIN_CHECK
  pinMode(CheckPin1, OUTPUT);
#endif

  pinMode(MCP4922Cs, OUTPUT);
  digitalWrite(MCP4922Cs, HIGH);  // set CS as inactive
  pinMode(MCP4922Ldac, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(MCP4922_SPISetting);

  Setup_timer2();

  // disable interrupts to avoid timing distortion
  cbi(TIMSK0, TOIE0);             // disable Timer0 !!! delay() is now not available
  sbi(TIMSK2, TOIE2);             // enable Timer2 Interrupt

  sei();
}

//-------------------------------------------------------------------------------------------------
// Main Loop
//
void loop()
{
}

