/*
 * Arduino LFO
 *
 * 2018.01.19
 *
 */
#include <SPI.h>
#include "avr/pgmspace.h"

#include "wavetable_12bit_8k.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define WAVESHAPE_NUM  (1)
#define DEBOUNCE_WAIT (255)

// Pin Assign
const int PotRate = 0;     // A0

const int ButtonWaveShape = 2;
const int LedSquare = 3;
const int LedSawUp = 4;
const int LedSawDown = 5;
const int LedTriangle = 6;
const int LedSine = 7;

const int MCP4922Cs = 10;
const int MCP4922Ldac = 9;

const int CheckPin1 = 18; // A4
const int CheckPin2 = 19; // A5

// MCP4922
SPISettings MCP4922_SPISetting(8000000, MSBFIRST, SPI_MODE0);

// Parameter
double drate = 100.0;                 // initial output rate (Hz)
const double refclk = 15625.0;       // = 16MHz / 8 / 128 

uint16_t *waveshapes[WAVESHAPE_NUM];
uint8_t waveshape_sel = 0;           // selected waveshape

// DDS
volatile uint32_t phaccu;
volatile uint32_t tword_m;

// Debounce
volatile uint16_t waveshape_pushed_wait = 0;

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
  digitalWrite(CheckPin1, HIGH);
  
  // synthesize
  phaccu = phaccu + tword_m;
  int idx = phaccu >> 19;  // use upper 13 bits
  
  MCP4922Write(0, pgm_read_word_near(waveshapes[waveshape_sel] + idx));
  
  // debounce
  if (waveshape_pushed_wait > 0)  waveshape_pushed_wait--;
  if (waveshape_pushed_wait == 0 && digitalRead(ButtonWaveShape) == LOW) {
    waveshape_sel++;
    if (waveshape_sel >= WAVESHAPE_NUM)  waveshape_sel = 0;
  }
  
  digitalWrite(CheckPin1, LOW);
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
  waveshapes[0] = sin_12bit_8k;
//  waveshapes[1] = tri256;
//  waveshapes[2] = saw1_256;
//  waveshapes[3] = saw2_256;
//  waveshapes[4] = sqr256;

  tword_m = pow(2, 32) * drate / refclk;  // calculate DDS tuning word;
  
  pinMode(ButtonWaveShape, INPUT_PULLUP);
  
  pinMode(LedSquare, OUTPUT);
  pinMode(LedSawUp, OUTPUT);
  pinMode(LedSquare, OUTPUT);
  pinMode(LedSawDown, OUTPUT);
  pinMode(LedTriangle, OUTPUT);
  pinMode(LedSine, OUTPUT);
  
  pinMode(CheckPin1, OUTPUT);
  pinMode(CheckPin2, OUTPUT);
  
  pinMode(MCP4922Cs, OUTPUT);
  pinMode(MCP4922Ldac, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(MCP4922_SPISetting);
  
  Setup_timer2();
  
  // disable interrupts to avoid timing distortion
  cbi(TIMSK0,TOIE0);              // disable Timer0 !!! delay() is now not available
  sbi(TIMSK2,TOIE2);              // enable Timer2 Interrupt
  
  sei();
}

//-------------------------------------------------------------------------------------------------
// Main Loop
//
void loop()
{
  digitalWrite(CheckPin2, HIGH);
  
  // drate = (float)analgoRead(PotRate) / 102.4f;
  tword_m = pow(2, 32) * drate / refclk;  // calulate DDS new tuning word
  
  if (waveshape_pushed_wait == 0 && digitalRead(ButtonWaveShape) == LOW)
      waveshape_pushed_wait = DEBOUNCE_WAIT;
  
  digitalWrite(CheckPin2, LOW);
}

