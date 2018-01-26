/*
   Arduino LFO

   2018.01.19

*/
#include <SPI.h>
#include "avr/pgmspace.h"

#include "wavetable_12bit_2k.h"

#define UART_TRACE  (1)
#define TITLE_STR1  ("Arduino LFO")
#define TITLE_STR2  ("20180126")

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define WAVESHAPE_NUM  (5)
#define DEBOUNCE_WAIT (1000)

// Pin Assign
const int PotRate = 0;          // A0
const int PotPulseWidth = 1;    // A1

const int ButtonWaveShape = 2;

const int LedSin = 3;
const int LedTri = 4;
const int LedSqr = 5;
const int LedSawUp = 6;
const int LedSawDown = 7;

const int MCP4922Ldac = 9;
const int MCP4922Cs = 10;

const int CheckPin1 = 18;      // A4
const int CheckPin2 = 19;      // A5

// MCP4922
SPISettings MCP4922_SPISetting(8000000, MSBFIRST, SPI_MODE0);

// Parameter
double drate = 10.0;                 // initial output rate (Hz)
const double refclk = 15625.0;       // = 16MHz / 8 / 128

enum {
  WS_SIN,
  WS_TRI,
  WS_SQR,
  WS_SAWUP,
  WS_SAWDOWN
};

int waveshape_sel = WS_SIN;           // selected waveshape

int pulse_width = 512;

// DDS
volatile uint32_t phaccu;
volatile uint32_t tword_m;

// Debounce
volatile int16_t waveshape_pushed_wait = 0;

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
  int idx = phaccu >> 21;  // use upper 11 bits

  switch (waveshape_sel) {
    case WS_SIN:
      MCP4922Write(0, pgm_read_word_near(sin_12bit_2k + idx));
      break;
    case WS_TRI:
      MCP4922Write(0, pgm_read_word_near(tri_12bit_2k + idx));
      break;
    case WS_SQR:
      MCP4922Write(0, pgm_read_word_near(sqr_12bit_2k + idx));
      break;
    case WS_SAWUP:
      MCP4922Write(0, pgm_read_word_near(sawup_12bit_2k + idx));
      break;
    case WS_SAWDOWN:
      MCP4922Write(0, pgm_read_word_near(sawdown_12bit_2k + idx));
      break;
  }

  // debounce
  if (waveshape_pushed_wait > 0) {
    waveshape_pushed_wait--;
    if (waveshape_pushed_wait == 0 && digitalRead(ButtonWaveShape) == LOW) {
      waveshape_sel++;
      if (waveshape_sel >= WAVESHAPE_NUM) {
        waveshape_sel = 0;
      }
    }
  }

  digitalWrite(CheckPin1, LOW);
}

void waveshape_pushed()
{
  waveshape_pushed_wait = DEBOUNCE_WAIT;
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

void LedsCheck()
{
  for (int j = 0; j < 3; j++) {
    for (int i = 3; i <= 7; i++) {
      PORTD |= (1 << i);
      delay(100);
      PORTD &= ~(1 << i);
    }
  }
}

void setup()
{
  tword_m = pow(2, 32) * drate / refclk;  // calculate DDS tuning word;

  pinMode(ButtonWaveShape, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ButtonWaveShape), waveshape_pushed, FALLING);

  // set LEDs (D3~D7) as OUTPUT
  DDRD |= 0xf8;
  LedsCheck();

  pinMode(CheckPin1, OUTPUT);
  pinMode(CheckPin2, OUTPUT);

  pinMode(MCP4922Cs, OUTPUT);
  digitalWrite(MCP4922Cs, HIGH);  // set CS as inactive
  pinMode(MCP4922Ldac, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(MCP4922_SPISetting);

  Setup_timer2();

  // disable interrupts to avoid timing distortion
  cbi(TIMSK0, TOIE0);             // disable Timer0 !!! delay() is now not available
  sbi(TIMSK2, TOIE2);             // enable Timer2 Interrupt

#if UART_TRACE
  Serial.begin(9600);
  Serial.println(TITLE_STR1);
  Serial.println(TITLE_STR2);
#endif

  sei();
}

//-------------------------------------------------------------------------------------------------
// Main Loop
//
void loop()
{
  digitalWrite(CheckPin2, HIGH);

  // DDS
  drate = (float)analogRead(PotRate) / 102.3f;
  tword_m = pow(2, 32) * drate / refclk;  // calulate DDS new tuning word

  // Pulse Width
  pulse_width = analogRead(PotPulseWidth);

  // Write to LEDs (D3~D7)
  byte portd_bits = (1 << (waveshape_sel + 3)) | (PORTD & 0x07);
  PORTD = portd_bits;

  digitalWrite(CheckPin2, LOW);

#if UART_TRACE
  Serial.print("waveshape_sel: ");
  Serial.print(waveshape_sel);
  Serial.print("\twaveshape_pushed_wait: ");
  Serial.print(waveshape_pushed_wait);
  Serial.print("\tdrate: ");
  Serial.print(drate);
  Serial.print("\tpulse_width: ");
  Serial.print(pulse_width);
  Serial.print("\tPORTD: ");
  Serial.print(portd_bits, BIN);
  Serial.println("");
#endif

}

