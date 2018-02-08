/*
   Arduino LFO

   2018.01.19

*/
#include <SPI.h>
#include "avr/pgmspace.h"

#include "wavetable_12bit_2k.h"
#define COUNT_OF_ENTRIES  (2048)

#define PIN_CHECK   (1)
#define UART_TRACE  (0)
#define TITLE_STR1  ("Arduino LFO")
#define TITLE_STR2  ("20180208")

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define DEBOUNCE_WAIT (1000)

// Pin Assign
const int PotRate = 0;          // A0
const int PotPulseWidth = 1;    // A1

const int ButtonWaveShape = 2;  // INT0
const int SynIn = 3;            // INT1

const int Led1 = 4;
const int Led2 = 5;
const int Led3 = 6;
const int Led4 = 7;

const int MCP4922Ldac = 9;
const int MCP4922Cs = 10;

#if (PIN_CHECK)
const int CheckPin1 = 18;      // A4
const int CheckPin2 = 19;      // A5
#endif

// MCP4922
SPISettings MCP4922_SPISetting(8000000, MSBFIRST, SPI_MODE0);

// Parameter
double drate = 10.0;                 // initial output rate (Hz)
const double refclk = 15625.0;       // = 16MHz / 8 / 128

enum {
  WS_TRI,
  WS_SQR,
  WS_SAWUP,
  WS_SAWDOWN,
  WS_SIN,
  WS_MAX
};

int waveshape_sel = WS_TRI;           // selected waveshape

int pulse_width = COUNT_OF_ENTRIES / 2;

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
#if (PIN_CHECK)
  digitalWrite(CheckPin1, HIGH);
#endif

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
      if (idx < pulse_width) {
        MCP4922Write(0, 4095);
      } else {
        MCP4922Write(0, 0);
      }
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
      if (waveshape_sel >= WS_MAX) {
        waveshape_sel = 0;
      }
    }
  }
#if (PIN_CHECK)
  digitalWrite(CheckPin1, LOW);
#endif
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
  for (int j = 0; j < 2; j++) {
    for (int i = 4; i <= 7; i++) {
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

  // set LEDs (D4~D7) as OUTPUT
  DDRD |= 0xf0;
  LedsCheck();

#if (PIN_CHECK)
  pinMode(CheckPin1, OUTPUT);
  pinMode(CheckPin2, OUTPUT);
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
#if (PIN_CHECK)
  digitalWrite(CheckPin2, HIGH);
#endif

  // DDS
  drate = (float)analogRead(PotRate) / 10.23f;
  tword_m = pow(2, 32) * drate / refclk;  // calulate DDS new tuning word

  // Pulse Width
  pulse_width = analogRead(PotPulseWidth) * COUNT_OF_ENTRIES / 1024;

  // Write to LEDs (D4~D7)
  byte portd_bits = 0;
  switch (waveshape_sel) {
    case WS_TRI:
    case WS_SQR:
    case WS_SAWUP:
    case WS_SAWDOWN:
      portd_bits = (1 << (waveshape_sel + 4)) | (PORTD & 0x0f);
      break;
    case WS_SIN:
      portd_bits = 0x30 | (PORTD & 0x0f); // LED:xxoo
      break;
  }
  PORTD = portd_bits;

#if (PIN_CHECK)
  digitalWrite(CheckPin2, LOW);
#endif

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

