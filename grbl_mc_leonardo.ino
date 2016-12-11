/*
  esc spindle controller for grbl
  by ms@ms-ite.de
  
  - PID controlled rpm
  - controlled via i2c
  - connects to a rotary encoder/switch for speed/manual stop
  - outputs a pwm signal (100Hz, 4-96% duty cycle) for spindle control
  - some spare pins
  - displays on a 160x128 SPI TFT with ST7735S controller
  - shows spindle on/off via TX led
  - shows coolant on/off via RX led
  - blinks L led as heartbeat/PID visualisation
  - uses IDLE sleep mode
*/

#define F_CPU 16000000UL  // 16 MHz

#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <util/twi.h>
//#include <util/delay.h>

#include <Wire.h>

#include <EEPROM.h>

#include <TFT.h>  // Arduino LCD library
#include <SPI.h>

//#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
 
// pin definition for the Leonardo
#define DP_CS   A2  // 20  // A0
#define DP_DC   A0  // 18  // A2
#define DP_RST  A1  // 19  // A1
#define SD_CS   A3  // 21  // A3

#define VERSION  "3.1e"

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

#define SDA 0
#define SCL 2

// switch to TMR3 PWM
#define DIR_SPINDLE_PWM  DDRC
#define PORT_SPINDLE_PWM PORTC
#define PIN_SPINDLE_PWM  PC6

#define DIR_LED_RX    DDRB
#define PORT_LED_RX   PORTB
#define TOGGLE_LED_RX PINB
#define PIN_LED_RX    PB0

#define DIR_LED_TX    DDRD
#define PORT_LED_TX   PORTD
#define TOGGLE_LED_TX PIND
#define PIN_LED_TX    PD5

#define DIR_LED_L    DDRC
#define PORT_LED_L   PORTC
#define TOGGLE_LED_L PINC
#define PIN_LED_L    PC7

#define PINB_UPDN   PIND
#define DIR_UPDN    DDRD
#define PORT_UPDN   PORTD
#define PIN_UP      PD2
#define PIN_DOWN    PD3

#define PINB_MUTE   PINE
#define DIR_MUTE    DDRE
#define PORT_MUTE   PORTE
#define PIN_MUTE    PE6

#define PINB_SPINDLE_CTRL  PINB
#define DIR_SPINDLE_CTRL   DDRB
#define PORT_SPINDLE_CTRL  PORTB

#define PIN_SPINDLE_SENSE   PB4
#define PIN_SPINDLE_DIR    PB6
#define PIN_SPINDLE_EN    PB5

#define DIR_RPM_SENSE   DDRD
#define PORT_RPM_SENSE  PORTD
#define PIN_RPM_SENSE   PD7

#define DIR_BACKLIGHT   DDRD
#define PORT_BACKLIGHT  PORTD
#define PIN_BACKLIGHT   PD6

#define DIR_COOLANT_CTRL   DDRD
#define PORT_COOLANT_CTRL  PORTD
#define PIN_COOLANT_MIST   PD0
#define PIN_COOLANT_FLOOD  PD1

#define DEBOUNCE_TICKS 2

// the timer is set to 16MHz / 256 = 62.500 Hz
// setting freq to 625clks @ 100Hz
//#define SPINDLE_PWM_MIN_ON 25      // 1ms
//#define SPINDLE_PWM_MIN_OFF 600   // 1ms

#define SPINDLE_PWM_PERIOD 1002   // 1ms

#define SPINDLE_PWM_OFF 1        // 0.5%
#define SPINDLE_PWM_SLOW 41
#define SPINDLE_PWM_MAX 1000      // 99.5%

#define BLINK_TICKS_MANUAL 5
#define BLINK_TICKS_MASTER 1

#define NOP 0
#define UP 1
#define DOWN 2

#define MSG_LEN 100
#define MSG_MAX 100

// defined commands using upper nibble
#define CMD_MX  0x00
#define CMD_TX  0x10
#define CMD_MSG 0x70

// values for CMD_MX, using lower nibble
#define CMD_M3  0x03
#define CMD_M4  0x04
#define CMD_M5  0x05

#define CMD_M6  0x06

#define CMD_M7  0x07
#define CMD_M8  0x08
#define CMD_M9  0x09

#define CMD_MSG_TXT   0x00
#define CMD_MSG_LIM   0x01
#define CMD_MSG_FLT   0x02

// spindle modes following Mx commands
#define SPINDLE_CW   3
#define SPINDLE_CCW  4
#define SPINDLE_OFF  5

// coolant bits
#define COOLANT_OFF   0
#define COOLANT_MIST  1
#define COOLANT_FLOOD 2
#define COOLANT_BOTH  3

#define ESC_NC         0
#define ESC_SETUP_MIN  1
#define ESC_SETUP_MAX  2
#define ESC_AUTO       3

#define ESC_TICKS_MIN  16
#define ESC_TICKS_MAX  4

#define RPM_OFF 0
#define RPM_MAX     15000
#define RPM_PWM_SCALE 25    //((RPM_MAX - RPM_OFF) / 120)

#define RPM_MAXHZ     (RPM_MAX / 60)  // 250
#define RPM_PPR       12  // 8 pulses per rotation

// calculate gate frequency
// RPM_MAXHZ * RPM_PPR = 250 * 12 = 3000 pulses per second
// make it fit in 8bit: 3000 / 250 = 12
// choose 16 

#define RPM_HZ        12  // gate frequency in Hz

// choose a reasonable size for the averaging buffer (this one also makes the rpm-multiplier easy to calculate (60 / 6 = 10))
#define RPM_BUFFER    6  // averaging buffer depth 

#define RPM_SCALE     10  //(((RPM_HZ / RPM_PPR) * (60 / RPM_BUFFER)))
#define RPM_TICKS     (15625 / RPM_HZ)            // timer ticks
//-----------------------------------------------------------
/*
enum SequencerState_t {
  PULSE,
  PAUSE
};
*/
volatile byte esc_state = ESC_NC;
volatile byte _esc_state = ESC_NC;
volatile char esc_ticks = 0;

volatile bool ui_update = true;

//volatile byte state_machine = PULSE;
//volatile byte servo_pause = 0;

volatile byte pin_updn = 0xff;
volatile byte pin_mute = 0xff;

volatile int _rpm_value = 4000;
volatile int _rpm_current = 0;

volatile uint16_t sys_ticks;

volatile uint16_t _rpm_avg[RPM_BUFFER];
volatile uint8_t _rpm_avg_idx = 0;
volatile uint16_t _rpm_avg_sum = 0;

volatile int _rpm_pwm = SPINDLE_PWM_OFF;
volatile int d_rpm_pwm = -1;

volatile boolean _motor_manual = false;
volatile boolean d_motor_manual = false;

bool setup_smode = true;
int _sMode = SPINDLE_OFF;
int d_sMode = SPINDLE_OFF;

bool setup_rpm = true;
int d_rpm_value = -1;
int d_rpm_current = -1;
uint16_t d_rpm_color = 0;

volatile uint8_t coolant = 0;
uint8_t d_coolant = 0;

bool setup_tooling = true;
volatile uint8_t tool_index = 0;
uint8_t d_tool_index = 0;
volatile uint8_t tool_current = 0;
uint8_t d_tool_current = 0;

volatile char message[MSG_LEN] = "";
char mbuffer[MSG_LEN] = "";
volatile bool msg_changed = false;

volatile uint8_t msg_flt = 0, _msg_flt = 0;
volatile uint8_t msg_lim = 0, _msg_lim = 0;

char cbuf[64], fbuf[ 16];

#define EEP_SETTINGS_ADDR  0

struct pidParms {
  double consKp, consKi, consKd;
};

struct Settings {
  pidParms pid;
  long crc;
};

Settings settings; 
long setCrc;

double Setpoint, Input, Output;
PID *myPID;

TFT TFTscreen = TFT( DP_CS, DP_DC, DP_RST);

void setup( void) {

  Serial.begin(9600);
  
  Serial.print("grbl_mc ");
  Serial.println( VERSION);

  eeprom_get( EEP_SETTINGS_ADDR, (uint8_t*) &settings, sizeof( Settings));
  
  setCrc = eeprom_crc( (uint8_t*) &settings.pid, sizeof( pidParms));

  if ( setCrc != settings.crc) {
    settings.pid.consKp = 0.1;    // 0.4
    settings.pid.consKi = 0.1;    // 0.25
    settings.pid.consKd = 0.1;    // 0.05
    settings.crc = eeprom_crc( (uint8_t*) &settings.pid, sizeof( pidParms));
    
    eeprom_put( EEP_SETTINGS_ADDR, (uint8_t*) &settings, sizeof( Settings));
//    EEPROM.put( EEP_SETTINGS_ADDR, &settings);
    
    Serial.println( "parms preset");
  } else Serial.println( "parms read");
  
  myPID = new PID(&Input, &Output, &Setpoint, settings.pid.consKp, settings.pid.consKi, settings.pid.consKd, DIRECT);
  
  TFTscreen.begin();
  TFTscreen.setRotation( 3);
  TFTscreen.background( ST7735_BLACK);
  
  TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
  TFTscreen.stroke( ST7735_WHITE);
  TFTscreen.fill( ST7735_BLACK);

  for( uint8_t i=0; i < RPM_BUFFER; i++) {
    _rpm_avg[ i] = 0;
  }

  //----------------------------------------------------------------------------
  // configure pins

  // inputs  
  DIR_UPDN &= ~( (1<< PIN_UP) | (1<< PIN_DOWN));
  PORT_UPDN |= ( (1<< PIN_DOWN) | (1<< PIN_UP));  // enable pullup for inputs

  DIR_MUTE  &= ~( (1<< PIN_MUTE));
  PORT_MUTE |= (1<< PIN_MUTE);  // enable pullup for input
  
  DIR_RPM_SENSE &= ~(1<< PIN_RPM_SENSE);
  PORT_RPM_SENSE |= (1<< PIN_RPM_SENSE);  // enable pullup for input

  // outputs

  DIR_SPINDLE_CTRL &= ~(1<< PIN_SPINDLE_SENSE);                    // IN: spindle sense
  DIR_SPINDLE_CTRL |= (1<< PIN_SPINDLE_DIR | 1<< PIN_SPINDLE_EN);  // OUT: EN and DIR
//  PORT_SPINDLE_CTRL &= ~(1<< PIN_SPINDLE_SENSE);  // enable pullup for input
  
  DIR_BACKLIGHT |= (1<< PIN_BACKLIGHT);
  PORT_BACKLIGHT &= ~(1<< PIN_BACKLIGHT);  // enable pullup for input

  DIR_COOLANT_CTRL |= ((1<< PIN_COOLANT_MIST) | (1<< PIN_COOLANT_FLOOD));
  PORT_COOLANT_CTRL &= ~((1<< PIN_COOLANT_MIST) | (1<< PIN_COOLANT_FLOOD));  // enable pullup for input

  // switch to PWM TMR3A, dir needs to be set "out"
  DIR_SPINDLE_PWM |= (1<< PIN_SPINDLE_PWM);      // set PE6 as output
//  PORT_SPINDLE_PWM &= ~(1<< PIN_SPINDLE_PWM);    // preset to 0

//  PORT_LED_TX &= ~(1<< PIN_LED_TX);    // preset to 0
  DIR_LED_L |= (1<< PIN_LED_L);
//  PORT_LED_L |= (1<< PIN_LED_L);
//  PORT_LED_L &= !(1<< PIN_LED_L);
    
  DIR_LED_RX |= (1<< PIN_LED_RX);
  PORT_LED_RX |= (1<< PIN_LED_RX);

  DIR_LED_TX |= (1<< PIN_LED_TX);      // set Pc7 as output
  PORT_LED_TX |= (1<< PIN_LED_TX);

  // set spares to input
//  DDRC &= ~(1<< PC7);                // high
  DDRF &= ~((1<< PF1) || (1<< PF0));                // high
  DDRD &= ~((1<< PD4) || (1<< PD1) || (1<< PD0)); // high

  // enable pin-change interrupts for up/down/mute sources
  EIMSK = 0;
  EICRA = ((0<< ISC31) | (1<< ISC30) | (0<< ISC21) | (1<< ISC20)) | ( EICRA & 0x0f);
  EICRB = (0<< ISC61) | (1<< ISC60);
  EIMSK |= (1<< INT6) | (1<< INT3) | (1<< INT2);
  
  PCMSK0 |= (1<< PCINT4);  // enable PCINT4 on PB4
  PCICR |= (1<< PCIE0);   // enable pin change interrupts
  
  //----------------------------------------------------------------------------
  // setup time/counters
  
  // setup prescalers
  GTCCR = (0<< TSM) | (0<< PSRSYNC);

  // setup counter for rpm mesuring
  // disconnect oc1a/oc1b, external clock, normal mode, no interrupts
  TCCR0A = 0x00;  //(0<< COM0A1) | (0<< COM0A0) | (0<< COM0B1) | (0<< COM0B0) | (0<< WGM01) | (0<< WGM00);
  TCCR0B = 0x07;  //(0<< FOC0A) | (0<< FOC0B) | (0<< WGM02) | (1<< CS02) | ( 1<< CS01) | (1<< CS00);
  TCNT0 = 0;
  OCR0A = 0;     // not used
  OCR0B = 0;    // not used
  TIMSK0 = 0x01;  //(0<< OCIE0B) | (0<< OCIE0A) | (0<< TOIE0);

  // setup timer for rpm measuring gate
  // disconnect pins, 1024x prescaler, ctc mode, interrupt on ocra match
  TCCR1A = 0x00;  //(0<< COM1A1) | (0<< COM1A0) | (0<<COM1B1) | (0<< COM1B0) | (0<<COM1C1) | (0<< COM1C0) | ( 0<< WGM11) | (0<< WGM10);
  TCCR1B = 0x0d;  //(0<< ICNC1) | (0<< ICES1) | (0<< WGM13) | (1<< WGM12) | (1<< CS12) | ( 0<< CS11) | (0<< CS10);
  TCCR1C = 0x00;  //(0<< FOC1A) | (0<< FOC1B) | (0<< FOC1C);
  TCNT1 = 0;
  OCR1A = RPM_TICKS;    //2604;   // set for 5Hz
  OCR1B = 0;    // not used
  OCR1C = 0;  // 16Mhz / 1024 / 20Hz 
  TIMSK1 = 0x02;  //(0<< ICIE1) | (0<< OCIE1C) | (0<< OCIE1B) | (1<< OCIE1A) | (0<< TOIE1);
  
  // setup timer for spindle PWM generation
  // clear oc3a on match/ set on reset, 64x prescaler, fast pwm ICR3-max OCR3-compare mode, int off [interrupt on ocra match
  TCCR3A = 0x82;  //(1<< COM3A1) | (0<< COM3A0) | (0<<COM3B1) | (0<< COM3B0) | (0<<COM3C1) | (0<< COM3C0) | ( 1<< WGM31) | (0<< WGM30);
  TCCR3B = 0x1a;  //(0<< ICNC3)  | (0<< ICES3)  |           0 | (1<< WGM33)  | (1<< WGM32) | (0<< CS32)   | ( 1<< CS31)  | (0<< CS30);
  TCCR3C = 0x00;  //(0<< FOC3A);
  TCNT3 = 0;
  ICR3 = SPINDLE_PWM_PERIOD;      // freqency register
  OCR3A = SPINDLE_PWM_OFF;   // compare register 2604;   // set for 5Hz
  OCR3B = 0;    // not used
  OCR3C = 0;
  TIMSK3 = 0x00;  // 0 | 0 | (0<< ICIE3) | 0 | (0<< OCIE3C) | (0<< OCIE3B) | (0<< OCIE3A) | (0<< TOIE3);

  // Init the internal PLL
  PLLFRQ = (0<< PINMUX) | (0<< PLLUSB) | (1<< PLLTM1) | (1<< PLLTM0) | (0<< PDIV3) | (1<< PDIV2) | (0<< PDIV1) | (0<< PDIV0);
  PLLCSR |= _BV(PLLE);
  while(!(PLLCSR & _BV(PLOCK)));
   
  // setup pwm mode for led
  // disconnect oc1a/oc1b, external clock, normal mode, no interrupts
  TC4H  = 0x00;
  TCCR4A = 0x82;  //(1<< COM4A1) | (0<< COM4A0) | (0<< COM4B1) | (0<< COM4B0) | (0<< FOC4A) | (0<< FOC4B) |(1<< PWM4A) | (0<< PWM4B);
  TCCR4B = 0x08;  //(0<< PWM4X) | (0<< PSR4) | (0<< DTPS40) | (0<< DTPS41) | (1<< CS43) | (0<< CS42) | (0<< CS41) | (0<< CS40);
  TC4H  = 0x03;
//  TCCR4C = 0x00;  //(0<< COM4A1S) | (0<< COM4A0S) | (0<< COM4B1S) | (0<< COM4B0S) | (0<< COM4D1) | (0<< COM4D0) | (0<< FOC4D) | (0<< PWM4D);
  TC4H  = 0x00;
  TCCR4D = 0x01;  //(0<< FPIE4) | (0<< FPEN4) | (0<< FPNC4) | (0<< FPES4) | (0<< FPAC4) | (0<< FPF4) | (0<< WGM41) | (1<< WGM40);
  TCCR4E = 0x00;  //(0<< TLOCK4) | (0<< ENHC4) | (0<< OC4OE5) | (0<< OC4OE4) | (0<< OC4OE3) | (0<< OE4OC2) | (0<< OE4OC1) | (0<< OE4OC0);
//  TCNT4 = 0;
  OCR4A = 0x80;     // pwm freq
  OCR4B = 0x0;    // not used
  TC4H  = 0x03;
  OCR4C = 0xff;     // top
  TC4H  = 0x00;
  OCR4D = 0;    // not used
  TIMSK4 = 0x00;  //(0<< OCIE4d) | (0<< OCIE4A) | (0<< OCIE4B) | (0<< TOIE4);

  //----------------------------------------------------------------------------
  // bring up services
  
  Wire.begin( 0x5c);            // join i2c bus with address $5c for SpeedControl
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  Setpoint = (double) _rpm_value;
  myPID->SetOutputLimits( 0, SPINDLE_PWM_MAX);
//  myPID.SetSampleTime( 160);
  myPID->SetMode(AUTOMATIC);

  esc_state = ( PINB_SPINDLE_CTRL & (1<< PIN_SPINDLE_SENSE)) ? ESC_AUTO : ESC_NC;
}

void loop( void) { 
  setupUI();

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  
  // now loop
  setSpindlePwm( SPINDLE_PWM_OFF);
  uint16_t tock = 0;

  while( true) {
//    setLED( ( tock & 0x01) ? 0x3ff : (_rpm_pwm >> 1), _motor_manual, _motor_manual);

    updateSMode( ui_update);
    updateRpm( ui_update);
    updateEscMode( ui_update);
    updateCoolant( ui_update);
    updateTooling( ui_update);
    updateFault( ui_update);
    updateLimit( ui_update);
    if ( msg_changed) updateMessage();

    // do busy waiting, using arduino delay/millis etc will block timer1
    // tock frquency is 6Hz
    if ( sys_ticks >= 6) {
      sys_ticks = 0;
      Serial.print( "M");
      Serial.print( _sMode);
      Serial.print( " S");
      Serial.print( _rpm_value);
      Serial.print( "/");
      Serial.print( _rpm_current);
      Serial.print( "/");
      Serial.print( _rpm_pwm);
      Serial.println( " tgt/cur/pwm");
    }

    while( sys_ticks < 6 && !ui_update) {
      sleep_cpu();
    }
    ui_update = false;
    tock++;
    
//    TOGGLE_LED_L |= (1<< PIN_LED_L);
  }
}

void dumpSerial() {
  Serial.print("grbl_mc ");
  Serial.println( VERSION);
  
  Serial.print( "parms crc [");
  Serial.print( setCrc);
  Serial.print( "_");
  Serial.print( settings.crc);
  Serial.println( "]");
  
  Serial.print( "parms [");
  Serial.print( settings.pid.consKp);
  Serial.print( "_");
  Serial.print( settings.pid.consKi);
  Serial.print( "_");
  Serial.print( settings.pid.consKd);
  Serial.println( "]");
}
  
//----------------------------------------------------------------------------
// eeprom

int eeprom_get( int ptr, uint8_t* data, int len) {
  int i;
  for( i=0; i < len; i++) {
    *data = EEPROM.read( ptr);
    ptr++;
    data++;
  }
  
  return i;
}
  
int eeprom_put( int ptr, uint8_t* data, int len) {
  int i;
  for( i=0; i < len; i++) {
    EEPROM.write( ptr, *data);
    ptr++;
    data++;
  }
  
  return i;
}
  
unsigned long eeprom_crc( uint8_t* data, int len) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0; index < len; ++index) {
    crc = crc_table[(crc ^ data[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (data[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

//----------------------------------------------------------------------------
// handle ui

void setupUI() {
  TFTscreen.setTextColor( ST7735_YELLOW, ST7735_BLACK);

  // rpmDisplay
  TFTscreen.setTextSize(4);
  TFTscreen.setCursor( 40, 2);
  TFTscreen.print( "-RPM-");

  TFTscreen.setTextSize(1);
  TFTscreen.setCursor( 0, 48);
  TFTscreen.print( "RPM");
  TFTscreen.setTextSize(2);
  TFTscreen.setCursor( 40, 48);
  TFTscreen.print( "-----");

  // spindleMode
  TFTscreen.setTextSize(4);
  TFTscreen.setCursor( 2, 2);
  TFTscreen.print( "-");

  // servo bar  
  TFTscreen.noFill();
  TFTscreen.rect( 2, 35, 156, 8, 0);

  // coolant
  TFTscreen.setTextSize(1);
  TFTscreen.setCursor( 0, 68);
  TFTscreen.print( "Coolant");

  // tlabelDisplay
  TFTscreen.setTextSize(1);
  TFTscreen.setCursor( 122, 48);
  TFTscreen.print( "->#");
  TFTscreen.setCursor( 110, 68);
  TFTscreen.print( "Tool#");

  // toolDisplay
  TFTscreen.setTextSize(3);
  TFTscreen.setCursor( 142, 70);
  TFTscreen.print( "T");

  // limits & faults
  TFTscreen.setTextSize(1);
  TFTscreen.setCursor( 88, 95);
  TFTscreen.print( "FLT [------]");
  TFTscreen.setCursor( 0, 95);
  TFTscreen.print( "[------] LIM");

  // msgDisplay
  TFTscreen.drawFastHLine( 0, 103, 160, ST7735_WHITE);
  
  TFTscreen.setTextSize(2);
  TFTscreen.setCursor( 0, 106);
  TFTscreen.print( "gCtrl");
  TFTscreen.setTextSize(1);
  sprintf( cbuf, "%s", VERSION);
  TFTscreen.setCursor( 136, 120);
  TFTscreen.print( cbuf);
  
  TFTscreen.setCursor( 0, 120);
  TFTscreen.print( "crc ");
  TFTscreen.print(( setCrc == settings.crc) ? "OK" : "ERR");
//  TFTscreen.print( settings.crc, HEX);
//  TFTscreen.print( "-");
//  TFTscreen.print( sizeof(pidParms), HEX);
  
  updateFault( true);
  updateLimit( true);
}

void updateEscMode( bool force) {
  if ( force || (esc_state != _esc_state)) {
    TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
    TFTscreen.setTextSize(1);
    TFTscreen.setCursor( 0, 48);
    switch( esc_state) {
      case ESC_NC: TFTscreen.print( "---"); break;
      case ESC_SETUP_MIN: TFTscreen.print( "min"); break;
      case ESC_SETUP_MAX: TFTscreen.print( "max"); break;
      case ESC_AUTO: TFTscreen.print( "RPM"); break;
    }
    _esc_state = esc_state;
  }
}

void updateSMode( bool force) {
  if ( _sMode != d_sMode || d_motor_manual != _motor_manual || force) {
//    TFTscreen.fill( 0,0,0);
//    TFTscreen.rect( 0,16,18,24);
    TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
    TFTscreen.setTextSize(4);
    TFTscreen.setCursor( 2, 2);
    if ( _motor_manual) {
      switch( _sMode) {
        case SPINDLE_CW: TFTscreen.print( "r"); break;
        case SPINDLE_CCW: TFTscreen.print( "l"); break;
        case SPINDLE_OFF:
        default: TFTscreen.print( "M"); break;
      }
    } else {
      switch( _sMode) {
        case SPINDLE_CW: TFTscreen.print( "R"); break;
        case SPINDLE_CCW: TFTscreen.print( "L"); break;
        case SPINDLE_OFF:
        default: TFTscreen.print( "-"); break;
      }
    }
    d_sMode = _sMode;
    d_motor_manual = _motor_manual;
  }
}

void updateRpm( bool force) {
  if ( d_rpm_pwm != _rpm_pwm || force) {  
    d_rpm_pwm = _rpm_pwm;
    int bar = max( 0, min( 154, round( _rpm_pwm / ( SPINDLE_PWM_MAX / 154))));
    TFTscreen.noStroke();
    TFTscreen.fill( ST7735_GREEN);
    TFTscreen.rect( 3, 36, bar, 6);
    if ( bar < 153) {
      TFTscreen.fill( ST7735_BLACK);
      TFTscreen.rect( bar+4, 36, 153-bar, 6);
    }
  }

  if ( d_rpm_value != _rpm_value || force) {  
    if ( _rpm_value) sprintf( cbuf, "%05d", _rpm_value);  //String( _rpm_value).toCharArray( cbuf, 5);  //GLCD.Printf("%05d", _rpm_value);
    else sprintf( cbuf, "-off-");
//    TFTscreen.fill( 0,0,0);
//    TFTscreen.rect( 0,0,30,8);
    TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
    TFTscreen.setTextSize(2);
    TFTscreen.setCursor(40, 48);
    TFTscreen.print( cbuf);
    d_rpm_value = _rpm_value;
  }
  
  int rpm_diff = _rpm_value - _rpm_current;
  uint16_t color;

  if ( abs( rpm_diff) > (_rpm_value / 8)) {
    if ( rpm_diff > 0) color = ST7735_BLUE;
    else color = ST7735_CYAN;
  } else color = ST7735_GREEN;

  if ( d_rpm_current != _rpm_current || d_rpm_color != color || force) {
    if ( _rpm_current < 20000) sprintf( cbuf, "%05d", _rpm_current);  //rpmDisplay.Printf("%05d", _rpm_current);

    TFTscreen.setTextColor( color, ST7735_BLACK);
  
    TFTscreen.setTextSize(4);
    TFTscreen.setCursor(40, 2);
    TFTscreen.print( cbuf);
    d_rpm_current = _rpm_current;
    d_rpm_color = color;
    TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
  }
}

void updateCoolant( bool force) {
  if ( coolant != d_coolant || force) {
    TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
    switch( coolant) {
      case COOLANT_BOTH: sprintf( cbuf, "[MST/FLD]"); break;
      case COOLANT_FLOOD: sprintf( cbuf, "[---/FLD]"); break;
      case COOLANT_MIST: sprintf( cbuf, "[MST/---]"); break;
      case COOLANT_OFF:
      default: sprintf( cbuf, "[---/---]"); break;
    }
//    TFTscreen.fill( 0,0,0);
//    TFTscreen.rect( 0,68,108,16);
    TFTscreen.setTextSize(2);
    TFTscreen.setCursor( 0, 78);
    TFTscreen.print( cbuf);
    d_coolant = coolant;
  }
}

void updateTooling( bool force) {
  TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
  if ( tool_index != d_tool_index || force) {
//    TFTscreen.fill( 0,0,0);
//    TFTscreen.rect( 120,60,6,8);
    TFTscreen.setTextSize(2);
    sprintf( cbuf, "%i", tool_index);
    TFTscreen.setCursor( 148, 48);
    TFTscreen.print( cbuf);
    d_tool_index = tool_index;
  }
  if ( tool_current != d_tool_current || force) {
//    TFTscreen.fill( 0,0,0);
//    TFTscreen.rect( 140,60,18,24);
    TFTscreen.setTextSize(3);
    sprintf( cbuf, "%i", tool_current);
    TFTscreen.setCursor( 142, 70);
    TFTscreen.print( cbuf);
    d_tool_current = tool_current;
  }
}

void bitString( uint8_t x, uint8_t bm) {
  char axs[7] = "XYZUVW";
  char text[2] = "@";
  uint8_t i, bt = 0x01;

  TFTscreen.setTextSize(1);
  
  for( i=0; i < 6; i++) {
    TFTscreen.setCursor( x, 95);
    if ( bm & bt) {
      TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
      text[ 0] = '-';
    } else {
      TFTscreen.setTextColor( ST7735_BLACK, ST7735_BLUE);
      text[ 0] = axs[ i];
    }
    TFTscreen.print( text);
    bt <<= 1;
    x += 6;
  }
}

void updateFault( bool force) {
  if ( _msg_flt != msg_flt || force) {
    bitString( 118, _msg_flt);
    msg_flt = _msg_flt;
  }
}

void updateLimit( bool force) {
  if ( _msg_lim != msg_lim || force) {
    bitString( 5, _msg_lim);
    msg_lim = _msg_lim;
  }
}

void updateMessage() {
  uint8_t i = 0;
  uint8_t ln = 106;
  
//  TFTscreen.noStroke();
  TFTscreen.fill( ST7735_BLACK);
  TFTscreen.rect( 0, 106, 159, 119);
  
  TFTscreen.setTextColor( ST7735_WHITE, ST7735_BLACK);
  TFTscreen.setTextSize(1);
  String line;
    
  while( i < MSG_MAX) {

    switch( message[i]) {
      case 0:
      case 10:
      case 13:
        if ( line.length()) {
          line.toCharArray( cbuf, 40);
          TFTscreen.setCursor( 0, ln);
          TFTscreen.print( cbuf);
          ln += 8;
          line = "";
        }
        if ( message[i] == 0) i = MSG_MAX;
      break;
    
      default:
        line += message[ i];
    }
    i++;
  }

  msg_changed = false;  
}

//----------------------------------------------------------------------------
// handle i2c protocol
  
void receiveEvent( int howMany) {
  int rpm = 0; // receive byte as a character
  uint8_t c = 0;

  while ( howMany > 0) {    
    
    c = Wire.read();
    howMany--;
    
    if ( c & 0x80) {
      // S-command
      rpm = ( c & 0x7f) << 8;
      if ( howMany > 0) {
        c = Wire.read();
        rpm |= c;
        howMany--;
      }
  
      if ( rpm > RPM_MAX) _rpm_value = RPM_MAX;
      else if ( rpm < RPM_OFF) _rpm_value = RPM_OFF;
      else _rpm_value = rpm;
      
      Setpoint = (double) _rpm_value;
    } else {
      uint8_t par = c & 0x0f;
      switch( c & 0x70) {
        // Mx commands
        case CMD_MX:
          switch( par) {
            // spindle control M3/4/5
            case CMD_M3:
              _sMode = par;
//              Setpoint = (double) _rpm_value;

              PORT_LED_TX &= !(1<< PIN_LED_TX);
              _motor_manual = false;
              setSpindleCW( true);
              setSpindleOn( true);
            break;
            case CMD_M4:
              _sMode = par;
//              Setpoint = (double) _rpm_value;

              PORT_LED_TX &= !(1<< PIN_LED_TX);
              _motor_manual = false;
              setSpindleCW( false);
              setSpindleOn( true);
            break;
            case CMD_M5: 
              _sMode = par; 
//              Setpoint = 0;
              PORT_LED_TX |= (1<< PIN_LED_TX);
              setSpindleOn( false);
            break;
            
            // tool change: M6
            case CMD_M6: tool_current = tool_index;
            break;
            
            // coolant control M7/8/9
            case CMD_M7: coolant |= COOLANT_MIST;
              PORT_LED_RX &= !(1<< PIN_LED_RX);
              setCoolantMist( true);
            break;
            case CMD_M8: coolant |= COOLANT_FLOOD;
              PORT_LED_RX &= !(1<< PIN_LED_RX);
              setCoolantFlood( true);
            break;
            case CMD_M9: coolant = COOLANT_OFF;
              PORT_LED_RX |= (1<< PIN_LED_RX);
              setCoolantMist( false);
              setCoolantFlood( false);
            break;
          }
          break;

        // Tx command          
        case CMD_TX:
          tool_index = par;
          break;
        
        // send message command
        case CMD_MSG:
          switch( par) {
            case CMD_MSG_LIM:
                if ( howMany > 0) {
                  _msg_lim = Wire.read();
                  howMany--;
                }
            break;
            
            case CMD_MSG_FLT:
                if ( howMany > 0) {
                  _msg_flt = Wire.read();
                  howMany--;
                }
            break;
            
            case CMD_MSG_TXT:
              byte msgidx = 0;
              while( howMany > 0 && msgidx < MSG_MAX) {
                c = Wire.read();
                message[ msgidx++] = c;
                howMany--;
              }
              message[ msgidx] = 0;
              msg_changed = true;
        }
      }
    }
  }
  
  ui_update = true;
}

void requestEvent() {
  Wire.write( _rpm_value);
}

void setSpindleOn( bool on) {
  if ( on) PORT_SPINDLE_CTRL |= (1 << PIN_SPINDLE_EN);
  else PORT_SPINDLE_CTRL &= ~(1 << PIN_SPINDLE_EN);
}

void setSpindleCW( bool cw) {
  if ( cw) PORT_SPINDLE_CTRL |= (1 << PIN_SPINDLE_DIR);
  else PORT_SPINDLE_CTRL &= ~(1 << PIN_SPINDLE_DIR);
}

void setSpindlePwm( int rpv) {
  OCR3A = _rpm_pwm = rpv;
}

void setCoolantMist( bool on) {
  if ( on) PORT_COOLANT_CTRL |= (1 << PIN_COOLANT_MIST);
  else PORT_COOLANT_CTRL &= ~(1 << PIN_COOLANT_MIST);
}

void setCoolantFlood( bool on) {
  if ( on) PORT_COOLANT_CTRL |= (1 << PIN_COOLANT_FLOOD);
  else PORT_COOLANT_CTRL &= ~(1 << PIN_COOLANT_FLOOD);
}

//----------------------------------------------------------------------------
// helpers
  
byte gamma_correction(byte input) {
  unsigned int multiplied = input * input;
  return multiplied / 256;
}

byte encoder( byte input) {
  byte res = 0;
  
  if ( input & ( 1<< PIN_UP)) res |= 0x02;
  if ( input & ( 1<< PIN_DOWN)) res |= 0x01;
  
  return res;
}

//----------------------------------------------------------------------------
// interrupt handler for esc pwr sense

ISR( PCINT0_vect) {
  byte pwr_sense = PINB_SPINDLE_CTRL & (1<< PIN_SPINDLE_SENSE);
  
  if ( pwr_sense) {
    esc_ticks = 0;
    esc_state = ESC_SETUP_MIN;
  } else {
    esc_state = ESC_NC;
  } 
  ui_update = true;
}
  
//----------------------------------------------------------------------------
// interrupt handlers for buttons
// level change interrupt on 

ISR( INT2_vect) {
  turnDecode();
}

ISR( INT3_vect) {
  turnDecode();
}
  
void turnDecode() {
  
  //      00 01 10 11
  //  00  .  -  +  .
  //  01  +  .  .  -
  //  11  .  +  -  .
  //  10  -  .  .  +
  
  const byte enc_states[] = { NOP, DOWN, UP, NOP, 
                              UP, NOP, NOP, DOWN, 
                              DOWN, NOP, NOP, UP, 
                              NOP, UP, DOWN, NOP};

  static byte encoder_pos = 0;
  
  byte pin_dir = PINB_UPDN & ((1<< PIN_UP) | (1<< PIN_DOWN));
  
  // should always be true as we're triggered only on edges
  if ( pin_dir ^ pin_updn) {
    pin_updn = pin_dir;

    byte enc = encoder( pin_dir);
    
    if ( enc != encoder_pos) {
      enc |= encoder_pos << 2;
      
      switch ( enc_states[ enc]) {
        case UP:
          if ( _rpm_value > RPM_PWM_SCALE) _rpm_value -= RPM_PWM_SCALE;
          else _rpm_value = RPM_OFF;
          Setpoint = (double) _rpm_value;
  
          ui_update = true;
        break;
        
        case DOWN:
          _rpm_value += RPM_PWM_SCALE;
          if ( _rpm_value > RPM_MAX) _rpm_value = RPM_MAX;
          Setpoint = (double) _rpm_value;
  
          ui_update = true;
        break;
        
        default:
          ;
      }
      encoder_pos = enc & 0x03;
    }
  }
}

// level change interrupt on 
ISR( INT6_vect) {
  
  byte mute = PINB_MUTE & (1<< PIN_MUTE);
  
  // should always be true as we're triggered only on edges
  if ( mute ^ pin_mute) {
    pin_mute = mute;

    if (( mute & (1<< PIN_MUTE)) == LOW) {
      _motor_manual = !_motor_manual;
    }
    setSpindleOn( _motor_manual);
    
    ui_update = true;
  }
}

//----------------------------------------------------------------------------
// rpm counter overflow

ISR( TIMER0_OVF) {
  ;
}

//----------------------------------------------------------------------------
// rpm counter gate interrupt
  
// rpm measurement gate clock
ISR(TIMER1_COMPA_vect) {
  setLED( ( sys_ticks & 0x01) ? 0x3ff : (_rpm_pwm >> 1), _motor_manual, _motor_manual);
  
  sys_ticks++;
  
  uint16_t rotations = (uint16_t) TCNT0;

  _rpm_avg_sum -= _rpm_avg[ _rpm_avg_idx];
  _rpm_avg_sum += rotations;

  _rpm_avg[ _rpm_avg_idx] = rotations;
  _rpm_avg_idx++;
  if ( _rpm_avg_idx >= RPM_BUFFER) _rpm_avg_idx = 0;
  
  uint16_t last_rpm = _rpm_current;
  _rpm_current = _rpm_avg_sum * RPM_SCALE;  // * RPM_HZ * 60 / RPM_BUFFER
  TCNT0 = 0;
  
  if ( _rpm_current != last_rpm) ui_update = true;

  switch( esc_state) {
    case ESC_SETUP_MIN:
      if ( ++esc_ticks >= ESC_TICKS_MIN) {
        esc_ticks = 0;
        esc_state = ESC_SETUP_MAX;
        setSpindlePwm( SPINDLE_PWM_MAX);
        ui_update = true;
      }
    break;
    case ESC_SETUP_MAX:
      if ( ++esc_ticks >= ESC_TICKS_MAX) {
        esc_ticks = 0;
        esc_state = ESC_AUTO;
        ui_update = true;
      } else break;

    case ESC_AUTO:
      if (_motor_manual ^ (_sMode != SPINDLE_OFF)) {
        Input = (double) _rpm_current;
        myPID->Compute();
        setSpindlePwm( (int) Output);
      } else {
        setSpindlePwm( SPINDLE_PWM_OFF);
      }
    break;
    case ESC_NC:
    default:
      setSpindlePwm( SPINDLE_PWM_OFF);
      ui_update = true;
  }
}

//----------------------------------------------------------------------------
// pwm timer
  
// timer interrupt for pwm signal generation
/*
ISR(TIMER3_COMPA_vect) {
    switch( state_machine) {
      case PULSE:
        OCR1A = SPINDLE_PWM_MIN_ON + _rpm_pwm;
        PORT_SPINDLE_PWM |= (1 << PIN_SPINDLE_PWM);
        state_machine = PAUSE;
      break;

      case PAUSE:
      default:
        OCR1A = SPINDLE_PWM_MIN_OFF - _rpm_pwm;
        PORT_SPINDLE_PWM &= ~( 1 << PIN_SPINDLE_PWM);

        state_machine = PULSE;  
    }
}//end ISR TIM0_COMPA_vect
*/

void setLED( int val, boolean doBlink, boolean freq) {

  TC4H  = (val >> 9) & 0x03;
  OCR4A = ( val >> 1) & 0xff;

  /*
  if ( doBlink) {
    if ( _blinkCount <= 0) {
      _blinkPhase = ! _blinkPhase;
      _blinkCount = freq ? BLINK_TICKS_MANUAL : BLINK_TICKS_MASTER;
    } else _blinkCount--;
    
    OCR0A = _blinkPhase ? gamma_correction( val) : 0;
  } else {
    OCR0A = gamma_correction( val);
  }
  */
}

