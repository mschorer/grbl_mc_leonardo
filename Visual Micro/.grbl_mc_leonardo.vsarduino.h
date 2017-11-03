/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Leonardo, Platform=avr, Package=arduino
*/

#define __AVR_ATmega32u4__
#define __AVR_ATmega32U4__
#define USB_VID 0x2341
#define USB_PID 0x8036
#define USB_MANUFACTURER 
#define USB_PRODUCT "\"Arduino Leonardo\""
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

void setup( void);
void loop( void);
void dumpSerial();
int eeprom_get( int ptr, uint8_t* data, int len);
int eeprom_put( int ptr, uint8_t* data, int len);
unsigned long eeprom_crc( uint8_t* data, int len);
void setupUI();
void updateEscMode( bool force);
void updateSMode( bool force);
void updateRpm( bool force);
void updateCoolant( bool force);
void updateTooling( bool force);
void bitString( uint8_t x, uint8_t bm);
void updateFault( bool force);
void updateLimit( bool force);
void updateMessage();
void receiveEvent( int howMany);
void requestEvent();
void clearRpmBuffer();
void setSpindleOn( bool on);
void setSpindleCW( bool cw);
void setSpindlePwm( int rpv);
void setSpindleValue( int rpm);
void setSpindleMode( int mode);
void setSpindle( int mode, int rpm);
void setLaserValue( int val);
void setLaserMode( int mode);
void setLaser( int mode, int val);
void mcToolPrepare( int tool);
void mcToolChange();
void mcToolMode( int mode);
void mcToolValue( int set, int current);
void mcStepValueUp();
void mcStepValueDown();
void mcSpindleControl( int val);
int getCurrentValue();
int transferPID( int set, int current);
int transferLSR( int set, int current);
void setCoolantMist( bool on);
void setCoolantFlood( bool on);
byte gamma_correction(byte input);
byte encoder( byte input);
void turnDecode();

#include "C:\dev\arduino-1.5.7\hardware\arduino\avr\variants\leonardo\pins_arduino.h" 
#include "C:\dev\arduino-1.5.7\hardware\arduino\avr\cores\arduino\arduino.h"
#include "C:\Users\markus\Documents\Arduino\grbl_mc_leonardo\grbl_mc_leonardo.ino"
