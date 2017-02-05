# grbl_mc_leonardo
grbl spindle controller for leonardo + spi TFT + knob + rpm sensor

- uses the same i2c commands but provides a 0-100%pwm, direction and enable outputs.
- provides cooling pins (not broken out on pcb yet)
- starting to use eeprom as parameter storage for pwm, pid ... etc parameters

planned:
- simple serial commands to set/read/status commands
