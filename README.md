# grbl_mc_leonardo
grbl spindle/LASER controller for leonardo + spi TFT + knob + rpm sensor

- uses i2c commands for T/S/M<3456> gcode cmds
- provides cooling pins (not broken out on pcb yet)
- uses eeprom as parameter storage for pwm, pid ... etc parameters
- tools #0/1/2 for spindle:

	- PID controlled rpm

	- outputs a pwm signal (100Hz, 4-96% duty cycle) for spindle control

- tool #3 for laser

	- direct pwm

planned:
- simple serial commands to set/read/status commands
