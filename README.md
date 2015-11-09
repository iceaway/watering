Automatic plant watering
========================

This project aims to create an automatic plant watering system using:
- Arduino UNO
- Soil moisture sensor
- Relay module
- Car washer fluid pump

But mostly its for fun and self educational purposes.

Lots of things remains to do:
- ~~Control algorithm for activating relay~~
- Calibrate and fine tune control algorithm
- Filter the ADC signal as it is quite noisy
- ~~Add control parameters as environment variables~~
- Save environment to EEPROM
- ~~Create some kind of logging function to record soil moisture over time
  as a help for calibrating the control algorithm~~
- Clean up environment periodically to remove old entries
- Add reset function (WDT enable)
- Move code from main.c to separate files
