# RocketDuino
This is an Arduino Board designed for use in High-Powered model rockets.

(Picture Goes Here)

##Features
Parachute Deployment:  Dual parachute deployment via high-power mosfets to route raw battery current directly through the ignitors.  (Designed for 9v)

GPS Tracking: On-board GPS used to verify hight measurements, get speed readouts, and know the location of the rocket.

Telemetry: LoRa telemetry transmission with a maximum range of over 8km.  This is used to relay location and height data to a ground control station to ease finding the rocket after parachute deployment.

####Hardware
Early versions (which were flown successfully) used a wemos D1 mini.  However, the current version uses a Teensy LC as the main microcontroller.  This change was made primarily to take advantage of the multiple hardware serial ports on the Teensy line.

GPS:  Works with any Serial GPS device that outputs classic NMEA messages, (GPGGA, etc) does NOT currently support the newer GNGGA messages.  The PCB is designed to directly mount a uBlox NEO-M6N module.  (Note that the config needs to be changes to output GPGGA, etc)

Barometer:  A BMP280 is used for more accurate real-time elevation measurement.  Note that the Teensy is NOT 5v compliant, so you need the 3.3v BMP280 board.

Telemetry:  Data Transmission is done using a E32-433T30D 1W LoRa trasmitter, but any data transmitter with a serial interface should work, with only minor changes to the board.  (The PCB is designed to mount the module directly.)

More information coming soon(ish)

##The Board
Short Description coming soon.

[For Detailed PCB Information, click here](https://github.com/1n5aN1aC/RocketDuino/tree/master/PCB)

##The Software
Short Description coming soon.

[For Detailed Arduino Information, click here](https://github.com/1n5aN1aC/RocketDuino/tree/master/Arduino)

##The Results
We haven't actually tested anything in the real world yet... :(
