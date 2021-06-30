# Noise-and-dust-monitoring-system-based-on-Lora-and-Bluetooth // Arduino

Design of a near real-time dust and acoustic hazard monitoring system based on wireless data transmission and a portable measurement station.

The proposed solution consists of two subsystems: observation (portable, measuring station) and alerting. 

We get most of the codes from Adafruit examples, some tutorials etc. and stitch them together. BLE beacon is just an advertising code, BLE connected to portable station has its own code which is get through second serial. Both LoRa portable and LoRa connected to the server has their unique addressing. 

The code isn’t clean and perfect, but it’s working so far. If you want to make the system better, you should add the button to switch on the LCD (it should not work constantly, because it’s using too much power). SD card should write the data once in a while, same for SPS30 – it should switch on for 1 measurement per minute every 15 or 30 seconds, but not constantly – it is also using too much power. We also don’t trust the battery check, it should be carefully tested. 

But, the station is reading the data, showing them on the LCD, writing on the SD card and sending them to the LoRa receiver (server) which is displaying all the information on the serial.
