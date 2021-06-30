// Feather9x_RX = Receiver = Server 

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define MOBILE_ADDRESS 4
#define RASPBERRY_ADDRESS 2

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 434.0

#define RETRIES 3 // Number of times the sendtoWait() will try to send a message. Default is 3
#define TIMEOUT 1000 // Timeout before sendWait() tries again to send a message 


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(rf95, RASPBERRY_ADDRESS);

// Blinky on receipt
#define LED 13
// #define SERIAL_DEBUG


void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Serial.println("Feather LoRa RX Test!");

  Serial.begin(115200);
  delay(1); {
  if (!manager.init())
    Serial.println("init failed - problem with addressing");
  else {
        Serial.println("Init OK - addressing successful!");
        }
  }
  delay(100);


  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Turning on the Lora radio failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Setting the frequency has failed");
    while (1);
  }
  Serial.println("Setting the frequency to: "); Serial.println(RF95_FREQ); Serial.println("");

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  manager.setRetries(RETRIES);
  manager.setTimeout(TIMEOUT);
}

uint8_t data[] = "Data received!"; 
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];



void loop()
{
  if (manager.available())
  {
    // Wait for a message addressed to us from the client (transmiter) and send an ACKNOWLEDGE if it is received with the right length
    uint8_t len = sizeof(buf);
    uint8_t from;
    
    if (manager.recvfromAck(buf, &len, &from))
    {

      Serial.println("");
      Serial.print("got request from: 0x");
      Serial.print(from, HEX);
      Serial.println("");
      Serial.println("RTC, BME, SPS30, Microphone, SDstatus [0 = missing SD card, 1 = SD card OK], BlueToothLE: ");
      Serial.println("Year/Month/Day/ Hour/Minutes/Seconds/ Temp/Humid/Pressure/ PM1/PM2,5/PM4/PM10/ Mic/ Battery/ SDstatus / "); 
      Serial.print((char*)buf);
      Serial.println("");
      Serial.print("LoRa RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      Serial.println("");
      delay(500); 
          
      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from))
        Serial.println("sendtoWait failed");
      else
        Serial.println("The reply was sent to the mobile TRANSMITTER");
        Serial.println("");
        delay(500); 
    }
    else
      Serial.println("There is no confirmation of addressing from the TRANSMITTER");
      delay(100); 
  }
  else 
    Serial.print(".");
    delay(1000);


}
