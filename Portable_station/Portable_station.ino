// Feather9x_TX = Transmiter = Client = Mobile
// BME + LoRa + LCD + Sensirion sps30 + Mic + SDstatus + BLE 
// based on open libraries and examples, compiled by Przemyslaw Oberbek and Szymon Jakubiak, oberbek@gmail.com; szjak@ciop.pl

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "DFRobot_RGBLCD.h" 

#include <RHReliableDatagram.h> 
#include <RH_RF95.h>
#include <SPI.h>

#include <sps30.h>

#include "RTClib.h"
#include <SD.h>


#define MOBILE_ADDRESS 4 // for specific addressing
#define RASPBERRY_ADDRESS 2 // for specific addressing

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 434.0

#define RETRIES 3 // Number of times the sendtoWait() will try to send a message. Default is 3
#define TIMEOUT 1000 // Timeout before sendWait() tries again to send a message 

#define VBATPIN A7


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// RH_RF95 driver;
// Class to manage message delivery and receipt, using the driver declared above 
RHReliableDatagram manager(rf95, MOBILE_ADDRESS); 

Adafruit_BME280 bme; 

// Interval between consecutive measurements (in miliseconds) - for sps30
const long interval = 30000;
// Variable to store time when last measurement was done - for sps30
unsigned long previousMillis = 0;

unsigned long delayTime;

int r,g,b; 
int t=0; 
DFRobot_RGBLCD lcd(16,2);

RTC_PCF8523 rtc;


// char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}; // for English users if needed
// char daysOfTheWeek[7][12] = {"Niedziela", "Poniedzial", "Wtorek", "Sroda", "Czwartek", "Piatek", "Sobota"}; // for Polish users if needed

// averaging microphone data
int MicMeasurements = 0;
unsigned long total = 0;
int Mic = 0;
int averagedMic = 0;
const int MicrophoneSensor_pin = A1;
const int measurement_interval = 10;
float ADCtomV=1500.0/1023.0; // The ADC value must be converted to mV 

// reading the battery status
float measuredvbat = analogRead(VBATPIN);


// Recording on a memory card -> Set the pins used
int SDstatus = 1;
#define cardSelect 10
String file_name = "";

void setup() 
{
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  pinMode(8, OUTPUT); // <-- important to switch off the LoRa for the SD card to work


  
  // sps30 
  int16_t ret; 
  uint8_t auto_clean_days = 4; 
  uint32_t auto_clean; 


  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial1.begin(9600); // <- serial for Bluetootha in the main device

  Serial.begin(115200); { 
  //while (!Serial) ; {   Wait for serial port to be available 
  delay(1); {

   if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } 
  rtc.start();
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   <- uncomment if you want the same time as on PC. 
  // Co do powyższego: This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  DateTime now = rtc.now();
  char date_buf[] = "MMDDhhmm";
  file_name += now.toString(date_buf);
  file_name += ".txt";

  
  lcd.init();

  sensirion_i2c_init();
  while (sps30_probe() != 0) { 
    Serial.print("sps30 sensor probing failed\n"); 
    delay(500); 
  }

  ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
  if (ret) {
    Serial.print("error setting sps30 auto-clean interval: ");
    Serial.println(ret);
  }

  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("error starting sps30 measurement\n");
  }

  delay(100); 


  
  if (!manager.init())
    Serial.println("LoRa addressing problem - Init failed!");
  else {
    Serial.println("LoRa addressing correct - Init OK!");
    }
  }
  delay(100);


  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  unsigned status; 
  status = bme.begin(0x76, &Wire);
  if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
 
  while (!rf95.init()) {
    Serial.println("LoRa radio - init failed!");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio - init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ); Serial.println("");
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  
  manager.setRetries(RETRIES);
  manager.setTimeout(TIMEOUT); 
  }
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

// Dont put this on the stack: 
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; 




void loop() {

  // reading the battery status
  Serial.println();
  Serial.println("battery status:"); Serial.print(measuredvbat); Serial.print(" V,");
  if(VBATPIN > 0 && VBATPIN < 1){
    Serial.println(" 25%");
  }else if(VBATPIN > 1 && VBATPIN < 2.775){
    Serial.println(" 50%");
  }else if(VBATPIN > 2.775 && VBATPIN < 3.7){
    Serial.println(" 75%");
  }else{
    Serial.println(" 100%");
  }
  int16_t Batt = (int16_t)(measuredvbat);


  DateTime time = rtc.now();
  DateTime now = rtc.now();
  struct sps30_measurement m; 
  char serial[SPS30_MAX_SERIAL_LEN];
  uint16_t data_ready;
  int16_t ret;

  // Read and count average from 100 microphone measurement results
  while (MicMeasurements < 100)
  {
    Mic = analogRead(MicrophoneSensor_pin);
    
    total = total + Mic;
    MicMeasurements++;
    delay(measurement_interval);
  }

  averagedMic = (total / 100);

  MicMeasurements = 0;
  total = 0;


  // Read BLE status 

  { 

    Serial1.print('R');
    
    Serial.println("BLE status: ");
    uint8_t serialbytes = Serial1.available();    
    while(serialbytes) {
        
          int inByte = Serial1.read();
          Serial.write(inByte);
          serialbytes--;
    
    }
    delay (50);
    Serial.print(" ");
   
  }

  // Read BME data

  float temp = bme.readTemperature();
  float humd = bme.readHumidity();
  float pres = bme.readPressure() / 100.0;
  
  int16_t tempt = (int16_t)(temp);
  int16_t humdt = (int16_t)(humd);
  int16_t prest = (int16_t)(pres);

  // Read sps30 data

  int16_t pm1 = (int16_t)(m.mc_1p0);
  int16_t pm2p5 = (int16_t)(m.mc_2p5);
  int16_t pm4 = (int16_t)(m.mc_4p0);
  int16_t pm10 = (int16_t)(m.mc_10p0); 

  // Read RTC data 
  
  int16_t year = (int16_t)(now.year());
  int16_t month = (int16_t)(now.month());
  int16_t day = (int16_t)(now.day());
  int16_t hour = (int16_t)(now.hour());
  int16_t minute = (int16_t)(now.minute());
  int16_t second = (int16_t)(now.second()); 

  // String times = time.timestamp(DateTime::TIMESTAMP_FULL);  
  //Full Timestamp
  // Serial.println(String("TIMESTAMP_FULL:\t")+time.timestamp(DateTime::TIMESTAMP_FULL));
  // Serial.println(times); 
  Serial.println();
  Serial.println("Microphone -> average AnalogRead: "); 
  
  // Read microphone data and recalculate voltage on decibels
  float a;
  float Micc {
    
    Micc = (20*log10(averagedMic*ADCtomV))+57.424 }; // reference to ADC voltage
    int16_t MicV2 = (int16_t)(Micc); // 
    String MicString; // 
    if(Micc > 85)
    {
        Serial.println(Micc);
        MicString = String(MicV2);  
    }
    else
    {
        Serial.println("Noise < set level ");
        MicString = "Noise < set level "; 
    }


  // Datalogger
  digitalWrite(8, HIGH); // <-- important to switch off the LoRa for the SD card to work

  String dataString = "";
  dataString += "T+H+P;PM1+PM2.5+PM4+PM10;Mic+MicString;Batt;";
  dataString += String(tempt);
  dataString += ",";
  dataString += String(humdt);
  dataString += ",";
  dataString += String(prest);
  dataString += ", ";
  dataString += String(pm1);
  dataString += ",";
  dataString += String(pm2p5);
  dataString += ",";
  dataString += String(pm4);
  dataString += ",";
  dataString += String(pm10);
  dataString += ", ";
  dataString += String(Micc);
  dataString += ", ";
  dataString += MicString;
  dataString += ", ";
  dataString += String(Batt);
  
  char buf_rtc[] = "YYYY.MM.DD,hh:mm:ss,";

  // SD card
  // Initialization (SD.begin(cardSelect)) could be in the setup section, although if card 
  // would be ejected during measurements and reinserted it would not start logging againg
  SD.begin(cardSelect);
  File dataFile = SD.open(file_name.c_str(), FILE_WRITE);
  if (dataFile)
  {
    dataFile.print(now.toString(buf_rtc));
    dataFile.println(dataString);
    dataFile.close();
    SDstatus = 1;
  
    // Serial0
    Serial.print("Datalogger: ");
    Serial.print(now.toString(buf_rtc));
    Serial.println(dataString);
  } 
  else
  {
    SDstatus = 0;
    Serial.println("Error opening data file");
  }


  digitalWrite(8, LOW);  // <-- important to switch on the LoRa after writing on the SD card
    
  Serial.println("Lora sending: RTC + BME + sps30 + Mic + MicString + Bat + SDstatus to rf95_reliable_datagram_server"); 
  char radiopacket[72] = "RTC, BME280, sps30, Mic, Bat, SD: #      "; 
  snprintf(radiopacket,sizeof(radiopacket), "%d,%d,%d, %d,%d,%d, %d,%d,%d, %d,%d,%d,%d, %d, %d, %d", year, month, day, hour, minute, second, tempt, humdt, prest, pm1, pm2p5, pm4, pm10, MicV2, Batt, SDstatus); 
  itoa(packetnum++, radiopacket+72, 10);  // The itoa() function constructs a string representation of an integer. 

  Serial.print("RTC + BME + sps30 data sent: "); Serial.println(radiopacket);
  radiopacket[72] = 0; 
    
  // Send a message to manager server (raspberry reciver)
  if (manager.sendtoWait((uint8_t *)radiopacket, sizeof(radiopacket), RASPBERRY_ADDRESS)) 
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf); 
    uint8_t from;    
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) 
    {
      Serial.print("got reply from: 0x"); 
      Serial.print(from, HEX); 
      Serial.print(": "); 
      Serial.println("");
      Serial.println((char*)buf); 
      Serial.println("");
      Serial.print("LoRa RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      Serial.println("");
    }
    else 
    {
      Serial.println("No reply, is rf95_reliable_datagram_server running?"); 
    }
  }
  else 
    Serial.println("sendtoWait FAILED!"); 
    Serial.println("");
  delay(500); 

  
  do {
    ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      Serial.print("error reading sps30 data-ready flag: ");
      Serial.println(ret);
    } else if (!data_ready)
      Serial.print("sps30 data not ready, no new measurement available\n");
    else
      break;
    delay(1000); // retry in 1s 
  } while (1);

  ret = sps30_read_measurement(&m);
  if (ret < 0) {
    Serial.print("error reading sps30 measurement\n");
  } else { 

// sps data on serial

  // LCD - RTC clock data
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");

  lcd.setRGB(000, 000, 200);
  
  lcd.setCursor(0,0);
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  lcd.print(now.second(), DEC);
  // if days of the week are needed on LCD
  // lcd.print(" ");
  // lcd.print(daysOfTheWeek[now.dayOfTheWeek()]); 
 
  lcd.setCursor(0,1);
  lcd.print(now.day(), DEC);
  lcd.print('/');
  lcd.print(now.month(), DEC);
  lcd.print('/');
  lcd.print(now.year(), DEC);
  lcd.print(" ");

  lcd.print("B:");
  // lcd.print(measuredvbat);
  // lcd.print("V ");

  if(VBATPIN > 0 && VBATPIN < 1){
    lcd.print("25%");
  }else if(VBATPIN > 1 && VBATPIN < 2.775){
    lcd.print("50%");
  }else if(VBATPIN > 2.775 && VBATPIN < 3.0){
    lcd.print("75%");
  }else if(VBATPIN > 3.0 && VBATPIN < 3.7){
    lcd.print("75-100%");
  }else{
    lcd.print("100%");
  }
  lcd.print(" ");
  delay(3200);


  // LCD - Climatic data
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setRGB(000, 200, 000);
  
  lcd.setCursor(0,0);
  lcd.print("P:");
  lcd.print(pres, 1); 
  lcd.print(" hPa");
  lcd.print(" ");
  
  lcd.setCursor(0,1);
  lcd.print("T:");
  lcd.print(temp, 1);
  lcd.print("C");
  lcd.print(" ");

  lcd.print(" H:");
  lcd.print(humd, 1);
  lcd.print("%");
  lcd.print(" ");
  delay(3200);


  // LCD - Microphone
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");

  // Noise in red if value exceeds 78 dB. 
  // Below 78 dB the noise is not properly calculated in this microphone so it should be stated as unimportant.
  if (Micc > 78) 
  {
    lcd.setRGB(200, 000, 000);

    lcd.setCursor(0,0);
    lcd.print("Noise: ");
    lcd.print(Micc, 1); 
    lcd.print(" dB");
    lcd.setCursor(0,1);  
    lcd.print(MicString);
  
    delay(3200);
  }
  else 
  {
    lcd.setRGB(255, 255, 255);

    lcd.setCursor(0,0);
    lcd.print("Noise below ");
    lcd.setCursor(0,1);  
    lcd.print("set level ");
  
    delay(3200);
  } 
  

  // LCD - sps30 data
  
  if (m.mc_2p5 > 25) // <--- PM2.5 in red if value exceeds 25 ug/m3
  {
    lcd.setRGB(200, 000, 000);
  }
  else 
  {
    lcd.setRGB(255, 255, 255);
  } 


  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");

  lcd.setCursor(0,0); 
  // lcd.print("  PM1 "); 
  // lcd.print(m.mc_1p0, 1);
  // lcd.print(" ug/m3"); 
  // lcd.setCursor(0,1); 
  lcd.print("PM2,5: "); 
  lcd.setCursor(0,1); 
  lcd.print(m.mc_2p5, 1);
  lcd.print(" ug/m3"); 
  delay(3200);

  if (m.mc_10p0 > 50) // <--- PM10 in red if value exceeds 50 ug/m3
  {
    lcd.setRGB(200, 0, 000);
  }
  else 
  {
    lcd.setRGB(255, 255, 255);
  }

  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");

  lcd.setCursor(0,0); 
  // lcd.print("  PM4 "); 
  // lcd.print(m.mc_4p0, 1);
  // lcd.print(" ug/m3"); 
  // lcd.setCursor(0,1); 
  lcd.print("PM10: "); 
  lcd.setCursor(0,1); 
  lcd.print(m.mc_10p0, 1);
  lcd.print(" ug/m3"); 
  }

}
