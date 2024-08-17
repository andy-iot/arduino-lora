#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include "SSD1306.h" 
#include "images.h"
#include <BME280I2C.h>


#define SCK     5    // GPIO5  -- SCK
#define MISO    19   // GPIO19 -- MISO
#define MOSI    27   // GPIO27 -- MOSI
#define SS      18   // GPIO18 -- CS
#define RST     23   // GPIO14 -- RESET (If Lora does not work, replace it with GPIO14)
#define DI0     26   // GPIO26 -- IRQ(Interrupt Request)
#define BAND    433E6


unsigned int counter = 0;

SSD1306 display(0x3c, 21, 22);
String rssi = "RSSI --";
String packSize = "--";
String packet ;

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
);

BME280I2C bme(settings);

void setup() {
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("LoRa Sender Test");
  
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa init ok");

  // bme280
  Wire.begin();

  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }

  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);
   
  delay(1500);
}

void loop() {

  String NodeId = WiFi.macAddress();

  float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  String msg = "{\"node\":\"" + NodeId + "\",\"temperature\":\"" + String(temp) + "\",\"pressure\":\"" + String(pres) + "\",\"humidity\":\"" + String(hum) + "\"}";

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0, 0, "Sending packet: ");
  display.drawString(90, 0, String(counter));
  display.drawString(190, 0, msg);
  Serial.println(String(counter));
  Serial.println(msg);
  display.display();

  // send packet
  LoRa.beginPacket();
  LoRa.print(msg);
  //LoRa.print(counter);
  LoRa.endPacket();

  counter++;
  digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}