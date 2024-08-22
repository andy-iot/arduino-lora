#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include "SSD1306.h" 
#include "images.h"
#include <BME280I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <libpax_api.h>

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
Adafruit_MPU6050 mpu;

struct count_payload_t count_from_libpax;

String NodeId;

void process_count(void) {
  printf("pax: %lu; %lu; %lu;\n", count_from_libpax.pax, count_from_libpax.wifi_count, count_from_libpax.ble_count);
}

void initPaxCount() {
  struct libpax_config_t configuration; 
  libpax_default_config(&configuration);
  configuration.blecounter = 1;
  configuration.blescantime = 0; // infinit
  configuration.wificounter = 1; 
  configuration.wifi_channel_map = WIFI_CHANNEL_ALL;
  configuration.wifi_channel_switch_interval = 5;
  configuration.wifi_rssi_threshold = -80;
  configuration.ble_rssi_threshold = -80;
  libpax_update_config(&configuration);

  // internal processing initialization
  libpax_counter_init(process_count, &count_from_libpax, 10, 1); 
  libpax_counter_start();
}

void static updateOled(int counter)
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0, 0, "Sending packet: ");
  display.drawString(90, 0, String(counter));
  display.display();
}

void static sendLoRaMsg(String msg)
{
  // send packet
  LoRa.beginPacket();
  LoRa.print("Message: " + msg);
  LoRa.endPacket();
}

void static detectedShake()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, mpuTemp;
  mpu.getEvent(&a, &g, &mpuTemp);

  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z);
  Serial.println("");

  String msg = "{\"type\":\"shake\", \"node\":\"" + NodeId + "\",\"AccelX\":\"" + a.acceleration.x + "\",\"AccelY\":\"" + a.acceleration.y + "\",\"AccelZ\":\"" + a.acceleration.z + "\"}";
  
  Serial.println("Sending Shake Message!");
  Serial.println(msg);
  sendLoRaMsg(msg);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  NodeId = WiFi.macAddress();

  Serial.println();
  Serial.println("LoRa Test");
  
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

  // setup the MPU
  Serial.println("Initialize MPU6050");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  initPaxCount();

  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);
   
  delay(1500);
}

void loop() {

  float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  String countStr = String(counter);

  if(mpu.getMotionInterruptStatus()) {
    detectedShake();
  }

  String msg = "{\"type\":\"temperature\", \"counter\":\"" + countStr + "\",\"node\":\"" + NodeId + "\",\"temperature\":\"" + String(temp) + "\",\"pressure\":\"" + String(pres) + "\",\"humidity\":\"" + String(hum) + "\"}";

  updateOled(counter);
  Serial.println("Sending Temp Message: " + String(counter));
  Serial.println(msg);

  sendLoRaMsg(msg);

  counter++;
  digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

