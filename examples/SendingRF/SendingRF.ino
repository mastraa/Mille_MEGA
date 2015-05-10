//INSERT LIBRARIES
#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Mille_MEGA.h>

uint8_t cs = 10;
uint8_t ce = 9;

#define OUT 0xF0F0F0F0E1LL
#define IN 0xF0F0F0F0D2LL

WIND wind(9,10, OUT, IN);

wind_t wind_T;

void setup() {
  Serial.begin(57600);
  wind.init();

}

void loop() {
  wind_T.speed = random (0, 100);
  wind_T.dir_1 = random (0, 1000);
  wind_T.dir_2 = random (1000, 2000);
  wind.write(
  
  if(wind.write(&wind_T, sizeof(wind_t))){
    Serial.print(wind_T.speed);Serial.print('\t');
    Serial.println("send ok");
  }
  
  delay(1000);

}
