#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <Mille_UNO.h>

boolean fix = 0;
long lat, lon;
float vel;
unsigned long gradi, date, times;

GPS gps(1); //1: EM406a, 2: G.top013


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
gps.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
 fix = gps.readGPS(&vel, &gradi, & date, & times, &lat, & lon);
  if (fix){
    Serial.print('\n');
    Serial.print(vel);Serial.print('\t');
    Serial.print(gradi);Serial.print('\t');
    Serial.print(lat);Serial.print('\t');
    Serial.print(lon);Serial.print('\t');
    Serial.print(times);Serial.print('\t');
    Serial.print(date);Serial.print('\n');
  }
}
