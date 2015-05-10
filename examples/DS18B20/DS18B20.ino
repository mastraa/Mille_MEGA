#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS.h>
#include "nRF24L01.h"
#include "RF24.h"

//includere sempre per ultimi e in questo ordine
#include <Mille_MEGA.h>

#define DS_PIN 2
TEMP ds(DS_PIN);


void setup() {
  // put your setup code here, to run once:
ds.check();
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(ds.getTemp());
  delay(300);
}
