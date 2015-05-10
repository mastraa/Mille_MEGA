#include <Wire.h>
#include <Mille.h>
#define MPU6050             0x68   //MPU6050 device ID address
#define MPU6050_ACC         0x3B   //MPU6050 accelerometer data start register
#define MPU6050_TEMP        0x41   //MPU6050 termometer data start register
#define MPU6050_GYRO        0x43   //MPU6050 gyro data start register


Communication i2c;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  byte buff[14];
  int  dataRaw[6];
  //start data reading from 
  i2c.readFrom(MPU6050, MPU6050_ACC, 14, buff);
  dataRaw[0] = (((int)buff[0]) << 8) | buff[1];
  dataRaw[1] = (((int)buff[2]) << 8) | buff[3];
  dataRaw[2] = (((int)buff[4]) << 8) | buff[5];
  dataRaw[3] = (((int)buff[8])  << 8) | buff[9];
  dataRaw[4] = (((int)buff[10]) << 8) | buff[11];
  dataRaw[5] = (((int)buff[12]) << 8) | buff[13];
  
  for (byte i=0; i<6; i++){
    Serial.print(dataRaw[i]);
    Serial.print('\t');
  };
  Serial.print('\n');
  delay(500);
}
