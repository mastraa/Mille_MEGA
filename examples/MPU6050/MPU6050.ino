//INSERT LIBRARIES
#include <Wire.h>
#include <OneWire.h>
#include <Mille_UNO.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <SD.h>
#include <SPI.h>

#define DEGTORAD        0.017453293f
#define RADTODEG       57.295780f
#define BETA            0.05

#define MPU6050_A0          LOW   //MPU6050 device AD0 level
#define GYRO_SAMPLES        1000  //number of samples for gyro offset calibration
#define TINY_ADD            0x26  //AtTiny address
#define LCD_ADD             0x27
#define BAUD                9600
#define NOMEFILE            "vela000.txt"

float acc[3];             //X, Y, Z accelerometer value (calibrated in G)
float offAcc[3];          //accelerometer offset parameters
float gainAcc[3][3];      //accelerometer calibration matrix

float gyr[3];             //X, Y, Z gyroscope value (calibrated in rad/s)
float offGyr[3];          //gyroscope offset parameters
float gainGyr[3][3];      //gyroscope calibration matrix

float mag[3];             //X, Y, Z gyroscope value (calibrated in rad/s)
float offMag[3];          //gyroscope offset parameters
float gainMag[3][3];      //gyroscope calibration matrix

float attitude[3];        //yaw, pitch, roll

float wind[2];            //speed, direction



//MPU6050 MPU(MPU6050_A0);
//HMC5883L HMC;
//ATTINY WIND (TINY_ADD);
//AHRSFILTER AHRS;
//LCD_I2C lcd(LCD_ADD, 16, 2); //indirizzo, colonne, righe
//LCD_CLASSIC lcd(41, 43, 45, 47, 49, 48, 16, 2); rs, en, d4-7, colonne, righe
SDCARD sd (10);


void setup() {
  //Starting libraries
  Wire.begin();
  Serial.begin(BAUD);
  sd.init();
  sd.setName(NOMEFILE);
  
  

  //Initialize function
  //MPU.setMPU(0x07, 0x04, 0x00, 0x00, 0x08, 0x00); //smprt_div, dlpf_conf, gyro_conf, acc_conf, pwr_mgmt_1, pwr_mgmt_2
  //HMC.setHMC(0x00, 0x38, 0x20); //mode, conf_a, conf_b
  //offValuesSetting(); //local function, will set offset and gain matrix, it calls MPU function
  //AHRS.start(); //initialize time variable
  //lcd.start();  //initialize lcd (classic and i2c)
  
  
  
}


void loop() {
  sd.openFile('w');
  sd.closeFile();
  
  
sd.setName( sd.getFreeName (NOMEFILE, 4, 1));
char* prova =sd.getName();
Serial.println(prova);
  
  
  //IMU request
  //MPU.readMPU(offAcc, gainAcc, offGyr, gainGyr, acc, gyr); //get calibrated values
  //MPU.readMPU(acc, gyr); //get raw values
  //HMC.readHMC(offMag, gainMag, mag);//get calibrated values values
  //HMC.readHMC(mag); //get raw values
  //AHRS.Filter(gyr, acc, mag, BETA, attitude);
  //Serial.print(attitude[0]);Serial.print('\t');
  //Serial.print(attitude[1]);Serial.print('\t');
  //Serial.print(attitude[2]);Serial.print('\n');

  
  //WIND.readTiny(5, wind); //Read AtTiny85 wind station
  delay(1000);
  
  }
    

void offValuesSetting(){
  offAcc[0] =  157.860561;
  offAcc[1] =  264.857188;
  offAcc[2] =  458.620122;
  
  gainAcc[0][0] =  0.992987 / 16384.;
  gainAcc[0][1] = -0.011437 / 16384.;
  gainAcc[0][2] =  0.066142 / 16384.;
  gainAcc[1][0] = -0.011437 / 16384.;
  gainAcc[1][1] =  1.000824 / 16384.;
  gainAcc[1][2] =  0.003092 / 16384.;
  gainAcc[2][0] =  0.066142 / 16384.;
  gainAcc[2][1] =  0.003092 / 16384.;
  gainAcc[2][2] =  0.984898 / 16384.;

  gainGyr[0][0] = DEGTORAD/131.0f;
  gainGyr[0][1] = 0.;
  gainGyr[0][2] = 0.;
  gainGyr[1][0] = 0.;
  gainGyr[1][1] = DEGTORAD/131.0f;
  gainGyr[1][2] = 0.;
  gainGyr[2][0] = 0.;
  gainGyr[2][1] = 0.;
  gainGyr[2][2] = DEGTORAD/131.0f;
  
  offMag[0] =    52.369866;
  offMag[1] =  -112.225169;
  offMag[2] =   135.192094;
  gainMag[0][0] =  0.928968;
  gainMag[0][1] =  0.004310;
  gainMag[0][2] =  0.015526;
  gainMag[1][0] =  0.004310;
  gainMag[1][1] =  0.867471;
  gainMag[1][2] =  0.022721;
  gainMag[2][0] =  0.015526;
  gainMag[2][1] =  0.022721;
  gainMag[2][2] =  0.993729;
  
  //Setting gyro offset matrix
  //MPU.offsetGyr(GYRO_SAMPLES, offGyr);

}

  
