//=====================================================================================================
// Mille_MEGA.cpp
//=====================================================================================================
//
// Libreria per datalogger ProjectR3
//
// Date			Author					Notes
// 11/08/2014	Andrea Mastrangelo		I release
//
//
//
//Credits:
//Ing.Andrea Mastrangelo
//Ing.Cristiano Battisti
//Stefano Pieretti
//=====================================================================================================


#include <Arduino.h>
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



//SECURE CLASS

Secure::Secure(int pin){
	pinMode(pin, OUTPUT);
	digitalWrite(pin, HIGH);
	_off = pin;
}

void Secure::secureOff(){
	digitalWrite(_off, LOW);
	
}



//I2C CLASS

I2C::I2C(byte address){
    _ADDR = address;
};

void I2C::writeTo (byte reg_address, byte val)
{
	Wire.beginTransmission (_ADDR); //start transmission to device
	Wire.write (reg_address);           //send register address
	Wire.write (val);               //send value to write
	Wire.endTransmission ();        //end transmission
}

//reads 'num' bytes starting from address register on device into buff array
void I2C::readFrom (byte reg_address, byte num, byte *buff)
{
	Wire.beginTransmission (_ADDR);   //start transmission to device
	Wire.write (reg_address);             //sends address to read from
    Wire.endTransmission ();           //end transmission
    Wire.beginTransmission (_ADDR); //start transmission to device
	Wire.requestFrom (_ADDR, num);    //request 'num' bytes from device
	int i=0;
	while(Wire.available() && i<num)  //device may send less than requested (abnormal)
	{
		buff[i] = Wire.read ();        //receive a byte
		i++;
	}
	Wire.endTransmission ();           //end transmission
}

void I2C::readFrom (byte num, byte *buff)
{
    Wire.beginTransmission (_ADDR); //start transmission to device
	Wire.requestFrom (_ADDR, num);    //request 'num' bytes from device
	int i=0;
	while(Wire.available() && i<num)  //device may send less than requested (abnormal)
	{
		buff[i] = Wire.read ();        //receive a byte
		i++;
	}
	Wire.endTransmission ();           //end transmission
}



//MPU6050 CLASS

MPU6050::MPU6050 (byte a0): I2C(MPU6050_ad+a0){
    _mpuAcc = 0x3B;
    _mpuTemp = 0x41;
    _mpuGyr = 0x43;
};

void MPU6050::readMPU(float *acc, float *gyr){
    byte buff[14];
    readFrom(_mpuAcc, 14, buff);
    acc[0] = (((int)buff[0]) << 8) | buff[1];
    acc[1] = (((int)buff[2]) << 8) | buff[3];
    acc[2] = (((int)buff[4]) << 8) | buff[5];
    
    gyr[0] = (((int)buff[8])  << 8) | buff[9];
    gyr[1] = (((int)buff[10]) << 8) | buff[11];
    gyr[2] = (((int)buff[12]) << 8) | buff[13];
}

void MPU6050::readMPU(float offAcc[3], float gainAcc[3][3], float offGyr[3], float gainGyr[3][3], float *acc, float *gyr){
    
    byte buff[14];
    int dataRaw[3];
    
    readFrom(_mpuAcc, 14, buff);
    
    dataRaw[0] = (((int)buff[0]) << 8) | buff[1];
    dataRaw[1] = (((int)buff[2]) << 8) | buff[3];
    dataRaw[2] = (((int)buff[4]) << 8) | buff[5];
    acc[0] = gainAcc[0][0]*(dataRaw[0]-offAcc[0]) + gainAcc[0][1]*(dataRaw[1]-offAcc[1]) + gainAcc[0][2]*(dataRaw[2]-offAcc[2]);
    acc[1] = gainAcc[1][0]*(dataRaw[0]-offAcc[0]) + gainAcc[1][1]*(dataRaw[1]-offAcc[1]) + gainAcc[1][2]*(dataRaw[2]-offAcc[2]);
    acc[2] = gainAcc[2][0]*(dataRaw[0]-offAcc[0]) + gainAcc[2][1]*(dataRaw[1]-offAcc[1]) + gainAcc[2][2]*(dataRaw[2]-offAcc[2]);
    
    dataRaw[0] = (((int)buff[8])  << 8) | buff[9];
    dataRaw[1] = (((int)buff[10]) << 8) | buff[11];
    dataRaw[2] = (((int)buff[12]) << 8) | buff[13];
    gyr[0] = gainGyr[0][0]*(dataRaw[0]-offGyr[0]) + gainGyr[0][1]*(dataRaw[1]-offGyr[1]) + gainGyr[0][2]*(dataRaw[2]-offGyr[2]);
    gyr[1] = gainGyr[1][0]*(dataRaw[0]-offGyr[0]) + gainGyr[1][1]*(dataRaw[1]-offGyr[1]) + gainGyr[1][2]*(dataRaw[2]-offGyr[2]);
    gyr[2] = gainGyr[2][0]*(dataRaw[0]-offGyr[0]) + gainGyr[2][1]*(dataRaw[1]-offGyr[1]) + gainGyr[2][2]*(dataRaw[2]-offGyr[2]);
}

void MPU6050::offsetGyr(int samples, float *offGyr){
    long sum[3];
    sum[0] = 0;
    sum[1] = 0;
    sum[2] = 0;
    byte buff[6];
    for(int i=0; i<samples; i++){
        readFrom(0x43, 6, buff);
        sum[0] += (((int)buff[0]) << 8) | buff[1];
        sum[1] += (((int)buff[2]) << 8) | buff[3];
        sum[2] += (((int)buff[4]) << 8) | buff[5];
        delay (2);
    }
    for (int i=0; i<3; i++) {
        offGyr[i] = (1./samples) * sum[i];
    }
    Serial.print('\n');
}

void MPU6050::setMPU (byte smprt_div, byte dlpf_conf, byte gyro_conf, byte acc_conf, byte pwr_mgmt_1, byte pwr_mgmt_2){
    
    writeTo(MPU_PWR_MGMT_1, pwr_mgmt_1);
    writeTo(MPU_PWR_MGMT_2, pwr_mgmt_2);
    writeTo(MPU_SMPRT_DIV,  smprt_div);
    writeTo(MPU_GYRO_CONF, gyro_conf);
    writeTo(MPU_ACC_CONF, acc_conf);
    writeTo(MPU_DLPF_CONF, dlpf_conf);
    
}



//HMC5883L CLASS

HMC5883L::HMC5883L(): I2C(HMC5883L_ad){
    _hmcMag = 0x03;
}

void HMC5883L::readHMC(float *mag){
        byte buff[6];
        readFrom(_hmcMag, 6, buff);
        mag[0] = (((int)buff[0]) << 8) | buff[1];
        mag[2] = (((int)buff[2]) << 8) | buff[3];
        mag[1] = (((int)buff[4]) << 8) | buff[5];
}

void HMC5883L::readHMC(float offMag[3], float gainMag[3][3], float *mag){
    byte buff[6];
    int dataRaw[3];
    
    readFrom(_hmcMag, 6, buff);
    
    dataRaw[0] = (((int)buff[0]) << 8) | buff[1];
    dataRaw[2] = (((int)buff[2]) << 8) | buff[3];
    dataRaw[1] = (((int)buff[4]) << 8) | buff[5];
    mag[0] = gainMag[0][0]*(dataRaw[0]-offMag[0]) + gainMag[0][1]*(dataRaw[1]-offMag[1]) + gainMag[0][2]*(dataRaw[2]-offMag[2]);
    mag[1] = gainMag[1][0]*(dataRaw[0]-offMag[0]) + gainMag[1][1]*(dataRaw[1]-offMag[1]) + gainMag[1][2]*(dataRaw[2]-offMag[2]);
    mag[2] = gainMag[2][0]*(dataRaw[0]-offMag[0]) + gainMag[2][1]*(dataRaw[1]-offMag[1]) + gainMag[2][2]*(dataRaw[2]-offMag[2]);
    }

void HMC5883L::setHMC(byte mode, byte conf_a, byte conf_b){
    writeTo(HMC_MODE, mode);  //wake up and set continuous measurement
    writeTo(HMC_CONF_A, conf_a);  //set 4-samples average & I2C writing rate @75Hz
    writeTo(HMC_CONF_B, conf_b);  //set measurement range to +/-1.3 Ga (gain = 1090 LSB/Ga)
}



//ATTINY CLASS

ATTINY::ATTINY(byte address): I2C(address){
    _time = millis(); //start time counter
}

void ATTINY::readTiny(byte num, float *wind){
    byte buff[num];
    readFrom(num, buff); //request to attiny
    wind[0] = float(buff[0]*170/(millis()-_time)); //insert speed value
    //ATTENZIONE LA FUNZIONE NON E' QUESTA, VA STUDIATA
    wind[1] = (float(buff[1]<<8 | buff[2])) +  (float(buff[3]<<8 | buff[4]));
    _time = millis();
}



//AHRSFILTER CLASS

AHRSFILTER::AHRSFILTER(){
    }

void AHRSFILTER::start(){
    _time = micros();
    q0=1.0, q1=0.0, q2=0.0, q3=0.0;
}

void AHRSFILTER::Filter(float gyr[3], float acc[3], float mag[3], float BETA, float *attitude) {
    float dT = (micros()-_time)*0.000001;
    _time = micros();
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    
    //rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (        -q1*gyr[0]  - q2*gyr[1] - q3*gyr[2]);
    qDot2 = 0.5f * ( q0*gyr[0]          + q2*gyr[2] - q3*gyr[1]);
    qDot3 = 0.5f * ( q0*gyr[1] - q1*gyr[2]          + q3*gyr[0]);
    qDot4 = 0.5f * ( q0*gyr[2] + q1*gyr[1] - q2*gyr[0]         );
    //compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!( (0.0f==acc[0]) && (0.0f==acc[1]) && (0.0f==acc[2]) ))
    {
        //normalize accelerometer measurement
        recipNorm = 1.0f / sqrt (acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
        acc[0] *= recipNorm;
        acc[1] *= recipNorm;
        acc[2] *= recipNorm;
        //normalize magnetometer measurement
        recipNorm = 1.0f / sqrt (mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
        mag[0] *= recipNorm;
        mag[1] *= recipNorm;
        mag[2] *= recipNorm;
        //auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0*mag[0];
        _2q0my = 2.0f * q0*mag[1];
        _2q0mz = 2.0f * q0*mag[2];
        _2q1mx = 2.0f * q1*mag[0];
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0*q2;
        _2q2q3 = 2.0f * q2*q3;
        q0q0 = q0*q0;
        q0q1 = q0*q1;
        q0q2 = q0*q2;
        q0q3 = q0*q3;
        q1q1 = q1*q1;
        q1q2 = q1*q2;
        q1q3 = q1*q3;
        q2q2 = q2*q2;
        q2q3 = q2*q3;
        q3q3 = q3*q3;
        //reference direction of Earth's magnetic field
        hx =   2.0f * ( mag[0]*(q0q0 + q1q1 - 0.5f) + mag[1]*(q1q2 - q0q3)        + mag[2]*(q0q2 + q1q3) );
        hy =   2.0f * ( mag[0]*(q0q3 + q1q2)        + mag[1]*(q0q0 + q2q2 - 0.5f) + mag[2]*(q2q3 - q0q1) );
        _2bx = sqrt (hx*hx + hy*hy);
        _2bz = 2.0f * ( mag[0]*(q1q3 - q0q2)        + mag[1]*(q0q1 + q2q3)        + mag[2]*(q0q0 + q3q3 - 0.5f) );
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;
        //gradient descent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - acc[0]) + _2q1 * (2.0f * q0q1 + _2q2q3 - acc[1]) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag[0]) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag[1]) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag[2]);
        s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 - acc[0]) + _2q0 * (2.0f * q0q1 + _2q2q3 - acc[1]) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - acc[2]) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag[0]) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag[1]) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag[2]);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - acc[0]) + _2q3 * (2.0f * q0q1 + _2q2q3 - acc[1]) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - acc[2]) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag[0]) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag[1]) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag[2]);
        s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 - acc[0]) + _2q2 * (2.0f * q0q1 + _2q2q3 - acc[1]) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag[0]) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag[1]) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag[2]);
        recipNorm = 1.0f / sqrt (s0*s0 + s1*s1 + s2*s2 + s3*s3); // normalize step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        //apply feedback step
        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;
    } //end of gyro correction
    //integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dT;
    q1 += qDot2 * dT;
    q2 += qDot3 * dT;
    q3 += qDot4 * dT;
    //normalise quaternion
    recipNorm = 1.0f / sqrt (q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    
    //Compute YAW - PITCH - ROLL
    attitude[0]  = atan2 (q1*q2 + q0*q3, q0*q0 + q1*q1 - 0.5f);
    attitude[1] = -asin (2.0f * (q1*q3 - q0*q2));
    attitude[2] = atan2 (q2*q3 + q0*q1, q0*q0 + q3*q3 - 0.5f);
    
    attitude[0] = RADTODEG * attitude[0];//YAW
    attitude[1] = RADTODEG * attitude[1];//PITCH
    attitude[2] = RADTODEG * attitude[2];//ROLL
}


//MEMSD CLASS
SDCARD::SDCARD(byte pin): File (){
    pinMode(53, OUTPUT); //Arduino MEGA
    //pinMode(10, OUTPUT); //Arduino UNO
    pinMode(pin, OUTPUT);
    _pin = pin;
}

SDCARD::SDCARD(): File (){
    pinMode(53, OUTPUT); //Arduino MEGA
    //pinMode(10, OUTPUT); //Arduino UNO
    _pin = 53;
}

void SDCARD::init(){
    if(!SD.begin(_pin)){
        Serial.println("problem SD INIT");
    };
}

char* SDCARD::getName(){
    return _name;
}

//Parameters: base_name, position of first number, 0 for two numbers, 1 for three numbers
char* SDCARD::getFreeName(char* name, byte i, boolean n){
    if (n){
        for (int x = 0; x < 1000; ++x) { //Trova il primo nome libero
            name[i] = x/100 + '0';
            name[i+1] = (x%100)/10 + '0';
            name[i+2] = (x%100)%10 + '0';
            if (! SD.exists(name)) {
                return name;
                break;
            }
        }
    }
    else{
        for (uint8_t x = 0; x < 100; ++x) { //Trova il primo nome libero
            name[i] = x/10 + '0';
            name[i+1] = x%10 + '0';
            if (! SD.exists(name)) {
                return name;
                break;
            }
        }
    }
}

void SDCARD::setName(char* name){
    _name=name;
}

void SDCARD::openFile(){ //default FILE_READ
    _file = SD.open(_name);
}

void SDCARD::openFile(char s){
    switch (s)
    {
        case 'r':
            _file = SD.open(_name);
            if (!_file) Serial.println("Opening file ERROR");
            break;
        case 'w':
            _file = SD.open(_name, FILE_WRITE);
            if (!_file) Serial.println("Opening file ERROR");
            break;
        default: Serial.print("Parametro non valido");break; //mettere errore in lcd
    }
}

void SDCARD::closeFile(){
    _file.close();
}

void SDCARD::printFile(char* string){
        _file.print(string);
}

void SDCARD::writeFile(char val){
    _file.print(val);
}

void SDCARD::printFile(int val){
        _file.print(val);
}

void SDCARD::printFile(unsigned int val){
    _file.print(val);
}

void SDCARD::printFile(long val){
    _file.print(val);
}

void SDCARD::printFile(unsigned long val){
    _file.print(val);
}

void SDCARD::newLineFile(){
    _file.print('\n');
}

void SDCARD::tabFile(){
    _file.print('\t');
}

void SDCARD::printFile(char** string, int x){
    for (int i=0; i<x; i++){
        _file.print(string[i]);
        _file.print('\t');
    }
}

void SDCARD::printFile(int* array, int x){
    for (int i=0; i<x; i++){
        _file.print(array[i]);
        _file.print('\t');
    }
}

void SDCARD::printFile(float* array, int x){
    for (int i=0; i<x; i++){
        _file.print(array[i]);
        _file.print('\t');
    }
}

void SDCARD::readFile(){
    while (_file.available()) {
        Serial.write(_file.read());
    }
}

void SDCARD::printMVUPC(struct Mvupc_t mvupc){
    _file.print("$MVUPC,");_file.print(millis());_file.print(",");
    _file.print(mvupc.lat);_file.print(",N,");
    _file.print(mvupc.lon);_file.print(",E,");_file.print(mvupc.vel);
    _file.print(",K,");_file.print(mvupc.gradi/100);_file.print(",C,");
    _file.print(mvupc.attitude[2]);_file.print(",");_file.print(mvupc.attitude[1]);
    _file.print(",");_file.print(mvupc.attitude[0]);_file.print(",RPY");
    newLineFile();
}

void SDCARD::printMVUP(struct Mvup_t mvup){
    _file.print("$MVUP,");_file.print(millis());_file.print(",");
    _file.print(mvup.lat);_file.print(",N,");
    _file.print(mvup.lon);_file.print(",E,");_file.print(mvup.vel);
    _file.print(",K,");_file.print(mvup.gradi/100);_file.print(",C,");
    _file.print(mvup.attitude[2]);_file.print(",");_file.print(mvup.attitude[1]);
    _file.print(",");_file.print(mvup.attitude[0]);_file.print(",RPY,");
    _file.print(mvup.Wspeed);_file.print(",WS,");_file.print(mvup.vale_1);
    _file.print(",");_file.print(mvup.vale_2);_file.print(",WD,");
    _file.print(mvup.left);_file.print(",");_file.print(mvup.right);_file.print(",L");
    newLineFile();
}


//LED CLASS
LED::LED(byte pin){
    _pin = pin;
    pinMode(_pin, OUTPUT);
    _state = 0;
    digitalWrite(_pin,_state);//not necessary
}

void LED::onOff(){
    _state = _state ^ 1;
    digitalWrite(_pin,_state);
}



//LCD CLASSES: I2C or CLASSIC
LCD_I2C::LCD_I2C(byte address, byte col, byte row): LiquidCrystal_I2C(address,col,row){
}
void LCD_I2C::start(){
    init();
    backlight();
}
void LCD_I2C::Attitude(float* attitude){
    clear();
    print(attitude[0]);print(" ");print(attitude[1]);
    setCursor(0,1);print(attitude[2]);
}


LCD_CLASSIC::LCD_CLASSIC(byte rs, byte enable, byte d4, byte d5, byte d6, byte d7, byte col, byte row): LiquidCrystal(rs, enable, d4, d5, d6, d7){
    _col = col;
    _row = row;
    }
void LCD_CLASSIC::start(){
    begin(_col, _row);
}
void LCD_CLASSIC::Attitude(float* attitude){
    clear();
    print(attitude[0]);print(" ");print(attitude[1]);
    setCursor(0,1);print(attitude[2]);
}



//GPS CLASS
GPS::GPS(byte type): TinyGPS(){
    _type = type;
}

GPS::GPS(byte type, uint8_t freq): TinyGPS(){
    _type = type;
    _freq = freq;
}

void GPS::baudChange(unsigned int baudrate){
    Serial1.begin(baudrate);
}

void GPS::writeGPS(char* string){
    Serial1.println(string);
}

unsigned int GPS::begin(unsigned int baudrate){
    Serial1.begin(baudrate);
    if (_type== 1){
        Serial.println("Gps is EM406a");
        return 65536-16000000/256/INT_EM406A;//min 75
    }
    else if (_type==2){
        Serial.println("Gps is G.top013");
        return 65536-16000000/256/INT_GTOP;//min 150 for 1hz
    }
    else if (_type==3){
        Serial.println("Gps is Ublox");
        return 65536-16000000/256/INT_UBLOX;//min 75
    }
}

unsigned int GPS::begin(){
    if (_type== 1){
        Serial.println("Gps is EM406a");
        Serial1.begin(4800);
        return 65536-16000000/256/INT_EM406A;
    }
    else if (_type==2){
        Serial.println("Gps is G.top013");
        Serial1.begin(9600);
        return 65536-16000000/256/INT_GTOP;
    }
    else if (_type==3){
        Serial.println("Gps is Ublox");
        Serial1.begin(4800);
        //Serial1.println("$PUBX,41,1,0007,0003,4800,0*13");//4800
        //Serial1.begin(4800);
        //Serial1.println("$PUBX,41,1,0007,0003,9600,0*10");//9600
        return 65536-16000000/256/INT_UBLOX;//min 75
    }
}

void GPS::autoset(){
    if (_type== 1){
        Serial1.println("$PSRF103,00,00,00,01*24"); //Disabilita GGA
        Serial1.println("$PSRF103,01,00,00,01*25"); //Disabilita GGL
        Serial1.println("$PSRF103,02,00,00,01*26"); //Disabilita GSA
        Serial1.println("$PSRF103,03,00,00,01*27"); //Disabilita GSV
        Serial1.println("$PSRF103,04,00,00,01*20"); //Disabilita RMC
        Serial1.println("$PSRF103,05,00,00,01*21"); //Disabilita VTG
        
        //Serial1.println("$PSRF103,00,00,01,01*25"); //Abilita GGA 1hz
        //Serial1.println("$PSRF103,01,00,01,01*24"); //Abilita GGL 1hz
        //Serial1.println("$PSRF103,02,00,01,01*27"); //Abilita GSA 1hz
        //Serial1.println("$PSRF103,03,00,01,01*26"); //Abilita GSV 1hz
        Serial1.println("$PSRF103,04,00,01,01*21"); //Abilita RMC 1hz
        //Serial1.println("$PSRF103,05,00,01,01*20"); //Abilita VTG 1hz
    }
    else if (_type==2){
        if (_freq==1) Serial1.println("$PMTK220,1000*1F");//1hz
        else if (_freq==5)Serial1.println("$PMTK220, 200*2C");//5hz
        else if (_freq==10)Serial1.println("$PMTK220, 100*2F");//10hz
        
        Serial1.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");//RMC every 2 position fix
        //string-position(gll,rmc,vtg,gga,gsa,gsv), position-fix-numers(0,1,2,3,4,5)
    }
    else if (_type==3){
        Serial1.println("$PUBX,40,GGA,0,0,0,0*5A");
        Serial1.println("$PUBX,40,GLL,0,0,0,0*5C");
        Serial1.println("$PUBX,40,GSA,0,0,0,0*4E");
        Serial1.println("$PUBX,40,GSV,0,0,0,0*59");
        Serial1.println("$PUBX,40,RMC,0,1,0,0*46");
        Serial1.println("$PUBX,40,VTG,0,0,0,0*5E");
        Serial1.println("$PUBX,40,GRS,0,0,0,0*5D");
        Serial1.println("$PUBX,40,GST,0,0,0,0*5B");
        Serial1.println("$PUBX,40,ZDA,0,0,0,0*44");
    }

    Serial.println("GPS setted");
}

void GPS::autoset(bool sentences[]){
    if (_type== 1){
        char* on[] = {
            "$PSRF103,00,00,00,01*24",
            "$PSRF103,01,00,00,01*25",
            "$PSRF103,02,00,00,01*26",
            "$PSRF103,03,00,00,01*27",
            "$PSRF103,04,00,00,01*20",
            "$PSRF103,05,00,00,01*21"
        };
        
        char* off[] = {
            "$PSRF103,00,00,01,01*25",
            "$PSRF103,01,00,01,01*24",
            "$PSRF103,02,00,01,01*27",
            "$PSRF103,03,00,01,01*26",
            "$PSRF103,04,00,01,01*21",
            "$PSRF103,05,00,01,01*20"
        };
        for (byte i =0; i< sizeof(sentences); i++){
            if (sentences[i]) Serial1.println(on[i]);
            else Serial1.println(off[i]);
        }
        
            }
    else if (_type==2){
        Serial.println("Not supported for this GPS, use writeGPS function");
        if (_freq==1) Serial1.println("$PMTK220,1000*1F");//1hz
        else if (_freq==5)Serial1.println("$PMTK220, 200*2C");//5hz
        else if (_freq==10)Serial1.println("$PMTK220, 100*2F");//10hz
        
        Serial1.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");//RMC every 2 position fix
        //string-position(gll,rmc,vtg,gga,gsa,gsv), position-fix-numers(0,1,2,3,4,5)
    }
    else if (_type==3){
        char* on[] = {
            "$PUBX,40,GGA,0,0,0,0*5A",
            "$PUBX,40,GLL,0,0,0,0*5C",
            "$PUBX,40,GSA,0,0,0,0*4E",
            "$PUBX,40,GSV,0,0,0,0*59",
            "$PUBX,40,RMC,0,0,0,0*47",
            "$PUBX,40,VTG,0,0,0,0*5E",
            "$PUBX,40,GRS,0,0,0,0*5D",
            "$PUBX,40,GST,0,0,0,0*5B",
            "$PUBX,40,ZDA,0,0,0,0*44"
        };
        
        char* off[] = {
            "$PUBX,40,GGA,0,1,0,0*5B",
            "$PUBX,40,GLL,0,1,0,0*5D",
            "$PUBX,40,GSA,0,1,0,0*4F",
            "$PUBX,40,GSV,0,1,0,0*58",
            "$PUBX,40,RMC,0,1,0,0*46",
            "$PUBX,40,VTG,0,1,0,0*5F",
            "$PUBX,40,GRS,0,1,0,0*5C",
            "$PUBX,40,GST,0,1,0,0*5A",
            "$PUBX,40,ZDA,0,1,0,0*45"
        };
        
        for (byte i =0; i< sizeof(sentences); i++){
            if (sentences[i]) Serial1.println(on[i]);
            else Serial1.println(off[i]);
        }
    }
    
    Serial.println("GPS setted");
}

void GPS::autosetAll(){
    if (_type== 1){
        
        Serial1.println("$PSRF103,00,00,01,01*25"); //Abilita GGA 1hz
        Serial1.println("$PSRF103,01,00,01,01*24"); //Abilita GGL 1hz
        Serial1.println("$PSRF103,02,00,01,01*27"); //Abilita GSA 1hz
        Serial1.println("$PSRF103,03,00,01,01*26"); //Abilita GSV 1hz
        Serial1.println("$PSRF103,04,00,01,01*21"); //Abilita RMC 1hz
        Serial1.println("$PSRF103,05,00,01,01*20"); //Abilita VTG 1hz
    }
    else if (_type==2){
        Serial.println("Not supported for this GPS, use writeGPS function");
        if (_freq==1) Serial1.println("$PMTK220,1000*1F");//1hz
        else if (_freq==5)Serial1.println("$PMTK220, 200*2C");//5hz
        else if (_freq==10)Serial1.println("$PMTK220, 100*2F");//10hz
        
        Serial1.println("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28");//RMC every 2 position fix
        //string-position(gll,rmc,vtg,gga,gsa,gsv), position-fix-numers(0,1,2,3,4,5)
    }
    else if (_type==3){
        Serial1.println("$PUBX,40,GGA,0,1,0,0*5B");
        Serial1.println("$PUBX,40,GLL,0,1,0,0*5D");
        Serial1.println("$PUBX,40,GSA,0,1,0,0*4F");
        Serial1.println("$PUBX,40,GSV,0,1,0,0*58");
        Serial1.println("$PUBX,40,RMC,0,1,0,0*46");
        Serial1.println("$PUBX,40,VTG,0,1,0,0*5F");
        Serial1.println("$PUBX,40,GRS,0,1,0,0*5C");
        Serial1.println("$PUBX,40,GST,0,1,0,0*5A");
        Serial1.println("$PUBX,40,ZDA,0,1,0,0*45");
    }
    
    Serial.println("GPS setted");
}

boolean GPS::readGPS(float* vel, unsigned long* gradi, unsigned long* date, unsigned long* times, long* lat, long* lon){
    if (Serial1.available()){  //controlla la seriale 1
        char c = Serial1.read();
        //Tiny_GPS
        if (encode(c)){
            //_vel = f_speed_knots();
            *vel = f_speed_kmph();
            get_position (lat, lon);
            *gradi= course();
            get_datetime(date, times);
            
            return 1;
            
        }//end_encode
        //else return 0;
    }//end_available
}



//WIND CLASS
WIND::WIND(uint8_t ce, uint8_t cs, uint64_t out, uint64_t in): RF24(ce,cs){
    if(out){
        _addr[0] = out;
    }

    if(in){
        _addr[1] = in;
    }
}

void WIND::init(){
    begin();
    setRetries(15,15);
    if (_addr[0]){
        openWritingPipe(_addr[0]);
    }
    if (_addr[1])
    {
        openReadingPipe(1, _addr[1]);
    }
}

bool WIND::receive(wind_t* buff){
    bool flag = 0;
    if (available()){
        flag = read(&buff, sizeof(wind_t));
    }
    return flag;
}

bool WIND::send(wind_t* buff){
    stopListening();
    bool flag = write(&buff, sizeof(wind_t));
    startListening();
    return flag;
}

uint64_t WIND::getAddress(bool i){
    return _addr[i];
}


//TEMP CLASS
TEMP::TEMP(byte pin): OneWire(pin){};
bool TEMP::check(){
    reset_search();  //resetta la ricerca in modo che al ciclo successivo search trovi ancora la stessa periferica
    _addr[0]=0x00;
    for (byte a=0; a<10; a++){//Controlla al massimo tra 10 periferiche
        if (!search(_addr)) {
            //no more sensors on chain, reset search
            reset_search();
            _check = 0;
            Serial.println("No OW sensor detected");
            
        }
        if ( OneWire::crc8(_addr, 7) == _addr[7]) {//Se il CRC è valido
            if (_addr[0] != 0x10 && _addr[0] != 0x28) {//Non è un DS18B20
                Serial.println("No DS18B20 recognized");
                _check = 0;
            }
        }
        else{
            _check = 1;
            Serial.println ("DS18B20 ok");
        }//end else
    }//end for
}

float TEMP::getTemp(){
    byte data[12];
    reset();
    select(_addr);
    write(0x44,1); // start conversion, with parasite power on at the end
    delay(600);
    byte present = reset();
    select(_addr);
    write(0xBE); // Read Scratchpad
    
    
    for (byte i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = read();
    }
    
    reset_search();
    
    byte MSB = data[1];
    byte LSB = data[0];
    
    return float(((MSB << 8) | LSB))/16; //using two's compliment
    
    
}


//ALTRE FUNZIONI SENZA CLASSE

uint8_t getCheckSum(char *string)
{
    int XOR = 0;
    for (int i = 0; i < strlen(string); i++)
    {
        XOR = XOR ^ string[i];
    }
    return XOR;
}


//void printSerial overload funzioni
void printSerial(char** string, int x){
    for (int i=0; i<x; i++){
        Serial.print(string[i]);
        Serial.print('\t');
    }
}
void printSerial(int* array, int x){
    for (int i=0; i<x; i++){
        Serial.print(array[i]);
        Serial.print('\t');
    }
}

void serialRaw(float* acc, float* gyr, float* mag){
    Serial.print(acc[0]);Serial.print('\t');
    Serial.print(acc[1]);Serial.print('\t');
    Serial.print(acc[2]);Serial.print('\t');
    Serial.print(gyr[0]);Serial.print('\t');
    Serial.print(gyr[1]);Serial.print('\t');
    Serial.print(gyr[2]);Serial.print('\t');
    Serial.print(mag[0]);Serial.print('\t');
    Serial.print(mag[1]);Serial.print('\t');
    Serial.print(mag[2]);Serial.print('\n');
}

void serialAttitude(float* attitude, boolean i){
    if (i){
        Serial.print('\n');
        Serial.print("yaw");Serial.print('\t');
        Serial.print("pitch");Serial.print('\t');
        Serial.print("roll");Serial.print('\n');
    }
    Serial.print(attitude[0]);Serial.print('\t');
    Serial.print(attitude[1]);Serial.print('\t');
    Serial.print(attitude[2]);Serial.print('\n');
}

boolean serialGPS(float vel, unsigned long gradi, unsigned long date, unsigned long times, long lat, long lon, boolean i){
    if (i){
        Serial.print('\n');
        Serial.print("speed");Serial.print('\t');
        Serial.print("course");Serial.print('\t');
        Serial.print("lat");Serial.print('\t');
        Serial.print("lon");Serial.print('\t');
        Serial.print("times");Serial.print('\t');
        Serial.print("date");Serial.print('\n');
    }
    Serial.print(vel);Serial.print('\t');
    Serial.print(gradi);Serial.print('\t');
    Serial.print(lat);Serial.print('\t');
    Serial.print(lon);Serial.print('\t');
    Serial.print(times);Serial.print('\t');
    Serial.print(date);Serial.print('\n');
}
