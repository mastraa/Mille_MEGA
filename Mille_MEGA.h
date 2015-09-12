//=====================================================================================================
// Mille_MEGA.h
//Last Update: 18/8/2015
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


#ifndef Mille_MEGA_h
#define Mille_MEGA_h
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


//Constant definitions
#define RADTODEG       57.295780f
#define DEGTORAD        0.017453293f

#define MPU6050_ad      0x68
#define MPU_SMPRT_DIV   0x19   //MPU6050 sample rate register
#define MPU_DLPF_CONF   0x1A   //MPU6050 digital low-pass filter cofig
#define MPU_GYRO_CONF   0x1B   //MPU6050 gyro configuration register
#define MPU_ACC_CONF    0x1C   //MPU6050 accelerometer config register
#define MPU_PWR_MGMT_1  0x6B   //MPU6050 power control register 1
#define MPU_PWR_MGMT_2  0x6C   //MPU6050 power control register 2

#define HMC5883L_ad    0x1E
#define HMC_CONF_A     0x00  //HMC5883L config register A
#define HMC_CONF_B     0x01  //HMC5883L config register B
#define HMC_MODE       0x02  //HMC5883L mode register

#define INT_EM406A      75
#define INT_GTOP        101
#define INT_UBLOX       75


// STRUCTURE DEFINITIONS
struct Mvupc_t //NMEA: Mille&una Vela UniPd Corta
{
    long lat,lon;
    unsigned long gradi, date, times;
    float vel, attitude[3];
};

struct Mvup_t //NMEA: &una Vela UniPd
{
    long lat,lon;
    unsigned long gradi, date, times;
    float vel, attitude[3], tempDS, left, right;
    byte Wspeed, vale_1, vale_2;
};

struct wind_t //Wifi wind datas
{
    byte speed;
    int dir_1;
    int dir_2;
};



class Secure{
    /*Security function:
     set a pin connected with a relay that command battery line
     when you call secureOff it will interrupt line
     */
public:
	Secure(byte pin);	//battery line relay pin-command
						//High level of pin-command
	void secureOff();	//Low level of pin-command
	
private:
	int _off;			//pin-command buffer
};


class I2C{
public: //Write and Read on I2C sensors
	I2C(byte address);
	void writeTo (byte reg_address, byte val);
	void readFrom (byte reg_address, byte num, byte *buff);
    void readFrom (byte num, byte *buff);
    
protected:
    byte _ADDR;
	
};


class MPU6050: public I2C
{
public:
    MPU6050(byte a0);
    void readMPU(float *acc, float *gyr); //to get raw values
    void readMPU(float offAcc[3], float gainAcc[3][3], float offGyr[3], float gainGyr[3][3], float *acc, float *gyr); //to get offsetted values
    void offsetGyr(int samples, float *offGyr); //set offset Gyro value
    void setMPU(byte smprt_div, byte dlpf_conf, byte gyro_conf, byte acc_conf, byte pwr_mgmt_1, byte pwr_mgmt_2); //set mpu register, parameter are bytes to set, not register address. This function use more memory than manual setting with writeTo and label for register address.
private:
    byte _mpuAcc;
    byte _mpuTemp;
    byte _mpuGyr;
};

class HMC5883L: public I2C
{
public:
    HMC5883L();
    void readHMC(float *mag); //to get raw values
    void readHMC(float offMag[3], float gainMag[3][3], float *mag); //to get offsetted values
    void setHMC(byte mode, byte conf_a, byte conf_b);
private:
    byte _hmcMag;
};

class ATTINY: public I2C
{
public:
    ATTINY(byte address);
    void readTiny(byte num, float *wind);
private:
    long _time;
    
};

class AHRSFILTER
{
public:
    AHRSFILTER();
    void start();
    void Filter(float gyr[3], float acc[3], float mag[3], float BETA, float *attitude);
private:
    long _time;
    float q0, q1, q2, q3;
};


class SDCARD: public File{
public:
    SDCARD(byte pin);
    SDCARD();
    void init();
    char* getName();
    char* getFreeName(char* name, byte i, boolean n); //nome, posizione primo numero, numeri 0-->2, 1-->3;
    void setName(char* name); //to call only after init();
    void openFile();
    void openFile(char s);
    void closeFile();
    //It will print all components of the array
    //Example calling function: printFile(prova, sizeof(prova)/sizeof(char*));
    void writeFile(char val);
    void printFile(int val);
    void printFile(unsigned int val);
    void printFile(long val);
    void printFile(unsigned long val);
    void printFile(char* string);
    void printFile(char** string, int x);
    void printFile(int* array, int x);
    void printFile(float* array, int x);
    //creare funzione che passi la struttura voluta
    //$MVUPC, $MVUPR
    void newLineFile();
    void tabFile();
    void readFile();
    void printMVUPC(struct Mvupc_t mvupc);
    void printMVUP(struct Mvup_t mvup);
    
    char* publicName;
    
private:
    char* _name;
    byte _pin;
    File _file;
};


//LED CLASS
class LED //va controllata nel loop generale per vedere i delay 'imposti'
{
public:
    LED(byte pin);
    void onOff();
private:
    byte _pin;
    boolean _state;
};


//LCD CLASSES
class LCD_I2C: public LiquidCrystal_I2C
{
public:
    LCD_I2C(byte address, byte col, byte row);
    void start();
    void Attitude (float* attitude);
    void AttitudeShort (float* attitude, char* string);
};

class LCD_CLASSIC: public LiquidCrystal
{
public:
    LCD_CLASSIC(byte rs, byte enable, byte d4, byte d5, byte d6, byte d7, byte col, byte row);
    void start();
    void Attitude (float* attitude);
    void AttitudeShort (float* attitude, char* string);
private:
    byte _col;
    byte _row;
};

//GPS CLASS
class GPS: public TinyGPS
{
public:
    GPS(byte type);
    GPS(byte type, uint8_t freq);
    unsigned int begin();
    unsigned int begin(unsigned int baudrate);
    void autoset();//standard scelta da me
    void autoset(bool sentences[]);//non valida per Gtop
    void autosetAll();
    void writeGPS(char* string);
    void baudChange(unsigned int baudrate);
    boolean readGPS(float* vel, unsigned long* gradi, unsigned long* date, unsigned long* times, long* lat, long* lon);
private:
    byte _type;
    uint8_t _freq;
};

//WIND CLASS
class WIND: public RF24
{
public:
    WIND(uint8_t ce, uint8_t cs, uint64_t out, uint64_t in); //to open writing and reading pipe
    void init();
    bool receive(wind_t* buff);
    bool send(wind_t* buff);
    uint64_t getAddress(bool i); //0: OUT, 1 IN
private:
    uint64_t _addr[2]; //reading address
};

class TEMP: public OneWire
{
public:
    TEMP(byte pin);
    bool check();
    float getTemp();
private:
    byte _addr[8];
};


//ALTRE FUNZIONI SENZA CLASSE

uint8_t getCheckSum(char *string);
uint8_t getCheckSum(byte *buff, int l)

//It will print all components of the array
void serialRaw(float* acc, float* gyr, float* mag);
void serialAttitude(float* attitude, boolean i);
boolean serialGPS(float vel, unsigned long gradi, unsigned long date, unsigned long times, long lat, long lon, boolean i);
void printMVUP(struct Mvup_t mvup);
void printFPVMVUP(struct Mvup_t mvup, byte mil);
void printMVUPC(struct Mvupc_t mvup);


#endif

//=====================================================================================================
// End of file
//=====================================================================================================