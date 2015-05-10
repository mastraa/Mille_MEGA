
#include <Mille_UNO.h>
#include <SD.h>
#include <SPI.h>

#define NOMEFILE            "vela000.txt"
#define FIRST_NUM            4


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  SDCARD sd (10);
  sd.init();
  sd.setName(NOMEFILE);
}

void loop() {
  sd.openFile('w');//opening file in write mode
  sd.closeFile(); //close file
  
  sd.setName( sd.getFreeName (NOMEFILE, 4, 1)); //set the first free name
  char* prova =sd.getName(); //get the name
  Serial.println(prova);

  delay(500);
}
