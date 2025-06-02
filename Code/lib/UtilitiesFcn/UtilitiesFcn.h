#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include <Arduino.h>
#include <EEPROM.h>



float matLinInt(float, float, float, float, float);
void setupWiFi();
void setup_rtc_time(ESP32Time *);
void check_DST(ESP32Time *);
bool isLegalTime(int , int , int , int );


void do_eprom_read();
void do_eprom_write();

typedef struct {
  String Eeprom_sData;  
  String Eeprom_sArray[4];          
  bool Eeprom_bArray[4];    

  bool Eeprom_bData1;
  bool Eeprom_bData2;
  bool Eeprom_bData3;    
  int Eeprom_iData1;        // 
  int eprom_good;        // check eeprom integrity
} Eeprom_Data_Type;

extern Eeprom_Data_Type eepromData;


class Led {
  private:
    int sts;
    int luminosity = 60; 

  public:
    byte pin;
    Led(byte pin); // costruttore

    void init();
    void init(byte defaultState);

    void on();
    void off();

    void red();
    void green();
    void blue();

    void toggle();
    
    int pinStatus();
};

#endif
