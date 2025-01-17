#include <ESP32Time.h>
#include <WiFiClientSecure.h>

#include "UtilitiesFcn.h"
#include "../../src/secret.h"
/*
Eeprom_Data_Type eepromData = {
      String(""),
      { String("CHAT_ID_1"), String("CHAT_ID_2"), String(""), String("") },
      { 1, 0, 0, 0 },
      false,                // Boolean can bee true or false, or LOW or HIGH
      false,
      LOW, 
      10,      
      14   
};
*/

void setupWiFi() {

  WiFi.setSleep(false); 
  WiFi.setAutoReconnect(true);

  // Connect to WiFi network
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  log_d("Connected to %s\t IP address: %s", ssid, WiFi.localIP().toString().c_str());

}


void setup_rtc_time(ESP32Time *rtc){
    // Configurazione del fuso orario
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;

    if (!getLocalTime(&timeinfo)) {
        if (!getLocalTime(&timeinfo)){
            log_e("Impossibile ottenere l'orario");
            return;
        }
    }

    rtc->setTime(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour+1, timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);

    check_DST(rtc);

}

void check_DST(ESP32Time *rtc) {
  // Regole per l'ora legale in Europa
  static bool firstTimeHere = true;
  static bool SolarTime = true; // variabile che indica se sono con l'orario solare o legale

  // L'ora legale inizia l'ultima domenica di marzo alle 2:00 AM e termina l'ultima domenica di ottobre alle 3:00 AM
  tm dt = rtc->getTimeStruct();

  int month = dt.tm_mon + 1;
  int day = dt.tm_mday;
  int hour = dt.tm_hour;
  int dayOfWeek = dt.tm_wday;
  
  log_d("Month: %d", month);
  log_d("Day: %d", day);
  log_d("Hour: %d", hour);
  log_d("DayOfWeek: %d", dayOfWeek);
  /*
    Serial.println();   // (String) returns time with specified format
    Serial.println("day: " + String(day));   // (String) returns time with specified format
    Serial.println("hour: " + String(hour));   // (String) returns time with specified format
    Serial.println("dayOfWeek: " + String(dayOfWeek));   // (String) returns time with specified format
  */


  if (month == 3) {
    log_d("month == 3: ");
    bool lastSunday = (31 - day)< 7;
    if ((lastSunday && dayOfWeek == 0 && hour >= 2)  && (SolarTime == false || firstTimeHere == true)) {
      // imposto l'ora legale facendo il tempo attuale piu 1h in millisecond
      
      // Serial.println("epoch time: " + String(rtc->getEpoch())); // (String) returns time
      rtc->setTime(rtc->getEpoch()+(60*60)); 
      // variabile che indica se sono con l'orario solare o legale
      SolarTime = true; 
      firstTimeHere = false;
    }
  } else if (month == 10) {
    log_d("month == 10: ");
    // Calcola l'ultima domenica di ottobre
    bool lastSunday = (31 - day)< 7;
    // Serial.println("lastSunday: " + String(lastSunday));   // (String) returns time with specified format
    
    if ( (lastSunday && dayOfWeek == 0 && hour >= 3)  && (SolarTime == true || firstTimeHere == true)) {
      log_d("month == 10: ");
      // imposto l'ora legale facendo il tempo attuale piu 1h in millisecond
      rtc->setTime(rtc->getEpoch()-(60*60)); 
      // variabile che indica se sono con l'orario solare o legale
      SolarTime = false; 
      firstTimeHere = false;
    }
  }

}

// FUNZIONI RELATIVE ALLA EEPROM
void do_eprom_read() {

  EEPROM.begin(200);
  EEPROM.get(0, eepromData);

  Serial.print("EEPROM STS_Before: ");
  Serial.println(eepromData.eprom_good);

  if (eepromData.eprom_good == 14) {
    Serial.println("Good settings in the EPROM ");
    Serial.print("EEPROM STS: ");
    Serial.println(eepromData.eprom_good);
    // eepromData.Eeprom_bArray[i] // go access to information inside the struct

  } else {
    Serial.println("EPROM canot be read ");
    eepromData = {
      String(""),
      { String("CHAT_ID_1"), String("CHAT_ID_2"), String(""), String("") },
      { 1, 0, 0, 0 },
      false,                // Boolean can bee true or false, or LOW or HIGH
      false,
      LOW, 
      10,      
      14   
    };
    do_eprom_write();
  }
}

void do_eprom_write() {
  Serial.println("Writing to EPROM ...");
  EEPROM.begin(200);
  EEPROM.put(0, eepromData);
  EEPROM.commit();
  EEPROM.end();
}


// Funzioni relative al led

Led::Led(byte pin){
  this->pin = pin;
}

void Led::init(){
  pinMode(pin, OUTPUT);
  off();
}

void Led::init(byte defaultState){
  pinMode(pin, OUTPUT);
  if (defaultState==HIGH){
    on();
  }else{
    off();
  }
}

void Led::on() {
  neopixelWrite(pin,64,64,64);
  sts = 1;
}

void Led::off() {
  neopixelWrite(pin,0,0,0);
  sts = 0;
}

void Led::toggle(){
  int sts_led = pinStatus();
  if (sts_led == 1){
    off();
  }else{
    on();
  }  
}

void Led::red(){
  neopixelWrite(pin,luminosity,0,0);
}
void Led::green(){
  neopixelWrite(pin,0,luminosity,0);
}
void Led::blue(){
  neopixelWrite(pin,0,0,luminosity);
}

int Led::pinStatus(){
  return sts;
}