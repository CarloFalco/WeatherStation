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


float matLinInt(float x1, float x2, float y1, float y2, float x0){
  return y1 + (x0 - x1) * (y2 - y1) / (x2 - x1);
}

/** * @brief Configura la connessione WiFi.
 * 
 * Questa funzione si occupa di configurare la connessione WiFi del dispositivo.
 * Inizializza il WiFi, disabilita il sonno e imposta la riconnessione automatica.
 * Quindi, avvia la connessione al network specificato e attende fino a quando non è connesso.
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


// FUNZIONI RELATIVE ALLA RTC

/**
 * @brief Configura l'orologio in tempo reale (RTC) e imposta il fuso orario.
 * 
 * Questa funzione configura l'orologio in tempo reale utilizzando NTP per ottenere l'ora corrente.
 * Imposta il fuso orario e verifica se è necessario applicare l'ora legale.
 * 
 * @param rtc Puntatore all'oggetto ESP32Time che rappresenta l'orologio in tempo reale.
 */
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


/**
 * @brief Controlla se è necessario applicare l'ora legale (DST) e regola l'orario di conseguenza.
 * 
 * Questa funzione verifica se l'ora corrente è in ora legale o solare e regola l'orario di conseguenza.
 * Le regole per l'ora legale in Europa sono:
 * - Inizio: ultima domenica di marzo alle 2:00 AM
 * - Fine: ultima domenica di ottobre alle 3:00 AM
 * 
 * @param rtc Puntatore all'oggetto ESP32Time che rappresenta l'orologio in tempo reale.
 */
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

  if (firstTimeHere && isLegalTime(month, day, hour, dayOfWeek)){
    SolarTime = false; 
    firstTimeHere = false;
    rtc->setTime(rtc->getEpoch()+(60*60));
  }


  if (month == 3) {
    log_d("month == 3: ");
    bool lastSunday = (31 - day)< 7;

    if ((lastSunday && dayOfWeek == 0 && hour >= 2  && SolarTime == true)) {
      // imposto l'ora legale facendo il tempo attuale piu 1h in millisecond
  
      // Serial.println("epoch time: " + String(rtc->getEpoch())); // (String) returns time
      rtc->setTime(rtc->getEpoch()+(60*60)); 
      // variabile che indica se sono con l'orario solare o legale
      SolarTime = false; 
    }

  } else if (month == 10) {
    log_d("month == 10: ");
    // Calcola l'ultima domenica di ottobre
    bool lastSunday = (31 - day)< 7;
    // Serial.println("lastSunday: " + String(lastSunday));   // (String) returns time with specified format
    
    if ( (lastSunday && dayOfWeek == 0 && hour >= 3 && SolarTime == false)) {
      log_d("month == 10: ");
      // imposto l'ora legale facendo il tempo attuale piu 1h in millisecond
      rtc->setTime(rtc->getEpoch()-(60*60)); 
      // variabile che indica se sono con l'orario solare o legale
      SolarTime = true; 
    }
  }
}

/**
 * @brief Verifica se l'ora è legale in base al mese, giorno, ora e giorno della settimana.
 * 
 * Questa funzione determina se l'ora corrente è legale in base alle regole dell'ora legale.
 * Le regole sono:
 * - L'ora legale inizia l'ultima domenica di marzo alle 2:00 AM.
 * - L'ora legale termina l'ultima domenica di ottobre alle 3:00 AM.
 * 
 * @param month Mese corrente (1-12).
 * @param day Giorno del mese (1-31).
 * @param hour Ora del giorno (0-23).
 * @param dow Giorno della settimana (0=Sunday, 1=Monday, ..., 6=Saturday).
 * @return true Se l'ora è legale.
 * @return false Se l'ora non è legale.
 */
bool isLegalTime(int month, int day, int hour, int dow) {

  // Calcola il giorno della prima domenica di marzo
  int lastSundayMarch = 31 - dow; // valido solo se chiamato a marzo

  // Calcola il giorno della prima domenica di ottobre
  int lastSundayOctober = 31 - dow; // valido solo se chiamato a ottobre


  if (month < 3 || month > 10) {
    return false; // inverno
  }

  if (month > 3 && month < 10) {
    return true; // estate
  }

  if (month == 3) {
    if (day > lastSundayMarch) return true;
    if (day < lastSundayMarch) return false;
    return hour >= 3; // proprio il giorno del cambio
  }

  if (month == 10) {
    if (day < lastSundayOctober) return true;
    if (day > lastSundayOctober) return false;
    return hour < 2; // fino alle 2:59 è ancora ora legale
  }

  return false;

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
void Led::yellow() {
  neopixelWrite(pin, luminosity, luminosity, 0); // rosso + verde
}
void Led::cyan() {
  neopixelWrite(pin, 0, luminosity, luminosity); // verde + blu
}
void Led::magenta() {
  neopixelWrite(pin, luminosity, 0, luminosity); // rosso + blu
}
void Led::orange() {
  neopixelWrite(pin, luminosity, luminosity / 2, 0); // arancione
}
void Led::purple() {
  neopixelWrite(pin, luminosity / 2, luminosity, 0); // arancione
}
void Led::setColor(String colorName) {
  colorName.toLowerCase();

  if (colorName == "red") {
    neopixelWrite(pin, luminosity, 0, 0);
  } else if (colorName == "green") {
    neopixelWrite(pin, 0, luminosity, 0);
  } else if (colorName == "blue") {
    neopixelWrite(pin, 0, 0, luminosity);
  } else if (colorName == "yellow") {
    neopixelWrite(pin, luminosity, luminosity, 0);
  } else if (colorName == "cyan") {
    neopixelWrite(pin, 0, luminosity, luminosity);
  } else if (colorName == "magenta") {
    neopixelWrite(pin, luminosity, 0, luminosity);
  } else if (colorName == "white") {
    neopixelWrite(pin, luminosity, luminosity, luminosity);
  } else if (colorName == "off") {
    neopixelWrite(pin, 0, 0, 0);
  } else if (colorName == "orange") {          // arancione
    neopixelWrite(pin, luminosity, luminosity / 2, 0);
  } else if (colorName == "lime") {            // verde-lime
    neopixelWrite(pin, luminosity / 2, luminosity, 0);
  } else if (colorName == "purple") {          // viola scuro
    neopixelWrite(pin, luminosity, 0, luminosity / 2);
  } else {
    ;
  }
}







int Led::pinStatus(){

  return sts;
}





