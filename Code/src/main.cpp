// #define ENABLE_ESP32_GITHUB_OTA_UPDATE_DEBUG // Uncomment to enable logs.

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Time.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#include <Adafruit_BME280.h>
#include <Adafruit_INA3221.h>
//#include "MICS6814.h"
//#include "SparkFunCCS811.h"
//#include "Adafruit_PM25AQI.h"


// My library 
#include "documentation.h"
#include "secret.h"
#include "UtilitiesFcn.h"



#include "HttpReleaseUpdate.h"
#include "ESP32GithubOtaUpdate.h"
#include "ESP32Mqtt.h"

#include "AllSensor.h"

#include "configuration.h"

/** 
 * @file main.cpp
 * @brief Descrizione generale del programma.
 * 
 * @mainpage Documentazione del Progetto Arduino
 * 
 * @section intro Introduzione
 * Questo è un progetto Arduino sviluppato con PlatformIO.
 * 
 * @section struttura Struttura del Codice
 * - `src/` contiene il file `main.cpp`
 * - `lib/` contiene librerie custom
 * - `include/` contiene gli header globali
*
 * @section TASK

 * @subsection Aggiornamento Via GIT
 * 
 * @subsection Creaziione delle funzioni legate al risveglio della centralina   
 * Stato: **In sviluppo**  
 * Completamento: **50%**  
 * `[====>     ]`
 * 
 *   - 1: Risveglio per pioggia
 * 
 *   - 2: Risveglio per mancanza di alimentazione 
 *     - avvio tramite pagina web, se non trovo gia le credenziali
 *     - impostazione tempo e fuso orario (possibile bug legato al orario primo risveglio)                                 
 *     
 *   - 3: Risveglio per normale funzionamento
 * 
 * 
 * @subsection salvataggio informazioni su eeprom        
 * Stato: **In sviluppo**  
 * Completamento: **50%**  
 * `[====>     ]`                         
 *  - 1: credenziali wif
 * 
 * 
 * 
 * @subsection Sensori stazione meteo:
* Stato: **In sviluppo**  
 * Completamento: **50%**  
 * `[====>     ]`  
 *     - 1: direzione vento                                 `[=>        ]`
 *     - 2: intensita vento                                 `[=>        ]`
 *     - 3: precipitazioni                                  `[=>        ]`
 *     - 4: temperatura e umidita                           `[=>        ]`
 *     - 5: umidita terreno                                 `[=>        ]`
 *     
 * 
*/




Led led(LED_BUILTIN); // #define LED_BUILTIN 97

// Variabili globali per la gestione del risveglio
RTC_DATA_ATTR int wakeUpCount = 0;
RTC_DATA_ATTR uint8_t needsToStayActive = 0; // probabilmente duplicato // TODO: verificare
RTC_DATA_ATTR unsigned long wakeUpPreviousTime = 0;
RTC_DATA_ATTR update_status_t avblUpdate = NO_UPDATE; // -> questa variabile dovrebbe essere mantenuta tra un accensione e l'altra
RTC_DATA_ATTR bool rqtUpdate = false;
RTC_DATA_ATTR int debugCount = 0;

bool needToStayAlive = 0;



ESP32GithubOtaUpdate otaUpdate;
WiFiClientSecure espClient;
PubSubClient mqtt_client(espClient);
ESP32Time rtc(0);  // Dichiarazione di rtc

// SENSORI
INA3211 ina;
Adafruit_BME280 bme; 
Windvane windvane(AS5600_ADDRESS);


// Definizione dei TASK
TaskHandle_t task1Handle = NULL;



void setup() {

  // pin configuration
  pinMode(PIN_ANEMOMETER, INPUT); // pinMode(PIN_ANEMOMETER, INPUT_PULLDOWN);
  pinMode(PIN_RAINGAUGE, INPUT);

  pinMode(PIN_5V, OUTPUT);
  pinMode(PIN_3V, OUTPUT);

  led.init();
  debugCount ++;

  Serial.begin(115200);
  delay(100);




  log_i("WelcomeToNewBuild");
  log_d("Wakeup Count: ", String(wakeUpCount));  


  // scopro il motivo del risveglio
  esp_sleep_wakeup_cause_t wakeUpRsn = esp_sleep_get_wakeup_cause();  

  log_d("wakeup reason num: ", String(wakeUpRsn));

  // se mi sono svegliato per un interrupt di un pin (vale sia il primo metodo che il secondo)
  if (wakeUpRsn == ESP_SLEEP_WAKEUP_EXT0 || wakeUpRsn == ESP_SLEEP_WAKEUP_EXT1) {WakeUp_Interrupt();}

  // se mi sono svegliato causa mancanza di batteria riinizializzo l'rtc
  if (wakeUpRsn == ESP_SLEEP_WAKEUP_TIMER) {WakeUp_Timer();}

  // se mi sono svegliato causa mancanza di batteria riinizializzo l'rtc
  if (wakeUpRsn == ESP_SLEEP_WAKEUP_UNDEFINED) {WakeUp_PowerLoss();}


}

void loop(){

  if (needsToStayActive == 0 && avblUpdate != UPDATE_GOING){
  esp_sleep_wakeup_cause_t wakeUpRsn = esp_sleep_get_wakeup_cause();  
  uint64_t timeToNextWakeUp = TIME_TO_SLEEP * S_TO_uS_FACTOR;
    if (wakeUpRsn == ESP_SLEEP_WAKEUP_EXT0 || wakeUpRsn == ESP_SLEEP_WAKEUP_EXT1) {
      delay(1000);
      log_d("[Interrupt]: wakeUpPreviousTime: %s", String(wakeUpPreviousTime));
      unsigned long tmp_1 = rtc.getEpoch() - wakeUpPreviousTime;
      unsigned long tmp_2 = tmp_1 * S_TO_uS_FACTOR;
      log_d("[Interrupt]: timeToNextWakeUp: %s\t tmp_1: %s\t tmp_2: %s", String(timeToNextWakeUp), String(tmp_1), String(tmp_2));

      timeToNextWakeUp = timeToNextWakeUp - tmp_2;
      log_d("[Interrupt]: timeToNextWakeUp: %s", String(timeToNextWakeUp));
      led.off();
    }

    if (wakeUpRsn == ESP_SLEEP_WAKEUP_TIMER) {

      vTaskDelete(task1Handle);
      task1Handle = NULL;  // Resetta l'handle dopo l'eliminazione
      wakeUpCount = 0;
      wakeUpPreviousTime = rtc.getEpoch() ;
      log_d("[Timer]: wakeUpPreviousTime: %s", String(wakeUpPreviousTime));

      String dataTime = rtc.getTime("%A, %B %d %Y %H:%M:%S");
      log_i("%s", dataTime.c_str());

      led.off();
    }
    
    if (wakeUpRsn == ESP_SLEEP_WAKEUP_UNDEFINED){
      
      String dataTime = rtc.getTime("%A, %B %d %Y %H:%M:%S");
      log_i("%s", dataTime.c_str());

      wakeUpPreviousTime = rtc.getEpoch();
      
    }

    esp_sleep_enable_ext1_wakeup((1ULL << WAKEUP_PIN_1) | (1ULL << WAKEUP_PIN_2), ESP_EXT1_WAKEUP_ANY_LOW);
    // Configuriamo il wakeup tramite timer

    esp_sleep_enable_timer_wakeup(timeToNextWakeUp);

    log_i("Entrando in modalità sleep...");
    led.off();
    // Se la seriale è attiva, aspettiamo che sia vuota prima di entrare in sleep
    if(Serial) {
      Serial.flush();
      }
    // Entriamo in modalità deep sleep
    esp_deep_sleep_start();
  }


}


void WakeUp_PowerLoss(void){

  log_d("[WakeUp_PowerLoss()]: Wake Up caused by powerloss");
  setupWiFi(); // setup wifi connection
  // setup time
  setup_rtc_time(&rtc);
  debugCount = 0; // resetto il contatore dei debug

  String dataTime = rtc.getTime("%A, %B %d %Y %H:%M:%S");
  log_i("%s", dataTime.c_str());

}

void WakeUp_Interrupt(void){
  log_d("[WakeUp_Interrupt()]: Wake Up caused by interrupt");
  wakeUpCount ++;
  log_d("[WakeUp_Interrupt()]: Set led to RED");
  led.red();
}

void WakeUp_Timer(void){
  log_d("[WakeUp_Timer()]: Wake Up caused by timer");
  setupWiFi(); 
  mqtt_init();
  otaUpdate.checkOTAOnce();
  xTaskCreate(led_blink_task, "LED blink task", 4096, NULL, 1, &task1Handle);   // in questo punto devo andarmi a definire tutti i task
  needsToStayActive = 1;

}




void led_blink_task(void* pvParameters) {  
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_FAST);
  int count_iter = 0;
  while (true) {
    // Esegui il codice del task 1
    mqtt_client.loop();

    // led.toggle();
    // led.blue();
    count_iter ++;
    log_i("count_iter: %d", count_iter);

    switch (count_iter) {
      case 1: { // primo giro accendo i pin 5V e 3V
        log_d(" turn on 5V and 3V");
        digitalWrite(PIN_5V, HIGH);  
        digitalWrite(PIN_3V, HIGH);
        break;
      }
      case 3: {
        led.orange();
        break;
      }
      case 5: {
        led.purple();
        break;
      }

      case 8: {
        led.red();
        log_d("Setup all sensors");
        // --------- SETUP INA3221 ---------- //
        ina.begin(INA_ADDRESS);  
        // --------- SETUP BME280 ---------- //
        if (!bme.begin(BME280_ADDRESS)) {
          log_w("Could not find a valid BME280 sensor, check wiring!");
          while (1);
        }

        break;
      }
      case 9: {
        led.green();
        log_d("Print Sensor");
        to_serial();
        break;
      }
      case 10: {
        led.blue();
        log_d("Update is avaiable");
        // String payload = "{\"msg\": \"pippo\", \"Value\": \"" + String(avblUpdate) + "\"}";
        String payload = "{\"IterationCount\":" + String(debugCount) + ", \"UpdateAvaiable\": \"" + String(avblUpdate) + "\"}";
        mqtt_client.publish("upd_avbl", payload.c_str(), 1);  
        log_d("msg: %d", avblUpdate);

        
        break;
      }
      case 12: {
        // --------- JSON ---------- //
        led.yellow();
        log_d("INVIO DEL MESSAGGIO JSON");
        char json_data_string[MAX_JSON_SIZE];  
        to_json(json_data_string);
        //Serial.println("[DEBUG] Contenuto JSON:");

        //Serial.println(json_data_string);  // <-- QUI visualizzi il contenuto serializzato
        log_d("JSON: %s", json_data_string);

        bool published = mqtt_client.publish("TESTWS", (const char *) json_data_string, 1);
        log_d("Publish JSON: %s", published ? "Success" : "Failed");
        if (!published) {
          bool published = mqtt_client.publish("TESTWS", (const char *) json_data_string, false);        
          log_d("Publish JSON 2 Test: %s", published ? "Success" : "Failed");

          log_w("Publish fallito, stato MQTT: %d", mqtt_client.state());
        }
        if (!published) {
          log_w("Publish fallito, stato MQTT: %d", mqtt_client.state());
        }



        break;
      }
      
      case 20: {
        led.cyan();
        log_d("we can turn off 5V and 3V");
        digitalWrite(PIN_5V, LOW);
        digitalWrite(PIN_3V, LOW);
        needsToStayActive = 0;
        break;
      }
      default:
        // do nothing
        break;
    }

    
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void to_serial(void) {
  log_i("-------------------------------");
  log_i("BME280 Temperature: %s [*C]", String(bme.readTemperature(), 1));
  log_i("BME280 Humidity: %s [%%]", String(bme.readHumidity(), 2));
  log_i("BME280 Pressure: %s [hPa]", String(bme.readPressure()/ 100.0F, 2));
  log_i("BME280 Altitude: %s m",  String(bme.readAltitude(1013.25))); // 1013.25 hPa is the standard sea level pressure
  log_i("-------------------------------");  
  log_i("Direzione del vento: %s []", windvane.getDirection());
  log_i("Velocità del vento: %s [*]", String(windvane.getWindAngle()));
  log_i("-------------------------------");  
  log_i("INA3221 Battery Voltage: %s [V]", String(ina.getBusVoltage(2)));
  log_i("INA3221 Battery Current: %S [mA]", String(ina.getCurrentAmps(2)* 1000.0F)); // Converti da A a mA
  log_i("INA3221 Battery Power: %S [W]", String(ina.getPower(2)));
  log_i("INA3221 Battery SoC: %s [%%]", String(ina.vbToSoc(ina.getBusVoltage(2) * 1000.0F))); // Converti da V a mV
  log_i("-------------------------------");
  // Aggiungi qui altre letture dei sensori se necessario
}


void to_json(char * json){

DynamicJsonDocument doc(MAX_JSON_SIZE);


JsonObject epoc = doc.createNestedObject("epoc");
epoc["Giorno"] = rtc.getDay();
epoc["Mese"] = rtc.getMonth() + 1;
epoc["Anno"] = rtc.getYear();
epoc["Ore"] = rtc.getTime();  // solo se rtc.getTime() restituisce già una stringa


JsonObject environment_sensor = doc.createNestedObject("environment_sensor");
environment_sensor["temperature"] = String(bme.readTemperature(), 1);  // tipo float
environment_sensor["humidity"] = String(bme.readHumidity());

environment_sensor["wind_direction"] = windvane.getDirection();
environment_sensor["wind_raw_angle"] = String(windvane.getWindAngle());

JsonObject power_managment = doc.createNestedObject("power_managment");
power_managment["SOC"] = ina.vbToSoc(ina.getBusVoltage(1) * 1000.0F);

JsonObject current = power_managment.createNestedObject("Current");
current["Pannel"] = String(ina.getCurrentAmps(0) * 1000.0F, 2);
current["Battery"] = String(ina.getCurrentAmps(1) * 1000.0F, 2);
current["Load"] = String(ina.getCurrentAmps(2) * 1000.0F);


JsonObject voltage = power_managment.createNestedObject("voltage");
voltage["Pannel"] = String(ina.getBusVoltage(0) * 1000.0F, 0);
voltage["Battery"] = String(ina.getBusVoltage(1) * 1000.0F, 0);
voltage["Load"] = String(ina.getBusVoltage(2) * 1000.0F, 0);

JsonObject power = power_managment.createNestedObject("Power");
power["Pannel"] = String(ina.getPower(0), 2);
power["Battery"] = String(ina.getPower(1), 2);
power["Load"] =  String(ina.getPower(2), 2);


doc.shrinkToFit();  // optional

size_t len = serializeJson(doc, json, MAX_JSON_SIZE);
log_d("Lunghezza JSON: %d bytes\n", len);

if (len > MQTT_MAX_PACKET_SIZE) {
    log_e("[ERRORE] Il payload JSON (%d byte) supera il limite massimo MQTT (%d byte).", len, MQTT_MAX_PACKET_SIZE);
}



// serializeJson(doc, Serial);


}
