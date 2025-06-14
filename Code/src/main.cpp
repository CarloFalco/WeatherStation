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
#include "configuration.h"

#include "UtilitiesFcn.h"

#include "HttpReleaseUpdate.h"
#include "ESP32GithubOtaUpdate.h"
#include "ESP32Mqtt.h"

#include "MICS6814.h"
#include "SparkFunCCS811.h"
#include "Adafruit_PM25AQI.h"

#include "AllSensor.h"


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
 * `[===>      ]`
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
 * `[===>      ]`                         
 *  - 1: credenziali wif
 * 
 * 
 * 
 * @subsection Sensori stazione meteo:
* Stato: **In sviluppo**  
 * Completamento: **50%**  
 * `[====>     ]`  
 *     - 1: direzione vento                                 `[==========]`
 *     - 2: intensita vento                                 `[=>        ]`
 *     - 3: precipitazioni                                  `[=>        ]`
 *     - 4: temperatura e umidita                           `[==========]`
 *     - 5: umidita terreno                                 `[=>        ]`
 *     - 6: PM10                                            `[=>        ]`
 *     - 6: CO/NH3/NO2                                      `[==========]`
 *     - 6: CO2/tVOC                                        `[=>        ]`
 * 
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
RTC_DATA_ATTR bool rqtReset = false;
RTC_DATA_ATTR int debugCount = 0;

bool needToStayAlive = 0;



ESP32GithubOtaUpdate otaUpdate;
WiFiClientSecure espClient;
PubSubClient mqtt_client(espClient);
ESP32Time rtc(0);  // Dichiarazione di rtc

// SENSORI
INA3211 ina;
INA3211::INA inaData;            // struttura globale per accedere ai dati

BME280 bme; 
BME280::BME bmeData; // struttura globale per accedere ai dati

Windvane windvane(AS5600_ADDRESS);
Windvane::WIN windvaneData; // struttura globale per accedere ai dati

MICS6814 gasSensor(PIN_CO, PIN_NO2, PIN_NH3);
MICS6814::MICS micsData; // struttura globale per accedere ai dati


// CCS811 co2Sensor(CCS811_ADDRESS);


// Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
// PM25_AQI_Data pmSensor;




Raingauge raingauge;

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
    // esp_sleep_enable_ext1_wakeup((1ULL << WAKEUP_PIN_1) | (1ULL << WAKEUP_PIN_2), ESP_EXT1_WAKEUP_ANY_LOW);
    
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN_1, 0);
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
  mqtt_init();

  sendDeviceStatus(); // invio lo stato del dispositivo

  // setup time
  setup_rtc_time(&rtc);
  debugCount = 0; // resetto il contatore dei debug

  String dataTime = rtc.getTime("%A, %B %d %Y %H:%M:%S");
  log_i("%s", dataTime.c_str());


}

void WakeUp_Interrupt(void){
  log_d("[WakeUp_Interrupt()]: Wake Up caused by interrupt");
  raingauge.countTray() ; // incremento il contatore di risvegli

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

      case 2: { // primo giro accendo i pin 5V e 3V
        log_d("update clock");
        if (rqtReset = true) {
          setup_rtc_time(&rtc);
        }

        break;
      }

      case 3: {
        log_d("Setup all sensors");
        // --------- SETUP INA3221 ---------- //
        ina.begin(INA_ADDRESS);  
        // --------- SETUP BME280 ---------- //
        bme.begin(BME280_ADDRESS); // Inizializza il sensore BME280
        // --------- SETUP mics6814 ---------- //
        gasSensor.begin(false);
        // --------- SETUP CCS811 ---------- //
        // co2Sensor.begin();
        // --------- SETUP PM25AQI ---------- //
        // Serial1.begin(9600, SERIAL_8N1, 17, 18);
        // aqi.begin_UART(&Serial1);

        break;
      }

      case 4: {
        log_d("Update is avaiable");
        sendDeviceStatus();
        if (avblUpdate == UPDATE_AVAILABLE) {
          log_d("Update is available,");       
        }
        break;
      }

      case 5: {
        log_d("Read all sensors");

        // --------- READ INA3221 ---------- //
        ina.read();
        inaData = ina.getData(); // recupera la struttura con i dati

        // --------- READ BME280 ---------- //
        bme.read();
        bmeData = bme.getData(); // recupera la struttura con i dati
        // --------- READ Windvane ---------- //

        windvane.getDirection();
        windvaneData = windvane.getData(); // recupera la struttura con i dati

        // --------- READ MICS6814 ---------- //
        gasSensor.read();
        micsData = gasSensor.getData(); // recupera la struttura con i dati

/*
        // --------- READ CCS811 ---------- //
        co2Sensor.read();
        co2SensorData = co2Sensor.getData();

        // --------- READ PM25AQI ---------- //
        aqi.read();
        pmSensor = aqi.getData();
*/
        to_serial(); 

        break;
      }

      case 6: {
        log_d("we can turn off 5V and 3V");
        digitalWrite(PIN_5V, LOW);
        digitalWrite(PIN_3V, LOW);
      }

      case 7: {
        // --------- Power Management ---------- //
        log_d("invio i dati power management");
        unsigned long t0 = micros();   // Tempo iniziale in µs
        sendPowerManagementData();
        unsigned long t1 = micros();   // Tempo finale in µs

        log_d("Tempo di esecuzione: %d µs", t1 - t0);
        break;
      }

      case 8: {
        // --------- Environment sensor ---------- //
        log_d("invio i dati environment sensor");
        sendEnvironmentData();

        break;
      }      

      case 9: {
        // --------- Smog sensor ---------- //
        log_d("invio i dati smog sensor");
        //sendSmogData();

        break;
      }


      case 10: {
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
  log_i("BME280 Temperature: %s [*C]", String(bmeData.temperature, 1));
  log_i("BME280 Humidity: %s [%%]", String(bmeData.humidity, 2));
  log_i("BME280 Pressure: %s [hPa]", String(bmeData.pressure, 2));
  log_i("BME280 Altitude: %s m",  String(bmeData.altitude)); // 1013.25 hPa is the standard sea level pressure
  log_i("-------------------------------");
  log_i("Direzione del vento: %s []", windvaneData.direction);
  log_i("Velocità del vento: %s [*]", String(windvaneData.angle));
  log_i("-------------------------------");
  log_i("Raingauge level: %s [mm]", String(raingauge.getLevel()));
  log_i("-------------------------------");
  log_i("INA3221 Battery Voltage: %s [V]", String(inaData.voltage[1]));
  log_i("INA3221 Battery Current: %S [mA]", String(inaData.current[1]* 1000.0F)); // Converti da A a mA
  log_i("INA3221 Battery Power: %S [W]", String(inaData.power[1]));

  log_i("INA3221 Battery SOC: %S [%]", String(inaData.soc));

  log_i("INA3221 PANNEL Voltage: %s [V]", String(inaData.voltage[2]));
  log_i("INA3221 PANNEL Current: %S [mA]", String(inaData.current[2]* 1000.0F)); // Converti da A a mA
  log_i("INA3221 PANNEL Power: %S [W]", String(inaData.power[2]));

  log_i("INA3221 LOAD Voltage: %s [V]", String(inaData.voltage[0]));
  log_i("INA3221 LOAD Current: %S [mA]", String(inaData.current[0]* 1000.0F)); // Converti da A a mA
  log_i("INA3221 LOAD Power: %S [W]", String(inaData.power[0]));
  log_i("-------------------------------");
  log_i("MICS6814 NH3 Value: %s [ppm]", String(micsData.NH3Value));
  log_i("MICS6814 CO Value: %s [ppm]", String(micsData.COValue));
  log_i("MICS6814 NO2 Value: %s [ppm]", String(micsData.NO2Value));
  log_i("-------------------------------");

  // Aggiungi qui altre letture dei sensori se necessario
}


void sendDeviceEpoc() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);



  publishJsonMessage("device_epoc", doc);
}

void sendDeviceStatus() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);

  doc["IterationCount"] = String(debugCount);  // se debugCount è int
  doc["UpdateAvailable"] = avblUpdate;         // se è bool o int
  doc["SWVersion"] = String(otaUpdate.getCurrentFirmwareVersion());

  publishJsonMessage("device_status", doc);
}

void sendEnvironmentData() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);

  JsonObject environment_sensor = doc.createNestedObject("environment_sensor");
  environment_sensor["temperature"] = String(bmeData.temperature, 1);  // tipo float
  environment_sensor["humidity"] = String(bmeData.humidity);

  environment_sensor["wind_direction"] = windvaneData.direction;
  environment_sensor["wind_raw_angle"] = String(windvaneData.angle);

  environment_sensor["NH3"] = String(micsData.NH3Value);
  environment_sensor["CO"] = String(micsData.COValue);
  environment_sensor["NO2"] = String(micsData.NO2Value);
  publishJsonMessage("environment_data", doc);
}

void sendPowerManagementData() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);

  JsonObject power_management = doc.createNestedObject("power_management");
  power_management["SOC"] = inaData.soc; // Stato di carica stimato in percentuale

  JsonObject current = power_management.createNestedObject("Current");
  current["Pannel"] = String(inaData.current[2] * 1000.0F, 2);
  current["Battery"] = String(inaData.current[1] * 1000.0F, 2);
  current["Load"] = String(inaData.current[0] * 1000.0F, 2);

  JsonObject voltage = power_management.createNestedObject("voltage");
  voltage["Pannel"] = String(inaData.voltage[2] * 1000.0F, 0);
  voltage["Battery"] = String(inaData.voltage[1] * 1000.0F, 0);
  voltage["Load"] = String(inaData.voltage[0] * 1000.0F, 0);

  JsonObject power = power_management.createNestedObject("Power");
  power["Pannel"] = String(inaData.power[2], 2);
  power["Battery"] = String(inaData.power[1], 2);
  power["Load"] =  String(inaData.power[0], 2);

    publishJsonMessage("power_management", doc);
}

void sendSmogData() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);
/*
  JsonObject smog = doc.createNestedObject("smog");
  smog["PM1"] = String(sensors.getPM1());
  smog["PM2.5"] = String(sensors.getPM25());
  smog["PM10"] = String(sensors.getPM10());
*/
  publishJsonMessage("smog_data", doc);
}
