// #define ENABLE_ESP32_GITHUB_OTA_UPDATE_DEBUG // Uncomment to enable logs.
#include <Arduino.h>

#include "documentation.h"
#include "secret.h"
#include "configuration.h"


#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Time.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>


#include <Adafruit_BME280.h>
#include <Adafruit_INA3221.h>
#include "MICS6814.h"
#include "SparkFunCCS811.h"
#include "Adafruit_PM25AQI.h"


// My library 
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
 * Completamento: **80%**  
 * `[======== ]`  
 *     - 1: direzione vento                                 `[==========]`
 *     - 2: intensita vento                                 `[=>        ]`
 *     - 3: precipitazioni                                  `[==========]`
 *     - 4: temperatura e umidita                           `[==========]`
 *     - 5: umidita terreno                                 `[=>        ]`
 *     - 6: PM10                                            `[==========]`
 *     - 6: CO/NH3/NO2                                      `[==========]`
 *     - 6: CO2/tVOC                                        `[==========]`
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


CCS811 co2Sensor(CCS811_ADDRESS);


Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
PM25_AQI_Data pmSensor;

AIRQuality airQuality;



Raingauge raingauge;

// Definizione dei TASK
TaskHandle_t task1Handle = NULL;


void setup() {

  // pin configuration
  pinMode(PIN_ANEMOMETER, INPUT); // pinMode(PIN_ANEMOMETER, INPUT_PULLDOWN);

  pinMode(PIN_RAINGAUGE, INPUT);

  pinMode(PIN_5V, OUTPUT);
  pinMode(PIN_3V, OUTPUT);
  pinMode(PIN_WAK, OUTPUT);

  digitalWrite(PIN_WAK, LOW);  // turn the WAK

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

      if (task1Handle != NULL) {
        log_d(" Deleting task1Handle");
        vTaskDelete(task1Handle);
        task1Handle = NULL; 
      }

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

  // setup time
  setup_rtc_time(&rtc);
  debugCount = 0; // resetto il contatore dei debug

  String dataTime = rtc.getTime("%A, %B %d %Y %H:%M:%S");
  log_i("%s", dataTime.c_str());


  sendDeviceStatus(); // invio lo stato del dispositivo

}

void WakeUp_Interrupt(void){
  log_d("[WakeUp_Interrupt()]: Wake Up caused by interrupt");
  raingauge.countTray() ; // incremento il contatore di risvegli

  log_d("[WakeUp_Interrupt()]: Set led to RED");
  led.red();
}

void WakeUp_Timer(void){
  log_d("Wake Up caused by timer");
  setupWiFi(); 
  mqtt_init();
  otaUpdate.checkOTAOnce();

  log_d(" turn on 5V and 3V");
  digitalWrite(PIN_5V, HIGH);  
  digitalWrite(PIN_3V, HIGH);
  digitalWrite(PIN_WAK, LOW);  // turn the WAK


  // xTaskCreate(CCS811_task, "CCS811 task", 4096, NULL, 1, &task2Handle);   // in questo punto devo andarmi a definire tutti i task
  xTaskCreate(sensor_task, "Sensor task", 4096, NULL, 1, &task1Handle);   // in questo punto devo andarmi a definire tutti i task

  needsToStayActive = 1;

}

void sensor_task(void* pvParameters) {  
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_FAST);
  log_d("TASK_FAST: %d, period ticks: %d", TASK_FAST, period);

  int count_iter = 0;
  while (true) {
    // Esegui il codice del task 1
    mqtt_client.loop();

    count_iter ++;
    log_i("count_iter: %d", count_iter);

    switch (count_iter) {
      case 1: {
        log_d("Setup all sensors");
        // --------- SETUP INA3221 ---------- //
        ina.begin(INA_ADDRESS);  
        // --------- SETUP BME280 ---------- //
        bme.begin(BME280_ADDRESS); // Inizializza il sensore BME280
        // --------- SETUP mics6814 ---------- //
        gasSensor.begin(false);
        // --------- SETUP CCS811 ---------- //
        co2Sensor.begin();

        // --------- SETUP PM25AQI ---------- //
        Serial1.begin(PM25_UART_BAUDRATE, SERIAL_8N1, PM25_RX, PM25_TX);
        aqi.begin_UART(&Serial1);

        break;
      }
      case 2: { // primo giro accendo i pin 5V e 3V
        log_d("update clock");
        if (rqtReset == true) {
          setup_rtc_time(&rtc);
        }
        break;
      }
      case 3: {
        log_d("Update is avaiable");
        sendDeviceStatus();
        if (avblUpdate == UPDATE_AVAILABLE) {
          log_d("Update is available,");       
        }
        break;
      }
      case 11: {
        log_d("Read all sensors");
        // --------- READ INA3221 ---------- //
        ina.read();
        inaData = ina.getData(); // recupera la struttura con i dati
        // --------- READ BME280 ---------- //
        bme.read();
        bmeData = bme.getData(); // recupera la struttura con i dati
        // --------- READ Raingauge ---------- //
        raingauge.update();
        // --------- READ windvane ---------- //
        windvane.getDirection();
        windvaneData = windvane.getData(); // recupera la struttura con i dati
        // --------- READ CCS811 ---------- //
        unsigned long startTime = micros();
        if (!co2Sensor.read(bmeData.humidity, bmeData.temperature)) {
          log_e("Non ho ottenuto i dati dal sensore CCS811.");
        } 
        unsigned long endTime = micros();
        log_d("CCS811 read time: %lu us", endTime - startTime);
        // --------- READ MICS6814 ---------- //
        startTime = micros();
        // gasSensor.read(); #TODO: questa funzione ci impiega troppo tempo ad eseguire
        micsData = gasSensor.getData(); // recupera la struttura con i dati
        endTime = micros();
        log_d("MICS6814 read time: %lu us", endTime - startTime);
        // --------- READ PM25AQI ---------- //
        startTime = micros();
        aqi.read(&pmSensor);
        endTime = micros();
        log_d("PM25AQI read time: %lu us", endTime - startTime);
        break;
      }

      case 12: {
        log_d("SPENGO I PIN 5V");
        digitalWrite(PIN_5V, LOW);
        break;
      }


      case 18: {
        // --------- READ CCS811 ---------- //
        if (!co2Sensor.read(bmeData.humidity, bmeData.temperature)) {
          log_e("Non ho ottenuto i dati dal sensore CCS811.");
        } 
        to_serial(); // Stampa i dati su seriale per debug
        break;
      }
      case 19: {
        // --------- Power Management ---------- //
        log_d("invio i dati power management");
        sendPowerManagementData();
        break;
      }

      case 20: {
        // --------- Environment sensor ---------- //
        log_d("invio i dati environment sensor");
        sendEnvironmentData();        
        break;
      }      

      case 21: {
        // --------- Smog sensor ---------- //
        log_d("invio i dati smog sensor");
        sendAirQualityData();
        break;
      }

      case 22: {
        digitalWrite(PIN_3V, LOW);
        digitalWrite(PIN_WAK, HIGH);  // turn the WAK
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
  log_i("raingauge intensity: %s", raingauge.getRainIntensity());
  log_i("-------------------------------");
  log_i("INA3221 Battery Voltage: %s [V]", String(inaData.voltage[BATTERY_IN]));
  log_i("INA3221 Battery Current: %S [mA]", String(inaData.current[BATTERY_IN]* 1000.0F)); // Converti da A a mA
  log_i("INA3221 Battery Power: %S [W]", String(inaData.power[BATTERY_IN]));

  log_i("INA3221 Battery SOC: %S [%]", String(inaData.soc));

  log_i("INA3221 PANNEL Voltage: %s [V]", String(inaData.voltage[PANNEL_IN]));
  log_i("INA3221 PANNEL Current: %S [mA]", String(inaData.current[PANNEL_IN]* 1000.0F)); // Converti da A a mA
  log_i("INA3221 PANNEL Power: %S [W]", String(inaData.power[PANNEL_IN]));

  log_i("INA3221 LOAD Voltage: %s [V]", String(inaData.voltage[LOAD_IN]));
  log_i("INA3221 LOAD Current: %S [mA]", String(inaData.current[LOAD_IN]* 1000.0F)); // Converti da A a mA
  log_i("INA3221 LOAD Power: %S [W]", String(inaData.power[LOAD_IN]));

  log_i("INA3221 Irradiance: %s [W/m^2]", String(inaData.irradiance));
  log_i("INA3221 Cloud Cover: %S []", String(inaData.cloud_cover)); 

  log_i("-------------------------------");
  log_i("MICS6814 NH3 Value: %s [ppm]", String(micsData.NH3Value));
  log_i("MICS6814 CO Value: %s [ppm]", String(micsData.COValue));
  log_i("MICS6814 NO2 Value: %s [ppm]", String(micsData.NO2Value, 4));
  log_i("-------------------------------");
  log_i("Carbon Dioxide = %s [ppm]", String(co2Sensor.getCO2()));
  log_i("Volatile Organic Compounds = %s [ppb]", String(co2Sensor.getTVOC()));
  log_i("-------------------------------");
  log_i("Air Quality Index = %s", String(airQuality.indiceQualita(pmSensor.pm100_standard, pmSensor.pm25_standard, co2Sensor.getCO2(), micsData.NO2Value, micsData.NH3Value, micsData.COValue), 1));
  log_i("Air Quality Value = %s", airQuality.getAirQualityString());
  log_i("-------------------------------");
  log_i("Concentration Units (standard)");
  log_i("PM 1.0: %s\t\t PM 2.5: %s\t\t PM 10: %s", String(pmSensor.pm10_standard), String(pmSensor.pm25_standard), String(pmSensor.pm100_standard));
  log_i("Concentration Units (environmental)");
  log_i("PM 1.0: %s\t\t PM 2.5: %s\t\t PM 10: %s", String(pmSensor.pm10_env), String(pmSensor.pm25_env), String(pmSensor.pm100_env));
  log_i("---------------------------------------");
  log_i("Particles > 0.3um / 0.1L air: %s", String(pmSensor.particles_03um));
  log_i("Particles > 0.5um / 0.1L air: %s", String(pmSensor.particles_05um));
  log_i("Particles > 1.0um / 0.1L air: %s", String(pmSensor.particles_10um));
  log_i("Particles > 2.5um / 0.1L air: %s", String(pmSensor.particles_25um));
  log_i("Particles > 5.0um / 0.1L air: %s", String(pmSensor.particles_50um));
  log_i("Particles > 10 um / 0.1L air: %s", String(pmSensor.particles_100um));
  log_i("---------------------------------------");
  // Aggiungi qui altre letture dei sensori se necessario
}

void sendDeviceEpoc() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);

  JsonObject epoc = doc.createNestedObject("epoc");
  epoc["Giorno"] = rtc.getDay();
  epoc["Mese"] = rtc.getMonth() + 1;
  epoc["Anno"] = rtc.getYear();
  epoc["Ore"] = rtc.getTime();  // solo se rtc.getTime() restituisce già una stringa


  publishJsonMessage("device_epoc", doc);
}

void sendDeviceStatus() {
  int rssi = WiFi.RSSI();  // Riceve il valore RSSI in dBm
  String wifiPower;
  if (rssi > -30){
    log_i("Segnale WiFi eccellente");
    wifiPower = "Eccellente";
  } else if (rssi > -50) {
    log_i("Segnale WiFi molto buono");
    wifiPower = "Molto buono";
  } else if (rssi > -60) {
    log_i("Segnale WiFi buono");
    wifiPower = "Buono";
  } else if (rssi > -70) {
    log_i("Segnale WiFi accettabile");
    wifiPower = "Accettabile";
  } else if (rssi > -80) {
    log_i("Segnale WiFi scarso");
    wifiPower = "Scarso";
  } else {
    log_i("Segnale WiFi quasi nullo");
    wifiPower = "Quasi nullo";
  }


  DynamicJsonDocument doc(MAX_JSON_SIZE);
  doc["IterationCount"] = String(debugCount);  // se debugCount è int
  doc["WIFISignal"] = String(rssi);
  doc["WIFIQuality"] = wifiPower;
  doc["UpdateAvailable"] = avblUpdate;         // se è bool o int
  doc["SWVersion"] = String(otaUpdate.getCurrentFirmwareVersion());

  JsonObject epoc = doc.createNestedObject("epoc");
  epoc["Giorno"] = rtc.getDay();
  epoc["Mese"] = rtc.getMonth() + 1;
  epoc["Anno"] = rtc.getYear();
  epoc["Ore"] = rtc.getTime();  // solo se rtc.getTime() restituisce già una stringa

  publishJsonMessage("device_status", doc);
}

void sendEnvironmentData() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);

  JsonObject epoc = doc.createNestedObject("epoc");
  epoc["Giorno"] = rtc.getDay();
  epoc["Mese"] = rtc.getMonth() + 1;
  epoc["Anno"] = rtc.getYear();
  epoc["Ore"] = rtc.getTime();  // solo se rtc.getTime() restituisce già una stringa

  JsonObject environment_sensor = doc.createNestedObject("environment_sensor");
  environment_sensor["temperature"] = String(bmeData.temperature, 1);  // tipo float
  environment_sensor["humidity"] = String(bmeData.humidity);
  environment_sensor["pressure"] = String(bmeData.pressure, 2);
  environment_sensor["altitude"] = String(bmeData.altitude, 2);

  JsonObject rain_sensor = doc.createNestedObject("rain_sensor");
  rain_sensor["rain_intensity"] = String(raingauge.getLevel());
  rain_sensor["rain_intensity"] = raingauge.getRainIntensity();

  JsonObject wind_sensor = doc.createNestedObject("wind_sensor");
  wind_sensor["wind_direction"] = windvaneData.direction;
  wind_sensor["wind_angle"] = String(windvaneData.angle);

  JsonObject sun_sensor = doc.createNestedObject("sun_sensor");
  sun_sensor["sunlight"] = String(inaData.irradiance);
  sun_sensor["cloud_cover"] = String(inaData.cloud_cover);



  publishJsonMessage("environment_data", doc);
}

void sendPowerManagementData() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);

  JsonObject epoc = doc.createNestedObject("epoc");
  epoc["Giorno"] = rtc.getDay();
  epoc["Mese"] = rtc.getMonth() + 1;
  epoc["Anno"] = rtc.getYear();
  epoc["Ore"] = rtc.getTime();  // solo se rtc.getTime() restituisce già una stringa

  JsonObject power_management = doc.createNestedObject("power_management");
  power_management["SOC"] = inaData.soc; // Stato di carica stimato in percentuale

  JsonObject current = power_management.createNestedObject("Current");
  current["Pannel"] = String(inaData.current[PANNEL_IN] * 1000.0F, 2);
  current["Battery"] = String(inaData.current[BATTERY_IN] * 1000.0F, 2);
  current["Load"] = String(inaData.current[LOAD_IN] * 1000.0F, 2);

  JsonObject voltage = power_management.createNestedObject("voltage");
  voltage["Pannel"] = String(inaData.voltage[PANNEL_IN] * 1000.0F, 0);
  voltage["Battery"] = String(inaData.voltage[BATTERY_IN] * 1000.0F, 0);
  voltage["Load"] = String(inaData.voltage[LOAD_IN] * 1000.0F, 0);

  JsonObject power = power_management.createNestedObject("Power");
  power["Pannel"] = String(inaData.power[PANNEL_IN], 2);
  power["Battery"] = String(inaData.power[BATTERY_IN], 2);
  power["Load"] =  String(inaData.power[LOAD_IN], 2);

    publishJsonMessage("power_management", doc);
}

void sendAirQualityData() {
  DynamicJsonDocument doc(MAX_JSON_SIZE);

  JsonObject epoc = doc.createNestedObject("epoc");
  epoc["Giorno"] = rtc.getDay();
  epoc["Mese"] = rtc.getMonth() + 1;
  epoc["Anno"] = rtc.getYear();
  epoc["Ore"] = rtc.getTime();  // solo se rtc.getTime() restituisce già una stringa

  JsonObject air_sensor = doc.createNestedObject("air_sensor");
  air_sensor["PM1"] = String(pmSensor.pm10_standard);
  air_sensor["PM2.5"] = String(pmSensor.pm25_standard);
  air_sensor["PM10"] = String(pmSensor.pm100_standard);

  JsonObject gas_sensor = doc.createNestedObject("gas_sensor");
  gas_sensor["NH3"] = String(micsData.NH3Value);
  gas_sensor["CO"] = String(micsData.COValue);
  gas_sensor["NO2"] = String(micsData.NO2Value, 4);
  gas_sensor["CO2"] = String(co2Sensor.getCO2());
  gas_sensor["tVOC"] = String(co2Sensor.getTVOC());
  gas_sensor["baseline"] = String(co2Sensor.getBaseline());

  JsonObject air_quality = doc.createNestedObject("air_quality");
  air_quality["Index"] = String(airQuality.indiceQualita(pmSensor.pm100_standard, pmSensor.pm25_standard, co2Sensor.getCO2(), micsData.NO2Value, micsData.NH3Value, micsData.COValue), 1);
  air_quality["Quality"] = airQuality.getAirQualityString();


  publishJsonMessage("air_quality", doc);
}







