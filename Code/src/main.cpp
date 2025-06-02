#define ENABLE_ESP32_GITHUB_OTA_UPDATE_DEBUG // Uncomment to enable logs.

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Time.h>

// My library 
#include "documentation.h"
#include "configuration.h"
#include "secret.h"
#include "UtilitiesFcn.h"



#include "HttpReleaseUpdate.h"
#include "ESP32GithubOtaUpdate.h"
#include "ESP32Mqtt.h"

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

bool needToStayAlive = 0;



ESP32GithubOtaUpdate otaUpdate;
WiFiClientSecure espClient;
PubSubClient mqtt_client(espClient);
ESP32Time rtc(0);  // Dichiarazione di rtc

// SENSORI
INA3211 ina;


// Definizione dei TASK
TaskHandle_t task1Handle = NULL;



void setup() {

  // pin configuration
  pinMode(PIN_ANEMOMETER, INPUT); // pinMode(PIN_ANEMOMETER, INPUT_PULLDOWN);
  pinMode(PIN_RAINGAUGE, INPUT);

  pinMode(PIN_5V, OUTPUT);
  pinMode(PIN_3V, OUTPUT);



  int count = 0;
  Serial.begin(115200);
  delay(100);


  log_i("[setup()]: WelcomeToNewBuild");
  log_d("[setup()]: Wakeup Count: ", String(wakeUpCount));  


  // scopro il motivo del risveglio
  esp_sleep_wakeup_cause_t wakeUpRsn = esp_sleep_get_wakeup_cause();  

  log_d("[setup()]: wakeup reason num: ", String(wakeUpRsn));

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

    log_d("[loop()]: Entrando in modalità sleep...");
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

  String dataTime = rtc.getTime("%A, %B %d %Y %H:%M:%S");
  log_i("%s", dataTime.c_str());

}

void WakeUp_Interrupt(void){
  log_d("[WakeUp_Interrupt()]: Wake Up caused by interrupt");
  led.init();
  wakeUpCount ++;
  log_d("[WakeUp_Interrupt()]: Set led to RED");
  led.red();
}

void WakeUp_Timer(void){
  log_d("[WakeUp_Timer()]: Wake Up caused by timer");
  led.init();
  
  setupWiFi(); 
  mqtt_init();

  xTaskCreate(led_blink_task, "LED blink task", 2048, NULL, 1, &task1Handle);   // in questo punto devo andarmi a definire tutti i task
  needsToStayActive = 1;

  otaUpdate.checkOTAOnce();

}

void led_blink_task(void* pvParameters) {  
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_FAST);
  int count_iter = 0;
  while (true) {
    // Esegui il codice del task 1
    mqtt_client.loop();

    led.toggle();
    // led.blue();
    count_iter ++;

    switch (count_iter) {
      case 1: { // primo giro accendo i pin 5V e 3V
        log_d("[led_blink_task()]: turn on 5V and 3V");
        digitalWrite(PIN_5V, HIGH);  
        digitalWrite(PIN_3V, HIGH);
        break;
      }
      case 2: {
        log_d("[led_blink_task()]: Setup all sensors");
        // --------- SETUP INA3221 ---------- //
        ina.begin(INA_ADDRESS);
        break;
      }
      case 3: {
        log_d("[led_blink_task()]: Print Sensor");
        printSensor();
        break;
      }
      case 10: {
        log_d("[led_blink_task()]: senn decide to publish the value of avblUpdate");
        String payload = "{\"msg\": \"pippo\", \"Value\": \"" + String(avblUpdate) + "\"}";
        mqtt_client.publish("upd_avbl", payload.c_str(), 1);  
        log_d("msg: %d", avblUpdate);
        break;
      }
      case 20: {
        log_d("[led_blink_task()]: we can turn off 5V and 3V");
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



void printSensor(void){

  for (uint8_t i = 0; i < 3; i++) {
    float voltage = ina.getBusVoltage(i);
    float current = ina.getCurrentAmps(i) * 1000; // Convert to mA
    Serial.print("Channel " + String(i) + ": ");
    Serial.print("Voltage = " + String(voltage) + "[V] ");
    Serial.println("Current = " + String(current) + " [mA]");
  }
  Serial.println("getPower: " + String(ina.getPower(2)));
  Serial.println("SOC = " + String(ina.vbToSoc(ina.getBusVoltage(1)*1000)) + " [ %]");

}
