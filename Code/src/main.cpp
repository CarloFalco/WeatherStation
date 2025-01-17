#define ENABLE_ESP32_GITHUB_OTA_UPDATE_DEBUG Uncomment to enable logs.

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Time.h>


// My library 
#include "HttpReleaseUpdate.h"
#include "ESP32GithubOtaUpdate.h"
#include "secret.h"
#include "UtilitiesFcn.h"

/* TODO: requirements

-> impostazione tempo e fuso orario                                   # [=========>]
-> Creaziione delle funzioni legate al risveglio della centralina     # [=========>]
-> creazione dei task delle                                           # [==========]
-> salvataggio informazioni su eeprom                                 # [=>        ]

-> Sensori stazione meteo:
    - 1: direzione vento                                # [=>        ]
    - 2: intensita vento                                # [=>        ]
    - 3: precipitazioni                                 # [=>        ]
    - 4: temperatura e umidita                          # [=>        ]
    - 5: umidita terreno                                # [=>        ]
    

*/




#define S_TO_uS_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define S_TO_mS_FACTOR 1000ULL  /* Conversion factor for milli seconds to seconds */
#define mS_TO_uS_FACTOR 1000ULL  /* Conversion factor for milli seconds to micro seconds */

#define TIME_TO_SLEEP  20  // s
#define TASK_FAST 500  // Periodo del task in millisecondi


#define WAKEUP_PIN_1 GPIO_NUM_10  // Pin RTC 33
#define WAKEUP_PIN_2 GPIO_NUM_11  // Pin RTC 34



Led led(97); // #define LED_BUILTIN 97


RTC_DATA_ATTR int wakeUpCount = 0;
RTC_DATA_ATTR uint8_t needsToStayActive = 0; // probabilmente duplicato // TODO: verificare
RTC_DATA_ATTR unsigned long wakeUpPreviousTime = 0;

ESP32Time rtc(0);  // Dichiarazione di rtc



const char* OTA_FILE_LOCATION = "https://raw.githubusercontent.com/CarloFalco/WeatherStation/refs/heads/main/Code/firmware.bin";
const char* VERSION_URL = "https://raw.githubusercontent.com/CarloFalco/WeatherStation/refs/heads/main/Code/version.txt";

const int current_fw_version = 2025010100;  // YYYYMMDDRR where R = release of the day



ESP32GithubOtaUpdate otaUpdate;
bool needToStayAlive = 0;
bool rqtUpdate = false;

update_status_t avblUpdate = NO_UPDATE; // -> questa variabile dovrebbe essere mantenuta tra un accensione e l'altra

WiFiClientSecure espClient;
PubSubClient mqtt_client(espClient);

#include "ESP32Mqtt.h"


TaskHandle_t task1Handle;

void led_blink_task(void* pvParameters);
void WakeUp_PowerLoss(void);
void WakeUp_Timer(void);
void WakeUp_Interrupt(void);




void setupOtaUpdate() {
  otaUpdate.setOTADownloadUrl(OTA_FILE_LOCATION);
  otaUpdate.setVersionCheckUrl(VERSION_URL);
  otaUpdate.setCurrentFirmwareVersion(current_fw_version);
  otaUpdate.setUpdateCheckInterval(60); // Check every 60 seconds.
  otaUpdate.checkOTAOnce();
  //otaUpdate.begin();
}

void setup() {

  // pin configuration
  pinMode(WAKEUP_PIN_1, INPUT_PULLDOWN);
  pinMode(WAKEUP_PIN_2, INPUT_PULLDOWN);
  int count = 0;
  Serial.begin(115200);
  delay(100);
  if (!Serial) {led.init();}
  while (!Serial) { // wait for serial port to connect. Needed for native USB
    count++; 
    delay(100);
    led.toggle();   
    if (count>=10) {
      led.off();
      break;
    }
  }

  print_i("[setup()]: WelcomeToNewBuild");
  print_d("[setup()]: Wakeup Count: " + String(wakeUpCount));  


  esp_sleep_wakeup_cause_t wakeUpRsn = esp_sleep_get_wakeup_cause();  

  print_d("[setup()]: wakeup reason num: " + String(wakeUpRsn));

  // se mi sono svegliato per un interrupt di un pin (vale sia il primo metodo che il secondo)
  if (wakeUpRsn == ESP_SLEEP_WAKEUP_EXT0 || wakeUpRsn == ESP_SLEEP_WAKEUP_EXT1) {WakeUp_Interrupt();}

  // se mi sono svegliato causa mancanza di batteria riinizializzo l'rtc
  if (wakeUpRsn == ESP_SLEEP_WAKEUP_TIMER) {WakeUp_Timer();}

  // se mi sono svegliato causa mancanza di batteria riinizializzo l'rtc
  if (wakeUpRsn == ESP_SLEEP_WAKEUP_UNDEFINED) {WakeUp_PowerLoss();}

}


void loop(){

    if (needsToStayActive == 0){
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
      wakeUpPreviousTime = rtc.getEpoch();
    }

    esp_sleep_enable_ext1_wakeup((1ULL << WAKEUP_PIN_1) | (1ULL << WAKEUP_PIN_2), ESP_EXT1_WAKEUP_ANY_HIGH);

    // Configuriamo il wakeup tramite timer
    esp_sleep_enable_timer_wakeup(timeToNextWakeUp);
    print_d("[loop()]: Entrando in modalità sleep...");
    if(Serial) {
      Serial.flush();
      }
    // Entriamo in modalità deep sleep
    esp_deep_sleep_start();
  }


}


void WakeUp_PowerLoss(void){
  print_d("[WakeUp_PowerLoss()]: Wake Up caused by powerloss");
  setupWiFi(); 
  
  // setup time
  setup_rtc_time(&rtc);

  String dataTime = rtc.getTime("%A, %B %d %Y %H:%M:%S");
  log_i("%s", dataTime.c_str());

}

void WakeUp_Interrupt(void){
  print_d("[WakeUp_Interrupt()]: Wake Up caused by interrupt");
  led.init();
  wakeUpCount ++;
  print_d("[WakeUp_Interrupt()]: Set led to RED");
  led.red();
}

void WakeUp_Timer(void){
  log_d("[WakeUp_Timer()]: Wake Up caused by timer");
  led.init();
  
  setupWiFi(); 
  setupOtaUpdate();
  mqtt_init();

  xTaskCreate(led_blink_task, "LED blink task", 2048, NULL, 1, &task1Handle);   // in questo punto devo andarmi a definire tutti i task
  needsToStayActive = 1;


}

void led_blink_task(void* pvParameters) {  
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_FAST);
  int count_iter = 0;
  while (true) {
    // Esegui il codice del task 1
    mqtt_client.loop();

    if (count_iter == 10) {
      String payload = "{\"msg\": \"pippo\", \"Value\": \"" + String(avblUpdate) + "\"}";
      mqtt_client.publish("upd_avbl", payload.c_str(), 1);  
      log_d("msg: %d", avblUpdate);
    }


    led.toggle();
    // led.blue();
    count_iter ++;
    if (count_iter > 20){      
      needsToStayActive = 0;
    }

    vTaskDelayUntil(&lastWakeTime, period);
  }
}