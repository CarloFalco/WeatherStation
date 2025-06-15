#ifndef DOCUMENT_H
#define DOCUMENT_H


/**
 * @brief Gestisce la procedura di riattivazione del dispositivo dopo un'interruzione di corrente.
 * 
 * Questa funzione viene chiamata quando il dispositivo si risveglia a seguito di una perdita di alimentazione.
 * Esegue le seguenti operazioni:
 * - Stampa un messaggio di debug per indicare la causa del risveglio.
 * - Stabilisce la connessione WiFi tramite `setupWiFi()`.
 * - Inizializza l'orologio in tempo reale (RTC) con `setup_rtc_time(&rtc)`.
 * - Recupera e stampa la data/ora attuale formattata in log.
 * 
 * @note Assicura che la variabile globale `rtc` sia inizializzata correttamente prima della chiamata.
 */
void WakeUp_PowerLoss(void);
 

 /**
 * @brief Gestisce il risveglio del dispositivo causato da un'interruzione esterna.
 * 
 * Questa funzione viene chiamata quando il dispositivo si risveglia a seguito di un'interruzione esterna
 * (ad esempio, un segnale su uno dei pin di wakeup). Esegue le seguenti operazioni:
 * - Inizializza il LED.
 * - Incrementa il contatore di risvegli (`wakeUpCount`).
 * - Imposta il LED su rosso per indicare lo stato di risveglio.
 */
void WakeUp_Interrupt(void);

/**
 * @brief Gestisce il risveglio del dispositivo causato da un timer.
 * g
 * Questa funzione viene chiamata quando il dispositivo si risveglia a seguito di un timer.
 * Esegue le seguenti operazioni:
 * - Inizializza il LED.
 * - Stabilisce la connessione WiFi tramite `setupWiFi()`.
 * - Inizializza l'aggiornamento OTA con `setupOtaUpdate()`.
 * - Inizializza il client MQTT con `mqtt_init()`.
 * - Crea un task per la gestione dei sensori (`sensor_task`).
 * - Imposta la variabile `needsToStayActive` a 1 per indicare che il dispositivo deve rimanere attivo.
 */
 
void WakeUp_Timer(void);

/** * @brief Task per la gestione dei sensori.
 *
 * Questo task esegue il loop del client MQTT e gestisce i dati dei sensori.
 * I dati vengono letti dai sensori e inviati tramite MQTT.

 * 
 * @param pvParameters Parametri del task (non utilizzati in questo caso).
*/

void sensor_task(void* pvParameters);


void to_serial(void);

void sendDeviceEpoc();

void sendDeviceStatus();

void sendEnvironmentData(); 

void sendPowerManagementData();

void sendAirQualityData();


 #endif