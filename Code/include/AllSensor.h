

#ifndef _ALL_SENSORS_H_
#define _ALL_SENSORS_H_

#include <Arduino.h>


#include "driver/gpio.h"

#include "configuration.h"
#include "UtilitiesFcn.h"
// ANEMOMETRO


/**
 * @class Anemometer
 * @brief Classe per la gestione di un anemometro con supporto a interrupt e media mobile.
 */
class Anemometer {
  public:
    /**
    * @brief Costruttore dell'anemometro.
    * @param pin Pin digitale a cui è collegato il sensore.
    * @param diameter Diametro del rotore dell'anemometro (in metri).
    */
    Anemometer(int pin, float diameter);

    /**
    * @brief Inizializza l'anemometro, attiva l'interrupt e avvia il task FreeRTOS.
    */
    void begin();
    
    /**
     * @brief Arresta il task e disattiva l'interrupt.
     */
    void kill();

    /**
     * @brief Aggiorna la velocità del vento usando una media mobile delle rotazioni.
     */
    void update();

    /**
     * @brief Ritorna la velocità del vento in metri al secondo.
     * @return Velocità del vento [m/s].
     */
    float getWindSpeed_ms(void);

    /**
     * @brief Ritorna la velocità del vento in chilometri all'ora.
     * @return Velocità del vento [km/h].
     */
    float getWindSpeed_kph(void);

    /**
     * @brief Restituisce una descrizione testuale dell’intensità del vento secondo la scala di Beaufort.
     * @return Stringa con l’intensità del vento (es. "Brezza leggera", "Tempesta").
     */
    String getIntensityString(void);
    
  private:
    int _pin;                         /**< Pin collegato al sensore */
    float _diameter;                  /**< Diametro del rotore */
    float _circumference;            /**< Circonferenza calcolata */
    volatile int _rotationCount;     /**< Contatore di giri (incrementato via interrupt) */
    int _arrayIndex;                 /**< Indice corrente per media mobile */
    static const int _numElements = 5; /**< Numero di elementi per la media mobile */
    int _rotationCounts[_numElements]; /**< Array per i conteggi delle rotazioni */

    float _windSpeed = 0;            /**< Velocità corrente del vento in m/s */

    TaskHandle_t _taskHandle;        /**< Handle del task FreeRTOS */

    static Anemometer* instance;     /**< Puntatore statico all'istanza per gestione interrupt */

    /**
     * @brief Funzione chiamata nell'interrupt per contare le rotazioni.
     */
    static void countRevolutions();

    /**
    * @brief Task FreeRTOS per aggiornare periodicamente la velocità del vento.
    * @param pvParameters Parametri passati al task (puntatore all'istanza).
    */
    static void anemometerTask(void* pvParameters);
};

Anemometer* Anemometer::instance = nullptr; // Inizializza il puntatore statico


Anemometer::Anemometer(int pin, float diameter) 
  : _pin(pin), _diameter(diameter), _rotationCount(0), _arrayIndex(0), _circumference(M_PI*_diameter){
  for (int i = 0; i < _numElements; i++) {
    _rotationCounts[i] = 0;
  }
  ;
}

void Anemometer::begin() {
  pinMode(_pin, INPUT_PULLUP);
  instance = this; // Assegna l'istanza corrente al puntatore statico
  attachInterrupt(digitalPinToInterrupt(_pin), countRevolutions, FALLING);
  xTaskCreate(anemometerTask, "evaluation of anemometer task", 2048, this, 1, &_taskHandle);
}

void Anemometer::kill() {
  detachInterrupt(digitalPinToInterrupt(_pin)); 
  vTaskDelete(_taskHandle);
}

void Anemometer::update() {
  // Aggiorna l'array delle rotazioni
  _rotationCounts[_arrayIndex] = _rotationCount;
  _arrayIndex = (_arrayIndex + 1) % _numElements; // Incrementa l'indice e lo resetta a 0 dopo 3 cicli

  // Calcola la media mobile delle rotazioni
  float sum = 0;
  for (int i = 0; i < _numElements; i++) {
    sum += _rotationCounts[i];
  }
  float averageRotationCountSum = sum / _numElements;

  // Calcola la frequenza media di rotazione
  float frequency = averageRotationCountSum / (TASK_ANEMOMETER / 1000.0); // Rotazioni per secondo

  // Calcola la velocità del vento
  float _windSpeed = frequency * _circumference;

  //print_Ses("Velocità del vento (media mobile): " + String(_windSpeed) + " [m/s]");
  //print_Ses("Velocità del vento (media mobile): " + String(_windSpeed * 3.6) + " [km/h]");
  //print_Ses("Velocità del vento (media mobile): " + getIntensityString());

  // Resetta il contatore di rotazioni
  _rotationCount = 0;
}

float Anemometer::getWindSpeed_ms(){
  return _windSpeed;
}

float Anemometer::getWindSpeed_kph(){
  return _windSpeed * 3.6;
}

String Anemometer::getIntensityString(void) {
  // FONTE : https://it.wikipedia.org/wiki/Scala_di_Beaufort
  float windSpeed_kph = _windSpeed * 3.6;
  if (windSpeed_kph < 1) return "Calma";
  else if (windSpeed_kph >= 1 && windSpeed_kph < 6) return "Bava di vento";
  else if (windSpeed_kph >= 6 && windSpeed_kph < 11) return "Brezza leggera";
  else if (windSpeed_kph >= 11 && windSpeed_kph < 19) return "Brezza tesa";
  else if (windSpeed_kph >= 19 && windSpeed_kph < 29) return "Vento moderato";
  else if (windSpeed_kph >= 29 && windSpeed_kph < 39) return "Vento teso";
  else if (windSpeed_kph >= 39 && windSpeed_kph < 50) return "Vento fresco";
  else if (windSpeed_kph >= 50 && windSpeed_kph < 62) return "Vento forte";
  else if (windSpeed_kph >= 62 && windSpeed_kph < 75) return "Burrasca";
  else if (windSpeed_kph >= 75 && windSpeed_kph < 87) return "Burrasca forte";
  else if (windSpeed_kph >= 88 && windSpeed_kph < 102) return "Tempesta";
  else if (windSpeed_kph >= 102 && windSpeed_kph < 117) return "Tempesta violenta";
  else if (windSpeed_kph >= 117) return "Uragano";
  else return "Calma";
}

void Anemometer::countRevolutions() {
  instance->_rotationCount++;
}

void Anemometer::anemometerTask(void* pvParameters) {
  Anemometer* anemometer = static_cast<Anemometer*>(pvParameters);
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TASK_ANEMOMETER);

  while (true) {
    anemometer->update();
    vTaskDelayUntil(&lastWakeTime, period);
  }
}



/**
 * @class Windvane
 * @brief Classe per la lettura della direzione del vento da una banderuola elettronica tramite I2C.
 */
class Windvane {
  public:
    /**
     * @brief Costruttore della classe Windvane.
     * @param addr Indirizzo I2C del sensore di direzione del vento.
     */
    Windvane(uint8_t addr);
    
    typedef struct {
      float angle;        /**< Angolo del vento in gradi */
      String direction;   /**< Direzione del vento in formato testuale */
    } WIN;

    WIN winData;  /**< Dati letti dal sensore di direzione del vento */
    /**
     * @brief Restituisce la direzione del vento in formato testuale (es. "N", "NE", ...).
     * @return Stringa rappresentante la direzione cardinale.
     */
    void getDirection(void);

    /**
     * @brief Restituisce la direzione del vento in formato testuale (es. "N", "NE", ...).
     * @return Stringa rappresentante la direzione cardinale.
     */
    String getWindDirection(void);
    /**
     * @brief Restituisce l'angolo della direzione del vento in gradi (0–360).
     * @return Angolo in gradi rispetto al nord.
     */
    float getWindAngle(void);

    WIN getData(void);

  private:
    /**
     * @brief Legge l'angolo grezzo dal sensore via I2C.
     */
    void readRawAngle(void);

    /**
     * @brief Converte il valore grezzo in un angolo compreso tra 0° e 360°.
     */
    void rawToAngle(void);

    uint16_t _rawAngle;  /**< Valore grezzo dell'angolo letto dal sensore */
    float _angle;        /**< Angolo convertito in gradi */
    String _direction;        /**< Direzione del vento in formato testuale */
    uint8_t _addr;       /**< Indirizzo I2C del sensore */
};

Windvane::Windvane(uint8_t addr) 
  : _addr(addr), _rawAngle(0), _angle(0){
    // Costruttore vuoto
}

void Windvane::readRawAngle(void) {
  Wire.beginTransmission(_addr);
  Wire.write(RAW_ANGLE);
  Wire.endTransmission();
  Wire.requestFrom(_addr, (uint8_t)2);
  _rawAngle = Wire.read() << 8 | Wire.read();
  rawToAngle();

}

void Windvane::rawToAngle(void) {
  _angle = (_rawAngle * 360.0) / 4096.0; // Converti il valore grezzo in gradi
}

void Windvane::getDirection(void) {
  readRawAngle();
  float angle = _angle;
  String direction = "";
  if (angle >= 337.5 || angle < 22.5) direction = "N";
  else if (angle >= 22.5 && angle < 67.5) direction = "NE";
  else if (angle >= 67.5 && angle < 112.5) direction = "E";
  else if (angle >= 112.5 && angle < 157.5) direction = "SE";
  else if (angle >= 157.5 && angle < 202.5) direction = "S";
  else if (angle >= 202.5 && angle < 247.5) direction = "SW";
  else if (angle >= 247.5 && angle < 292.5) direction = "W";
  else direction = "NW";
  _direction = direction; // Aggiorna la direzione interna

  winData.angle = _angle;
  winData.direction = _direction;
}

String Windvane::getWindDirection(void){
  return _direction; 
}

float Windvane::getWindAngle(void){
  return _angle; 
}

Windvane::WIN Windvane::getData(void){
  return winData;
}

/**
 * @class Raingauge
 * @brief Classe per la gestione di un pluviometro basato su conteggio degli scatti della bascula.
 */
class Raingauge {
  public:
    /**
     * @brief Costruttore di default del pluviometro.
     */
    Raingauge(void);

    /**
     * @brief Incremente il conteggio delle bascule del pluviometro (tipicamente da ISR).
     */
    void countTray();

    /**
     * @brief Calcola la quantità di pioggia caduta in base al conteggio e al tempo.
     * Azzera il contatore interno dopo l'elaborazione.
     */
    void update();   

    /**
     * @brief Restituisce il livello di pioggia calcolato in millimetri.
     * @return Quantità di pioggia (approssimata all'intero) in mm.
     */
    int getLevel(); 
    /**
     * @brief Restituisce una stringa che descrive l'intensità della pioggia.
     * @return Stringa con l'intensità della pioggia (es. ).
     */
    String getRainIntensity();

  private:
    int _rainDroppCount;   /**< Conteggio degli scatti della bascula */
    float _rainDropp;      /**< Quantità di pioggia calcolata in mm */
};

Raingauge::Raingauge(void) 
  : _rainDroppCount(0), _rainDropp(0){
}

void Raingauge::update() {

  float iterationHour = _rainDroppCount *  3600 / TIME_TO_SLEEP;
  _rainDropp = iterationHour * CONVERSIONE_PLUVIOMETRO;
  // print_Ses("rainDropp: " + String(_rainDropp) + "mm");

  _rainDroppCount = 0;

}

void Raingauge::countTray() {
  _rainDroppCount++;
}

int Raingauge::getLevel(){
  return (int)_rainDropp;
}

String Raingauge::getRainIntensity(void) {
  if (_rainDropp == 0) return "nessuna pioggia";
  if (_rainDropp <= 2.5) return "pioggia leggera";
  if (_rainDropp <= 7.6) return "pioggia moderata";
  if (_rainDropp <= 50)  return "pioggia intensa";
  return "pioggia molto intensa";
}



/**
 * @class INA3211
 * @brief Estensione della libreria Adafruit_INA3221 per aggiungere funzionalità di monitoraggio della potenza e stato di carica (SoC).
 */
class INA3211 : public Adafruit_INA3221 {
  public:
    /**
    * @struct INA
    * @brief Struttura dati per contenere informazioni elettriche su 3 canali.
    * - Channel 0: Load (carico).
    * - Channel 1: Batteria.
    * - Channel 2: Pannello solare.
    * 
    */
    typedef struct {
      float current[3];  /**< Corrente per ogni canale (A) */
      float voltage[3];  /**< Tensione per ogni canale (V) */
      float power[3];    /**< Potenza per ogni canale (W) */
      int soc;           /**< Stato di carica stimato (%) */
      int irradiance;    /**< Irraggiamento solare (W/m²) */
      String cloud_cover; /**< Copertura nuvolosa */
    } INA;

    INA inaData;  /**< Dati letti dai 3 canali del sensore INA3211 */

    /**
     * @brief Costruttore di default. Inizializza l'oggetto INA3211.
     */
    INA3211() : Adafruit_INA3221() {}

    /**
      * @brief Inizializza il sensore INA3221 con configurazione personalizzata.
      * Sovrascrive il begin() originale della libreria Adafruit.
      * 
      * @param addr Indirizzo I2C del sensore (default 0x40).
      * @return true se inizializzazione riuscita, false altrimenti.
      */
    bool begin(uint8_t addr = 0x40);

    /**
     * @brief Calcola la potenza in Watt su uno dei canali INA3221.
     * @param channel Canale da cui leggere (valori validi: 1, 2, 3).
     * @return Potenza calcolata (volt × ampere) in Watt.
     */
    float getPower(uint8_t channel);

    /**
     * @brief Stima lo stato di carica (State of Charge, SoC) in percentuale da una misura di tensione batteria.
     * @param Vmeas Tensione della batteria in mV.
     * @return Stato di carica stimato in percentuale (0–100%).
     */
    int vbToSoc(float Vmeas);

    float readIrraggiamento(void);
    String readCloudCover(void);
    /**
     * @brief Legge i dati dai 3 canali del sensore INA3221 e li memorizza in una struttura INA.
     */
    void read(void);

    INA getData(void);

    private:
    bool CorrSOC = false;  /**< Flag per abilitare la correzione dello stato di carica (SoC) */


};

bool INA3211::begin(uint8_t addr) {
  if (!Adafruit_INA3221::begin(addr)) {
    return false;
  }

  // Imposta media su 16 campioni
  setAveragingMode(INA3221_AVG_16_SAMPLES);

  // Imposta resistenze shunt per tutti i canali a 0.05 Ohm
  for (uint8_t i = 0; i < 3; i++) {
    setShuntResistance(i, 0.05);
  }

  // Imposta limiti validi di potenza  3.0 lower limit, 15.0 upper limit
  setPowerValidLimits(3.0, 15.0);

  return true;
}

float INA3211::getPower(uint8_t channel) {
  float voltage = getBusVoltage(channel);  // Legge la tensione
  float current = getCurrentAmps(channel) ;  // Converte da mA a A
  return voltage * current;  // Restituisce la potenza in Watt
}

int INA3211::vbToSoc(float Vmeas){
  int Vbat[] = {4187,4111,4080,4047,3996,3940,3891,3849,3806,3759,3714,3670,3632,3596,3554,3501,3456,3411,3309,3150,2925};
  int DOD[] = {0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100};
  int n = sizeof(Vbat) / sizeof(Vbat[0]); // Numero di punti nel vettore.

  // Cerca l'intervallo in cui si trova x0.

  if (Vmeas>Vbat[0]) {
      return (int)100;
      }
  if (Vmeas<Vbat[n-1]) {
      return (int)0;
      }
       
  for (int i = 0; i < n - 1; i++) {
      if (Vbat[i] >= Vmeas && Vmeas > Vbat[i + 1]) {        
          return (int)(100 - matLinInt(Vbat[i], Vbat[i + 1], DOD[i], DOD[i + 1], Vmeas));
      }
  }
  return (int)0;
}

float INA3211::readIrraggiamento(){
  float power = getPower(PANNEL_IN);
  // Calcola l'irraggiamento solare in W/m²
  if (power <= 0.0) {
    return 0.0; // Evita divisioni per zero
  }
  power = power / EFFICIENZA_PANNELLO_SOLARE; // Considera l'efficienza del pannello solare
  float irradiance = power / AREA_PANNELLO_SOLARE; // Potenza in W divisa per l'area in m²

  // Converte l'irraggiamento in W/m²
  irradiance = constrain(irradiance, 0.0, 1000.0); // Limita l'irraggiamento a un massimo di 1000 W/m²
  return irradiance;
}

String INA3211::readCloudCover() {

  float irradiance = readIrraggiamento();
  if (irradiance > 800) {
    return "Sereno";
  } else if (irradiance > 500) {
    return "Parzialmente nuvoloso";
  } else if (irradiance > 200) {
    return "Nuvoloso";
  } else if (irradiance > 50) {
    return "Molto nuvoloso";
  } else {
    return "Notte";
  }

}

// TODO : Rivedere la struttura INA e il metodo read() per restituire un oggetto INA con i dati dei 3 canali.
void INA3211::read() {
  for (uint8_t i = 0; i < 3; i++) {
    auto safe = [&](float v) { return v > 0 ? v : 0; };
    inaData.current[i] = safe(getCurrentAmps(i));
    inaData.voltage[i] = safe(getBusVoltage(i));
    inaData.power[i] = getPower(i);
  }
  if (CorrSOC == true) {
      float battery_voltage = inaData.voltage[BATTERY_IN] - inaData.current[BATTERY_IN] * 0.01;
    // Se la correzione dello stato di carica è abilitata, sottrae 0.01V per compensare la caduta di tensione
    inaData.soc = vbToSoc(battery_voltage * 1000.0F); // Converti da V a mV
  }else{
    // Altrimenti, usa la tensione misurata direttamente
    inaData.soc = vbToSoc(inaData.voltage[BATTERY_IN] * 1000.0F); // Converti da V a mV
  }
  inaData.irradiance = readIrraggiamento(); // Legge l'irraggiamento solare
  inaData.cloud_cover = readCloudCover(); // Legge la copertura nuvolosa

  // return ina;
}

INA3211::INA INA3211::getData(void) {
  return inaData;  // Restituisce i dati letti
}



class BME280 : public Adafruit_BME280 {

  public:
      typedef struct {
      float temperature; /**< Temperatura (°C) */
      float humidity;    /**< Umidità relativa (%) */
      float pressure;    /**< Pressione atmosferica (hPa) */
      float altitude;    /**< Altitudine calcolata (m) */
    } BME;

    BME bmeData;  /**< Dati letti dal sensore BME280 */

    /**
     * @brief Costruttore di default. Inizializza l'oggetto BME280.
     */
    BME280() : Adafruit_BME280() {}
    /**
     * @brief Inizializza il sensore BME280 con configurazione personalizzata.
     * Sovrascrive il begin() originale della libreria Adafruit.
     * 
     * @param addr Indirizzo I2C del sensore (default 0x76).
     * @return true se inizializzazione riuscita, false altrimenti.
     */
    bool begin(uint8_t addr = 0x76);

    /**
     * @brief Legge i dati dal sensore BME280 e li memorizza in variabili.
     */
    void read(void);

   BME getData(void);

};

bool BME280::begin(uint8_t addr) {
  if (!Adafruit_BME280::begin(addr)) {
    return false;
  }

  return true;
}

void BME280::read() {
  // Legge i dati dal sensore BME280
  bmeData.temperature = readTemperature();
  bmeData.humidity = readHumidity();
  bmeData.pressure = readPressure() / 100.0F; // Converti da Pa a hPa
  bmeData.altitude = readAltitude(bmeData.pressure); // Calcola l'altitudine
  return;
}

BME280::BME BME280::getData(void){
  return bmeData;
}






/**
 * @brief Calcola l'indice di qualità dell'aria in base ai valori dei principali inquinanti.
 * 
 * L'indice viene calcolato come media ponderata dei punteggi relativi a:
 * - PM2.5 (particolato fine)
 * - PM10 (particolato grossolano)
 * - CO₂ (anidride carbonica)
 * - NO₂ (biossido di azoto)
 * - NH₃ (ammoniaca)
 * - CO (monossido di carbonio)
 * 
 * Ogni parametro contribuisce all'indice secondo un peso specifico.
 * 
 * @param valPM25 Valore di PM2.5 in µg/m³.
 * @param valPM10 Valore di PM10 in µg/m³. OMS: media giornaliera consigliata ≤ 45 µg/m³ Effetti respiratori visibili > 70 µg/m³
 * @param valCO2 Valore di CO₂ in ppm.
 * @param valNO2 Valore di NO₂ in ppm.  OMS: ≤ 25 ppb (media annuale), ≤ 106 ppb (1 ora) 200 ppb → sintomi respiratori acuti
 * @param valNH3 Valore di NH₃ in ppm. Irritazione oculare già da 1 ppm Soglia olfattiva ≈ 0.2–0.4 ppm
 * @param valCO Valore di CO in ppm. 15 ppm: potenziali effetti cardiovascolari
 * @return Indice di qualità dell'aria (0-100), dove valori più alti indicano aria migliore.
 */
class AIRQuality {
  public:

    /*
    PM2.5	30%	Molto pericoloso per i polmoni
    PM10	15%	Meno penetrante
    CO₂	15%	Disagio e ventilazione
    NO₂	15%	Irritante respiratorio
    NH₃	10%	Tossico ma meno comune
    CO	15%	Tossico acuto
    */
    float indiceQualita(float valPM25, float valPM10, int valCO2, float valNO2, float valNH3, float valCO);
    String getAirQualityString();

  private:

    int punteggioCO2(int ppm);
    int punteggioPM25(float ugm3);
    int punteggioPM10(float pm10);
    int punteggioNO2(float no2_ppm);
    int punteggioNH3(float nh3_ppm);
    int punteggioCO(float co_ppm);

    const float PESO_PM25 = 0.30;
    const float PESO_PM10 = 0.15;
    const float PESO_CO2  = 0.15;
    const float PESO_NO2  = 0.15;
    const float PESO_NH3  = 0.10;
    const float PESO_CO   = 0.15;
    float _score; /**< Punteggio totale dell'indice di qualità dell'aria */
};


int AIRQuality::punteggioCO2(int ppm) {
  if (ppm <= 600) return 100;
  if (ppm <= 1000) return 80;
  if (ppm <= 1500) return 60;
  if (ppm <= 2000) return 40;
  return 20;
}
int AIRQuality::punteggioPM25(float ugm3) {
  if (ugm3 <= 10) return 100;
  if (ugm3 <= 25) return 80;
  if (ugm3 <= 50) return 60;
  if (ugm3 <= 75) return 40;
  return 20;
}

int AIRQuality::punteggioPM10(float pm10) {
  if (pm10 <= 20) return 100;
  if (pm10 <= 40) return 80;
  if (pm10 <= 70) return 60;
  if (pm10 <= 100) return 40;
  return 20;
}
int AIRQuality::punteggioNO2(float no2_ppm) {
  float no2_ppb = no2_ppm * 1000;  // conversione ppm → ppb

  if (no2_ppb <= 40) return 100;
  if (no2_ppb <= 75) return 80;
  if (no2_ppb <= 125) return 60;
  if (no2_ppb <= 200) return 40;
  return 20;
}
int AIRQuality::punteggioNH3(float nh3_ppm) {
  if (nh3_ppm <= 0.2) return 100;
  if (nh3_ppm <= 0.5) return 80;
  if (nh3_ppm <= 1.0) return 60;
  if (nh3_ppm <= 2.0) return 40;
  return 20;
}
int AIRQuality::punteggioCO(float co_ppm) {
  if (co_ppm <= 1) return 100;
  if (co_ppm <= 5) return 80;
  if (co_ppm <= 9) return 60;
  if (co_ppm <= 15) return 40;
  return 20;
}

float AIRQuality::indiceQualita(float valPM25, float valPM10, int valCO2, float valNO2, float valNH3, float valCO) { 
  _score = 0;
  _score += punteggioPM25(valPM25) * PESO_PM25;
  _score += punteggioPM10(valPM10) * PESO_PM10;
  _score += punteggioCO2(valCO2) * PESO_CO2;
  _score += punteggioNO2(valNO2) * PESO_NO2;
  _score += punteggioNH3(valNH3) * PESO_NH3;
  _score += punteggioCO(valCO) * PESO_CO;
  return _score;
}

String AIRQuality::getAirQualityString() {
  float score = _score;
  if (score >= 80) return "Eccellente";
  else if (score >= 60) return "Buona";
  else if (score >= 40) return "Moderata";
  else if (score >= 20) return "Scarsa";
  else return "Molto scarsa";
} 



#endif // _ALL_SENSORS_H_