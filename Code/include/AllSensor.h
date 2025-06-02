
//#include <Adafruit_BME280.h>
#include <Adafruit_INA3221.h>
//#include "MICS6814.h"
//#include "SparkFunCCS811.h"
//#include "Adafruit_PM25AQI.h"

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

    /**
     * @brief Restituisce la direzione del vento in formato testuale (es. "N", "NE", ...).
     * @return Stringa rappresentante la direzione cardinale.
     */
    String getDirection(void);

    /**
     * @brief Restituisce l'angolo della direzione del vento in gradi (0–360).
     * @return Angolo in gradi rispetto al nord.
     */
    float getWindAngle(void);

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
  Wire.requestFrom(_addr, 2);
  _rawAngle = Wire.read() << 8 | Wire.read();
  rawToAngle();

}

void Windvane::rawToAngle(void) {
  _angle = (_rawAngle * 360.0) / 4096.0; // Converti il valore grezzo in gradi
}

String Windvane::getDirection(void) {
  readRawAngle();
  float angle = _angle;
  if (angle >= 337.5 || angle < 22.5) return "N";
  else if (angle >= 22.5 && angle < 67.5) return "NE";
  else if (angle >= 67.5 && angle < 112.5) return "E";
  else if (angle >= 112.5 && angle < 157.5) return "SE";
  else if (angle >= 157.5 && angle < 202.5) return "S";
  else if (angle >= 202.5 && angle < 247.5) return "SW";
  else if (angle >= 247.5 && angle < 292.5) return "W";
  else return "NW";
}

float Windvane::getWindAngle(void){
  readRawAngle();
  return _angle; 
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

// Dichiarazione del oggetto Energy Sensor



/**
 * @class INA3211
 * @brief Estensione della libreria Adafruit_INA3221 per aggiungere funzionalità di monitoraggio della potenza e stato di carica (SoC).
 */
class INA3211 : public Adafruit_INA3221 {
  public:
    /**
     * @brief Costruttore di default. Inizializza l'oggetto INA3211.
     */
    INA3211() : Adafruit_INA3221() {}

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

  private:
    /**
    * @struct INA
    * @brief Struttura dati per contenere informazioni elettriche su 3 canali.
    * - Channel 1: Pannello solare.
    * - Channel 2: Batteria.
    * - Channel 3: Load (carico).
    * 
     */
    typedef struct {
      float current[3];  /**< Corrente per ogni canale (A) */
      float voltage[3];  /**< Tensione per ogni canale (V) */
      float power[3];    /**< Potenza per ogni canale (W) */
      int soc;           /**< Stato di carica stimato (%) */
    } INA;


};


float INA3211 :: getPower(uint8_t channel) {
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