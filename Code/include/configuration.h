

#ifndef _TYPE_CONVERSION_H_
#define _TYPE_CONVERSION_H_



#define S_TO_uS_FACTOR 1000000UL                /* Conversion factor for seconds to micro seconds */
#define uS_TO_S_FACTOR (1.0 / S_TO_uS_FACTOR)   /* Conversion factor for micro seconds to seconds */

#define S_TO_mS_FACTOR 1000ULL                  /* Conversion factor for seconds to milli seconds */
#define mS_TO_S_FACTOR (1.0 / S_TO_mS_FACTOR)   /* Conversion factor for milli seconds to seconds */

#define mS_TO_uS_FACTOR 1000ULL  /* Conversion factor for milli seconds to micro seconds */


#define h_TO_S_FACTOR 3600   /* Conversion factor for micro seconds to seconds */


#define m_TO_mm_FACTOR 1000ULL    /* Conversion factor for meter to millimeter */
#define mm_TO_m_FACTOR (1.0/m_TO_mm_FACTOR)    /* Conversion factor for millimeter to meter */


/* https://learn.microsoft.com/it-it/cpp/c-runtime-library/math-constants?view=msvc-170 */
// #define M_PI    3.141592        /* PIGRECO*/
// #define M_1_PI 	0.31831         /* 1/PIGRECO*/
// #define M_SQRT2 1.4142          /* sqrt(2)*/

// #define SWVERSION "1.0.0" // Versione del firmware
#define SWVERSION_NUM 2025061504 // Versione del firmware in formato YYYYMMDDRR (RR = Release del giorno)

// POWERLINE
#define PIN_5V GPIO_NUM_11 // Pin 11 del GPIO, che è il pin 5V
#define PIN_3V GPIO_NUM_4 // Pin 4 del GPIO, che è il pin 3V3


#define TIME_TO_SLEEP  300  // Time to sleep in seconds // TODO: da ripristinare a 300
#define TASK_FAST 500   // Task execution time in milliseconds
#define TASK_CCS811 1000   // Task execution time in milliseconds


// ANEMOMETER
#define PIN_ANEMOMETER GPIO_NUM_20
#define WAKEUP_PIN_1 PIN_ANEMOMETER  // Pin RTC 33

#define ANEMOMETER_DIAMETER 80 * 2 * mm_TO_m_FACTOR
#define TASK_ANEMOMETER 200  // Task execution time in milliseconds

// WINDVANE
#define AS5600_ADDRESS 0x36
#define RAW_ANGLE 0x0C


// RAINGAUGE
#define PIN_RAINGAUGE GPIO_NUM_19
#define WAKEUP_PIN_2 PIN_RAINGAUGE  // Pin RTC 34


#define AREA_PLUVIOMETRO 63.25 //11.5 x 5.5 = 63.25 cm2
#define AREA_VASCHETTA 3 //3 ml ogni iterazione
#define CONVERSIONE_PLUVIOMETRO ((AREA_VASCHETTA)/(AREA_PLUVIOMETRO)*1000) // (3) 3 x 10^-3 /( 63.25 x 10^-6) = 3 x 10^3 / 63.25 = 47.43  [mm h oppure Litri per m2 ora]

#define CYCLE_RAIN_LENGTH 10 // tempo in secondi



// TERMOMETER
#define BME280_ADDRESS 0x76

// GASSENSOR MICS6814   
#define PIN_CO  ADC1_CHANNEL_4
#define PIN_NO2 ADC1_CHANNEL_5
#define PIN_NH3 ADC1_CHANNEL_6


#define MAX_NH3 35
#define MIN_NH3 25

#define MAX_NO2 0.1
#define MIN_NO2 0.021

#define MAX_CO2 15000
#define MIN_CO2 5000

#define MAX_CO 100
#define MIN_CO 20

// GASSENSOR CCS811
#define CCS811_ADDRESS 0x5A
#define PIN_WAK GPIO_NUM_41
#define CCS811_BASELINE 5201 // Baseline value for CCS811 sensor, used for TVOC and CO2 calculations
#define CCS811_MODE 1 // Mode for CCS811 sensor
//Mode 0 = Idle
//Mode 1 = read every 1s
//Mode 2 = every 10s
//Mode 3 = every 60s
//Mode 4 = RAW mode

// 0,3 mg/m³ ; 0,3 - 0,5 mg/m³; 0,5 - 1 mg/m³, 1 - 3 mg/m³, Oltre 3 mg/m³
#define MAX_TVOC 600
#define MIN_TVOC 100

// CURRENT SENSOR INA
#define INA_ADDRESS 0x40

// definisce in che canale sono collegati i sensori
#define PANNEL_IN 2 // PANNEL IN 
#define BATTERY_IN 1 // BATTERY IN   
#define LOAD_IN 0 // LOAD IN

// predisposizione per considerare il verso della corrente
#define PANNEL_DIR 0 // PANNEL IN A
#define BATTERY_DIR 0 // BATTERY IN A   
#define LOAD_DIR 0 // LOAD IN A

#define AREA_PANNELLO_SOLARE 0.01 // Area del pannello solare in m² (esempio: 0.1 m²)
#define EFFICIENZA_PANNELLO_SOLARE 0.15 // Efficienza del pannello solare (esempio: 15%)

// PM25AQI
#define PM25_TX 0x12 // PIN 18 (RX) del esp32-s3-devkitc-1
#define PM25_RX 0x11 // PIN 17 (TX) del esp32-s3-devkitc-1
#define PM25_UART_BAUDRATE 9600




// LED
#undef LED_BUILTIN
#define LED_BUILTIN 97




#define CONVERSIONE_POTENZA ((TIME_TO_SLEEP*100)/(TASK_FAST)) 

#endif // _TYPE_CONVERSION_H_