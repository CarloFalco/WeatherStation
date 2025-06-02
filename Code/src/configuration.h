

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
#define M_PI    3.141592        /* PIGRECO*/
#define M_1_PI 	0.31831         /* 1/PIGRECO*/
#define M_SQRT2 1.4142          /* sqrt(2)*/


// POWERLINE
#define PIN_5V GPIO_NUM_11 // Pin 11 del GPIO, che è il pin 5V
#define PIN_3V GPIO_NUM_4 // Pin 4 del GPIO, che è il pin 3V3


#define TIME_TO_SLEEP  20  // Time to sleep in seconds
#define TASK_FAST 500   // Task execution time in milliseconds




// ANEMOMETER
#define PIN_ANEMOMETER GPIO_NUM_19

#define ANEMOMETER_DIAMETER 80 * 2 * mm_TO_m_FACTOR
#define TASK_ANEMOMETER 200  // Task execution time in milliseconds

// RAINGAUGE
#define PIN_RAINGAUGE GPIO_NUM_20


#define AREA_PLUVIOMETRO 63.25 //11.5 x 5.5 = 63.25 cm2
#define AREA_VASCHETTA 3 //3 ml ogni iterazione
#define CONVERSIONE_PLUVIOMETRO ((AREA_VASCHETTA)/(AREA_PLUVIOMETRO)*1000) // (3) 3 x 10^-3 /( 63.25 x 10^-6) = 3 x 10^3 / 63.25 = 47.43  [mm h oppure Litri per m2 ora]

#define CYCLE_RAIN_LENGTH 10 // tempo in secondi




// LED
#define LED_BUILTIN 97



#endif // _TYPE_CONVERSION_H_