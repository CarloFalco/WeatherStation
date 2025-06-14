#ifndef MICS6814_H
#define MICS6814_H

#include "Arduino.h"
#include <driver/adc.h>

typedef enum{
  CH_NH3, CH_RED, CH_OX
} channel_t;

typedef enum{
  CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH
} gas_t;

class MICS6814
{

public:

    typedef struct {
      float NH3Value;        /**< Valore di NH3 in ppm */
      float COValue;         /**< Valore di CO in ppm */
      float NO2Value;       /**< Valore di NO2 in ppm */
    } MICS;

    MICS micsData;  /**< Dati letti dal sensore di direzione del vento */

	MICS6814(adc1_channel_t pinCO, adc1_channel_t pinNO2, adc1_channel_t pinNH3);

  	void begin(bool EnableCalibrationProcedure);
	int calibrate();
	void loadCalibrationData(uint16_t base_NH3,uint16_t base_RED,uint16_t base_OX);
	void read();
	MICS getData() const;

	float measure(gas_t gas);

	uint16_t getResistance    (channel_t channel) const;
	uint16_t getBaseResistance(channel_t channel) const;
	float    getCurrentRatio  (channel_t channel) const;

private:
	adc1_channel_t _pinCO;
	adc1_channel_t _pinNO2;
	adc1_channel_t _pinNH3;

	uint16_t _baseNH3;
	uint16_t _baseCO;
	uint16_t _baseNO2;
};

#endif