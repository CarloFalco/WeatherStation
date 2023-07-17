# WeatherStation
This program is based on ESP 32, to monitor the weather

This program is based on [ESP32](https://github.com/CarloFalco/WeatherStation/tree/main/Datasheet/ESP32-38.png) to monitor the weather

Il progetto è stato suddiviso in sottocartelle 

| Cartella 		| Contenuto 		|
| ------ 		| ------ 			|
| 3D Model  	| File da stampare 	|
| Code  		| Codice   			|


## Components
### Pluviometro
basato su sensore ad effetto hall
### Anemometro
basato su sensore ad effetto hall
direzione vento

### Termometro / Barometro
vado a leggere entrambi i sensori e faccio una media pesata come pare a me


### pluviometro
integro 60s e publico il risultato


basato su sensore [BME280](Datasheet/BME280_Datasheet.pdf)
Library: 
https://github.com/adafruit/Adafruit_BME280_Library.git

basato su sensore [DHT22](Datasheet/DHT11.pdf)
Library: 
https://github.com/adafruit/Adafruit_BME280_Library.git

basato su sensore [MQ135](Datasheet/MQ-135_Hanwei.pdf)
Library: 
https://github.com/adafruit/Adafruit_BME280_Library.git


## to DO:
creare un .h con tutte le info che ci interessano 
typedef struct {

Vento // m/s
direzione // 0-360 

Temperatura // °C
Umidita 	// %
pressione   // 


pluviometro[24] // ora per ora




pm10
pm2.5
CO
NH3
NO2

CO2 
TVOC


} MyStruct;


aggingere la scrittura su sd con time stamp 




