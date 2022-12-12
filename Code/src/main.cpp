/*********
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>



/*#include <SPI.h>
#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15*/

#define SEALEVELPRESSURE_HPA (1013.25)
void printValues(void);

Adafruit_BME280 bme; // I2C
unsigned long delayTime;
float BME_Temperature; // Temperature
float BME_Humidity; // Humidity
float BME_Pressure; // Pressure
float BME_Altitude; // Altitude


#define DHTPIN 19   //Pin a cui è connesso il sensore
#define DHTTYPE DHT22   //Tipo di sensore che stiamo utilizzando (DHT22)
DHT dht(DHTPIN, DHTTYPE); //Inizializza oggetto chiamato "dht", parametri: pin a cui è connesso il sensore, tipo di dht 11/22
//Variabili
float DHT_Temperature; // Temperature
float DHT_Humidity; // Humidity


//Include the library
#include <MQUnifiedsensor.h>

//Definitions
#define placa "ESP 32"
#define Voltage_Resolution 3.3 // 3.3
#define pin A0 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO 12
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  

// Creo l'oggetto
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton  | 34.668 | -3.369
  */



void setup() {
    Serial.begin(9600);
    Serial.println(F("BME280 test"));

    bool status;

    // configurazione del BME280
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin(0x76);  
    if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    }

    Serial.println("-- Default Test --");
    delayTime = 1000;

    //configurazione del DHT22
    dht.begin();
    Serial.println();

    //configurazione del DHT22  
    MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to to calculate NH4 concentration
    MQ135.init(); 

    Serial.print("Calibrating please wait.");
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
    }
    MQ135.setR0(calcR0/10);
    Serial.println("  done!.");

    if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
    if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
    /*****************************  MQ CAlibration ********************************************/ 
    //MQ135.serialDebug(true);

}


void loop() { 
    printValues();
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    //MQ135.serialDebug(); // Will print the table on the serial port
    Serial.println(MQ135.readSensor()); // Serial
    delay(delayTime);

}


void printValues() {

    BME_Temperature = bme.readTemperature();
    BME_Humidity = bme.readHumidity();
    BME_Pressure = bme.readPressure() / 100.0F;
    BME_Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.println("--------------------");
    Serial.println("BME280");
    Serial.print("Temperature = ");
    Serial.print(BME_Temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(BME_Pressure);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(BME_Altitude);
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(BME_Humidity);
    Serial.println(" %");

    Serial.println();

    if (!isnan(dht.readHumidity())) {
        DHT_Humidity = dht.readHumidity();
        DHT_Temperature= dht.readTemperature();
    }
    Serial.println("--------------------");
    Serial.println("DHT22");
    Serial.print("Umidità: ");
    Serial.print(DHT_Humidity);
    Serial.print(" %, Temp: ");
    Serial.print(DHT_Temperature);
    Serial.println(" Celsius");

}