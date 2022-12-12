/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int BitsensorValueN = 0;  // variable to store the value coming from the sensor
float BitsensorValue = 0;  // variable to store the value coming from the sensor
float Voltsensor1 = 0;  // Voltage on the sensor
float Voltsensor2 = 0;  // Voltage on the pin

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600); 
}

void loop() {
  // read the value from the sensor:
  for (int i = 1; i<=15; i ++){
    BitsensorValueN = BitsensorValueN + analogRead(sensorPin);
    delayMicroseconds(100);
    } 
  BitsensorValue = BitsensorValueN/15;
  Voltsensor2 = BitsensorValue*3.35/4095+0.14;
  Voltsensor1 = Voltsensor2*15/10;
  Serial.print(BitsensorValue); 
  Serial.print(" [bit]  |   ");
  
  Serial.print(Voltsensor2); 
  Serial.print(" [V]  |   ");
  Serial.print(Voltsensor1); 
  Serial.println(" [V]");
  delay(500);
  BitsensorValueN = 0;
} 
