/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
// initialization counters
unsigned long Count500ms;
unsigned long PreviousCount500ms = 0;

unsigned long Count1s;
unsigned long PreviousCount1s = 0;



int itsAliveLed = 16;
float sensorValue = 0;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(itsAliveLed, OUTPUT);
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {


  // TASK 500ms
  Count500ms = millis();
  if (Count500ms - PreviousCount500ms > 500)
  { 
    PreviousCount500ms = Count500ms;  
    sensorValue = analogRead(A0)*3.3/1024; 
    // print out the value you read:
    Serial.printf("sensor = %.3f\n", sensorValue);
  }


  // TASK 1s
  Count1s = millis();
  if (Count1s - PreviousCount1s > 1000)
  {
    PreviousCount1s = Count1s;

    digitalWrite(itsAliveLed, !digitalRead(itsAliveLed));
  }
}
