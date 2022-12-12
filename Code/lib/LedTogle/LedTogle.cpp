#include "LedTogle.h"
#include "Arduino.h"


// QUI METTO LE COSE RELATIVE AL LAMPEGGIO DEL LED
int ledState;                           // ledState used to set the LED


void ledTogle_PwrOn(void)
{

  ledState = LOW;             // ledState used to set the LED
  pinMode(INTERNAL_LED, OUTPUT);  
}




void ledTogle_tsk(void)
{
    if (ledState == LOW)
      {
        ledState = HIGH;
        Serial.println("LED ON");
        Serial.println(ledState);
      }
    else
      {
        ledState = LOW;
        Serial.println("LED OFF");
        Serial.println(ledState);
      }
    digitalWrite(INTERNAL_LED, ledState);
}