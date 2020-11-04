/*
  PIR

  Changes the behavior between on and off an LED when a PIR sensor is activated.
  Assumes the use of a PIR sensor that needs no pull-up

  This example code is in the public domain.

  created in Feb 2019 by D. Cuartielles
  based on work by F. Vanzati (2011) and M. Loglio (2013)
 */

// include the EduIntro library
#include <EduIntro.h>

PIR pir(D7);	// creating the object 'pir' on pin D7
#define BUZZER_PIN 8
Led led(D10);		// creating the object 'led' on pin D10
long long activated_time;
bool is_activated = false;


void setup() {
//nothing here
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop()
{
  // check the state of the PIR sensor
  // if you just want to detect the turn from not active to active
  // use pir.activated() instead
  if (pir.activated()) {
      led.on();
      Serial.println("DEACTIVE");
      
  }
  else if (pir.deactivated()) {
      led.off();
      Serial.println("ACTIVE");
      tone(BUZZER_PIN, HIGH);
      activated_time = millis();
      is_activated = true;
  }

  if (is_activated && millis() - activated_time > 10) {
    digitalWrite(BUZZER_PIN, LOW);
    activated_time = 0;
    is_activated = false;
  }
}
