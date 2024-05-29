//Motion, light and tilt switch Alarm

#include "Timer.h"
#include <NewPing.h>

//global variables
unsigned char 
const int tiltLEDpin = 8;
const int tiltSwitch = 12;
const int buzzerPin = 9;
const int photoResistorPin = A0;
const int photoLEDpin = 7;
//

//read photo resistor light values
int lightVal = analogRead(photoResistorPin);
static unsigned int lastDist = 0;
unsigned int currentDist = sonar.ping_cm(); //gets the current distance of the nearest object.

enum TiltState {INITIAL, ACTUATED};  
enum LightLevel {LIGHT_DETECT, LIGHT_OFF, LIGHT_ON};
enum MotionDetect {NONE, MOTION };
enum SystemOperations { STARTUP, NORMAL_OPERATION, ALARM, SHUTDOWN };
//configure LED states for the system, when tilt switch is on + no alarm + no motion: LED on, tilt switch on + motion + alarm = LED flashing, tilt switch off = LED off
enum LEDstates {INIT, OFF, SOLID, FLASH};
//implement this soon.

gState = INITIAL, LIGHT_DETECT, NONE;
SystemOperations operations = STARTUP;


struct SystemStates {

  TiltState tilt;
  LightLevel light;
  MotiionDetect motion;

};


void updateTiltState();
void updateLightLevel();
void updateMotionDetect();
void actions();

//ultrasonic sensor
//NewPing sonar (triggerPin, echoPin, maxDistance(cm))
NewPing sonar(10, 11, 50);

void setup() {
  //Set up pins
  pinMode (tiltLEDpin, OUTPUT);
  pinMode (buzzerPin, OUTPUT);
  pinMode (tiltSwitch, INPUT_PULLUP);

  //LED and buzzer are turned off
  digitalWrite (tiltLEDpin, LOW);
  digitalWrite (buzzerPin, LOW);

  //dont forget to setup the tilt switch.
  Serial.begin(9600);

}

void loop() {
  
  updateTiltState();
  updateLightLevel();
  updateMotionDetect();
  actions();
  
  while (!TimerFlag) {}
  TimerFlag = 0;

}

void updateTiltState() {
    /*if (digitalRead(tiltSwitch) == HIGH) {
      digitalWrite (tiltLEDpin, HIGH);
      digitalWrite (buzzerPin, HIGH);
    }*/
  // Read the tilt ball switch and update the orientation
  //transitions
  switch (gState) {
    case INITIAL:
      if (digitalRead(tiltSwitch) == HIGH) {
        gState = ACTUATED;
      }
      else {
        gState = INITIAL;
      }

      break;
    case ACTUATED:
      if (digitalRead(tiltSwitch) == LOW) {
        gState = INITIAL;
      }
      else {
        gState = ACTUATED;
      }

      break;
  }

  //actions
  switch (gState) {
    case INITIAL:
      digitalWrite(tiltLEDpin, LOW);
      break;
    case ACTUATED:
      digitalWrite(tiltLEDpin, HIGH);
      break; 
  }
}

void updateLightLevel() {
  // Read and update light sensor/state
  //transitions
  switch(gState) {
    case LIGHT_DETECT:
      //lightVal < 20 = dark, else = light;
      if (lightVal <= 20) {
        gState = LIGHT_OFF;
      }
      else {
        gState = LIGHT_ON;
      }
    case LIGHT_OFF:
      if (lightVal > 20) {
        gState = LIGHT_ON;
      }
      else {
        gState = LIGHT_OFF;
      }
    case LIGHT_ON:
      if (lightVal > 20) {
        gState = LIGHT_ON;
      }
      else {
        gState = LIGHT_OFF;
      }
  }
    //actions
    switch (gState) {
      case LIGHT_DETECT:
        break;
      case LIGHT_OFF:
        digitalWrite(photoLEDpin, HIGH); //turn on photo resistor LED
        break;
      case LIGHT_ON:
        digitalWrite(photoLEDpin, LOW); // turn off
        break;
    }
}

void updateMotionDetect() {
  currDist = sonar.ping_cm();
  // Read the ultrasonic sensor
} 

void actions(SystemStates &states) {
  unsigned short currentLightLevel;
  unsigned short currentObjectDetection;
  switch (gState) {
    //transitions
    case STARTUP:

    break;

    case NORMAL_OPERATION:

    break;

    case ALARM:

    break;

    case SHUTDOWN:

    break;
  }
    
    //actions
  switch (gState) {
    case STARTUP:

      break;
    case NORMAL_OPERATION:
    
      break;
    case ALARM:
     //sounds the active buzzer and maybe flashes the LED.
      break;
    case SHUTDOWN:

      break;
  }
}
