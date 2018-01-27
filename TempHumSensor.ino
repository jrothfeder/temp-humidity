#include "DHT.h"
#include "Math.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <AccelStepper.h>

#define STEPS 600
#define DHTTYPE DHT22
#define DHTPIN 5
#define SPEED 60.0
#define HOME -100
#define CALLIBRATION_MODE false
#define ACTION_MS 2000
#define FACTOR 4.4

DHT dht(DHTPIN, DHTTYPE);

int lastOutputTemp=0;
int lastOutputHumd=0;

Adafruit_MotorShield afms = Adafruit_MotorShield(); 

Adafruit_StepperMotor *tempMotor = afms.getStepper(600, 1);
Adafruit_StepperMotor *humdMotor = afms.getStepper(600, 2);

void hForwardstep1() {  
  humdMotor->onestep(FORWARD, SINGLE);
}
void hBackwardstep1() {  
  humdMotor->onestep(BACKWARD, SINGLE);
}

void tForwardstep1() {  
  tempMotor->onestep(FORWARD, SINGLE);
}

void tBackwardstep1() {  
  tempMotor->onestep(BACKWARD, SINGLE);
}

AccelStepper hStepper(hForwardstep1, hBackwardstep1);
AccelStepper tStepper(tForwardstep1, tBackwardstep1);

bool initialize = true;

void setup() {
  Serial.begin(9600);
  
  afms.begin();
  dht.begin();

  tStepper.setMaxSpeed(SPEED);
  tStepper.setAcceleration(SPEED);
  
  hStepper.setMaxSpeed(SPEED);
  hStepper.setAcceleration(SPEED);

  // Move to an initial position
  tStepper.move(HOME);
  hStepper.move(HOME);
}

long lastTempCheck = 0;

void moveDials(float temp, float humidity) {
  int toMoveF = roundf(humidity*FACTOR);
  int toMoveH = roundf(temp*FACTOR);

  tStepper.moveTo(toMoveF);
  hStepper.moveTo(toMoveH);

  Serial.print("Temperature: ");
  Serial.println(toMoveF);
  Serial.println(temp);

  Serial.print("Humidity: ");
  Serial.println(toMoveH);
  Serial.println(humidity);
}

void checkTempFromSensorsAndSetDials() {
  if( (millis() - lastTempCheck) > 2000) {
    float h = dht.readHumidity();
    float f = dht.readTemperature(true);
    lastTempCheck = millis();
    
      // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(f)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      moveDials(f,h);
    }
  }
}

int callibrationVal = 0;
void callibrate() {
  moveDials(callibrationVal, callibrationVal);
  if(callibrationVal < 100) {
    callibrationVal += 10;
  } else {
    callibrationVal = 0;
  }
}

void performAction() {
  if(CALLIBRATION_MODE) {
    callibrate();
  } else {
    checkTempFromSensorsAndSetDials();
  }
}

void loop() {
  
  if(hStepper.distanceToGo() != 0 || tStepper.distanceToGo() != 0) {
    // If we're currently moving, carry on
  } else {
    if(initialize) {
      // If we were initializing, we've found '0'
      initialize = false;
      tStepper.setCurrentPosition(0);
      hStepper.setCurrentPosition(0);
    }

    if( (millis() - lastTempCheck) > ACTION_MS) {
      performAction();
      lastTempCheck = millis();
    } 
  }
  tStepper.run();
  hStepper.run();
}


