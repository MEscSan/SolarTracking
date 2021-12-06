#include "Stepper.h"
#include "SolarCalculator.h"

// Macros for stepper motors
#define DIR_X_PIN 5     // Nema17 X-Direction
#define STEP_X_PIN 2    // Nema17 X-Step
#define DIR_Y_PIN 6     // Nema 17 Y-Direction
#define STEP_Y_PIN 3    // Nema 17 Y-Step
#define LED_PIN 13
#define MICROSECONDS_PER_STEP 1000
#define MICROSECONDS_PER_STEP_X 1000
#define MICROSECONDS_PER_STEP_Y 1000
#define GEAR_RATIO 5 // Input speed / Output speed : Ratio > 1 => gear slows movement down 
#define GEAR_RATIO_X 19.2 
#define GEAR_RATIO_Y 19.2 
#define NUM_STEPPERS 2
#define STEPS_PER_REVOLUTION_NEMA17 200
#define STEPS_PER_REVOLUTION_28BYJ 2048
#define STEPPER_TYPE StepperType::NEMA17 

int xStepperPins[2] = { DIR_X_PIN, STEP_X_PIN };
int yStepperPins[2] = { DIR_Y_PIN, STEP_Y_PIN };

Stepper xStepper(xStepperPins, GEAR_RATIO_X, 0, STEPPER_TYPE, MICROSECONDS_PER_STEP_X, STEPS_PER_REVOLUTION_NEMA17);
Stepper yStepper(yStepperPins, GEAR_RATIO_Y, 1, STEPPER_TYPE, MICROSECONDS_PER_STEP_Y, STEPS_PER_REVOLUTION_NEMA17);
DateTime programTime;
SolarPosition sunPosition;
SolarPosition trackerPosition;
GeographicalCoordinate coords;
ISR_Flags flags;

void setup(){
  Serial.begin(9600);    
  
  programTime = DateTime(F(__DATE__), F(__TIME__));

  coords.Latitude = 52.51;
  coords.Longitude = 13.41;

  Serial.print(programTime.year(), DEC);
  Serial.print('/');
  Serial.print(programTime.month(), DEC);
  Serial.print('/');
  Serial.print(programTime.day(), DEC);
  Serial.print(" (");
  Serial.print(programTime.dayOfTheWeek());
  Serial.print(") ");
  Serial.print(programTime.hour(), DEC);
  Serial.print(':');
  Serial.print(programTime.minute(), DEC);
  Serial.print(':');
  Serial.print(programTime.second(), DEC);
  Serial.println();
}


void loop(){

    // Assume tracker in start position
    trackerPosition.Azimuth = 0;
    trackerPosition.Zenith = 0;
    sunPosition = SolarCalculator::getSolarPosition(programTime, coords);

    // azimuth  angle-difference between sun and tracker
    double dAzimuth = sunPosition.Azimuth - trackerPosition.Azimuth;
    
    // rotate x-Stepper
    xStepper.prepareMovement(dAzimuth, &flags);
    Serial.print("Delta-Azimuth: ");
    Serial.print(dAzimuth);
    Serial.print(" -> ");
    Serial.print(xStepper.getTotalStepsRequested());
    Serial.print(" Steps => ");
    Serial.print((double) xStepper.getTotalStepsRequested()/STEPS_PER_REVOLUTION_NEMA17); 
    Serial.println(" Revolutions");
    // zenith  angle-difference between sun and tracker

    double zenithOld = trackerPosition.Zenith;
    double zenithNew = sunPosition.Zenith;
    //Serial.println(zenithNew);
    double dZenith = 60;
    // translate zenith difference to motor-rotations
    double dZenithMotor = SolarCalculator::elevationAngleChange2MotorAngle(zenithOld, zenithNew);
    yStepper.prepareMovement(dZenithMotor, &flags);
    
    Serial.print("Delta Zenith: ");
    Serial.print(dZenithMotor);
    Serial.print(" -> ");
    Serial.print(yStepper.getTotalStepsRequested());
    Serial.print(" Steps => ");
    Serial.print((double)yStepper.getTotalStepsRequested()/STEPS_PER_REVOLUTION_NEMA17); 
    Serial.println(" Revolutions");
    while(true);
    // Update tracker-position
    /*trackerPosition.Azimuth += dAzimuth;
    trackerPosition.Zenith += dZenith;
    isInStartPosition = false;
    lcdPrintSolarPosition(lcd, trackerPosition, 1);*/
  
  
}
