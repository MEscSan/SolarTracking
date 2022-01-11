// Main programm for 2-axis Arduino
#include "Stepper.h"            //  Stepper Motors
#include "SolarCalculator.h"    //  Static class for getting the position of the sun

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
#define GEAR_RATIO 5 // Input speed / Output speed : Ratio > 1 => gear slows movement down 
#define GEAR_RATIO_X 960 // =19,2*50
#define GEAR_RATIO_Y 19.2 
#define GEAR_RATIO_WORM_DRIVE 50
#define NUM_STEPPERS 2
#define STEPS_PER_REVOLUTION_NEMA17 200
#define STEPS_PER_REVOLUTION_28BYJ 2048
#define STEPPER_TYPE StepperType::NEMA17 

// Macros for ICR-Interrup
#define TIMER1_PRESCALER 64
#define TIMER1_INTERVAL_US 1000 //Timer interval in microseconds

// Macros for millis() ant timeouts
#define STATE0_TIMEOUT 30000 // 30s= 30000 ms
#define GPS_TIMEOUT 30000  // 90s= 90000 ms
#define TRACKER_DELAY 1 // 1 min
#define MILLIS_30_SECONDS 30000 // 30s= 30000 ms
#define MILLIS_20_MINUTES 1200000 // 20min=1200000 ms

// Macros for IR-Remote control (check for your model whether the codes correspond to the buttons!)
#define RECV_PIN 22
#define IR_REMOTE_PREV 0xFFE01F
#define IR_REMOTE_NEXT 0xFFA857
#define IR_REMOTE_PLAY 0xFF906F
#define IR_REMOTE_0 0xFF6897
#define IR_REMOTE_1 0xFF30CF
#define IR_REMOTE_2 0xFF18E7
#define IR_REMOTE_3 0xFF7A85
#define IR_REMOTE_4 0xFF10EF
#define IR_REMOTE_5 0xFF38C7
#define IR_REMOTE_6 0xFF5AA5
#define IR_REMOTE_7 0xFF42BD
#define IR_REMOTE_8 0xFF4AB5
#define IR_REMOTE_9 0xFF52AD
#define IR_BUTTON_STILL_PRESSED 0xFFFFFFFF

// Macros for End-Point-Button
#define ENDPOINT_PIN 26

#pragma region Global variables
//For Nema17
int dirX = -1;
volatile int timerCount = 0;
int dirY = -1;
int xStepperPins[2] = { DIR_X_PIN, STEP_X_PIN };
int yStepperPins[2] = { DIR_Y_PIN, STEP_Y_PIN };
bool ledState = true;  //LED-status variable
bool isInStartPosition = false;
bool isLeveled = false;
//bool isRightTime = false;
bool isReadyForLeveling = false;
bool isNotOriented = true;
double sunElevationNew = 0;
double sunElevationOld = 0;
unsigned long programMillis = 0;
unsigned long programMicros = 0; 
unsigned long azimuthSteps = 0;
unsigned long elevationSteps = 0;
DateTime programTime;
SolarPosition sunPosition;
SolarPosition trackerPosition;
GeographicalCoordinate coords;


//****Struct Instances****
ISR_Flags flags;

//****Object Instances****
// Hardware
Stepper xStepper(xStepperPins, GEAR_RATIO_X, 0, STEPPER_TYPE, MICROSECONDS_PER_STEP_X, STEPS_PER_REVOLUTION_NEMA17);
Stepper yStepper(yStepperPins, GEAR_RATIO_Y, 1, STEPPER_TYPE, MICROSECONDS_PER_STEP_Y, STEPS_PER_REVOLUTION_NEMA17);



//****Volatile variables (shared by interrupt and normal code)****
volatile Stepper steppers[NUM_STEPPERS] = {xStepper, yStepper};
#pragma endregion


#pragma region Arduino
/******************************************************************************
Arduino control
******************************************************************************/
void setup() {
   
    // Set serial monitor
    Serial.begin(9600);
    trackerPosition.Azimuth = 0;
    trackerPosition.ElevationAngle = 0;
}

void loop() {
   for(int i = 0; i<60; i+=3){
    sunElevationOld = sunElevationNew;
    sunElevationNew = i; 
    
    double dElevationAngle = sunElevationNew - sunElevationOld;
  
    // translate elevation angle difference to motor-rotations
    double dElevationAngleMotor = SolarCalculator::elevationAngleChange2MotorAngle(sunElevationOld, sunElevationNew);
    //Serial.print("\nElevation\t");

    Serial.print(sunElevationNew);
    //Serial.print("\t-> dElevationAngle (Motor):\t");
    Serial.print("\t");
    Serial.print(dElevationAngleMotor);
    Serial.print("\t");
    Serial.print(abs(dElevationAngleMotor));

    // rotate y-Stepper
    steppers[1].prepareMovement(-dElevationAngleMotor, &flags);
    Serial.print(" \t");
    Serial.println(steppers[1].getTotalStepsRequested());
    elevationSteps += steppers[1].getTotalStepsRequested();
   }
   while(true);
}
#pragma endregion
