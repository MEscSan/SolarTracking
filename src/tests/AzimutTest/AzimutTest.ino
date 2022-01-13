// Test-program for mirror inclination(elevation angle)
// It changes the mirror inclination from 0° to 90° and back in steps of 10 degrees

#include "Stepper.h"            //  Stepper Motors
#include "SolarCalculator.h"    //  Static class for getting the position of the sun
#include "ICR_Functions.h"
#include <LiquidCrystal_I2C.h>

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

#pragma region Global variables
LiquidCrystal_I2C lcd(0x27, 16, 2); //Instantiate and initialize LCD-Display, set the I2C Address to 0x27 for the LCD (16 chars and 2 line Display)
//For Nema17
int xStepperPins[2] = { DIR_X_PIN, STEP_X_PIN };
int yStepperPins[2] = { DIR_Y_PIN, STEP_Y_PIN };
int counter = 0;

double azimuthNew = 0;
double azimuthOld = 0;
double elevationNew = 0;
double elevationOld = 0;

unsigned long programMillis = 0;
unsigned long programMicros = 0; 
unsigned long azimuthSteps = 0;
unsigned long elevationSteps = 0;


SolarPosition trackerPosition;

//****Struct Instances****
ISR_Flags flags;

//****Object Instances****
// Hardware
Stepper xStepper(xStepperPins, GEAR_RATIO_X, 0, STEPPER_TYPE, MICROSECONDS_PER_STEP_X, STEPS_PER_REVOLUTION_NEMA17);
Stepper yStepper(yStepperPins, GEAR_RATIO_Y, 1, STEPPER_TYPE, MICROSECONDS_PER_STEP_Y, STEPS_PER_REVOLUTION_NEMA17);

//****Volatile variables (shared by interrupt and normal code)****
volatile Stepper steppers[NUM_STEPPERS] = {xStepper, yStepper};
#pragma endregion

ISR(TIMER1_COMPA_vect) {
    unsigned int tmpCtr = OCR1A;
    OCR1A = 65500;
    for (int i = 0; i < NUM_STEPPERS; i++) {

        // Jump the for-loop for all motors that are not remaining
        // 00001111 & 01000000 => 00000000 => !0 == 1 => true
        if (!((1 << i) & flags.remainingSteppersFlag))
            continue;

        // Jump the for-loop for all motors that are not the one in the nextStepperFlag
        // 00000010 & 00000010 => 00000010 => !2 = 0 => false
        // 00000010 & 00000100 => 00000000 => !0 = 1 => true
        if (!(flags.nextStepperFlag & (1 << i))) {
            continue;
        }

        // Get remaining motor
        //Serial.println(i);
        volatile Stepper& s = steppers[i];
        // Serial.print("Requested Steps");
        //Serial.println(s.getTotalStepsRequested());
        volatile unsigned long stepsRequested = s.getTotalStepsRequested();
        
        // Run one step in the motor
        if (s.getStepCountInMovement() < stepsRequested) {
            s.oneStep();
            // Update step-counter for current movement
            unsigned long newStepCount = s.getStepCountInMovement();
            newStepCount++;
            s.setStepCountInMovement(newStepCount);
            //Serial.println(s.getStepCountInMovement());
            
            // Update total step-counter
            //Hier ist das Problem
            ////////////////////
             // Update total step-counter
            long newStepPosition = s.getStepPosition();
            newStepPosition += s.getDirection() == 0 ? 1 : -1;
            s.setStepPosition(newStepPosition);
            
            if (s.getStepCountInMovement() >= stepsRequested) {
                s.setMovementDone(true);
                flags.remainingSteppersFlag &= ~(1 << i);
            }
        }
    }

    timer1CompA_SetNextInterruptInterval(&flags, steppers, NUM_STEPPERS, TIMER1_PRESCALER);

    TCNT1 = 0;
}

//Run the timer routine concurrently to the rest of the tasks included in the function
void runAndWait() {
    timer1CompA_SetNextInterruptInterval(&flags, steppers, NUM_STEPPERS, TIMER1_PRESCALER);
    timer1CompA_On();

    while (flags.remainingSteppersFlag) {
       lcd.setCursor(9,1);
       lcd.print("Running");
    }

    flags.remainingSteppersFlag = 0;
    flags.nextStepperFlag = 0;
}


/******************************************************************************
Arduino control
******************************************************************************/
void setup() {

    // Set Timer1
    timer1CompA_Init(TIMER1_PRESCALER,TIMER1_INTERVAL_US);
    timer1CompA_Off();
    
    // Set serial monitor
    Serial.begin(9600);
    Serial.print("Steps init: ");
    Serial.println( steppers[1].getStepPosition());

    // Set LCD-Display 
    lcd.init();       // initialize lcd
    lcd.backlight();  // turn on backlight
    lcd.setCursor(0, 0);
    
    trackerPosition.Azimuth = 0;
    trackerPosition.ElevationAngle = 0;
}

void loop() {
   /*for(int i = 0; i<60; i+=3){
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
    Serial.print(steppers[1].getTotalStepsRequested());
    elevationSteps += steppers[1].getTotalStepsRequested();
    Serial.print(" \t");
    
    Serial.print("\n");
   }*/
   
   azimuthOld = trackerPosition.Azimuth;
   elevationOld = trackerPosition.ElevationAngle;
   if(counter < 9){
    azimuthNew += 10;  
    elevationNew += 10; 
   }
   else
   if(counter < 18 && counter>0){
    
    azimuthNew -= 10;
    elevationNew -= 10;
   }
   else{
    Serial.print("That was it!      "); 
    while(true);
   }


   double motorAngle = SolarCalculator::elevationAngleChange2MotorAngle(elevationOld, elevationNew);
   steppers[1].prepareMovement(-motorAngle, &flags);
   
   double dAzimuth = azimuthNew-azimuthOld;
   
   steppers[0].prepareMovement(dAzimuth, &flags);
   
   runAndWait();

   Serial.print("\nAzimuth: ");
   Serial.print( steppers[0].getStepPosition());
   Serial.print("  ");
   Serial.println( steppers[0].getAngle());
   Serial.print("Elevation: ");
   Serial.print( steppers[1].getStepPosition());
   Serial.print("  ");
   Serial.println( steppers[1].getAngle());

   trackerPosition.Azimuth =  azimuthNew; 
   trackerPosition.ElevationAngle = elevationNew; 
   counter++;
   
}
