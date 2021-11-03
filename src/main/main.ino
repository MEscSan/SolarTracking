// Main programm for 2-axis Arduino
#include "Stepper.h"            //  Stepper Motors
#include "SolarCalculator.h"    //  Static class for getting the position of the sun
#include <LiquidCrystal_I2C.h>  //  LCD-Headers
#include <SoftwareSerial.h>     //  Serielle Kommunikation über digitale Pins 
#include "DEV_Config.h"         //  GPS-Headers (I)
#include "L76X.h"               //  GPS-Headers (II)
#include "RTClib.h"             //  RTC-Clock
#include "ICR_Functions.h"      //  Complementary function-set for Timer Interrupts
#include "Display_Functions.h"  //  Complementary function-set for 


#define DIR_X_PIN 5     // Nema17 X-Direction
#define STEP_X_PIN 2    // Nema17 X-Step
#define DIR_Y_PIN 6     // Nema 17 Y-Direction
#define STEP_Y_PIN 3    // Nema 17 Y-Step
#define LED_PIN 13
#define CLICKS_PER_STEP 1500
#define GEAR_RATIO 1 // Input speed / Output speed : Ratio > 1 => gear slows movement down 
#define GEAR_RATIO_X 1 
#define GEAR_RATIO_Y 1 
#define NUM_STEPPERS 2
#define STEPS_PER_REVOLUTION_NEMA17 200
#define STEPS_PER_REVOLUTION_28BYJ 2048
#define STEPPER_TYPE StepperType::BYJ28 
#define TIMER1_OCR 550


//****Global variables****
/*/For Nema17
int xStepperPins[2] = { DIR_X_PIN, STEP_X_PIN };
int yStepperPins[2] = { DIR_Y_PIN, STEP_Y_PIN };
*/
//For 28BYJ:
int xStepperPins[4] = { 11, 10, 9, 8 }; // Motor X: 11...8  
int yStepperPins[4] = {  7,  6, 5, 4 }; // Motor Y: 7...4
bool  ledState = true;  //LED-status variable
//****Struct Instances****
ISR_Flags flags;

//****Object Instances****
LiquidCrystal_I2C lcd(0x27, 16, 2); //Instantiate and initialize LCD-Display, set the I2C Address to 0x27 for the LCD (16 chars and 2 line Display)
RTC_DS1307 rtc;
Stepper xStepper(xStepperPins, GEAR_RATIO_X, 0, STEPPER_TYPE, CLICKS_PER_STEP, STEPS_PER_REVOLUTION_28BYJ);
Stepper yStepper(yStepperPins, GEAR_RATIO_Y, 1, STEPPER_TYPE, CLICKS_PER_STEP, STEPS_PER_REVOLUTION_28BYJ);

//****Volatile variables (shared by interrupt and normal code)****
volatile Stepper steppers[NUM_STEPPERS] = {xStepper, yStepper};


#pragma region ICR

// Run the timer routine concurrently to the rest of the tasks included in the function
void runAndWait() {
    timer1CompA_setNextInterruptInterval(&flags, steppers, NUM_STEPPERS);
    timer1CompA_On();

    while (flags.remainingSteppersFlag) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        //delay(500);
        lcdPrintTime(lcd, rtc);
    }

    flags.remainingSteppersFlag = 0;
    flags.nextStepperFlag = 0;
}

//****Interrupt routine Timer 1****
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
        volatile unsigned int stepsRequested = s.getTotalStepsRequested();
        
        // Run one step in the motor
        if (s.getStepCountInMovement() < stepsRequested) {
            s.oneStep();
            // Update step-counter for current movement
            unsigned int newStepCount = s.getStepCountInMovement();
            newStepCount++;
            s.setStepCountInMovement(newStepCount);

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

    timer1CompA_setNextInterruptInterval(&flags, steppers, NUM_STEPPERS);

    TCNT1 = 0;
}
#pragma endregion

#pragma region Display-Functions

#pragma endregion
#pragma endregion
void setup() {

    // Set Timer1
    noInterrupts();
    //1. Reset Timer/Counter Control Register A and B (TCCR1A, TCCR1B)
    TCCR1A = 0;
    TCCR1B = 0;
    //2. Reset Timer/Counter Register
    TCNT1 = 0;
    //3. Set PWM-Mode to CTC
    TCCR1B |= (1 << WGM12);
    //4. Set prescaler value in Clock-Select bits of TCCR1B-Register (CS12, CS11, CS10)
    TCCR1B |= B00000011;    //Prescaler 64
    //5. Set timer Mode in Timer Interrupt Mask Register 1 (TIMSK1)
    TIMSK1 |= (1 << OCIE1A);
    //6. Set number of Timer pulses till interrupt
    OCR1A = TIMER1_OCR;
    interrupts();

    // Set serial monitor
    Serial.begin(9600);

    // Set LCD-Display 
    lcd.init();       // initialize lcd
    lcd.backlight();  // turn on backlight// Set LCD-Display
    lcd.init();       // initialize lcd
    lcd.backlight();  // turn on backlight
    lcd.setCursor(0, 0);

    // Set RTC-Clock
    rtc.begin();

    // Set LED-Pin
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
     
  DateTime now = rtc.now();
  GeographicalCoordinate g;
  g.Latitude = 0;//52.51;
  g.Longitude = 0;//13.41;
  SolarPosition s =  SolarCalculator::getSolarPosition(now, g);
  lcdPrintSolarPosition(lcd,s);
  
  steppers[0].prepareMovement(-35, &flags);
  runAndWait();

  steppers[1].prepareMovement(-35, &flags);
  runAndWait();
  
   //while(true);
}
