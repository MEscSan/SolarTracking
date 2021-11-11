// Main programm for 2-axis Arduino
#include "Stepper.h"            //  Stepper Motors
#include "SolarCalculator.h"    //  Static class for getting the position of the sun
#include <LiquidCrystal_I2C.h>  //  LCD-Headers
#include <SoftwareSerial.h>     //  Serielle Kommunikation über digitale Pins 
#include "DEV_Config.h"         //  GPS-Headers (I)
#include "L76X.h"               //  GPS-Headers (II)
#include "RTClib.h"             //  RTC-Clock
#include "ICR_Functions.h"      //  Complementary function-set for Timer Interrupts
#include "Display_Functions.h"  //  Complementary function-set for LCD-Display
#include "miniIMU.h"            //  9-DOF IMU

#define DIR_X_PIN 5     // Nema17 X-Direction
#define STEP_X_PIN 2    // Nema17 X-Step
#define DIR_Y_PIN 6     // Nema 17 Y-Direction
#define STEP_Y_PIN 3    // Nema 17 Y-Step
#define LED_PIN 13
#define MICROSECONDS_PER_STEP 7000
#define MICROSECONDS_PER_STEP2 7000
#define GEAR_RATIO 5 // Input speed / Output speed : Ratio > 1 => gear slows movement down 
#define GEAR_RATIO_X 5 
#define GEAR_RATIO_Y 5 
#define NUM_STEPPERS 2
#define STEPS_PER_REVOLUTION_NEMA17 200
#define STEPS_PER_REVOLUTION_28BYJ 2048
#define STEPPER_TYPE StepperType::NEMA17 
#define TIMER1_PRESCALER 64
#define TIMER1_INTERVAL_US 1000 //Timer interval in microseconds


/******************************************************************************
Global variables
******************************************************************************/
//For Nema17
int xStepperPins[2] = { DIR_X_PIN, STEP_X_PIN };
int yStepperPins[2] = { DIR_Y_PIN, STEP_Y_PIN };
/*
//For 28BYJ:
int xStepperPins[4] = { 11, 10, 9, 8 }; // Motor X: 11...8  
int yStepperPins[4] = {  7,  6, 5, 4 }; // Motor Y: 7...4
*/
bool  ledState = true;  //LED-status variable

//****Struct Instances****
ISR_Flags flags;

//****Object Instances****
LiquidCrystal_I2C lcd(0x27, 16, 2); //Instantiate and initialize LCD-Display, set the I2C Address to 0x27 for the LCD (16 chars and 2 line Display)
RTC_DS1307 rtc;
GNRMC gps;
MiniIMU imu;//, DeviceVersion::V4,AxisDefinition::Y_right_Z_Down, true);
Stepper xStepper(xStepperPins, GEAR_RATIO_X, 0, STEPPER_TYPE, MICROSECONDS_PER_STEP, STEPS_PER_REVOLUTION_NEMA17);
Stepper yStepper(yStepperPins, GEAR_RATIO_Y, 1, STEPPER_TYPE, MICROSECONDS_PER_STEP2, STEPS_PER_REVOLUTION_NEMA17);

//****Volatile variables (shared by interrupt and normal code)****
volatile Stepper steppers[NUM_STEPPERS] = {xStepper, yStepper};


#pragma region ICR

//Run the timer routine concurrently to the rest of the tasks included in the function
void runAndWait() {
    timer1CompA_SetNextInterruptInterval(&flags, steppers, NUM_STEPPERS, TIMER1_PRESCALER);
    timer1CompA_On();

    while (flags.remainingSteppersFlag) {
        ledState = !ledState;
        //digitalWrite(LED_PIN, ledState);
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

    timer1CompA_SetNextInterruptInterval(&flags, steppers, NUM_STEPPERS, TIMER1_PRESCALER);

    TCNT1 = 0;
}
#pragma endregion


void setup() {

    // Set Timer1
    timer1CompA_Init(TIMER1_PRESCALER,TIMER1_INTERVAL_US);
    /*noInterrupts();
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
    interrupts();*/

    // Set serial monitor
    Serial.begin(9600);

    // Set LCD-Display 
    lcd.init();       // initialize lcd
    lcd.backlight();  // turn on backlight
    lcd.setCursor(0, 0);

    // Set RTC-Clock
    rtc.begin();
    if (! rtc.isrunning()) {
      Serial.println("RTC is NOT running, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }


    // Set LED-Pin
    pinMode(LED_PIN, OUTPUT);
    
    // Ste GPS-Device
    L76X_Init_9600();
}

void loop() {
     
  DateTime now = rtc.now();
  gps = L76X_Gat_GNRMC();
  Serial.print("\r\n");
  Serial.print("Time:");
  Serial.print(gps.Time_H);
  Serial.print(":");
  Serial.print(gps.Time_M); 
  Serial.print(":");
  Serial.println(gps.Time_S);
  Serial.print("Status: ");
  Serial.println(gps.Status);
  Serial.print("Latitude: ");
  Serial.print(gps.Lat);
  Serial.print("\t Longitude: ");
  Serial.println(gps.Lon);
  Serial.print("Latitude area: ");
  Serial.print(gps.Lat_area);
  Serial.print("\t Longitude area: ");
  Serial.print(gps.Lon_area);
  lcdPrintGPS(lcd, gps);
  
  GeographicalCoordinate g;
  g.Latitude = 52.51;//gps.Lat;
  g.Longitude = 13.41;//gps.Lon;
  SolarPosition s =  SolarCalculator::getSolarPosition(now, g);
  lcdPrintSolarPosition(lcd,s);
  
  
  steppers[0].prepareMovement(45, &flags);
  steppers[1].prepareMovement(45, &flags);
  runAndWait();
  
   //while(true);
}
