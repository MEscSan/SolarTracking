// Main programm for 2-axis Arduino
#include <LiquidCrystal_I2C.h>  //  LCD-Headers
#include <SoftwareSerial.h>     //  Serielle Kommunikation über digitale Pins 
#include <StateMachine.h>
#include <IRremote.h>
//#include <EEPROM.h>

#include "Stepper.h"            //  Stepper Motors
#include "SolarCalculator.h"    //  Static class for getting the position of the sun
#include "DEV_Config.h"         //  GPS-Headers (I)
#include "L76X.h"               //  GPS-Headers (II)
#include "RTClib.h"             //  RTC-Clock
#include "ICR_Functions.h"      //  Complementary function-set for Timer Interrupts
#include "Display_Functions.h"  //  Complementary function-set for LCD-Display
#include "miniIMU.h"            //  9-DOF IMU

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
/******************************************************************************
Global variables
******************************************************************************/
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
LiquidCrystal_I2C lcd(0x27, 16, 2); //Instantiate and initialize LCD-Display, set the I2C Address to 0x27 for the LCD (16 chars and 2 line Display)
RTC_DS3231 rtc;
GNRMC gps;
MiniIMU imu;//, DeviceVersion::V4,AxisDefinition::Y_right_Z_Down, true);
Stepper xStepper(xStepperPins, GEAR_RATIO_X, 0, STEPPER_TYPE, MICROSECONDS_PER_STEP_X, STEPS_PER_REVOLUTION_NEMA17);
Stepper yStepper(yStepperPins, GEAR_RATIO_Y, 1, STEPPER_TYPE, MICROSECONDS_PER_STEP_Y, STEPS_PER_REVOLUTION_NEMA17);
IRrecv irrecv(RECV_PIN); // Create a class object used to receive IR-remote control commands
decode_results results; // Create a decoding results class object for IR-Remote
StateMachine machine = StateMachine();


//****Volatile variables (shared by interrupt and normal code)****
volatile Stepper steppers[NUM_STEPPERS] = {xStepper, yStepper};
#pragma endregion

#pragma region ICR
/******************************************************************************
Interrupt Routine Timer 1
******************************************************************************/

//Run the timer routine concurrently to the rest of the tasks included in the function
void runAndWait() {
    timer1CompA_SetNextInterruptInterval(&flags, steppers, NUM_STEPPERS, TIMER1_PRESCALER);
    timer1CompA_On();

    while (flags.remainingSteppersFlag) {
        ledState = !ledState;
    }
    
    flags.remainingSteppersFlag = 0;
    flags.nextStepperFlag = 0;
}

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
            //unsigned long newStepPosition = s.getStepPosition();
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

ISR(TIMER1_COMPB_vect){   
      /*  
      switch (results.value) {
        case IR_REMOTE_2:    
            dirY = 0;   // Y+
            Serial.println("S2: Y+  ");
            break;
        case IR_REMOTE_8:  
            dirY = 1;  // Y-
            Serial.println("S2: Y-  ");
            break;
        case IR_REMOTE_4:  
            dirX = 0;  // X+
            Serial.println("S2: X+  ");
            break;
        case IR_REMOTE_6:
            dirX = 1;    // X-
            Serial.println("S2: X-  ");
            break;
        case IR_REMOTE_5: 
            dirX = -1;   // Stop both motors
            dirY = -1;
            Serial.println("S2: Stop  ");
            break;
        case IR_REMOTE_1:
             isNotOriented = false;
             Serial.println("S2: North found!");
             break;
        default: // Any other button pressed (except for 0, which changes State)
            // Do nothing
            break;
        }
      */
      //if(dirX >= 0){
      steppers[0].oneStep(1);
      //}
      //if(diY >)
     if(timerCount<4000){
      timerCount++;
      }
     else{
      timer1CompB_Off();
      }
     OCR1B = microseconds2Clicks(TIMER1_PRESCALER,900);
     Serial.println(micros());
     TCNT1  = 0;
}

#pragma endregion

#pragma region State machine
/******************************************************************************
State machine
******************************************************************************/
// States
// State 0: turn on
void state0_TurnOn(){
    
    if(machine.executeOnce) {
        programTime = rtc.now();
        programMillis = millis();

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Einschalten");
        lcdPrintTime(lcd, rtc,1);
    }

    // Receive IR-remote signal
    if (irrecv.decode(&results)) { // Waiting for decoding
       irrecv.resume(); // Receive the next value
    }

     switch (results.value) {
      case IR_REMOTE_0:    
          isReadyForLeveling = true;
          break;
      case IR_REMOTE_3:
           isInStartPosition = true;
           break;
      default: // Any other button pressed (except for 0, which changes State)
          // Do nothing
          break;
    }


    // Change to state  after 30s
    if((millis()-programMillis) > 30000){
      programMillis = millis();
      isReadyForLeveling = true;  
    }
}
// State 1: initalisation, adjust tripod
void state1_InitLeveling() {

    lcd.setCursor(0, 0);
    lcd.print("Nivelieren");
    lcdPrintTime(lcd, rtc, 1);
};
// State 2: initalisation, set mirror/solar-panel to the north
void state2_InitNorth() {

    //Stop GPS to avoid interference with motor communication
    L76X_Stop(); 
    
    if(machine.executeOnce){
      isNotOriented = true;
      programMillis = millis();
      lcd.setCursor(0, 0);
      lcd.print("Nach Norden");
      dirX = -1;
      dirY = -1;
      
    }
    
      //timer1CompB_On();
    // Turn the mirror into vertical position
    mirror2VerticalPosition();

    while(isNotOriented){
      if (irrecv.decode(&results)) { // Waiting for decoding

        // Print out the decoded results
         switch (results.value) {
          case IR_REMOTE_2:    
              dirY = 1;   // Y+
              lcd.clear();
              lcd.print("Y+  ");
              break;
          case IR_REMOTE_8:  
              dirY = 0;  // Y-
              lcd.clear();
              lcd.print("Y-  ");
              break;
          case IR_REMOTE_4:  
              dirX = 0;  // X+
              lcd.clear();
              lcd.print("X+  ");
              break;
          case IR_REMOTE_6:
              dirX = 1;    // X-
              lcd.clear();
              lcd.print("X-  ");
              break;
          case IR_REMOTE_5: 
              dirX = -1;   // Stop motor
              dirY = -1;
              lcd.clear();
              lcd.print("Stopp  ");
              break;
          case IR_REMOTE_1:
               isNotOriented = false;
               /*if(isVertical){
                isNotOriented = false;
                lcd.clear();
                lcd.print("North found!"); 
               }
               else{
                   lcd.clear();
                   lcd.print("y-Endpunkt noch");
                   lcd.setCursor(0,1);
                   lcd.print("nicht erreicht!");
               }*/
               
               break;
          default: // Any other button pressed (except for 0, which changes State)
              // Do nothing
              break;
         }
         if(dirX >= 0){
          steppers[0].setDirection(dirX);
         }
         if(dirY >= 0){
           steppers[1].setDirection(dirY);
         }
         irrecv.resume(); // Receive the next value
      }
                
      if(dirX >= 0){
          steppers[0].oneStep();
        }
       if(dirY >= 0){
          steppers[1].oneStep();
        }

      delayMicroseconds(MICROSECONDS_PER_STEP);
    }

    //Serial.println(micros() - programMicros);
    //programMicros = micros();
};
// State 3: track the sun (tracker updates its angles every 20 minutes)
void state3_SolarTrack() {

    bool isFirstRun = false;
    if(machine.executeOnce) {
        // Set GPS-Device
        L76X_Start(9600);
        L76X_Init_9600();
        
        // Get coordinates from GPS
        int gpsTimeout = 1;// minute
        programMillis = millis();
        isFirstRun = true;
        azimuthSteps = 0;
        elevationSteps = 0;
        
        lcd.setCursor(0, 0);
        lcd.print("GPS suchen");
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
        Serial.println(gps.Lon_area);

        // Wait until GPS-Sensor delivers realistic values or timeout is over
        while(gps.Status == 0 || gps.Lat == 0.00 || gps.Lon == 0.00) {
            gps = L76X_Gat_GNRMC();

            lcdPrintGPS(lcd, gps, 1);

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
            Serial.println(gps.Lon_area);
            
            coords.Latitude = gps.Lat;
            coords.Longitude = gps.Lon;

            if ((millis() - programMillis) > GPS_TIMEOUT){
            //if ((millis() - programMillis) > 0){
            
                // Assume we're in Karl-Marx-City
                coords.Latitude = 52.51;
                coords.Longitude = 13.41;

                lcd.setCursor(0, 0);
                lcd.print("GPS Fehler       ");
                lcd.setCursor(0, 1);
                lcd.print("Default:Chemnitz");

                // Wait for 5s
                delay(5000);
                break;
            }
        }
        if(gps.Status!= 0){
          
          lcdPrintGPS(lcd, gps, 0);
          delay(5000);
        }
        
        // Assume tracker in start position
        trackerPosition.Azimuth = 0;
        trackerPosition.ElevationAngle = 0;

        isInStartPosition = true; 
        
        //Stop GPS to avoid interference with motor communication
        L76X_Stop(); 
    }

    DateTime now = rtc.now();
    
    sunPosition = SolarCalculator::getSolarPosition(programTime, coords);
    
    // Update Display only once a second
    if((programMillis - millis()) > 1000 || isFirstRun){
      
      lcdPrintSolarPosition(lcd, sunPosition, 0);
      //lcdPrintCoords(lcd, coords, 1);
      programMillis = millis();
    }

    // Move motors only if sun is already risen/no yet set
    if(sunPosition.ElevationAngle > 0){   
      // Move motors only when the sun has moved 3° in either azimut or elevation angle or if 
      // the state is running for the first time
            // Elevation Angle  angle-difference between sun and tracker
      double elevationAngleOld = trackerPosition.ElevationAngle;
      double elevationAngleNew = sunPosition.ElevationAngle;
      double dElevationAngle = elevationAngleNew - elevationAngleOld;
  
      // translate elevation angle difference to motor-rotations
      double dElevationAngleMotor = SolarCalculator::elevationAngleChange2MotorAngle(elevationAngleOld, elevationAngleNew);
      Serial.print("\nElevation ");
      Serial.print(sunPosition.ElevationAngle);
      Serial.print(" -> dElevationAngle (Motor) : ");
      Serial.print(dElevationAngleMotor);
  
      if(dElevationAngle > 1 || isFirstRun){
        // rotate y-Stepper
        steppers[1].prepareMovement(-dElevationAngleMotor, &flags);
        Serial.print(" -> Steps: ");
        Serial.print(steppers[1].getTotalStepsRequested());
        elevationSteps += steppers[1].getTotalStepsRequested();
      
        Serial.print("\nMotor Y");
        lcd.setCursor(0,1);
        lcd.print("Hoehenwinkel neu");
        runAndWait();
  
        trackerPosition.ElevationAngle += dElevationAngle;

        //Update display
        Serial.print("\nStep-position y: ");
        Serial.print(steppers[1].getStepPosition());

        //isVertical= false;
      }
     
      // azimuth  angle-difference between sun and tracker
      double dAzimuth = sunPosition.Azimuth - trackerPosition.Azimuth;
      Serial.print("\nAzimuth ");
      Serial.print(sunPosition.Azimuth);
      Serial.print(" -> dAzimuth : ");
      Serial.print(dAzimuth);

      if(dAzimuth > 3|| isFirstRun){
        // rotate x-Stepper
        steppers[0].prepareMovement(dAzimuth, &flags);
        Serial.print("-> Steps: ");
        Serial.println(steppers[0].getTotalStepsRequested());
        azimuthSteps += steppers[0].getTotalStepsRequested();
            
        Serial.println("Motor X");
        lcd.setCursor(0,1);
        lcd.print("Azimut neu      ");
        runAndWait();
  
        Serial.print("\nStep-position x: ");
        Serial.println(steppers[0].getStepPosition());
      
        // Update tracker-position
        //trackerPosition.Azimuth += dAzimuth;
        trackerPosition.Azimuth += steppers[0].getAngle();

        lcd.setCursor(0,1);
        lcd.print("A:");
        lcd.print(steppers[0].getAngle());
        Serial.print("Azimut");
        Serial.print(steppers[0].getAngle());
        lcd.print(" E:");
        lcd.print(trackerPosition.ElevationAngle);
        lcd.print("         ");
      }
    
      isInStartPosition = false;
    }
    programTime = rtc.now();
};
// State 4: move the tracker back to start position (Azimuth 0, elevation angle 0)
void state4_Return2Start() {

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Zurueckfahren...");

    // Move motors only if mirror position was already changed
    if(azimuthSteps > 0){
        
      // Turn motors back to start-position (Azimuth 0, elevation angle 0)
      // azimuth  angle-difference between sun and tracker
      //double dAzimuth = 0 - trackerPosition.Azimuth;
      // rotate x-Stepper
      steppers[0].prepareMovementSteps(azimuthSteps, -1, &flags);
      
      Serial.println("Motor X");
      lcd.setCursor(0,1);
      lcd.print("Azimut neu      ");
      runAndWait();
    }

    if(elevationSteps >0){
      // elevation angle-difference between sun and tracker
      //double dElevationAngle = 0 - trackerPosition.ElevationAngle;
      // rotate y-Stepper
      Serial.println(elevationSteps);
      //steppers[1].prepareMovementSteps(elevationSteps, -1, &flags);*/
  
      // Set mirror back to vertical position
      mirror2VerticalPosition();
    }
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Zurueckfahren...");
    //runAndWait();

    lcd.setCursor(0,1);
    lcd.print("A:");
    lcd.print(steppers[0].getAngle(),2);
    lcd.print(" E:");
    lcd.print(steppers[1].getAngle(),2);
    lcd.print("         ");
    
    // Update tracker-position and reset step-count variables
    trackerPosition.Azimuth = 0;
    trackerPosition.ElevationAngle = 0;
    azimuthSteps = 0;
    elevationSteps = 0;
    
    isInStartPosition = true;
};

//  Turn the mirror into vertical position (elevation angle 0)
//  Returns true when the end-point button is pressed
void mirror2VerticalPosition() {

    // Set Y-Motor direction to -Y
    steppers[1].setDirection(0);
    
    lcd.setCursor(0, 1);
    lcd.print("Nicht vertikal");

    while (digitalRead(ENDPOINT_PIN) == 1) {
        steppers[1].oneStep();
        delayMicroseconds(900);
    }

    lcd.setCursor(0, 1);
    lcd.print("Spiegel vertikal!    ");
}

// Pass states to the machine:
State* S0 = machine.addState(&state0_TurnOn);
State* S1 = machine.addState(&state1_InitLeveling);
State* S2 = machine.addState(&state2_InitNorth);
State* S3 = machine.addState(&state3_SolarTrack);
State* S4 = machine.addState(&state4_Return2Start);

// Transitions
// Change from state 0 to state 1 if button 0 in remote control is pressed 
// or after 1 minute in state 0
bool transitionS0S1(){
    bool changeState = false;
    
    if(isReadyForLeveling){
      changeState = true;  
      lcd.clear();
    }
    

    return changeState;
}
// Change from state 0 to state 3 if button 3 in remote control is pressed 
bool transitionS0S3() {
    bool changeState = false;

    if(isInStartPosition){
      changeState = true;
      lcd.clear();  
    }
    
    return changeState;
}
// Change from state 1 to state 2 if button 0 is pressed
bool transitionS1S2(){
    bool changeState = false;
    // Receive IR-remote signal
    if (irrecv.decode(&results)) { // Waiting for decoding

        if (results.value == IR_REMOTE_0) {
            changeState = true;
            lcd.clear();
        }
        irrecv.resume(); // Receive the next value
    }

    return changeState;
}
// Change from state 2 to state 3 if button 0 is pressed
bool transitionS2S3() {
    bool changeState = false;
    // Receive IR-remote signal
    if (!isNotOriented) { // Waiting for decoding
        changeState = true;
        lcd.clear();
    }
    
    return changeState;
}
// Change from state 3 to state 4 if button 0 is pressed
bool transitionS3S4() {
    bool changeState = false;

    // Change state automatically if the sun is already set/ not yet risen
    if(sunPosition.ElevationAngle < 0){
      programTime = rtc.now();
      if(programTime.hour() <12){
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Sonne noch nicht");
          lcd.setCursor(0,1);
          lcd.print("aufgegangen");
      }
      else{
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Sonne bereits");
          lcd.setCursor(0,1);
          lcd.print("untergegangen");
      }
      delay(5000);
      changeState = true;
    }
    
    //Serial.println("Transition 3->4");
    // Receive IR-remote signal
    else
    if (irrecv.decode(&results)) { // Waiting for decoding

        if (results.value == IR_REMOTE_0) {
            changeState = true;
            lcd.clear();
        }
        irrecv.resume(); // Receive the next value
    }

    return changeState;
}
// Change from state 4 to state 1 if the start position is reached
bool transitionS4S1() {
    bool changeState = false;

    if (isInStartPosition) {
        changeState = true;
    }

    return changeState;
}
#pragma endregion

#pragma region Arduino
/******************************************************************************
Arduino control
******************************************************************************/
void setup() {
    // Set Timer1
    //timer1CompA_Init(TIMER1_PRESCALER,TIMER1_INTERVAL_US);
    timer1CompA_Init(TIMER1_PRESCALER,TIMER1_INTERVAL_US);
    timer1CompB_Init(TIMER1_PRESCALER,TIMER1_INTERVAL_US);
    timer1CompA_Off();
    timer1CompB_Off();

    // Set serial monitor
    Serial.begin(9600);
    // Set LCD-Display 
    lcd.init();       // initialize lcd
    lcd.backlight();  // turn on backlight
    lcd.setCursor(0, 0);
    
    // Set RTC-Clock
    rtc.begin();
    if (rtc.lostPower()) {
      Serial.println("RTC is NOT running, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      DateTime now = rtc.now();

      Serial.print(now.year(), DEC);
      Serial.print('/');
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.println();

    }
    // Initialize global time variable
    programTime = (DateTime)rtc.now();
    programMicros = micros();
    // Set LED-Pin
    pinMode(LED_PIN, OUTPUT);
    
    // Set GPS-Device
    L76X_Init_9600();

    // Set MiniIMU
    //imu.Init();
     
    // Set State-Machine, add transitions to the states
    S0->addTransition(&transitionS0S1, S1);
    S0->addTransition(&transitionS0S3, S3);
    S1->addTransition(&transitionS1S2, S2);
    S2->addTransition(&transitionS2S3, S3);
    S3->addTransition(&transitionS3S4, S4);
    S4->addTransition(&transitionS4S1, S1);

    // Set IR-Reciever
    irrecv.enableIRIn();

    //xStepper.stepFunc = xStepNema17;
    // Set End-Point-Pin
    pinMode(ENDPOINT_PIN, INPUT_PULLUP);
}

void loop() {
    machine.run();
    //xStepper.oneStep(dirX);
    //delayMicroseconds(1000);
}
#pragma endregion
