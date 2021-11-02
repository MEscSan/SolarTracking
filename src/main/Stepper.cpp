#pragma once
#include "Stepper.h"


//****Constructor****
Stepper::Stepper(volatile int stepperPins[], volatile LiquidCrystal_I2C lcd,  float gearRatio = 1, unsigned int id,StepperType t=StepperType::NEMA17, unsigned long clicksPerStep=CLICKS_PER_STEP, unsigned int stepsPerRevolution=STEPS_PER_REVOLUTION_NEMA17){
    _type = t;
    _stepperPins = stepperPins;
    _clicksPerStep = clicksPerStep;
    _stepsPerRevolution = stepsPerRevolution;
    _gearRatio = gearRatio;
    _motorId = id;
    _lcd = lcd;

    // Initialize Stepper-Pins:
    int numPins = 0;
    switch (_type)
    {
        case    StepperType::BYJ28:
            numPins = 4;
        break;
        default:
            numPins = 2;
        break;
    }

    for (int i = 0; i < numPins; i++) {
        pinMode(_stepperPins[i], OUTPUT);
    }

}

//****Public methods****
// Move one Step in the direction of _dir
void Stepper::oneStep() {
    if (_type == StepperType::NEMA17) {
        stepNema17();
    }
    else{
        step28BYJ();
    }
}

void Stepper::step28BYJ() {
    // Define a variable, use four low bit to indicate the state of port
    static byte out = 0x01;
    // Decide the shift direction according to the rotation direction
    if (_dir) { // ring shift left
        out != 0x08 ? out = out << 1 : out = 0x01;
    }
    else { // ring shift right
        out != 0x01 ? out = out >> 1 : out = 0x08;
    }
    // Output singal to each port
    for (int i = 0; i < 4; i++) {
        digitalWrite(_stepperPins[i], (out & (0x01 << i)) ? HIGH : LOW);
    }
}

void Stepper::stepNema17() {
    
    int dirPin = _stepperPins[0];
    int stepPin = _stepperPins[1];
    
    if (_dir) {
        //Counterclockwise
        digitalWrite(dirPin, LOW);
    }
    else {
        // Clockwise
        digitalWrite(dirPin, HIGH);
    }
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
}

// Reset stepper information so far
void Stepper::resetStepperInfo() {
    _stepCountInMovement = 0;
    _totalStepsRequested = 0;
    _stepPosition = 0;
    _movementDone = false;
}

// Reset stepper information from previous movement
void Stepper::resetStepperMovement() {
    _stepCountInMovement = 0;
    _movementDone = false;
}

// Prepare the next motor-movement of the selected motor (reset info from previous movement, set new requested number of steps)
void Stepper::prepareMovement(int angleRequested, ISRFlags flags) {
    long stepsRequested = Stepper::angle2Steps(angleRequested);

    _dir = stepsRequested > 0 ? 0 : 1;
    _totalStepsRequested = abs(stepsRequested);
    resetStepperMovement();
    flags.remainingSteppersFlag |= (1 << _motorId); // "Add" motor to byte-flag
}

// Convert from degree to motor-steps
int Stepper::angle2Steps(int motorAngle) {
    int motorSteps = 0;

    //motorAngle = motorAngle - (motorAngle/360)*360; // Check that angle isn't bigger than 360�
    motorSteps = (long)motorAngle * _stepsPerRevolution * _gearRatio / 360.0;

    /*
    //For Debug-purpose only
    Serial.print(motorAngle);
    Serial.print("�");
    Serial.print("\t");
    Serial.print(motorSteps);
    Serial.println(" Steps");
    */

    return motorSteps;
}

//Convert from motor-steps to degree
int Stepper::steps2Angle(long motorSteps) {
    int motorAngle = 0;

    motorAngle = (int)360 * motorSteps / (_stepsPerRevolution * _gearRatio);
    //motorAngle = motorAngle - (motorAngle/360)*360.0; // Check that angle isn't bigger than 360�

    /*
    //For Debug-purpose only
    Serial.print(motorSteps);
    Serial.print(" Steps");
    Serial.print("\t");
    Serial.print(motorAngle);
    Serial.println("�");
    */

    return motorAngle;
}

// Show current motor Position on lcd-Screen
void Stepper::lcdPrintMotorAngle() {

    int motorAngle = Stepper::steps2Angle(whichMotor, steppers[whichMotor].stepPosition);
    int motorAngleDisplay = motorAngle - (motorAngle / 360) * 360; // Check that angle isn't bigger than 360�

    _lcd.setCursor(10, whichMotor); // set the cursor to column 8, line 0 or 1 depending on the motor
    _lcd.print(" ");
    _lcd.print(motorAngleDisplay);
    //lcd.setCursor(15,whichMotor);
    _lcd.print((char)223);
    _lcd.print("    ");
}

// Prepare next Interrupt-interval: 
//  1.  Set inteval till next interrupt (OCR-Parameter) => motor speed 
//  2.  Decide what motor is next (according to remaining Steppers Flag):
//  Example: remainingSteppersFlag: 00110011(Motors 0,1,4,5 remaining; all motors have the same speed)
//                       &  1 << 0: 00000001 => true  => nextStepperFlag: 00000001  (Stepper 0)
//                       &  1 << 2: 00000100 => false 
void setNextInterruptInterval() {
    bool movementComplete = true;
    unsigned long minSpeed = 999999;
    nextStepperFlag = 0;

    //Decide what motor has to make the next step(the slowest one)
    for (int i = 0; i < NUM_STEPPERS; i++) {
        if (((1 << i) & remainingSteppersFlag) && steppers[i].clicksPerStep < minSpeed) {
            minSpeed = steppers[i].clicksPerStep;
        }
    }

    for (int i = 0; i < NUM_STEPPERS; i++) {
        if (!steppers[i].movementDone)
            movementComplete = false;
        if (((1 << i) & remainingSteppersFlag) && steppers[i].clicksPerStep == minSpeed)
            nextStepperFlag |= (1 << i);
    }

    // If all movements complete, reset timer-interval to the maximum (65500 
    if (remainingSteppersFlag == 0) {
        TIMER1_INTERRUPTS_OFF;
        OCR1A = 65500; //
    }

    OCR1A = minSpeed;
}

//****Interrupt routine Timer 1****
ISR(TIMER1_COMPA_vect) {
    unsigned int tmpCtr = OCR1A;

    OCR1A = 65500;

    for (int i = 0; i < NUM_STEPPERS; i++) {

        // Jump the for-loop for all motors that are not remaining
        // 00001111 & 01000000 => 00000000 => !0 == 1 => true
        if (!((1 << i) & remainingSteppersFlag))
            continue;

        // Jump the for-loop for all motors that are not the one in the nextStepperFlag
        // 00000010 & 00000010 => 00000010 => !2 = 0 => false
        // 00000010 & 00000100 => 00000000 => !0 = 1 => true
        if (!(nextStepperFlag & (1 << i))) {
            continue;
        }
        // Get remaining motor
        volatile StepperInfo& s = steppers[i];

        // Run one step in the motor
        if (s.stepCountInMovement < s.totalStepsRequested) {
            oneStep(s.t, s.dir, s.stepperPins);
            s.stepCountInMovement++;
            s.stepPosition += s.dir == 0 ? 1 : -1;
            if (s.stepCountInMovement >= s.totalStepsRequested) {
                s.movementDone = true;
                remainingSteppersFlag &= ~(1 << i);
            }
        }
    }

    setNextInterruptInterval();

    TCNT1 = 0;
}
