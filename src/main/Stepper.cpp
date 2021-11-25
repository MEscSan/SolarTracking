#pragma once
#include "Stepper.h"


//****Constructor****
Stepper::Stepper(volatile int stepperPins[], float gearRatio, unsigned int id, StepperType t, unsigned long microsecondsPerStep, unsigned int stepsPerRevolution)
{
    _type = t;
    _microsecondsPerStep = microsecondsPerStep;
    _stepsPerRevolution = stepsPerRevolution;
    _gearRatio = gearRatio;
    _motorId = id;

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
    
    // memmove(destination, source, size in bytes)
    memmove(_stepperPins, stepperPins, numPins*sizeof(volatile int));

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
void Stepper::prepareMovement(int angleRequested, ISR_Flags *flags) {
    long stepsRequested = angle2Steps(angleRequested);
    _dir = stepsRequested > 0 ? 0 : 1;
    _totalStepsRequested = abs(stepsRequested);
    resetStepperMovement();
    flags->remainingSteppersFlag |= (1 << _motorId); // "Add" motor to byte-flag
}


// Convert from degree to motor-steps
int Stepper::angle2Steps(int motorAngle) {
    int motorSteps = 0;

    //motorAngle = motorAngle - (motorAngle/360)*360; // Check that angle isn't bigger than 360�
    motorSteps = (int)motorAngle * _stepsPerRevolution * _gearRatio / 360.0;

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

volatile unsigned long Stepper::getMicrosecondsPerStep(){
  return _microsecondsPerStep;
}

volatile bool Stepper::getMovementDone(){
  return _movementDone;  
}

volatile int Stepper::getDirection(){
    return _dir;
}

volatile unsigned int Stepper::getStepCountInMovement() {
    return _stepCountInMovement;
}

volatile unsigned int Stepper::getTotalStepsRequested() {
    return _totalStepsRequested;
}

long Stepper::getStepPosition() {
    return _stepPosition;
}

void Stepper::setMovementDone(volatile bool value){
  _movementDone = value;
}

void Stepper::setDirection(volatile int dir) {
    // Direction can only be 1 (counterclockwise) or 0 (clockwise)
    if (dir == 0 || dir == 1) {
        _dir = dir;
    }
}

void Stepper::setStepCountInMovement(volatile unsigned int value){
  _stepCountInMovement = value;  
}

void Stepper::setStepPosition(long value) {
    _stepPosition = value;
}
