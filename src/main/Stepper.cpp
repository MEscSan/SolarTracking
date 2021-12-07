#pragma once
#include "Stepper.h"


//****Constructor****
Stepper::Stepper(volatile int stepperPins[], float gearRatio, unsigned int id, StepperType t, unsigned long microsecondsPerStep, unsigned long stepsPerRevolution)
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

// Move one Step in the direction of _dir
void Stepper::oneStep(int dir) {
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

void Stepper::step28BYJ(int dir) {
    // Define a variable, use four low bit to indicate the state of port
    static byte out = 0x01;
    // Decide the shift direction according to the rotation direction
    if (dir) { // ring shift left
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

void Stepper::stepNema17(int dir) {
    
    int dirPin = _stepperPins[0];
    int stepPin = _stepperPins[1];
    
    if (dir) {
        //Clockwise
        digitalWrite(dirPin, HIGH);
    }
    else {
        // Counterlockwise
        digitalWrite(dirPin, LOW);
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

// Prepare the next motor-movement of the selected motor for a given angle (reset info from previous movement, set new requested number of steps)
void Stepper::prepareMovement(double angleRequested, ISR_Flags *flags) {
    unsigned long stepsRequested = angle2Steps(angleRequested);
    _dir = stepsRequested > 0 ? 0 : 1;
    _totalStepsRequested = abs(stepsRequested);
    resetStepperMovement();
    flags->remainingSteppersFlag |= (1 << _motorId); // "Add" motor to byte-flag
}

// Prepare the next motor-movement of the selected motor for a given number of steps (reset info from previous movement, set new requested number of steps)
void Stepper::prepareMovementSteps(unsigned long stepsRequested, int dir, ISR_Flags *flags) {
    _dir = dir > 0 ? 0 : 1;
    _totalStepsRequested = abs(stepsRequested);
    resetStepperMovement();
    flags->remainingSteppersFlag |= (1 << _motorId); // "Add" motor to byte-flag
}

// Convert from degree to motor-steps
unsigned long Stepper::angle2Steps(double motorAngle) {
    unsigned long motorSteps = 0;

    motorSteps = (unsigned long) (abs(motorAngle) * _stepsPerRevolution * _gearRatio )/ 360.0;

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
double Stepper::steps2Angle(unsigned long motorSteps) {
    int motorAngle = 0;

    motorAngle = (int)360 * motorSteps / (_stepsPerRevolution * _gearRatio);

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

volatile unsigned long Stepper::getStepCountInMovement() {
    return _stepCountInMovement;
}

volatile unsigned long Stepper::getTotalStepsRequested() {
    return _totalStepsRequested;
}

long Stepper::getStepPosition() {
    return _stepPosition;
}

double Stepper::getAngle(){
 double stepperAngle = steps2Angle(_stepPosition);
 return stepperAngle;
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

void Stepper::setStepCountInMovement(volatile unsigned long value){
  _stepCountInMovement = value;  
}

void Stepper::setStepPosition(long value) {
    _stepPosition = value;
}
