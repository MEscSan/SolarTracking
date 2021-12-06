// Class for stepper-motor control using interrupt routine
// The class assumes the existence of an Interrupt-Routine which has to be defined in the main-program
// Code based on library by http://www.iforce2d.net/sketches/

#ifndef _STEPPER_H_
#define _STEPPER_H_

#pragma once
#include "Arduino.h"

#define MICROSECONDS_PER_STEP 600
#define STEPS_PER_REVOLUTION_NEMA17 200
#define STEPS_PER_REVOLUTION_28BYJ 2048


#pragma region Enums
enum StepperType {
    NEMA17,
    BYJ28 
};
#pragma endregion

#pragma region Structs

/// <summary>
/// Set of flags needed for the ISR-Routine to work(only needed for motor coordination);
/// These are byte Flags: (Example for remainingSteppersFlag) 
/// No Motor remaining:  00000000
/// Motor 1  remanining: 00000010
/// Motor 3  remaining:  00001000
/// Motors 4 to 6 remaining: 01110000 
/// Instead of ++ Operator, the number of remaining motors is tracked by bit-shifting
/// Note: the flag allows a maximum of 8 motors
/// </summary>
typedef struct ISR_Flags {
    volatile byte remainingSteppersFlag = 0;
    volatile byte nextStepperFlag = 0;
    volatile int remainingSteppers = 0;
};

#pragma endregion


class Stepper {
    public:
        
        /// <summary>
        /// Constructor for stepper-motor class
        /// </summary>
        /// <param name="stepperPins"> Int-Array with the Pins of the stepper:
        ///        NEMA17  :   {dir, step}
        ///        28BYJ   :   {pin1, pin2, pin3, pin4}</param>
        /// <param name="lcd"></param>
        /// <param name="gearRatio"></param>
        /// <param name="id"> Motor Id for using several motors</param>
        /// <param name="t"></param>
        /// <param name="clicksPerStep"></param>
        /// <param name="stepsPerRevolution"></param>
        Stepper(volatile int stepperPins[], float gearRatio = 1, unsigned int id = 0, StepperType t = StepperType::NEMA17, unsigned long microsecondsPerStep = MICROSECONDS_PER_STEP, unsigned long stepsPerRevolution = STEPS_PER_REVOLUTION_NEMA17);
            
        // Public Methods
        void oneStep();
        void oneStep(int dir);
        void resetStepperInfo();
        void resetStepperMovement();
        void prepareMovement(double angleRequested, ISR_Flags *flags);

        // Getter- and Setter
        volatile bool getMovementDone();
        volatile int getDirection();
        volatile unsigned int getStepCountInMovement();
        volatile unsigned long getTotalStepsRequested();
        volatile unsigned long getMicrosecondsPerStep();
        long getStepPosition();
        double getAngle();
        
      
        void setMovementDone(volatile bool value);
        void setDirection(volatile int dir);
        void setStepCountInMovement(volatile unsigned long value);
        void setStepPosition(long value);
        
    private:
        //Attributes
        volatile StepperType _type;                  // Motor type, either eihter NEMA17 or 28BYJ
        volatile unsigned long _microsecondsPerStep;       // timer clicks between two steps, smaller is faster  
        volatile unsigned int _stepsPerRevolution;
        volatile float _gearRatio;                   // Input speed / Output speed : Ratio > 1 => gear slows movement down 
        volatile unsigned int _motorId;               // Motor Id for using several motors
            // derived parameters (automatically updated)
            // per movement variables (only changed once per movement)
        volatile int _dir;                           // current direction of movement, used to keep track of position
        volatile unsigned long _totalStepsRequested;  // number of steps requested for current movement
        volatile bool _movementDone = false;         // true if the current movement has been completed (used by main program to wait for completion)
        volatile unsigned long _stepPosition;                          // current position of stepper (total of all movements taken so far)

            // per iteration variables (potentially changed every interrupt)
        volatile unsigned long _stepCountInMovement;  // number of steps completed in current movement

        //Private methods
        void step28BYJ();
        void step28BYJ(int dir);
        void stepNema17();
        void stepNema17(int dir);
        int angle2Steps(int motorAngle);
        int steps2Angle(long motorSteps);
        unsigned long angle2Steps(double motorAngle);
        double steps2Angle(unsigned long motorSteps);

        // Motor set-up (Member of variable length)
        volatile int _stepperPins[4];                  // Int-Array with the Pins of the stepper: 
                                                      //      NEMA17  :   {dir, step}
                                                      //      28BYJ   :   {pin1, pin2, pin3, pin4}    
};

#pragma region Obsolete procedural Structure
/*
//****Structs****
typedef struct StepperInfo {
    // Motor set-up
    volatile StepperType t;                     // Motor type, either eihter NEMA17 or 28BYJ
    volatile unsigned long clicksPerStep;       // timer clicks between two steps, smaller is faster
    volatile unsigned int stepsPerRevolution;
    volatile float gearRatio;                   // Input speed / Output speed : Ratio > 1 => gear slows movement down
    volatile LiquidCrystal_I2C lcd;             // LCD-Display to which the motor delivers

    // derived parameters (automatically updated)
    // per movement variables (only changed once per movement)
    volatile int dir;                           // current direction of movement, used to keep track of position
    volatile unsigned int totalStepsRequested;  // number of steps requested for current movement
    volatile bool movementDone = false;         // true if the current movement has been completed (used by main program to wait for completion)
    long stepPosition;                          // current position of stepper (total of all movements taken so far)

    // per iteration variables (potentially changed every interrupt)
    volatile unsigned int stepCountInMovement;  // number of steps completed in current movement

    // Motor set-up (Member of variable length)
    volatile int stepperPins[];                        // Int-Array with the Pins of the stepper:
                                                //      NEMA17  :   {dir, step}
                                                //      28BYJ   :   {pin1, pin2, pin3, pin4}
};

//****Functions****
void oneStep(StepperType t, int dir, int stepperPins[]);
void step28BYJ(int dir, int stepperPins[]);
void stepNema17(int dir, int stepperPins[]);
void resetStepperInfo(StepperInfo& si);
void resetStepperMovement(volatile StepperInfo& si);
void prepareMovement(int whichMotor, int angleRequested);
void setNextInterruptInterval();
int angle2Steps(int whichMotor, int motorAngle);
int steps2Angle(int whichMotor, long motorSteps);
void lcdPrintMotorAngle(int whichMotor);*/
#pragma endregion


#endif
