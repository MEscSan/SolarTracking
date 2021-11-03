#pragma once
#include"ICR_Functions.h"

// Prepare next Interrupt-interval: 
//  1.  Set inteval till next interrupt (OCR-Parameter) => motor speed 
//  2.  Decide what motor is next (according to remaining Steppers Flag):
//  Example: remainingSteppersFlag: 00110011(Motors 0,1,4,5 remaining; all motors have the same speed)
//                       &  1 << 0: 00000001 => true  => nextStepperFlag: 00000001  (Stepper 0)
//                       &  1 << 2: 00000100 => false 
void timer1CompA_setNextInterruptInterval(ISR_Flags* flags, Stepper steppers[], int numSteppers) {
    bool movementComplete = true;
    unsigned long minSpeed = 999999;
    flags->nextStepperFlag = 0;

    //Decide what motor has to make the next step(the slowest one_)
    for (int i = 0; i < numSteppers; i++) {
        if (((1 << i) & flags->remainingSteppersFlag) && steppers[i].getClicksPerStep() < minSpeed) {
            minSpeed = steppers[i].getClicksPerStep();
        }
    }

    for (int i = 0; i < numSteppers; i++) {
        if (!steppers[i].getMovementDone())
            movementComplete = false;
        if (((1 << i) & flags->remainingSteppersFlag) && steppers[i].getClicksPerStep() == minSpeed)
            flags->nextStepperFlag |= (1 << i);
    }

    // If all movements complete, reset timer-interval to the maximum (65500 
    if (flags->remainingSteppersFlag == 0) {
        timer1CompA_Off();
        OCR1A = 65500; //
    }

    OCR1A = minSpeed;
}

// Turn Timer-Interrupt 1 - Compare match A on
// Set bit OCIE1A (2nd bit) in register TIMSK1 to 1
void timer1CompA_On(){
    TIMSK1 |= (1 << OCIE1A);
}

// Turn Timer-Interrupt 1 - Compare match A off
// Set bit OCIE1A (2nd bit) in register TIMSK1 to 0 (0 == ~1)
void timer1CompA_Off(){
    TIMSK1 &= ~(1 << OCIE1A);
}
