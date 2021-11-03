/*
	Set of functions for implementing Timer Interrupt Routines with Stepper-Motors
	Warning: requires the Stepper-Class (Stepper.h)
*/
#ifndef _ICR_FUNCTIONS_H_
#define _ICR_FUNCTIONS_H_

#pragma once

#include "Stepper.h" 

#pragma region TIMER 1 Compare/Match A

void timer1CompA_setNextInterruptInterval(ISR_Flags* flags, Stepper stepers[], int numSteppers);
void timer1CompA_Off();
void timer1CompA_On();

#pragma endregion

#endif
