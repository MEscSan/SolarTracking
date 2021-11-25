#pragma once
#include"ICR_Functions.h"

/******************************************************************************
function:
    convert a time interval in microseconds 
data:
    prescaler :timer prescaler (1,8, 64, 256, 1024)
    interval  :timer interval in microseconds
    clkFrequency: processor clock frequency in MHz, default is 16 MHz (ATmega328p)
******************************************************************************/
unsigned long microseconds2Clicks(int prescaler, unsigned long interval, int clkFrequency = 16){
    unsigned long intervalInClicks = 0;
    intervalInClicks = (unsigned long) (interval/(prescaler /(double)clkFrequency));
    return intervalInClicks;
}

#pragma region TIMER 1 Compare/Match A
/******************************************************************************
function:
    Initialize timer 1 in compare/match mode (A)
parameter:
    prescaler :timer prescaler (1,8, 64, 256, 1024)
    interval  :timer interval (in microseconds)
******************************************************************************/
void timer1CompA_Init(int prescaler,unsigned long interval) {

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
    switch (prescaler){
        case 8:
            TCCR1B |= B00000010;    //Prescaler 8
            break;
        case 64:
            TCCR1B |= B00000011;    //Prescaler 64
            break;
        case 256:
            TCCR1B |= B00000100;    //Prescaler 256
            break;
        case 1024:
            TCCR1B |= B00000101;    //Prescaler 1024
            break;
        default:
            TCCR1B |= B00000001;    //Prescaler 1
            break;
    }
    //5. Set timer Mode in Timer Interrupt Mask Register 1 (TIMSK1)
    TIMSK1 |= (1 << OCIE1A);
    //6. Set number of Timer pulses till interrupt
    int intervalInClicks = microseconds2Clicks(prescaler, interval);
    OCR1A = intervalInClicks;
    interrupts();
}

/******************************************************************************
function:
    Prepare next Interrupt-interval: 
    1.  Set inteval till next interrupt (OCR-Parameter) => motor speed 
    2.  Decide what motor is next (according to remaining Steppers Flag):
    Example: remainingSteppersFlag: 00110011(Motors 0,1,4,5 remaining; all motors have the same speed)
                       &  1 << 0: 00000001 => true  => nextStepperFlag: 00000001  (Stepper 0)
                       &  1 << 2: 00000100 => false 
parameter:
    flags :pointer to flag-Struct for motor coordination
    steppers[]: pointer to array of stepper-motors to be moved
    numSteppers: number of stepper-motors, size of the steppers-array
    prescaler: timer prescaler (1,8, 64, 256, 1024)
******************************************************************************/
void timer1CompA_SetNextInterruptInterval(ISR_Flags* flags, Stepper steppers[], int numSteppers, int prescaler) {
    bool movementComplete = true;
    unsigned int minSpeed = 999999;
    flags->nextStepperFlag = 0;

    //Decide what motor has to make the next step(the slowest one_)
    for (int i = 0; i < numSteppers; i++) {
        if (((1 << i) & flags->remainingSteppersFlag) && steppers[i].getMicrosecondsPerStep() < minSpeed) {
            minSpeed = steppers[i].getMicrosecondsPerStep();
        }
    }

    for (int i = 0; i < numSteppers; i++) {
        if (!steppers[i].getMovementDone())
            movementComplete = false;
        if (((1 << i) & flags->remainingSteppersFlag) && steppers[i].getMicrosecondsPerStep() == minSpeed)
            flags->nextStepperFlag |= (1 << i);
    }

    // If all movements complete, reset timer-interval to the maximum (65500 
    if (flags->remainingSteppersFlag == 0) {
        timer1CompA_Off();
        OCR1A = 65500; //
    }

    unsigned int minSpeedInClicks = microseconds2Clicks(prescaler,minSpeed);
    OCR1A = minSpeedInClicks;
}

/******************************************************************************
function:
    Turn Timer-Interrupt 1 - Compare match A on
    Set bit OCIE1A (2nd bit) in register TIMSK1 to 1
******************************************************************************/
void timer1CompA_On(){
    TIMSK1 |= (1 << OCIE1A);
}

/******************************************************************************
function:
    Turn Timer-Interrupt 1 - Compare match A off
    Set bit OCIE1A (2nd bit) in register TIMSK1 to 0 (0 == ~1)
******************************************************************************/
void timer1CompA_Off(){
    TIMSK1 &= ~(1 << OCIE1A);
}
#pragma endregion

#pragma region TIMER 1 Compare/Match B
/******************************************************************************
function:
    Initialize timer 1 in compare/match mode (B)
parameter:
    prescaler :timer prescaler (1,8, 64, 256, 1024)
    interval  :timer interval (in microseconds)
******************************************************************************/
void timer1CompB_Init(int prescaler, unsigned long interval) {

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
    switch (prescaler) {
    case 8:
        TCCR1B |= B00000010;    //Prescaler 8
        break;
    case 64:
        TCCR1B |= B00000011;    //Prescaler 64
        break;
    case 256:
        TCCR1B |= B00000100;    //Prescaler 256
        break;
    case 1024:
        TCCR1B |= B00000101;    //Prescaler 1024
        break;
    default:
        TCCR1B |= B00000001;    //Prescaler 1
        break;
    }
    //5. Set timer Mode in Timer Interrupt Mask Register 1 (TIMSK1)
    TIMSK1 |= (1 << OCIE1B);
    //6. Set number of Timer pulses till interrupt
    int intervalInClicks = microseconds2Clicks(prescaler, interval);
    OCR1B = intervalInClicks;
    interrupts();
}

/******************************************************************************
function:
    Turn Timer-Interrupt 1 - Compare match B on
    Set bit OCIE1B (3rd bit) in register TIMSK1 to 1
******************************************************************************/
void timer1CompB_On() {
    TIMSK1 |= (1 << OCIE1B);
}

/******************************************************************************
function:
    Turn Timer-Interrupt 1 - Compare match A off
    Set bit OCIE1B (3rd bit) in register TIMSK1 to 0 (0 == ~1)
******************************************************************************/
void timer1CompB_Off() {
    TIMSK1 &= ~(1 << OCIE1B);
}
#pragma endregion