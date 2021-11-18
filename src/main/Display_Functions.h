/*
	Set of functions for information display in a I2C-LCD
	Warning: requires the Stepper-Class (Stepper.h) and LiquidCrystal_I2C-Class (LiquidCrystal_I2C.h) 
*/
#ifndef _DISP_FUNCTIONS_H_
#define _DISP_FUNCTIONS_H_

#pragma once
#include <LiquidCrystal_I2C.h>
#include "Stepper.h"
#include "SolarCalculator.h" 
#include "RTClib.h"  
#include "L76X.h"
#include "MiniIMU.h"

void lcdPrintGPS(LiquidCrystal_I2C lcd, GNRMC gps);
void lcdPrintTime(LiquidCrystal_I2C lcd, RTC_DS1307 rtc);
void lcdPrintSolarPosition(LiquidCrystal_I2C lcd, SolarPosition s);
void lcsPrintYaw(LiquidCrystal_I2C lcd, MiniIMU imu);

#endif
