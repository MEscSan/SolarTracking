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

void lcdPrintCoords(LiquidCrystal_I2C lcd, GeographicalCoordinate coords, int row = 1);
void lcdPrintGPS(LiquidCrystal_I2C lcd, GNRMC gps, int row = 1);
void lcdPrintTime(LiquidCrystal_I2C lcd, RTC_DS3231 rtc, int row = 0);
void lcdPrintSolarPosition(LiquidCrystal_I2C lcd, SolarPosition s, int row = 1);
void lcdPrintCompassHeading(LiquidCrystal_I2C lcd, MiniIMU imu, int row = 0);

#endif
