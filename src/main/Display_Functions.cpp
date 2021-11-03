#pragma once
#include "Display_Functions.h"

void lcdPrintTime(LiquidCrystal_I2C lcd, RTC_DS1307 rtc){
	DateTime now = rtc.now();
	lcd.setCursor(0, 0);
	lcd.print(now.day(), DEC);
	lcd.print("/");
	lcd.print(now.month(), DEC);
	lcd.print("/");
	lcd.print(now.year(), DEC);
	lcd.print(" ");
	lcd.print(now.hour(), DEC);
	lcd.print(":");
	lcd.print(now.minute(), DEC);
	lcd.print("  ");
	delay(2000);
}

void lcdPrintSolarPosition(LiquidCrystal_I2C lcd, SolarPosition s){
	lcd.setCursor(0, 1);
	lcd.print("A");
	lcd.print(s.Azimuth);
	lcd.print(" ");
	lcd.print("Z");
	lcd.print(s.Zenith);
	lcd.print(" ");
}
