#pragma once
#include "Display_Functions.h"

void lcdPrintCoords(LiquidCrystal_I2C lcd, GeographicalCoordinate coords, int row = 1){

  	lcd.setCursor(0, row);
  	lcd.print("Lat");
  	lcd.print(coords.Latitude);
  	lcd.print(" ");
  	lcd.print("Lon");
  	lcd.print(coords.Longitude);
  	lcd.print(":");
}

void lcdPrintGPS(LiquidCrystal_I2C lcd, GNRMC gps, int row){

  if(gps.Status != 1) {
    lcd.setCursor(0, row);
    lcd.print("GPS unavaliable");
  }
  else{
    lcd.setCursor(0, row);
    lcd.print("Lat");
    lcd.print(gps.Lat);
    lcd.print(" ");
    lcd.print("Lon");
    lcd.print(gps.Lon);
    lcd.print(":");
  }

}

void lcdPrintTime(LiquidCrystal_I2C lcd, RTC_DS3231 rtc, int row){
	DateTime now = rtc.now();
	lcd.setCursor(0, row);
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
	//delay(2000);
}

void lcdPrintSolarPosition(LiquidCrystal_I2C lcd, SolarPosition s, int row){
	lcd.setCursor(0, row);
	lcd.print("A:");
	lcd.print(s.Azimuth);
	lcd.print(" ");
	lcd.print("E:");
	lcd.print(s.ElevationAngle);
	lcd.print(" ");
}

void lcdPrintCompassHeading(LiquidCrystal_I2C lcd, MiniIMU imu, int row){
    float yaw = imu.GetYaw();
    lcd.setCursor(0,row);
    lcd.print("N ");
    lcd.print(yaw,0);
    lcd.print("    ");
}
