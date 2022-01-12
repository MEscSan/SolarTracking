#pragma once
#include "SolarCalculator.h"

SolarCalculator::SolarCalculator() {}

//  Get the number of the day within the year from 1 to 365 (366 in leapyears)
//  1st January = 1
int SolarCalculator::getDayInYear(DateTime d) {

  int currentYear = d.year();
  int currentMonth = d.month();
  int currentDay = d.day();
  
  bool leapYear = isLeapYear(currentYear);
  int dayInYear = 0;

  //  Day in year for common years
  switch (currentMonth) {
  case 1:
      dayInYear = currentDay;
      break;
  case 2:
      dayInYear = 31 + currentDay;
      break;
  case 3:
      dayInYear = 31 + 28 + currentDay;
      break;
  case 4:
      dayInYear = 59 + 31 + currentDay;
      break;
  case 5:
      dayInYear = 90 + 30 + currentDay;
      break;
  case 6:
      dayInYear = 120 + 31 + currentDay;
      break;
  case 7:
      dayInYear = 151 + 30 + currentDay;
      break;
  case 8:
      dayInYear = 181 + 31 + currentDay;
      break;
  case 9:
      dayInYear = 212 + 31 + currentDay;
      break;
  case 10:
      dayInYear = 243 + 30 + currentDay;
      break;
  case 11:
      dayInYear = 273 + 31 + currentDay;
      break;
  case 12:
      dayInYear = 304 + 30 + currentDay;
      break;
  default:
      dayInYear = -1;
      break;
  }
  // Extra day if leap year
  if (leapYear & currentMonth > 2) {
      dayInYear = dayInYear + 1;
  }

  return dayInYear;
}

// Check if a year is a leap year
bool SolarCalculator::isLeapYear(int currentYear) {
  bool isLeapYear = false;

  if (currentYear % 4 != 0) {
      isLeapYear = false;
  }
  else if (currentYear % 100 != 0) {
      isLeapYear = true;
  }
  else if (currentYear % 400 != 0) {
      isLeapYear = false;
  }
  else {
      isLeapYear = true;
  }

  return isLeapYear;
}

//  Equation of time 
double SolarCalculator::equationOfTime(int dayInYear) {
  double eqTime = 0;
  eqTime = 60 * (-0.171 * sin(0.0337 * dayInYear + 0.465) - 0.1299 * sin(0.01787 * dayInYear - 0.168));
  return eqTime;
}


// Solar declination angle
double SolarCalculator::decline(int dayInYear) {
  double decl = 0;
  decl = -23.45 * cos(2 * PI * (dayInYear + 10) / 365);
  return decl;
}

// Solar hour angle
double SolarCalculator::hourAngle(double currentLongitude, double eqTime, int currentHour, int currentMinute) {
  double hourAngle = 0;
  hourAngle = 15 * (currentHour + currentMinute / 60.0 - (15 - currentLongitude) / 15.0 - 12 + eqTime / 60.0);
  return hourAngle;
}

//  Solar height angle (elevationAngle) in degrees
//  Sunrise and sunset => elevationAngle = 0�
double SolarCalculator::elevationAnglePrecalculation(double currentLatitude, AstronomicalCoordinate a) {
  double elevationAnglePreCalc = 0;
  elevationAnglePreCalc = sin(PI / 180 * currentLatitude) * sin(PI / 180 * a.Decline) + cos(PI / 180 * currentLatitude) * cos(PI / 180 * a.Decline) * cos(PI / 180 * a.HourAngle);
  return elevationAnglePreCalc;
}

double SolarCalculator::elevationAngle(double elevationAnglePrecalculation) {
  double elevationAngle;
  elevationAngle = asin(elevationAnglePrecalculation) * 180 / PI;
  return elevationAngle;
}

// Convert from elevationAngle to Motor-Angle, needed for elevation-angle-Tracking
/*******************************************************************
Conditions:
-> threaded bar with thread-pitch 1.25
-> constants are application specific
********************************************************************/
double SolarCalculator::elevationAngleChange2MotorAngle(double elevationAngleOld_deg, double elevationAngleNew_deg) {

    // 0. Convert from degrees to rad
    double elevationAngleOld = elevationAngleOld_deg*3.1416/180;
    double elevationAngleNew = elevationAngleNew_deg*3.1416/180;
    
    Serial.print(elevationAngleNew);
    Serial.print("\t");
    
    // 1. Get required threaded-bar translation
    double c0Sqr = ((-cos(elevationAngleOld - RHO - PI_HALF) * 2 * A * B) + (pow(A, 2) + pow(B, 2)));
    double c0 = sqrt(c0Sqr);

    double c1Sqr = ((-cos(elevationAngleNew - RHO - PI_HALF) * 2 * A * B) + (pow(A, 2) + pow(B, 2)));
    double c1 = sqrt(c1Sqr);

    // Translation
    double deltaC = c1 - c0;
    
    Serial.print(deltaC);
    Serial.print("\t");
    
    // ToDo: Check that deltaC is within Min-Max

    // 2. Convert translation to thread-bar turns
    double numRevolutions = deltaC / THREAD_PITCH;

    // 3. Convert thread-bar turns to motor-steps
    double motorAngle = numRevolutions * 360;

    return motorAngle;
}

// Solar azimuth in degrees
double SolarCalculator::azimuth(double elevationAnglePrecalculation, GeographicalCoordinate g, AstronomicalCoordinate a, DateTime t) {
  double azimuth = 0;
  double x = elevationAnglePrecalculation;

  azimuth = -(sin(PI / 180 * g.Latitude) * x - sin(PI / 180 * a.Decline)) / (cos(PI / 180 * g.Latitude) * sin(acos(x)));
  if ((t.hour() + t.minute() / 60) <= (13 - g.Longitude / 15 - a.EqTime / 60)) {
      azimuth = acos(azimuth) * 180 / PI;
  }
  else {
      azimuth = 360 - acos(azimuth) * 180 / PI;
  }
  return azimuth;
}

// Print date and day in year in serial monitor
void SolarCalculator::printDate(int y, int m, int d, int dayInYear) {
  Serial.print("\nDate: ");
  Serial.print(y);
  Serial.print("/");
  if (m < 10) {
      Serial.print("0");
  }
  Serial.print(m);
  Serial.print("/");
  if (d < 10) {
      Serial.print("0");
  }
  Serial.print(d);
  Serial.print(" => ");
  Serial.print(dayInYear);
}

void SolarCalculator::printAstroCoordinates(AstronomicalCoordinate a) {

  Serial.print("\t");
  Serial.print(a.Decline);
  Serial.print("\t");
  Serial.print(a.EqTime);
  Serial.print("\t");
  Serial.print(a.HourAngle);
}

void SolarCalculator::printSolarPosition(SolarPosition s) {

  Serial.print("\t");
  Serial.print(s.ElevationAngle);
  Serial.print("\t");
  Serial.print(s.Azimuth);
}

SolarPosition SolarCalculator::getSolarPosition(DateTime dt, GeographicalCoordinate g) {
    
  AstronomicalCoordinate a;
  SolarPosition s;

  int currentYear = dt.year();
  int currentMonth = dt.month();
  int currentDay = dt.day();
  int dayInYear = 0;
  int currentHour = dt.hour();
  int currentMinute= dt.minute();


  dayInYear = getDayInYear(dt);
  a.EqTime = equationOfTime(dayInYear);
  a.Decline = decline(dayInYear);
  a.HourAngle = hourAngle(g.Longitude, a.EqTime, currentHour, currentMinute );
  s.ElevationAnglePrecalculation = elevationAnglePrecalculation(g.Latitude, a);
  s.ElevationAngle = elevationAngle(s.ElevationAnglePrecalculation);
  s.Azimuth = azimuth(s.ElevationAnglePrecalculation, g, a, dt);

  return s; 
}
