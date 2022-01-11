// Class with static functions for calculating solar-position and the needed angles for the solar-tracking axis
// Equations from: http://geoastro.de/SME/tk/index.htm
#ifndef _SOLARCALCULATOR_H_
#define _SOLARCALCULATOR_H_

#pragma once

#include <math.h>
#include "Arduino.h"
#include "RTClib.h" 

// Mathematical constants
//#define PI 3.14159265359

// Constants and length for elevationAngle-to-motorAngle calculation
#define PI_HALF 1.57079632679 
#define RHO 0.84318499417
#define A  21.42574386106
#define B 13.8
#define THREAD_PITCH 1.25

//****Structs****
/*typedef struct Date {
	int Year;
	int Month;
	int Day;
	int DayInYear;
};

typedef struct Time {
	int Hour;
	int Minute;
};*/

typedef struct AstronomicalCoordinate {
	double EqTime;
	double Decline;
	double HourAngle;
};


typedef struct GeographicalCoordinate {
	double Longitude;
	double Latitude;
};

typedef struct SolarPosition {
	double Azimuth;
	double ElevationAnglePrecalculation;
	double ElevationAngle;
};

class SolarCalculator {
	public:
		SolarCalculator();
		static SolarPosition getSolarPosition(DateTime dt, GeographicalCoordinate g);

		static bool isLeapYear(int currentYear);
		static int getDayInYear(DateTime d);
		static double elevationAngleChange2MotorAngle(double elevationAngleOld, double elevationAngleNew);

		static double equationOfTime(int dayInYear);
		static double decline(int dayInYear);
		static double hourAngle(double currentLongitude, double eqTime, int currentHour, int currentMinute);
		static double elevationAnglePrecalculation(double currentLatitude, AstronomicalCoordinate a);
		static double elevationAngle(double elevationAnglePrecalculation);
		static double azimuth(double elevationAnglePrecalculation, GeographicalCoordinate g, AstronomicalCoordinate a, DateTime t);
		
		static void printDate(int y, int m, int d, int dayInYear);
		static void printAstroCoordinates(AstronomicalCoordinate a);
		static void printSolarPosition(SolarPosition s);
};

#endif
