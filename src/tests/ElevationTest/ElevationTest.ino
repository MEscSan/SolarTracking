// Test-program for mirror inclination(elevation angle)
// It changes the mirror inclination from 0° to 90° and back in steps of 10 degrees
          
#include "SolarCalculator.h"    //  Static class for getting the position of the sun
#include "RTClib.h"             //  RTC-Clock


// Macros for millis() ant timeouts
#define STATE0_TIMEOUT 30000 // 30s= 30000 ms
#define GPS_TIMEOUT 30000  // 90s= 90000 ms
#define TRACKER_DELAY 1 // 1 min
#define MILLIS_30_SECONDS 30000 // 30s= 30000 ms
#define MILLIS_20_MINUTES 1200000 // 20min=1200000 ms


SolarPosition trackerPosition;
GeographicalCoordinate coords;
DateTime today = DateTime(2022, 1,20,19,17,90);

int nowHour = 0, nowMin = 0;

/******************************************************************************
Arduino control
******************************************************************************/
void setup() {

    // Set serial monitor
    Serial.begin(9600);

    coords.Latitude = 50;
    coords.Longitude = 12;
}

void loop() {

  today = DateTime(2022, 1,20,nowHour,nowMin,0);

  trackerPosition = SolarCalculator::getSolarPosition(today, coords);

  Serial.print(today.hour());
  Serial.print(" :\t");
  Serial.print(today.minute());
  Serial.print(" ->\t");
  Serial.println(trackerPosition.ElevationAngle);
  
  if(nowMin == 0){
    nowMin = 30;  
  }
  else{
    nowMin = 0;
    nowHour += 1;  
  }

  if(nowHour == 24){
    
    while(true) ; 
  }
}
