// Class for Pololu Mini IMU-9
/* 
Code based on
MinIMU - 9 - Arduino - AHRS
Pololu MinIMU - 9 + Arduino AHRS(Attitude and Heading Reference System):
https://github.com/pololu/minimu-9-ahrs-arduino

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/
sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.
*/

#ifndef _MINI_IMU_H_
#define _MINI_IMU_H_

#pragma once
#include "Arduino.h"
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <L3G.h>
#include <LSM303.h>

//#define IMU_V5

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board
#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

//byte gyro_sat = 0;

#pragma region Arduino script  

#pragma endregion

#pragma region Enums

enum DeviceVersion{
    V4, 
    V5
};

enum AxisDefinition{
    Y_right_Z_Down,
    Y_left_Z_Up
};

enum OutputType{
    DCM,
    ANALOGS,
    EULER_ANG
};
#pragma endregion


class MiniIMU {
	public:
		MiniIMU(DeviceVersion version = DeviceVersion::V4, AxisDefinition axis = AxisDefinition::Y_right_Z_Down, bool correctedData = true);

		//Compass
		void Compass_Heading();

		// DCM
		void Normalize();
		void Drift_correction();
		void Matrix_update();
		void Euler_angles();

		//I2C
		void I2C_Init();
		void Gyro_Init();
		void Read_Gyro();
		void Accel_Init();
		void Read_Accel();
		void Compass_Init();
		void Read_Compass();

		// Data output
		void Serial_Printdata(OutputType outType);
        float* GetEulerAng();
        float* GetDcmMatrix();
        float* GetAnalogGyro();
        float* GetAnalogAccel();
        float* GetAnalogCompass();

        // Update
        void Update_IMU_Values();

		//	Vector Operations
		float Vector_Dot_Product(float vector1[3], float vector2[3]);
		void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]);
		void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2);
		void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);

        // Matrix Operations
		void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3]);

    private:
        DeviceVersion _version;
        AxisDefinition _axisDef;
        bool _correctedData;    //True will print the corrected data,
                                //False will print uncorrected data of the gyros (with drift)
        LSM6 _gyro_acc;
        LIS3MDL _mag;
        
        L3G _gyro;
        LSM303 _compass;
        
        float _G_Dt;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
        unsigned int _counter;
        
        long _timer;   //general purpuse timer
        long _timer_old;
        long _timer24; //Second timer used to print values
        int _AN[6]; //array that stores the gyro and accelerometer data
        int _AN_OFFSET[6] = { 0,0,0,0,0,0 }; //Array that stores the Offset of the sensors
        int _SENSOR_SIGN[9];
        int _gyro_x;
        int _gyro_y;
        int _gyro_z;
        int _accel_x;
        int _accel_y;
        int _accel_z;
        int _magnetom_x;
        int _magnetom_y;
        int _magnetom_z;
        float _c_magnetom_x;
        float _c_magnetom_y;
        float _c_magnetom_z;
        float MAG_Heading;

        float _Accel_Vector[3] = { 0,0,0 }; //Store the acceleration in a vector
        float _Gyro_Vector[3]= { 0,0,0 };//Store the gyros turn rate in a vector
        float _Omega_Vector[3]= { 0,0,0 }; //Corrected Gyro_Vector data
        float _Omega_P[3] = { 0,0,0 };//Omega Proportional correction
        float _Omega_I[3]= { 0,0,0 };;//Omega Integrator
        float _Omega[3]={0,0,0};


        // Euler angles
        float _roll;
        float _pitch;
        float _yaw;

        float _errorRollPitch[3]={0,0,0};
        float _errorYaw[3]={0,0,0};

        float _DCM_Matrix[3][3]= {
           {1,0,0},
           {0,1,0},
           {0,0,1}
    };
        float _Update_Matrix[3][3]{ 
            {0,1,2}, 
            {3,4,5}, 
            {6,7,8} 
    }; //Gyros here

        float _Temporary_Matrix[3][3]{
            {0,0,0}, 
            {0,0,0}, 
            {0,0,0}
    }; 
};


#endif 
