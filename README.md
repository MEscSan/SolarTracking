# Solar Tracking for Arduino

Warnings: 
1.	Stepper.h and Stepper.cpp are procedurally programmed and they are only to be used from "main.ino"-Srcipt
2.	L76X.h/.cpp and DEV_Config.h/.cpp files delivered by WaveShare, source: https://www.waveshare.com/wiki/L76X_GPS_Module

# Required Libraries:
For State-Machine
* StateMachine: https://github.com/jrullan/StateMachine
* LinkedList (in library-directory delete file test.cpp): https://github.com/ivanseidel/LinkedList/blob/master/examples/Sort/Sort.ino

For IR-Remote control
-> IRremote: https://github.com/Arduino-IRremote/Arduino-IRremote

For RTC-Clock:
-> RTClib

For miniIMU9 (all by Pololu):
-> LSM6
-> LIS3MDL
-> L3G
-> LSM303

# Pins:(Arduino MEGA, CNC-Shield rev3)
-> GPS-L76X: RX: 52 (Blau), TX: 53 (Orange)
-> IR-Receiver: 22
