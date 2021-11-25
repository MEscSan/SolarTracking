# Solar Tracking for Arduino

Warnings: 
1.	Stepper.h and Stepper.cpp are procedurally programmed and they are only to be used from "main.ino"-Srcipt
2.	L76X.h/.cpp and DEV_Config.h/.cpp files delivered by WaveShare, source: https://www.waveshare.com/wiki/L76X_GPS_Module

# Required Libraries:

-> StateMachine: https://github.com/jrullan/StateMachine
-> LinkedList: https://github.com/ivanseidel/LinkedList/blob/master/examples/Sort/Sort.ino

# Pins:(Arduino MEGA, CNC-Shield rev3)
-> GPS-L76X: RX: 52, TX: 53
-> IR-Receiver: 22
