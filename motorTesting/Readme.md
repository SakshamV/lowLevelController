# Brief Description

1. This repo has the platform.txt file used for building this code.
2. This code was written for Arduino Uno Rev3 , built on Ubuntu 20.04.
3. Arduino IDE 2.0.4 was used to build and flash this code.
4. Built with C11 and CPP14 standard.

# Tasks

1. This arduino codebase is used for wheel odometry, and wheel velocity control of the terpbot.
2. The controller talks to an RPI over serial port 0.
3. Arduino Uno is connected to 2 encoders and drives 2 motors via a dual DC motor driver shield.
4. The controller transmits current wheel velocities as ticks per unit time (75Hz) to the RPI.
5. The controller receives target wheel velocities as ticks per unit time (75Hz) from the RPI.
6. An anti-windup PID controller is used to control the wheel velocities.
7. For the wiring logic, please look at the initialization of aptly named variables in the code.