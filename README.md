# Ball-Beam-Balance
An introductory controls project to understand and implement the PID and Linear-state Feedback controllers.

A simple strucuture for the balance was made out of cardboard. The aim is to balance a ball on a beam using a servo to control the tilt of the beam. 
This is a SISO (Single Input Single Output) system with the input being the beam angle controlled by the servo and output being the position of the ball. An HC-SR04 ultrasonic sensor is used to measure the ball’s position in real time, and an ESP32 acts as the main controller. The measured position is fed back into the control loop to stabilize the system.

The controller design was first tested in MATLAB simulations, but a significant sim-to-real gap was observed, requiring retuning of gains for the real system. This repository includes:
- MATLAB simulation file
- ESP32 implementation code
- Videos demonstrating real-time performance with both PID and Linear State Feedback controllers
This project was a hands-on introduction to practical control implementation.
