# Ball-Beam-Balance
An introductory controls project to understand and implement the PID and Linear-state Feedback controllers.

A simple strucuture for the balance was made out of cardboard. The aim is to balance a ball on a beam using a servo to control the tilt of the beam. 
This is a SISO (Single Input Single Output) system with the input being the beam angle controlled by the servo and output being the position of the ball. The feedback loop feeds the ball's position to the controller using an ultrasonic sensor (HC SR04).
