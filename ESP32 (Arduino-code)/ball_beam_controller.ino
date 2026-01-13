#include <ESP32Servo.h>

// Define the servo and the pin it is connected to
Servo myServo;
const int servoPin = 23;
// Define pins for the ultrasonic sensor
const int echoPin = 25;
const int trigPin = 26;

// Define the minimum and maximum pulse widths for the servo
const int minPulseWidth = 500; // 0.5 ms
const int maxPulseWidth = 2500; // 2.5 ms
// y = mx + c  ; c = 500
float m = (2500.0f - 500.0f) / 180.0f;
int theta = 50; // This angle results in almost horizontal position of the beam
float theta_LSFC, theta_PID;
// Linear-state Feedback control
float A[4] = {0, 1, 0, 0};
float B[2] = {0, 9.81};
float C[2] = {1, 0};
// float D = 0;
// float K[2] = {0.4921, 0.4077};
float K[2] = {-0.035, -0.0135};
// float K[2] = {-3.5, -1.5};
float G = -0.041;
float L[2] = {18, 97.75};
float x[2] = {0, 0};
float u, u_pid;

// PID control (0.035,0.00035,0.015)
float kp = 0.035;
// float ki = 0;
float ki = 0.00035; // without dt in error sum term  
// float ki = 0.003; // with dt in error sum term
float kd = 0.015;
float P, I, D;
// General Variables 
double distance[3] = {0, 0, 0}; // For d_t-1 , approx d_t, filtered d_t
float vel = 0;
float dt;
float x_ref = 15.0; // 15cm
float error_sum = 0;
unsigned long currTime = 0;
unsigned long prevTime = 0;


void setup() {
  // Attach the servo to the specified pin and set its pulse width range
  myServo.attach(servoPin, minPulseWidth, maxPulseWidth);
  // Set the PWM frequency for the servo
  myServo.setPeriodHertz(50); // Standard 50Hz servo
  // Begin serial communication at 115200 baud rate
  Serial.begin(115200);
  // Set echoPin as input and trigPin as output
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  // Print sensor information to the serial monitor
  Serial.println("Ultrasonic sensor:");  
}

void loop() {

  digitalWrite(trigPin, LOW); 
  delayMicroseconds(1);
  // Send a 10-microsecond high signal to the trigPin
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  // Return to low signal
  digitalWrite(trigPin, LOW);
  // Measure the duration of the high signal on the echoPin
  unsigned long duration = pulseIn(echoPin, HIGH);
  currTime = micros();
  // Calculate the distance in cm using the speed of sound (0.034 centimeter per µs)
  distance[1] = duration * 0.034 / 2;
  // Filter for distance data: x_filtered = 0.7*x_prev + 0.3*x_raw;
  distance[2] = 0.7*distance[0] + 0.3*distance[1];
  // distance[2] = distance[1];
  if ((distance[2] < 2) || (distance[2] > 300))
  {
    distance[2] = 0;
  }

  // // #####################################################################################
  // // Linear State feedback controller: u = -K*x + G*r
  // // #####################################################################################
  // Using Ultrasonic measurement converted from cm to m
  x[0] = distance[2]/100.0f; 
  // Velocity converted from cm/microsec to m/s 
  dt = (currTime - prevTime) * 1e-6; // seconds
  x[1] = ((distance[2] - distance[0]) / 100.0f) / dt;  // m/s
  // u is the required beam angle in radians
  u = - (K[0]*x[0] + K[1]*x[1]) + G * x_ref/100.0f;
  u = u * 100; // compare with pid

  // #####################################################################################
  // PID controller: u = kp * (x-xref) + ki * errorsum + + kd * (vel)
  // #####################################################################################
  vel = x[1]*100;
  error_sum = error_sum + ((distance[2]-x_ref)) * dt;
  if (millis() < 5000) 
  {
    error_sum = 0;
  }
  P = kp * (distance[2]-x_ref);
  I = ki * error_sum;
  D = kd * (vel);
  u_pid = P+I+D ; // distance is measured in cm. So, x_ref should also be in cm.

  // #####################################################################################
  // Apply input to the servo
  // #####################################################################################
  // Assuming that the required servo angle(theta) is twice as the beam angle (2 to 1 linear relation) but the beam is at 0 deg when servo arm is at 50 
  theta_LSFC = u * 57.29577951 + 50; //LSFC
  theta_PID = u_pid * 57.29577951 + 50; // PID
  // SELECT THE CONTROLLER (THETA_PID OR THETA_LSFC)
  theta = theta_LSFC; 
  // theta = theta_PID;
  float theta_print = theta;
  theta = constrain(theta, 25, 75);
  myServo.writeMicroseconds(theta * m + 500);

  Serial.print("X_POSITION: ");   
  Serial.print(distance[2]);   
  Serial.print(" , ");
  // Serial.print(vel);
  // Serial.print(" , ");
  // Serial.print(P / u * 100);
  // Serial.print(" , ");
  // Serial.print(I / u * 100);
  // Serial.print(" , ");
  // Serial.print(D / u * 100);
  Serial.print("u: ");
  Serial.print(u,5);
  Serial.print(" , ");
  Serial.print("u_pid: ");
  Serial.print(u_pid,5);
  Serial.print(" , ");
  Serial.print("theta_LSFC: ");
  Serial.print(theta_LSFC,5);
  Serial.print(" , ");
  Serial.print("theta_pid: ");
  Serial.println(theta_PID, 5);
  

  distance[0] = distance[2];
  prevTime = currTime;
}