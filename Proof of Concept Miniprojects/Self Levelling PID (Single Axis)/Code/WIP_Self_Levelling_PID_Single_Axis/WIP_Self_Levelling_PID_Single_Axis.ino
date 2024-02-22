#include "MPU6050.h" //The MPU6050 Uses the I2C communication protocol, so that library must be included.
#include "Wire.h"    //Arduinos official I2C communication library
#include "I2Cdev.h"  //Another I2C library
#include "Servo.h"   //Servo library

//------------------------------------------------VARIABLES----------------------------------------------

//Objects: IMU and X axis Servo
MPU6050 imu1;
Servo servoX;

//Servo Variables
int servoXPin = 11;

//IMU variables:
float gx;
int gxOffset;
int FSSEL = 1;
float gyroScaleFactor = 65.5; //This value is based off of the MPU6050 datasheet, pg 12. We are using FS_SEL = 1

//IMU Calibration variables
float average;
float total;
int calibrationPoints = 2000;

//Angle variables, one for the set angle and one for the observed/measured angle
float angularVelX = 0;
float x = 0; //we want the servo to attempt to keep the sensor flat on the X axis
int xAngleSet = 90;
float xAngleMeasured = 0;

//Timing variables
float refresh = 100; //this is in hz
float deltaT = 1/refresh;
long loop_timer;

//PID variables
float pGain = 0.5; //
float iGain = 0.0;
float dGain = 0.5;

float previousXError = 0;
float currentXError = 0;
float proportionalError = 0;
float integralError = 0;
float derivativeError = 0;

//other variables
float xAngleDesired = 0;

//------------------------------------------------SET UP----------------------------------------------

void setup() 
{
  Wire.begin();
  Serial.begin(38400);
  imu1.initialize();
  imu1.setFullScaleGyroRange(FSSEL);
  Serial.println("Sensor Initialized");
  Serial.println("Calibrating");
  
  //take 2000 readings and add them all up
  for(int i = 1; i <= calibrationPoints; i++)
    {
      gx = imu1.getRotationX();            
      total = total + gx;
      Serial.println(gx);    
    }
    //when the loop is finished, calculate the average of all the points and set it as the gyro X offset
    gxOffset = total/calibrationPoints;
  Serial.print("Gyro X Offset Value: "); Serial.println(gxOffset);

  //assign the servo to a pin and set it to a starting angle
  servoX.attach(servoXPin);
  servoX.write(xAngleSet);
  delay(5000);
  
}

//------------------------------------------------LOOP----------------------------------------------

void loop() 
{
  loop_timer = millis();

  //calculate the current angle of the sensor
  gx = imu1.getRotationX();
  angularVelX = (gx - gxOffset)/gyroScaleFactor;
  xAngleMeasured = angularVelX*deltaT + xAngleMeasured; //Serial.println(xAngleMeasured);

  //determine the error between where we are, and where we want to be
  currentXError = xAngleDesired - xAngleMeasured;

  //apply this error into the PID components
  proportionalError = currentXError;
  integralError = integralError + currentXError*deltaT;
  derivativeError = (currentXError - previousXError)/deltaT;

  Serial.print(proportionalError); Serial.print(" | "); Serial.print(integralError); Serial.print(" | "); Serial.print(derivativeError); Serial.println(" | ");

  //multiply these errors by their repsective gains, add them togehter and write the new angle to the servo it to the servo
  xAngleSet = xAngleSet + proportionalError*pGain + integralError*iGain + derivativeError*dGain;
  servoX.write(xAngleSet);
  
  //update the errors in preparation for the next loop
  previousXError = currentXError;
  
  while(millis() < loop_timer + 1000/refresh); {} 
}
