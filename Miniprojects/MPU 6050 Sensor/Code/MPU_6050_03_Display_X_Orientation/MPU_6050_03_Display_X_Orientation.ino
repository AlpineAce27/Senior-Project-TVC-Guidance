#include "MPU6050.h" //The MPU6050 Uses the I2C communication protocol, so that library must be included.
#include "Wire.h"    //Arduinos official I2C communication library
#include "I2Cdev.h"  //Another I2C library

//------------------------------------------------VARIABLES----------------------------------------------

//Objects: IMU
MPU6050 imu1;

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
float xAngleMeasured = 0;
float angularVelX = 0;

//Timing variables
float refresh = 100; //this is in hz
float deltaT = 1/refresh;
long loop_timer;

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
}

//------------------------------------------------LOOP----------------------------------------------

void loop() 
{
  loop_timer = millis();

  gx = imu1.getRotationX();

  angularVelX = (gx - gxOffset)/gyroScaleFactor;
  
  xAngleMeasured = angularVelX*deltaT + xAngleMeasured;

  Serial.println(xAngleMeasured);
  
  while(millis() < loop_timer + 1000/refresh); {} 
}
