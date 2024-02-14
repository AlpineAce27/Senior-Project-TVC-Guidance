#include "MPU6050.h" //The MPU6050 Uses the I2C communication protocol, so that library must be included.
#include "Wire.h"    //Arduinos official I2C communication library
#include "I2Cdev.h"  //Another I2C library

//------------------------------------------------VARIABLES----------------------------------------------

//Objects: IMU
MPU6050 imu1;

//IMU variables:
float gx;
int gxOffset;

//IMU Calibration variables
float average;
float total;
int calibrationPoints = 2000;

//------------------------------------------------SET UP----------------------------------------------

void setup() 
{
  Wire.begin();
  Serial.begin(38400);
  imu1.initialize();
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
  delay(3000);
}

//------------------------------------------------LOOP----------------------------------------------

void loop() 
{
  gx = imu1.getRotationX() - gxOffset;
  Serial.println(gx);
  //if calibration was done correctly, this should be close to 0 (+-50ish) when the sensor is not moving
}