

#include "MPU6050.h" //The MPU6050 Uses the I2C communication protocol, so that library must be included.
#include "Wire.h"    //Arduinos official I2C communication library
#include "I2Cdev.h"  //Another I2C library
#include "Servo.h"   //Servo library

MPU6050 imu1; //create an MPU6050 object called "imu1" if there was second, we'd call it "imu2"
Servo xServo;
int xServoAngle = 90;
bool clockwise = true;

//create 6x 16 bit integers, 3 for xyz of acceleration and 3 for xyz gyroscope
int16_t gx;

/*
* the mpu readings do not come back as flat 0's with no motion, so some offsets need to be applied to even them out and get them to a common datum
*/
int gxOffset = 0;
float angleX = 0;
float refresh = 100;
float deltaT = 1/refresh;
float tempIMU;

/*
* According to the datasheet, https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
* on page 12, it says "Sensitivity Scale Factor    FS_SEL=0      131       LSB/(º/s)" which means at the default Factor Scale setting of 0,
* it will give a reading of 131 for every 1 deg/sec. This is what we need to divide by to convert the readings into degree/sec.
*/
int counter = 1; //tracks how many point have been avged
int avg1 = 0;
int avg0 = 0;
int calibrationPoints = 2000; //the amount of data points needed to reach full calibartion (more points will take longer, but be more accurate
float gyroSensScaleFactor = 65.5;

//make a bool blinkstate so we can blink the LED if needed
bool blinkstate = false;

long loop_timer; 

void setup() 
{
  Wire.begin(); //Start communiction over I2c
  Serial.begin(38400); //Start the serial port
  Serial.println();
  Serial.println("Initializing MPU6050 over I2C protocol...");
  imu1.initialize();
  Serial.println("Sensor initialized successfully");

  //calibration is needed since the sensor has significant offsets on each axis

  //give a countdown to warn the user (time of the countdown is in the for loop header)
  Serial.println("Time to calibrate the IMU. Please place the sensor on a flat surface and do not disturb it during the calibration.");
  for(int counter = 3; counter > 0; counter--)
  {
    delay(1000);
    Serial.println(counter);
  }
  
 
  //create the variables that will be used in the averagin equations for calibration
  float dp0;
  float runningTotal = 0; //data point 0
  float average = 0;
  int calibrationPoints = 2000; //the amount of data points needed to reach full calibartion (more points will take longer, but be more accurate)
  

    for(int i = 1; i <= calibrationPoints; i++)
    {
      //run a recursive averaging filter throught the required datapoints
      dp0 = imu1.getRotationX();            //get the rotation value fromt the sensor
      runningTotal = runningTotal + dp0;    //add it to the running total
      Serial.println(dp0);    
    }
    //when the loop is finished, calculate the average of all the points and set it as the gyro X offset
    gxOffset = runningTotal/calibrationPoints;
    
    imu1.setFullScaleGyroRange(1);
    Serial.print("Gyro X axis offset: "); Serial.println(gxOffset);
    Serial.print("Gyro Full Scale Range:  "); Serial.println(imu1.getFullScaleGyroRange());
    Serial.println("Gyroscope calibration complete!");
    xServo.attach(11);
    xServo.write(90);
 
  delay(2000);  
  
  loop_timer = millis();
}
 /*
   * 1st ORDER LOW PASS FILTER:
   * New Filtered Data = alpha*(old Filtered Data) + beta*(New Raw data) + beta*(Old Raw data)
   * 
   * alpha and beta are constants. They are determined through complicated math (shown here: https://www.youtube.com/watch?v=HJ-C4Incgpw&t=115s at 3:15)
   * alpha and beta in this case straight up copied/pasted from this video: https://www.youtube.com/watch?v=eM4VHtettGg at 1:07
   * 
   * Ideally I'd like to find out how to calculate my own values for alpha and beta, but this will work for now.
   */
   
//Variables needed for a low pass filter on the X gyroscope
float gx1 = 0;  //Data Point 1 (current unfiltered data)
float gx0 = 0; //Data Point 0 (previous unfiltered data)
float fgx1 = 0;  //Filtered Data Point 1 (current filtered data)
float fgx0 = 0; //Filtered Data Point 0 (previous filtered data)

void loop()
{
  loop_timer = millis();       
  tempIMU = imu1.getRotationX();     //Reset the loop timer
  gx1 = (tempIMU*2 - gxOffset)/gyroSensScaleFactor;      //GYRO X AXIS get the data from the imu, apply the offset found earlier and then divided by the scale factor to find the deg/sec
 
  // Serial.print(gx1);
  // Serial.print("  = (");
  // Serial.print(tempIMU*2);
  // Serial.print(" - ");
  // Serial.print(gxOffset);
  // Serial.print(") / ");
  // Serial.print(gyroSensScaleFactor);
  // Serial.println();
  
  //run them through a first order low pass filter
  //fgx1 = 0.969*fgx0 + 0.0155*gx1 + 0.0155*gx0; 
  //update the variables for the next iteration
  //gx0 = gx1;   
  //fgx0 = fgx1;

  //print them to the serial port
  //Serial.print(gx1);
  //Serial.print(fgx0);
                                            
  angleX = (gx1*deltaT) + angleX;   //Calculate the traveled x angle and add this to the current x angle variable (1/20 = 0.1)
  Serial.println(angleX);
  
  if(xServoAngle <= 150 && clockwise == true)
  {
    xServoAngle=xServoAngle + 2;
  }
  if(xServoAngle >= 30 && clockwise == false)
  {
    xServoAngle=xServoAngle - 2;
  }
  if(xServoAngle == 30 || xServoAngle == 150)
  {
    clockwise = !clockwise;
  }

  xServo.write(xServoAngle);

  while(millis() < loop_timer + 1000/refresh);
  {}                                                              //Wait until the loop_timer reaches 20 milliseconds (10Hz) before starting the next loop                                            
}
