

#include "MPU6050.h" //The MPU6050 Uses the I2C communication protocol, so that library must be included.
#include "Wire.h"    //Arduinos official I2C communication library
#include "I2Cdev.h"  //Another I2C library

MPU6050 imu1; //create an MPU6050 object called "imu1" if there was second, we'd call it "imu2"

//create 6x 16 bit integers, 3 for xyz of acceleration and 3 for xyz gyroscope
int16_t ax, ay, az, gx, gy, gz;

//pre-installed led is usually on pin 13
#define LED_PIN 13

  /*
   * the mpu readings do not come back as flat 0's with no motion, so some offsets need to be applied to even them out and get them to a common datum
   */
int gxOffset = 0;
int gyOffset = 0;
int gzOffset = 0;

  /*
   * According to the datasheet, https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
   * on page 12, it says "Sensitivity Scale Factor    FS_SEL=0      131       LSB/(º/s)" which means at the default Factor Scale setting of 0,
   * it will give a reading of 131 per deg/sec. This is what we need to divide by to convert the readings into degree/sec.
   */
int gyroSensScaleFactor = 131;

//make a bool blinkstate so we can blink the LED if needed
bool blinkstate = false;

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
  
   /*
   * RECURSIVE MOVING AVGERATING FILTER:
   *  Current Avg = Previous Avg + (Current data point - previous data point)/counter
   *     * 
   *  source: https://www.youtube.com/watch?v=HCd-leV8OkU&list=WL&index=8&t=2224s @21:00
   */
 
  //create the variables that will be used in the recursive equations for calibration
  int dp0 = 0; //data point 0
  int dp1 = 1; //data point 1
  int counter = 1; //tracks how many point have been avged
  int avg1 = 0;
  int avg0 = 0;
  int calibrationPoints = 100; //the amount of data points needed to reach full calibartion (more points will take longer, but be more accurate)
  

    for(int i = 1; i <= calibrationPoints; i++)
    {
      //run a recursive averaging filter throught the required datapoints
      dp1 = imu1.getRotationX(); //get the rotation value fromt the sensor
      avg1 = avg0 + (dp1 - dp0)/counter; //run it through the avg filter
      counter++; //increase the counter
      dp0 = dp1; //update the data points
      avg0 = avg1; //update the average
      //Serial.print(dp0); Serial.print(" "); Serial.print(avg0); Serial.println();    
    }
    //when the loop is finished, set average as the gyro X offset
    gxOffset = avg1;
    

  //Reset the data points and avgs for a new axis calibration
  dp0 = 0;
  dp1 = 0;
  counter = 1;
  avg0 = 0;
  avg1 = 0;
 
    for(int i = 1; i <= calibrationPoints; i++)
    {
      //run a recursive averaging filter throught the required datapoints
      dp1 = imu1.getRotationY(); //get the rotation value fromt the sensor
      avg1 = avg0 + (dp1 - dp0)/counter; //run it through the avg filter
      counter++; //increase the counter
      dp0 = dp1; //update the data points
      avg0 = avg1; //update the average
      //Serial.print(dp0); Serial.print(" "); Serial.print(avg0); Serial.println();    
    }
    //when the loop is finished, set average as the gyro Y offset
    gyOffset = avg1;
    
    
//Reset the data points and avgs for a new axis calibration
  dp0 = 0;
  dp1 = 0;
  counter = 1;
  avg0 = 0;
  avg1 = 0;
 
    for(int i = 1; i <= calibrationPoints; i++)
    {
      //run a recursive averaging filter throught the required datapoints
      dp1 = imu1.getRotationZ(); //get the rotation value fromt the sensor
      avg1 = avg0 + (dp1 - dp0)/counter; //run it through the avg filter
      counter++; //increase the counter
      dp0 = dp1; //update the data points
      avg0 = avg1; //update the average
      //Serial.print(dp0); Serial.print(" "); Serial.print(avg0); Serial.println();    
    }
    //when the loop is finished, set average as the gyro Z offset
    gzOffset = avg1;
    
    Serial.print("Gyro X axis offset: "); Serial.println(gxOffset);
    Serial.print("Gyro Y axis offset: "); Serial.println(gyOffset);
    Serial.print("Gyro Z axis offset: "); Serial.println(gzOffset);
    Serial.println("Gyroscope calibration complete!");
 
  delay(5000);  
  
  pinMode(LED_PIN, OUTPUT);

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

//Variables needed for a low pass filter on the Y gyroscope
float gy1 = 0;  //Data Point 1 (current unfiltered data)
float gy0 = 0; //Data Point 0 (previous unfiltered data)
float fgy1 = 0;  //Filtered Data Point 1 (current filtered data)
float fgy0 = 0; //Filtered Data Point 0 (previous filtered data)

//Variables needed for a low pass filter on the X gyroscope
float gz1 = 0;  //Data Point 1 (current unfiltered data)
float gz0 = 0; //Data Point 0 (previous unfiltered data)
float fgz1 = 0;  //Filtered Data Point 1 (current filtered data)
float fgz0 = 0; //Filtered Data Point 0 (previous filtered data)

void loop()
{
  //GYRO X AXIS get the data from the imu, apply the offset found earlier, and then apply the scale factor shown to get degree/sec
  gx1 = (imu1.getRotationX()- gxOffset)/gyroSensScaleFactor;  
  //run them through a first order low pass filter
  fgx1 = 0.969*fgx0 + 0.0155*gx1 + 0.0155*gx0; 
  //update the variables for the next iteration
  gx0 = gx1;   
  fgx0 = fgx1;

  //GYRO Y AXIS get the data from the imu, apply the offset found earlier, and then apply the scale factor shown to get degree/sec
  gy1 = (imu1.getRotationY()- gyOffset)/gyroSensScaleFactor;  
  //run them through a first order low pass filter
  fgy1 = 0.969*fgy0 + 0.0155*gy1 + 0.0155*gy0; 
  //update the variables for the next iteration
  gy0 = gy1;   
  fgy0 = fgy1;

 //GYRO Z AXIS get the data from the imu, apply the offset found earlier, and then apply the scale factor shown to get degree/sec
  gz1 = (imu1.getRotationZ()- gzOffset)/gyroSensScaleFactor;  
  //run them through a first order low pass filter
  fgz1 = 0.969*fgz0 + 0.0155*gz1 + 0.0155*gz0; 
  //update the variables for the next iteration
  gz0 = gz1;   
  fgz0 = fgz1;

  //print them to the serial port
  //Serial.print(gx0); Serial.print("\t"); Serial.print(gy0); Serial.print("\t"); Serial.print(gz0); Serial.println();//unfiltered data
  Serial.print(fgx0); Serial.print("\t"); Serial.print(gy0); Serial.print("\t"); Serial.print(gz0); Serial.println();//filtered data
 

  blinkstate = !blinkstate;
  digitalWrite(LED_PIN, blinkstate);
}
