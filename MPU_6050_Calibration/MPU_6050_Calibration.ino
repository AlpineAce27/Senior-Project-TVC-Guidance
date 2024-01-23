#include "MPU6050.h" //The MPU6050 Uses the I2C communication protocol, so that library must be included.
#include "Wire.h"    //Arduinos official I2C communication library
#include "I2Cdev.h"  //Another I2C library

MPU6050 imu1; //create an MPU6050 object called "imu1" if there was second, we'd call it "imu2"

//create 6x 16 bit integers, 3 for xyz of acceleration and 3 for xyz gyroscope
int16_t ax, ay, az, gx, gy, gz;

  /*
   * the mpu readings do not come back as flat 0's with no motion, 
   * so some offsets need to be applied to even them out and get them to a common datum
   */
int gxOffset = 0;
int gyOffset = 0;
int gzOffset = 0;

void setup() 
{
  Wire.begin(); //Start communiction over I2c
  Serial.begin(38400); //Start the serial port
  Serial.println();
  Serial.println("Initializing MPU6050 over I2C protocol...");
  imu1.initialize();
  Serial.println("Sensor initialized successfully");

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
  int dp0 = 0; //previous data point
  int dp1 = 1; //current data point
  int counter = 1; //tracks how many point have been avged
  int avg0 = 0; //previouis average
  int avg1 = 0; //current average
  int calibrationPoints = 1000; //the amount of data points needed to reach full calibartion. 
  //1000 is overkill but it looks cooler on the serial plotter. This could be done with only 50 points and would also be faster
  
  //calibrate the three gyro axes using the recursive avg filter

    for(int i = 1; i <= calibrationPoints; i++)
    {
      //run a recursive averaging filter throught the required datapoints
      dp1 = imu1.getRotationX(); //get the rotation value fromt the sensor
      avg1 = avg0 + (dp1 - dp0)/counter; //run it through the avg filter
      counter++; //increase the counter
      dp0 = dp1; //update the data points
      avg0 = avg1; //update the average

      //you can uncomment the line below and open the serial plotter to watch the equation in action.
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

      //you can uncomment the line below and open the serial plotter to watch the equation in action.
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
      
      //you can uncomment the line below and open the serial plotter to watch the equation in action.
      //Serial.print(dp0); Serial.print(" "); Serial.print(avg0); Serial.println();    
    }
    //when the loop is finished, set average as the gyro Z offset
    gzOffset = avg1;
    
    Serial.print("Gyro X axis offset: "); Serial.println(gxOffset);
    Serial.print("Gyro Y axis offset: "); Serial.println(gyOffset);
    Serial.print("Gyro Z axis offset: "); Serial.println(gzOffset);
    Serial.println("Gyroscope calibration complete!");
  


}

void loop()
{
  
}
