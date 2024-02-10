



#include "MPU6050.h" //The MPU6050 Uses the I2C communication protocol, so that library must be included.
#include <Servo.h>   //The library to control the 9gram servo
#include "Wire.h"    //Arduinos official I2C communication library
#include "I2Cdev.h"  //Another I2C library

MPU6050 imu1; //create an MPU6050 object called "imu1" if there was second, we'd call it "imu2"
Servo xServo;


//create 6x 16 bit integers, 3 for xyz of acceleration and 3 for xyz gyroscope
int16_t ax, ay, az, gx, gy, gz;
int servoAngle = 90;
int Xangle;

//pre-installed led is usually on pin 13
#define LED_PIN 13
#define servoPin 11

//the mpu readings do not come back as flat 0's with no motion, so some offsets need to be applied to even them out and get them to a common datum
int gxOffset = 0;

//make a bool blinkstate so we can blink the LED if needed
bool blinkstate = false;

//make a bool for testing the servo direction
bool clockwise = true;

void setup() 
{
  //tell the arduino that we want to control the X servo with servo pin (#11)
  xServo.attach(servoPin);
  xServo.write(servoAngle); //the servo can take any value between 0 and 180. This means 90 should be the middle)
  
  Wire.begin(); //Start communiction over I2c
  Serial.begin(38400); //Start the serial port
  Serial.println("Initializing MPU6050 over I2C protocol...");
  imu1.initialize();
  delay(2000);
  Serial.println("Sensor initialized successfully");

  //calibration is needed since the sensor has significant offsets on each axis

  //give a countdown to warn the user
  Serial.println("Time to calibrate the IMU. Please place the sensor on a flat surface and do not disturb it during the calibration.");
  for(int counter = 3; counter > 0; counter--)
  {
    delay(1000);
    Serial.println(counter);
  }

  //create the variables that will be used in the recursive equations for calibration
  int dp0 = 0; //data point 0
  int dp1 = 0; //data point 1
  int calibrationPoints = 1000; //the amount of data points needed to reach full calibartion (more points will take longer, but be more accurate)

  //calibrate the three gyro axes using the recursive avg filter
  
  /*
   * RECURSIVE AVGERATING FILTER:
   *  dataPoint1 = alpha * dataPoint0 + (1-alpha)*sensorReading
   *  
   *  where alpha = last count/current count    or    (k-1)/k   or    (counter-1)/counter
   *  
   *  source: https://www.youtube.com/watch?v=HCd-leV8OkU&list=WL&index=8&t=2224s @10:30
   */

  Serial.println("Calibrating Gyroscope X axis, do not touch the sensor");
  for(int counter = 1; counter <= calibrationPoints; counter++)
  {
    //run a recursive averaging filter throught the required datapoints
    dp1 = ((counter-1)/counter)*dp0 + (1 - ((counter-1)/counter))*imu1.getRotationX();
    dp0 = dp1;

    //when the counter is finished, set dp1 as the gyro X offset
    if(counter = calibrationPoints)
    {
      gxOffset = dp1;
    }
  }
 
  Serial.println("Gyroscope calibration complete.");
  Serial.print(gxOffset); Serial.print("\t");
 
  delay(5000);
  

  
  pinMode(LED_PIN, OUTPUT);
}

long gxdp0 = 0;
long gxdp1 = 0;
long alpha = 0.7;


void loop()
{
  
  //Use a lowpas filter to filter the signal coming from the sensor

  /*
   * 1st ORDER LOW PASS FILTER:
   * dataPoint1 = alpha*dataPoint0 + (1-alpha)*sensorReading
   * 
   * where (0 < alpha < 1) 
   * alpha = 1 means no prioritation on new data and alpha = 0 means maximum prioritization on new data
   * alpha = .7 is a good starting point.
   */
   
  //get the x gyroscope values from the imu, run them through a first order low pass filter
  //gxdp1 = alpha*gxdp0 + (1-alpha)*(imu1.getRotationX()-gxOffset);
  //Serial.println(gxdp1);
  //gxdp0 = gxdp1;

  Xangle += imu1.getRotationX() * 0.0000611;
  Serial.println(Xangle);
  //move the servo to get the angle back to 0
  
  

  blinkstate = !blinkstate;
  digitalWrite(LED_PIN, blinkstate);
}
