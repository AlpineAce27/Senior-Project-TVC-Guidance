#include "MPU6050.h" //The MPU6050 library
#include "Wire.h"    //I2C library
#include "SPI.h"     //SPI library
#include "SD.h"      //make sure this is the Teensyduino SD Card library
#include "Servo.h"  //make sure this is the Teensyduino Servo Card library
#include "Adafruit_BMP280.h" //BMP280 Barometer Library


//Objects: IMU, X and Z axis Servos, Aux Servos
MPU6050 imu1;
Adafruit_BMP280 bmp;
Servo servoX;
Servo servoZ;
Servo servoAux1;
Servo servoAux2;

//---------------------------------------------------------------PINS-------------------------------------------------------------------
const int Servo1Pin =   2;
const int Servo3Pin =   3;
const int Servo2Pin =   4;
const int Servo4Pin =   5;
const int buzzer =     6;
const int blueLED =  7;
const int redLED =   8;
const int greenLED = 9;
                      //11 is the MOSI pin    
                      //12 is the MISO pin
const int Aux14 =       14;
const int Aux15 =       15;
const int Aux16 =       16;
const int pyroPin =     17;
                      //18 is the SDA pin
                      //19 is the SCL pin
const int SDCS =        20;
const int FlashCS =     21;
const int Aux22 =       22;



//---------------------------------------------------------------VARIABLES---------------------------------------------------------------

//IMU gyro variables
float gx;
float gy;
float gz;
int gxOffset;
int gyOffset;
int gzOffset;
float gyroScaleFactor = 65.5; //This value is based off of the MPU6050 datasheet, pg 12. We are using FS_SEL = 1 for +- 500 deg/sec
int FS_SEL = 1;

//IMU accelerometer variables
float ax;
float ay;
float az;
int axOffset;
int ayOffset;
int azOffset;
float accelScaleFactor = 4096; //This value is based off of the MPU6050 datasheet, pg 13. We are using AFS_SEL=2, for +-8 G's
int AFS_SEL = 2;

//IMU Calibration variables
float average;
float total;
int calibrationPoints = 2000;

//Angle variables, one for the set angle and one for the observed/measured angle
float angularVelX = 0;
float angularVelY = 0;
float angularVelZ = 0;
float xAngle = 0;
float yAngle = 0;
float zAnlge = 0;

int xTVCAngle = 90;
int zTVCAngle = 90;
int noseconeServoAngle = 90;

//Timing variables
float refresh = 100; //this is in hz
float deltaT = 1/refresh;
long loop_timer;

//PID variables
float pGain = 0.5; //
float iGain = 0.0;
float dGain = 0.5;

//x axis PID components
float previousXError = 0;
float currentXError = 0;
float proportionalXError = 0;
float integralXError = 0;
float derivativeXError = 0;

//z axis PID components
float previousZError = 0;
float currentZError = 0;
float proportionalZError = 0;
float integralZError = 0;
float derivativeZError = 0;

//other variables
float xAngleDesired = 0;
float yAngleDesired = 0;

//tone variables for the piezo buzzer
const int happyTone = 3300;
const int neutralTone = 3000;
const int sadTone = 2000;

//---------------------------------------------------------------SETUP---------------------------------------------------------------
void setup() 
{
  //wake up sequence
  tone(buzzer, sadTone);
  delay(100);
  tone(buzzer, neutralTone);
  delay(100);
  tone(buzzer, happyTone);
  delay(100);
  noTone(buzzer);
  digitalWrite(redLED, HIGH);
  delay(500);                
  digitalWrite(redLED, LOW); 
  digitalWrite(blueLED, HIGH);
  delay(500);
  digitalWrite(blueLED, LOW); 
  digitalWrite(greenLED, HIGH);   
  delay(500);
  digitalWrite(greenLED, LOW);
  
  //initialize components
  Wire.begin();
  SPI.begin();
  Serial.begin(38400);
  
  imu1.initialize();
  imu1.setFullScaleGyroRange(FS_SEL);
  imu1.setFullScaleAccelRange(AFS_SEL);
  Serial.println("IMU Initialized");
  Serial.println("Calibrating");

  if (!SD.begin(SDCS)) 
    {
    Serial.println("Card failed, or not present");
    {
      digitalWrite(redLED, HIGH);
      tone(buzzer, sadTone);
      delay(100);
      digitalWrite(redLED, LOW);
      noTone(buzzer);
      delay(3000);
    }
  }
  Serial.println("card initialized.");
  //define outputs and inputst.
  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
}

// the loop() methor runs over and over again,
// as long as the board has power

//---------------------------------------------------------------LOOP---------------------------------------------------------------
void loop() 
{
  delay(100);

  
              
}

