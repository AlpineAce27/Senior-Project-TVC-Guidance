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
const int Servo2Pin =   3;
const int Servo3Pin =   4;
const int Servo4Pin =   5;
const int buzzer =      6;
const int blueLED =     7;
const int redLED =      8;
const int greenLED =    9;
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
const int arm =         23;



//---------------------------------------------------------------VARIABLES---------------------------------------------------------------

//Flight Logic Variables
bool armed = false;
bool inFlight = false;
bool landed = false;
bool dataTransferred = false;

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
float zAngle = 0;

int maximumXdeflection = 25;
int maximumZdeflection = 25;
int xTVCAngle = 90;
int zTVCAngle = 90;
int noseconeServoAngle = 90;

//Timing variables
float refresh = 50; //this is in hz
float deltaT = 1/refresh;
long loop_timer;

//PID variables
float pGain = 50; //
float iGain = 0;
float dGain = 0;

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
float zAngleDesired = 0;
float acceptableMargin = 1;

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
  
  //------------------------Initialize Components------------------
  Wire.begin();
  SPI.begin();
  Serial.begin(38400);
  
  //IMU
  imu1.initialize();
  imu1.setFullScaleGyroRange(FS_SEL);
  imu1.setFullScaleAccelRange(AFS_SEL);
  Serial.println("IMU Initialized");



  //SD Card
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
  else
  {Serial.println("SD Card initialized.");}

  //Flash SD
  if (!SD.begin(FlashCS)) 
    {
    Serial.println("Flash SD failed.");
    {
      digitalWrite(redLED, HIGH);
      tone(buzzer, sadTone);
      delay(100);
      digitalWrite(redLED, LOW);
      noTone(buzzer);
      delay(3000);
    }
  }
  else
  {Serial.println("Flash SD initialized.");}
  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  else
  {Serial.println("Barometer initialized.");}
  
  //define outputs and inputst.
  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(arm, INPUT_PULLUP);

  servoX.attach(Servo1Pin);
  servoX.write(xTVCAngle);
  servoZ.attach(Servo2Pin);
  servoZ.write(zTVCAngle);

  digitalWrite(greenLED, HIGH);
  tone(buzzer, happyTone);
  delay(500);
  noTone(buzzer);
  delay(500);
  tone(buzzer, happyTone);
  delay(100);
  noTone(buzzer);
  delay(100);
  tone(buzzer, happyTone);
  delay(100);
  noTone(buzzer);
  digitalWrite(greenLED, LOW);

/*
  //the arming key must be inserted for the program to continue
  Serial.println("Waiting on arming key");
  while (digitalRead(arm) == HIGH)
  {
    digitalWrite(blueLED, HIGH);
    armed = true;
  }

  //notify the user when the key is inserted
  Serial.println("Arming key present");
  digitalWrite(blueLED, LOW);
  armed = false;
  delay(100);
  //now that the arming key is inserted, we wait until it is removed to arm the vehicle and begin data logging
  while(digitalRead(arm) == LOW)
  { }
*/

  /*once the key is removed, notify the user with some lights and beeps, and wait 10 seconds for the vehicle to settle
  before taking accel movements to determine orientation.*/
  Serial.println("Vehicle Armed.");
  digitalWrite(greenLED, HIGH);
  tone(buzzer, happyTone);
  delay(100);
  noTone(buzzer);
  delay(100);
  digitalWrite(greenLED, HIGH);
  armed = true;
  inFlight = false;
  delay(5000);

  //We need to calibrate the IMU before we can effectively use it. Here we take many readings and averge them to get offset values
  for(int i = 1; i <= calibrationPoints; i++)
    {
      gx = imu1.getRotationX();            
      total = total + gx;
      Serial.println(gx);    
    }
    //when the loop is finished, calculate the average of all the points and set it as the gyro X offset
    gxOffset = total/calibrationPoints;
  Serial.print("Gyro X Offset Value: "); Serial.println(gxOffset);
  total = 0;

  for(int i = 1; i <= calibrationPoints; i++)
    {
      gz = imu1.getRotationZ();            
      total = total + gz;
      Serial.println(gz);    
    }
    //when the loop is finished, calculate the average of all the points and set it as the gyro Z offset
    gzOffset = total/calibrationPoints;
  Serial.print("Gyro Z Offset Value: "); Serial.println(gzOffset);

  //TVC deflection test
  //while(1==1)
  {
  servoX.write(90+maximumXdeflection);
  delay(500);
  servoX.write(90-maximumXdeflection);
  delay(500);
  servoX.write(90);
  delay(500);
  servoZ.write(90+maximumZdeflection);
  delay(500);
  servoZ.write(90-maximumZdeflection);
  delay(500);
  servoZ.write(90);
  delay(500);
  servoX.write(90+maximumXdeflection);
  servoZ.write(90+maximumZdeflection);
  delay(500);
  servoX.write(90+maximumXdeflection);
  servoZ.write(90-maximumZdeflection);
  delay(500);
  servoX.write(90-maximumXdeflection);
  servoZ.write(90-maximumZdeflection);
  delay(500);
  servoX.write(90-maximumXdeflection);
  servoZ.write(90+maximumZdeflection);
  delay(500);
  servoX.write(90);
  servoZ.write(90);
  delay(1000);
  }
  /*once the vehicle is settled, use the accelerometers on the IMU to determine the current orientation of the vehicle 
  and assign them to the X Y and Z angle values. This will be the starting point for the control to kick in once the vehicle starts moving. */


  //beep loudly to indicate startup
  //digitalWrite(buzzer, HIGH);
  delay(2000);
  digitalWrite(buzzer, LOW);
  
}

//---------------------------------------------------------------LOOP---------------------------------------------------------------
void loop() 
{
  loop_timer = millis();

  //read the angular velocities from the sensor
  gx = imu1.getRotationX();
  gz = imu1.getRotationZ();

  //adjust these values using the scale factor from the data sheet
  angularVelX = (gx - gxOffset)/gyroScaleFactor;
  angularVelZ = (gz - gzOffset)/gyroScaleFactor;

  //convert the angular velocities into angular positions
  xAngle = angularVelX*deltaT + xAngle; 
  zAngle = angularVelZ*deltaT + zAngle;
  Serial.print(xAngle); Serial.print(" | "); Serial.println(zAngle);

  //determine the error between where we are, and where we want to be
  currentXError = xAngleDesired - xAngle;
  currentZError = zAngleDesired - zAngle;

  //apply this error into the PID components
  proportionalXError = currentXError;
  proportionalZError = currentZError;
  integralXError = integralXError + currentXError*deltaT;
  integralZError = integralZError + currentZError*deltaT;
  derivativeXError = (currentXError - previousXError)/deltaT;
  derivativeZError = (currentZError = previousZError)/deltaT;

  //Serial.print(proportionalXError); Serial.print(" | "); Serial.print(integralXError); Serial.print(" | "); Serial.print(derivativeXError); Serial.println(" | ");
  
  //multiply these errors by their repsective gains, add them togehter and write the new angle to the servo it to the servo
  xTVCAngle = xTVCAngle + proportionalXError*pGain/100 + integralXError*iGain/100 + derivativeXError*dGain/100;
  zTVCAngle = zTVCAngle + proportionalZError*pGain/100 + integralZError*iGain/100 + derivativeZError*dGain/100;
  
  

  //we dont want to stress the servos. for this reason we will put an effective maximum deflection value in, if the assigned value exceeds this maximum, we limit it
  
  if(xTVCAngle > 90+maximumXdeflection)
  {xTVCAngle = 90+maximumXdeflection;}
  if(xTVCAngle < 90-maximumXdeflection) 
  {xTVCAngle = 90-maximumXdeflection;}
  
  if(zTVCAngle > 90+maximumZdeflection)
  {zTVCAngle = 90+maximumZdeflection;}
  if(zTVCAngle < 90-maximumZdeflection) 
  {zTVCAngle = 90-maximumZdeflection;}

  //Serial.print(xTVCAngle); Serial.print(" | "); Serial.println(zTVCAngle);
  //if(zAngle < acceptableMargin || xAngle > acceptableMargin*-1)
  {
    //servoX.write(90);
  }
  //else
  {
    servoX.write(xTVCAngle);
  }
  //if(zAngle < acceptableMargin || zAngle > acceptableMargin*-1)
  {
    //servoZ.write(90);
  }
  //else
  {
    servoZ.write(zTVCAngle);
  }
  //update the errors in preparation for the next loop
  previousXError = currentXError;
  previousZError = currentZError;

  while(millis() < loop_timer + 1000/refresh); {} 

}
