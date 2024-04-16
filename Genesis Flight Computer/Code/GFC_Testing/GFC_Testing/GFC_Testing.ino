#include "MPU6050.h" //The MPU6050 library
#include "Wire.h"    //I2C library
#include "SPI.h"     //SPI library
#include "math.h"    //needed to do more compelx maths like sine, cosine, and tangent
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
int noseconeDeployedServoAngle = 60;

//Timing variables
float refresh = 50; //this is in hz
float deltaT = 1/refresh;
long loop_timer;

//-------------------------------------------------------------------------------------PID variables
float pGain = 200; //
float iGain = 0;
float dGain = 100;

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
float acceptableMargin = 1; //this is the accetable deviation from 0 in degrees (+ and -)
float baroReading;
float tempReading;

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
  if (!SD.begin(SDCS)) {Serial.println("Card failed, or not present"); sadBeeps();}
  else {Serial.println("SD Card initialized.");}
  File myFile = SD.open("data.txt", FILE_WRITE);
  myFile.println("File Created");
  myFile.close();
  //barometer
  if (!bmp.begin()) {Serial.println("Barometer Failed."); sadBeeps();}
  else {Serial.println("Barometer initialized.");}
  
  //define outputs and inputst.
  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(arm, INPUT_PULLUP);

  //attach Servo's to their respective pins
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

  //We need to calibrate the IMU before we can effectively use it. Here we take many readings and averge them to get offset values
  for(int i = 1; i <= calibrationPoints; i++)
    {
      gx = imu1.getRotationX();            
      total = total + gx;   
    }
    //when the loop is finished, calculate the average of all the points and set it as the gyro X offset
    gxOffset = total/calibrationPoints;
  Serial.print("Gyro X Offset Value: "); Serial.println(gxOffset);
  total = 0;

  for(int i = 1; i <= calibrationPoints; i++)
    {
      gz = imu1.getRotationZ();            
      total = total + gz;    
    }
    //when the loop is finished, calculate the average of all the points and set it as the gyro Z offset
    gzOffset = total/calibrationPoints;
  Serial.print("Gyro Z Offset Value: "); Serial.println(gzOffset);
  total = 0;

  for(int i = 1; i <= calibrationPoints; i++)
    {
      gy = imu1.getRotationY();            
      total = total + gy;    
    }
    //when the loop is finished, calculate the average of all the points and set it as the gyro Z offset
    gyOffset = total/calibrationPoints;
  Serial.print("Gyro Y Offset Value: "); Serial.println(gyOffset);
  total = 0;

 //the arming key must be inserted for the program to continue
  Serial.println("Waiting on arming key");
  while (digitalRead(arm) == HIGH)
  {
    digitalWrite(blueLED, HIGH);
    armed = true;
  }
   //notify the user when the key is inserted
  /*Serial.println("Arming key present");
  digitalWrite(blueLED, LOW);
  armed = false;
  delay(100);
  //now that the arming key is inserted, we wait until it is removed to arm the vehicle and begin data logging
  while(digitalRead(arm) == LOW)
  { }
  
  once the key is removed, notify the user with some lights and beeps, and wait 10 seconds for the vehicle to settle
  before taking accel movements to determine orientation.*/
  Serial.println("Vehicle Armed.");
  happyBeeps();
  armed = true;
  inFlight = false;
  delay(5000);

  /*once the vehicle is settled, use the accelerometers on the IMU to determine the current orientation of the vehicle 
  and assign them to the X Y and Z angle values. This will be the starting point for the control to kick in once the vehicle starts moving. */
  ax = (imu1.getAccelerationX() - axOffset)/accelScaleFactor;
  ay = (imu1.getAccelerationY() - ayOffset)/accelScaleFactor; //(this should equal 1 if the rocket is pointed up and level)
  az = (imu1.getAccelerationZ() - azOffset)/accelScaleFactor;
  xAngle = ((180/3.1415)*acos(ax/1)-90)*-1;
  zAngle = ((180/3.1415)*acos(az/1)-90)*-1;
  Serial.print(xAngle); Serial.print("  "); Serial.print(zAngle); Serial.print("  ");
  
  //beep loudly to indicate startup
  digitalWrite(buzzer, HIGH);
  delay(2000);
  digitalWrite(buzzer, LOW);
  
}

//---------------------------------------------------------------LOOP---------------------------------------------------------------
void loop() 
{
  loop_timer = millis();

  //read the angular velocities from the sensor
  //gx = imu1.getRotationX();
  //gz = imu1.getRotationZ();

  //adjust these values using the scale factor from the data sheet
  angularVelX = (imu1.getRotationX() - gxOffset)/gyroScaleFactor;
  angularVelZ = (imu1.getRotationZ() - gzOffset)/gyroScaleFactor;

  //convert the angular velocities into angular positions
  xAngle = angularVelX*deltaT + xAngle; 
  zAngle = angularVelZ*deltaT + zAngle;
  Serial.print("X:"); Serial.print(xAngle); Serial.print(" | "); Serial.print("Z:"); Serial.println(zAngle);

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
  xTVCAngle = xTVCAngle - (proportionalXError*pGain + integralXError*iGain + derivativeXError*dGain);
  zTVCAngle = zTVCAngle - (proportionalZError*pGain + integralZError*iGain + derivativeZError*dGain);
  
  

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
  //if(xAngle < acceptableMargin || xAngle > 90-acceptableMargin*-1)
  {
    //servoX.write(90);
  }
  //else
  {
    servoX.write(xTVCAngle);
  }

  //if(zAngle < 90+acceptableMargin || zAngle > 90-acceptableMargin*-1)
  {
    //servoZ.write(90);
  }
 // else
  {
    servoZ.write(zTVCAngle);
  }

  //update the errors in preparation for the next loop
  previousXError = currentXError;
  previousZError = currentZError;

  //Datalogging
  String dataString = "";
  baroReading = bmp.readPressure();                       //Serial.print(baroReading); Serial.print(" Pa"); Serial.print(" - "); 
  tempReading = bmp.readTemperature();                    //Serial.print(tempReading); Serial.print(" C"); Serial.print(" - ");
  ax = imu1.getAccelerationX()/accelScaleFactor;          //Serial.print("X: "); Serial.print(ax); Serial.print("g"); Serial.print("  ");
  ay = imu1.getAccelerationY()/accelScaleFactor;          //Serial.print("Y: "); Serial.print(ay); Serial.print("g"); Serial.print("  ");//(this should equal 1 if the rocket is pointed up and level)
  az = imu1.getAccelerationZ()/accelScaleFactor;          //Serial.print("Z: "); Serial.print(az); Serial.print("g"); Serial.print(" - ");
  gx = (imu1.getRotationX() - gxOffset)/gyroScaleFactor;  //Serial.print("X: "); Serial.print(gx); Serial.print("deg/sec"); Serial.print("  ");
  gy = (imu1.getRotationY() - gyOffset)/gyroScaleFactor;  //Serial.print("Y: "); Serial.print(gy); Serial.print("deg/sec"); Serial.print("  ");
  gz = (imu1.getRotationZ() - gzOffset)/gyroScaleFactor;  //Serial.print("Z: "); Serial.print(gy); Serial.print("deg/sec"); Serial.print("  ");
  dataString = String(millis()) + ", " + String(baroReading) + ", " + String(tempReading) + ", " + String(ax) + ", " + String(ay) + ", " + String(az) + ", " + String(gx) + ", " + String(gy) + ", " + String(gz) + ", " + String(currentXError)+ ", " + String(currentZError);
  //Serial.println(dataString);
  
  //record that data to the SD card
  //File myFile = SD.open("data.txt", FILE_WRITE);
  //myFile.println(dataString);
  //myFile.close();
  while(millis() < loop_timer + 1000/refresh); {} 

}

void happyBeeps()
{
  digitalWrite(greenLED, HIGH);
  tone(buzzer, happyTone);
  delay(100);
  noTone(buzzer);
  delay(100);
  digitalWrite(greenLED, LOW);
  tone(buzzer, happyTone);
  delay(250);
  noTone(buzzer);
}
void neutralBeeps()
{
  digitalWrite(blueLED, HIGH);
  tone(buzzer, neutralTone);
  delay(100);
  noTone(buzzer);
  delay(100);
  digitalWrite(blueLED, LOW);
  tone(buzzer, neutralTone);
  delay(250);
  noTone(buzzer);
}
void sadBeeps()
{
  digitalWrite(redLED, HIGH);
  tone(buzzer, sadTone);
  delay(100);
  noTone(buzzer);
  delay(100);
  digitalWrite(redLED, LOW);
  tone(buzzer, sadTone);
  delay(250);
  noTone(buzzer);
}