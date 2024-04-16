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

//tone variables for the piezo buzzer
const int happyTone = 3300;
const int neutralTone = 3000;
const int sadTone = 2000;

//data management variables
float baroReading;
float tempReading;

//---------------------------------------------------------------SETUP---------------------------------------------------------------
void setup() 
{
  //define outputs and inputst.
  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(arm, INPUT_PULLUP);

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
   Serial.println("initializing components. Please do not touch the vehicle.");

  //IMU
  imu1.initialize();
  imu1.setFullScaleGyroRange(FS_SEL);
  imu1.setFullScaleAccelRange(AFS_SEL);
  Serial.println("IMU Initialized");

  //SD Card
  if (!SD.begin(SDCS)) {Serial.println("Card failed, or not present"); sadBeeps(); while (1) delay(10);}
  else{Serial.println("SD Card initialized.");}

  //Flash SD
  //if (!SD.begin(FlashCS)) {Serial.println("Flash SD failed.");sadBeeps(); while (1) delay(10);}
  //else{Serial.println("Flash SD initialized.");}

  //Barometer
  if (!bmp.begin()) {Serial.println("Barometer failed."); sadBeeps(); while (1) delay(10); }
  else{Serial.println("Barometer initialized.");}
  
  neutralBeeps();

  //We need to calibrate the IMU before we can effectively use it. Here we take many readings and averge them to get offset values
  Serial.println("Calibrating IMU. Please do not touch the vehicle.");
  for(int i = 1; i <= calibrationPoints; i++)
    {gx = imu1.getRotationX(); total = total + gx;}
    //when the loop is finished, calculate the average of all the points and set it as the gyro X offset
    gxOffset = total/calibrationPoints;
  Serial.print("Gyro X Offset Value: "); Serial.println(gxOffset); total = 0;

  for(int i = 1; i <= calibrationPoints; i++)
    {gz = imu1.getRotationZ(); total = total + gz;}
    //when the loop is finished, calculate the average of all the points and set it as the gyro Z offset
    gzOffset = total/calibrationPoints;
  Serial.print("Gyro Z Offset Value: "); Serial.println(gzOffset); total = 0;

  for(int i = 1; i <= calibrationPoints; i++)
    {gy = imu1.getRotationY(); total = total + gy;}
    //when the loop is finished, calculate the average of all the points and set it as the gyro Z offset
    gyOffset = total/calibrationPoints;
  Serial.print("Gyro Y Offset Value: "); Serial.println(gyOffset); total = 0;

  /*once the vehicle is settled, use the accelerometers on the IMU to determine the current orientation of the vehicle 
  and assign them to the X Y and Z angle values. This will be the starting point for the control to kick in once the vehicle starts moving. */
  ax = imu1.getAccelerationX()/accelScaleFactor;
  ay = imu1.getAccelerationY()/accelScaleFactor; //(this should equal 1 if the rocket is pointed up and level)
  az = imu1.getAccelerationZ()/accelScaleFactor;
  xAngle = ((180/3.1415)*acos(ax/1)-90)*-1;
  zAngle = ((180/3.1415)*acos(az/1)-90)*-1;
  Serial.print(xAngle); Serial.print("  "); Serial.print(zAngle); Serial.println("  ");
  
  File myFile = SD.open("test.txt", FILE_WRITE);
  myFile.println("File created");
  myFile.close();
 
  happyBeeps();

  
  for(int i = 0; i<5; i++)
  {
  String dataString = "";

  //read data from barometer, accelerometers, and gyroscopes.
  baroReading = bmp.readPressure();                       //Serial.print(baroReading); Serial.print(" Pa"); Serial.print(" - "); 
  tempReading = bmp.readTemperature();                    //Serial.print(tempReading); Serial.print(" C"); Serial.print(" - ");
  ax = imu1.getAccelerationX()/accelScaleFactor;          //Serial.print("X: "); Serial.print(ax); Serial.print("g"); Serial.print("  ");
  ay = imu1.getAccelerationY()/accelScaleFactor;          //Serial.print("Y: "); Serial.print(ay); Serial.print("g"); Serial.print("  ");//(this should equal 1 if the rocket is pointed up and level)
  az = imu1.getAccelerationZ()/accelScaleFactor;          //Serial.print("Z: "); Serial.print(az); Serial.print("g"); Serial.print(" - ");
  gx = (imu1.getRotationX() - gxOffset)/gyroScaleFactor;  //Serial.print("X: "); Serial.print(gx); Serial.print("deg/sec"); Serial.print("  ");
  gy = (imu1.getRotationY() - gyOffset)/gyroScaleFactor;  //Serial.print("Y: "); Serial.print(gy); Serial.print("deg/sec"); Serial.print("  ");
  gz = (imu1.getRotationZ() - gzOffset)/gyroScaleFactor;  //Serial.print("Z: "); Serial.print(gy); Serial.print("deg/sec"); Serial.print("  ");
  dataString = String(millis()) + ", " + String(baroReading) + ", " + String(tempReading) + ", " + String(ax) + ", " + String(ay) + ", " + String (az) + ", " + String (gx) + ", " + String (gy) + ", " + String (gz);
  Serial.println(dataString);
  
  //record that data to the SD card
  File myFile = SD.open("test.txt", FILE_WRITE);
  myFile.println(dataString);
  myFile.close();
  
  
  
  }
  
  Serial.println("Data recording finished");

  Serial.println("Reading back the data");
  delay(2000);
  myFile = SD.open("test.txt");
  while(myFile.available())
  {Serial.write(myFile.read());}
  myFile.close();
}

void loop()

{
 

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
