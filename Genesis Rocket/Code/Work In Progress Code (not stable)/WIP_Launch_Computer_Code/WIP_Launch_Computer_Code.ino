/*
RESOURCES:
4 digt 7 segment display with TM1647 driver: https://www.makerguides.com/tm1637-arduino-tutorial/

*/

#include <TM1637Display.h>
#include <Wire.h>

const int relay = 2;
const int buzzer = 3;
const int safetySwitch = 4;
const int startButton = 5;
const int DIO = 6;
const int CLK = 7;
int countdownStartingPoint= 10; //seconds to start the countdown at
int countdown = 0;
bool blinkstate = true;
bool readystate = false;
bool launching = false;

// create a display object of type TM1637Display
TM1637Display display = TM1637Display(CLK, DIO);

//check pin function
bool CheckPin(int pinNumber)
{
  if(digitalRead(pinNumber) == HIGH)
  {return true;}
  else
  {return false;}
}

//Flash numbers to display function
void flashNumbers(int number, int interval, int qty)
{
  for(int i; i < qty; i++)
  {
  display.showNumberDecEx(number, 0b11100000, false, 4, 0);
  delay(interval);
  display.clear();
  delay(interval);
  }
}

void setup() 
{
  display.clear();
  display.setBrightness(7); // set the brightness to 7 (0:dimmest, 7:brightest)
  pinMode(relay, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(safetySwitch, INPUT);
  pinMode(startButton, INPUT);
}

void loop() 
{
  //check to see if the safety switch is flipped
  if(CheckPin(safetySwitch) == true)
  {
    //if it is, then we are ready for launch. Flash 8's and sound the buzzer (the led should also show up red)
    display.showNumberDecEx(8888, 0b11100000, false, 4, 0);
    //digitalWrite(buzzer, HIGH);
    readystate = true;
    if(CheckPin(startButton) == true)
    {
      flashNumbers(5555, 50, 20);
      launching = true;
    }
  }
  else
  {//if it isn't we are not ready for launch and the buzzer should be off
    readystate = false;
    launching = false;
    countdown = countdownStartingPoint;
    digitalWrite(buzzer, LOW);
    display.clear();
  }
 
  //the loop to execute while in the countdown
  while(launching == true && CheckPin(safetySwitch) == true)
  {
    display.showNumberDec(countdown); //display the countdown on the display
    
    //turn the buzzer on and off every second
    if (blinkstate == true)
    {
      digitalWrite(buzzer, HIGH);
    }
    else
    {
      digitalWrite(buzzer, LOW);
    }

    //wait 1 second
    delay(1000);
    
    //if the countdown has concluded, start the engine
    if (countdown == 0)
      {
        digitalWrite(buzzer, HIGH);
        digitalWrite(relay, HIGH);
        flashNumbers(8888, 1000, 10);
      }

      countdown = countdown-1;
  }

  
  
  


}
