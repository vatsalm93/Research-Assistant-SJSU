#include "Keyboard.h"

int Pin1 = 9;      // LED connected to digital pin 9
int Pin2 = 11;
int Pin3 = 4;
int Pin4 = 5;
//int analogPin = 3;   // potentiometer connected to analog pin 3
int val1 = 660; //680        // variable to store the read value
int val2 = 665;
void setup()
{
  pinMode(Pin1, OUTPUT);   // sets the pin as output
  pinMode(Pin2, OUTPUT);
  Serial.begin(9600);
}

void loop()
{

  /*
  if(b < 500){
    val1++;
    }
  else if(b>501){
    val1--;
    }
   if(c<500){
    val2++;
    }
    else if(c>501){
      val2--;
      }
    */


  
  if(Serial.available())
  {
  char inChar = Serial.read();
      switch(inChar)
      {
        case('w'): val1=val1+30;
        if(val1>760)
        {
          val1=760;
        }
        break;
         case('s'): val1=val1-30;
        if(val1<550)
        {
          val1=550;
        }
        break;
        case('a'):val2=val2-10;
        if(val2<550)
        {
          val2=550;
        }
        break;
        case('d'):val2=val2+10;
        if(val2>710)
        {
          val2=710;
        }
        break;
      }
  }

   Serial.print(val1);
   Serial.print(":");
   Serial.println(val2);
   analogWrite(Pin1, val1);
  analogWrite(Pin2,val2);
}
