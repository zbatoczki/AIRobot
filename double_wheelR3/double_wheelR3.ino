// Code tuned for robot 3
// modified: 10/03/2017, Houman Dallali
// COMP 590

#include <Servo.h>
// #define ROTATION 663 // According to manual
#define ROTATION_1 398
#define ROTATION_2 398

// encoder variables:

// encoder0
int encoder0PinA = 2;
int encoder0PinB = 3;
volatile int encoder0Pos = 0;
volatile int encoder0PinALast = LOW;
volatile int n = LOW;
int valNew = 0;
int valOld = 0;
volatile int m = LOW;

// encoder1
int encoder1PinA = 18;
int encoder1PinB = 19;
volatile int encoder1Pos = 0;
volatile int encoder1PinALast = LOW;
volatile int n1 = LOW;
int valNew1 = 0;
int valOld1 = 0;
volatile int m1 = LOW;

int loopCnt=0;
int Ts = 1; //1ms

Servo ST0, ST1; // We'll name the Sabertooth servo channel objects ST0 and ST1.
                // For how to configure the Sabertooth, see the DIP Switch Wizard for
                //   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                // Be sure to select RC Microcontroller Mode for use with this sample.
                //
                // Connections to make:
                //   Arduino Pin 9  ->  Sabertooth S1
                //   Arduino Pin 10 ->  Sabertooth S2
                //   Arduino GND    ->  Sabertooth 0V
                //   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
                //
                // Sabertooth accepts servo pulses from 1000 us to 2000 us.
                // We need to specify the pulse widths in attach(). 0 degrees will be full reverse, 180 degrees will be
                // full forward. Sending a servo command of 90 will stop the motor. Whether the servo pulses control
                // the motors individually or control throttle and turning depends on your mixed mode setting.

// Notice these attach() calls. The second and third arguments are important.
// With a single argument, the range is 44 to 141 degrees, with 92 being stopped.
// With all three arguments, we can use 0 to 180 degrees, with 90 being stopped.
void setup()
{
  ST0.attach(9, 1000, 2000);
  ST1.attach(10, 1000, 2000);

  // encoder0 setup
  pinMode (encoder0PinA,INPUT); 
  pinMode (encoder0PinB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), CountA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), StateB, FALLING);
  
   // encoder1 setup
  pinMode (encoder1PinA,INPUT); 
  pinMode (encoder1PinB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), CountA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), StateB1, FALLING); 
   
  Serial.begin (9600);

}

void loop()
{
  int power0, power1;
  int error0, error1;

    // proportional feedback gain:
    int kp=10;
        
    float turns0    = ((float)encoder0Pos)/ROTATION_1;
    float turns1    = ((float)encoder1Pos)/ROTATION_2;
    
    Serial.print(" T0: ");
    Serial.print(turns0, DEC);
    Serial.print(" T1 :");
    Serial.print(turns1, DEC);
    
    // desired no. of turns for motor0
    float desTurns0 = 1;
    error0=kp*(desTurns0-turns0); 
    // desired no. of turns for motor1
    
    float desTurns1 = 1;
    error1=kp*(desTurns1-turns1); 

    // Control of Motor 0
    if (error0 < -.2)
    {
      power0=60;
      //power0=map(error0,-10*kp, 10*kp, 10,80);
    }
    else if (error0 < .2)
    {
      power0 = 90;
    }
    else
    {
      power0=120;
      //power0=map(error0,-10*kp, 10*kp, 175,90);
    }
       
    // Control Calc for Motor 1
    if (error1 < -.2)
    {
      power1 = 60;
      //power1=map(error1,-10*kp, 10*kp, 100,120);
    }
    else if (error1 < .2)
    {
      power1 = 90;
    }
    else
    {
      power1=120;
      //power1=map(error1,-10*kp, 10*kp, 80,50);
    }

    // sending the control with PWM 
    ST0.write(90);
    // ST1.write(90);
    // ST0.write(power0);
    ST1.write(power1);
    
    delay(Ts/4);

  encoder0PinALast = n;
  valNew = encoder0Pos;
  if (valNew != valOld) {
    valOld = valNew;
  }

  encoder1PinALast = n1;
  valNew1 = encoder1Pos;
  if (valNew1 != valOld1) {
    valOld1 = valNew1;
  }
  
  // printing encoder values to serial monitor
  Serial.print(" Enc0 Pos: ");
  Serial.print (encoder0Pos, DEC);
  Serial.print(" Enc1 Pos: ");
  Serial.print (encoder1Pos, DEC); 
  Serial.print(" Error1: ");
  Serial.print(error1, DEC);
  Serial.print(" Power1: ");
  Serial.println(power1, DEC);
   
  loopCnt++;
}

void CountA()
{
  n = digitalRead(encoder0PinA); 
  if ((encoder0PinALast == LOW) && (n == HIGH)) { 
    if (m == LOW) { 
      encoder0Pos--; 
    } 
    else { 
      encoder0Pos++; 
    } 
  }
}

void StateB()
{
  m = digitalRead(encoder0PinB);
}


void CountA1()
{
  n1 = digitalRead(encoder1PinA); 
  if ((encoder1PinALast == LOW) && (n1 == HIGH)) { 
    if (m1 == LOW) { 
      encoder1Pos--; 
    } 
    else { 
      encoder1Pos++; 
    } 
  }
}

void StateB1()
{
  m1 = digitalRead(encoder1PinB);
}
