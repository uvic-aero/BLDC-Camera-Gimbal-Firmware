
#include <Servo.h>

#define SERVOPIN (10)
float t = 0; // time, in ms
int dt = 10; // tie delta, in milliseconds
int y = 0;
float T = 5.0; //seconds
float omega = 2 * 3.14159 / T; // rad/s


Servo servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo.attach(SERVOPIN);
  servo.write(256/2);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
   y = (int)(255*(1.0 + cos(omega*(t/1000.0))) / 2.0);
   servo.write(y);
   Serial.println(y);

   if ( t >= 1000.0*T )
      t = 0.0;
   else
      t = t + (float)dt;
      
   delay(dt);
}
