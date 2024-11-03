#define servo_PIN1 4
#define servo_PIN2 1
#define x_length 60//正方形的边长
#define y_length 60

// 修正安装角度的系统误差
#define ANGLE_FIX1 102-90       // if you want 90, you should write 102
#define ANGLE_FIX2 102-90

#define H 170
#define L 4
//#define Pot_Pin A0
#include <Arduino.h>
#include <ESP32Servo.h>
// #include <Servo.h>
#include <math.h>
Servo servo1;//垂直方向
Servo servo2;//水平方向
void setup() {
  // put your setup code here, to run once:
  //pinMode(servo_PIN,OUTPUT);
  Serial.begin(9600);
  //pinMode(Pot_Pin,INPUT);
  servo1.attach(servo_PIN1);
  servo2.attach(servo_PIN2);

  // debug code for calibrate the angle
  while( 0 ) {
    String s = Serial.readStringUntil('\r');
    int a1,a2;
    if( 2 == sscanf(s.c_str(), "%d,%d", &a1, &a2)) {
      servo1.write(a1);
      servo2.write(a2);
    }
  }
  // delay(1000000);
  
}

void loop() {
  double angle1,angle2;//1为垂直方向
  int stage = 1;
  double x = -x_length,y = y_length+150;

  double des_x,des_y;

  des_x = x_length;
  des_y = y_length +150;
  move(x,y,angle1,angle2,des_x,des_y);
  des_x = x_length;
  des_y = -y_length +150;
  move(x,y,angle1,angle2,des_x,des_y);
  des_x = -x_length;
  des_y = -y_length +150;
  move(x,y,angle1,angle2,des_x,des_y);
  des_x = -x_length;
  des_y = y_length +150;
  move(x,y,angle1,angle2,des_x,des_y);
}
  // angle1 =180-(atan(sqrt(x*x+y*y)/h)*180)/M_PI;
  // Serial.println(angle1);
  // angle2 = ((atan(x/y)*180)/M_PI)+90;
  // //Serial.println(angle2);
  // servo1.write(angle1);
  // servo2.write(angle2);
  // x = x + 1;
  // delay(500);
  
  

  // put your main code here, to run repeatedly:
  //int pot = analogRead(Pot_Pin);
  //int anglenum = map(pot,0,1023,0,180);
  //Serial.println(anglenum);

  //for(int i=0;i<=50;i++){
    //PWMServo(anglenum);

  //}

int move(double &x,double &y,double &angle1,double &angle2,double des_x,double des_y){
  while(1){
    if(des_x!=x){
      if(des_x>x){
        x+=1;
      }
      if(des_x<x){
        x-=1;
      }
    }
    if(des_y!=y){
      if(des_y>y){
        y+=1;
      }
      if(des_y<y){
        y-=1;
      }
    }
    if(x==des_x&&y==des_y){
      break;
    }
    angle1 =180- (atan(sqrt(x*x+y*y)/H)*180/M_PI) - (acos(L/sqrt(x*x+y*y+H*H))*180/M_PI) + 90;
    angle2 = ((atan(x/y)*180)/M_PI)+90;
    Serial.println(String(angle1) + "," + String(angle2));
    servo1.write( angle1 + ANGLE_FIX1);
    servo2.write( angle2 + ANGLE_FIX2);
    delay(10);
  }
  return 0; 
}

// void PWMServo(int angleArg){
//   if(angleArg<0) angleArg = 0;
//   if(angleArg>180) angleArg = 180;
//   int PWMWidth = (angleArg * 11) + 500;
//   digitalWrite(servo_PIN,HIGH);
//   delayMicroseconds(PWMWidth);
//   digitalWrite(servo_PIN,LOW);
//   delayMicroseconds(20000-PWMWidth);
// }
