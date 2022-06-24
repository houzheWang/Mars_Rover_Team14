#include <Arduino.h>
#include <HardwareSerial.h>
#include "SPI.h"
#include <Robojax_L298N_DC_motor.h>



#define PIN_SS        5
#define PIN_MISO      19
#define PIN_MOSI      23
#define PIN_SCK       18

#define PIN_MOUSECAM_RESET     35
#define PIN_MOUSECAM_CS        5

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c 
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17

#define CHA 0

#define ENA 32 // this pin must be PWM enabled pin if Arduino board is used

#define IN1 25

#define IN2 33

// left settings

#define IN3 26

#define IN4 27

#define ENB 4 // this pin must be PWM enabled pin if Arduino board is used

#define CHB 1
const int CCW = 2; // do not change
const int CW  = 1; // do not change
#define motor1 1 // do not change
#define motor2 2 // do not change

Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB);




int total_x = 0;
int total_y = 0;


int total_x1 = 0;
int total_y1 = 0;


int x=0;
int y=0;

int a=0;
int b=0;

int distance_x=0;
int distance_y=0;

volatile byte movementflag=0;
volatile int xydat[2];


int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }


int tdistance = 0;



//pid control

int v=0;







void mousecam_reset()
{
  digitalWrite(PIN_MOUSECAM_RESET,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET,LOW);
  delay(35); // 35ms from reset to functional
}


int mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);

  digitalWrite(PIN_MOUSECAM_CS,HIGH);

  mousecam_reset();
  return 1;
}

void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(1);
  return ret;
}

struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};


void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(5);
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(byte *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);

  digitalWrite(PIN_MOUSECAM_CS, LOW);

  SPI.transfer(ADNS3080_PIXEL_BURST);
  delayMicroseconds(50);

  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if(started==0)
    {
      if(pix&0x40)
        started = 1;
      else
      {
        timeout++;
        if(timeout==100)
        {
          ret = -1;
          break;
        }
      }
    }
    if(started==1)
    {
      pdata[count++] = (pix & 0x3f)<<2; // scale to normal grayscale byte range
    }
  }

  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(14);

  return ret;
}

void setup()
{ 
  Serial.begin(9600);
  robot.begin();

  pinMode(PIN_SS,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);
  pinMode(PIN_SCK,OUTPUT);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);


    if(mousecam_init()==-1)
  {
    Serial.println("Mouse cam failed to init");
    while(1);
  }

}

char asciiart(int k)
{
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4];
}

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];


//pid control--------------------------------------------------------------------------------------------------------------------------------
class PID_position
{
private:
  float kp;
  float ki;
  float kd;
  float target;
  float actual;
  float e;
  float e_pre;
  float integral;
public:
  PID_position();
  ~PID_position(){};
  PID_position(float p,float i, float d);
  float pid_control(float tar, float act);
  void pid_show();
};


PID_position::PID_position():kp(0),ki(0),kd(0),target(0),actual(0),integral(0)
{  
  e=target-actual;
  e_pre = e;
}

PID_position::PID_position(float p, float i, float d):kp(p),ki(i),kd(d),target(0),actual(0),integral(0)
{
  e=target-actual;
  e_pre=e;
}

float PID_position::pid_control(float tar, float act)
{
  float u;
  target=tar;
  actual=act;
  e=target-actual;
  integral+=e; //integral=integral+e
  u=kp*e+ki*integral+kd*(e-e_pre);
  e_pre=e;
  return u;
}

void PID_position::pid_show()
{
  Serial.print("the coefficients are");
  Serial.println("kp="+ String(kp));
  Serial.println("ki="+ String(ki));
  Serial.println("kd="+ String(kd));
  Serial.println("integral="+ String(integral));
  Serial.println("target="+ String(target));
  Serial.println("actual="+ String(actual));
  Serial.println("e="+ String(e));
  Serial.println("e_pre="+ String(e_pre));
}



PID_position pid1(0.1,0.05,0);
float target=0;
float actual;
float u;



//sensor------------------------------------------------------------------------------------------------------------------------
void sensor(){
  #if 0
/*
    if(movementflag){

    tdistance = tdistance + convTwosComp(xydat[0]);
    Serial.println("Distance = " + String(tdistance));
    movementflag=0;
    delay(3);
    }

*/
  // if enabled this section grabs frames and outputs them as ascii art

  if(mousecam_frame_capture(frame)==0)
  {
    int i,j,k;
    for(i=0, k=0; i<ADNS3080_PIXELS_Y; i++)
    {
      for(j=0; j<ADNS3080_PIXELS_X; j++, k++)
      {
        Serial.print(asciiart(frame[k]));
        Serial.print(' ');
      }
      Serial.println();
    }
  }
  Serial.println();
  delay(250);

  #else

  // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
  // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.

  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);
  for(int i=0; i<md.squal/4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val*100)/351);
  Serial.print(' ');
  Serial.print(md.shutter); Serial.print(" (");
  Serial.print((int)md.dx); Serial.print(',');
  Serial.print((int)md.dy); Serial.println(')');

  // Serial.println(md.max_pix);
  delay(100);


    distance_x = convTwosComp(md.dx);// md.dx; //
    distance_y = convTwosComp(md.dy);//md.dy; //

total_x1 = total_x1 + distance_x;
total_y1 = total_y1 + distance_y;

total_x = total_x1/157;
total_y = total_y1/157;


if(v%2 == 0){
  distance_x = actual ;
  u=pid1.pid_control(target,distance_x);
}
else if(v%2==1){
  distance_y = actual;
  u=pid1.pid_control(target,distance_y);
}



Serial.print('\n');

//Serial.println("Distance_x = " + String(total_x));
//Serial.println("Distance_y = " + String(total_y));
Serial.println("distance_moved_x = " + String(distance_x));
Serial.println("distance_moved_y = " + String(distance_y));
Serial.println("coordinate_x1 = " + String(total_x1));
Serial.println("coordinate_y1 = " + String(total_y1));
Serial.println("error=" + String(target - actual));
Serial.println("actual=" + String(actual));
Serial.println("pid output=" + String(u));
Serial.println("v=" + String(v));

Serial.print('\n');

  delay(250);

  #endif
 
}

//motor--------------------------------------------------------------------------------------------------------------------------------
void STOP(){
  robot.brake(1);
  robot.brake(2);
  delay(500);
}

void FORWARD(){
  robot.rotate(motor1, 90, CW);
  robot.rotate(motor2, 80 + u, CCW);
  delay(250);
}

void BACKWARD(){
  robot.rotate(motor1, 70, CCW);
  robot.rotate(motor2, 70 + u, CW);
  delay(500);
}

void LEFTTURN90(){
  v = v+1;
  robot.rotate(motor1, 0, CCW);
  robot.rotate(motor2, 70, CCW);
  delay(1100);
  
}


void RIGHTTURN90(){
  v = v+1;
  robot.rotate(motor1, 70, CW);
  robot.rotate(motor2, 0, CW);
  delay(1180);

}

void TURNAROUND(){
  robot.rotate(motor1, 70, CW);
  robot.rotate(motor2, 70, CW);
  delay(1200);
}



void gostraight(){
  sensor();
  FORWARD();
}

void turnleft(){
  LEFTTURN90();
  sensor();
}

void turnright(){
  RIGHTTURN90();
  sensor();
}

void Duration_moving(int t){
  for(int i=0; i<t; i++){
    gostraight();
  }
}

void Duration_braking(int t1){
  for(int i=0; i<t1; i++){
    STOP();
  }
}

//loop--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{


  Duration_moving(10);
  Duration_braking(2);
  turnleft();
  Duration_braking(2);
//  Duration_moving(1);
//  Duration_braking(2);
  turnleft();
  Duration_braking(2);
//  
  Duration_moving(10);
  Duration_braking(2);
  turnright();
  Duration_braking(2);
//  Duration_moving(1);
//  Duration_braking(2);
  turnright();
  Duration_braking(2);
  
  

  // // move straight 
//   robot.rotate(motor1, 60, CW);
//   robot.rotate(motor2, 60-output, CCW);
//   delay(1000);
//   robot.brake(1);
//   robot.brake(2);  
//   delay(500);

  // // rotate 
//   robot.rotate(motor1, 70, CW);
//   robot.rotate(motor2, 70, CW);
//   delay(1700);
//   robot.brake(1);
//   robot.brake(2);   
//   delay(500);
  
//  #if 0
///*
//    if(movementflag){
//
//    tdistance = tdistance + convTwosComp(xydat[0]);
//    Serial.println("Distance = " + String(tdistance));
//    movementflag=0;
//    delay(3);
//    }
//
//*/
//  // if enabled this section grabs frames and outputs them as ascii art
//
//  if(mousecam_frame_capture(frame)==0)
//  {
//    int i,j,k;
//    for(i=0, k=0; i<ADNS3080_PIXELS_Y; i++)
//    {
//      for(j=0; j<ADNS3080_PIXELS_X; j++, k++)
//      {
//        Serial.print(asciiart(frame[k]));
//        Serial.print(' ');
//      }
//      Serial.println();
//    }
//  }
//  Serial.println();
//  delay(250);
//
//  #else
//
//  // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
//  // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.
//
//  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
//  MD md;
//  mousecam_read_motion(&md);
//  for(int i=0; i<md.squal/4; i++)
//    Serial.print('*');
//  Serial.print(' ');
//  Serial.print((val*100)/351);
//  Serial.print(' ');
//  Serial.print(md.shutter); Serial.print(" (");
//  Serial.print((int)md.dx); Serial.print(',');
//  Serial.print((int)md.dy); Serial.println(')');
//
//  // Serial.println(md.max_pix);
//  delay(100);
//
//
//    distance_x = convTwosComp(md.dx);// md.dx; //
//    distance_y = convTwosComp(md.dy);//md.dy; //
//
//total_x1 = total_x1 + distance_x;
//total_y1 = total_y1 + distance_y;
//
//total_x = total_x1/157;
//total_y = total_y1/157;
//
//
//Serial.print('\n');
//
//
//Serial.println("Distance_x = " + String(total_x));
//
//Serial.println("Distance_y = " + String(total_y));
//
//Serial.println("distance_moved_x = " + String(distance_x));
//Serial.println("distance_moved_y = " + String(distance_y));
//Serial.println("coordinate_x1 = " + String(total_x1));
//Serial.println("coordinate_y1 = " + String(total_y1));
//Serial.println("error=" + String(error));
//Serial.println("pid output=" + String(output));
//
//Serial.print('\n');
//
//if(v%2 == 0){
//  error = distance_x - setpoint;
//  myPID.Compute();
//}
//else if(v%2==1){
//  error = distance_y - setpoint;
//  myPID.Compute();
//}
//
//
//
//
//
//
//String dataTocontrol = String(distance_x) + ',' + String(distance_y);
//
//Serial1.print(dataTocontrol);
//Serial.print("Data to control -- success! ");
//  delay(100);
//
//  #endif
 
}
