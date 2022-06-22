#include <Wire.h>

#define SMOOTH_ITERATIONS 200
#define SMOOTH_DELAY_MICRO 100

#define NEG_90_DEG -76.5
#define POS_90_DEG 94.3

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265;
int maxVal=402;

double x;
double y;
double z;


void mpuSetup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}


double getAngle() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
  
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  // Make it signed
  if(x > 180)
    x -= 360;

  // Normalize
  if(x < 0)  x = map(x, NEG_90_DEG, 0, -90, 0);
  else       x = map(x, 0, POS_90_DEG, 0, 90);
  
  return x;
}

double getAngleSmooth(){
  double angle = 0;

  for(int i = 0; i < SMOOTH_ITERATIONS; i++){
    angle += getAngle();
    delayMicroseconds(SMOOTH_DELAY_MICRO);
  }

  angle = angle / SMOOTH_ITERATIONS;

  return angle;
}
