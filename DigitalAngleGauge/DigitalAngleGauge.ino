//Written by Ahmet Burkay KIRNIK
//TR_CapaFenLisesi
//Measure Angle with a MPU-6050(GY-521)

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include "MyMPU6050.h"

// OLED stuff
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

#define ZERO_BUTTON_PIN 3
#define ZERO_DELAY 500

Quaternion myAngle;
double myAngleDegrees;
Quaternion zero;
bool zeroFlag = false;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);

  setupMPU();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 18);
  display.cp437(true);
  display.println(F("  Digital"));
  display.println(F("  Angle"));
  display.println(F("  Gauge"));
  display.display();

  delay(2000);
  display.clearDisplay();
  display.display();

  pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_BUTTON_PIN), SetZero, FALLING);
  SetZero();
}

void loop() {
  if (!isDmpReady()) return;
  
  myAngle = loopMPU();
  Serial.print(myAngle.w);
  Serial.print(" ");
  Serial.print(myAngle.x);
  Serial.print(" ");
  Serial.print(myAngle.y);
  Serial.print(" ");
  Serial.println(myAngle.z);
  
  if (zeroFlag) {
    zero = myAngle;
    zeroFlag = false;
  }

  myAngleDegrees = GetAngleBetweenQuaternions(myAngle, zero);

  
  WriteToDisplay(myAngleDegrees);
  //Serial.println(myAngleDegrees);

  delay(100);
}

double GetAngleBetweenQuaternions(Quaternion q1, Quaternion q2) {
  //theta = 2*asin(norm of the vector part of the quaternion product q1*q2^(-1))
  double theta;
  double norm;
  Quaternion q2Inv, qMult;

  // invert q2
  q2Inv = InvertQuaternion(q2);

  // mult q1 x q2Inv
  qMult = MultiplyQuaternions(q1, q2Inv);
  //qMult = MultiplyQuaternions(q2Inv, q1);

  // get norm of the vector part
  // norm(Q)=√w^2+x^2+y^2+z^2
  // norm(Q)=√x^2+y^2+z^2  only vector part
  norm = sqrt(qMult.x * qMult.x + qMult.y * qMult.y + qMult.z * qMult.z);

  theta = 2* asin(norm);
  
  return theta * RAD_TO_DEG;
}

Quaternion InvertQuaternion(Quaternion q){
  Quaternion newQ;

  double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  double normSqared = norm*norm;
  newQ.w = q.w/normSqared;
  newQ.x = -q.x/normSqared;
  newQ.y = -q.y/normSqared;
  newQ.z = -q.z/normSqared;

  return newQ;
}

Quaternion MultiplyQuaternions(Quaternion q1, Quaternion q2){
  Quaternion result;
  
  /* Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm */
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

  return result;
}

double GetQuaternionNorm(Quaternion q){
  return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}


void WriteToDisplay(double angle) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 45);
  //display.println(abs(angle), 1);
  display.println(angle, 1);
  display.display();
}

void SetZero() {
  zeroFlag = true;
}
