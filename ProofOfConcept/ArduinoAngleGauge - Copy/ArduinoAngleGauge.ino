//Written by Ahmet Burkay KIRNIK
//TR_CapaFenLisesi
//Measure Angle with a MPU-6050(GY-521)

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// OLED stuff
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

#define ANGLE_SMOOTHING_ITERATIONS 50
#define ANGLE_SMOOTHING_DELAY 1

#define ZERO_BUTTON_PIN 3
#define ZERO_DELAY 500

struct Quaternion
{
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

const int MPU_addr = 0x68;

const int minVal = 265;
const int maxVal = 402;

EulerAngles angleEuler, zeroEuler;
Quaternion angleQuaternion, zeroQuaternion;
double angleDouble;

bool zeroFlag = false;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 15);
  display.cp437(true);
  display.println("  Digital");
  display.println("  Angle");
  display.println("  Gauge");
  display.display();

  delay(2000);
  display.clearDisplay();
  display.display();

  pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ZERO_BUTTON_PIN), SetZero, FALLING);
  SetZero();
}

void loop() {
  angleEuler = GetAngleSmooth();

  if (zeroFlag) {
    delay(ZERO_DELAY);
    zeroEuler = GetAngleSmooth();
    zeroFlag = false;
  }

  angleQuaternion = ToQuaternion(angleEuler);
  zeroQuaternion = ToQuaternion(zeroEuler);
  angleQuaternion = ConvertToUnitQuaternion(angleQuaternion);
  zeroQuaternion = ConvertToUnitQuaternion(zeroQuaternion);
  angleDouble = GetAngleBetweenQuaternions(angleQuaternion, zeroQuaternion);

  WriteToDisplay(angleDouble);

  /*Serial.print("X: ");
  Serial.print(angleEuler.roll);
  Serial.print("  Y: ");
  Serial.print(angleEuler.pitch);
  Serial.print("  Z: ");
  Serial.print(angleEuler.yaw);

  Serial.print("\t\tZERO   X: ");
  Serial.print(zeroEuler.roll);
  Serial.print("  Y: ");
  Serial.print(zeroEuler.pitch);
  Serial.print("  Z: ");
  Serial.print(zeroEuler.yaw);

  Serial.print("\t\tANGLE: ");
  Serial.println(angleDouble);*/

  delay(1);
}

EulerAngles GetAngleSmooth() {
  EulerAngles angle;
  EulerAngles sum;

  for (int i = 0; i < ANGLE_SMOOTHING_ITERATIONS; i++) {
    angle = GetAngle();
    sum.roll += angle.roll;
    sum.pitch += angle.pitch;
    sum.yaw += angle.yaw;
    delay(ANGLE_SMOOTHING_DELAY);
  }

  sum.roll /= ANGLE_SMOOTHING_ITERATIONS;
  sum.pitch /= ANGLE_SMOOTHING_ITERATIONS;
  sum.yaw /= ANGLE_SMOOTHING_ITERATIONS;
  return sum;
}

EulerAngles GetAngle() {
  int16_t AcX, AcY, AcZ;
  double x;
  double y;
  double z;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  EulerAngles angle;
  angle.roll = x;
  angle.pitch = y;
  angle.yaw = z;

  return angle;
}

void WriteToDisplay(double angle) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.println(angle, 1);
  display.display();
}

void SetZero() {
  zeroFlag = true;
}

double GetAngleBetweenQuaternions(Quaternion q1, Quaternion q2) {
  //theta = 2*asin(norm of the vector part of the quaternion product q1*q2^(-1))
  double theta;
  double norm;
  Quaternion q2Inv, qMult;

  // invert q2
  q2Inv = InvertQuaternion(q2);

  // mult q1 x q2Inv
  //qMult = MultiplyQuaternions(q1, q2Inv);
  qMult = MultiplyQuaternions(q2Inv, q1);

  // get norm of the vector part
  // norm(Q)=√w^2+x^2+y^2+z^2
  // norm(Q)=√x^2+y^2+z^2  only vector part
  norm = sqrt(qMult.x * qMult.x + qMult.y * qMult.y + qMult.z * qMult.z);

  theta = 2* asin(norm);
  
  /*Serial.print("Q1 W: ");
  Serial.print(q1.w);
  Serial.print("  X: ");
  Serial.print(q1.x);
  Serial.print("  Y: ");
  Serial.print(q1.y);
  Serial.print("  Z: ");
  Serial.print(q1.z);
  
  Serial.print("\t\tQ2 W: ");
  Serial.print(q2.w);
  Serial.print("  X: ");
  Serial.print(q2.x);
  Serial.print("  Y: ");
  Serial.print(q2.y);
  Serial.print("  Z: ");
  Serial.print(q2.z);
  
  Serial.print("\t\tQ2 Inv W: ");
  Serial.print(q2Inv.w);
  Serial.print("  X: ");
  Serial.print(q2Inv.x);
  Serial.print("  Y: ");
  Serial.print(q2Inv.y);
  Serial.print("  Z: ");
  Serial.print(q2Inv.z);
  
  Serial.print("\t\tQ Mult W: ");
  Serial.print(qMult.w);
  Serial.print("  X: ");
  Serial.print(qMult.x);
  Serial.print("  Y: ");
  Serial.print(qMult.y);
  Serial.print("  Z: ");
  Serial.print(qMult.z);
  
  Serial.print("\t\tNorm: ");
  Serial.print(norm);
  
  Serial.print("\t\tTheta: ");
  Serial.println(theta);*/

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

double GetQuaternionNorm(Quaternion q){
  return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quaternion ConvertToUnitQuaternion(Quaternion q){
  // A unit quaternion is a quaternion of norm one
  // Dividing a non-zero quaternion q by its norm produces a unit quaternion Uq called the versor of q
  Quaternion Uq;
  double norm = GetQuaternionNorm(q);
  Uq.w = q.w/norm;
  Uq.x = q.x/norm;
  Uq.y = q.y/norm;
  Uq.z = q.z/norm;
  return Uq;
}

Quaternion MultiplyQuaternions(Quaternion q1, Quaternion q2){
  Quaternion result;

  /*
  Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
           a*e - b*f - c*g - d*h
      + i (b*e + a*f + c*h- d*g)
      + j (a*g - b*h + c*e + d*f)
      + k (a*h + b*g - c*f + d*e)
  */
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

  return result;
}

Quaternion ToQuaternion(EulerAngles a)
{
  // roll (X), pitch (Y), yaw (Z)
  // Abbreviations for the various angular functions
  double cy = cos(a.yaw * 0.5);
  double sy = sin(a.yaw * 0.5);
  double cp = cos(a.pitch * 0.5);
  double sp = sin(a.pitch * 0.5);
  double cr = cos(a.roll * 0.5);
  double sr = sin(a.roll * 0.5);

  Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

EulerAngles ToEulerAngles(Quaternion q) {
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (abs(sinp) >= 1)
    angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    angles.pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = atan2(siny_cosp, cosy_cosp);

  return angles;
}
