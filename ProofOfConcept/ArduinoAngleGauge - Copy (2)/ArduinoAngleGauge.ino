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

struct Angles {
  double roll, pitch, yaw;
};

struct Vector3 {
  double x, y, z;
};

const int MPU_addr = 0x68;

const int minVal = 265;
const int maxVal = 402;

Angles zero;
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
  Angles angles;
  double angle;
  Vector3 angleVector, zeroVector;
  
  angles = GetAnglesSmooth();

  if (zeroFlag) {
    delay(ZERO_DELAY);
    zero = GetAnglesSmooth();
    zeroFlag = false;
  }

  angleVector = AngleToVector(angles);
  zeroVector = AngleToVector(zero);

  angle = AngleBetweenVectors(angleVector, zeroVector);

  WriteToDisplay(angle);

  delay(1);
}

Angles GetAnglesSmooth() {
  Angles angle;
  Angles sum;

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

Angles GetAngle() {
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

  Angles angle;
  angle.roll = x;
  angle.pitch = y;
  angle.yaw = z;

  return angle;
}

Vector3 AngleToVector(Angles a){
  Vector3 v;

  v.x = -cos(a.yaw) * sin(a.pitch) * sin(a.roll) - sin(a.yaw) * cos(a.roll);
  v.y = -sin(a.yaw) * sin(a.pitch) * sin(a.roll) + cos(a.yaw) * cos(a.roll);
  v.z = cos(a.pitch) * sin(a.roll);
  
  return v;
}

//angle = arccos[     (xa * xb + ya * yb + za * zb)    /   [    √(xa^2 + ya^2 + za^2) * √(xb^2 + yb^2 + zb^2)  ]  ]
double AngleBetweenVectors(Vector3 a, Vector3 b){
  double angle = 0;
  angle = acos( (a.x*b.x + a.y*b.y + a.z*b.z) / ( sqrt(a.x*a.x + a.y*a.y + a.z*a.z) * sqrt(b.x*b.x + b.y*b.y + b.z*b.z) ) );
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
