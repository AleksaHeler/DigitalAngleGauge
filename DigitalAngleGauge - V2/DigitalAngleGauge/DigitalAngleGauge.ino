// Measure Angle with a MPU-6050(GY-521)
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
#define ZERO_DELAY 1000

double myAngle;
double zeroAngle;
bool zeroFlag = false;
double myMax = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);

  // OLED Display
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
  // Set zero
  if(zeroFlag == true){
    delay(ZERO_DELAY);
    zeroAngle = getAngleSmooth();
    zeroFlag = false;
  }
  
  myAngle = getAngleSmooth() - zeroAngle;
  
  WriteToDisplay(myAngle);
  Serial.println(myAngle);
  
  delay(10);
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
