#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_TCS34725.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

#define a_ENA 2
#define a_DIR 3
#define a_PUL 4
#define a_RUNSW A0
#define a_DIRSW A1

#define b_ENA 5
#define b_DIR 6
#define b_PUL 7
#define b_RUNSW A2
#define b_DIRSW A3

#define c_ENA 8
#define c_DIR 9
#define c_PUL 10
#define c_RUNSW 11
#define c_DIRSW 12

int a_runState = 0, a_dirState = 0;
int b_runState = 0, b_dirState = 0;
int c_runState = 0, c_dirState = 0;

int a_lastRunButton = HIGH;
int b_lastRunButton = HIGH;
int c_lastRunButton = HIGH;

unsigned long a_lastStepTime = 0;
unsigned long b_lastStepTime = 0;
unsigned long c_lastStepTime = 0;

unsigned long a_interval = 50;
unsigned long b_interval = 3000;
unsigned long c_interval = 50;

unsigned long lastRGBUpdate = 0;
const unsigned long RGB_INTERVAL = 100000;

bool tcs_found = false;
uint16_t rVal = 0, gVal = 0, bVal = 0;

void setup() {
  pinMode(a_ENA, OUTPUT);
  pinMode(a_DIR, OUTPUT);
  pinMode(a_PUL, OUTPUT);
  pinMode(a_RUNSW, INPUT_PULLUP);
  pinMode(a_DIRSW, INPUT_PULLUP);
  
  pinMode(b_ENA, OUTPUT);
  pinMode(b_DIR, OUTPUT);
  pinMode(b_PUL, OUTPUT);
  pinMode(b_RUNSW, INPUT_PULLUP);
  pinMode(b_DIRSW, INPUT_PULLUP);
  
  pinMode(c_ENA, OUTPUT); 
  pinMode(c_DIR, OUTPUT);
  pinMode(c_PUL, OUTPUT);
  pinMode(c_RUNSW, INPUT_PULLUP);
  pinMode(c_DIRSW, INPUT_PULLUP);
  
  digitalWrite(a_ENA, HIGH);
  digitalWrite(b_ENA, HIGH);
  digitalWrite(c_ENA, HIGH);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (tcs.begin()) tcs_found = true;
}

void loop() {
  unsigned long nowMillis = millis();

  int a_runButton = digitalRead(a_RUNSW);
  int b_runButton = digitalRead(b_RUNSW);
  int c_runButton = digitalRead(c_RUNSW);

  if (tcs_found && nowMillis - lastRGBUpdate >= RGB_INTERVAL) {
    uint16_t r, g, b, cVal;
    tcs.getRawData(&r, &g, &b, &cVal);
    rVal = r;
    gVal = g;
    bVal = b;
    lastRGBUpdate = nowMillis;
  }

  if (a_runButton == LOW && a_lastRunButton == HIGH) {
    a_runState = !a_runState;
    delay(50);
  }
  if (b_runButton == LOW && b_lastRunButton == HIGH) {
    b_runState = !b_runState;
    delay(50);
  }
  if (c_runButton == LOW && c_lastRunButton == HIGH) {
    c_runState = !c_runState;
    delay(50);
  }

  a_lastRunButton = a_runButton;
  b_lastRunButton = b_runButton;
  c_lastRunButton = c_runButton;

  if(digitalRead(a_DIRSW)==LOW) {
    delay(50);
    if(digitalRead(a_DIRSW)==LOW) {
      a_dirState=!a_dirState;
      digitalWrite(a_DIR, a_dirState);
      while(digitalRead(a_DIRSW)==LOW);
    }
  }

  if(digitalRead(b_DIRSW)==LOW) {
    delay(50);
    if(digitalRead(b_DIRSW)==LOW) {
      b_dirState=!b_dirState;
      digitalWrite(b_DIR, b_dirState);
      while(digitalRead(b_DIRSW)==LOW);
    }
  }

  if(digitalRead(c_DIRSW)==LOW) {
    delay(50);
    if(digitalRead(c_DIRSW)==LOW) {
      c_dirState=!c_dirState;
      digitalWrite(c_DIR, c_dirState);
      while(digitalRead(c_DIRSW)==LOW);
    }
  }

  unsigned long nowMicros = micros();

  if (a_runState && nowMicros - a_lastStepTime >= a_interval) {
    digitalWrite(a_PUL, HIGH);
    delayMicroseconds(10);
    digitalWrite(a_PUL, LOW);
    a_lastStepTime = nowMicros;
  }

  if (b_runState && nowMicros - b_lastStepTime >= b_interval) {
    digitalWrite(b_PUL, HIGH);
    delayMicroseconds(10);
    digitalWrite(b_PUL, LOW);
    b_lastStepTime = nowMicros;
  }

  if (c_runState && nowMicros - c_lastStepTime >= c_interval) {
    digitalWrite(c_PUL, HIGH);
    delayMicroseconds(10);
    digitalWrite(c_PUL, LOW);
    c_lastStepTime = nowMicros;
  }

  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
  display.setCursor(0, 0);
  display.print("A: ");
  display.print(a_runState ? "START / " : "STOP / ");
  display.println(a_dirState ? "BWD" : "FWD");
  display.print("B: ");
  display.print(b_runState ? "START / " : "STOP / ");
  display.println(b_dirState ? "BWD" : "FWD");
  display.print("C: ");
  display.print(c_runState ? "START / " : "STOP / ");
  display.println(c_dirState ? "BWD" : "FWD");
  display.print("R:"); display.print(rVal);
  display.print(" G:"); display.print(gVal);
  display.print(" B:"); display.print(bVal);
  display.display();
}
