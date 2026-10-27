#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MAX30105.h>
#include <Adafruit_ADXL345_U.h>
#include "filters.h"

/* ---------------- LCD ---------------- */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ---------------- ADXL345 ---------------- */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
char activity[10];

/* ---------------- GSR ---------------- */
#define GSR_PIN A3
int gsr_average = 0;

/* ---------------- ECG ---------------- */
#define ECG_PIN   A0
#define LO_PLUS   10
#define LO_MINUS  11
int ecgValue = 0;

/* ---------------- TEMP ---------------- */
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

/* ---------------- SPO2 ---------------- */
MAX30105 sensor;
float spo2 = 0;
bool finger_detected = false;

const unsigned long kFingerThreshold = 10000;
LowPassFilter low_pass_red(5.0, 400.0);
LowPassFilter low_pass_ir(5.0, 400.0);
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

/* Calibration */
float A = 1.5958422;
float B = -34.6596622;
float C = 112.6898759;

/* ---------------- THRESHOLDS ---------------- */
#define SPO2_CRITICAL 70
#define SPO2_ABNORMAL 120

#define ECG_LOW  200
#define ECG_HIGH 800

#define GSR_STRESS   400
#define GSR_ABNORMAL 650

#define TEMP_ABNORMAL 35.0

/* ---------------- BUZZER ---------------- */
#define BUZZER_PIN 3

/* ---------------- SLIDESHOW ---------------- */
unsigned long lastSlide = 0;
int slideIndex = 0;

/* ---------- ACTIVITY FUNCTION ---------- */
void detectActivity(float x, float y, float z) {
  float totalAcc = sqrt(x*x + y*y + z*z);

  if (totalAcc > 18.0)
    strcpy(activity, "FALL");
  else if (z > 7.5 && z < 10.5)
    strcpy(activity, "SIT");
  else if (z > 6.0)
    strcpy(activity, "STAND");
  else
    strcpy(activity, "ABNORMAL");
}

/* ---------- CLEAR LINE FUNCTION ---------- */
void clearLine(uint8_t line) {
  lcd.setCursor(0, line);
  lcd.print("                "); // 16 spaces
  lcd.setCursor(0, line);
}

/* ---------- PRINT LINE 2 PROPERLY ---------- */
void printLine2(String text) {
  clearLine(1);
  lcd.print(text);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.print("System Init...");
  delay(1500);
  lcd.clear();

  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  tempSensor.begin();
  tempSensor.setWaitForConversion(false);

  if (!sensor.begin()) {
    lcd.print("MAX30105 FAIL");
    while (1);
  }

  sensor.setup();
  sensor.setSampleRate(400);
  sensor.setPulseWidth(411);
  sensor.setPulseAmplitudeRed(255);
  sensor.setPulseAmplitudeIR(255);

  if (!accel.begin()) {
    lcd.print("ADXL345 FAIL");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_2_G);

  strcpy(activity, "IDLE");
}

void loop() {
  static unsigned long lastUpdate = 0;

  /* -------- READ SENSORS -------- */
  sensor.check();
  long redVal = sensor.getRed();
  long irVal  = sensor.getIR();

  if (irVal > kFingerThreshold) {
    finger_detected = true;
    float red = low_pass_red.process(redVal);
    float ir  = low_pass_ir.process(irVal);

    stat_red.process(red);
    stat_ir.process(ir);

    float r = ((stat_red.maximum() - stat_red.minimum()) / stat_red.average()) /
              ((stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average());

    spo2 = constrain(A*r*r + B*r + C, 0, 130);
  } else {
    finger_detected = false;
    spo2 = 0;
    stat_red.reset();
    stat_ir.reset();
  }

  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);

  long sum = 0;
  for (int i = 0; i < 10; i++) sum += analogRead(GSR_PIN);
  gsr_average = sum / 10;

  if (digitalRead(LO_PLUS) == LOW && digitalRead(LO_MINUS) == LOW)
    ecgValue = analogRead(ECG_PIN);
  else
    ecgValue = 0;

  sensors_event_t event;
  accel.getEvent(&event);
  detectActivity(event.acceleration.x, event.acceleration.y, event.acceleration.z);

  /* -------- CONTINUOUS BUZZER MONITORING -------- */
  bool abnormalDetected = false;
  if (strcmp(activity,"FALL") == 0) abnormalDetected = true;
  if (tempC > TEMP_ABNORMAL) abnormalDetected = true;
  if (finger_detected && (spo2 < SPO2_CRITICAL || spo2 > SPO2_ABNORMAL)) abnormalDetected = true;
  if ((digitalRead(LO_PLUS) == LOW && digitalRead(LO_MINUS) == LOW) &&
      (ecgValue < ECG_LOW || ecgValue > ECG_HIGH)) abnormalDetected = true;

  digitalWrite(BUZZER_PIN, abnormalDetected ? HIGH : LOW);

  /* -------- UPDATE LCD EVERY 1.5s -------- */
  if (millis() - lastUpdate >= 1500) {
    lastUpdate = millis();
    slideIndex = (slideIndex + 1) % 5;

    switch (slideIndex) {
      case 0: // ACTIVITY
        lcd.setCursor(0,0);
        lcd.print("ACTIVITY:");
        printLine2(activity);
        break;

      case 1: // TEMP
        lcd.setCursor(0,0);
        lcd.print("TEMP:    ");
        if (tempC > TEMP_ABNORMAL) printLine2("ABNORMAL");
        else printLine2(String(tempC,1) + " C");
        break;

      case 2: // SPO2
        lcd.setCursor(0,0);
        lcd.print("SPO2:    ");
        if (!finger_detected) printLine2("NO FINGER");
        else if (spo2 < SPO2_CRITICAL) printLine2("CRITICAL");
        else if (spo2 > SPO2_ABNORMAL) printLine2("ABNORMAL");
        else printLine2(String((int)spo2) + "% NORMAL");
        break;

      case 3: // ECG
        lcd.setCursor(0,0);
        lcd.print("ECG:     ");
        if (digitalRead(LO_PLUS) || digitalRead(LO_MINUS)) printLine2("LEADS OFF");
        else if (ecgValue < ECG_LOW || ecgValue > ECG_HIGH) printLine2("ABNORMAL");
        else printLine2(String(ecgValue) + " NORMAL");
        break;

      case 4: // GSR
      //  case 4: // GSR
  lcd.setCursor(0,0);
  lcd.print("GSR:     ");

  if (gsr_average > GSR_ABNORMAL)
    printLine2("HIGH STRESS");
  else if (gsr_average > GSR_STRESS)
    printLine2("Normal");
  else
    printLine2("Stress");
  break;

    }
  }
}