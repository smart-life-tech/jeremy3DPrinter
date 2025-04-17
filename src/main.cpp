/*
  🔧 ENCLOSURE CONTROLLER FIRMWARE (TotalSense Edition)

  ✅ Included Features:
  - PID control for 2 independent heaters (Enclosure 1 & 2)
  - PWM output via LEDC (ESP32-native)
  - EEPROM-ready structure
  - OLED display with idle-time rotation (Filament + Enclosures)
  - Per-zone timers with on-screen countdown
  - Manual or timer mode selection per enclosure
  - Overheat failsafe (130°F cutoff) + Buzzer alert loop
  - Buzzer alerts:
      • 5 minutes left → single beep
      • 30 sec countdown → 4 beeps + final long beep
      • Continuous beeping on thermal runaway (130°F+)
  - Optional AUTO SHUT-OFF after 56 hours of runtime (toggleable)

  🔹 MAIN MENU:
  - 3D Enclosure 1
  - 3D Enclosure 2
  - Filament Box 1
  - Filament Box 2
  - All Sensors Display
  - SHUT ALL OFF
  - Settings (including auto-off toggle and PID Auto-Tune)

  🔹 SUBMENU FOR 3D ENCLOSURES:
  - Enclosure Temp Set
  - Mode: Manual / Timer
  - Timer Duration Selection (20min to 48hr)
  - Enclosure Temp Display

  🔹 SUBMENU FOR FILAMENT BOXES:
  - Temp & Humidity Display Only

  🔹 PINOUT MAP:
  - Heater 1 (ENC 1):      GPIO12 (PWM CH0)
  - Heater 2 (ENC 2):      GPIO13 (PWM CH1)
  - I2C SDA / SCL:         GPIO21 / GPIO22
  - OLED Reset:            GPIO16
  - Buzzer:                GPIO27 (active LOW — connect − to GPIO, + to 5V)
  - TCA9548A I2C MUX:
      - CH0 = Filament Sensor (Si7021)
      - CH1 = Enclosure 1 Sensor
      - CH2 = Enclosure 2 Sensor
  - Rotary Encoder:        CLK: GPIO32, DT: GPIO33, SW: GPIO25
  - Back Button:           GPIO26

  Author: Jeremy @ TotalSense
  Last Update: April 2025
*/

#include <Arduino.h>
#include "driver/ledc.h"
#include <Wire.h>
#include <TCA9548A.h>
#include <Adafruit_Si7021.h>
#include <PID_v1.h>
#include <FastPID.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Encoder.h>

#define HEATER1_PIN 12
#define HEATER2_PIN 13
#define BUZZER_PIN 27
#define OLED_RESET 16
#define EEPROM_SIZE 128
#define ENCODER_CLK 32
#define ENCODER_DT 33
#define ENCODER_SW 25
#define BACK_BUTTON 26
#define PWM_FREQ 5000
#define PWM_RES 8
#define HEATER1_CH 0
#define HEATER2_CH 1
#define SCREEN_IDLE_TIMEOUT 300000
#define SCREEN_SWITCH_INTERVAL 5000
#define MAX_MANUAL_RUNTIME_SECONDS 201600

TCA9548A tca;
Adafruit_Si7021 sensor = Adafruit_Si7021();
U8G2_SSD1309_128X64_NONAME2_F_HW_I2C u8g2(U8G2_R0, OLED_RESET);
Encoder encoder(ENCODER_CLK, ENCODER_DT);

long lastEncoderPos = 0;
bool encoderButtonPressed = false;
bool backButtonPressed = false;
bool autoShutoffEnabled = true;
bool beepOnPush = true;
int screenIndex = 100; // Start in main menu
int mainMenuIndex = 0;
int settingsMenuIndex = 0;

float filamentTemp = 0;
float filamentTemp2 = 0;
int humidityLimitF1 = 65;
bool humidityAlarmF1 = false;
int humidityLimitF2 = 65;
bool humidityAlarmF2 = false;

struct Zone
{
  String name;
  float currentTemp;
  float targetTemp;
  unsigned long timerSeconds;
  unsigned long timerStart;
  bool heaterOn;
  bool useTimer;
  unsigned long manualStart;
  int heaterPin;
  double input, output, setpoint;
  PID *pid;
};

Zone enclosure1 = {"3D Enclosure 1", 0, 90.0, 0, 0, false, true, 0, HEATER1_PIN, 0, 0, 90.0, nullptr};
int humidityLimit1 = 65;
bool humidityAlarm1 = false;
Zone enclosure2 = {"3D Enclosure 2", 0, 90.0, 0, 0, false, true, 0, HEATER2_PIN, 0, 0, 90.0, nullptr};
int humidityLimit2 = 65;
bool humidityAlarm2 = false;

FastPID fastPID1(2.0, 5.0, 1.0, 255, true);
FastPID fastPID2(2.0, 5.0, 1.0, 255, true);
bool tuning = false;

unsigned long lastInteraction = 0;
unsigned long lastScreenSwitch = 0;
bool inIdleMode = false;
bool buzzerLocked = false;

// Placeholder functions for screen routing

void displayMainMenu()
{
  int enclosureMenuIndex = 0;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 12);
  u8g2.print("MAIN MENU:");
  const char *items[] = {
      "3D Enclosure 1",
      "3D Enclosure 2",
      "Filament Box 1",
      "Filament Box 2",
      "All Sensors",
      "SHUT ALL OFF",
      "Settings"};
  for (int i = 0; i < 7; i++)
  {
    u8g2.setCursor(0, 24 + i * 10);
    u8g2.print((i == mainMenuIndex) ? "> " : "  ");
    u8g2.print(items[i]);
  }
  u8g2.sendBuffer();
}

void displaySettingsMenu()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 12);
  u8g2.print("SETTINGS MENU:");
  const char *settings[] = {
      "Auto-Tune ENC 1",
      "Auto-Tune ENC 2",
      "Toggle Auto-OFF",
      "Beep on Push"};
  for (int i = 0; i < 6; i++)
  {
    u8g2.setCursor(0, 28 + i * 10);
    u8g2.print((i == settingsMenuIndex) ? "> " : "  ");
    u8g2.print(settings[i]);
  }
  u8g2.sendBuffer();
}

void displayZone(const Zone &z)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 12);
  u8g2.print(z.name);
  u8g2.setCursor(0, 28);
  u8g2.print("Temp: ");
  u8g2.print(z.currentTemp);
  u8g2.print(" F");

  // Display humidity for corresponding enclosure channel
  int ch = (z.name.indexOf("1") >= 0) ? 1 : 2;
  float hum = 0;
  tca.openChannel(ch);
  if (sensor.begin())
    hum = sensor.readHumidity();
  tca.closeChannel(ch);

  u8g2.setCursor(0, 40);
  u8g2.print("Humidity: ");
  u8g2.print(hum);
  u8g2.print(" %");

  u8g2.setCursor(0, 52);
  if ((z.name.indexOf("1") >= 0 && hum > humidityLimit1 && humidityAlarm1) ||
      (z.name.indexOf("2") >= 0 && hum > humidityLimit2 && humidityAlarm2))
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 24);
    u8g2.print("HIGH HUMIDITY!");
    u8g2.setCursor(0, 44);
    u8g2.print(z.name);
    u8g2.sendBuffer();
    delay(500);
    u8g2.clear();
    u8g2.sendBuffer();
    delay(300);
  }
  u8g2.setCursor(0, 56);
  u8g2.print("Heater: ");
  u8g2.print(z.heaterOn ? "ON" : "OFF");
  u8g2.sendBuffer();
}

void displayFilament()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 12);
  u8g2.print("Filament Temps:");
  u8g2.setCursor(0, 28);
  u8g2.print("Box 1: ");
  u8g2.print(filamentTemp);
  u8g2.print(" F");
  u8g2.setCursor(0, 44);
  u8g2.print("Box 2: ");
  u8g2.print(filamentTemp2);
  u8g2.print(" F");
  u8g2.sendBuffer();
}

void updateScreen();

void displayAllTemps()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 12);
  u8g2.print("ALL TEMPS:");
  u8g2.setCursor(0, 28);
  u8g2.print("Fil 1: ");
  u8g2.print(filamentTemp);
  u8g2.print(" F");
  u8g2.setCursor(0, 38);
  u8g2.print("Fil 2: ");
  u8g2.print(filamentTemp2);
  u8g2.print(" F");
  u8g2.setCursor(0, 48);
  u8g2.print("ENC 1: ");
  u8g2.print(enclosure1.currentTemp);
  u8g2.print(" F");
  u8g2.setCursor(0, 58);
  u8g2.print("ENC 2: ");
  u8g2.print(enclosure2.currentTemp);
  u8g2.print(" F");
  u8g2.sendBuffer();
}
void displayMainMenu();
void displaySettingsMenu();
void displayZone(const Zone &z);
void displayFilament();
void displayAllTemps();

void setup()
{
  EEPROM.get(0, humidityLimit1);
  EEPROM.get(4, humidityLimit2);
  EEPROM.get(8, humidityLimitF1);
  EEPROM.get(12, humidityLimitF2);
  EEPROM.get(16, humidityAlarm1);
  EEPROM.get(17, humidityAlarm2);
  EEPROM.get(18, humidityAlarmF1);
  EEPROM.get(19, humidityAlarmF2);
  Serial.begin(115200);
  Wire.begin();
  tca.begin();
  EEPROM.begin(EEPROM_SIZE);
  u8g2.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(BACK_BUTTON, INPUT_PULLUP);

  ledcSetup(HEATER1_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(HEATER1_PIN, HEATER1_CH);
  ledcSetup(HEATER2_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(HEATER2_PIN, HEATER2_CH);

  enclosure1.pid = new PID(&enclosure1.input, &enclosure1.output, &enclosure1.setpoint, 2.0, 5.0, 1.0, DIRECT);
  enclosure2.pid = new PID(&enclosure2.input, &enclosure2.output, &enclosure2.setpoint, 2.0, 5.0, 1.0, DIRECT);
  enclosure1.pid->SetMode(AUTOMATIC);
  enclosure2.pid->SetMode(AUTOMATIC);
  enclosure1.pid->SetOutputLimits(0, 255);
  enclosure2.pid->SetOutputLimits(0, 255);

  lastInteraction = millis();
  Serial.println("System initialized.");
}

void loop()
{
  // Read all sensors
  tca.openChannel(0);
  if (sensor.begin())
    filamentTemp = sensor.readTemperature();
  tca.closeChannel(0);
  tca.openChannel(3);
  if (sensor.begin())
    filamentTemp2 = sensor.readTemperature();
  tca.closeChannel(3);
  tca.openChannel(1);
  if (sensor.begin())
    enclosure1.currentTemp = sensor.readTemperature();
  tca.closeChannel(1);
  tca.openChannel(2);
  if (sensor.begin())
    enclosure2.currentTemp = sensor.readTemperature();
  tca.closeChannel(2);

  // PID control if not tuning
  if (!tuning)
  {
    enclosure1.input = enclosure1.currentTemp;
    enclosure2.input = enclosure2.currentTemp;

    if (enclosure1.currentTemp >= 130.0 || enclosure2.currentTemp >= 130.0)
    {
      Serial.println("!! OVERHEAT DETECTED — SYSTEM SHUTDOWN !!");
      enclosure1.heaterOn = false;
      enclosure2.heaterOn = false;
      ledcWrite(HEATER1_CH, 0);
      ledcWrite(HEATER2_CH, 0);

      for (int i = 0; i < 3; i++)
      {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(0, 24);
        u8g2.print("!!! OVERHEAT !!!");
        u8g2.setCursor(0, 44);
        u8g2.print("SYSTEM SHUTDOWN");
        u8g2.sendBuffer();
        delay(500);
        u8g2.clear();
        u8g2.sendBuffer();
        delay(300);
      }

      for (int i = 0; i < 10; i++)
      {
        digitalWrite(BUZZER_PIN, LOW);
        delay(300);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
      }
      buzzerLocked = true;
    }
    else
    {
      enclosure1.input = enclosure1.currentTemp;
      enclosure1.output = fastPID1.step(enclosure1.input, enclosure1.setpoint);
      enclosure1.heaterOn = true;

      enclosure2.input = enclosure2.currentTemp;
      enclosure2.output = fastPID2.step(enclosure2.input, enclosure2.setpoint);
      enclosure2.heaterOn = true;
      ledcWrite(HEATER1_CH, (int)enclosure1.output);
      ledcWrite(HEATER2_CH, (int)enclosure2.output);
    }
  }

  // Encoder navigation
  long newPos = encoder.read() / 4;
  if (newPos != lastEncoderPos)
  {
    lastEncoderPos = newPos;
    lastInteraction = millis();
    if (screenIndex == 100)
      mainMenuIndex = abs(newPos) % 7;
    else if (screenIndex == 99)
      settingsMenuIndex = abs(newPos) % 4;
  }

  // Encoder button press
  if (digitalRead(ENCODER_SW) == LOW && !encoderButtonPressed)
  {
    if (beepOnPush)
    {
      digitalWrite(BUZZER_PIN, LOW);
      delay(20);
      digitalWrite(BUZZER_PIN, HIGH);
    }
    encoderButtonPressed = true;
    if (screenIndex == 100)
    {
      switch (mainMenuIndex)
      {
      case 0:
        screenIndex = 10;
        break;
      case 1:
        screenIndex = 11;
        break;
      case 2:
        screenIndex = 12;
        break;
      case 3:
        screenIndex = 13;
        break;
      case 4:
        screenIndex = 3;
        break;
      case 5:
        ledcWrite(HEATER1_CH, 0);
        ledcWrite(HEATER2_CH, 0);
        break;
      case 6:
        screenIndex = 99;
        break;
      }
    }
    else if (screenIndex == 12)
    {
      switch (lastEncoderPos % 2)
      {
      case 0:
        screenIndex = 26;
        break;
      case 1:
        humidityAlarmF1 = !humidityAlarmF1;
        EEPROM.put(18, humidityAlarmF1);
        EEPROM.commit();
        break;
      }
    }
    else if (screenIndex == 13)
    {
      switch (lastEncoderPos % 2)
      {
      case 0:
        screenIndex = 36;
        break;
      case 1:
        humidityAlarmF2 = !humidityAlarmF2;
        EEPROM.put(19, humidityAlarmF2);
        EEPROM.commit();
        break;
      }
    }
    else if (screenIndex == 10)
    {
      switch (lastEncoderPos % 6)
      {
      case 0:
        screenIndex = 24;
        break;
      case 1:
        enclosure1.useTimer = !enclosure1.useTimer;
        break;
      case 2:
        screenIndex = 20;
        break;
      case 3:
        screenIndex = 10;
        displayZone(enclosure1);
        delay(1000);
        break;
      case 4:
        screenIndex = 25;
        break;
      case 5:
        humidityAlarm1 = !humidityAlarm1;
        EEPROM.put(16, humidityAlarm1);
        EEPROM.commit();
        break;
      }
    }
    else if (screenIndex == 11)
    {
      switch (lastEncoderPos % 6)
      {
      case 0:
        screenIndex = 34;
        break;
      case 1:
        enclosure2.useTimer = !enclosure2.useTimer;
        break;
      case 2:
        screenIndex = 30;
        break;
      case 3:
        screenIndex = 11;
        displayZone(enclosure2);
        delay(1000);
        break;
      case 4:
        screenIndex = 35;
        break;
      case 5:
        humidityAlarm2 = !humidityAlarm2;
        EEPROM.put(17, humidityAlarm2);
        EEPROM.commit();
        break;
      }
    }
    else if (screenIndex == 99)
    {
      if (settingsMenuIndex == 0 && !tuning)
      {
        Serial.println("FastPID tuning is not dynamic — settings are applied statically.");
      }
      else if (settingsMenuIndex == 1 && !tuning)
      {
        Serial.println("FastPID tuning is not dynamic — settings are applied statically.");
      }
      else if (settingsMenuIndex == 2)
        autoShutoffEnabled = !autoShutoffEnabled;
      else if (settingsMenuIndex == 3)
        beepOnPush = !beepOnPush;
    }
  }
  else if (digitalRead(ENCODER_SW) == HIGH)
    encoderButtonPressed = false;

  if (digitalRead(BACK_BUTTON) == LOW && !backButtonPressed)
  {
    backButtonPressed = true;
    screenIndex = 100;
  }
  else if (digitalRead(BACK_BUTTON) == HIGH)
    backButtonPressed = false;

  // Update screen

  switch (screenIndex)
  {
  case 100:
    displayMainMenu();
    break;
  case 99:
    displaySettingsMenu();
    break;
  case 10:
  {
    const char *enc1menu[] = {"Set Temp", "Toggle Mode", "Set Timer", "View Temp", "Set Humidity Alert", "Toggle Humidity Alarm"};
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 12);
    u8g2.print("Enclosure 1 Menu:");
    for (int i = 0; i < 4; i++)
    {
      u8g2.setCursor(0, 28 + i * 10);
      u8g2.print((i == (lastEncoderPos % 4)) ? "> " : "  ");
      u8g2.print(enc1menu[i]);
    }
    u8g2.sendBuffer();
    break;
  }
  case 11:
  {
    const char *enc2menu[] = {"Set Temp", "Toggle Mode", "Set Timer", "View Temp", "Set Humidity Alert", "Toggle Humidity Alarm"};
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 12);
    u8g2.print("Enclosure 2 Menu:");
    for (int i = 0; i < 4; i++)
    {
      u8g2.setCursor(0, 28 + i * 10);
      u8g2.print((i == (lastEncoderPos % 4)) ? "> " : "  ");
      u8g2.print(enc2menu[i]);
    }
    u8g2.sendBuffer();
    break;
  }
  case 12:
  {
    const char *f1menu[] = {"Set Humidity Alert", "Toggle Humidity Alarm"};
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 12);
    u8g2.print("Filament Box 1:");
    for (int i = 0; i < 2; i++)
    {
      u8g2.setCursor(0, 28 + i * 10);
      u8g2.print((i == (lastEncoderPos % 2)) ? "> " : "  ");
      u8g2.print(f1menu[i]);
    }
    u8g2.sendBuffer();
    break;
  }
  case 13:
  {
    const char *f2menu[] = {"Set Humidity Alert", "Toggle Humidity Alarm"};
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 12);
    u8g2.print("Filament Box 2:");
    for (int i = 0; i < 2; i++)
    {
      u8g2.setCursor(0, 28 + i * 10);
      u8g2.print((i == (lastEncoderPos % 2)) ? "> " : "  ");
      u8g2.print(f2menu[i]);
    }
    u8g2.sendBuffer();
    break;
  }
  case 14:
    displayAllTemps();
    break;
  case 26:
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 20);
    u8g2.print("FBox1 Humidity Alert:");
    int limit = 30 + (abs(lastEncoderPos) % 70);
    u8g2.setCursor(0, 40);
    u8g2.print(limit);
    u8g2.print(" %");
    humidityLimitF1 = limit;
    EEPROM.put(8, humidityLimitF1);
    EEPROM.commit();
    u8g2.sendBuffer();
    break;
  }
  case 36:
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 20);
    u8g2.print("FBox2 Humidity Alert:");
    int limit = 30 + (abs(lastEncoderPos) % 70);
    u8g2.setCursor(0, 40);
    u8g2.print(limit);
    u8g2.print(" %");
    humidityLimitF2 = limit;
    EEPROM.put(12, humidityLimitF2);
    EEPROM.commit();
    u8g2.sendBuffer();
    break;
  }
  case 23:
    displayCountdown(enclosure1);
    break;
  case 33:
    displayCountdown(enclosure2);
    break;
  case 24:
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 20);
    u8g2.print("Set ENC1 Temp:");
    int tempSet = 70 + (abs(lastEncoderPos) % 51); // 70-120°F
    u8g2.setCursor(0, 40);
    u8g2.print(tempSet);
    u8g2.print(" F");
    enclosure1.setpoint = tempSet;
    u8g2.sendBuffer();
    break;
  }
  case 34:
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 20);
    u8g2.print("Set ENC2 Temp:");
    int tempSet = 70 + (abs(lastEncoderPos) % 51);
    u8g2.setCursor(0, 40);
    u8g2.print(tempSet);
    u8g2.print(" F");
    enclosure2.setpoint = tempSet;
    u8g2.sendBuffer();
    break;
  }
  case 25:
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 20);
    u8g2.print("ENC1 Humidity Alert:");
    int limit = 30 + (abs(lastEncoderPos) % 70);
    u8g2.setCursor(0, 40);
    u8g2.print(limit);
    u8g2.print(" %");
    humidityLimit1 = limit;
    EEPROM.put(0, humidityLimit1);
    EEPROM.commit();
    u8g2.sendBuffer();
    break;
  }
  case 35:
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 20);
    u8g2.print("ENC2 Humidity Alert:");
    int limit = 30 + (abs(lastEncoderPos) % 70);
    u8g2.setCursor(0, 40);
    u8g2.print(limit);
    u8g2.print(" %");
    humidityLimit2 = limit;
    EEPROM.put(4, humidityLimit2);
    EEPROM.commit();
    u8g2.sendBuffer();
    break;
  }
  case 20:
  {
    screenIndex = 21;
    enclosure1.useTimer = !enclosure1.useTimer;
    break;
  }
  case 21:
  {
    enclosure1.timerSeconds = (max((abs(lastEncoderPos) % 289), 2)) * 600;
  //  displayTimerDuration(enclosure1);
    break;
  }
  case 22:
  {
    enclosure1.timerStart = millis();
    enclosure1.heaterOn = true;
    screenIndex = 23;
    break;
  }
  case 30:
  {
    screenIndex = 31;
    enclosure2.useTimer = !enclosure2.useTimer;
    break;
  }
  case 31:
  {
    enclosure2.timerSeconds = (max((abs(lastEncoderPos) % 289), 2)) * 600;
    //displayTimerDuration(enclosure2);
    break;
  }
  case 32:
  {
    enclosure2.timerStart = millis();
    enclosure2.heaterOn = true;
    screenIndex = 33;
    break;
  }
  }

  // Timer check logic
  if (enclosure1.useTimer && enclosure1.heaterOn)
  {
    unsigned long elapsed = (millis() - enclosure1.timerStart) / 1000UL;
    unsigned long remaining = enclosure1.timerSeconds - elapsed;
    if (remaining == 300 && !buzzerLocked)
    { // 5 minutes left
      digitalWrite(BUZZER_PIN, LOW);
      delay(500);
      digitalWrite(BUZZER_PIN, HIGH);
    }
    if (remaining <= 30 && remaining > 0 && !buzzerLocked)
    {
      for (int i = 0; i < 4; i++)
      {
        digitalWrite(BUZZER_PIN, LOW);
        delay(150);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(150);
      }
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      delay(1000);
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerLocked = true;
    }
    if (elapsed >= enclosure1.timerSeconds)
    {
      enclosure1.heaterOn = false;
      ledcWrite(HEATER1_CH, 0);
      buzzerLocked = false;
    }
  }

  if (enclosure2.useTimer && enclosure2.heaterOn)
  {
    unsigned long elapsed = (millis() - enclosure2.timerStart) / 1000UL;
    unsigned long remaining = enclosure2.timerSeconds - elapsed;
    if (remaining == 300 && !buzzerLocked)
    {
      digitalWrite(BUZZER_PIN, LOW);
      delay(500);
      digitalWrite(BUZZER_PIN, HIGH);
    }
    if (remaining <= 30 && remaining > 0 && !buzzerLocked)
    {
      for (int i = 0; i < 4; i++)
      {
        digitalWrite(BUZZER_PIN, LOW);
        delay(150);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(150);
      }
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      delay(1000);
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerLocked = true;
    }
    if (elapsed >= enclosure2.timerSeconds)
    {
      enclosure2.heaterOn = false;
      ledcWrite(HEATER2_CH, 0);
      buzzerLocked = false;
    }
  }

  updateScreen();
  delay(500);
}

void displayTimerDuration(const Zone &z)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 12);
  u8g2.print("Set Timer:");
  u8g2.setCursor(0, 28);
  unsigned long sec = z.timerSeconds;
  u8g2.print(sec / 3600);
  u8g2.print("h ");
  u8g2.print((sec % 3600) / 60);
  u8g2.print("m");
  u8g2.sendBuffer();
}

void displayCountdown(const Zone &z)
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 12);
  u8g2.print(z.name);
  u8g2.setCursor(0, 28);
  u8g2.print("RUNNING (Timer):");
  unsigned long elapsed = (millis() - z.timerStart) / 1000UL;
  unsigned long remaining = (z.timerSeconds > elapsed) ? (z.timerSeconds - elapsed) : 0;
  u8g2.setCursor(0, 44);
  u8g2.print("Time Left: ");
  u8g2.print(remaining / 3600);
  u8g2.print("h ");
  u8g2.print((remaining % 3600) / 60);
  u8g2.print("m");
  u8g2.sendBuffer();
}
