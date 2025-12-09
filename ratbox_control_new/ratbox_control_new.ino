// Controller: Arduino Nano

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define ENABLE_EASE_PRECISION
#define ENABLE_EASE_CUBIC
// #define ENABLE_MIN_AND_MAX_CONSTRAINTS
// #define DISABLE_MICROS_AS_DEGREE_PARAMETER
// #define DISABLE_PAUSE_RESUME
#define USE_LIGHTWEIGHT_SERVO_LIBRARY
#include <ServoEasing.hpp>
#include "modes.h"

// Initiate Servo object
ServoEasing pedestalServo;
ServoEasing curtainServo;

// Initiate LCD object
LiquidCrystal_I2C lcd(0x27, 16, 2);

const byte SERVO_PIN_PEDESTAL = 9;
const byte SERVO_PIN_CURTAIN = 10;
const byte FRONT_SENSOR_PIN = 7;
const byte HAPTIC_MOTOR_PIN = 5;
const byte CAMERA_PIN = 3;
const byte PEDESTAL_PHOTO_SENSOR_PIN = A3;

const byte LED_PIN = 13;
const byte PUSH_BUTTON_1_PIN = A0;
const byte PUSH_BUTTON_2_PIN = A1;
const byte PUSH_BUTTON_3_PIN = A2;
const byte PUSH_BUTTON_4_PIN = A6;
const byte PUSH_BUTTON_5_PIN = A7;
const byte DEBOUNCE_DELAY = 250;  // Debounce delay in millisecond

int selectedMode = 0;
//Notes:
//Moves 7mm/10 degrees, so total allowed movement of 12.6 cm, not expected 20cm
//////////////////////////////////////////////////////////////////////////////
void blinkLED(int numberOfBlinks, int blinkDuration) {
  for (int i = 0; i < numberOfBlinks; i++) {
    digitalWrite(LED_PIN, HIGH);
    
    delay(blinkDuration);
    digitalWrite(LED_PIN, LOW);
    delay(blinkDuration);
  }
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Debounce function for button presses
unsigned long lastDebounceTime = 0;                 // Stores the last time the button state changed
bool defaultButtonStateBase = HIGH;                 // Default button state for active high buttons
bool lastButtonStateBase = defaultButtonStateBase;  // Stores the last button state
bool debounceButton(byte pin, bool defaultButtonStateSpecific = HIGH) {
  bool currentButtonState = digitalRead(pin);
  defaultButtonStateBase = defaultButtonStateSpecific;
  if (currentButtonState != defaultButtonStateBase) {
    lastDebounceTime = millis();  // Reset debounce timer
    Serial.println(F("Button state changed."));
    lastButtonStateBase = currentButtonState;
    return true;
  }

  // Check if the button state is stable for the debounce delay
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // If the button state has changed
    if (currentButtonState != lastButtonStateBase) {
      lastButtonStateBase = currentButtonState;

      // Return true if the button is pressed (LOW for active low)
      if (defaultButtonStateSpecific == HIGH) {
        if (currentButtonState == LOW) {
          return true;
        }
      } else {
        if (currentButtonState == HIGH) {
          return true;
        }
      }
    }
  } else {
    Serial.println(F("Button state not stable."));
  }
  return false;
}
//////////////////////////////////////////////////////////////////////////////
void setup() {
  DDRB = 0x0f;  //D10 D11 motor pins set to output
  PORTB = 0;    //put step driver to sleep
  pinMode(PUSH_BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_4_PIN, INPUT);
  pinMode(PUSH_BUTTON_5_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);  // Set LED pin as an output
  pinMode(PEDESTAL_PHOTO_SENSOR_PIN, INPUT_PULLUP);
  pinMode(HAPTIC_MOTOR_PIN, OUTPUT);
  pinMode(CAMERA_PIN, OUTPUT);
  // pinMode(FRONT_SENSOR_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("Initializing..."));
  delay(500);
  pedestalServo.setEasingType(EASE_CUBIC_IN_OUT);
  pedestalServo.setSpeed(5);
  pedestalServo.attach(SERVO_PIN_PEDESTAL, 0);  // Attach the servo to pin D9
  delay(500);
  curtainServo.setEasingType(EASE_CUBIC_IN_OUT);
  pedestalServo.setSpeed(5);
  curtainServo.attach(SERVO_PIN_CURTAIN, 74);

  // Blink LED to indicate mode selection
  blinkLED(5, 200);

  // Wait 20 seconds for user to select mode
  int waitTime = 2000;
  unsigned long startMillis = millis();
  Serial.println(F("Select mode within 20 seconds."));  
  lcd.setCursor(0, 0);
  lcd.print(F("                "));
  delay(10);
  lcd.setCursor(0, 0);
  lcd.print(F("Select mode 20s"));
  delay(10);
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  delay(10);
  lcd.setCursor(0, 1);
  lcd.print(F("Press btn 1-5"));
  while (millis() - startMillis < waitTime) {
    if (debounceButton(PUSH_BUTTON_1_PIN)) {
      selectedMode = 1;
      Serial.println(F("Mode 1 selected."));
      break;
    } else if (debounceButton(PUSH_BUTTON_2_PIN)) {
      selectedMode = 2;
      Serial.println(F("Mode 2 selected."));
      break;
    } else if (debounceButton(PUSH_BUTTON_3_PIN)) {
      selectedMode = 3;
      Serial.println(F("Mode 3 selected."));
      break;
    } else if (debounceButton(PUSH_BUTTON_4_PIN, LOW)) {
      selectedMode = 4;
      Serial.println(F("Mode 4 selected."));
      break;
    } else if (debounceButton(PUSH_BUTTON_5_PIN, LOW)) {
      selectedMode = 5;
      Serial.println(F("Mode 5 selected."));
      break;
    }
  }

  // If no mode is selected, default to mode 1
  if (selectedMode == 0) {
    selectedMode = 5;
    Serial.println(F("No mode selected.Defaulting to mode 5."));
  }

  // Create mode object as needed
  switch (selectedMode) {
    case 1:
      {
        static Mode1 mode1(curtainServo, pedestalServo, lcd);
        currentMode = &mode1;
        break;
      }
    case 2:
      {
        static Mode2 mode2(curtainServo, pedestalServo, lcd);
        currentMode = &mode2;
        break;
      }
    case 3:
      {
        static Mode3 mode3(curtainServo, pedestalServo, lcd);
        currentMode = &mode3;
        break;
      }
    case 4:
      {
        static Mode4 mode4(curtainServo, pedestalServo, lcd);
        currentMode = &mode4;
        break;
      }
    case 5:
      {
        static Mode5 mode5(curtainServo, pedestalServo, lcd);
        currentMode = &mode5;
        break;
      }
  }

  if (currentMode->initialize()) {
    Serial.println(F("Mode initialized successfully."));
  } else {
    Serial.println(F("Mode initialization failed."));
  }
}

void loop() {
    if (currentMode != nullptr) {
      delay(10);  // Add a small delay to avoid overwhelming the loop
      currentMode->checkButtonPress();
      delay(10);  // Add a small delay to avoid overwhelming the loop
      currentMode->checkFrontSensor();
    }
}
