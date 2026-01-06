#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define ENABLE_EASE_ELASTIC
#define ENABLE_EASE_PRECISION
#include <ServoEasing.h>
#include "modes.h"

// Define the current mode pointer
Mode* currentMode = nullptr;

///////////////////////////////////////////////
// Initialization functions
bool Mode::getInitialized() {
    return isInitialized;
}

void Mode::setInitialized(bool initialized) {
    isInitialized = initialized;
}

// Barrier functions
void Mode::setServoAngle(int angle) {
    current_servo_angle = angle;
}

int Mode::getServoAngle() {
    return current_servo_angle;
}

int Mode::getAngularSpeed() {
    return angular_speed;
}

void Mode::setAngularSpeed(int speed) {
    barrier_servo.setSpeed(speed);
    angular_speed = speed;
}

void Mode::moveBarrierToPosition(int target_angle, bool blocking = true) {
    // current_servo.startEaseTo(target_angle, angular_speed, START_UPDATE_BY_INTERRUPT);
    if (blocking) {
        setServoAngle(target_angle);
        barrier_servo.easeTo(target_angle, angular_speed);
    } else {
        setServoAngle(target_angle);
        barrier_servo.startEaseTo(target_angle, angular_speed, START_UPDATE_BY_INTERRUPT);
        delay(500);
    }

}

void Mode::setBarrierPosition(int target_angle) {
    moveBarrierToPosition(target_angle);
}

void Mode::moveBarrierUp(int step_angle = 10, int barrier_speed = -1) {
    int current_angle = getServoAngle();
    int target_angle = current_angle + step_angle;
    if (barrier_speed == -1) {
        setAngularSpeed(barrier_up_speed);
    }
    barrierUpTriggered = true;
    barrierDownTriggered = false;
    moveBarrierToPosition(barrier_open_angle);

}

void Mode::moveBarrierDown(int step_angle = 10, int barrier_speed = -1) {
    int current_angle = getServoAngle();
    int target_angle = current_angle - step_angle;
    // setAngularSpeed(20);
    if (barrier_speed == -1) {
        setAngularSpeed(barrier_down_speed);
    }
    moveBarrierToPosition(barrier_close_angle);
    // Serial.println(F("Barrier moving down."));
    barrierDownTriggered = true;
    barrierUpTriggered = false;
}

// Pedestal functions
void Mode::setPedestalPosition(int run_time, bool is_up) {
    // Move the pedestal up or down
    // Serial.println(F("Feeder moving..."));
    if (is_up) {
        digitalWrite(9, LOW);
        digitalWrite(8, HIGH);
    } else {
        digitalWrite(9, HIGH);
        digitalWrite(8, LOW);
    }
    delay(run_time);

    // Stop the pedestal
    digitalWrite(9, LOW);
    digitalWrite(8, LOW);
}

void Mode::movePedestalUp(int run_time) {
    setPedestalPosition(run_time, true);
    setPedestalStatus(1); // Feeder is up
}

void Mode::movePedestalDown(int run_time) {
    setPedestalPosition(run_time, false);
    setPedestalStatus(2); // Feeder is down
}

void Mode::movePedestalToHome(int angle=-1, int speed=500) {
    // Move the feeder to the home position
    // Serial.println(F("Moving feeder to home position..."));
    if (angle == -1) {
        angle = home_angle;
    }
    pedestal_servo.easeTo(angle, speed);
    pedestal_servo.easeTo(angle, speed);
    digitalWrite(haptic_motor_pin, HIGH);
    delay(350);
    digitalWrite(haptic_motor_pin, LOW);
    delay(10);
    setPedestalStatus(0); // Feeder is down at home position
}

void Mode::movePedestalToCheck(int angle=-1, int speed=30) {
    // Move the feeder to the check position
    // Serial.println(F("Moving feeder to check position..."));
    if (angle == -1) {
        angle = check_angle;
    }
    pedestal_servo.easeTo(angle, speed);
    setPedestalStatus(1); // Feeder is up
}

void Mode::movePedestalToFeed(int angle=-1, int speed=-1) {
    // Move the feeder to the feed position
    // Serial.println(F("Moving feeder to feed position..."));
    if (angle == -1) {
        angle = feed_angle;
    }
    if (speed == -1) {
        speed = feed_speed;
    }
    pedestal_servo.easeTo(angle, speed);
    setPedestalStatus(3); // Feeder is at the feed position, load
}

void Mode::movePedestalToServe() {
    // Move the feeder to the serve position
    // Serial.println(F("Moving feeder to serve position..."));
    movePedestalToHome();
    delay(50);
    movePedestalToCheck();
    delay(100);
    while (!checkPelletStatus()) {
        movePedestalToHome();
        delay(500);
        movePedestalToCheck();
        delay(100);
    }
    // Move the feeder to the feed position
    movePedestalToFeed();
}

int Mode::getPedestalStatus() {
    // Implementation to get the current feeder position
    return pedestal_status; // Placeholder return value
}

int Mode::setPedestalStatus(int status) {
    // Implementation to set the current feeder position
    // 0: Feeder is at home position
    // 1: Feeder is at the check position, empty
    // 2: Feeder is at the check position, load
    // 3: Feeder is at the feed position, load
    pedestal_status = status;
    return pedestal_status;
}

bool Mode::calibratePedestal() {
    // Logic to calibrate the pedestal
    Serial.println(F("Calibrating pedestal..."));
    lcd.setCursor(0, 1);
    lcd.print(F("Calibrating..."));
    // // Move the pedestal to the home position and make sure it is there
    for (byte i = 0; i < 3; i++) {
        pedestal_servo.easeTo(home_angle, 500);
        delay(1000);
    }
    
    Serial.print(F("Make sure the pedestal is loaded with pellets."));
    delay(1000);
    byte calibrate_angle = check_angle;
    for (byte i = 0; i < 25; i++) {
        pedestal_servo.easeTo(check_angle, 100);
        delay(1000);
        if (checkPelletStatus()) {
            while (checkPelletStatus() && digitalRead(pellet_sensor_pin) == HIGH) {
                calibrate_angle = calibrate_angle - 1;
                pedestal_servo.easeTo(calibrate_angle-6, 50);
                delay(500);
                pedestal_servo.easeTo(calibrate_angle+5, 50);
                Serial.println(F("Pedestal might be too high."));
                Serial.println(calibrate_angle);
                delay(1000);
            }
            if (calibrate_angle < check_angle) {
                calibrate_angle = 20;
            }
            check_angle = calibrate_angle + 1 + 2;
            // check_angle = 20;
            feed_angle = check_angle + feed_angle_from_check;
            Serial.println(F("Calibration complete.")); 
            Serial.println(F("Check angle set to: "));
            Serial.println(check_angle);
            return true;
        } else {
            while (!checkPelletStatus() && digitalRead(pellet_sensor_pin) == LOW) {
                calibrate_angle = calibrate_angle + 1;
                pedestal_servo.easeTo(calibrate_angle, 50);
                
                Serial.println(F("Pedestal might be too low."));
                Serial.println(calibrate_angle);
                delay(1000);
            }
            if (calibrate_angle < check_angle) {
                calibrate_angle = 20;
            }
            check_angle = calibrate_angle + 1;
            // check_angle = 20;
            feed_angle = check_angle + feed_angle_from_check;
            Serial.println(F("Calibration complete.")); 
            Serial.println(F("Check angle set to: "));
            Serial.println(check_angle);
            return true;
        }
    }
    Serial.println(F("Calibration complete.")); 
    Serial.print(F("Check angle set to: "));
    Serial.println(check_angle);
    return true;
}

bool Mode::resetPedestal() {
    // Logic to reset the pedestal
    Serial.println(F("Resetting pedestal..."));
    pedestal_servo.easeTo(home_angle, 500);
    while (checkPelletStatus()) {
        pedestal_servo.easeTo(home_angle, 500);
        delay(50);
    }
    return true;
}

bool Mode::reloadPelletwithBarrierDown() {
    // Logic to reload the pedestal with the barrier
    Serial.println(F("Reloading pedestal with barrier..."));
    moveBarrierDown(); // Set barrierDownTriggered to true and barrierUpTriggered to false
    movePedestalToServe();
    delay(100);
    moveBarrierUp();
    return true;
}
    
bool Mode::loadPellet() {
    // Logic to load the pellet
    // Serial.println(F("Loading pellet..."));
    while (!checkPelletStatus()) {
        setPedestalStatus(0);
        movePedestalToHome();
        delay(500);
        movePedestalToCheck();
        delay(30);
    }
    // Move the feeder to the feed position
    setPedestalState(3);
    movePedestalToFeed();
    return true;
}

// Base mode functions
void Mode::blinkLockedButtonLED() {
    // Rapidly blink the LED to indicate a locked button press
    digitalWrite(6, HIGH);
    delay(100);
    digitalWrite(6, LOW);
    delay(100);
}

void Mode::reselectMode() {
    // Logic to reselect mode
    // Serial.println("Mode re-selection triggered.");
}

bool Mode::debounceButton(int pin, int pinIndex) {
    // Check if the button state is stable for the debounce delay
    if ((millis() - lastDebounceTime[pinIndex]) > debounce_delay) {
        isClicked[pinIndex] = false;
    }
    bool currentButtonState = digitalRead(pin);
    if (currentButtonState != defaultButtonState[pinIndex]) {
        if (isClicked[pinIndex] == false) {
            lastDebounceTime[pinIndex] = millis(); // Reset debounce timer
            Serial.println(F("Button state changed."));
            isClicked[pinIndex] = true;
            return true;
        } else {
            return false;
        }
    }
    return false;
}

bool Mode::checkButtonPress() {
    if (isInitialized == false) {
        return false;
    } else {
        if (digitalRead(button_pin_3) == LOW) {
            Serial.println(F("Button 3 pressed."));
            reloadPelletwithBarrierDown();
        } else if (analogRead(button_pin_4) ==0) {
            Serial.println(F("Button 4 pressed."));
            movePedestalToServe();
        } else if (digitalRead(button_pin_2) == LOW) {
            Serial.println(F("Button 2 pressed."));
            moveBarrierUp();
            if (isStarted == false) {
                isStarted = true;
                lcd.setCursor(0, 1);
                lcd.print(F("               "));
                delay(50);
                lcd.setCursor(0, 1);
                lcd.print("Attempt #: ");
            }
        } else if (digitalRead(button_pin_1) == LOW) {
            Serial.println(F("Button 1 pressed."));
            moveBarrierDown(); // Set barrierDownTriggered to true and barrierUpTriggered to false
        } else if (analogRead(button_pin_5) ==0) {
            Serial.println(F("Button 5 pressed."));
            resetPedestal();
        }
        return true;
    }
}

bool Mode::checkFrontSensor() {
    if (digitalRead(front_sensor_pin) == LOW) { // Front sensor is triggered
        if (reachingAttempt == false) {
            barrierDownTimer = millis();
            setReachingAttempt(true);
            Serial.println(F("Reaching attempt initiated."));
        }
    }
    if ((millis() - barrierDownTimer) > 10000 && reachingAttempt == true) {
        moveBarrierDown(); // Set barrierDownTriggered to true and barrierUpTriggered to false
        if (barrierDownTriggered) {
            setReachingAttempt(false);
            barrierUpTimer = millis(); // Start the timer before the barrier goes up
            barrierDownTimer = 0; // Reset the timer
        }
    } else {
        if (barrierDownTimer == 0 && reachingAttempt == false && barrierUpTimer != 0) {
            if ((millis() - barrierUpTimer) > 10000) {
                moveBarrierUp();
                if (barrierUpTriggered) {
                    setReachingAttempt(false);
                    barrierUpTimer = 0; // Reset the timer
                }
            }
        }
    }
    
    
    return true;
}

bool Mode::checkPelletStatus() {
    int sensorValue = digitalRead(pellet_sensor_pin);  // Read photodiode value
    Serial.println(F("Pellet sensor value: "));
    Serial.println(sensorValue);  // Print value for debugging
    // 
    // Serial.println(sensorValue);
    if (sensorValue == HIGH) { // The laser is blocked
        // Serial.println(F("Pellet detected."));
        return true;
    } else { // 
        // Serial.println(F("No pellet detected."));
        return false;
    }
    return true;
}

bool Mode::setReachingAttempt(bool attempt) {
    reachingAttempt = attempt;
    return reachingAttempt;
}

bool Mode::getReachingAttempt() {
    return reachingAttempt;
}   

bool Mode::setMode(int selected_mode) {
    // mode = selected_mode;
    lcd.setCursor(0, 0);
    lcd.print(F("               "));
    delay(50);
    lcd.setCursor(0, 0);
    lcd.print(F("Mode "));
    lcd.print(selected_mode);
    delay(10);

    lcd.setCursor(0, 1);
    lcd.print(F("               "));
    delay(50);
    lcd.setCursor(0, 1);
    lcd.print(F("Ready to start!"));
    delay(50);

    return true;
}
    
int Mode::getMode() {
    return true;
}

int Mode::getButtonPins(int pin_number) {
    // Implementation to get the button pins
    return 0; // Placeholder return value
}

bool Mode::setButtonPins(int pin_1, int pin_2, int pin_3, int pin_4, int pin_5) {
    // Implementation to set the button pins
    return true;
}

int Mode::getBarrierPosition() {
    // Implementation to get the barrier position
    return 0; // Placeholder return value
}

void Mode::setBarrierState(bool state) {
    barrierDownTriggered = state;
} // Switch between automated and manual barrier control

bool Mode::setPedestalState(int state) {
    // Implementation to set the feeder state
    return true;
} // Switch between automated and manual feeder control

Mode::Mode(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd)
    : barrier_servo(barrier_servo),
      pedestal_servo(pedestal_servo),
      lcd(lcd) {
        // barrier_servo.setEasingType(EASE_ELASTIC_IN_OUT);
        // barrier_servo.setSpeed(5);
        Serial.println(F("Mode object created. Servo speed set to 20."));
        Serial.println(F("Waiting for mode selection..."));
}

// Mode unique functions
// Mode 1: Fully automatic
Mode1::Mode1(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd) 
    : Mode(barrier_servo, pedestal_servo, lcd) {
        setInitialized(true);
    }

bool Mode1::initialize() {
    setBarrierPosition(barrier_close_angle);
    setInitialized(true);
    delay(100);
    calibratePedestal();
    delay(100);
    movePedestalToHome();
    delay(100);
    movePedestalToServe();
    pinMode(12, OUTPUT); //camera trigger, rising edge
    setInitialized(true);
    setMode(1);
    return true;
}


bool Mode1::checkFrontSensor() {
    // Serial.print(F("Button 1: "));
    // Serial.println(analogRead(button_pin_1));
    // Serial.print(F("Button 2: "));
    // Serial.println(analogRead(button_pin_2));
    // Serial.print(F("Button 3: "));
    // Serial.println(analogRead(button_pin_3));
    // Serial.print(F("Button 4: "));
    // Serial.println(analogRead(button_pin_4));
    // Serial.print(F("Button 5: "));
    // Serial.println(analogRead(button_pin_5));
    delay(10);
    return true;
}

// Mode 2: Semi-automatic 1
Mode2::Mode2(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd) 
    : Mode(barrier_servo, pedestal_servo, lcd) {
        setInitialized(true);
    }

bool Mode2::initialize() {
    setBarrierPosition(barrier_close_angle);
    delay(100);
    calibratePedestal();
    delay(100);
    movePedestalToHome();
    setInitialized(true);
    setMode(2);
    lcd.setCursor(0, 0);
    lcd.print(F("Mode 2"));
    return true;
}

bool Mode2::checkFrontSensor() {
    int front_sensor_reading = digitalRead(front_sensor_pin);
    if (front_sensor_reading == LOW) { // Front sensor is triggered
        if (reachingAttempt == false) {
            barrierDownTimer = millis();
            setReachingAttempt(true);
            Serial.println(F("Reaching attempt initiated."));
        }
    }
    if ((millis() - barrierDownTimer) > 10000 && reachingAttempt == true) {
        moveBarrierDown(); // Set barrierDownTriggered to true and barrierUpTriggered to false
        if (barrierDownTriggered) {
            setReachingAttempt(false);
            barrierUpTimer = millis(); // Start the timer before the barrier goes up
            barrierDownTimer = 0; // Reset the timer
        }
    }
    return true;
}

// Mode 3: Semi-automatic 2
Mode3::Mode3(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd) 
    : Mode(barrier_servo, pedestal_servo, lcd) {
        setInitialized(true);
    }

bool Mode3::initialize() {
    setBarrierPosition(barrier_close_angle);
    delay(100);
    calibratePedestal();
    delay(100);
    movePedestalToHome();
    setInitialized(true);
    setMode(3);
    return true;
}

bool Mode3::checkFrontSensor() {
    if (digitalRead(front_sensor_pin) == LOW && barrierUpTriggered== true ) { // Front sensor is triggered
        if (reachingAttempt == false) {
            digitalWrite(camera_pin, HIGH); // trigger the camera by generating a 100ms pulse
            delay(1000); //adjust the width of the Pulse here, for example 100 for a 100 ms pulse
            digitalWrite(camera_pin, LOW);
            barrierDownTimer = millis();
            setReachingAttempt(true);
            Serial.println(F("Reaching attempt initiated."));
        }
    }
    if ((millis() - barrierDownTimer) > 5000 && reachingAttempt == true && barrierDownTriggered == false) {
        moveBarrierDown(); // Set barrierDownTriggered to true and barrierUpTriggered to false
        if (barrierDownTriggered) {
            barrierUpTimer = millis(); // Start the timer before the barrier goes up
            barrierDownTimer = 0; // Reset the timer
        }
    }
    if ((millis() - barrierUpTimer) > 5000 && barrierDownTriggered == true) {
        moveBarrierUp();
        if (barrierUpTriggered) {
            setReachingAttempt(false);
            barrierUpTimer = millis(); // Reset the timer
        }
    }
    return true;
}

// Mode 4: Manual
Mode4::Mode4(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd) 
    : Mode(barrier_servo, pedestal_servo, lcd) {
        setInitialized(true);
    }

bool Mode4::initialize() {
    setBarrierPosition(barrier_close_angle);
    delay(100);
    calibratePedestal();
    delay(100);
    setInitialized(true);
    setMode(4);
    pinMode(12, OUTPUT); //camera trigger, rising edge
    return true;
}

bool Mode4::checkFrontSensor() {
    if (digitalRead(front_sensor_pin) == LOW) { // Front sensor is triggered
        if (reachingAttempt == false) {
            barrierDownTimer = millis();
            setReachingAttempt(true);
            digitalWrite(camera_pin, HIGH); // trigger the camera by generating a 100ms pulse
            delay(1000); //adjust the width of the Pulse here, for example 100 for a 100 ms pulse
            digitalWrite(camera_pin, LOW);
            Serial.println(F("Reaching attempt initiated."));
        }
    }
    if ((millis() - barrierDownTimer) > 10000 && reachingAttempt == true) {
        moveBarrierDown(); // Set barrierDownTriggered to true and barrierUpTriggered to false
        if (barrierDownTriggered) {
            setReachingAttempt(false);
            barrierUpTimer = millis(); // Start the timer before the barrier goes up
            barrierDownTimer = 0; // Reset the timer
        }
    }
    return true;
}

// Mode 5: Manual testing
Mode5::Mode5(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd) 
    : Mode(barrier_servo, pedestal_servo, lcd) {
        setInitialized(true);
    }

bool Mode5::initialize() {
    lcd.setCursor(0, 0);
    lcd.print(F("               "));
    delay(50);
    lcd.setCursor(0, 0);
    lcd.print(F("Mode 5"));
    Serial.println(F("Analog read of each pin:"));
    Serial.print(F("Button 1: "));
    Serial.println(analogRead(button_pin_1));
    Serial.print(F("Button 2: "));
    Serial.println(analogRead(button_pin_2));
    Serial.print(F("Button 3: "));
    Serial.println(analogRead(button_pin_3));
    Serial.print(F("Button 4: "));
    Serial.println(digitalRead(button_pin_4));
    Serial.print(F("Button 5: "));
    Serial.println(analogRead(button_pin_5));
    setBarrierPosition(barrier_close_angle);
    setInitialized(true);
    calibratePedestal();
    delay(100);
    movePedestalToHome();
    delay(100);
    movePedestalToServe();
    setMode(5);
    pinMode(12, OUTPUT); //camera trigger, rising edge
    return true;
}

bool Mode5::checkFrontSensor() {

    if (digitalRead(front_sensor_pin) == LOW && barrierUpTriggered== true ) { // Front sensor is triggered
        if (reachingAttempt == false) {
            digitalWrite(camera_pin, HIGH); // trigger the camera by generating a 100ms pulse
            delay(200); //adjust the width of the Pulse here, for example 100 for a 100 ms pulse
            digitalWrite(camera_pin, LOW);
            barrierDownTimer = millis();
            setReachingAttempt(true);
            movePedestalToHome();
            attempt_count++;
            Serial.println(F("Reaching attempt initiated."));
            if (attempt_count < 10) {
                lcd.setCursor(12, 1);
                lcd.print(attempt_count);
            } else if (attempt_count < 100) {
                lcd.setCursor(11, 1);
                lcd.print(attempt_count);
            } else {
                lcd.setCursor(10, 1);
                lcd.print(attempt_count);
            }
        }
    }
    if ((millis() - barrierDownTimer) > barrierDownTimeDuration && reachingAttempt == true && barrierDownTriggered == false) {
        moveBarrierDown(); // Set barrierDownTriggered to true and barrierUpTriggered to false
        movePedestalToServe();
        if (barrierDownTriggered) {
            barrierUpTimer = millis(); // Start the timer before the barrier goes up
            barrierDownTimer = 0; // Reset the timer
            
        }
    }
    int pedestal_status = getPedestalStatus();
    if ((millis() - barrierUpTimer) > barrierUpWaitTimeDuration && barrierDownTriggered == true && pedestal_status == 3 && reachingAttempt == true) {
        setReachingAttempt(false);
        moveBarrierUp();
    }
    return true;
}