#ifndef MODES_H
#define MODES_H
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define ENABLE_EASE_PRECISION
#include <ServoEasing.h>


// Base class (optional, for a common interface)
class Mode {
    protected:
        ServoEasing& barrier_servo;
        ServoEasing& pedestal_servo;
        LiquidCrystal_I2C& lcd;
        byte angular_speed = 200; // Default angular speed
        bool isInitialized = false;
        bool isStarted = false;
        byte pedestal_status = 0;
        int pedestal_sensor_threshold_upper = 30;
        int  pedestal_sensor_threshold_lower = 20;// Threshold for the photodiode sensor sensing the pellet
        byte selectedMode = 2;

        const byte button_pin_1 = A0;
        const byte button_pin_2 = A1;
        const byte button_pin_3 = A2;
        const byte button_pin_4 = A6;
        const byte button_pin_5 = A7;
        const byte led_pin = 13;
        const byte camera_pin = 3;

        // Debounce variables
        unsigned long debounce_delay = 250;
        int lastDebounceTime[5] = {0, 0, 0, 0, 0}; // Stores the last time the button state changed
        bool defaultButtonState[5] = {HIGH, HIGH, HIGH, LOW, LOW}; // Default button state for active high buttons
        bool lastButtonState[5] = {HIGH, HIGH, HIGH, LOW, LOW}; // Stores the last button state
        bool isClicked[5] = {false, false, false, false, false}; // Stores the last button state

        // Front sensor variables
        const byte front_sensor_pin = 7;
        const byte haptic_motor_pin = 5;
        const byte pellet_sensor_pin = A3;

        byte pellet_sensor_threshold = 18; // Threshold for the photodiode sensor sensing the pellet
        bool reachingAttempt = false;
        bool barrierUpTriggered = false;
        bool barrierDownTriggered = false;
        unsigned long barrierDownTimeDuration = 2000;
        unsigned long barrierUpTimeDuration = 500;
        unsigned long barrierUpWaitTimeDuration = 100; 
        unsigned long barrierDownTimer = 0;
        unsigned long barrierUpTimer = 0;

        int home_angle = 0;
        int check_angle = 10;
        int feed_angle = 80;
        int feed_angle_from_check = 50;
        int feed_speed = 50;

        unsigned int barrier_close_angle = 80;
        unsigned int barrier_open_angle = 15;
        unsigned int barrier_up_speed = 120;
        unsigned int barrier_down_speed = 40;

        unsigned int attempt_count = 0;

    public:
        Mode(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd);
        virtual ~Mode() {};
        virtual void blinkLockedButtonLED();
        virtual void reselectMode();
        virtual bool checkButtonPress();
        virtual bool checkFrontSensor();
        virtual bool checkPelletStatus();
        virtual bool calibratePedestal();
        virtual bool resetPedestal();
        virtual bool debounceButton(int pin, int pinIndex);

        // set functions
        virtual void setPedestalPosition(int angle, bool is_up);
        virtual int setPedestalStatus(int status);
        virtual void movePedestalUp(int angle);
        virtual void movePedestalDown(int angle);
        virtual void movePedestalToCheck(int angle, int speed);
        virtual void movePedestalToFeed(int angle, int speed);
        virtual void movePedestalToHome(int angle, int speed);
        virtual void movePedestalToServe();
        virtual bool loadPellet();
        virtual bool setMode(int selected_mode);
        virtual bool setButtonPins(int pin_1, int pin_2, int pin_3, int pin_4, int pin_5);
        virtual bool setPedestalState(int state); // Switch between automated and manual feeder control
        virtual bool setReachingAttempt(bool attempt);

        virtual void setBarrierPosition(int target_angle);
        virtual void moveBarrierUp(int step_angle, int barrier_speed); 
        virtual void moveBarrierDown(int step_angle, int barrier_speed);
        virtual void moveBarrierToPosition(int target_angle, bool block = true);
        virtual void setServoAngle(int angle);
        virtual void setAngularSpeed(int speed);
        virtual void setInitialized(bool initialized);
        virtual void setBarrierState(bool state); // Switch between automated and manual barrier control

        virtual bool reloadPelletwithBarrierDown();

        // get functions
        virtual int getBarrierPosition();
        virtual int getPedestalStatus();
        virtual int getServoAngle();
        virtual int getAngularSpeed();
        virtual bool getInitialized();
        virtual int getMode();
        virtual int getButtonPins(int pin_number);
        virtual bool getReachingAttempt();
        
        // Initialization functions
        virtual bool initialize() = 0;

        int current_servo_angle = 0;
};

// Mode 1: Fully automatic
class Mode1 : public Mode {
    public:
        Mode1(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd);
        bool initialize() override;
        bool checkFrontSensor() override;
};  

// Mode 2: Semi-automatic 1
class Mode2 : public Mode {
    public:
        Mode2(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd);
        bool initialize() override;
        bool checkFrontSensor() override;
};

// Mode 3: Semi-automatic 2
class Mode3 : public Mode {
    public:
        Mode3(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd);
        bool initialize() override;
        bool checkFrontSensor() override;
};

// Mode 4: Manual
class Mode4 : public Mode {
    public:
        Mode4(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd);
        bool initialize() override;
        bool checkFrontSensor() override;
};

// Mode 5: Manual for testing
class Mode5 : public Mode {
    public:
        Mode5(ServoEasing& barrier_servo, ServoEasing& pedestal_servo, LiquidCrystal_I2C& lcd);
        bool initialize() override;
        bool checkFrontSensor() override;
};

// Externally declare the current mode pointer
extern Mode* currentMode;

#endif // MODES_H