#include <Arduino.h>
#define ENABLE_EASE_ELASTIC
#define ENABLE_EASE_CUBIC
// #define ENABLE_EASE_PRECISION
// #define ENABLE_EASE_USER
#include <ServoEasing.h>
#include "servo_config.h"

// Initiate Servo object
ServoEasing pedestalServo;
ServoEasing curtainServo;
