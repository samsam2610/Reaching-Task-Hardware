#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

// Forward declaration only
// class ServoEasing;
#define ENABLE_EASE_ELASTIC
#define ENABLE_EASE_CUBIC
#define ENABLE_EASE_PRECISION
// #define ENABLE_EASE_USER
#include <ServoEasing.h>
extern ServoEasing pedestalServo;
extern ServoEasing curtainServo;

#endif
