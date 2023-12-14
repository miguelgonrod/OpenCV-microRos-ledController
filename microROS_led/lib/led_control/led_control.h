#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>

// Pin Definitions
#define pin_led 2

// Functions
void led_setup();
void turnOn();
void turnOff();

#endif