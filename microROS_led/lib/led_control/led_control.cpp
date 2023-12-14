#include "led_control.h"

void led_setup(){
    pinMode (pin_led, OUTPUT);
}

void turnOn(){
    digitalWrite(pin_led, HIGH);
}

void turnOff(){
    digitalWrite(pin_led, LOW);
}
