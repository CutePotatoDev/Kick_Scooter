#include <Arduino.h>
#include "src/LedControl/src/LedControl.h"
#include "src/RC_ESC-master/ESC.h"
#include "src/SimpleKalmanFilter/src/SimpleKalmanFilter.h"

ESC controller(3, 1000, 2000, 1490);
SimpleKalmanFilter ADC_filter = SimpleKalmanFilter(8, 8, 0.05);
LedControl display = LedControl(12, 11, 10, 1);

void setup() {
  // controller.calib();
    controller.arm();

//    display.shutdown(0, false);
//    display.setIntensity(0, 8);
//    display.clearDisplay(0);

    // controller.speed(1330);
    Serial.begin(9600);
}

char out[20];
int tick = 0;
int neutral = 1450;
bool forward = true;
bool brake = false;

void loop() {
    int value = analogRead(1);
    value = ADC_filter.updateEstimate(value);

    if(value < 9)
        value = 9;
    else if (value > 395)
        value = 395;

    int pwm = map(value, 10, 395, 1500, 2000);

    controller.speed(pwm);

//    display.setDigit(0, 3, (pwm / 1000) % 10, false);
//    display.setDigit(0, 2, (pwm / 100) % 10, false);
//    display.setDigit(0, 1, (pwm / 10) % 10, false);
//    display.setDigit(0, 0, (pwm / 1) % 10, false);

    sprintf(out, "ADC: %d, PWM: %d", value, pwm);
//    sprintf(out, "%d", value);
    Serial.println(out);
    delay(100);
}

