#ifndef PIN_SETTINGS_H_
#define PIN_SETTINGS_H_

#include <wirish.h>

const uint8 MOTOR_L_PWM = 0; // PWM
const uint8 MOTOR_L_DIR = 1; // OUTPUT

const uint8 MOTOR_R_PWM = 2; // PWM
const uint8 MOTOR_R_DIR = 3; // OUTPUT

const uint8 ENC_L_A = 32; // INPUT
const uint8 ENC_L_B = 31; // INPUT
const uint8 ENC_R_A = 34; // INPUT
const uint8 ENC_R_B = 33; // INPUT

const uint8 ENC_GND = 36; // OUTPUT, LOW
const uint8 ENC_VCC = 37; // OUTPUT, HIGH

const uint8 IRSENSORS[] = { 15, 17 };

void pinSetup(void);

#endif
