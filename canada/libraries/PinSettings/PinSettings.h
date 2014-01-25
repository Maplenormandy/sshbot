#ifndef PIN_SETTINGS_H_
#define PIN_SETTINGS_H_

#include <wirish.h>

const uint8 MOTOR_L_PWM = 0; // PWM
const uint8 MOTOR_L_DIR = 1; // OUTPUT

const uint8 MOTOR_R_PWM = 2; // PWM
const uint8 MOTOR_R_DIR = 3; // OUTPUT

const uint8 ROLLER_PWM = 4;
const uint8 ROLLER_DIR = 5;

const uint8 SAS_PWM = 6;
const uint8 SAS_DIR = 7;

const uint8 SCREW_PWM = 4;
const uint8 SCREW_DIR = 5;

const uint8 KICK_SERVO = 8;
const uint8 PAC_SERVO = 9;
const uint8 GATE_G_SERVO = 10;
const uint8 GATE_R_SERVO = 11;

const uint8 ENC_L_A = 32; // INPUT
const uint8 ENC_L_B = 31; // INPUT
const uint8 ENC_R_A = 34; // INPUT
const uint8 ENC_R_B = 33; // INPUT

const uint8 ENC_GND = 36; // OUTPUT, LOW
const uint8 ENC_VCC = 37; // OUTPUT, HIGH

const uint8 IR_L_FWD = 15;
const uint8 IR_L_MID = 16;
const uint8 IR_L_BAK = 17;
const uint8 IR_R_FWD = 18;
const uint8 IR_R_MID = 19;
const uint8 IR_R_BAK = 20;
const uint8 IR_FWD_L = 27;
const uint8 IR_FWD_R = 28;


void pinSetup(void);

#endif
