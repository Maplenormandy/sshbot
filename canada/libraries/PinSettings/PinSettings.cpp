#include "PinSettings.h"

void pinSetup(void)
{
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(BOARD_BUTTON_PIN, INPUT);

    pinMode(MOTOR_L_PWM, PWM);
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_PWM, PWM);
    pinMode(MOTOR_R_DIR, OUTPUT);

    pinMode(ROLLER_PWM, PWM);
    pinMode(ROLLER_DIR, OUTPUT);

    pinMode(SAS_PWM, PWM);
    pinMode(SAS_DIR, OUTPUT);
    pinMode(SAS_CUR, INPUT);

    pinMode(SCREW_PWM, PWM);
    pinMode(SCREW_DIR, OUTPUT);

    pinMode(KICK_SERVO, PWM);
    pinMode(PAC_SERVO, PWM);
    pinMode(GATE_G_SERVO, PWM);
    pinMode(GATE_R_SERVO, PWM);

    pinMode(ENC_L_A, INPUT);
    pinMode(ENC_L_B, INPUT);
    pinMode(ENC_R_A, INPUT);
    pinMode(ENC_R_B, INPUT);

    //pinMode(ENC_GND, OUTPUT);
    //pinMode(ENC_VCC, OUTPUT);

    pinMode(IR_L_FWD, INPUT);
    pinMode(IR_L_MID, INPUT);
    pinMode(IR_L_BAK, INPUT);
    pinMode(IR_R_FWD, INPUT);
    pinMode(IR_R_MID, INPUT);
    pinMode(IR_R_BAK, INPUT);
    pinMode(IR_FWD_L, INPUT);
    pinMode(IR_FWD_R, INPUT);

    //digitalWrite(ENC_GND, LOW);
    //digitalWrite(ENC_VCC, HIGH);
}
