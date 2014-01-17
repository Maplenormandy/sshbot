#include "PinSettings.h"

void pinSetup(void)
{
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(BOARD_BUTTON_PIN, INPUT);

    pinMode(MOTOR_L_PWM, PWM);
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_PWM, PWM);
    pinMode(MOTOR_R_DIR, OUTPUT);

    pinMode(ENC_L_A, INPUT);
    pinMode(ENC_L_B, INPUT);
    pinMode(ENC_R_A, INPUT);
    pinMode(ENC_R_B, INPUT);

    pinMode(ENC_GND, OUTPUT);
    pinMode(ENC_VCC, OUTPUT);

    pinMode(IR_L_FWD, INPUT);
    pinMode(IR_L_MID, INPUT);
    pinMode(IR_L_BAK, INPUT);
    pinMode(IR_FWD, INPUT);

    digitalWrite(ENC_GND, LOW);
    digitalWrite(ENC_VCC, HIGH);
}
