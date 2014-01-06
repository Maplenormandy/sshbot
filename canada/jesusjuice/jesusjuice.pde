int PWM_PIN = 2;
int DIR_PIN = 3;
int powpow = 1000;

void setup()
{
    pinMode(BOARD_LED_PIN, PWM);
    pinMode(PWM_PIN, PWM);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, HIGH);
}

void loop()
{
    powpow += 5000;
    if (powpow > 65000)
    {
        powpow = 1000;
    }
    pwmWrite(BOARD_LED_PIN, powpow);
    pwmWrite(PWM_PIN, powpow);
    delay(1000);
}

