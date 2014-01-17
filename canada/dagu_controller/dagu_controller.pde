

void setup()
{
    pinMode(6, PWM);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);
}

boolean fwd = true;

void loop()
{
    if (SerialUSB.available())
    {
        SerialUSB.read();
        if (fwd)
        {
          digitalWrite(7, LOW);
        }
        else
        {
          digitalWrite(7, HIGH);
        }
        fwd = !fwd;
    }

    analogWrite(6, 65535);
    delay(100);
}
