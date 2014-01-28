uint8 PWM_PIN = 5;
uint8 DIR_PIN = 4;

void setup()
{
    pinMode(PWM_PIN, PWM);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(10, INPUT);
    digitalWrite(DIR_PIN, HIGH);
}

boolean fwd = true;

long times_fwd[4];
long times_bak[4];

int i = 1;

void loop()
{
    if (SerialUSB.available())
    {
        SerialUSB.read();
        if (fwd)
        {
          times_bak[i-1] -= micros();
          times_fwd[i%4] = micros();
          digitalWrite(DIR_PIN, LOW);
          
          long bak = 0;
          for (int j = 0; j < 4; ++j)
          {
            bak += times_bak[j];
          }
          SerialUSB.print("bak");
          SerialUSB.println(bak/4);
        }
        else
        {
          times_fwd[i%4] -= micros();
          times_bak[i%4] = micros();
          digitalWrite(DIR_PIN, HIGH);
          
          i = i%4+1;
          
          long fwd = 0;
          for (int j = 0; j < 4; ++j)
          {
            fwd += times_fwd[j];
          }
          SerialUSB.print("fwd");
          SerialUSB.println(fwd/4);
        }
        fwd = !fwd;
    }   

    pwmWrite(PWM_PIN, 20000);
    SerialUSB.println(analogRead(10));
    delay(30);
}
