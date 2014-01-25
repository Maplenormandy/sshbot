

void setup()
{
    pinMode(6, PWM);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);
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
          digitalWrite(7, LOW);
          
          long bak = 0;
          for (int j = 0; j < 4; ++j)
          {
            bak += times_bak[j];
          }
          SerialUSB.println("bak");
          SerialUSB.println(bak/4);
        }
        else
        {
          times_fwd[i%4] -= micros();
          times_bak[i%4] = micros();
          digitalWrite(7, HIGH);
          
          i = i%4+1;
          
          long fwd = 0;
          for (int j = 0; j < 4; ++j)
          {
            fwd += times_fwd[j];
          }
          SerialUSB.println("fwd");
          SerialUSB.println(fwd/4);
        }
        fwd = !fwd;
    }
    
    

    analogWrite(6, 10000);
    delay(30);
}
