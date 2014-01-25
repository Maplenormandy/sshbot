
void setup()
{
  pinMode(20, INPUT);
}

void loop()
{
    if (SerialUSB.available())
    {
        SerialUSB.read();

        for (int i = 0; i < 32; ++i)
        {
            SerialUSB.println(analogRead(20));
            delay(50+random(0,10));
        }
    }

    delay(30);
}
