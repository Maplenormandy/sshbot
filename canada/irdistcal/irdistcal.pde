
void setup()
{
    //
}

void loop()
{
    if (SerialUSB.available())
    {
        SerialUSB.read();

        for (int i = 0; i < 32; ++i)
        {
            SerialUSB.println(analogRead(15));
            delay(50+random(0,10));
        }
    }

    delay(30);
}
