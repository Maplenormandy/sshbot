void setup()
{
}

void loop()
{
    delay(100);
    while (SerialUSB.available())
    {
        char input = SerialUSB.read();
        SerialUSB.println(input);
    }
}

