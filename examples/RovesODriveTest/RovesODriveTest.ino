#include "RoveComm.h"
#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
void setup()
{
    Serial.begin(115200);
    Drive1.begin();

    delay(100);
    Drive1.motor[0].setPolePairs(4);
    Drive1.motor[0].setKV(480);
    Drive1.motor[0].setControlMode(CTRL_MODE_SENSORLESS_VELOCITY_CONTROL);
    Drive1.motor[0].writeConfig();
    Serial.println("Initialised");
}

void loop()
{
    while(Serial7.available())
    {
        Serial.write(Serial7.read());
    }
}