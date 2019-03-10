#include "RoveComm.h"
#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
void setup()
{
    Drive1.begin();

    delay(100);
    Drive1.Motor[0].setControlMode(CTRL_MODE_SENSORLESS_VELOCITY_CONTROL);
    Drive1.Motor[0].setPolePairs(4);
    Drive1.Motor[0].setKV(480);
    Drive1.Motor[0].writeConfig();
}

void loop()
{

}