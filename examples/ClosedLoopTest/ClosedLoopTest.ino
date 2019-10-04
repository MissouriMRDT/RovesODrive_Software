#include "RoveComm.h"
#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
RoveCommEthernetUdp RoveComm;
void setup()
{
    Serial.begin(115200);
    Drive1.begin();

    delay(100);
    Drive1.requestState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    //Drive1.motor[0].writeConfig();
    delay(100);
    RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET);
    Serial.println("Initialised");
}

void loop()
{
    if(Serial7.available())
    {
        Serial.print("--->");
        while(Serial7.available())
        {
            Serial.write(Serial7.read());
        }
    }

    if(Serial.available())
    {
        while(Serial.available())
        {
            Drive1.motor[0]Serial.write(Serial.read());
        }
    }
}