#include "RoveComm.h"
#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
RoveCommEthernetUdp RoveComm;
void setup()
{
    Serial.begin(115200);
    Drive1.begin();

    delay(100);
    Drive1.motor[0].setPolePairs(4);
    Drive1.motor[0].setKV(480);
    Drive1.motor[0].setControlMode(CTRL_MODE_SENSORLESS_VELOCITY_CONTROL);
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
    
    rovecomm_packet packet = RoveComm.read();
    switch(packet.data_id)
    {
        case(RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID):
            Drive1.motor[0].setSpeed(packet.data[0]);
            Drive1.motor[1].setSpeed(packet.data[1]);
            Serial.println(packet.data[0]);
    }
}