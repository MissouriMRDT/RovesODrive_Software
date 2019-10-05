#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
String targetString;

void setup()
{
    Serial.begin(115200);
    Drive1.begin();

    delay(100);
    Serial7.write("w axis0.requested_state AXIS_STATE_CLOSED_LOOP_CONTROL\n");
    Serial.println("Initialised");
    delay(100);
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

    while (Serial.available()) 
    {
        char c = Serial.read();  
        targetString += c; 
        delay(2);  
    }

    if (targetString.length() >0) 
    {
        Serial.println(targetString);  
        int command = targetString.toInt();  
        Drive1.motor[0].setTrapTarget(command);
        targetString = "";
    }
}