#include "RoveComm.h"
//#include "RovesODrive.h"
void setup()
{
  Serial7.begin(115200);
  Serial.begin(115200);

   Serial7.println("");

  Serial7.write("w axis0.requested_state AXIS_STATE_IDLE\n");
  Serial7.write("w axis0.requested_state 5\n");
  delay(100);
   Serial.println("Initialized");
}

void loop()
{
    int before = millis();
    Serial7.flush();
    Serial7.write("r axis0.controller.vel_ramp_target\n");
 
    if(Serial7.available())
    {
        while(Serial7.available())
        {
            Serial.write(Serial7.read());
        }
        Serial.println(millis()-before);
    }
    delay(100);
    if(Serial7.available())
    {
        Serial7.flush();
    }
}