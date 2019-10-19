#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
String targetString;
float pos_est;

void setup()
{
    Serial.begin(115200);
    Drive1.begin();

    while (!Serial) ; // wait for Arduino Serial Monitor to open   
    
    char output[255];

    Serial.println("ODriveArduino");
    Serial.println("Setting parameters...");    
    Serial7.write("w axis0.requested_state 8 \n");
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
        Serial.println();

        if (targetString == "pestimate")
        {
            pos_est = Drive1.motor[0].requestPosEstimate();
            Serial.println(pos_est);
        }

        else if (targetString == "reboot")
        {
            Drive1.motor[0].reboot(); 
        } 

        else if (targetString == "sconfig")
        {
            Drive1.motor[0].saveConfig();
        }

        else if (targetString == "econfig")
        {
            Drive1.motor[0].eraseConfig();
        }    

        else if (targetString == "errors")
        {
            Drive1.motor[0].checkAxisError();
        }

        else
        {
            int command = targetString.toInt();  
            char output[255];

            //wait for it, to send a new move command, 
            //we disable the closed loop state which causes the command to stop
            //then we renable closed loop and give it a new set point
            Serial7.write("w axis0.requested_state 1 \n");
            Serial7.write("w axis0.requested_state 8 \n");
            
            Drive1.motor[0].setTrapTarget(command);

        }
        targetString = "";
    }
}