#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
String targetString;
float pos_est;
Error_Axis axerror;
Error_Motor merror;
Error_Encoder enerror;
Error_Controller cerror;

void setup()
{
    Serial.begin(115200);
    Drive1.begin();

    while (!Serial); // wait for Arduino Serial Monitor to open   
    
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
        Serial.println(targetString);

        if (targetString == "reboot")
        {
            Drive1.motor[0].reboot();
        }

        else if (targetString == "pestimate")
        {
            pos_est =  Drive1.motor[0].requestPosEstimate();
            Serial.println(pos_est);
            pos_est = 0;
        }

        else if (targetString == "axerror")
        {
            axerror =  Drive1.motor[0].checkAxisErrors();
            Serial.println(axerror);

            if(axerror != ERROR_NONE_A)
            {
                Drive1.motor[0].reboot();
            }
            axerror = 0;
        }

        else if (targetString == "merror")
        {
            merror =  Drive1.motor[0].checkMotorErrors();
            Serial.println(merror);

            if(merror != ERROR_NONE_M)
            {
                Drive1.motor[0].reboot();
            }
            merror = 0;
        }

        else if (targetString == "enerror")
        {
            enerror =  Drive1.motor[0].checkEncoderErrors();
            Serial.println(enerror);

            if(enerror != ERROR_NONE_E)
            {
                Drive1.motor[0].reboot();
            }
            enerror = 0;
        }

        else if (targetString == "cerror")
        {
            cerror =  Drive1.motor[0].checkControllerErrors();
            Serial.println(cerror);
            
            if(cerror != ERROR_NONE_C)
            {
                Drive1.motor[0].reboot();
            }
            cerror = 0;
        }

        else if (targetString == "SaveConfig")
        {
            Drive1.motor[0].saveConfig();
        }

        else if (targetString == "EraseConfig")
        {
            Drive1.motor[0].eraseConfig();
        }

        else 
        {
            int command = targetString.toInt();
              
            char output[255];

            //wait for it, to send a new move command, 
            //we disable the closed loop state which causes the command to stop
            //then we renable closed loop and give it a new set point

            Drive1.motor[0].setTrapTarget(command);

            targetString = "";
        }
        targetString = "";
    }
}