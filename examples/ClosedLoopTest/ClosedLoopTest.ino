#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
String target;
float pos_est, set_pos, tvlimit, trapAPC, talimit, tdlimit;
int command, crtl_mode;
Axis_State state;
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
        target += c; 
        delay(2);  
    }

    if (target.length() >0) 
    {
        Serial.println(target);

        if (target == "reboot")
        {
            Serial.println("Rebooting");
            Drive1.motor[0].reboot();
        }

        else if (target == "pestimate")
        {
            serial.println("Requesting Position Estimate...");
            pos_est =  Drive1.motor[0].requestPosEstimate();
            Serial.println(pos_est);
            pos_est = 0;
        }

        else if (target == "axisError")
        {
            serial.println("Checking Axis Errors...");
            axerror =  Drive1.motor[0].checkAxisErrors();
            Serial.println(axerror);

            if(axerror != ERROR_NONE_A)
            {
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axerror = 0;
            }
        }

        else if (target == "motorError")
        {
            serial.println("Checking Motor Errors");
            merror =  Drive1.motor[0].checkMotorErrors();
            Serial.println(merror);

            if(merror != ERROR_NONE_M)
            {
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                merror = 0;
            }
        }

        else if (target == "encoderError")
        {
            serial.println("Checking Encoder Errors...");
            enerror =  Drive1.motor[0].checkEncoderErrors();
            Serial.println(enerror);

            if(enerror != ERROR_NONE_E)
            {
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                enerror = 0;
            }
        }

        else if (target == "controllerError")
        {
            serial.println("Checking Controller Errors");
            cerror =  Drive1.motor[0].checkControllerErrors();
            Serial.println(cerror);
            
            if(cerror != ERROR_NONE_C)
            {
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                cerror = 0;
            }
        }

        else if (target == "saveConfig")
        {
            serial.println("Saving Configuration...");
            Serial.println("Rebooting");
            Drive1.motor[0].saveConfig();
        }

        else if (target == "eraseConfig")
        {
            serial.println("Erasing Configuration...");
            Serial.println("Rebooting");
            Drive1.motor[0].eraseConfig();
        }

        else if (target == "positionSetPoint")
        {
            serial.println("Enter target");
            target = "";
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            serial.println("Setting Position Set Point...");
            set_pos = target.toFloat();
            Drive1.motor[0].setPosSetPoint(set_pos);
        }

        else if (target == "trapVLimit")
        {
            Serial.println("Enter Trap Velocity Limit");
            target = "";
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Trap Velocity Limit...");
            tvlimit = target.toFloat();
            Drive1.motor[0].setTrapVelocityLimit(tvlimit);
        }

        else if (target == "trapALimit")
        {
            Serial.println("Enter Trap Acceleration Limit");
            target = "";
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            serial.println("Setting Trap Acceleration Limit...");
            talimit = target.toFloat();
            Drive1.motor[0].setTrapAccelerationLimit(talimit);
        }

        else if (target == "trapDLimit")
        {
            Serial.println("Enter Trap Deceleration Limit");
            target = "";
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            serial.println("Setting Trap Deceleration Limit...");
            tdlimit = target.toFloat();
            Drive1.motor[0].setTrapDecelerationLimit(tdlimit);
        }        

        else if (target == "trapAPC")
        {
            Serial.println("Enter Trap Acceleration Per Counts");
            target = "";
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            serial.println("Setting Trap Acceleration Per Counts...");
            trapAPC = target.toFloat();
            Drive1.motor[0].setTrapAccelerationPerCounts(trapAPC);
        }

        else if (target == "rControlMode")
        {
            serial.println("Requesting Control Mode...");
            Drive1.motor[0].requestControlMode();
        }

        else if (target == "wControlMode")
        {
            Serial.println("Enter Control Mode");
            target = "";
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            serial.println("Setting Control Mode...");
            crtl_mode = target.toInt();
            Drive1.motor[0].WriteControlMode(crtl_mode);
        }

        else if (target == "rState")
        {
            serial.println("Requesting State...");
            Drive1.motor[0].requestState();
        }

        else if (target == "wState")
        {
            Serial.println("Enter State");
            target = "";
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            serial.println("Setting State...");
            state = target.toInt();
            Drive1.motor[0].writeState(state);
        }

        else 
        {
            command = target.toInt();
            char output[255];

            serial.println("Setting Trap Target...");
            Drive1.motor[0].setTrapTarget(command);
        }
        target = "";
    }
}