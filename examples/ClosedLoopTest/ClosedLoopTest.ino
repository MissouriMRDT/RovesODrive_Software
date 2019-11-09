#include "RovesODrive.h"

RovesODrive Drive1(&Serial7);
String target;
float posEst, tvLimit, trapAPC, taLimit, tdLimit, vGain, pGain, viGain; 
int command, pos1, pos2, pos3, vel1, vel2;

Axis_State state;
Control_Mode crtlMode;
Error_Axis axError;
Error_Motor mError;
Error_Encoder enError;
Error_Controller cError;

void setup()
{
    Serial.begin(115200);
    Drive1.begin();

    while (!Serial); // wait for Arduino Serial Monitor to open   
    
    Serial.println("ODriveArduino");
    Serial.println("Setting state to closed loop control...");    

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
            Serial.println("Requesting Position Estimate...");
            posEst =  Drive1.motor[0].readPosEstimate();
            Serial.println(posEst);
            posEst = 0;
        }

        else if(target == "loop")
        {
            Serial.println("We have initiated loop protocol");
            while(true)
            {
                Drive1.motor[0].writePosSetPoint(-819315, 2, 0);
                do
                {
                    posEst = Drive1.motor[0].readPosEstimate();
                }while(posEst > (-819315+10000));

                Drive1.motor[0].writePosSetPoint(819315, 2, 0);
                do
                {
                    posEst = Drive1.motor[0].readPosEstimate();
                }while(posEst < (819315-10000));
            }

        }

        else if (target == "axisError")
        {
            Serial.println("Checking Axis Errors...");
            axError =  Drive1.motor[0].checkAxisErrors();
            switch (axError)
            {
            case 0:
                Serial.println("ERROR_NONE_A");
                break;
            
            case 1:
                Serial.println("ERROR_INVALID_STATE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 2:
                Serial.println("ERROR_DC_BUS_UNDER_VOLTAGE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 3:
                Serial.println("ERROR_DC_BUS_OVER_VOLTAGE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 4:
                Serial.println("ERROR_CURRENT_MEASUREMENT_TIMEOUT");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 5:
                Serial.println("ERROR_BRAKE_RESISTOR_DISARMED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 6:
                Serial.println("ERROR_MOTOR_DISARMED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 7:
                Serial.println("ERROR_MOTOR_FAILED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 8:
                Serial.println("ERROR_SENSORLESS_ESTIMATOR_FAILED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 9:
                Serial.println("ERROR_ENCODER_FAILED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;

            case 10:
                Serial.println("ERROR_CONTROLLER_FAILED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                axError = ERROR_NONE_A;
                break;
            
            default:
                break;
            }
        }

        else if (target == "motorError")
        {
            Serial.println("Checking Motor Errors");
            mError =  Drive1.motor[0].checkMotorErrors();
            switch (mError)
            {
            case 0:
                Serial.println("ERROR_NONE_M");
                break;
            
            case 1:
                Serial.println("ERROR_PHASE_RESISTANCE_OUT_OF_RANGE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 2:
                Serial.println("ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 3:
                Serial.println("ERROR_ADC_FAILED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 4:
                Serial.println("ERROR_DRV_FAULT");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 5:
                Serial.println("ERROR_CONTROL_DEADLINE_MISSED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 6:
                Serial.println("ERROR_NOT_IMPLEMENTED_MOTOR_TYPE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 7:
                Serial.println("ERROR_BRAKE_CURRENT_OUT_OF_RANGE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 8:
                Serial.println("ERROR_MODULATION_MAGNITUDE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 9:
                Serial.println("ERROR_BRAKE_DEADTIME_VIOLATION");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 10:
                Serial.println("ERROR_UNEXPECTED_TIMER_CALLBACK");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;
            
            case 11:
                Serial.println("ERROR_CURRENT_SENSE_SATURATION");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 12:
                Serial.println("ERROR_INVERTER_OVER_TEMP");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            case 13:
                Serial.println("ERROR_CURRENT_UNSTABLE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                mError = ERROR_NONE_M;
                break;

            default:
                break;
            }
        }

        else if (target == "encoderError")
        {
            Serial.println("Checking Encoder Errors...");
            enError =  Drive1.motor[0].checkEncoderErrors();
            switch (enError)
            {
            case 0:
                Serial.println("ERROR_NONE_E");
                break;
            
            case 1:
                Serial.println("ERROR_UNSTABLE_GAIN");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                enError = ERROR_NONE_E;
                break;

            case 2:
                Serial.println("ERROR_CPR_OUT_OF_RANGE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                enError = ERROR_NONE_E;
                break;

            case 3:
                Serial.println("ERROR_NO_RESPONSE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                enError = ERROR_NONE_E;
                break;

            case 4:
                Serial.println("ERROR_UNSUPPORTED_ENCODER_MODE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                enError = ERROR_NONE_E;
                break;

            case 5:
                Serial.println("ERROR_ILLEGAL_HALL_STATE");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                enError = ERROR_NONE_E;
                break;

            case 6:
                Serial.println("ERROR_INDEX_NOT_FOUND_YET");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                enError = ERROR_NONE_E;
                break;
            
            default:
                break;
            }
        }

        else if (target == "controllerError")
        {
            Serial.println("Checking Controller Errors");
            cError =  Drive1.motor[0].checkControllerErrors();
            switch (cError)
            {
            case 0:
                Serial.println("ERROR_NONE_C");
                break;
            
            case 1:
                Serial.println("ERROR_OVERSPEED");
                Serial.println("Rebooting");
                Drive1.motor[0].reboot();
                cError = ERROR_NONE_C;
                break;
            
            default:
                break;
            }
        }

        else if (target == "saveConfig")
        {
            Serial.println("Saving Configuration...");
            Serial.println("Rebooting");
            Drive1.motor[0].saveConfig();
        }

        else if (target == "eraseConfig")
        {
            Serial.println("Erasing Configuration...");
            Serial.println("Rebooting");
            Drive1.motor[0].eraseConfig();
        }

        else if (target == "p")
        {
            Serial.println("Enter target Position");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            pos1 = target.toInt();

            Serial.println("Enter Second Value");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            pos2 = target.toInt();

            Serial.println("Enter third Value");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            pos3 = target.toInt();

            Serial.println("Setting Position Set Point...");
            Drive1.motor[0].writePosSetPoint(pos1, pos2, pos3);
        }

        else if (target == "trapVLimit")
        {
            Serial.println("Enter Trap Velocity Limit");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Trap Velocity Limit...");
            tvLimit = target.toFloat();
            Serial.println(tvLimit);

            Drive1.motor[0].writeTrapVelocityLimit(tvLimit);
            Serial.println("Done");

        }

        else if (target == "trapALimit")
        {
            Serial.println("Enter Trap Acceleration Limit");
            while (!Serial.available()); 
            target = "";
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Trap Acceleration Limit...");
            taLimit = target.toFloat();
            Drive1.motor[0].writeTrapAccelerationLimit(taLimit);
        }

        else if (target == "trapDLimit")
        {
            Serial.println("Enter Trap Deceleration Limit");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Trap Deceleration Limit...");
            tdLimit = target.toFloat();
            Drive1.motor[0].writeTrapDecelerationLimit(tdLimit);
        }        

        else if (target == "trapAPC")
        {
            Serial.println("Enter Trap Acceleration Per Counts");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Trap Acceleration Per Counts...");
            trapAPC = target.toFloat();
            Drive1.motor[0].writeTrapAccelerationPerCounts(trapAPC);
        }

        else if (target == "rControlMode")
        {
            Serial.println("Requesting Control Mode...");
            Drive1.motor[0].readControlMode();
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

            Serial.println("Setting Control Mode...");
            crtlMode = (Control_Mode)target.toInt();
            Drive1.motor[0].writeControlMode(crtlMode);
        }

        else if (target == "rState")
        {
            Serial.println("Requesting State...");
            Drive1.motor[0].readState();
        }

        else if (target == "wState")
        {
            Serial.println("Enter State");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting State...");
            state = (Axis_State)target.toInt();
            Drive1.motor[0].writeState(state);
        }

        else if(target == "vGain")
        {
            Serial.println("Enter Velocity Gain");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Velocity Gain...");
            Serial.println(target);
            vGain = target.toFloat();
            Serial.println(vGain, 6);

            Drive1.motor[0].writeVelocityGain(vGain);
        }

        else if (target == "rvGain")
        {
            Serial.println("Reading Velocity Gain...");
            Drive1.motor[0].readVelocityGain();
        }

        else if (target == "pGain")
        {
            Serial.println("Enter Position Gain");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Position Gain...");
            Serial.println(target);
            pGain = target.toFloat();
            Drive1.motor[0].writePositionGain(pGain);
        }

        else if (target == "rpGain")
        {
            Serial.println("Reading Position Gain...");
            Drive1.motor[0].readPositionGain();
        }

        else if (target == "viGain")
        {
            Serial.println("Enter Velocity Integrator Gain");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Velocity Integrator Gain...");
            Serial.println(target);
            viGain = target.toFloat();
            Drive1.motor[0].writeVelocityIntegratorGain(viGain);
        }

        else if(target == "rviGain")
        {
            Serial.println("Reading Velocity Integrator Gain...");
            Drive1.motor[0].readVelocityIntegratorGain();
        }

        else if(target == "velSetP")
        {
            Serial.println("Enter target Velocity");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            vel1 = target.toInt();

            Serial.println("Enter Second Value");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            vel2 = target.toInt();
            Serial.println("Setting Velocity Set Point...");
            Drive1.motor[0].writeVelocitySetpoint(vel1, vel2);
        }

        else 
        {
            command = target.toInt();

            Serial.println("Setting Trap Target...");
            Drive1.motor[0].writeTrapTarget(command);
        }
        target = "";
    }
}