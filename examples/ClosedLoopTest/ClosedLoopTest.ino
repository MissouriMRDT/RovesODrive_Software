#include "RovesODrive.h"

RovesODrive Drive1;
String target;
float posEst, tvLimit, trapAPC, taLimit, tdLimit, vGain, pGain, viGain, vSetPoint, cLimit, vLimit, cCal, bRes, eCPR, vRR, cSP, vRT, wT, cprSP; 
int command, pos1, pos2, pos3, vel1, vel2, pPairs, mSide, cFF1, cFF2;
bool vRE = false;

Axis_State state, state2;
Control_Mode crtlMode;
Error_Axis axError;
Error_Motor mError;
Error_Encoder enError;
Error_Controller cError;

void setup()
{
    Serial.begin(115200);
    Drive1.begin(&Serial7);

    while (!Serial); // wait for Arduino Serial Monitor to open   
    
    Serial.println("ODriveArduino");

    Serial.println("Setting state to Closed Loop Control...");    

    Serial7.write("w axis0.requested_state 8 \n");

    Serial7.write("w axis1.requested_state 8 \n");

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
            Drive1.reboot();
            Serial.println("Done");
        }

        else if (target == "pestimate")
        {
            Serial.println("Requesting Position Estimate...");
            posEst =  Drive1.left.readPosEstimate();
            Serial.println(posEst);
            posEst = 0;
            Serial.println("Done");
        }

        else if(target == "loop")
        {
            Serial.println("We have initiated loop protocol");
            while(true)
            {
                Drive1.left.writePosSetPoint(-819315, 2, 0);
                do
                {
                    posEst = Drive1.left.readPosEstimate();
                }while(posEst > (-819315+10000));

                Drive1.left.writePosSetPoint(819315, 2, 0);
                do
                {
                    posEst = Drive1.left.readPosEstimate();
                }while(posEst < (819315-10000));
            }

        }

        else if (target == "aError")
        {
            Serial.println("Checking Axis Errors...");
            axError =  Drive1.left.checkAxisErrors();
            switch (axError)
            {
            case 0:
                Serial.println("ERROR_NONE_A");
                break;
            
            case 1:
                Serial.println("ERROR_INVALID_STATE");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 2:
                Serial.println("ERROR_DC_BUS_UNDER_VOLTAGE");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 3:
                Serial.println("ERROR_DC_BUS_OVER_VOLTAGE");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 4:
                Serial.println("ERROR_CURRENT_MEASUREMENT_TIMEOUT");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 5:
                Serial.println("ERROR_BRAKE_RESISTOR_DISARMED");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 6:
                Serial.println("ERROR_MOTOR_DISARMED");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 7:
                Serial.println("ERROR_MOTOR_FAILED");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 8:
                Serial.println("ERROR_SENSORLESS_ESTIMATOR_FAILED");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 9:
                Serial.println("ERROR_ENCODER_FAILED");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;

            case 10:
                Serial.println("ERROR_CONTROLLER_FAILED");
                Serial.println("Rebooting");
                Drive1.reboot();
                axError = ERROR_NONE_A;
                break;
            
            default:
                break;
            }
        }

        else if (target == "mError")
        {
            Serial.println("Checking Motor Errors");
            mError =  Drive1.left.checkMotorErrors();
            switch (mError)
            {
            case 0:
                Serial.println("ERROR_NONE_M");
                break;
            
            case 1:
                Serial.println("ERROR_PHASE_RESISTANCE_OUT_OF_RANGE");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 2:
                Serial.println("ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 3:
                Serial.println("ERROR_ADC_FAILED");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 4:
                Serial.println("ERROR_DRV_FAULT");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 5:
                Serial.println("ERROR_CONTROL_DEADLINE_MISSED");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 6:
                Serial.println("ERROR_NOT_IMPLEMENTED_MOTOR_TYPE");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 7:
                Serial.println("ERROR_BRAKE_CURRENT_OUT_OF_RANGE");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 8:
                Serial.println("ERROR_MODULATION_MAGNITUDE");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 9:
                Serial.println("ERROR_BRAKE_DEADTIME_VIOLATION");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 10:
                Serial.println("ERROR_UNEXPECTED_TIMER_CALLBACK");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;
            
            case 11:
                Serial.println("ERROR_CURRENT_SENSE_SATURATION");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 12:
                Serial.println("ERROR_INVERTER_OVER_TEMP");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            case 13:
                Serial.println("ERROR_CURRENT_UNSTABLE");
                Serial.println("Rebooting");
                Drive1.reboot();
                mError = ERROR_NONE_M;
                break;

            default:
                break;
            }
        }

        else if (target == "eError")
        {
            Serial.println("Checking Encoder Errors...");
            enError =  Drive1.left.checkEncoderErrors();
            switch (enError)
            {
            case 0:
                Serial.println("ERROR_NONE_E");
                break;
            
            case 1:
                Serial.println("ERROR_UNSTABLE_GAIN");
                Serial.println("Rebooting");
                Drive1.reboot();
                enError = ERROR_NONE_E;
                break;

            case 2:
                Serial.println("ERROR_CPR_OUT_OF_RANGE");
                Serial.println("Rebooting");
                Drive1.reboot();
                enError = ERROR_NONE_E;
                break;

            case 3:
                Serial.println("ERROR_NO_RESPONSE");
                Serial.println("Rebooting");
                Drive1.reboot();
                enError = ERROR_NONE_E;
                break;

            case 4:
                Serial.println("ERROR_UNSUPPORTED_ENCODER_MODE");
                Serial.println("Rebooting");
                Drive1.reboot();
                enError = ERROR_NONE_E;
                break;

            case 5:
                Serial.println("ERROR_ILLEGAL_HALL_STATE");
                Serial.println("Rebooting");
                Drive1.reboot();
                enError = ERROR_NONE_E;
                break;

            case 6:
                Serial.println("ERROR_INDEX_NOT_FOUND_YET");
                Serial.println("Rebooting");
                Drive1.reboot();
                enError = ERROR_NONE_E;
                break;
            
            default:
                break;
            }
        }

        else if (target == "cError")
        {
            Serial.println("Checking Controller Errors");
            cError =  Drive1.left.checkControllerErrors();
            switch (cError)
            {
            case 0:
                Serial.println("ERROR_NONE_C");
                break;
            
            case 1:
                Serial.println("ERROR_OVERSPEED");
                Serial.println("Rebooting");
                Drive1.reboot();
                cError = ERROR_NONE_C;
                break;
            
            default:
                break;
            }
        }

        else if (target == "sConfig")
        {
            Serial.println("Saving Configuration...");
            Serial.println("Rebooting");
            Drive1.left.saveConfig();
            Serial.println("Done");
        }

        else if (target == "eConfig")
        {
            Serial.println("Erasing Configuration...");
            Serial.println("Rebooting");
            Drive1.left.eraseConfig();
            Serial.println("Done");
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
            Drive1.left.writePosSetPoint(pos1, pos2, pos3);
            pos1 = 0;
            pos2 = 0;
            pos3 = 0;
            Serial.println("Done");
        }

        else if (target == "wTVL")
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
            Drive1.left.writeTrapVelocityLimit(tvLimit);
            
            Serial.println(tvLimit);
            tvLimit = 0;
            Serial.println("Done");

        }

        else if(target == "rTVL")
        {
            Serial.println("Requesting Trap Velocity Limit...");
            tvLimit = Drive1.left.readTrapVelocityLimit();
            Serial.println(tvLimit);
            tvLimit = 0;
            Serial.println("Done");
        }

        else if (target == "wTAL")
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
            Drive1.left.writeTrapAccelerationLimit(taLimit);
            taLimit = 0;
            Serial.println("Done");
        }

        else if (target == "rTAL")
        {
            Serial.println("Requesting Trap Acceleration Limit...");
            taLimit = Drive1.left.readTrapAccelerationLimit();
            Serial.println(taLimit);
            taLimit = 0;
            Serial.println("Done");
        }

        else if (target == "wTDL")
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
            Drive1.left.writeTrapDecelerationLimit(tdLimit);
            tdLimit = 0;
            Serial.println("Done");
        }       

        else if(target == "rTDL") 
        {
            Serial.println("Requesting Trap Deceleration Limit...");
            tdLimit = Drive1.left.readTrapDecelerationLimit();
            Serial.println(tdLimit);
            tdLimit = 0;
            Serial.println("Done");
        }

        else if (target == "wTAPC")
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
            Drive1.left.writeTrapAccelerationPerCounts(trapAPC);
            trapAPC = 0;
            Serial.println("Done");
        }

        else if(target == "rTAPC")
        {
            Serial.println("Requesting Trap Acceleration Per Counts...");
            trapAPC = Drive1.left.readTrapAccelerationPerCounts();
            Serial.println(trapAPC);
            trapAPC = 0;
            Serial.println("Done");
        }

        else if (target == "rCM")
        {
            Serial.println("Requesting Control Mode...");
            crtlMode = Drive1.left.readControlMode();
            Serial.println(crtlMode);
            crtlMode = CTRL_MODE_POSITION_CONTROL;
            Serial.println("Done");
        }

        else if (target == "wCM")
        {
            Serial.println("Enter Control Mode");
            target = "";
            while (!Serial.available());
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Control Mode...");
            crtlMode = (Control_Mode)target.toInt();
            Drive1.left.writeControlMode(crtlMode);
            crtlMode = CTRL_MODE_POSITION_CONTROL;
            Serial.println("Done");
        }

        else if (target == "rState")
        {
            Serial.println("Requesting State...");
            state = Drive1.left.readState();
            state2 = Drive1.right.readState();
            Serial.println(state);
            Serial.println(state2);
            Serial.println("Done");
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
            Drive1.left.writeState(state);
            Drive1.right.writeState(state);
            Serial.println("Done");
        }

        else if(target == "wVG")
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
            Drive1.left.writeVelocityGain(vGain);
            Serial.println(vGain);
            vGain = 0;
            Serial.println("Done");
        }

        else if (target == "rVG")
        {
            Serial.println("Reading Velocity Gain...");
            vGain = Drive1.left.readVelocityGain();
            Serial.println(vGain);
            vGain = 0;
            Serial.println("Done");
        }

        else if (target == "wPG")
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
            Drive1.left.writePositionGain(pGain);
            pGain = 0;
            Serial.println("Done");
        }

        else if (target == "rPG")
        {
            Serial.println("Reading Position Gain...");
            pGain = Drive1.left.readPositionGain();
            Serial.println(pGain);
            pGain = 0;
            Serial.println("Done");
        }

        else if (target == "wVIG")
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
            Drive1.left.writeVelocityIntegratorGain(viGain);
            viGain = 0;
            Serial.println("Done");
        }

        else if(target == "rVIG")
        {
            Serial.println("Reading Velocity Integrator Gain...");
            viGain = Drive1.left.readVelocityIntegratorGain();
            Serial.println(viGain);
            viGain = 0;
            Serial.println("Done");
        }

        else if(target == "wVP")
        {
            Serial.println("Enter target Velocity for left motor");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            vel1 = target.toInt();

            Serial.println("Enter Second Value for left motor");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            cFF1 = target.toInt();

            Serial.println("Enter target Velocity for right motor");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            vel2 = target.toInt();

            Serial.println("Enter Second Value for right motor");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }
            cFF2 = target.toInt();

            Serial.println("Setting Velocity Set Point...");

            Drive1.left.writeVelocitySetpoint(vel1, cFF1);
            Drive1.right.writeVelocitySetpoint(vel2, cFF2);
            Serial.println("Done");
        }

        else if(target == "rVP")
        {
            Serial.println("Reading Velocity Setpoint...");
            vSetPoint = Drive1.left.readVelocitySetpoint();
            Serial.println(vSetPoint);
            vSetPoint = 0;
            Serial.println("Done");
        }

        else if(target == "vCM")
        {
            Serial.println("Setting Velocity Control Mode...");
            Drive1.left.writeVelocityControlMode();
            Serial.println("Done");
        }

        else if (target == "wCL")
        {
            Serial.println("Enter Current Limit");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Current Limit...");
            Serial.println(target);
            cLimit = target.toFloat();
            Drive1.left.writeCurrentLimit(cLimit);
            cLimit = 0;
            Serial.println("Done");
        }

        else if(target == "rCL")
        {
            Serial.println("Reading Current Limit...");
            cLimit = Drive1.left.readCurrentLimit();
            Serial.println(cLimit);
            cLimit = 0;
            Serial.println("Done");
        }

        else if (target == "wVL")
        {
            Serial.println("Enter Velocity Limit");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Velocity Limit...");
            Serial.println(target);
            vLimit = target.toFloat();
            Drive1.left.writeVelocityLimit(vLimit);
            vLimit = 0;
            Serial.println("Done");
        }

        else if(target == "rVL")
        {
            Serial.println("Reading Velocity Limit...");
            vLimit = Drive1.left.readVelocityLimit();
            Serial.println(vLimit);
            vLimit = 0;
            Serial.println("Done");
        }

        else if (target == "wCC")
        {
            Serial.println("Enter Current Calibration");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Current Calibration...");
            Serial.println(target);
            cCal = target.toFloat();
            Drive1.left.writeCurrentCalibration(cCal);
            cCal = 0;
            Serial.println("Done");
        }

        else if(target == "rCC")
        {
            Serial.println("Reading Current Calibration...");
            cCal = Drive1.left.readCurrentCalibration();
            Serial.println(cCal);
            cCal = 0;
            Serial.println("Done");
        }

        else if (target == "wBR") // does not work 
        {
            Serial.println("Enter Brake Resistance");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Brake Resistance...");
            Serial.println(target);
            bRes = target.toFloat();
            Drive1.left.writeBrakeResistance(bRes);
            bRes = 0;
            Serial.println("Done");
        }

        else if(target == "rBR")
        {
            Serial.println("Reading Brake Resistance...");
            bRes = Drive1.left.readBrakeResistance();
            Serial.println(bRes);
            bRes = 0;
            Serial.println("Done");
        }

        else if (target == "wPP")
        {
            Serial.println("Enter Pole Pairs");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Pole Pairs...");
            Serial.println(target);
            pPairs = target.toFloat();
            Drive1.left.writePolePairs(pPairs);
            pPairs = 0;
            Serial.println("Done");
        }

        else if(target == "rPP")
        {
            Serial.println("Reading Pole Pairs...");
            pPairs = Drive1.left.readPolePairs();
            Serial.println(pPairs);
            pPairs = 0;
            Serial.println("Done");
        }

        else if (target == "wECPR")
        {
            Serial.println("Enter Encoder CPR");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Encoder CPR...");
            Serial.println(target);
            eCPR = target.toFloat();
            Drive1.left.writeEncoderCPR(eCPR);
            eCPR = 0;
            Serial.println("Done");
        }

        else if(target == "rECPR")
        {
            Serial.println("Reading Encoder CPR...");
            eCPR = Drive1.left.readEncoderCPR();
            Serial.println(eCPR);
            eCPR = 0;
            Serial.println("Done");
        }

        else if (target == "wVRR")
        {
            Serial.println("Enter Velocity RR");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Velocity RR...");
            Serial.println(target);
            vRR = target.toFloat();
            Drive1.left.writeVelocityRampRate(vRR);
            vRR = 0;
            Serial.println("Done");
        }

        else if(target == "rVRR")
        {
            Serial.println("Reading Velocity RR...");
            vRR = Drive1.left.readVelocityRampRate();
            Serial.println(vRR);
            vRR = 0;
            Serial.println("Done");
        }

        else if (target == "wCSP")
        {
            Serial.println("Enter Current SetPoint");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Current SetPoint...");
            Serial.println(target);
            cSP = target.toFloat();
            Drive1.left.writeCurrentSetPoint(cSP);
            cSP = 0;
            Serial.println("Done");
        }

        else if(target == "rCSP")
        {
            Serial.println("Reading Current SetPoint...");
            cSP = Drive1.left.readCurrentSetPoint();
            Serial.println(cSP);
            cSP = 0;
            Serial.println("Done");
        }

        else if (target == "wVRT")
        {
            Serial.println("Enter Velocity Ramp Target");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Velocity Ramp Target...");
            Serial.println(target);
            vRT = target.toFloat();
            Drive1.left.writeVelocityRampTarget(vRT);
            vRT = 0;
            Serial.println("Done");
        }

        else if(target == "rVRT")
        {
            Serial.println("Reading Velocity Ramp Target...");
            vRT = Drive1.left.readVelocityRampTarget();
            Serial.println(vRT);
            vRT = 0;
            Serial.println("Done");
        }

        else if(target == "rVRE")
        {
            Serial.println("Reading Velocity Ramp Enable...");
            vRE = Drive1.left.readVelocityRampEnable();
            Serial.println(vRE);
            vRE = false;
            Serial.println("Done");
        }

        else if (target == "wWT")
        {
            Serial.println("Enter Watchdog Timeout");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting Watchdog Timeout...");
            Serial.println(target);
            wT = target.toFloat();
            Drive1.left.writeWatchdogTimeout(wT);
            wT = 0;
            Serial.println("Done");
        }

        else if(target == "rWT")
        {
            Serial.println("Reading Watchdog Timeout...");
            wT = Drive1.left.readWatchdogTimeout();
            Serial.println(wT);
            wT = 0;
            Serial.println("Done");
        }

        else if (target == "wCPRSP")
        {
            Serial.println("Enter CPR SetPoint");
            target = "";
            while (!Serial.available()); 
            while (Serial.available()) 
            {
                char c = Serial.read();  
                target += c; 
                delay(2);  
            }

            Serial.println("Setting CPR SetPoint...");
            Serial.println(target);
            cprSP = target.toFloat();
            Drive1.left.writeCPRSetpoint(cprSP);
            cprSP = 0;
            Serial.println("Done");
        }

        else if(target == "rPCPR")
        {
            Serial.println("Reading Position CPR...");
            cprSP = Drive1.left.readPosCPR();
            Serial.println(cprSP);
            cprSP = 0;
            Serial.println("Done");
        }

        else 
        {
            command = target.toInt();

            Serial.println("Setting Trap Target...");
            Drive1.left.writeTrapTarget(command);
            command = 0;
            Serial.println("Done");
        }
        target = "";
    }
}