#include <RovesODrive.h>
#include <Energia.h>
#include <HardwareSerial.h>
#include <RovesOUtilities.h>

void writeODriveConfig(HardwareSerial* mySerial, bool write_read, char* id, char* param, uint8_t axis)
{
	char output[MAX_STRING_CHARS];

	sprintf(output, "%s ", (write_read == WRITE)? "w":"r");

	sprintf(output, "%s%s%d.", output, "axis", axis);

	sprintf(output, "%s%s %s\n", output, id, param);
	mySerial->write(output);
	return;

}

void writeODriveConfig(HardwareSerial* mySerial, bool write_read, char* id, float& param, uint8_t axis)
{
	char output[MAX_STRING_CHARS];

	char data[20];

	floatToChar(data, param, 4);

	sprintf(output, "%s ", (write_read == WRITE)? "w":"r");

	sprintf(output, "%s%s%d.", output, "axis", axis);

	sprintf(output, "%s%s %s\n", output, id, data);
	mySerial->write(output);
	return;

}


void writeODriveCommand(HardwareSerial* mySerial, char* id, char* param1, char* param2, char* param3, uint8_t axis) //TODO: add two additional parameters for commands
{
	char output[MAX_STRING_CHARS];

	sprintf(output, "%s", id);

	sprintf(output, "%s %d", output, axis);

	sprintf(output, "%s %s", output, param1);

	sprintf(output, "%s %s", output, param2);

	sprintf(output, "%s %s \n", output, param3);

	mySerial->write(output);
}

String RovesODriveMotor::getSerial()
{
	String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    while( true ) 
	{
        while (!m_serial->available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = m_serial->read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}

void RovesODrive::begin()
{
	m_serial->begin(115200);
	delay(10);
	m_serial->write("\n");
}

void RovesODriveMotor::writeState(Axis_State state) 
{
	char data[10];
	intToChar(data, state);
	writeODriveConfig(m_serial, WRITE, WRITE_CURRENT_STATE, data, motor_number);
}

Axis_State RovesODriveMotor::readState()
{
	String state = ""; 
	writeODriveConfig(m_serial, READ, READ_CURRENT_STATE, "", motor_number);
	state = getSerial();
	return (Axis_State)state.toInt();
}

void RovesODriveMotor::writeControlMode(Control_Mode mode)
{
	char data[2];
	intToChar(data, mode);
	writeODriveConfig(m_serial, WRITE, CONTROL_MODE, data, motor_number);
}

Control_Mode RovesODriveMotor::readControlMode()
{
	String mode = "";
	writeODriveConfig(m_serial, READ, CONTROL_MODE, "", motor_number);
	mode = getSerial();
	return (Control_Mode)mode.toInt();
}

void RovesODriveMotor::writeTrapVelocityLimit(float limit)
{
	writeODriveConfig(m_serial, WRITE, TRAP_VELOCITY_LIMIT , limit, motor_number);
}

float RovesODriveMotor::readTrapVelocityLimit()
{
	String trapVLimit;
	writeODriveConfig(m_serial, READ, TRAP_VELOCITY_LIMIT , "", motor_number);
	trapVLimit = getSerial();
	return trapVLimit.toFloat();
}

void RovesODriveMotor::writeTrapAccelerationLimit(float limit)
{
	writeODriveConfig(m_serial, WRITE, TRAP_ACCELERATION_LIMIT, limit, motor_number);
}

float RovesODriveMotor::readTrapAccelerationLimit()
{
	String trapALimit = "";
	writeODriveConfig(m_serial, READ, TRAP_ACCELERATION_LIMIT , "", motor_number);
	trapALimit = getSerial();
	return trapALimit.toFloat();
}

void RovesODriveMotor::writeTrapDecelerationLimit(float limit )
{
	writeODriveConfig(m_serial, WRITE, TRAP_DECELERATION_LIMIT , limit, motor_number);
}

float RovesODriveMotor::readTrapDecelerationLimit()
{
	String trapDLimit = "";
	writeODriveConfig(m_serial, READ, TRAP_DECELERATION_LIMIT , "", motor_number);
	trapDLimit = getSerial();
	return trapDLimit.toFloat();
}

void RovesODriveMotor::writeTrapAccelerationPerCounts(float limit )
{
	writeODriveConfig(m_serial, WRITE, TRAP_ACCELERATION_PER_COUNTS , limit, motor_number);
}

float RovesODriveMotor::readTrapAccelerationPerCounts()
{
	String trapAPC = "";
	writeODriveConfig(m_serial, READ, TRAP_ACCELERATION_PER_COUNTS , "", motor_number);
	trapAPC = getSerial();
	return trapAPC.toFloat();
}

void RovesODriveMotor::writeTrapTarget(int32_t target)
{
	char data[12];
	intToChar(data, target);
	if ( target <= (m_position - 50) || target >= (m_position + 50) ) // checks if the new target is far enough from the previous position to move
	{
		writeODriveCommand(m_serial, MOTOR_TRAJ, data, "", "", motor_number);
		m_position = target;
	}
}

void RovesODriveMotor::writePosSetPoint(int32_t position, int32_t velFF, int32_t crrtFF) // Sets the position setpoint and receives a position, velocity feed, and current feed parameters
{
	char data1[12], data2[12], data3[12];
	intToChar(data1, position);
	intToChar(data2, velFF);
	intToChar(data3, crrtFF);
	writeODriveCommand(m_serial, MOTOR_POS, data1, data2, data3, motor_number);
}


float RovesODriveMotor::readPosEstimate() // returns the Position estimate of the motor
{
	String position = "";
	writeODriveConfig(m_serial, READ, POS_ESTIMATE, "", motor_number);
	position = getSerial();
	return position.toFloat();
}

void RovesODriveMotor::reboot()
{
 	writeODriveCommand(m_serial, SYSTEM_REBOOT, "", "", "", motor_number);
}

void RovesODriveMotor::saveConfig() 
{
    writeODriveCommand(m_serial, SAVE_CONFIG, "", "", "", motor_number);
}

void RovesODriveMotor::eraseConfig()
{
	writeODriveCommand(m_serial, ERASE_CONFIG, "", "", "", motor_number);
}

Error_Axis RovesODriveMotor::checkAxisErrors() 
{
	String error = "";
	writeODriveConfig(m_serial, READ, AXIS_ERRORS, "", motor_number);
	error = getSerial();
	return (Error_Axis)error.toInt();
}

Error_Motor RovesODriveMotor::checkMotorErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, MOTOR_ERRORS, "", motor_number);
	error = getSerial();
	return (Error_Motor)error.toInt();
}

Error_Encoder RovesODriveMotor::checkEncoderErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, ENCODER_ERRORS, "", motor_number);
	error = getSerial();
	return (Error_Encoder)error.toInt();
}

Error_Controller RovesODriveMotor::checkControllerErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, CONTROLLER_ERRORS, "", motor_number);
	error = getSerial();
	return (Error_Controller)error.toInt();
}

String* RovesODriveMotor::checkAllErrors()
{
	String* errors = new String[4];
	errors[0] = checkAxisErrors();
	errors[1] = checkMotorErrors();
	errors[2] = checkEncoderErrors();
	errors[3] = checkControllerErrors();
	return errors;
}

void RovesODriveMotor::writeVelocityGain(float target)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_GAIN, target, motor_number);
}

float RovesODriveMotor::readVelocityGain()
{
	String vGain = "";
	writeODriveConfig(m_serial, READ, VELOCITY_GAIN, "", motor_number);
	vGain = getSerial();
	return vGain.toFloat();
}

void RovesODriveMotor::writePositionGain(float target)
{
	writeODriveConfig(m_serial, WRITE, POSITION_GAIN, target, motor_number);
}

float RovesODriveMotor::readPositionGain()
{
	String pGain = "";
	writeODriveConfig(m_serial, READ, POSITION_GAIN, "", motor_number);
	pGain = getSerial();
	return pGain.toFloat();
}

void RovesODriveMotor::writeVelocityIntegratorGain(float target)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_iNTEGRATOR_GAIN, target, motor_number);
}

float RovesODriveMotor::readVelocityIntegratorGain()
{
	String vIGain = "";
	writeODriveConfig(m_serial, READ, VELOCITY_iNTEGRATOR_GAIN, "", motor_number);
	vIGain = getSerial();
	return vIGain.toFloat();
}

void RovesODriveMotor::writeVelocityControlMode()
{
	Control_Mode mode = CTRL_MODE_VELOCITY_CONTROL;
	writeControlMode(mode);
}

void RovesODriveMotor::writeVelocitySetpoint(float velPoint, float currentFF)
{
	char data1[30], data2[30];
	intToChar(data1, velPoint);
	intToChar(data2, currentFF);
	writeODriveCommand(m_serial, MOTOR_VEL, data1, data2, "", motor_number);
}

float RovesODriveMotor::readVelocitySetpoint()
{
	String setPoint = "";
	writeODriveConfig(m_serial, READ, VELOCITY_SETPOINT, "", motor_number);
	setPoint = getSerial();
	return setPoint.toFloat();
}

void RovesODriveMotor::writeCurrentLimit(float limit)
{
	writeODriveConfig(m_serial, WRITE, CURRENT_LIMIT, limit, motor_number);
}

float RovesODriveMotor::readCurrentLimit()
{
	String cLimit = "";
	writeODriveConfig(m_serial, READ, CURRENT_LIMIT, "", motor_number);
	cLimit = getSerial();
	return cLimit.toFloat();
}

void RovesODriveMotor::writeVelocityLimit(float limit)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_LIMIT, limit, motor_number);
}

float RovesODriveMotor::readVelocityLimit()
{
	String vLimit = "";
	writeODriveConfig(m_serial, READ, VELOCITY_LIMIT, "", motor_number);
	vLimit = getSerial();
	return vLimit.toFloat();
}

void RovesODriveMotor::writeCurrentCalibration(float limit)
{
	writeODriveConfig(m_serial, WRITE, CALIBRATION_CURRENT, limit, motor_number);
}

float RovesODriveMotor::readCurrentCalibration()
{
	String cCalibration = "";
	writeODriveConfig(m_serial, READ, CALIBRATION_CURRENT, "", motor_number);
	cCalibration = getSerial();
	return cCalibration.toFloat();
}

void RovesODriveMotor::writeBrakeResistance(float bResist)
{
	writeODriveConfig(m_serial, WRITE, BRAKE_RESISTANCE, bResist, motor_number);
}

float RovesODriveMotor::readBrakeResistance()
{
	String brakeResistance = "";
	writeODriveConfig(m_serial, READ, BRAKE_RESISTANCE, "", motor_number);
	brakeResistance = getSerial();
	return brakeResistance.toFloat();
}

void RovesODriveMotor::writePolePairs(int32_t pairs)
{
	char data[12];
	intToChar(data, pairs);
	writeODriveConfig(m_serial, WRITE, POLE_PAIRS, data, motor_number);
}

int32_t RovesODriveMotor::readPolePairs()
{
	String pPairs = "";
	writeODriveConfig(m_serial, READ, POLE_PAIRS, "", motor_number);
	pPairs = getSerial();
	return pPairs.toInt();
}

void RovesODriveMotor::writeEncoderCPR(float cpr)
{
	writeODriveConfig(m_serial, WRITE, CPR, cpr, motor_number);
}

float RovesODriveMotor::readEncoderCPR()
{
	String enCPR = "";
	writeODriveConfig(m_serial, READ, CPR, "", motor_number);
	enCPR = getSerial();
	return enCPR.toFloat();
}

void RovesODriveMotor::writeVelocityRampRate(float rRate)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_RAMP_RATE, rRate, motor_number);
}

float RovesODriveMotor::readVelocityRampRate()
{
	String vRampRate = "";
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_RATE, "", motor_number);
	vRampRate = getSerial();
	return vRampRate.toFloat();
}

void RovesODriveMotor::writeCurrentSetPoint(float point)
{
	char data[12];
	floatToChar(data, point, 12);
	writeODriveCommand(m_serial, MOTOR_CURRENT, data, "", "", motor_number);
}

float RovesODriveMotor::readCurrentSetPoint()
{
	String cSP = "";
	writeODriveConfig(m_serial, READ, CURRENT_SETPOINT, "", motor_number);
	cSP = getSerial();
	return cSP.toFloat();
}

void RovesODriveMotor::writeVelocityRampTarget(float target)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_RAMP_TARGET, target, motor_number);
}

float RovesODriveMotor::readVelocityRampTarget()
{
	String vRampTarget = "";
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_TARGET, "", motor_number);
	vRampTarget = getSerial();
	return vRampTarget.toFloat();
}

bool RovesODriveMotor::readVelocityRampEnable()
{
	String vRampRateEnable = "";
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_ENABLE, "", motor_number);
	vRampRateEnable = getSerial();
	if (vRampRateEnable != "")
		return true;
	else
		return false;
	
}

void RovesODriveMotor::writeCPRSetpoint(bool state)
{
	char data[12];
	boolToChar(data, state);
	writeODriveConfig(m_serial, WRITE, CPR_SETPOINT, data, motor_number);
}

float RovesODriveMotor::readPosCPR() // returns position cpr estimate
{
	String position = "";
	writeODriveConfig(m_serial, READ, POS_CPR, "", motor_number);
	position = getSerial();
	return position.toFloat();
}

void RovesODriveMotor::writeWatchdogTimeout(int32_t time)
{
	char data[12];
	intToChar(data, time);
	writeODriveConfig(m_serial, WRITE, WATCHDOG, data, motor_number);
}

int32_t RovesODriveMotor::readWatchdogTimeout()
{
	String wTimeout = "";
	writeODriveConfig(m_serial, READ, WATCHDOG, "", motor_number);
	wTimeout = getSerial();
	return wTimeout.toInt();
}

