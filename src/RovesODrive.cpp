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
}

void writeODriveCommand(HardwareSerial* mySerial, char* id, char* param1, char* param2, char* param3, uint8_t axis) //TODO: add two additional parameters for commands
{
	char output[MAX_STRING_CHARS];

	sprintf(output, "%s", id);

	sprintf(output, "%s %d", output, axis);

	sprintf(output, "%s %s", output, param1);

	sprintf(output, "%s %s", output, param2);

	sprintf(output, "%s %s \n", output, param3);

	Serial.println(output);
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
	Serial.println("Beginning");
	m_serial->begin(115200);
	delay(10);
	m_serial->write("\n");
	Serial.println("Drive Serial Init");
}

void RovesODriveMotor::writeState(Axis_State state) 
{
	char data[2];
	intToChar(data, state);
	writeODriveConfig(m_serial, WRITE, WRITE_CURRENT_STATE, data, motor_number);
}

void RovesODriveMotor::readState()
{
	writeODriveConfig(m_serial, READ, READ_CURRENT_STATE, "", motor_number);
}

void RovesODriveMotor::writeControlMode(Control_Mode mode)
{
	char data[2];
	intToChar(data, mode);
	writeODriveConfig(m_serial, WRITE, CONTROL_MODE, data, motor_number);
}

void RovesODriveMotor::readControlMode()
{
	writeODriveConfig(m_serial, READ, CONTROL_MODE, "", motor_number);
}

void RovesODriveMotor::writeTrapVelocityLimit(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, TRAP_VELOCITY_LIMIT , data, motor_number);
}

void RovesODriveMotor::readTrapVelocityLimit()
{
	writeODriveConfig(m_serial, READ, TRAP_VELOCITY_LIMIT , "", motor_number);
}

void RovesODriveMotor::writeTrapAccelerationLimit(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, TRAP_ACCELERATION_LIMIT, data, motor_number);
}

void RovesODriveMotor::readTrapAccelerationLimit()
{
	writeODriveConfig(m_serial, READ, TRAP_ACCELERATION_LIMIT , "", motor_number);
}

void RovesODriveMotor::writeTrapDecelerationLimit(float limit )
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, TRAP_DECELERATION_LIMIT , data, motor_number);
}

void RovesODriveMotor::readTrapDecelerationLimit()
{
	writeODriveConfig(m_serial, READ, TRAP_DECELERATION_LIMIT , "", motor_number);
}

void RovesODriveMotor::writeTrapAccelerationPerCounts(float limit )
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, TRAP_ACCELERATION_PER_COUNTS , data, motor_number);
}

void RovesODriveMotor::readTrapAccelerationPerCounts()
{
	writeODriveConfig(m_serial, READ, TRAP_ACCELERATION_PER_COUNTS , "", motor_number);
}

void RovesODriveMotor::writeTrapTarget(int32_t target)
{
	char data[12];
	intToChar(data, target);
	if ( target <= (m_position - 50) || target >= (m_position + 50) ) // checks if the new target is far enough from the prevois position to move
	{
		writeODriveConfig(m_serial, WRITE, CONTROL_MODE, "3", motor_number);
		writeODriveCommand(m_serial, MOTOR_TRAJ, data, "", "", motor_number);
		m_position = target;
	}
	else
	{
		Serial.println("We are going to that position already");
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
	char data[12];
	floatToChar(data, target, 12);
	writeODriveConfig(m_serial, WRITE, VELOCITY_GAIN, data, motor_number);
}

void RovesODriveMotor::readVelocityGain()
{
	writeODriveConfig(m_serial, READ, VELOCITY_GAIN, "", motor_number);
}

void RovesODriveMotor::writePositionGain(float target)
{
	char data[12];
	floatToChar(data, target, 12);
	writeODriveConfig(m_serial, WRITE, POSITION_GAIN, data, motor_number);
}

void RovesODriveMotor::readPositionGain()
{
	writeODriveConfig(m_serial, READ, POSITION_GAIN, "", motor_number);
}

void RovesODriveMotor::writeVelocityIntegratorGain(float target)
{
	char data[12];
	floatToChar(data, target, 12);
	writeODriveConfig(m_serial, WRITE, VELOCITY_iNTEGRATOR_GAIN, data, motor_number);
}

void RovesODriveMotor::readVelocityIntegratorGain()
{
	writeODriveConfig(m_serial, READ, VELOCITY_iNTEGRATOR_GAIN, "", motor_number);
}

void RovesODriveMotor::writeVelocityControlMode()
{
	Control_Mode mode = CTRL_MODE_VELOCITY_CONTROL;
	writeControlMode(mode);
}

void RovesODriveMotor::writeVelocitySetpoint(float velPoint, float currentFF)
{
	char data1[12], data2[12];
	intToChar(data1, velPoint);
	intToChar(data2, currentFF);
	writeODriveCommand(m_serial, MOTOR_VEL, data1, data2, "", motor_number);
}

void RovesODriveMotor::readVelocitySetpoint()
{
	writeODriveConfig(m_serial, READ, VELOCITY_SETPOINT, "", motor_number);
}

void RovesODriveMotor::writeCurrentLimit(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, CURRENT_LIMIT, data, motor_number);
}

void RovesODriveMotor::readCurrentLimit()
{
	writeODriveConfig(m_serial, READ, CURRENT_LIMIT, "", motor_number);
}

void RovesODriveMotor::writeVelocityLimit(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, VELOCITY_LIMIT, data, motor_number);
}

void RovesODriveMotor::readVelocityLimit()
{
	writeODriveConfig(m_serial, READ, VELOCITY_LIMIT, "", motor_number);
}

void RovesODriveMotor::writeCurrentCalibration(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, CALIBRATION_CURRENT, data, motor_number);
}

void RovesODriveMotor::readCurrentCalibration()
{
	writeODriveConfig(m_serial, READ, CALIBRATION_CURRENT, "", motor_number);
}

void RovesODriveMotor::writeBrakeResistance(float bResist)
{
	char data[12];
	floatToChar(data, bResist, 12);
	writeODriveConfig(m_serial, WRITE, BRAKE_RESISTANCE, data, motor_number);
}

void RovesODriveMotor::readBrakeResistance()
{
	writeODriveConfig(m_serial, READ, BRAKE_RESISTANCE, "", motor_number);
}

void RovesODriveMotor::writePolePairs(int32_t pairs)
{
	char data[12];
	intToChar(data, pairs);
	writeODriveConfig(m_serial, WRITE, POLE_PAIRS, data, motor_number);
}

void RovesODriveMotor::readPolePairs()
{
	writeODriveConfig(m_serial, READ, POLE_PAIRS, "", motor_number);
}

void RovesODriveMotor::writeEncoderCPR(float cpr)
{
	char data[12];
	floatToChar(data, cpr, 12);
	writeODriveConfig(m_serial, WRITE, CPR, data, motor_number);
}

void RovesODriveMotor::readEncoderCPR()
{
	writeODriveConfig(m_serial, READ, CPR, "", motor_number);
}

void RovesODriveMotor::moveToPos(float pos) // moves the motor to an absolute positon
{
	char data[12];
	floatToChar(data, pos, 12);
	writeODriveConfig(m_serial, WRITE, MOVE_TO_POS, data, motor_number);
}

void RovesODriveMotor::moveIncremental(float incr) // moves the motor to a positon based off its current position
{
	char data[12];
	floatToChar(data, incr, 12);
	writeODriveConfig(m_serial, WRITE, MOVE_TO_POS, data, motor_number);
}

void RovesODriveMotor::writeVelocityRampRate(float rRate)
{
	char data[12];
	floatToChar(data, rRate, 12);
	writeODriveConfig(m_serial, WRITE, VELOCITY_RAMP_RATE, data, motor_number);
}

void RovesODriveMotor::readVelocityRampRate()
{
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_RATE, "", motor_number);
}

void RovesODriveMotor::writeCurrentSetPoint(float point)
{
	char data[12];
	floatToChar(data, point, 12);
	writeODriveCommand(m_serial, MOTOR_CURRENT, data, "", "", motor_number);
}

void RovesODriveMotor::writeVelocityRampTarget(float target)
{
	char data[12];
	floatToChar(data, target, 12);
	writeVelocityRampEnable(true);
	writeODriveConfig(m_serial, WRITE, VELOCITY_RAMP_TARGET, data, motor_number);
}

void RovesODriveMotor::readVelocityRampTarget()
{
	writeVelocityRampEnable(true);
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_TARGET, "", motor_number);
}

void RovesODriveMotor::writeVelocityRampEnable(bool state)
{
	char data[12];
	boolToChar(data, state);
	writeODriveConfig(m_serial, WRITE, VELOCITY_RAMP_ENABLE, data, motor_number);
}

void RovesODriveMotor::readVelocityRampEnable()
{
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_ENABLE, "", motor_number);
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

void RovesODriveMotor::readWatchdogTimeout()
{
	writeODriveConfig(m_serial, READ, WATCHDOG, "", motor_number);
}

