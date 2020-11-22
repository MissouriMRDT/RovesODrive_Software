#include "RovesODrive.h"
#include <Energia.h>
#include <HardwareSerial.h>
#include "RovesOUtilities.h"

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


void writeODriveCommand(HardwareSerial* mySerial, char* id, char* param1, char* param2, char* param3, uint8_t axis) 
{
	char output[MAX_STRING_CHARS];

	sprintf(output, "%s", id);

	sprintf(output, "%s %d", output, axis);

	sprintf(output, "%s %s", output, param1);

	sprintf(output, "%s %s", output, param2);

	sprintf(output, "%s %s \n", output, param3);

	mySerial->write(output);
}

void RovesODrive::begin(HardwareSerial* mySerial)
{
	//init serial and motors
	m_serial = mySerial;
	left.motor_number = 0;
	left.m_serial = mySerial;
	right.motor_number = 1;
	right.m_serial = mySerial;

	//startup serial with proper baud rate
	m_serial->begin(115200);
	m_serial->write("\n");
}

String RovesODriveMotor::getSerial()
{
	String str = "";
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

void RovesODriveMotor::writeState(Axis_State state) 
{
	char data[2];
	intToChar(data, state);
	writeODriveConfig(m_serial, WRITE, WRITE_CURRENT_STATE, data, motor_number);
}

Axis_State RovesODriveMotor::readState()
{ 
	writeODriveConfig(m_serial, READ, READ_CURRENT_STATE, "", motor_number);
	return (Axis_State)getSerial().toInt();
}

void RovesODriveMotor::writeControlMode(Control_Mode mode)
{
	char data[2];
	intToChar(data, mode);
	writeODriveConfig(m_serial, WRITE, CONTROL_MODE, data, motor_number);
}

Control_Mode RovesODriveMotor::readControlMode()
{
	writeODriveConfig(m_serial, READ, CONTROL_MODE, "", motor_number);
	return (Control_Mode)getSerial().toInt();
}

void RovesODriveMotor::writeTrapVelocityLimit(float limit)
{
	writeODriveConfig(m_serial, WRITE, TRAP_VELOCITY_LIMIT , limit, motor_number);
}

float RovesODriveMotor::readTrapVelocityLimit()
{
	writeODriveConfig(m_serial, READ, TRAP_VELOCITY_LIMIT , "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeTrapAccelerationLimit(float limit)
{
	writeODriveConfig(m_serial, WRITE, TRAP_ACCELERATION_LIMIT, limit, motor_number);
}

float RovesODriveMotor::readTrapAccelerationLimit()
{
	writeODriveConfig(m_serial, READ, TRAP_ACCELERATION_LIMIT , "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeTrapDecelerationLimit(float limit )
{
	writeODriveConfig(m_serial, WRITE, TRAP_DECELERATION_LIMIT , limit, motor_number);
}

float RovesODriveMotor::readTrapDecelerationLimit()
{
	writeODriveConfig(m_serial, READ, TRAP_DECELERATION_LIMIT , "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeTrapAccelerationPerCounts(float limit )
{
	writeODriveConfig(m_serial, WRITE, TRAP_ACCELERATION_PER_COUNTS , limit, motor_number);
}

float RovesODriveMotor::readTrapAccelerationPerCounts()
{
	writeODriveConfig(m_serial, READ, TRAP_ACCELERATION_PER_COUNTS , "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeTrapTarget(int32_t target)
{
	char data[32];
	intToChar(data, target);
	writeODriveCommand(m_serial, MOTOR_TRAJ, data, "", "", motor_number);
	m_position = target;
}

void RovesODriveMotor::writePosSetPoint(int32_t position, int32_t velFF, int32_t crrtFF) // Sets the position setpoint and receives a position, velocity feed, and current feed parameters
{
	char data1[32], data2[32], data3[32];
	intToChar(data1, position);
	intToChar(data2, velFF);
	intToChar(data3, crrtFF);
	writeODriveCommand(m_serial, MOTOR_POS, data1, data2, data3, motor_number);
}


float RovesODriveMotor::readPosEstimate() // returns the Position estimate of the motor
{
	writeODriveConfig(m_serial, READ, POS_ESTIMATE, "", motor_number);
	return getSerial().toFloat();
}

void RovesODrive::reboot()
{
 	writeODriveCommand(m_serial, SYSTEM_REBOOT, "", "", "", 0);
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
	writeODriveConfig(m_serial, READ, AXIS_ERRORS, "", motor_number);
	return (Error_Axis)getSerial().toInt();
}

Error_Motor RovesODriveMotor::checkMotorErrors()
{
	writeODriveConfig(m_serial, READ, MOTOR_ERRORS, "", motor_number);
	return (Error_Motor)getSerial().toInt();
}

Error_Encoder RovesODriveMotor::checkEncoderErrors()
{
	writeODriveConfig(m_serial, READ, ENCODER_ERRORS, "", motor_number);
	return (Error_Encoder)getSerial().toInt();
}

Error_Controller RovesODriveMotor::checkControllerErrors()
{
	writeODriveConfig(m_serial, READ, CONTROLLER_ERRORS, "", motor_number);
	return (Error_Controller)getSerial().toInt();
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
	writeODriveConfig(m_serial, READ, VELOCITY_GAIN, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writePositionGain(float target)
{
	writeODriveConfig(m_serial, WRITE, POSITION_GAIN, target, motor_number);
}

float RovesODriveMotor::readPositionGain()
{
	writeODriveConfig(m_serial, READ, POSITION_GAIN, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeVelocityIntegratorGain(float target)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_iNTEGRATOR_GAIN, target, motor_number);
}

float RovesODriveMotor::readVelocityIntegratorGain()
{
	writeODriveConfig(m_serial, READ, VELOCITY_iNTEGRATOR_GAIN, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeVelocitySetpoint(float velPoint, float currentFF)
{
	char data1[12], data2[12];
	floatToChar(data1, velPoint, 6);
	floatToChar(data2, currentFF, 6);
	writeODriveCommand(m_serial, MOTOR_VEL, data1, data2, "", motor_number);
}

float RovesODriveMotor::readVelocitySetpoint()
{
	writeODriveConfig(m_serial, READ, VELOCITY_SETPOINT, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeCurrentLimit(float limit)
{
	writeODriveConfig(m_serial, WRITE, CURRENT_LIMIT, limit, motor_number);
}

float RovesODriveMotor::readCurrentLimit()
{
	writeODriveConfig(m_serial, READ, CURRENT_LIMIT, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeVelocityLimit(float limit)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_LIMIT, limit, motor_number);
}

float RovesODriveMotor::readVelocityLimit()
{
	writeODriveConfig(m_serial, READ, VELOCITY_LIMIT, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeCurrentCalibration(float limit)
{
	writeODriveConfig(m_serial, WRITE, CALIBRATION_CURRENT, limit, motor_number);
}

float RovesODriveMotor::readCurrentCalibration()
{
	writeODriveConfig(m_serial, READ, CALIBRATION_CURRENT, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeBrakeResistance(float bResist)
{
	writeODriveConfig(m_serial, WRITE, BRAKE_RESISTANCE, bResist, motor_number);
}

float RovesODriveMotor::readBrakeResistance()
{
	writeODriveConfig(m_serial, READ, BRAKE_RESISTANCE, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writePolePairs(int32_t pairs)
{
	char data[32];
	intToChar(data, pairs);
	writeODriveConfig(m_serial, WRITE, POLE_PAIRS, data, motor_number);
}

int32_t RovesODriveMotor::readPolePairs()
{
	writeODriveConfig(m_serial, READ, POLE_PAIRS, "", motor_number);
	return getSerial().toInt();
}

void RovesODriveMotor::writeEncoderCPR(float cpr)
{
	writeODriveConfig(m_serial, WRITE, CPR, cpr, motor_number);
}

float RovesODriveMotor::readEncoderCPR()
{
	writeODriveConfig(m_serial, READ, CPR, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeVelocityRampRate(float rRate)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_RAMP_RATE, rRate, motor_number);
}

float RovesODriveMotor::readVelocityRampRate()
{
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_RATE, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeCurrentSetPoint(float point)
{
	char data[12];
	floatToChar(data, point, 12);
	writeODriveCommand(m_serial, MOTOR_CURRENT, data, "", "", motor_number);
}

float RovesODriveMotor::readCurrentSetPoint()
{
	writeODriveConfig(m_serial, READ, CURRENT_SETPOINT, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeVelocityRampTarget(float target)
{
	writeODriveConfig(m_serial, WRITE, VELOCITY_RAMP_TARGET, target, motor_number);
}

float RovesODriveMotor::readVelocityRampTarget()
{
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_TARGET, "", motor_number);
	return getSerial().toFloat();
}

bool RovesODriveMotor::readVelocityRampEnable()
{
	writeODriveConfig(m_serial, READ, VELOCITY_RAMP_ENABLE, "", motor_number);
	if (getSerial() != "")
		return true;
	else
		return false;
	
}

void RovesODriveMotor::writeCPRSetpoint(bool state)
{
	char data[1];
	boolToChar(data, state);
	writeODriveConfig(m_serial, WRITE, CPR_SETPOINT, data, motor_number);
}

float RovesODriveMotor::readPosCPR() // returns position cpr estimate
{
	writeODriveConfig(m_serial, READ, POS_CPR, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::writeWatchdogTimeout(float time)
{
	char data[12];
	floatToChar(data, time, 4);
	writeODriveConfig(m_serial, WRITE, WATCHDOG, data, motor_number);
}

float RovesODriveMotor::readWatchdogTimeout()
{
	writeODriveConfig(m_serial, READ, WATCHDOG, "", motor_number);
	return getSerial().toFloat();
}

void RovesODriveMotor::updateWatchdog()
{
	writeODriveCommand(m_serial, WATCHDOG_UPDATE, "", "", "", motor_number);
}

float RovesODriveMotor::readCurrent()
{
	writeODriveConfig(m_serial, READ, READ_CURRENT, "", motor_number);
	return getSerial().toFloat();
}