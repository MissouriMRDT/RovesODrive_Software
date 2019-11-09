#include "RovesODrive.h"
#include <Energia.h>
#include <HardwareSerial.h>

int charToInt(char input[])
{
	int value = 0;
	int i = 0;
	while(input[i] != '\0')
	{
		value *= 10;
		value += input[i] - '0';
		i++;
	}
	return(value);
}

float charToFloat(char input[])
{
	float value = 0;
	int i = 0;
	while(input[i] != '.')
	{
		value *= 10;
		value += input[i] - '0';
		i++;
	}
	int j = i++;
	while(input[i] != '\0')
	{
		value *= 10;
		value += input[i] - '0';
		i++;
	}
	return(value/(10*(i-j)));
}

bool charToBool(char input[])
{
	return(1);
}

void intToChar(char* output, int value)
{
	sprintf(output, "%d", value);
}

void boolToChar(char* output, bool value)
{
	sprintf(output, "%d", value);
}

void floatToChar(char* output, float value, uint8_t precision)
{
	float frac_part, int_part;
	frac_part = modf(value, &int_part);
	
	sprintf(output, "%d.", (int)int_part);

	for(int i = 0; i<precision; i++)
	{
		frac_part *= 10;
		frac_part = modf(frac_part, &int_part);
		sprintf(output, "%s%d", output, (int)int_part);
	}
}

void writeODriveConfig(HardwareSerial* mySerial, bool write_read, char* id, char* param, uint8_t axis)
{
	char output[255];

	sprintf(output, "%s ", (write_read == WRITE)? "w":"r");

	sprintf(output, "%s%s%d.", output, "axis", axis);

	sprintf(output, "%s%s %s\n", output, id, param);
	mySerial->write(output);
}

void writeODriveCommand(HardwareSerial* mySerial, char* id, char* param1, char* param2, char* param3, uint8_t axis) //TODO: add two additional parameters for commands
{
	char output[255];

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
	writeODriveConfig(m_serial, WRITE, SET_CURRENT_STATE_TAG, data, motor_number);
}

void RovesODriveMotor::readState()
{
	writeODriveConfig(m_serial, READ, GET_CURRENT_STATE_TAG, "", motor_number);
}

void RovesODriveMotor::writeControlMode(Control_Mode mode)
{
	char data[2];
	intToChar(data, mode);
	writeODriveConfig(m_serial, WRITE, CONTROL_MODE_TAG, data, motor_number);
}

void RovesODriveMotor::readControlMode()
{
	writeODriveConfig(m_serial, READ, CONTROL_MODE_TAG, "", motor_number);
}

void RovesODriveMotor::writeTrapVelocityLimit(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, VELOCITY_LIMIT , data, motor_number);
}

void RovesODriveMotor::readTrapVelocityLimit()
{
	writeODriveConfig(m_serial, READ, VELOCITY_LIMIT , "", motor_number);
}

void RovesODriveMotor::writeTrapAccelerationLimit(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, ACCELERATION_LIMIT, data, motor_number);
}

void RovesODriveMotor::readTrapAccelerationLimit()
{
	writeODriveConfig(m_serial, READ, ACCELERATION_LIMIT , "", motor_number);
}

void RovesODriveMotor::writeTrapDecelerationLimit(float limit )
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, DECELERATION_LIMIT , data, motor_number);
}

void RovesODriveMotor::readTrapDecelerationLimit()
{
	writeODriveConfig(m_serial, READ, DECELERATION_LIMIT , "", motor_number);
}

void RovesODriveMotor::writeTrapAccelerationPerCounts(float limit )
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, ACCELERATION_PER_COUNTS , data, motor_number);
}

void RovesODriveMotor::readTrapAccelerationPerCounts()
{
	writeODriveConfig(m_serial, READ, ACCELERATION_PER_COUNTS , "", motor_number);
}

void RovesODriveMotor::writeTrapTarget(int32_t target)
{
	char data[12];
	intToChar(data, target);
	if ( target <= (m_position - 50) || target >= (m_position + 50) )
	{
		writeODriveConfig(m_serial, WRITE, CONTROL_MODE_TAG, "3", motor_number);
		writeODriveCommand(m_serial, "t", data, "", "", motor_number);
		m_position = target;
	}
	else
	{
		Serial.println("We are going to that position already");
	}
	
}

void RovesODriveMotor::writePosSetPoint(int32_t target1, int32_t target2, int32_t target3)
{
	char data1[12], data2[12], data3[12];
	intToChar(data1, target1);
	intToChar(data2, target2);
	intToChar(data3, target3);
	writeODriveCommand(m_serial, "p", data1, data2, data3, motor_number);
}


float RovesODriveMotor::readPosEstimate()
{
	String position = "";
	writeODriveConfig(m_serial, READ, "encoder.pos_estimate", "", motor_number);
	position = getSerial();
	return position.toFloat();
}

void RovesODriveMotor::reboot()
{
 	writeODriveCommand(m_serial, "sr", "", "", "", motor_number);
}

void RovesODriveMotor::saveConfig()
{
    writeODriveCommand(m_serial, "ss", "", "", "", motor_number);
}

void RovesODriveMotor::eraseConfig()
{
	writeODriveCommand(m_serial, "se", "", "", "", motor_number);
}

Error_Axis RovesODriveMotor::checkAxisErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, "error", "", motor_number);
	error = getSerial();
	return (Error_Axis)error.toInt();
}

Error_Motor RovesODriveMotor::checkMotorErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, "motor.error", "", motor_number);
	error = getSerial();
	return (Error_Motor)error.toInt();
}

Error_Encoder RovesODriveMotor::checkEncoderErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, "encoder.error", "", motor_number);
	error = getSerial();
	return (Error_Encoder)error.toInt();
}

Error_Controller RovesODriveMotor::checkControllerErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, "controller.error", "", motor_number);
	error = getSerial();
	return (Error_Controller)error.toInt();
}

String* RovesODriveMotor::checkAllErrors(String error[4])
{
	String* errors = error;
	error[0] = checkAxisErrors();
	error[1] = checkMotorErrors();
	error[2] = checkEncoderErrors();
	error[3] = checkControllerErrors();
	return errors;
}

void RovesODriveMotor::writeVelocityGain(float target)
{
	char data[12];
	floatToChar(data, target, 12);
	writeODriveConfig(m_serial, WRITE, "controller.config.vel_gain", data, motor_number);
}

void RovesODriveMotor::readVelocityGain()
{
	writeODriveConfig(m_serial, READ, "controller.config.vel_gain", "", motor_number);
}

void RovesODriveMotor::writePositionGain(float target)
{
	char data[12];
	floatToChar(data, target, 12);
	writeODriveConfig(m_serial, WRITE, "controller.config.pos_gain", data, motor_number);
}

void RovesODriveMotor::readPositionGain()
{
	writeODriveConfig(m_serial, READ, "controller.config.pos_gain", "", motor_number);
}

void RovesODriveMotor::writeVelocityIntegratorGain(float target)
{
	char data[12];
	floatToChar(data, target, 12);
	writeODriveConfig(m_serial, WRITE, "controller.config.vel_integrator_gain", data, motor_number);
}

void RovesODriveMotor::readVelocityIntegratorGain()
{
	writeODriveConfig(m_serial, READ, "controller.config.vel_integrator_gain", "", motor_number);
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
	writeODriveCommand(m_serial, "v", data1, data2, "", motor_number);
}

void RovesODriveMotor::writeCurrentLimit(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, "motor.config.current_lim", data, motor_number);
}

void RovesODriveMotor::readCurrentLimit()
{
	writeODriveConfig(m_serial, READ, "motor.config.current_lim", "", motor_number);
}

void RovesODriveMotor::writeVelocityLimit(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, "controller.config.vel_limit", data, motor_number);
}

void RovesODriveMotor::readVelocityLimit()
{
	writeODriveConfig(m_serial, READ, "controller.config.vel_limit", "", motor_number);
}

void RovesODriveMotor::writeCurrentCalibration(float limit)
{
	char data[12];
	floatToChar(data, limit, 12);
	writeODriveConfig(m_serial, WRITE, "motor.config.calibration_current", data, motor_number);
}

void RovesODriveMotor::readCurrentCalibration()
{
	writeODriveConfig(m_serial, READ, "motor.config.calibration_current", "", motor_number);
}

void RovesODriveMotor::writeBrakeResistance(float bResist)
{
	char data[12];
	floatToChar(data, bResist, 12);
	writeODriveConfig(m_serial, WRITE, "config.brake_resistance", data, motor_number);
}

void RovesODriveMotor::readBrakeResistance()
{
	writeODriveConfig(m_serial, READ, "config.brake_resistance", "", motor_number);
}

void RovesODriveMotor::writePolePairs(int32_t pairs)
{
	char data[12];
	intToChar(data, pairs);
	writeODriveConfig(m_serial, WRITE, "motor.config.pole_pairs", data, motor_number);
}

void RovesODriveMotor::readPolePairs()
{
	writeODriveConfig(m_serial, READ, "motor.config.pole_pairs", "", motor_number);
}

void RovesODriveMotor::writeEncoderCPR(float cpr)
{
	char data[12];
	floatToChar(data, cpr, 12);
	writeODriveConfig(m_serial, WRITE, "encoder.config.cpr", data, motor_number);
}

void RovesODriveMotor::readEncoderCPR()
{
	writeODriveConfig(m_serial, READ, "encoder.config.cpr", "", motor_number);
}

void RovesODriveMotor::moveToPos(float pos)
{
	char data[12];
	floatToChar(data, pos, 12);
	writeODriveConfig(m_serial, WRITE, "controller.move_to_pos", data, motor_number);
}

void RovesODriveMotor::moveIncremental(float incr)
{
	char data[12];
	floatToChar(data, incr, 12);
	writeODriveConfig(m_serial, WRITE, "controller.move_incremental", data, motor_number);
}

void RovesODriveMotor::writeVelocityRampRate(float rRate)
{
	char data[12];
	floatToChar(data, rRate, 12);
	writeODriveConfig(m_serial, WRITE, "controller.config.vel_ramp_rate", data, motor_number);
}

void RovesODriveMotor::readVelocityRampRate()
{
	writeODriveConfig(m_serial, READ, "controller.config.vel_ramp_rate", "", motor_number);
}

void RovesODriveMotor::writeCurrentSetPoint(float point)
{
	char data[12];
	floatToChar(data, point, 12);
	writeODriveCommand(m_serial, "c", data, "", "", motor_number);
}

void RovesODriveMotor::writeVelocityRampTarget(float target)
{
	char data[12];
	floatToChar(data, target, 12);
	writeVelocityRampEnable(true);
	writeODriveConfig(m_serial, WRITE, "controller.vel_ramp_target", data, motor_number);
}

void RovesODriveMotor::readVelocityRampTarget()
{
	writeVelocityRampEnable(true);
	writeODriveConfig(m_serial, READ, "controller.vel_ramp_target", "", motor_number);
}

void RovesODriveMotor::writeVelocityRampEnable(bool state)
{
	char data[12];
	boolToChar(data, state);
	writeODriveConfig(m_serial, WRITE, "controller.vel_ramp_enable", data, motor_number);
}

void RovesODriveMotor::readVelocityRampEnable()
{
	writeODriveConfig(m_serial, READ, "controller.vel_ramp_enable", "", motor_number);
}

void RovesODriveMotor::writeCPRSetpoint(bool state)
{
	char data[12];
	boolToChar(data, state);
	writeODriveConfig(m_serial, WRITE, "controller.config.setpoints_in_cpr", data, motor_number);
}

float RovesODriveMotor::readPosCPR()
{
	String position = "";
	writeODriveConfig(m_serial, READ, "encoder.pos_cpr", "", motor_number);
	position = getSerial();
	return position.toFloat();
}

void RovesODriveMotor::writeWatchdogTimeout(int32_t time)
{
	char data[12];
	intToChar(data, time);
	writeODriveConfig(m_serial, WRITE, "config.watchdog_timeout", data, motor_number);
}

void RovesODriveMotor::readWatchdogTimeout()
{
	writeODriveConfig(m_serial, READ, "config.watchdog_timeout", "", motor_number);
}

