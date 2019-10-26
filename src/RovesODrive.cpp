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

void writeODriveConfig(HardwareSerial* mySerial, bool write_read, char* id, char* value, uint8_t axis)
{
	char output[255];

	sprintf(output, "%s ", (write_read == WRITE)? "w":"r");
	if(axis != 3)
	{
		sprintf(output, "%s%s%d.", output, "axis", axis);
	}
	sprintf(output, "%s%s %s\n", output, id, value);
	mySerial->write(output);
}

void writeODriveCommand(HardwareSerial* mySerial, char* id, char* value, uint8_t axis)
{
	char output[255];

	sprintf(output, "%s", id);
	if(axis != 3)
	{
		sprintf(output, "%s %d", output, axis);
	}
	sprintf(output, "%s %s \n", output, value);
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


///////////////////////////////////////////////////////////////
void RovesODriveMotor::writeState(Axis_State state)
{
	char data[2];
	intToChar(data, state);
	writeODriveConfig(m_serial, WRITE, SET_CURRENT_STATE_TAG, data, motor_number);
}

void RovesODriveMotor::requestState()
{
	writeODriveConfig(m_serial, READ, GET_CURRENT_STATE_TAG, "", motor_number);
}

void RovesODriveMotor::writeControlMode(uint8_t mode)
{
	char data[2];
	intToChar(data, mode);
	writeODriveConfig(m_serial, WRITE, CONTROL_MODE_TAG, data, motor_number);
}

void RovesODriveMotor::requestControlMode()
{
	writeODriveConfig(m_serial, READ, CONTROL_MODE_TAG, "", motor_number);
}

void RovesODriveMotor::setTrapVelocityLimit(float limit )
{
	char data[12];
	floatToChar(data, limit);
	writeODriveConfig(m_serial, WRITE, VELOCITY_LIMIT , data, motor_number);
}

void RovesODriveMotor::setTrapAccelerationLimit(float limit )
{
	char data[12];
	floatToChar(data, limit);
	writeODriveConfig(m_serial, WRITE, ACCELERATION_LIMIT , data, motor_number);
}

void RovesODriveMotor::setTrapDecelerationLimit(float limit )
{
	char data[12];
	floatToChar(data, limit);
	writeODriveConfig(m_serial, WRITE, DECELERATION_LIMIT , data, motor_number);
}

void RovesODriveMotor::setTrapAccelerationPerCounts(float limit )
{
	char data[12];
	floatToChar(data, limit);
	writeODriveConfig(m_serial, WRITE, ACCELERATION_PER_COUNTS , data, motor_number);
}

void RovesODriveMotor::setTrapTarget(int32_t target)
{
	char data[12];
	float pest = 0;
	intToChar(data, target);

	pest = requestPosEstimate();

	if ( pest >= (target - 50) || pest >= (target + 50) )
	{
		writeODriveCommand(m_serial, "t", data, motor_number);
	}
	
	else
	{
		writeODriveConfig(m_serial, WRITE, SET_CURRENT_STATE_TAG, "1", motor_number);
		writeODriveConfig(m_serial, WRITE, SET_CURRENT_STATE_TAG, "8", motor_number);
		writeODriveCommand(m_serial, "t", data, motor_number);
	}
}

void RovesODriveMotor::setPosSetPoint(int32_t target)
{
	char data[12];
	intToChar(data, target);
	writeODriveCommand(m_serial, "p", data, motor_number);
}


float RovesODriveMotor::requestPosEstimate()
{
	String position = "";
	writeODriveConfig(m_serial, READ, "encoder.pos_estimate", "", motor_number);
	position = getSerial();
	return position.toFloat();
}

void RovesODriveMotor::reboot()
{
 	writeODriveCommand(m_serial, "sr", "", motor_number);
}

void RovesODriveMotor::saveConfig()
{
    writeODriveCommand(m_serial, "ss", "", motor_number);
	reboot();
}

void RovesODriveMotor::eraseConfig()
{
	writeODriveCommand(m_serial, "se", "", motor_number);
	reboot();
}

Error_Axis RovesODriveMotor::checkAxisErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, "error", "", motor_number);
	error = getSerial();
	return error.toInt();
}

Error_Motor RovesODriveMotor::checkMotorErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, "motor.error", "", motor_number);
	error = getSerial();
	return error.toInt();
}

Error_Encoder RovesODriveMotor::checkEncoderErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, "encoder.error", "", motor_number);
	error = getSerial();
	return error.toInt();
}

Error_Controller RovesODriveMotor::checkControllerErrors()
{
	String error = "";
	writeODriveConfig(m_serial, READ, "controller.error", "", motor_number);
	error = getSerial();
	return error.toInt();
}
