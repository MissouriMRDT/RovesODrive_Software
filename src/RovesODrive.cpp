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
	//Serial.print("Value: ");
	//Serial.println(value);
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

String RovesODriveMotor::getSerial(char packet[])
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

bool RovesODriveMotor::speedLow(int16_t speed)
{
	return(abs(speed) < vel_shutoff_threshold);
}

void RovesODriveMotor::setControlMode(Control_Mode mode)
{
	switch(mode)
	{
		case CTRL_MODE_SENSORLESS_VELOCITY_CONTROL:
			m_control_mode = CTRL_MODE_SENSORLESS_VELOCITY_CONTROL;
			writeControlMode(CTRL_MODE_VELOCITY_CONTROL);
			writePMFluxLinkage(PM_FLUX_LINKAGE_CONST/(motor_pole_pairs*motor_kv));
	}
}

void RovesODriveMotor::setSpeed(int16_t speed)
{
  switch(m_control_mode)
  {
	  case CTRL_MODE_SENSORLESS_VELOCITY_CONTROL:

		writeVelrampEnable(true);

		if(speedLow(speed))
		{
			speed = 0;
		}
		
		if(speedLow(vel_setpoint) && !speedLow(speed))  //If speeding up from low speed
		{
			Serial.println("IN RAMPUP");
			if(speed>0) 
			{
				setDirection(1);
			}
			else
			{
				setDirection(-1);
			}
			//writeSpinUpTargetVel(spin_up_target_vel);

			writeState(AXIS_STATE_IDLE);
			writeState(AXIS_STATE_SENSORLESS_CONTROL);
			if(do_current_ramp) current_setpoint = current_ramp_start;
			writeCurrentSetopint(current_setpoint);
		}

		if(speed)
		{
			if(do_current_ramp)
			{
				if(current_setpoint<current_ramp_end) writeCurrentSetopint(current_setpoint+=current_ramp_inc);
			}
			Serial.print("Current Setpoint:");
			Serial.println(current_setpoint);
		}
		else
		{
			writeCurrentSetopint(idle_current);
		}
		

		writeVelRampTarget(m_direction*speed);  

		vel_setpoint = speed;
  }
  
}

void RovesODriveMotor::setRampValue(int16_t value)
{
	vel_ramp_rate = value;
}


void RovesODriveMotor::writeConfig()
{
	switch(m_control_mode)
	{
		case CTRL_MODE_SENSORLESS_VELOCITY_CONTROL:
			//writeSpinUpAcceleration(spin_up_acceleration);
			//writeSpinUpTime(.5);
			writeVelRampRate(vel_ramp_rate);
			//writeSpinUpCurrent(spin_up_current);
		break;
	}
}

void RovesODriveMotor::setPolePairs(uint8_t pole_pairs)
{
	motor_pole_pairs = pole_pairs;
}

void RovesODriveMotor::setKV(uint16_t KV)
{
	motor_kv = KV;
}

void RovesODriveMotor::setRamp(uint16_t rate)
{
	//constrain(rate, MAX_VELOCITY_RAMP_RATE, -MAX_VELOCITY_RAMP_RATE);

	setSpinupAccleleration(rate);
	setRampRate(rate);
}

void RovesODriveMotor::setSpinupAccleleration(uint16_t acceleration)
{
	spin_up_acceleration = acceleration;
}

void RovesODriveMotor::setRampRate(uint16_t rate)
{
	vel_ramp_rate = rate;
}

void RovesODriveMotor::idleMotor()
{
	writeState(AXIS_STATE_IDLE);
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
void RovesODriveMotor::setDirection(int8_t direction)
{
	m_direction = direction;
	writeDirection(m_direction);
}

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

void RovesODriveMotor::writeStartupClosedLoop(bool b_startup)
{
	char data[2];
	boolToChar(data, b_startup);
	writeODriveConfig(m_serial, WRITE, STARTUP_CLOSED_LOOP_TAG, data, motor_number);
}

void RovesODriveMotor::requestStartupClosedLoop()
{
	writeODriveConfig(m_serial, READ, STARTUP_CLOSED_LOOP_TAG, "", motor_number);
}

void RovesODriveMotor::writeVelSetpoint(int16_t setpoint)
{
	char data[6];
	intToChar(data, setpoint);
	writeODriveConfig(m_serial, WRITE, VELOCITY_SETPOINT_TAG, data, motor_number);
}

void RovesODriveMotor::requestVelSetpoint()
{
	writeODriveConfig(m_serial, READ, VELOCITY_SETPOINT_TAG, "", motor_number);
}

void RovesODriveMotor::writePolepairs(uint8_t polepairs)
{
	char data[2];
	intToChar(data, polepairs);
	writeODriveConfig(m_serial, WRITE, POLE_PAIRS_TAG, data, motor_number);
}

void RovesODriveMotor::setTrapTarget(int32_t target)
{
	char data[12];
	intToChar(data, target);
	writeODriveCommand(m_serial, "t", data, motor_number);
}


float RovesODriveMotor::requestPosEstimate(const string target)
{
	if(target == "position")
	{
		String position = "";
		char input[22];
		writeODriveConfig(m_serial, READ, "encoder.pos_estimate", "", motor_number);
		position = getSerial(input);
		return position.toFloat();
	}
	else
	{
		return 0;
	}
	
	
}

void RovesODriveMotor::reboot(const string target)
{
	if(target == "reboot")
    {
        Serial.println("Rebooting");
     	writeODriveConfig(m_serial, WRITE, "sr", "", motor_number);
     }
	return;
}

void RovesODriveMotor::saveConfig(const string target)
{
	if(target == "save_config")
    {
        Serial.println("Saving");
     	writeODriveConfig(m_serial, WRITE, "ss", "", motor_number);
     }
	return;
}

void RovesODriveMotor::eraseConfig(const string target)
{
	if(target == " erase_config")
    {
        Serial.println("Erasing");
     	writeODriveConfig(m_serial, WRITE, "se", "", motor_number);
     }
	return;
}
