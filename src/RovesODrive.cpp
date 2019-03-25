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

void writeODrive(HardwareSerial* mySerial, bool write_read, char* id, char* value, uint8_t axis)
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

PacketStatus RovesODriveMotor::getSerial(char packet[])
{
	//Serial.print("Read:");
	if(!m_serial->available())
	{
		//Serial.println("No Packet");
		return NoPacket;
	}
	//Serial.print("Packet-");
	uint8_t count = 0;
	while(m_serial->available())
	{
		packet[count] = m_serial->read();
		//Serial.print(packet[count]);
		count ++;
		if(count > sizeof(packet)) return(OverflowPacket);
	}
	//Serial.println("");
	packet[count] = '\0';
	return(ValidPacket);
}

bool RovesODriveMotor::speedLow(int16_t speed)
{
	return(abs(speed) < vel_shutoff_threshold);
}

void RovesODriveMotor::setControlMode(uint8_t mode)
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
			writeSpinUpTargetVel(spin_up_target_vel);

			writeState(AXIS_STATE_IDLE);
			writeState(AXIS_STATE_SENSORLESS_CONTROL);
			//writeCurrentSetopint(current_setpoint);
		}

		writeVelRampTarget(m_direction*speed);  

		vel_setpoint = speed;
  }
  
}

int16_t RovesODriveMotor::getSpeed()
{
	int16_t speed = 0;
	getSpeed(speed);
	return(speed);
}

PacketStatus RovesODriveMotor::getSpeed(int16_t &speed)
{
	requestVelRampTarget();
	char input[10];
	PacketStatus status = getSerial(input);
	if(status = ValidPacket)
	{
		speed = charToInt(input);
		Serial.println(speed);
	}
	return(status);
	
}

void RovesODriveMotor::setRampValue(int16_t value)
{
	vel_ramp_rate = value;
}

SerialStatus RovesODriveMotor::checkSerial()
{
	switch(m_control_mode)
	{
		case CTRL_MODE_SENSORLESS_VELOCITY_CONTROL:
			int16_t speed;
			if(getSpeed(speed) != ValidPacket)
			{
				return(SerialFault);
			}
			else if(speed != vel_setpoint)
			{
				return(SerialFault);
			}
		break;
	}

	return(SerialGood);
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

void RovesODrive::begin()
{
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

void RovesODriveMotor::writeState(uint8_t state)
{
	char data[2];
	intToChar(data, state);
	writeODrive(m_serial, WRITE, SET_CURRENT_STATE_TAG, data, motor_number);
}

void RovesODriveMotor::requestState()
{
	writeODrive(m_serial, REQUEST, GET_CURRENT_STATE_TAG, "", motor_number);
}

void RovesODriveMotor::writeControlMode(uint8_t mode)
{
	char data[2];
	intToChar(data, mode);
	writeODrive(m_serial, WRITE, CONTROL_MODE_TAG, data, motor_number);
}

void RovesODriveMotor::writeStartupClosedLoop(bool b_startup)
{
	char data[2];
	boolToChar(data, b_startup);
	writeODrive(m_serial, WRITE, STARTUP_CLOSED_LOOP_TAG, data, motor_number);
}

void RovesODriveMotor::requestStartupClosedLoop()
{
	writeODrive(m_serial, REQUEST, STARTUP_CLOSED_LOOP_TAG, "", motor_number);
}

void RovesODriveMotor::writeStartupSensorless(bool b_startup)
{
	char data[2];
	boolToChar(data, b_startup);
	writeODrive(m_serial, WRITE, STARTUP_SENSORLESS_TAG, data, motor_number);
}

void RovesODriveMotor::writeSpinUpAcceleration(int16_t acceleration)
{
	char data[6];
	intToChar(data, acceleration);
	writeODrive(m_serial, WRITE, SPINUP_TARGET_ACCEL_TAG, data, motor_number);
}

void RovesODriveMotor::writeSpinUpTargetVel(int16_t speed)
{
	char data[6];
	intToChar(data, speed);
	writeODrive(m_serial, WRITE, SPINUP_TARGET_VEL_TAG, data, motor_number);
}

void RovesODriveMotor::requestSpinUpTargetVel()
{
	writeODrive(m_serial, REQUEST, SPINUP_TARGET_VEL_TAG, "", motor_number);
}

void RovesODriveMotor::writeSpinUpCurrent(uint16_t current)
{
	char data[6];
	intToChar(data, current);
	writeODrive(m_serial, WRITE, SPINUP_CURRENT_TAG, data, motor_number);
}

void RovesODriveMotor::requestSpinUpCurrent()
{
	writeODrive(m_serial, REQUEST, SPINUP_TARGET_VEL_TAG, "", motor_number);
}

void RovesODriveMotor::writeCurrentSetopint(uint16_t current)
{
	char data[6];
	intToChar(data, current);
	writeODrive(m_serial, WRITE, CURRENT_SETPOINT_TAG, data, motor_number);
}

void RovesODriveMotor::writeSpinUpTime(float time)
{
	char data[7];
	floatToChar(data, time, 5);
	writeODrive(m_serial, WRITE, SPINUP_TIME_TAG, data, motor_number);
}

void RovesODriveMotor::writeDirection(int8_t value)
{
	char data[7];
	intToChar(data, value);
	writeODrive(m_serial, WRITE, DIRECTION_TAG, data, motor_number);
}

void RovesODriveMotor::writeVelRampTarget(int16_t target)
{
	char data[6];
	intToChar(data, target);
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_TARGET_TAG, data, motor_number);
}

void RovesODriveMotor::requestVelRampTarget()
{
	writeODrive(m_serial, REQUEST, VELOCITY_RAMP_TARGET_TAG, "", motor_number);
}

void RovesODriveMotor::writeVelRampRate(int16_t rate)
{
	char data[6];
	intToChar(data, rate);
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_RATE_TAG, data, motor_number);
}

void RovesODriveMotor::writeVelrampEnable(bool enable)
{
	char data[2];
	boolToChar(data, enable);
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_ENABLE_TAG, data, motor_number);
}

void RovesODriveMotor::writeVelSetpoint(int16_t setpoint)
{
	char data[6];
	intToChar(data, setpoint);
	writeODrive(m_serial, WRITE, VELOCITY_SETPOINT_TAG, data, motor_number);
}

void RovesODriveMotor::requestVelSetpoint()
{
	writeODrive(m_serial, REQUEST, VELOCITY_SETPOINT_TAG, "", motor_number);
}

void RovesODriveMotor::writePolepairs(uint8_t polepairs)
{
	char data[2];
	intToChar(data, polepairs);
	writeODrive(m_serial, WRITE, POLE_PAIRS_TAG, data, motor_number);
}

void RovesODriveMotor::writePMFluxLinkage(float linkage)
{
	char data[22];
	floatToChar(data, linkage, 22);
	writeODrive(m_serial, WRITE, PM_FLIX_LINKAGE_TAG, data, motor_number);
}


