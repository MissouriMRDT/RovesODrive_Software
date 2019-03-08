#include "RovesODrive.h"
#include <Energia.h>
#include <Stream.h>

int charToInt(char[] input)
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

float charToFloat(char[] input)
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
	return(value/(10*(i-j));
}

bool charToBool(char[] input)
{
	return(1);
}

void writeODrive(Stream& mySerial, bool write_request, char id[MAX_STRING_CHARS], int value)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %d\n", (write_request == WRITE)? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

void writeODrive(Stream& mySerial, bool write_request, char id[MAX_STRING_CHARS], float value = 0.0)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %f\n", (write_request == WRITE)? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

void writeODrive(Stream& mySerial, bool write_request, char id[MAX_STRING_CHARS], bool value = TRUE)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %d\n", (write_request == WRITE)? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

PacketStatus RovesODriveMotor::getSerial(char packet[])
{
	if!(m_serial->available())
	{
		return NoPacket;
	}
	uint8_t count = 0
	while(m_serial->available())
	{
		packet[count] = m_serial->read();
		count ++;
	}
	packet[count] = '\0';
	return(ValidPacket);
}

bool RovesODriveMotor::speedLow(uint16_t speed)
{
	return(abs(speed) < vel_shutoff_threshold);
}

void RovesODriveMotor::setControlMode(uint8_t mode)
{
	switch(mode)
	{
		case CTRL_MODE_SENSORLESS_VELOCITY_CONTROL:
			control_mode = CTRL_MODE_SENSORLESS_VELOCITY_CONTROL;
			writeControlMode(CTRL_MODE_VELOCITY_CONTROL);
			writePMFluxLinkage(PM_FLUX_LINKAGE_CONST/(motor_pole_pairs*motor_kv));
	}
}

void RovesODriveMotor::setSpeed(int speed)
{
  switch(control_mode)
  {
	  case CTRL_MODE_SENSORLESS_VELOCITY_CONTROL:
	  	static int last_speed = 0;

		writeVelrampEnable(TRUE);

		if(speedLow(speed)
		{
			speed = 0;
		}
		writeVelRampTarget(speed);  

		if(speedLow(last_speed) && !speedLow(speed))  //If speeding up from low speed
		{
			if(speed>0) 
			{
			writeSpinUpTargetVel(spin_up_target_vel);
			}
			else
			{
			writeSpinUpTargetVel(-spin_up_target_vel);
			}
			writeState(AXIS_STATE_IDLE);
			writeState(AXIS_STATE_SENSORLESS_CONTROL);
		}

		last_speed = speed;
  }
  
}

uint16_t RovesODriveMotor::getSpeed()
{
	requestVelRampTarget();
	char[] input;
	getSerial(input);
	return((uint16*)charToInt(input));
}

void RovesODriveMotor::writeState(uint8_t state)
{
	writeODrive(m_serial, WRITE, SET_CURRENT_STATE_TAG, state);
}

void RovesODriveMotor::requestState()
{
	writeODrive(m_serial, REQUEST, GET_CURRENT_STATE_TAG);
}

void RovesODriveMotor::writeControlMode(uint8_t mode)
{
	writeODrive(m_serial, WRITE, CONTROL_MODE_TAG, mode);
}

void RovesODriveMotor::requestControlMode()
{
	writeODrive(m_serial, REQUEST, CONTROL_MODE_TAG);
}

void RovesODriveMotor::writeStartupClosedLoop(bool b_startup)
{
	writeODrive(m_serial, WRITE, STARTUP_CLOSED_LOOP_TAG, b_startup);
}

void RovesODriveMotor::requestStartupClosedLoop()
{
	writeODrive(m_serial, REQUEST, STARTUP_CLOSED_LOOP_TAG);
}

void RovesODriveMotor::writeStartupSensorless(bool b_startup)
{
	writeODrive(m_serial, WRITE, STARTUP_SENSORLESS_TAG, b_startup);
}

void RovesODriveMotor::requestStartupSensorless()
{
	writeODrive(m_serial, REQUEST, STARTUP_SENSORLESS_TAG);
}

void RovesODriveMotor::writeStartupCalibrate(bool b_startup)
{
	writeODrive(m_serial, WRITE, STARTUP_MOTOR_CALBRATION_TAG, b_startup);
}

void RovesODriveMotor::requestStartupCalibrate()
{
	writeODrive(m_serial, REQUEST, STARTUP_MOTOR_CALBRATION_TAG);
}

void RovesODriveMotor::writeSpinUpAcceleration(uint16_t acceleration)
{
	writeODrive(m_serial, WRITE, SPINUP_TARGET_ACCEL_TAG, acceleration);
}

void RovesODriveMotor::requestSpinUpAcceleration()
{
	writeODrive(m_serial, REQUEST, SPINUP_TARGET_ACCEL_TAG);
}

void RovesODriveMotor::writeSpinUpTargetVel(uint16_t speed)
{
	writeODrive(m_serial, WRITE, SPINUP_TARGET_VEL_TAG, speed);
}

void RovesODriveMotor::requestSpinUpTargetVel()
{
	writeODrive(m_serial, REQUEST, SPINUP_TARGET_VEL_TAG);
}

void RovesODriveMotor::writeSpinUpCurrent(uint16_t current)
{
	writeODrive(m_serial, WRITE, SPINUP_CURRENT_TAG, current);
}

void RovesODriveMotor::requestSpinUpCurrent()
{
	writeODrive(m_serial, REQUEST, SPINUP_CURRENT_TAG);
}

void RovesODriveMotor::writeVelRampTarget(uint16_t target)
{
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_TARGET_TAG, target);
}

void RovesODriveMotor::requestVelRampTarget()
{
	writeODrive(m_serial, REQUEST, VELOCITY_RAMP_TARGET_TAG);
}

void RovesODriveMotor::writeVelRampRate(uint16_t rate)
{
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_RATE_TAG, rate);
}

void RovesODriveMotor::requestVelRampRate()
{
	writeODrive(m_serial, REQUEST, VELOCITY_RAMP_RATE_TAG);
}

void RovesODriveMotor::writeVelrampEnable(bool enable)
{
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_ENABLE_TAG, enable);
}

void RovesODriveMotor::requestVelrampEnable()
{
	writeODrive(m_serial, REQUEST, VELOCITY_RAMP_ENABLE_TAG);
}

void RovesODriveMotor::writeVelSetpoint(uint16_t setpoint)
{
	writeODrive(m_serial, WRITE, VELOCITY_SETPOINT_TAG, setpoint);
}

void RovesODriveMotor::requestVelSetpoint()
{
	writeODrive(m_serial, REQUEST, VELOCITY_SETPOINT_TAG);
}

void RovesODriveMotor::writePolepairs(uint8_t polepairs)
{
	writeODrive(m_serial, WRITE, POLE_PAIRS_TAG, polepairs);
}

void RovesODriveMotor::requestPolepairs()
{
	writeODrive(m_serial, REQUEST, POLE_PAIRS_TAG);
}

void RovesODriveMotor::writeVelocityGain(float gain)
{
	writeODrive(m_serial, WRITE, VELOCITY_GAIN_TAG, gain);
}

void RovesODriveMotor::requestVelocityGain()
{
	writeODrive(m_serial, REQUEST, VELOCITY_GAIN_TAG);
}

void RovesODriveMotor::writeVelocityIntegratorGain(float gain)
{
	writeODrive(m_serial, WRITE, VELOCITY_INTEGRATOR_TAG, gain);
}

void RovesODriveMotor::requestVelocityIntegratorGain()
{
	writeODrive(m_serial, REQUEST, VELOCITY_INTEGRATOR_TAG);
}

void RovesODriveMotor::writeVelocityLimit(float limit)
{
	writeODrive(m_serial, WRITE, VELOCITY_LIMIT_TAG, limit);
}

void RovesODriveMotor::requestVelocityLimit()
{
	writeODrive(m_serial, REQUEST, VELOCITY_LIMIT_TAG);
}

void RovesODriveMotor::writeCurrentGain(float gain)
{
	writeODrive(m_serial, WRITE, CURRENT_PGAIN_TAG, gain);
}

void RovesODriveMotor::requestCurrentGain()
{
	writeODrive(m_serial, REQUEST, CURRENT_PGAIN_TAG);
}

void RovesODriveMotor::writeCurrentIntegratorGain(float gain)
{
	writeODrive(m_serial, WRITE, CURRENT_IGAIN_TAG, gain);
}

void RovesODriveMotor::requestCurrentIntegratorGain()
{
	writeODrive(m_serial, REQUEST, CURRENT_IGAIN_TAG);
}

void RovesODriveMotor::writeCurrentLimit(float limit)
{
	writeODrive(m_serial, WRITE, MAX_ALLOWED_CURRENT_TAG, limit);
}

void RovesODriveMotor::requestCurrentLimit()
{
	writeODrive(m_serial, REQUEST, MAX_ALLOWED_CURRENT_TAG);
}

void RovesODriveMotor::requestPhaseCurrent()
{
	writeODrive(m_serial, REQUEST, PHASE_B_IMEAS);
}

void RovesODriveMotor::requestBusCurrent()
{
	writeODrive(m_serial, REQUEST, BUS_CURRENT_TAG);
}

void RovesODriveMotor::writePMFluxLinkage(float linkage)
{
	writeODrive(m_serial, WRITE, PM_FLIX_LINKAGE_TAG, linkage);
}

void RovesODriveMotor::requestPMFluxLinkage()
{
	writeODrive(m_serial, REQUEST, PM_FLIX_LINKAGE_TAG);
}

void RovesODriveMotor::requestError()
{
	writeODrive(m_serial, REQUEST, ERROR_TAG);
}

void RovesODriveMotor::requestDRVError()
{	
	writeODrive(m_serial, REQUEST, DRV_FAULT_TAG);
}

void RovesODriveMotor::requestIsPreCalibrated()
{
	writeODrive(m_serial, REQUEST, PRE_CALIBRATED_TAG);
}

void RovesODriveMotor::requestIsCalibrated()
{
	writeODrive(m_serial, REQUEST, IS_CALIBRATED_TAG);
}


