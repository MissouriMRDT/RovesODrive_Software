#include "RovesODrive.h"
#include <Energia.h>
#include <Stream.h>


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

RovesODriveMotor::void writeState(uint8_t state)
{
	writeODrive(m_serial, WRITE, SET_CURRENT_STATE_TAG, state);
}

RovesODriveMotor::void  requestState()
{
	writeODrive(m_serial, REQUEST, GET_CURRENT_STATE_TAG);
}

RovesODriveMotor::void writeControlMode(uint8_t mode)
{
	writeODrive(m_serial, WRITE, CONTROL_MODE_TAG, mode);
}

RovesODriveMotor::void  requestControlMode()
{
	writeODrive(m_serial, REQUEST, CONTROL_MODE_TAG);
}

RovesODriveMotor::void writeStartupClosedLoop(bool b_startup)
{
	writeODrive(m_serial, WRITE, STARTUP_CLOSED_LOOP_TAG, b_startup);
}

RovesODriveMotor::void  requestStartupClosedLoop()
{
	writeODrive(m_serial, REQUEST, STARTUP_CLOSED_LOOP_TAG);
}

RovesODriveMotor::void writeStartupSensorless(bool b_startup)
{
	writeODrive(m_serial, WRITE, STARTUP_SENSORLESS_TAG, b_startup);
}

RovesODriveMotor::void  requestStartupSensorless()
{
	writeODrive(m_serial, REQUEST, STARTUP_SENSORLESS_TAG);
}

RovesODriveMotor::void writeStartupCalibrate(bool b_startup)
{
	writeODrive(m_serial, WRITE, STARTUP_MOTOR_CALBRATION_TAG, b_startup);
}

RovesODriveMotor::void  requestStartupCalibrate()
{
	writeODrive(m_serial, REQUEST, STARTUP_MOTOR_CALBRATION_TAG);
}

RovesODriveMotor::void writeSpinUpAcceleration(uint16_t acceleration)
{
	writeODrive(m_serial, WRITE, SPINUP_TARGET_ACCEL_TAG, acceleration);
}

RovesODriveMotor::void requestSpinUpAcceleration()
{
	writeODrive(m_serial, REQUEST, SPINUP_TARGET_ACCEL_TAG);
}

RovesODriveMotor::void writeSpinUpTargetVel(uint16_t speed)
{
	writeODrive(m_serial, WRITE, SPINUP_TARGET_VEL_TAG, speed);
}

RovesODriveMotor::void requestSpinUpTargetVel()
{
	writeODrive(m_serial, REQUEST, SPINUP_TARGET_VEL_TAG);
}

RovesODriveMotor::void writeSpinUpCurrent(uint16_t current)
{
	writeODrive(m_serial, WRITE, SPINUP_CURRENT_TAG, current);
}

RovesODriveMotor::void requestSpinUpCurrent()
{
	writeODrive(m_serial, REQUEST, SPINUP_CURRENT_TAG);
}

RovesODriveMotor::void writeVelRampTarget(uint16_t target)
{
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_TARGET_TAG, target);
}

RovesODriveMotor::void requestVelRampTarget()
{
	writeODrive(m_serial, REQUEST, VELOCITY_RAMP_TARGET_TAG);
}

RovesODriveMotor::void writeVelRampRate(uint16_t rate)
{
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_RATE_TAG, rate);
}

RovesODriveMotor::void requestVelRampRate()
{
	writeODrive(m_serial, REQUEST, VELOCITY_RAMP_RATE_TAG);
}

RovesODriveMotor::void writeVelrampEnable(bool enable)
{
	writeODrive(m_serial, WRITE, VELOCITY_RAMP_ENABLE_TAG, enable);
}

RovesODriveMotor::void requestVelrampEnable()
{
	writeODrive(m_serial, REQUEST, VELOCITY_RAMP_ENABLE_TAG);
}

RovesODriveMotor::void writeVelSetpoint(uint16_t setpoint)
{
	writeODrive(m_serial, WRITE, VELOCITY_SETPOINT_TAG, setpoint);
}

RovesODriveMotor::void requestVelSetpoint()
{
	writeODrive(m_serial, REQUEST, VELOCITY_SETPOINT_TAG);
}

RovesODriveMotor::void writePolepairs(uint8_t polepairs)
{
	writeODrive(m_serial, WRITE, POLE_PAIRS_TAG, polepairs);
}

RovesODriveMotor::void requestPolepairs()
{
	writeODrive(m_serial, REQUEST, POLE_PAIRS_TAG);
}

RovesODriveMotor::void writeVelocityGain(float gain)
{
	writeODrive(m_serial, WRITE, VELOCITY_GAIN_TAG, gain);
}

RovesODriveMotor::void requestVelocityGain()
{
	writeODrive(m_serial, REQUEST, VELOCITY_GAIN_TAG);
}

RovesODriveMotor::void writeVelocityIntegratorGain(float gain)
{
	writeODrive(m_serial, WRITE, VELOCITY_INTEGRATOR_TAG, gain);
}

RovesODriveMotor::void requestVelocityIntegratorGain()
{
	writeODrive(m_serial, REQUEST, VELOCITY_INTEGRATOR_TAG);
}

RovesODriveMotor::void writeVelocityLimit(float limit)
{
	writeODrive(m_serial, WRITE, VELOCITY_LIMIT_TAG, limit);
}

RovesODriveMotor::void requestVelocityLimit()
{
	writeODrive(m_serial, REQUEST, VELOCITY_LIMIT_TAG);
}

RovesODriveMotor::void writeCurrentGain(float gain)
{
	writeODrive(m_serial, WRITE, CURRENT_PGAIN_TAG, gain);
}

RovesODriveMotor::void requestCurrentGain()
{
	writeODrive(m_serial, REQUEST, CURRENT_PGAIN_TAG);
}

RovesODriveMotor::void writeCurrentIntegratorGain(float gain)
{
	writeODrive(m_serial, WRITE, CURRENT_IGAIN_TAG, gain);
}

RovesODriveMotor::void requestCurrentIntegratorGain()
{
	writeODrive(m_serial, REQUEST, CURRENT_IGAIN_TAG);
}

RovesODriveMotor::void writeCurrentLimit(float limit)
{
	writeODrive(m_serial, WRITE, MAX_ALLOWED_CURRENT_TAG, limit);
}

RovesODriveMotor::void requestCurrentLimit()
{
	writeODrive(m_serial, REQUEST, MAX_ALLOWED_CURRENT_TAG);
}

RovesODriveMotor::void requestPhaseCurrent()
{
	writeODrive(m_serial, REQUEST, PHASE_B_IMEAS);
}

RovesODriveMotor::void requestBusCurrent()
{
	writeODrive(m_serial, REQUEST, BUS_CURRENT_TAG);
}

RovesODriveMotor::void writePMFluxLinkage(float linkage)
{
	writeODrive(m_serial, WRITE, PM_FLIX_LINKAGE_TAG, linkage);
}

RovesODriveMotor::void requestPMFluxLinkage()
{
	writeODrive(m_serial, REQUEST, PM_FLIX_LINKAGE_TAG);
}

RovesODriveMotor::void requestError()
{
	writeODrive(m_serial, REQUEST, ERROR_TAG);
}

RovesODriveMotor::void requestDRVError()
{	
	writeODrive(m_serial, REQUEST, DRV_FAULT_TAG);
}

RovesODriveMotor::void requestIsPreCalibrated()
{
	writeODrive(m_serial, REQUEST, PRE_CALIBRATED_TAG);
}

RovesODriveMotor::void requestIsCalibrated()
{
	writeODrive(m_serial, REQUEST, IS_CALIBRATED_TAG);
}


