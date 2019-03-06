#include "RovesODrive.h"
#include <Energia.h>
#include <Stream.h>


void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], int value)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %d\n", write_read? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], float value = 0.0)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %f\n", write_read? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], bool value = TRUE)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %d\n", write_read? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

RovesODriveMotor::void setState(uint8_t state);
RovesODriveMotor::uint8_t  getState();
RovesODriveMotor::void setControlMode(uint8_t mode);
RovesODriveMotor::uint8_t  getControlMode();
RovesODriveMotor::void setStartupClosedLoop(bool b_startup);
RovesODriveMotor::bool  getStartupClosedLoop();
RovesODriveMotor::void setStartupSensorless(bool b_startup);
RovesODriveMotor::bool  getStartupSensorless();
RovesODriveMotor::void setStartupCalibrate(bool b_startup);
RovesODriveMotor::bool  getStartupCalibrate();
RovesODriveMotor::void setSpinUpAcceleration(uint16_t acceleration);
RovesODriveMotor::uint16_t getSpinUpAcceleration();
RovesODriveMotor::void setSpinUpTargetVel(uint16_t speed);
RovesODriveMotor::uint16_t getSpinUpTargetVel();
RovesODriveMotor::void setSpinUpCurrent(uint16_t current);
RovesODriveMotor::uint16_t getSpinUpCurrent();
RovesODriveMotor::void setVelRampTarget(uint16_t target);
RovesODriveMotor::uint16_t getVelRampTarget();
RovesODriveMotor::void setVelRampRate(uint16_t rate);
RovesODriveMotor::uint16_t getVelRampRate();
RovesODriveMotor::void setVelrampEnable(bool target);
RovesODriveMotor::bool getVelrampEnable();
RovesODriveMotor::void setVelSetpoint(uint16_t setpoint);
RovesODriveMotor::uint16_t getVelSetpoint();
RovesODriveMotor::void setPolepairs(uint8_t kv);
RovesODriveMotor::uint8_t getPolepairs();
RovesODriveMotor::void setKV(uint16_t kv);
RovesODriveMotor::uint16_t getKV();
RovesODriveMotor::void setVelocityGain(float gain);
RovesODriveMotor::float getVelocityGain();
RovesODriveMotor::void setVelocityIntegratorGain(float gain);
RovesODriveMotor::float getVelocityIntegratorGain();
RovesODriveMotor::void setVelocityLimit(float limit);
RovesODriveMotor::float getVelocityLimit();
RovesODriveMotor::void setCurrentGain(float gain);
RovesODriveMotor::float getCurrentGain();
RovesODriveMotor::void setCurrentIntegratorGain(float gain);
RovesODriveMotor::float getCurrentIntegratorGain();
RovesODriveMotor::void setCurrentLimit(float limit);
RovesODriveMotor::float getCurrentLimit();
RovesODriveMotor::float getPhaseCurrent();
RovesODriveMotor::float getBusCurrent();
RovesODriveMotor::void setPMFluxLinkage(float linkage);
RovesODriveMotor::float setPMFluxLinkage();
RovesODriveMotor::uint16_t getError();
RovesODriveMotor::uint16_t getDRVError();
RovesODriveMotor::bool getIsPreCalibrated();
RovesODriveMotor::bool getIsCalibrated();

