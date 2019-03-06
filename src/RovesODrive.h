#ifndef _RovesODrive_h
#define _RovesODrive_h


#include "ODriveArduino.h"
#include <stdio.h>
#include <Energia.h>
#include <Stream.h>

#define MAX_STRING_CHARS 50

#define WRITE	true
#define	READ 	false

//Axis states
#define AXIS_STATE_IDLE 						1
#define AXIS_STATE_STARTUP_SEQUENCE  			2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE  	3
#define AXIS_STATE_SENSORLESS_CONTROL   		4
#define AXIS_STATE_ENCODER_INDEX_SEARCH   		5
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION	6
#define AXIS_STATE_CLOSED_LOOP_CONTROL  		7

//Control modes
#define CTRL_MODE_POSITION_CONTROL	1
#define CTRL_MODE_VELOCITY_CONTROL	2
#define CTRL_MODE_CURRENT_CONTROL	3
#define CTRL_MODE_VOLTAGE_CONTROL 	4

#define PM_FLUX_LINKAGE 	5.51328895422 


void writeODrive(Stream& mySerial, bool write_read, String id, int value = 0);


struct ODrivePacket()
{
	uint8_t length
	String data[];
}
	
class RovesODriveMotor
{
	public:
		RovesODriveMotor(Stream& mySerial, uint8_t motor_number);
		void begin(int baud);
		void setSpeed(uint16_t speed);
		uint16_t getSpeed();
		
	private:
		bool speedLow(uint16_t speed);
		
		
		void setSpinUpAcceleration(uint16_t acceleration);
		uint16_t getSpinUpAcceleration();
		void setSpinUpTargetVel(uint16_t speed);
		uint16_t getSpinUpTargetVel();
		
		void setVelRampTarget(uint16_t target);
		uint16_t getVelRampTarget();
		void setVelrampEnable(bool target);
		bool getVelrampEnable();
		
		void setPolepairs(uint8_t kv);
		uint8_t getPolepairs();
		void setKV(uint16_t kv);
		uint16_t getKV();
		
		void setVelocityGain(float gain);
		float getVelocityGain();
		void setVelocityIntegratorGain(float gain);
		float getVelocityIntegratorGain();
		
		//Member Vars
		uint8_t motor_number;
		String motor_name;
		uint16_t vel_shutoff_threshold = 100;
		
		
		uint16_t vel_setpoint;
		
		//Spin Up parameters
		uint16_t spin_up_acceleration;
		uint16_t spin_up_target_vel;
		
		//Ramp Parameters
		uint16_t vel_ramp_target;
		bool vel_ramp_enable;
		
		//Motor Parameters
		uint8_t pole_pairs;
		uint16_t motor_kv;
		
		//Velocity Parameters
		float vel_gain;
		float vel_integrator_gain;
		
		
		parsePacket(ODrivePacket packet);
};

class RovesODrive()
{
	public:
		RovesODrive(Stream& mySerial);
		void begin(int baud);
		void read();
		bool isConnected();
		
		RovesODriveMotor motor[2];
	private:
		void saveConfiguration();
		void eraseConfiguration();
		void setState(int state);
		int  getState();
		void ping();
		
		parsePacket(ODrivePacket packet);
};

#endif
