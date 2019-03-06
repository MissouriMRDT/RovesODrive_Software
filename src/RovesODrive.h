#ifndef _RovesODrive_h
#define _RovesODrive_h


#include "ODriveArduino.h"
#include <stdio.h>
#include <Energia.h>
#include <Stream.h>

#define MAX_STRING_CHARS 255

#define WRITE	true
#define	READ 	false

//Axis Data Tags
//configs
#define GET_CURRENT_STATE_TAG			"current_state"							//int
#define SET_CURRENT_STATE_TAG			"requested_state"						//int
#define STARTUP_CLOSED_LOOP_TAG 		"config.startup_closed_loop_control"	//bool
#define STARTUP_SENSORLESS_TAG 			"config.startup_sensorless_control"		//bool
#define STARTUP_MOTOR_CALBRATION_TAG	"config.startup_motor_calibration"		//bool

//Spinup Config
#define SPINUP_TARGET_VEL_TAG 			"config.spin_up_target_vel"				//float
#define SPINUP_TARGET_ACCEL_TAG 		"config.spin_up_acceleration"			//float
#define SPINUP_CURRENT_TAG		 		"config.spin_up_current"				//float

//Velocity Ramp
#define VELOCITY_RAMP_ENABLE_TAG		"controller.vel_ramp_enable"			//bool
#define VELOCITY_RAMP_TARGET_TAG		"controller.vel_ramp_target"			//float
#define VELOCITY_RAMP_RATE_TAG			"controller.config.vel_ramp_rate"		//float

//Velocity Config
#define VELOCITY_SETPOINT_TAG	 		"controller.vel_setpoint"				//float
#define VELOCITY_INTEGRATOR_TAG			"controller.config.vel_integrator_gain"	//float
#define VELOCITY_GAIN_TAG				"controller.config.vel_gain"			//float
#define CONTROL_MODE_TAG				"controller.config.control_mode"		//int
#define VELOCITY_LIMIT_TAG				"controller.config.vel_limit"			//float

//Error
#define ERROR_TAG						"controller.error"						//int
#define DRV_FAULT_TAG					"motor.gate_driver.drv_fault"			//int

//current
#define CURRENT_IGAIN_TAG				"motor.current_control.i_gain"			//float
#define CURRENT_PGAIN_TAG				"motor.current_control.p_gain"			//float
#define BUS_CURRENT_TAG					"motor.current_control.Ibus"			//float
#define MAX_ALLOWED_CURRENT_TAG			"motor.current_control.max_allowed_current"	//float
#define PHASE_B_IMEAS	

#define PRE_CALIBRATED_TAG				"motor.config.pre_calibrated"			//bool
#define IS_CALIBRATED_TAG				"motor.is_calibrated"					//bool
#define POLE_PAIRS_TAG					"motor.confog.pole_pairs"				//int				
#define PHASE_B_CURRENT_TAG					"motor.current_meas_phB"			//float
#define PM_FLIX_LINKAGE_TAG				"sensorless_estimator.config.pm_flux_linkage"	//float


//Controller Data Tags
#define SAVE_CONFIGURATION_TAG	"save_configuration"
#define REBOOT_TAG				"reboot"

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

void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], int value = 0);
void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], float value = 0.0);
void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], bool value = TRUE);


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

		//Startup and states
		void writeState(uint8_t state);
		void  requestState();
		void writeControlMode(uint8_t mode);
		void  requestControlMode();
		void writeStartupClosedLoop(bool b_startup);
		void  requestStartupClosedLoop();
		void writeStartupSensorless(bool b_startup);
		void  requestStartupSensorless();
		void writeStartupCalibrate(bool b_startup);
		void  requestStartupCalibrate();

		void writeSpinUpAcceleration(uint16_t acceleration);
		void requestSpinUpAcceleration();
		void writeSpinUpTarrequestVel(uint16_t speed);
		void requestSpinUpTarrequestVel();
		void writeSpinUpCurrent(uint16_t current);
		void requestSpinUpCurrent();
		
		void writeVelRampTarrequest(uint16_t tarrequest);
		void requestVelRampTarrequest();
		void writeVelRampRate(uint16_t rate);
		void requestVelRampRate();
		void writeVelrampEnable(bool tarrequest);
		void requestVelrampEnable();

		void writeVelSetpoint(uint16_t writepoint);
		void requestVelSetpoint();
		
		void writePolepairs(uint8_t kv);
		void requestPolepairs();
		void writeKV(uint16_t kv);
		void requestKV();
		
		void writeVelocityGain(float gain);
		void requestVelocityGain();
		void writeVelocityIntegratorGain(float gain);
		void requestVelocityIntegratorGain();
		void writeVelocityLimit(float limit);
		void requestVelocityLimit();

		void writeCurrentGain(float gain);
		void requestCurrentGain();
		void writeCurrentIntegratorGain(float gain);
		void requestCurrentIntegratorGain();
		void writeCurrentLimit(float limit);
		void requestCurrentLimit();

		void requestPhaseCurrent();
		void requestBusCurrent();

		void writePMFluxLinkage(float linkage);
		void requestPMFluxLinkage();

		void requestError();
		void requestDRVError();
		bool requestIsPreCalibrated();
		bool requestIsCalibrated();
		
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
		
		//Motor Parameters
		uint8_t pole_pairs;
		uint16_t motor_kv;

		Stream& m_serial;
		
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
		void ping();
		
		Stream& m_serial;
		parsePacket(ODrivePacket packet);
};

#endif
