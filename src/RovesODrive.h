#ifndef _RovesODrive_h
#define _RovesODrive_h

#include <stdio.h>
#include <Energia.h>
#include <HardwareSerial.h>

#define MAX_STRING_CHARS 255

#define WRITE	true
#define	READ	false

//Axis Data Tags

//Configs
#define BRAKE_RESISTANCE				"config.brake_resistance"				//float	
#define CALIBRATION_CURRENT				"motor.config.calibration_current"		//float
#define CONTROL_MODE					"controller.config.control_mode"		//int
#define CURRENT_LIMIT					"motor.config.current_lim"				//float
#define POLE_PAIRS						"motor.config.pole_pairs"				//int
#define READ_CURRENT_STATE				"current_state"							//int
#define VELOCITY_LIMIT					"controller.config.vel_limit"			//float
#define WRITE_CURRENT_STATE				"requested_state"						//int
#define WATCHDOG						"config.watchdog_timeout"				//int

//Controller
#define CURRENT_SETPOINT				"controller.current_setpoint"			//float
#define VELOCITY_RAMP_ENABLE			"controller.vel_ramp_enable"			//bool
#define VELOCITY_RAMP_RATE				"controller.config.vel_ramp_rate"		//float
#define VELOCITY_RAMP_TARGET			"controller.vel_ramp_target"			//float
#define VELOCITY_SETPOINT				"controller.vel_setpoint"				//float

//Encoder 
#define CPR								"encoder.config.cpr"					//float
#define CPR_SETPOINT					"controller.config.setpoints_in_cpr"	//float
#define POS_CPR							"encoder.pos_cpr"						//float
#define POS_ESTIMATE					"encoder.pos_estimate"					//float

//Errors
#define AXIS_ERRORS						"error"									//Error_Axis
#define CONTROLLER_ERRORS				"controller.error"						//Error_Controller
#define ENCODER_ERRORS					"encoder.error"							//Error_Encoder
#define MOTOR_ERRORS					"motor.error"							//Error_Motor

//Gains
#define POSITION_GAIN					"controller.config.pos_gain"			//float
#define VELOCITY_GAIN					"controller.config.vel_gain"			//float
#define VELOCITY_iNTEGRATOR_GAIN		"controller.config.vel_integrator_gain" //float

//Motor Control
#define MOTOR_CURRENT					"c"										//float
#define MOTOR_POS						"p"										//float, float, float
#define MOTOR_TRAJ						"t"										//float
#define MOTOR_VEL						"v"										//float

//System Commands
#define ERASE_CONFIG					"se"									//void
#define SAVE_CONFIG						"ss"									//void
#define SYSTEM_REBOOT					"sr"									//void

//Trap Trajectory
#define TRAP_ACCELERATION_LIMIT			"trap_traj.config.accel_limit" 			//float
#define TRAP_ACCELERATION_PER_COUNTS	"trap_traj.config.A_per_css" 			//float
#define TRAP_DECELERATION_LIMIT			"trap_traj.config.decel_limit" 			//float
#define TRAP_VELOCITY_LIMIT  			"trap_traj.config.vel_limit" 			//float

enum Axis_State
{
	AXIS_STATE_IDLE = 1,
	AXIS_STATE_STARTUP_SEQUENCE = 2,
	AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
	AXIS_STATE_MOTOR_CALIBRATION_SEQUENCE = 4,
	AXIS_STATE_SENSORLESS_CONTROL = 5,
	AXIS_STATE_ENCODER_INDEX_SEARCH = 6,
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
	AXIS_STATE_CLOSED_LOOP_CONTROL = 8,	
	};

enum Control_Mode
{
	CTRL_MODE_POSITION_CONTROL = 0,
	CTRL_MODE_VELOCITY_CONTROL = 1,
	CTRL_MODE_CURRENT_CONTROL = 2,
	CTRL_MODE_VOLTAGE_CONTROL = 3,
	CTRL_MODE_SENSORLESS_VELOCITY_CONTROL = 4,
};
enum Error_Axis
{
	ERROR_NONE_A = 0,
    ERROR_INVALID_STATE = 1, 
    ERROR_DC_BUS_UNDER_VOLTAGE = 2,
    ERROR_DC_BUS_OVER_VOLTAGE = 3,
    ERROR_CURRENT_MEASUREMENT_TIMEOUT = 4,
    ERROR_BRAKE_RESISTOR_DISARMED = 5, 
    ERROR_MOTOR_DISARMED = 6, 
    ERROR_MOTOR_FAILED = 7, 
    ERROR_SENSORLESS_ESTIMATOR_FAILED = 8,
    ERROR_ENCODER_FAILED = 9, 
    ERROR_CONTROLLER_FAILED = 10,
};

enum Error_Controller
{
	ERROR_NONE_C = 0,
    ERROR_OVERSPEED = 1,
};

enum Error_Encoder
{
	ERROR_NONE_E = 0,
    ERROR_UNSTABLE_GAIN = 1,
    ERROR_CPR_OUT_OF_RANGE = 2,
    ERROR_NO_RESPONSE = 3,
    ERROR_UNSUPPORTED_ENCODER_MODE = 4,
    ERROR_ILLEGAL_HALL_STATE = 5,
    ERROR_INDEX_NOT_FOUND_YET = 6,
};

enum Error_Motor
{
	ERROR_NONE_M = 0,
    ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 1,
    ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 2,
    ERROR_ADC_FAILED = 3,
    ERROR_DRV_FAULT = 4,
    ERROR_CONTROL_DEADLINE_MISSED = 5,
    ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 6,
    ERROR_BRAKE_CURRENT_OUT_OF_RANGE = 7,
    ERROR_MODULATION_MAGNITUDE = 8,
    ERROR_BRAKE_DEADTIME_VIOLATION = 9,
    ERROR_UNEXPECTED_TIMER_CALLBACK = 10,
    ERROR_CURRENT_SENSE_SATURATION = 11,
    ERROR_INVERTER_OVER_TEMP = 12,
    ERROR_CURRENT_UNSTABLE = 13,
};

enum Packet_Status 
{
	ValidPacket = 0, 
	InvalidPacket = 1, 
	NoPacket = 2, 
	OverflowPacket = 3,
};

enum Serial_Status 
{
	SerialGood = 0, 
	SerialFault = 1,
};

//Serial Command and Config
void writeODriveConfig(HardwareSerial* mySerial, bool write_read, char* id, char* param, uint8_t axis);

void writeODriveConfig(HardwareSerial* mySerial, bool write_read, char* id, float& param, uint8_t axis);

void writeODriveCommand(HardwareSerial* mySerial, char* id, char* param1, char* param2, char* param3, uint8_t axis);


class RovesODriveMotor 
{
	public:
		//class member variables
		HardwareSerial* m_serial;

		int32_t m_position = 0;

		uint8_t motor_number;

		// member functions
		Error_Axis checkAxisErrors();

		Error_Controller checkControllerErrors();

		Error_Encoder checkEncoderErrors();

		Error_Motor checkMotorErrors();

		float readPosCPR();

		float readPosEstimate();

		String* checkAllErrors();

		String getSerial();

		void eraseConfig();

		void writeTrapVelocityLimit(float limit);

		float readTrapVelocityLimit();

		void writeTrapAccelerationLimit(float limit);

		float readTrapAccelerationLimit();

		void writeTrapDecelerationLimit(float limit);

		float readTrapDecelerationLimit();

		void writeTrapAccelerationPerCounts(float limit);

		float readTrapAccelerationPerCounts();

		float readCurrentSetPoint();

		void reboot();

		void saveConfig();

		void writeTrapTarget(int32_t target);

		void writePosSetPoint(int32_t position, int32_t velFF, int32_t crrtFF);

		void writeVelocityGain(float target);

		float readVelocityGain();

		void writePositionGain(float target);

		float readPositionGain();

		void writeVelocityIntegratorGain(float target);

		float readVelocityIntegratorGain();

		Axis_State readState();

		Control_Mode readControlMode();

		void writeVelocityControlMode();

		void writeVelocitySetpoint(float velPoint, float currentFF);

		float readVelocitySetpoint();

		void writeCurrentLimit(float limit);

		float readCurrentLimit();

		void writeVelocityLimit(float limit);

		float readVelocityLimit();

		void writeCurrentCalibration(float limit);

		float readCurrentCalibration();

		void writeBrakeResistance(float bResist);

		float readBrakeResistance();

		void writePolePairs(int32_t pairs);

		int32_t readPolePairs();

		void writeEncoderCPR(float cpr);

		float readEncoderCPR();

		void writeVelocityRampEnable(bool state);

		bool readVelocityRampEnable();

		void writeVelocityRampRate(float rRate);

		float readVelocityRampRate();

		void writeCurrentSetPoint(float point);

		void writeVelocityRampTarget(float target);

		float readVelocityRampTarget();

		void writeCPRSetpoint(bool state);

		void writeWatchdogTimeout(int32_t time);

		int32_t readWatchdogTimeout();

		void writeState(Axis_State state);
		
		void writeControlMode(Control_Mode mode);		
};

class RovesODrive
{
	public:
		RovesODrive(HardwareSerial* mySerial)
		{
			m_serial = mySerial;
			this->motor[0].motor_number = 0;
			this->motor[0].m_serial = mySerial;
			this->motor[1].motor_number = 1;
			this->motor[1].m_serial = mySerial;
		}

		void begin();

		bool isConnected();

		void read();
		
		RovesODriveMotor motor[2];

	private:
		HardwareSerial* m_serial;
};

#endif
