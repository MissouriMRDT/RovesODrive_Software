#ifndef _RovesODrive_h
#define _RovesODrive_h

#include <stdio.h>
#include <Energia.h>
#include <HardwareSerial.h>

#define MAX_STRING_CHARS 255

#define WRITE	true
#define	READ	false

//Axis Data Tags

//configs
#define GET_CURRENT_STATE_TAG			"current_state"							//int
#define SET_CURRENT_STATE_TAG			"requested_state"						//int
#define CONTROL_MODE_TAG				"controller.config.control_mode"		//int	

//Trap Trajectory
 #define VELOCITY_LIMIT  				"trap_traj.config.vel_limit" 			//float
 #define ACCELERATION_LIMIT				"trap_traj.config.accel_limit" 			//float
 #define DECELERATION_LIMIT				"trap_traj.config.decel_limit" 			//float
 #define ACCELERATION_PER_COUNTS		"trap_traj.config.A_per_css" 			//float

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

enum Error_Controller
{
	ERROR_NONE_C = 0,
    ERROR_OVERSPEED = 1,
};

int charToInt(char input[]);

float charToFloat(char input[]);

bool charToBool(char input[]);

void intToChar(char* output, int value);

void boolToChar(char* output, int value);

void floatToChar(char* output, int value, uint8_t precision);

void writeODriveConfig(HardwareSerial* mySerial, bool write_request, char* id, char* value1, char* value2, char* value3, uint8_t axis);

void writeODriveCommand(HardwareSerial* mySerial, char* id, char* value, uint8_t axis);

class RovesODriveMotor //TODO: Create a constructor for this
{
	public:
		String getSerial();

		Error_Axis checkAxisErrors();

		Error_Motor checkMotorErrors();

		Error_Encoder checkEncoderErrors();

		Error_Controller checkControllerErrors();

		String* checkAllErrors(String error[4]);

		void writeTrapVelocityLimit(float limit);

		void readTrapVelocityLimit();

		void writeTrapAccelerationLimit(float limit);

		void readTrapAccelerationLimit();

		void writeTrapDecelerationLimit(float limit);

		void readTrapDecelerationLimit();

		void writeTrapAccelerationPerCounts(float limit);

		void readTrapAccelerationPerCounts();

		void reboot();

		void saveConfig();

		void eraseConfig();

		void writeTrapTarget(int32_t target);

		void writePosSetPoint(int32_t target1, int32_t target2, int32_t target3);

		void writeVelocityGain(float target);

		void readVelocityGain();

		void writePositionGain(float target);

		void readPositionGain();

		void writeVelocityIntegratorGain(float target);

		void readVelocityIntegratorGain();

		float readPosEstimate();

		int32_t m_position = 0;

		uint8_t motor_number;

		HardwareSerial* m_serial;

		void readState();

		void readControlMode();

		void writeVelocityControlMode();

		void writeVelocitySetpoint(float velPoint, float currentFF);

		void writeCurrentLimit(float limit);

		void readCurrentLimit();

		void writeVelocityLimit(float limit);

		void readVelocityLimit();

		void writeCurrentCalibration(float limit);

		void readCurrentCalibration();

		void writeBrakeResistance(float bResist);

		void readBrakeResistance();

		void writePolePairs(int32_t pairs);

		void readPolePairs();

		void writeEncoderCPR(float cpr);

		void readEncoderCPR();

		void moveToPos(float pos);

		void moveIncremental(float incr);

		void writeVelocityRampEnable(bool state);

		void readVelocityRampEnable();

		void writeVelocityRampRate(float rRate);

		void readVelocityRampRate();

		void writeCurrentSetPoint(float point);

		void writeVelocityRampTarget(float target);

		void readVelocityRampTarget();

		void writeCPRSetpoint(bool state);

		float readPosCPR();

		void writeWatchdogTimeout(int32_t time);

		void readWatchdogTimeout();

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

		void read();

		bool isConnected();
		
		RovesODriveMotor motor[2];

	private:
		HardwareSerial* m_serial;
};

#endif
