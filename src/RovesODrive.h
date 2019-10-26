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
 #define VELOCITY_LIMIT  				"trap_traj.config.vel_limit" //float
 #define ACCELERATION_LIMIT				"trap_traj.config.accel_limit" //float
 #define DECELERATION_LIMIT				"trap_traj.config.decel_limit" //float
 #define ACCELERATION_PER_COUNTS		"trap_traj.config.A_per_css" //float
 #define CURRENT_LIMIT					"motor.config.current_lim" //float
 #define VELOCITY_LIMIT_CONFIG			"controller.config.vel_limit" //float 

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

void writeODriveConfig(HardwareSerial* mySerial, bool write_request, char* id, char* value, uint8_t axis);

void writeODriveCommand(HardwareSerial* mySerial, char* id, char* value, uint8_t axis);



struct ODrivePacket
{
	uint8_t length;
	String data[];
};
	
class RovesODriveMotor
{
	public:
		String getSerial();

		Error_Axis checkAxisErrors();

		Error_Motor checkMotorErrors();

		Error_Encoder checkEncoderErrors();

		Error_Controller checkControllerErrors();

		void setTrapVelocityLimit(float limit);

		void setTrapAccelerationLimit(float limit);

		void setTrapDecelerationLimit(float limit);

		void setTrapAccelerationPerCounts(float limit);

		void reboot();

		void saveConfig();

		void eraseConfig();

		void setTrapTarget(int32_t target);

		void aetPosSetPoint(int32_t target);

		float requestPosEstimate();

		uint8_t motor_number;

		HardwareSerial* m_serial;

		void requestState();

		void requestControlMode();

		void writeState(Axis_State state);
		
		void writeControlMode(uint8_t mode);		
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
