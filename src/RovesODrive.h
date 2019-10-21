#ifndef _RovesODrive_h
#define _RovesODrive_h

#include <stdio.h>
#include <Energia.h>
#include <HardwareSerial.h>

#define MAX_STRING_CHARS 255

#define WRITE	true
#define	READ	false

#define MAX_VELOCITY_RAMP_RATE 10000

//Axis Data Tags
//configs
#define GET_CURRENT_STATE_TAG			"current_state"							//int
#define SET_CURRENT_STATE_TAG			"requested_state"						//int
#define STARTUP_CLOSED_LOOP_TAG 		"config.startup_closed_loop_control"	//bool
#define STARTUP_MOTOR_CALBRATION_TAG	"config.startup_motor_calibration"		//bool

//Velocity Config
#define VELOCITY_SETPOINT_TAG	 		"controller.vel_setpoint"				//float
#define VELOCITY_INTEGRATOR_TAG			"controller.config.vel_integrator_gain"	//float
#define VELOCITY_GAIN_TAG				"controller.config.vel_gain"			//float
#define CONTROL_MODE_TAG				"controller.config.control_mode"		//int
#define VELOCITY_LIMIT_TAG				"controller.config.vel_limit"			//float

//Error
#define ERROR_TAG						"controller.error"						//int
#define DRV_FAULT_TAG					"motor.gate_driver.drv_fault"			//int

#define PRE_CALIBRATED_TAG				"motor.config.pre_calibrated"			//bool
#define IS_CALIBRATED_TAG				"motor.is_calibrated"					//bool
#define POLE_PAIRS_TAG					"motor.config.pole_pairs"				//int				

//Trap Trajectory
 #define VELOCITY_LIMIT  				"trap_traj.config.vel_limit" //float
 #define ACCELERATION_LIMIT				"trap_traj.config.accel_limit" //float
 #define DECELERATION_LIMIT				"trap_traj.config.decel_limit" //float
 #define ACCELERATION_PER_COUNTS		"trap_traj.config.A_per_css" //float
 #define CURRENT_LIMIT					"motor.config.current_lim" //float
 #define VELOCITY_LIMIT_CONFIG			"controller.config.vel_limit" //float

#define PM_FLUX_LINKAGE_CONST 	5.51328895422 

enum Axis_State
	{
	AXIS_STATE_IDLE,
	AXIS_STATE_STARTUP_SEQUENCE,
	AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
	AXIS_STATE_MOTOR_CALIBRATION_SEQUENCE,
	AXIS_STATE_SENSORLESS_CONTROL,
	AXIS_STATE_ENCODER_INDEX_SEARCH,
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
	AXIS_STATE_CLOSED_LOOP_CONTROL	
	};

enum Control_Mode
	{
	CTRL_MODE_POSITION_CONTROL,
	CTRL_MODE_VELOCITY_CONTROL,
	CTRL_MODE_CURRENT_CONTROL,
	CTRL_MODE_VOLTAGE_CONTROL,
	CTRL_MODE_SENSORLESS_VELOCITY_CONTROL
	};

enum Packet_Status 
	{
	ValidPacket, 
	InvalidPacket, 
	NoPacket, 
	OverflowPacket
	};

enum Serial_Status 
	{
	SerialGood, 
	SerialFault
	};

enum Error_Axis
{
	ERROR_NONE_A,
    ERROR_INVALID_STATE, 
    ERROR_DC_BUS_UNDER_VOLTAGE,
    ERROR_DC_BUS_OVER_VOLTAGE,
    ERROR_CURRENT_MEASUREMENT_TIMEOUT,
    ERROR_BRAKE_RESISTOR_DISARMED, 
    ERROR_MOTOR_DISARMED, 
    ERROR_MOTOR_FAILED, 
    ERROR_SENSORLESS_ESTIMATOR_FAILED,
    ERROR_ENCODER_FAILED, 
    ERROR_CONTROLLER_FAILED
};

enum Error_Motor
{
	ERROR_NONE_M,
    ERROR_PHASE_RESISTANCE_OUT_OF_RANGE,
    ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE,
    ERROR_ADC_FAILED,
    ERROR_DRV_FAULT,
    ERROR_CONTROL_DEADLINE_MISSED,
    ERROR_NOT_IMPLEMENTED_MOTOR_TYPE,
    ERROR_BRAKE_CURRENT_OUT_OF_RANGE,
    ERROR_MODULATION_MAGNITUDE,
    ERROR_BRAKE_DEADTIME_VIOLATION,
    ERROR_UNEXPECTED_TIMER_CALLBACK,
    ERROR_CURRENT_SENSE_SATURATION,
    ERROR_INVERTER_OVER_TEMP,
    ERROR_CURRENT_UNSTABLE
};

enum Error_Encoder
{
	ERROR_NONE_E,
    ERROR_UNSTABLE_GAIN,
    ERROR_CPR_OUT_OF_RANGE,
    ERROR_NO_RESPONSE,
    ERROR_UNSUPPORTED_ENCODER_MODE,
    ERROR_ILLEGAL_HALL_STATE,
    ERROR_INDEX_NOT_FOUND_YET
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

		int checkAxisErrors();

		int checkMotorErrors();

		int checkEncoderErrors();

		int checkControllerErrors();

		void reboot();

		void saveConfig();

		void eraseConfig();

		void setControlMode(Control_Mode mode);

		void setTrapTarget(int32_t target);

		float requestPosEstimate();

		void setSpeed(int16_t speed);

		void setRampValue(int16_t value);

		void setPolePairs(uint8_t pole_pairs);

		void setKV(uint16_t KV);
		
		void calibrate();

		void writeConfig();

		bool speedLow(int16_t speed);

		uint8_t motor_number;

		HardwareSerial* m_serial;

		void requestState();

		void setRamp(uint16_t rate);

		void setSpinupAccleleration(uint16_t pos);

		void setRampRate(uint16_t rate);

		void idleMotor();

		
		
	private:
		void setDirection(int8_t direction);

		//Startup and states
		void writeState(Axis_State state);
		
		void writeControlMode(uint8_t mode);

		void requestControlMode();

		void writeStartupClosedLoop(bool b_startup);

		void requestStartupClosedLoop();

		void writeStartupSensorless(bool b_startup);

		void requestStartupSensorless();

		void writeStartupCalibrate(bool b_startup);

		void requestStartupCalibrate();

		void writeSpinUpAcceleration(int16_t acceleration);

		void requestSpinUpAcceleration();

		void writeSpinUpTargetVel(int16_t speed);

		void requestSpinUpTargetVel();

		void writeSpinUpCurrent(uint16_t current);

		void requestSpinUpCurrent();

		void writeSpinUpTime(float time);

		void writeDirection(int8_t value);

		void writeCurrentSetopint(uint16_t setpoint);
		
		void writeVelRampTarget(int16_t target);

		void requestVelRampTarget();

		void writeVelRampRate(int16_t rate);

		void requestVelRampRate();

		void writeVelrampEnable(bool enabled);

		void requestVelrampEnable();

		void writeVelSetpoint(int16_t setpoint);

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

		void requestIsPreCalibrated();

		void requestIsCalibrated();
		
		//State vars
		Control_Mode m_control_mode;

		int8_t m_direction = 1;
		//Member Vars
		
		int16_t vel_shutoff_threshold = 100;
		
		int16_t vel_setpoint = 0;
		
		//Spin Up parameters
		int16_t spin_up_acceleration = 200;

		int16_t spin_up_target_vel = 200;

		int16_t spin_up_current = 20;

		int16_t current_setpoint = 100;

		bool do_current_ramp = true;

		int16_t current_ramp_start = 30;

		int16_t current_ramp_end = 100;

		int16_t current_ramp_inc = 5;

		int16_t idle_current = 20;
		
		//Ramp Parameters
		int16_t vel_ramp_target;

		int16_t vel_ramp_rate = 200;
		
		//Motor Parameters
		uint8_t motor_pole_pairs;

		uint16_t motor_kv;

		
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
		void saveConfiguration();

		void eraseConfiguration();

		void ping();
		
		HardwareSerial* m_serial;
};

#endif
