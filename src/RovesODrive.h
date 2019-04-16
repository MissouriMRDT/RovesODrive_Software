#ifndef _RovesODrive_h
#define _RovesODrive_h

#include <stdio.h>
#include <Energia.h>
#include <HardwareSerial.h>

#define MAX_STRING_CHARS 255

#define WRITE	true
#define	REQUEST	false

#define MAX_VELOCITY_RAMP_RATE 10000

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
#define SPINUP_TIME_TAG					"config.ramp_up_time"					//float

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
#define PHASE_B_IMEAS					"motor.TODO"

#define PRE_CALIBRATED_TAG				"motor.config.pre_calibrated"			//bool
#define IS_CALIBRATED_TAG				"motor.is_calibrated"					//bool
#define POLE_PAIRS_TAG					"motor.confog.pole_pairs"				//int				
#define PHASE_B_CURRENT_TAG				"motor.current_meas_phB"				//float
#define PM_FLIX_LINKAGE_TAG				"sensorless_estimator.config.pm_flux_linkage"	//float

#define CURRENT_SETPOINT_TAG			"controller.current_setpoint"		//float
#define DIRECTION_TAG					"motor.config.direction"			//int


//Controller Data Tags
#define SAVE_CONFIGURATION_TAG	"save_configuration"
#define REBOOT_TAG				"reboot"

//Axis states
#define AXIS_STATE_IDLE 						1
#define AXIS_STATE_STARTUP_SEQUENCE  			2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE  	3
#define AXIS_STATE_MOTOR_CALIBRATION_SEQUENCE  	4
#define AXIS_STATE_SENSORLESS_CONTROL   		5
#define AXIS_STATE_ENCODER_INDEX_SEARCH   		6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION	7
#define AXIS_STATE_CLOSED_LOOP_CONTROL  		8

//Control modes
#define CTRL_MODE_POSITION_CONTROL				1
#define CTRL_MODE_VELOCITY_CONTROL				2
#define CTRL_MODE_CURRENT_CONTROL				3
#define CTRL_MODE_VOLTAGE_CONTROL 				4
#define CTRL_MODE_SENSORLESS_VELOCITY_CONTROL 	5

#define PM_FLUX_LINKAGE_CONST 	5.51328895422 

enum PacketStatus {ValidPacket, InvalidPacket, NoPacket, OverflowPacket};
enum SerialStatus {SerialGood, SerialFault};

int charToInt(char input[]);
float charToFloat(char input[]);
bool charToBool(char input[]);
void intToChar(char* output, int value);
void boolToChar(char* output, int value);
void floatToChar(char* output, int value, uint8_t precision);

void writeODrive(HardwareSerial* mySerial, bool write_request, char* id, char* value, uint8_t axis);



struct ODrivePacket
{
	uint8_t length;
	String data[];
};
	
class RovesODriveMotor
{
	public:
		PacketStatus getSerial(char packet[]);

		SerialStatus checkSerial();

		void setControlMode(uint8_t mode);
		void setSpeed(int16_t speed);
		PacketStatus getSpeed(int16_t &speed);
		int16_t getSpeed();
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

		void setSpinupAccleleration(uint16_t acceleration);

		void setRampRate(uint16_t rate);

		
		
	private:
		void setDirection(int8_t direction);

		//Startup and states
		void writeState(uint8_t state);
		
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
		uint8_t m_control_mode;

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
