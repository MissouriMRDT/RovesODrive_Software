import odrive
from odrive.enums import *
import time

print("Connecting to drive")
odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))
axis = odrv0.axis0
mo = axis.motor
enc = axis.encoder


NEUMOTOR_POLEPAIRS = 4
NEUMOTOR_KV = 480

POLEPAIRS = NEUMOTOR_POLEPAIRS
MOTORKV = NEUMOTOR_KV


text = input("Tune Motor 0 [y/n]:")
if(text == 'y') or (text == 'Y'):
	odrv0.axis0.motor.config.pole_pairs = POLEPAIRS
	#Calibrate Motor
	print("Calibrating Motor")
	odrv0.axis0.motor.config.pre_calibrated = False
	odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
	time.sleep(.1)
	while odrv0.axis0.current_state == AXIS_STATE_MOTOR_CALIBRATION:
		print(".",end='')
		time.sleep(0.1)
	print("\nMotor Calibrated")
	odrv0.axis0.motor.config.pre_calibrated = True

text = input("Setup Sensorless Velocity Mode[y/n]:")
if(text == 'y') or (text == 'Y'):
	#Configure Closed Loop params
	odrv0.axis0.controller.config.vel_gain = 0.1
	odrv0.axis0.controller.config.vel_integrator_gain = 0.1
	odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
	odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (POLEPAIRS * MOTORKV)

	odrv0.axis0.requested_state = AXIS_STATE_IDLE
	odrv0.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL
	time.sleep(.1)

	#Tuning Parameters
	odrv0.axis0.controller.vel_ramp_target = 500
	odrv0.axis0.controller.vel_ramp_enable = True
	odrv0.axis0.config.startup_motor_calibration = True
	
	odrv0.axis0.config.spin_up_acceleration = 1000
	odrv0.axis0.config.ramp_up_time = .1
	odrv0.axis0.controller.current_setpoint = 20
	odrv0.axis0.config.spin_up_target_vel = 100
	odrv0.axis0.config.spin_up_current = 20
	odrv0.axis0.motor.config.current_lim = 27

	while(1):
		print(odrv0.axis0.controller.current_setpoint)
		text = input("RPM[q]:")
		if(text == 'q'):
			break
		else:
			try:
				odrv0.axis0.controller.vel_ramp_target = int(text)
			except:
				print("Must type an integer or '2'")
			
	time.sleep(.1)		
	odrv0.axis0.controller.vel_ramp_target = 0
	print(odrv0.axis0.controller.vel_setpoint)






text = input("Tune Motor 1 [y/n]:")
if(text == 'y') or (text == 'Y'):
	odrv0.axis1.motor.config.pole_pairs = POLEPAIRS
	
	#Calibrate Motor
	print("Calibrating Motor")
	odrv0.axis1.motor.config.pre_calibrated = False
	odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
	time.sleep(.1)
	while odrv0.axis1.current_state == AXIS_STATE_MOTOR_CALIBRATION:
		print(".",end='')
		time.sleep(0.1)
	print("\nMotor Calibrated")
	odrv0.axis1.motor.config.pre_calibrated = True

text = input("Setup Sensorless Velocity Mode[y/n]:")
if(text == 'y') or (text == 'Y'):
	#Configure Closed Loop params
	odrv0.axis1.controller.config.vel_gain = 0.1
	odrv0.axis1.controller.config.vel_integrator_gain = 0.1
	odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
	odrv0.axis1.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (POLEPAIRS * MOTORKV)

	odrv0.axis1.requested_state = AXIS_STATE_IDLE
	odrv0.axis1.requested_state = AXIS_STATE_SENSORLESS_CONTROL
	time.sleep(.1)

	#Tuning Parameters
	odrv0.axis1.controller.vel_ramp_target = 500
	odrv0.axis1.controller.vel_ramp_enable = True
	odrv0.axis1.config.startup_motor_calibration = True
	
	odrv0.axis1.config.spin_up_acceleration = 1000
	odrv0.axis1.config.ramp_up_time = .1
	odrv0.axis1.controller.current_setpoint = 20
	odrv0.axis1.config.spin_up_target_vel = 100
	odrv0.axis1.config.spin_up_current = 20
	odrv0.axis1.motor.config.current_lim = 27

	while(1):
		print(odrv0.axis1.controller.current_setpoint)
		text = input("RPM[q]:")
		if(text == 'q'):
			break
		else:
			try:
				odrv0.axis1.controller.vel_ramp_target = int(text)
			except:
				print("Must type an integer or '2'")
			
	time.sleep(.1)		
	odrv0.axis1.controller.vel_ramp_target = 0


text = input("Save configuration{y/n]:")
if(text == 'y') or (text == 'Y'):
	odrv0.save_configuration()


