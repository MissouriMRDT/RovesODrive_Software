#include "ODriveArduino.h"
#include "RoveComm.h"
#include <stdio.h>
#include "RovesODrive.h"

#define MAX_OUT_CHARS 50

char buffer[MAX_OUT_CHARS+1];

ODriveArduino odrive(Serial7);
RoveCommEthernetUdp RoveComm;

int left_speed = 0;
int right_speed = 0;

void setSpeed(int speed);

void setup() {
  Serial7.begin(115200);
  Serial.begin(115200);
  delay(10);

  Serial7.println("");

  Serial7.write("w axis0.requested_state AXIS_STATE_IDLE\n");
  Serial7.write("w axis0.requested_state 5\n");
  //Serial7.write("w axis0.controller.vel_setpoint 0\n");
  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET);
  delay(100);
  Serial.println("Initialized");
  //pinMode(PJ_0, INPUT_PULLUP);
  //pinMode(PJ_1, INPUT_PULLUP);
}

void loop() {
  //odrive.SetVelocity(0, 600);
  /*
  Serial7.write("w axis0.requested_state 1\n");
  delay(5000);
  //odrive.SetVelocity(0, 0);
  Serial7.write("w axis0.requested_state 5\n");
  delay(1000);
  */
  /*
  int speed = 0;
  if(!digitalRead(PJ_0))
  {
    speed = 500;
  }

  if(!digitalRead(PJ_1))
  {
    speed = -500;
  }
  setSpeed(speed);
  */
  
  rovecomm_packet packet;

  packet = RoveComm.read();

  switch(packet.data_id)
  {
    case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID:
      left_speed = packet.data[0];
      right_speed = packet.data[1];
      setSpeed(left_speed);
  }
  
  //Serial7.write("r axis0.current_state\n");
  delay(100);
  
  if(Serial7.available())
  {
    while(Serial7.available())
  {
    Serial.write(Serial7.read());
  }
  Serial.println("");
  }
}

void setSpeed(int speed)
{
  static int last_speed = 0;

  //Serial.println(speed);
  speed = speedHigh(speed)? speed : 0;

  Serial7.write("w axis0.controller.vel_ramp_enable 1\n");

  sprintf(buffer, "w axis0.controller.vel_ramp_target %d\n", speed);
  Serial.println(buffer);
  Serial7.write(buffer);

  Serial7.write("r axis0.controller.vel_ramp_target\n");
  

  if(!speedHigh(last_speed) && speedHigh(speed))  //If speeding up from low speed
  {
    Serial.println("Ramp Up");
    if(speed>0) 
    {
      Serial7.write("w axis0.config.spin_up_target_vel 100\n");
      //Serial.println(100);
    }
    else
    {
      Serial7.write("w axis0.config.spin_up_target_vel -100\n");
      //Serial.println(-100);
    }
    
    Serial7.write("w axis0.requested_state 1\n");              //Set state
    Serial7.write("w axis0.requested_state 5\n"); 
  }

  last_speed = speed;
}

bool speedHigh(int speed)
{
  return abs(speed)>100? 1:0;
}

