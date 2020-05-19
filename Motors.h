#ifndef _MOTORS_H
#define _MOTORS_H

#include <iostream>
#include "dynamixel_sdk.h"

using namespace std;
using namespace dynamixel;

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_OPERATING_MODE			11

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                          1                   // Dynamixel ID: 1
#define DXL_ID2                          2
#define DXL_ID3                          3
#define DXL_ID4                          4
#define DXL_ID5                          5
#define DXL_ID6                          6
#define DXL_ID7                          7
#define BAUDRATE                        57600
#define DEVICENAME                      "COM3"      // Check which port is being used on your controller
															// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      3072              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     1                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

class Motors
{
public:
	Motors();
	void moveTogether(int P1, int P2, int P3, int P4, int P5, int P6, int P7);
	void moveOne(int position, int dxl_id);
	void connect(int dxl_id);
	void disconnect(int dxl_id);
	void setParamGoalArray(int position);
	void backToNomial(vector<int> orgPos);
	int getOneMotorPosition(int dxl_id);
	vector<int> getAllMotorPosition();

private:
	PortHandler* portHandler;
	PacketHandler* packetHandler;
	int dxl_comm_result = COMM_TX_FAIL;
	bool dxl_addparam_result = false;                 // addParam result
	bool dxl_getdata_result = false;                  // GetParam result
	int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };
	bool isOpen = false;
	uint8_t param_goal_position[4];
	uint8_t dxl_error = 0;


};

#endif