#ifndef AGILEX_SCOUT_PROTOCO_
#define AGILEX_SCOUT_PROTOCO_

/*motion parameters of scout*/
#define MAX_LIN_VELOCITY       1.5  //1.5m/s
#define MAX_ANG_VELOCITY       (0.7853*1) //0.7853rad/s
#define PI                     3.14159265

/*CAN ids of agilex scout*/
#define CAN_ID_CHASSIS_STATUS  0x151
#define CAN_ID_MOTION_STATUS   0x131
#define CAN_ID_MOTION_CONTROL   0x130


/*0x151: chassis status filed*/
/*
byte0: chassis status, 00 : ok; 01: error
byte1: control_mode
byte2: battery voltage high byte
byte3: battery voltage low byte
byte4: error info high byte
byte5: error info low byte
byte6: count
byte7: check_sum
*/

/*0x131: motion status filed*/
/*
byte0: linear velocity high byte     (real velocity)*1000
byte1: linear velocity low byte
byte2: angular velocity high byte
byte3: angular velocity low byte
byte4: RSV
byte5: RSV
byte6: count
byte7: check_sum
*/

/*0x130: motion control filed*/
/*
byte0: control_mode
byte1: clear error cmd
byte2: linear_velocity_percentage  xx% of 1.5m/s
byte3: angular_velocity_percentage xx% of 0.7853 rad/s
byte4: RSV
byte5: RSV
byte6: count
byte7: check_sum
*/
enum control_mode
{
	REMOTE_MODE=0,
	CMD_MODE
};

enum clear_err_cmd
{
	NO_CLEAR=0,
	CLEAR_BATTERY_LOW,
	CLEAR_BATTERY_OVER,
	CLEAR_MOTOR1_COMM,
	CLEAR_MOTOR2_COMM,
	CLEAR_MOTOR3_COMM,
	CLEAR_MOTOR4_COMM,
	CLEAR_MOTOR_OVER_TEMP,
	CLEAR_MOTOR_OVER_CURR
};

#endif