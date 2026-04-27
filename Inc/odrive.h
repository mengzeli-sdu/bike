#ifndef __ODRIVE_H__
#define __ODRIVE_H__

#define AXIS1_CAN_NODE_ID (0x010) // 飞轮

#define AXIS0_CAN_NODE_ID (0x018)

#define NODE_ID(num) ((num) == 0 ? AXIS0_CAN_NODE_ID : AXIS1_CAN_NODE_ID)
typedef enum
{
    MSG_CO_NMT_CTRL = 0x000, // CANOpen NMT Message REC
    MSG_ODRIVE_HEARTBEAT,
    MSG_ODRIVE_ESTOP,
    MSG_GET_MOTOR_ERROR, // Errors
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,
    MSG_SET_AXIS_REQUESTED_STATE,
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,
    MSG_GET_ENCODER_COUNT,
    MSG_SET_CONTROLLER_MODES,
    MSG_SET_INPUT_POS,
    MSG_SET_INPUT_VEL,
    MSG_SET_INPUT_CURRENT,
    MSG_SET_VEL_LIMIT,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_A_PER_CSS,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_VBUS_VOLTAGE,
    MSG_CLEAR_ERRORS,
    MSG_CO_HEARTBEAT_CMD = 0x700, // CANOpen NMT Heartbeat  SEND
} OdriveMsg_t;

typedef struct
{
    float set_speed0; // 飞轮目标速度
    float set_speed1; // 后驱目标速度
    float fliter_speed0[3];
    float fliter_speed1[3];
    float now_speed0;
    float now_speed1;
    int speed0_i; //平均滤波滤波索引
    int speed1_i;
}OdirveTypeDef;

extern OdirveTypeDef odrive;
void odrive_canFilter_init(void);
void odrive_init(void);
void odrive_vel_callback(unsigned char num);
void odrive_speed_ctrl(unsigned char num, float speed);
#endif


