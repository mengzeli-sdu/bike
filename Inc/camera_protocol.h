#ifndef __CAMERA_PROTOCOL_H
#define __CAMERA_PROTOCOL_H

#include <stdint.h>

#define CAM_HEAD1                 0xA5
#define CAM_HEAD2                 0x5A
#define CAM_MAX_PAYLOAD           16

/* 摄像头 -> STM32 */
#define CAM_MSG_LINE              0x01
#define CAM_MSG_OBSTACLE          0x02
#define CAM_MSG_CROSSWALK         0x03
#define CAM_MSG_FINISH            0x04
#define CAM_MSG_HEARTBEAT         0x05

/* STM32 -> 摄像头 / 双向 */
#define CAM_MSG_ACK               0x80
#define MCU_MSG_STATE             0x81
#define MCU_MSG_AVOID_EXIT        0x82
#define MCU_MSG_HEARTBEAT         0x83

#define CAM_SIDE_UNKNOWN          0
#define CAM_SIDE_LEFT             1
#define CAM_SIDE_RIGHT            2

#define VISION_CENTER             0
#define VISION_FOLLOW_LEFT        1
#define VISION_FOLLOW_RIGHT       2

uint8_t Camera_XorChecksum(uint8_t type, uint8_t seq, uint8_t len, const uint8_t *payload);

#endif
