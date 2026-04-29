#ifndef __UPPER_H
#define __UPPER_H

#include <stdint.h>
#include "camera_protocol.h"

void Camera_SendFrame(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len);
void Camera_SendAck(uint8_t ack_type, uint8_t ack_seq);
void Camera_RequestAvoidExit(void);
void Camera_PeriodicTx(void);
uint8_t Camera_IsAlive(void);

#endif
