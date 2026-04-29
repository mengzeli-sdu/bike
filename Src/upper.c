#include "upper.h"
#include "usart.h"
#include "car_task.h"
#include "camera_protocol.h"

/*
 * UART7 摄像头通信协议：
 * A5 5A TYPE SEQ LEN PAYLOAD XOR
 * XOR 校验范围：TYPE, SEQ, LEN, PAYLOAD
 */

static volatile uint32_t camera_last_rx_tick = 0;

static uint8_t avoid_exit_pending = 0;
static uint8_t avoid_exit_seq = 1;
static uint32_t avoid_exit_last_send_tick = 0;

uint8_t Camera_XorChecksum(uint8_t type, uint8_t seq, uint8_t len, const uint8_t *payload)
{
    uint8_t x = 0;
    x ^= type;
    x ^= seq;
    x ^= len;
    for(uint8_t i = 0; i < len; i++)
    {
        x ^= payload[i];
    }
    return x;
}

void Camera_SendFrame(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    uint8_t checksum;

    if(len > CAM_MAX_PAYLOAD)
    {
        return;
    }

    checksum = Camera_XorChecksum(type, seq, len, payload);

    LL_USART_TransmitData8(UART7, CAM_HEAD1);
    while((UART7->SR & 0x40) == 0){}

    LL_USART_TransmitData8(UART7, CAM_HEAD2);
    while((UART7->SR & 0x40) == 0){}

    LL_USART_TransmitData8(UART7, type);
    while((UART7->SR & 0x40) == 0){}

    LL_USART_TransmitData8(UART7, seq);
    while((UART7->SR & 0x40) == 0){}

    LL_USART_TransmitData8(UART7, len);
    while((UART7->SR & 0x40) == 0){}

    for(uint8_t i = 0; i < len; i++)
    {
        LL_USART_TransmitData8(UART7, payload[i]);
        while((UART7->SR & 0x40) == 0){}
    }

    LL_USART_TransmitData8(UART7, checksum);
    while((UART7->SR & 0x40) == 0){}
}

void Camera_SendAck(uint8_t ack_type, uint8_t ack_seq)
{
    uint8_t payload[2];
    payload[0] = ack_type;
    payload[1] = ack_seq;
    Camera_SendFrame(CAM_MSG_ACK, ack_seq, payload, 2);
}

void Camera_RequestAvoidExit(void)
{
    avoid_exit_pending = 1;
    avoid_exit_seq++;
    if(avoid_exit_seq == 0)
    {
        avoid_exit_seq = 1;
    }
    avoid_exit_last_send_tick = 0;
}

void Camera_PeriodicTx(void)
{
    uint8_t payload[1] = {0};

    if(avoid_exit_pending)
    {
        if(HAL_GetTick() - avoid_exit_last_send_tick >= 50)
        {
            avoid_exit_last_send_tick = HAL_GetTick();
            Camera_SendFrame(MCU_MSG_AVOID_EXIT, avoid_exit_seq, payload, 1);
        }
    }
}

uint8_t Camera_IsAlive(void)
{
    return (HAL_GetTick() - camera_last_rx_tick < 500) ? 1 : 0;
}

static int16_t be16_to_i16(uint8_t h, uint8_t l)
{
    return (int16_t)(((uint16_t)h << 8) | (uint16_t)l);
}

static void Camera_OnFrame(uint8_t type, uint8_t seq, uint8_t *payload, uint8_t len)
{
    camera_last_rx_tick = HAL_GetTick();

    if(type == CAM_MSG_ACK)
    {
        if(len >= 2)
        {
            uint8_t ack_type = payload[0];
            uint8_t ack_seq  = payload[1];

            if(ack_type == MCU_MSG_AVOID_EXIT && ack_seq == avoid_exit_seq)
            {
                avoid_exit_pending = 0;
            }
        }
        return;
    }

    if(type == CAM_MSG_HEARTBEAT)
    {
        return;
    }

    if(type == CAM_MSG_LINE)
    {
        if(len >= 4)
        {
            int16_t err = be16_to_i16(payload[0], payload[1]);
            uint8_t confidence = payload[2];
            uint8_t vision_state = payload[3];
            (void)vision_state;
            Car_ProcessCameraFrame(type, seq, err, CAM_SIDE_UNKNOWN, confidence);
        }
        return;
    }

    if(type == CAM_MSG_OBSTACLE || type == CAM_MSG_CROSSWALK || type == CAM_MSG_FINISH)
    {
        /* 关键事件必须 ACK。即使是重复事件，也要 ACK，避免摄像头一直重发。 */
        Camera_SendAck(type, seq);

        if(len >= 4)
        {
            int16_t err = be16_to_i16(payload[0], payload[1]);
            uint8_t side = payload[2];
            uint8_t confidence = payload[3];
            Car_ProcessCameraFrame(type, seq, err, side, confidence);
        }
        return;
    }
}

void UART7_IRQHandler(void)
{
    static uint8_t state = 0;
    static uint8_t type = 0;
    static uint8_t seq = 0;
    static uint8_t len = 0;
    static uint8_t payload[CAM_MAX_PAYLOAD];
    static uint8_t index = 0;

    uint8_t data;

    if(LL_USART_IsActiveFlag_RXNE(UART7) && LL_USART_IsEnabledIT_RXNE(UART7))
    {
        data = LL_USART_ReceiveData8(UART7);

        switch(state)
        {
            case 0:
                if(data == CAM_HEAD1)
                {
                    state = 1;
                }
                break;

            case 1:
                if(data == CAM_HEAD2)
                {
                    state = 2;
                }
                else if(data == CAM_HEAD1)
                {
                    state = 1;
                }
                else
                {
                    state = 0;
                }
                break;

            case 2:
                type = data;
                state = 3;
                break;

            case 3:
                seq = data;
                state = 4;
                break;

            case 4:
                len = data;
                index = 0;
                if(len == 0)
                {
                    state = 6;
                }
                else if(len <= CAM_MAX_PAYLOAD)
                {
                    state = 5;
                }
                else
                {
                    state = 0;
                }
                break;

            case 5:
                payload[index++] = data;
                if(index >= len)
                {
                    state = 6;
                }
                break;

            case 6:
            {
                uint8_t checksum = Camera_XorChecksum(type, seq, len, payload);
                if(checksum == data)
                {
                    Camera_OnFrame(type, seq, payload, len);
                }
                state = 0;
                break;
            }

            default:
                state = 0;
                break;
        }

        LL_USART_EnableIT_RXNE(UART7);
    }
}
