#include "upper.h"
#include "usart.h"
#include "car_task.h"

#define RX_HEAD 0xA5

void UART7_IRQHandler(void)
{
    static uint8_t rx_state = 0;
    static uint8_t cmd = 0;
    static uint8_t err_h = 0;
    static uint8_t err_l = 0;
    static uint8_t checksum = 0;

    uint8_t data;

    if(LL_USART_IsActiveFlag_RXNE(UART7) && LL_USART_IsEnabledIT_RXNE(UART7))
    {
        data = LL_USART_ReceiveData8(UART7);

        switch(rx_state)
        {
            case 0:
                if(data == RX_HEAD)
                {
                    rx_state = 1;
                }
                break;

            case 1:
                cmd = data;
                rx_state = 2;
                break;

            case 2:
                err_h = data;
                rx_state = 3;
                break;

            case 3:
                err_l = data;
                rx_state = 4;
                break;

            case 4:
                checksum = data;

                if(checksum == (RX_HEAD ^ cmd ^ err_h ^ err_l))
                {
                    int16_t err;
                    err = (int16_t)((err_h << 8) | err_l);

                    Camera_ProcessFrame(cmd, err);
                }

                rx_state = 0;
                break;

            default:
                rx_state = 0;
                break;
        }

        LL_USART_EnableIT_RXNE(UART7);
    }
}
