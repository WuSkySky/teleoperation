//
// Created by Mechrev on 2026/4/9.
//

#include "usb_serial_task.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "INS_task.h"

static uint8_t send_buf[64];

void usb_serial_task(void *argument)
{
    osDelay(200);
    MX_USB_DEVICE_Init();

    while(1)
    {
        send_buf[0] = 0x55;
        send_buf[1] = 0xAA;
        memcpy(&send_buf[2], INS_quat, sizeof(INS_quat));
        send_buf[62] = 0x0D;
        send_buf[63] = 0x0A;
        CDC_Transmit_FS(send_buf, sizeof(send_buf));
        osDelay(1000);
    }
}
