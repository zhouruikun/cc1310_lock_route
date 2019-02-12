/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>
/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

#include <ti/devices/DeviceFamily.h>
#include <common_node.h>
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)


#define MAX_UART 100

#define UART_TASK_STACK_SIZE 2048
#define UART_TASK_PRIORITY   2
static uint8_t uartTaskStack[UART_TASK_STACK_SIZE];
Task_Struct uartTask;    /* not static so you can see in ROV */
extern PIN_Handle ledPinHandle;
uint8_t        input[MAX_UART];
const char  echoPrompt[] = "Echoing characters:\r\n";
UART_Handle uart;
UART_Params uartParams;
static Task_Params uartTaskParams;
extern Mailbox_Handle mbx_uart_Handle;
typedef struct MsgObj {
     uint8_t packetLength;
     uint8_t* packetDataPointer;
} MsgObj;
PIN_Handle uartpinHandle;
PIN_State uartpinState;
const PIN_Config uartpinsleep[] = {
                                 /* UART RX via debugger back channel */
    CC1310_LAUNCHXL_UART_TX | PIN_INPUT_EN  | PIN_PULLDOWN,                        /* UART TX via debugger back channel */
    PIN_TERMINATE
};
const PIN_Config uartpinwork[] = {
    CC1310_LAUNCHXL_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                              /* UART RX via debugger back channel */
    CC1310_LAUNCHXL_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,                        /* UART TX via debugger back channel */
    PIN_TERMINATE
};
/* TX Semaphore */
static Semaphore_Struct txSemaphore;
Semaphore_Handle txSemaphoreHandle;
extern float WOR_WAKEUPS_PER_SECOND_rx;
static void uartTaskFunction(UArg arg0, UArg arg1);
void uartTaskInit()
{
    /* Initialize TX semaphore */


    Semaphore_construct(&txSemaphore, 0, NULL);
    txSemaphore.f2 = ti_sysbios_knl_Semaphore_Mode_BINARY;
    txSemaphoreHandle = Semaphore_handle(&txSemaphore);

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = UART_TASK_STACK_SIZE;
    uartTaskParams.priority = UART_TASK_PRIORITY;
    uartTaskParams.stack = &uartTaskStack;
    Task_construct(&uartTask, uartTaskFunction, &uartTaskParams, NULL);
}
static void uartTaskFunction(UArg arg0, UArg arg1)
{
    static int_fast32_t rec_cnt=0;
    MsgObj msg;
    /* Call driver init functions */
    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uartParams.readTimeout = 1000;
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
    while(1)
       {
        if(!DEBUG)
        {
            /* Wait for a button press */
            UART_close(uart);
            //set tx as input with pull up
            uartpinHandle= PIN_open(&uartpinState, uartpinsleep);
            Semaphore_pend(txSemaphoreHandle, BIOS_WAIT_FOREVER);
            PIN_close(uartpinHandle);
            uart = UART_open(Board_UART0, &uartParams);
            WOR_WAKEUPS_PER_SECOND_rx=WAKE_TIMES;
            set_sleep_flag(STATUS_WORKING);
        }
        while(1)
        {
            //指令判断是否休眠
            if (!DEBUG&&(get_sleep_flag()== STATUS_SLEEPING))
            {
                Task_sleep(100);
                Semaphore_reset(txSemaphoreHandle,0);
                WOR_WAKEUPS_PER_SECOND_rx=SLEEP_TIMES;
                break;
            }
            //串口读取数据
              rec_cnt =  UART_read(uart, input, 1);
              if(rec_cnt>0)
              {
                  rec_cnt = UART_read(uart, input+1, MAX_UART);
                  rec_cnt+=1;


                  msg.packetDataPointer = get_malloc();
                  msg.packetLength=rec_cnt;

                  memcpy( msg.packetDataPointer,input, msg.packetLength);
                  Mailbox_post(mbx_uart_Handle, &msg, BIOS_NO_WAIT);
//                  lockToWir(input,rec_cnt);//调用锁端到射频协议处理代码
              }
        }

       }
}

//void uart_485_send(const void *buffer, size_t size)
//{
//    PIN_setOutputValue(ledPinHandle, Board_DE485_PIN, 1);
//    // 任务休眠 1 秒，  1000000us, 下面函数的单位是10us
//     Task_sleep(1000);
//    UART_write(uart,buffer, size);
//    Task_sleep(1000);
//    PIN_setOutputValue(ledPinHandle, Board_DE485_PIN, 0);
//}
