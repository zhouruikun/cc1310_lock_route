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
#include <common_route.h>
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)


#define MAX_UART 100

#define UART_TASK_STACK_SIZE 1024
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

static void uartTaskFunction(UArg arg0, UArg arg1)
{
    static int_fast32_t rec_cnt=0;
    /* Call driver init functions */
    MsgObj msg;
    UART_init();


    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = (1+PIN_getInputValue(Board_BAUD_PIN))*4800;
    uartParams.readTimeout = (1+PIN_getInputValue(Board_BAUD_PIN))*1000;
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    UART_write(uart, echoPrompt, sizeof(echoPrompt));
    while(1)
       {

        //串口读取数据
          rec_cnt =  UART_read(uart, input, 1);
          if(rec_cnt!=0)
          {
              rec_cnt = UART_read(uart, input+1, MAX_UART);
              rec_cnt+=1;

              msg.packetDataPointer = get_malloc();
              msg.packetLength=rec_cnt;

              memcpy( msg.packetDataPointer,input, msg.packetLength);
              Mailbox_post(mbx_uart_Handle, &msg, BIOS_NO_WAIT);

              }

       }
}
void uartTaskInit()
{
    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = UART_TASK_STACK_SIZE;
    uartTaskParams.priority = UART_TASK_PRIORITY;
    uartTaskParams.stack = &uartTaskStack;
    Task_construct(&uartTask, uartTaskFunction, &uartTaskParams, NULL);
}
void uart_485_send(const void *buffer, size_t size)
{
    PIN_setOutputValue(ledPinHandle, Board_DE485_PIN, 1);
    // 任务休眠 1 秒，  1000000us, 下面函数的单位是10us
     Task_sleep(1000);
    UART_write(uart,buffer, size);
    Task_sleep(1000);
    PIN_setOutputValue(ledPinHandle, Board_DE485_PIN, 0);
}
