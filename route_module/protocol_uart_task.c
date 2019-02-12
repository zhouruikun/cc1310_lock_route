/*
 * protocol_task.c
 *
 *  Created on: 2019年1月29日
 *      Author: Administrator
 */
/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>
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
/* TI-RTOS Task configuration */
#define PROTOCOL_TASK_STACK_SIZE 512
#define PROTOCOL_TASK_PRIORITY   2
/***** Variable declarations *****/
/* TX task objects and task stack */
static Task_Params protocol_uart_TaskParams;
static Task_Struct protocol_uart_Task;
/* protocol task function. Executed in Task context by TI-RTOS when the scheduler starts. */
static void protocol_uart_TaskFunction(UArg arg0, UArg arg1);
static uint8_t protocol_uart_TaskStack[PROTOCOL_TASK_STACK_SIZE];

static Semaphore_Struct protocol_uart_Semaphore;
Semaphore_Handle protocol_uart_SemaphoreHandle;



#define NUMMSGS         5

/*
 * This type is accessed by the application. When changing the data members of
 * this structure, considerations should be made for padding and data alignment.
 */
typedef struct MsgObj {
     uint8_t packetLength;
     uint8_t* packetDataPointer;
} MsgObj;

/*
 * Mailbox messages are stored in a queue that requires a header in front of
 * each message. Mailbox_MbxElem is defined such that the header and its size
 * are factored into the total data size requirement for a mailbox instance.
 * Because Mailbox_MbxElem contains Int data types, padding may be added to
 * this struct depending on the data members defined in MsgObj.
 */
typedef struct MailboxMsgObj {
    Mailbox_MbxElem  elem;      /* Mailbox header        */
    MsgObj           obj;       /* Application's mailbox */
} MailboxMsgObj;

/* This buffer is not directly accessed by the application */
MailboxMsgObj mailboxBuffer[NUMMSGS];

Mailbox_Struct mbx_uart_Struct;
Mailbox_Handle mbx_uart_Handle;
MsgObj msg;

void protocol_uart_TaskInit()
{    Mailbox_Params mbxParams;
/* Construct a Mailbox instance */
    Mailbox_Params_init(&mbxParams);
    mbxParams.buf = (Ptr)mailboxBuffer;
    mbxParams.bufSize = sizeof(mailboxBuffer);
    /* Initialize TX semaphore */
    Semaphore_construct(&protocol_uart_Semaphore, 0, NULL);
    protocol_uart_SemaphoreHandle = Semaphore_handle(&protocol_uart_Semaphore);
    Mailbox_construct(&mbx_uart_Struct, sizeof(MsgObj), NUMMSGS, &mbxParams, NULL);
    mbx_uart_Handle = Mailbox_handle(&mbx_uart_Struct);
    Task_Params_init(&protocol_uart_TaskParams);
    protocol_uart_TaskParams.stackSize = PROTOCOL_TASK_STACK_SIZE;
    protocol_uart_TaskParams.priority = PROTOCOL_TASK_PRIORITY;
    protocol_uart_TaskParams.stack = &protocol_uart_TaskStack;
    Task_construct(&protocol_uart_Task, protocol_uart_TaskFunction, &protocol_uart_TaskParams, NULL);
}
/* protocol task function. Executed in Task context by TI-RTOS when the scheduler starts. */
static void protocol_uart_TaskFunction(UArg arg0, UArg arg1)
{
    while(1)
    {
        Mailbox_pend(mbx_uart_Handle, &msg, BIOS_WAIT_FOREVER);
        wirToLock(msg.packetDataPointer, msg.packetLength);//调用锁端处理到射频代码
        free_malloc(msg.packetDataPointer);
    }


}
