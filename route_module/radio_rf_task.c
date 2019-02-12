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
/* Wake-on-Radio configuration */
#define WOR_WAKEUPS_PER_SECOND_TX 2
#define NUMMSGS         5
/* TX number of random payload bytes */

/* WOR Example configuration defines */
#define WOR_PREAMBLE_TIME_RAT_TICKS(x) \
    ((uint32_t)(4000000*(1.0f/(x))))
/* TI-RTOS Task configuration */
#define RX_TASK_STACK_SIZE 1024
#define RX_TASK_PRIORITY   2

/* TX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             PAYLOAD_LENGTH /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     1  /* Length byte included in the stored packet */

/***** Type declarations *****/
/* General wake-on-radio RX statistics */
struct WorStatistics {
  uint32_t doneIdle;
  uint32_t doneIdleTimeout;
  uint32_t doneRxTimeout;
  uint32_t doneOk;
};
/***** Variable declarations *****/
/* TX task objects and task stack */
static Task_Params rxTaskParams;
static Task_Struct rxTask;
static uint8_t rxTaskStack[RX_TASK_STACK_SIZE];
/* RF driver object and handle */
static RF_Object rfObject;
RF_Handle rfHandle;
extern Mailbox_Handle mbx_rf_Handle;
/* General wake-on-radio sniff status statistics and statistics from the RF Core about received packets */
static volatile struct WorStatistics worStatistics;


/* Advanced TX command for sending long preamble */
rfc_CMD_PROP_TX_ADV_t RF_cmdPropTxAdv;

/* TX packet payload (length +1 to fit length byte) and sequence number */
uint8_t packet[PAYLOAD_LENGTH +1];

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */


/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxDataEntryBuffer, 4);
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#endif

/* RX Data Queue and Data Entry pointer to read out received packets */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;

/* Received packet's length and pointer to the payload */
static uint8_t packetLength;
static uint8_t* packetDataPointer;
static volatile uint8_t dummy;
Queue_Handle send_rf_queue;
/* Sniff command for doing combined Carrier Sense and RX*/
rfc_CMD_ABORT_t RF_cmdABORT;
/***** Prototypes *****/
static void rxTaskFunction(UArg arg0, UArg arg1);
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

typedef struct MsgObjp {
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

MailboxMsgObj mailboxBuffer[NUMMSGS];

Mailbox_Struct mbx_radio_Struct;
Mailbox_Handle mbx_radio_Handle;

/* Copy the basic RX configuration from CMD_PROP_RX to CMD_PROP_RX_SNIFF command. */
static void initializeTxAdvCmdFromTxCmd(rfc_CMD_PROP_TX_ADV_t* RF_cmdPropTxAdv, rfc_CMD_PROP_TX_t* RF_cmdPropTx)
{
    #define RADIO_OP_HEADER_SIZE 14

    /* Copy general radio operation header from TX commmand to TX_ADV */
    memcpy(RF_cmdPropTxAdv, RF_cmdPropTx, RADIO_OP_HEADER_SIZE);

    /* Set command to CMD_PROP_TX_ADV */
    RF_cmdPropTxAdv->commandNo = CMD_PROP_TX_ADV;

    /* Copy over relevant parameters */
    RF_cmdPropTxAdv->pktConf.bFsOff = RF_cmdPropTx->pktConf.bFsOff;
    RF_cmdPropTxAdv->pktConf.bUseCrc = RF_cmdPropTx->pktConf.bUseCrc;
    RF_cmdPropTxAdv->syncWord = RF_cmdPropTx->syncWord;
}
/***** Function definitions *****/
/* RX task initialization function. Runs once from main() */
void rxTaskInit()
{
    /* Initialize the display and try to open both UART and LCD types of display. */
    /* Initialize the radio */
    RF_Params rfParams;
    RF_Params_init(&rfParams);
    Mailbox_Params mbxParams;
    /* Construct a Mailbox instance */
        Mailbox_Params_init(&mbxParams);
        mbxParams.buf = (Ptr)mailboxBuffer;
        mbxParams.bufSize = sizeof(mailboxBuffer);
        Mailbox_construct(&mbx_radio_Struct, sizeof(MsgObj), NUMMSGS, &mbxParams, NULL);
        mbx_radio_Handle = Mailbox_handle(&mbx_radio_Struct);

    /* Initialize TX_ADV command from TX command */
    initializeTxAdvCmdFromTxCmd(&RF_cmdPropTxAdv, &RF_cmdPropTx);

    /* Set application specific fields */
    RF_cmdPropTxAdv.pktLen = PAYLOAD_LENGTH +1; /* +1 for length byte */
    RF_cmdPropTxAdv.pPkt = packet;
    RF_cmdPropTxAdv.preTrigger.triggerType = TRIG_REL_START;
    RF_cmdPropTxAdv.preTime = WOR_PREAMBLE_TIME_RAT_TICKS(WOR_WAKEUPS_PER_SECOND_TX);
    send_rf_queue = Queue_create(NULL, NULL);
    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RX_TASK_STACK_SIZE;
    rxTaskParams.priority = RX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    Task_construct(&rxTask, rxTaskFunction, &rxTaskParams, NULL);


}
#define EasyLink_CmdHandle_isValid(handle) (handle >= 0)
#define EASYLINK_RF_CMD_HANDLE_INVALID -1
//Handle for last Async command, which is needed by EasyLink_abort
static RF_CmdHandle asyncCmdHndl = EASYLINK_RF_CMD_HANDLE_INVALID;
uint8_t   rec_flag=0;



void send_to_rf(uint8_t *data,uint8_t len)
{
    //force abort (gracefull param set to 0)
    MsgObj msg;
    msg.packetDataPointer = get_malloc();//malloc(1);
    msg.packetLength=len;
    memcpy( msg.packetDataPointer,data,msg.packetLength);
    if(!Mailbox_post(mbx_radio_Handle, &msg, BIOS_NO_WAIT))
    {
        while(1);
    }
    if(rec_flag==1)
    {
        RF_cancelCmd(rfHandle, asyncCmdHndl, 1);
    }
}

/* RX task function. Executed in Task context by TI-RTOS when the scheduler starts. */
static void rxTaskFunction(UArg arg0, UArg arg1)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Route out LNA active pin to LED1 */

    /* Create queue and data entries */
    if (RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }


    RF_cmdABORT.commandNo=CMD_ABORT;


    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    /* Set frequency */
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, &callback, 0);

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;
    /* Enter main loop */
    while(1)
    {
        /* Enter RX mode and stay forever in RX */
        rec_flag=1;
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                                   RF_PriorityNormal, &callback,
                                                   RF_EventRxEntryDone);
        rec_flag=0;
        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;

        MsgObj msg;
         while(Mailbox_pend(mbx_radio_Handle, &msg, 10))
                {
                    memcpy(&packet,msg.packetDataPointer,msg.packetLength);
                    RF_cmdPropTxAdv.pktLen = packet[0] +1; /* +1 for length byte */
                    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityNormal, NULL, 0);
                    free_malloc(msg.packetDataPointer);
                }



        switch(cmdStatus)
        {
            case PROP_DONE_IDLE:
                /* Idle based on RSSI */
                worStatistics.doneIdle++;
                break;
            case PROP_DONE_IDLETIMEOUT:
                /* Idle based on PQT */
                worStatistics.doneIdleTimeout++;
                break;
            case PROP_DONE_RXTIMEOUT:
                /* Got valid preamble on the air, but did not find sync word */
                worStatistics.doneRxTimeout++;
                break;
            case PROP_DONE_OK:
                /* Received packet */
                worStatistics.doneOk++;
                break;
            default:
                /* Unhandled status */
                break;
        };
    }
}


/* Called for every received packet and command done */
void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    MsgObj msg;
    /* If we've received a new packet and it's available to read out */
    if (e & RF_EventRxEntryDone)
    {
        do
        {
            /* Get current unhandled data entry */
            currentDataEntry = RFQueue_getDataEntry();

            /* Handle the packet data, located at &currentDataEntry->data:
             * - Length is the first byte with the current configuration
             * - Data starts from the second byte */
            packetLength      = *(uint8_t*)(&currentDataEntry->data);
            packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

            /* This code block is added to avoid a compiler warning.
            * Normally, an application will reference these variables for
            * useful data. */
            dummy = packetLength + packetDataPointer[0];
            msg.packetDataPointer = get_malloc();
            msg.packetLength=packetLength;
            memcpy( msg.packetDataPointer,packetDataPointer,msg.packetLength);
            Mailbox_post(mbx_rf_Handle, &msg, BIOS_NO_WAIT);
        } while(RFQueue_nextEntry() == DATA_ENTRY_FINISHED);
    }
}

