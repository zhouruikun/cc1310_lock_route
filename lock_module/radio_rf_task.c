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
#include "application_settings.h"
#include <ti/devices/DeviceFamily.h>
#include <common_node.h>
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
/* Wake-on-Radio configuration */
#define WOR_WAKEUPS_PER_SECOND_TX 2
/* Wake-on-Radio mode. Can be:
 * - RSSI only
 * - PQT, preamble detection
 * - Both, first RSSI and then PQT if RSSI  */
#define WOR_MODE CarrierSenseMode_RSSIandPQT

/* Threshold for RSSI based Carrier Sense in dBm */
#define WOR_RSSI_THRESHOLD      ((int8_t)(-111))

/* Data Rate in use */
#define WOR_RF_PHY_DATARATE_50KBPS  0 // 2-GFSK 50Kbps

#define WOR_RF_PHY_DATARATE WOR_RF_PHY_DATARATE_50KBPS

/* Macro used to set actual wakeup interval */
#define WOR_WAKE_UP_MARGIN_S 0.005f
#define WOR_WAKE_UP_INTERVAL_RAT_TICKS(x) \
    ((uint32_t)(4000000*(1.0f/(x) - (WOR_WAKE_UP_MARGIN_S))))
/* TX number of random payload bytes */

/* WOR Example configuration defines */
#define WOR_PREAMBLE_TIME_RAT_TICKS(x) \
    ((uint32_t)(4000000*(1.0f/(x))))
/* TI-RTOS Task configuration */
#define RX_TASK_STACK_SIZE 1024
#define RX_TASK_PRIORITY   5
/* Number of times the CS command should run when the channel is BUSY */
#define CS_RETRIES_WHEN_BUSY    2000
/* The channel is reported BUSY is the RSSI is above this threshold */
#define RSSI_THRESHOLD_DBM      -80
#define IDLE_TIME_US            6000
/* TX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             PAYLOAD_LENGTH /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     1  /* Length byte included in the stored packet */
float WOR_WAKEUPS_PER_SECOND_rx=SLEEP_TIMES;
/***** Type declarations *****/
/***** Type declarations *****/
/* General wake-on-radio RX statistics */
struct WorStatistics {
  uint32_t doneIdle;
  uint32_t doneIdleTimeout;
  uint32_t doneRxTimeout;
  uint32_t doneOk;
};

/* Modes of carrier sense possible */
enum CarrierSenseMode {
    CarrierSenseMode_RSSI,
    CarrierSenseMode_PQT,
    CarrierSenseMode_RSSIandPQT,
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
static rfc_CMD_PROP_RX_SNIFF_t RF_cmdPropRxSniff;
/* General wake-on-radio sniff status statistics and statistics from the RF Core about received packets */
static volatile struct WorStatistics worStatistics;
static rfc_propRxOutput_t rxStatistics;
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
/* Sniff command for doing combined Carrier Sense and RX*/
rfc_CMD_ABORT_t RF_cmdABORT;
/***** Prototypes *****/
static void rxTaskFunction(UArg arg0, UArg arg1);
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static uint32_t calculateSymbolRate(uint8_t prescaler, uint32_t rateWord);
/* Copies all RX options from the SmartRF Studio exported RX command to the RX Sniff command */
static void initializeSniffCmdFromRxCmd(rfc_CMD_PROP_RX_SNIFF_t* rxSniffCmd, rfc_CMD_PROP_RX_t* rxCmd);
/* Configures Sniff-mode part of the RX_SNIFF command based on mode, datarate and wakeup interval */
static void configureSniffCmd(rfc_CMD_PROP_RX_SNIFF_t* rxSniffCmd, enum CarrierSenseMode mode, uint32_t datarate, uint8_t wakeupPerSecond);


#define NUMMSGS         5

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
/* This buffer is not directly accessed by the application */

MailboxMsgObj mailboxBuffer[NUMMSGS];

Mailbox_Struct mbx_radio_Struct;
Mailbox_Handle mbx_radio_Handle;

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
//    msg.packetDataPointer = get_malloc();//malloc(100);
    msg.packetDataPointer = malloc(len);
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

    /* Copy all RX options from the SmartRF Studio exported RX command to the RX Sniff command */
    initializeSniffCmdFromRxCmd(&RF_cmdPropRxSniff, &RF_cmdPropRx);

    /* Configure RX part of RX_SNIFF command */
    RF_cmdPropRxSniff.pQueue    = &dataQueue;
    RF_cmdPropRxSniff.pOutput   = (uint8_t*)&rxStatistics;
    RF_cmdPropRxSniff.maxPktLen = MAX_LENGTH;

    /* Discard ignored packets and CRC errors from Rx queue */
    RF_cmdPropRxSniff.rxConf.bAutoFlushIgnored = 1;
    RF_cmdPropRxSniff.rxConf.bAutoFlushCrcErr  = 1;

    /* Calculate datarate from prescaler and rate word */
    uint32_t datarate = calculateSymbolRate(RF_cmdPropRadioDivSetup.symbolRate.preScale,
                                          RF_cmdPropRadioDivSetup.symbolRate.rateWord);

    /* Configure Sniff-mode part of the RX_SNIFF command */
    configureSniffCmd(&RF_cmdPropRxSniff, WOR_MODE, datarate, WOR_WAKEUPS_PER_SECOND_rx);

    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

        RF_cmdNop.startTrigger.triggerType = TRIG_ABSTIME;
        RF_cmdNop.startTrigger.pastTrig = 1;

        /* Set up the next pointers for the command chain */
        RF_cmdNop.pNextOp = (rfc_radioOp_t*)&RF_cmdPropCs;
        RF_cmdPropCs.pNextOp = (rfc_radioOp_t*)&RF_cmdCountBranch;
        RF_cmdCountBranch.pNextOp = (rfc_radioOp_t*)&RF_cmdPropTx;
        RF_cmdCountBranch.pNextOpIfOk = (rfc_radioOp_t*)&RF_cmdPropCs;

        /* Customize the API commands with application specific defines */
        RF_cmdPropCs.rssiThr = RSSI_THRESHOLD_DBM;
        RF_cmdPropCs.csEndTime = (IDLE_TIME_US + 150) * 4; /* Add some margin */
        RF_cmdCountBranch.counter = CS_RETRIES_WHEN_BUSY;
    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    /* Set frequency */
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, &callback, 0);

    /* Save the current radio time */
    RF_cmdPropRxSniff.startTime = RF_getCurrentTime();



    /* Enter main loop */
    while(1)
    {
        /* Set next wakeup time in the future */
        RF_cmdPropRxSniff.startTime += WOR_WAKE_UP_INTERVAL_RAT_TICKS(WOR_WAKEUPS_PER_SECOND_rx);
        /* Schedule RX */
        rec_flag=1;
        RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRxSniff, RF_PriorityNormal, &callback, RF_EventRxEntryDone);
        rec_flag=0;
        MsgObj msg;
         while(Mailbox_pend(mbx_radio_Handle, &msg, 10))
                {
                    memcpy(&packet,msg.packetDataPointer+1,msg.packetLength-1);
                    RF_cmdPropTx.pktLen = msg.packetLength-1;
                    /* Set absolute TX time to utilize automatic power management */
                    RF_cmdNop.startTime = RF_getCurrentTime();
                    /* Send packet */
                    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdNop, RF_PriorityNormal,
                              &callback, 0);
                    RF_cmdNop.status = IDLE;
                    RF_cmdPropCs.status = IDLE;
                    RF_cmdCountBranch.status = IDLE;
                    RF_cmdPropTx.status = IDLE;
                    RF_cmdCountBranch.counter = CS_RETRIES_WHEN_BUSY;
//                    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
                    //free_malloc(msg.packetDataPointer);
                    free(msg.packetDataPointer);
                }

        /* Log RX_SNIFF status */
        switch(RF_cmdPropRxSniff.status) {
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

/* Calculates datarate from prescaler and rate word */
static uint32_t calculateSymbolRate(uint8_t prescaler, uint32_t rateWord)
{
    /* Calculate datarate according to TRM Section 23.7.5.2:
     * f_baudrate = (R * f_ref)/(p * 2^20)
     *   - R = rateWord
     *   - f_ref = 24Mhz
     *   - p = prescaler */
    uint64_t numerator = rateWord*24000000ULL;
    uint64_t denominator = prescaler*1048576ULL;
    uint32_t result = (uint32_t)(numerator/denominator);
    return result;
}

/* Copies all RX options from the SmartRF Studio exported RX command to the RX Sniff command */
static void initializeSniffCmdFromRxCmd(rfc_CMD_PROP_RX_SNIFF_t* rxSniffCmd, rfc_CMD_PROP_RX_t* rxCmd)
{

    /* Copy RX configuration from RX command */
    memcpy(rxSniffCmd, rxCmd, sizeof(rfc_CMD_PROP_RX_t));

    /* Change to RX_SNIFF command from RX command */
    rxSniffCmd->commandNo = CMD_PROP_RX_SNIFF;
}

/* Configures Sniff-mode part of the RX_SNIFF command based on mode, datarate and wakeup interval */
static void configureSniffCmd(rfc_CMD_PROP_RX_SNIFF_t* rxSniffCmd, enum CarrierSenseMode mode, uint32_t datarate, uint8_t wakeupPerSecond)
{
    /* Enable or disable RSSI */
    if ((mode == CarrierSenseMode_RSSI) || (mode == CarrierSenseMode_RSSIandPQT)) {
        rxSniffCmd->csConf.bEnaRssi        = 1;
    } else {
        rxSniffCmd->csConf.bEnaRssi        = 0;
    }

    /* Enable or disable PQT */
    if ((mode == CarrierSenseMode_PQT) || (mode == CarrierSenseMode_RSSIandPQT)) {
        rxSniffCmd->csConf.bEnaCorr        = 1;
        rxSniffCmd->csEndTrigger.triggerType  = TRIG_REL_START;
    } else {
        rxSniffCmd->csConf.bEnaCorr        = 0;
        rxSniffCmd->csEndTrigger.triggerType  = TRIG_NEVER;
    }

    /* General Carrier Sense configuration */
    rxSniffCmd->csConf.operation       = 1; /* Report Idle if RSSI reports Idle to quickly exit if not above
                                                 RSSI threshold */
    rxSniffCmd->csConf.busyOp          = 0; /* End carrier sense on channel Busy (the receiver will continue when
                                                 carrier sense ends, but it will then not end if channel goes Idle) */
    rxSniffCmd->csConf.idleOp          = 1; /* End on channel Idle */
    rxSniffCmd->csConf.timeoutRes      = 1; /* If the channel is invalid, it will return PROP_DONE_IDLE_TIMEOUT */

    /* RSSI configuration */
    rxSniffCmd->numRssiIdle            = 1; /* One idle RSSI samples signals that the channel is idle */
    rxSniffCmd->numRssiBusy            = 1; /* One busy RSSI samples signals that the channel is busy */
    rxSniffCmd->rssiThr    = (int8_t)WOR_RSSI_THRESHOLD; /* Set the RSSI threshold in dBm */

    /* PQT configuration */
    rxSniffCmd->corrConfig.numCorrBusy = 1;   /* One busy PQT samples signals that the channel is busy */
    rxSniffCmd->corrConfig.numCorrInv  = 1;   /* One busy PQT samples signals that the channel is busy */

    /* Calculate basic timing parameters */
    uint32_t symbolLengthUs  = 1000000UL/datarate;
    uint32_t preambleSymbols = (1000000UL/wakeupPerSecond)/symbolLengthUs;
    uint8_t syncWordSymbols  = RF_cmdPropRadioDivSetup.formatConf.nSwBits;

    /* Calculate sniff mode parameters */
    #define US_TO_RAT_TICKS 4
    #define CORR_PERIOD_SYM_MARGIN 16
    #define RX_END_TIME_SYM_MARGIN 8
    #define CS_END_TIME_MIN_TIME_SYM 30
    #define CS_END_TIME_MIN_TIME_STATIC_US 150


    /* Represents the time in which we need to receive corrConfig.numCorr* correlation peaks to detect preamble.
     * When continously checking the preamble quality, this period has to be wide enough to also contain the sync
     * word, with a margin. If it is not, then there is a chance the SNIFF command will abort while receiving the
     * sync word, as it no longer detects a preamble. */
    uint32_t correlationPeriodUs = (syncWordSymbols + CORR_PERIOD_SYM_MARGIN)*symbolLengthUs;

    /* Represents the time where we will force a check if preamble is present (only done once).
     * The main idea is that his should be shorter than "correlationPeriodUs" so that if we get RSSI valid, but
     * there is not a valid preamble on the air, we will leave RX as quickly as possible. */
    uint32_t csEndTimeUs = (CS_END_TIME_MIN_TIME_SYM*symbolLengthUs + CS_END_TIME_MIN_TIME_STATIC_US);

    /* Represents the maximum time from the startTrigger to when we expect a sync word to be received. */
    uint32_t rxEndTimeUs = (preambleSymbols + syncWordSymbols + RX_END_TIME_SYM_MARGIN)*symbolLengthUs;

    /* Set sniff mode timing configuration in sniff command in RAT ticks */
    rxSniffCmd->corrPeriod = (uint16_t)(correlationPeriodUs * US_TO_RAT_TICKS);
    rxSniffCmd->csEndTime  = (uint32_t)(csEndTimeUs * US_TO_RAT_TICKS);
    rxSniffCmd->endTime    = (uint32_t)(rxEndTimeUs * US_TO_RAT_TICKS);

    /* Set correct trigger types */
    rxSniffCmd->endTrigger.triggerType   = TRIG_REL_START;
    rxSniffCmd->startTrigger.triggerType = TRIG_ABSTIME;
    rxSniffCmd->startTrigger.pastTrig    = 1;
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
            if(! Mailbox_post(mbx_rf_Handle, &msg, 10000))
            {
                while(1);
            }
        } while(RFQueue_nextEntry() == DATA_ENTRY_FINISHED);
    }
}

