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
#include <ti/sysbios/knl/Semaphore.h>
/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>
/* Board Header files */
#include "Board.h"
#include <common_node.h>
/* Application Header files */ 
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

#include <ti/devices/DeviceFamily.h>
#include <ti/devices/cc13x0/inc/hw_fcfg1.h>

#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/***** Defines *****/
/* Wake-on-Radio wakeups per second */




/* TI-RTOS Task configuration */
#define RX_TASK_STACK_SIZE 1024
#define RX_TASK_PRIORITY   2

/* TX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             128 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     1  /* Length byte included in the stored packet */




/***** Variable declarations *****/
/* TX task objects and task stack */

static Task_Struct rxTask;
static Task_Struct uartTask;



/* RF driver object and handle */
static RF_Object rfObject;
RF_Handle rfHandle;
extern UART_Handle uart;
extern UART_Params uartParams;
/* Pin driver object and handle */
PIN_Handle outPinHandle;
PIN_State outPinState;
PIN_Handle inPinHandle;
PIN_State inPinState;


/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config outPinTable[] =
{
     Board_PIN_D_CE | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_OPENDRAIN | PIN_DRVSTR_MAX,
     IOID_12 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config inPinTable[] = {
     Board_PIN_D_CE | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
     IOID_12 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};



static volatile uint8_t dummy;
uint64_t macAddress=0;

/* Sniff command for doing combined Carrier Sense and RX*/



extern Semaphore_Handle txSemaphoreHandle;
extern void uartTaskInit();
extern void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId);
/***** Function definitions *****/

void set_pin_mode(uint8_t mode){


    PIN_close(outPinHandle);
    if(mode == PIN_IN  )
    {
        outPinHandle = PIN_open(&outPinState, inPinTable);
        if (PIN_registerIntCb(outPinHandle, &buttonCallbackFunction) != 0) {
            /* Error registering button callback function */
            while(1);
        }
    }
    else if(mode == PIN_OUT  )
    {
        outPinHandle = PIN_open(&outPinState, outPinTable);
    }
}

void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId) {
    int_fast32_t temp=0;
    /* Simple debounce logic, only toggle if the button is still pushed (low) */

//    if (!PIN_getInputValue(pinId)) {
    if ( (pinId)==Board_PIN_D_CE) {
        while(!PIN_getInputValue(pinId))
        {
            CPUdelay((uint32_t)((48000000/4)*0.010f));
            temp++;
            if(temp>10)
            {
                PIN_setOutputValue(outPinHandle, IOID_12, 0);
//                usleep(10000);
                CPUdelay((uint32_t)((48000000/4)*0.110f));
                PIN_setOutputValue(outPinHandle, IOID_12, 1);
                /* Post TX semaphore to TX task */
                Semaphore_post(txSemaphoreHandle);
                break;
            }
        }

    }
}
/*
 *  ======== main ========
 */

int main(void)
    {

    /* Call driver init functions. */
    Board_initGeneral();
    outPinHandle = PIN_open(&outPinState, inPinTable);
    if (PIN_registerIntCb(outPinHandle, &buttonCallbackFunction) != 0) {
        /* Error registering button callback function */
        while(1);
    }

    read_rolling_code();
     macAddress = *((uint64_t *)(FCFG1_BASE + FCFG1_O_MAC_15_4_0));
     setLogicMac( macAddress);

     protocol_rf_TaskInit();
     protocol_uart_TaskInit();

    /* Initialize task */
     rxTaskInit();
      uartTaskInit();
    /* Start BIOS */
    BIOS_start();


    return (0);
}
