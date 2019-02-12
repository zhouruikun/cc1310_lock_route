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
#include <ti/drivers/uart.h>
#include <ti/display/Display.h>
#include <ti/drivers/Watchdog.h>
/* Board Header files */
#include "Board.h"
#include <ti/devices/cc13x0/inc/hw_fcfg1.h>
/* RF settings */
#include "smartrf_settings/smartrf_settings.h"
#include <common_route.h>
/***** Defines *****/


/* TX task stack size and priority */
#define AUX_TASK_STACK_SIZE 512
#define AUX_TASK_PRIORITY   2


/***** Variable declarations *****/
/* TX task objects and task stack */
static Task_Params auxTaskParams;
Task_Struct auxTask;    /* not static so you can see in ROV */
static uint8_t auxTaskStack[AUX_TASK_STACK_SIZE];

uint64_t macAddress=0;
/* RF driver objects and handles */
static RF_Object rfObject;
extern RF_Handle rfHandle;

/* Pin driver objects and handles */
PIN_Handle ledPinHandle;
static PIN_Handle buttonPinHandle;
static PIN_State ledPinState;
static PIN_State buttonPinState;
Watchdog_Handle watchdogHandle;
/* TX Semaphore */
static Semaphore_Struct auxSemaphore;
Semaphore_Handle auxSemaphoreHandle;
static Semaphore_Struct RFSemaphore;
Semaphore_Handle RFSemaphoreHandle;


/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
 Board_STATUS_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_OPEN_LOCK_OUT_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_DE485_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_SET_PIN | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BAUD_PIN | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_DIS,
    Board_OPEN_LOCK_IN_PIN | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,
    PIN_TERMINATE
};


/***** Function definitions *****/

static void auxTaskFunction(UArg arg0, UArg arg1);
extern void rxTaskInit();
/* Pin interrupt Callback function board buttons configured in the pinTable. */
void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId) {
    uint8_t key_press_cnt=0;
    /* Simple debounce logic, only toggle if the button is still pushed (low) */

    CPUdelay((uint32_t)((48000000/4)*0.050f));
     if(pinId==Board_SET_PIN) {
        if(!PIN_getInputValue(pinId))
        {
            while(!PIN_getInputValue(pinId))
                {
                    CPUdelay((uint32_t)((48000000/4)*0.050f));
                    key_press_cnt++;
                }
            if(key_press_cnt>120){
                //长按6秒
                resetKeyPressed();
            }
            else if(key_press_cnt>60)
            {
                //长按3秒
                pairKeyPressed();
            }
        }
    }
     if(pinId==Board_OPEN_LOCK_IN_PIN)  {
         if (PIN_getInputValue(pinId))
            {

                openDoorKeyPressed();
            }
    }
}

/* TX task initialization function. Runs once from main() */
void auxTaskInit()
{
    /* Initialize TX semaphore */
    Semaphore_construct(&auxSemaphore, 0, NULL);
    auxSemaphoreHandle = Semaphore_handle(&auxSemaphore);

    /* Initialize and create TX task */
    Task_Params_init(&auxTaskParams);
    auxTaskParams.stackSize = AUX_TASK_STACK_SIZE;
    auxTaskParams.priority = AUX_TASK_PRIORITY;
    auxTaskParams.stack = &auxTaskStack;
    Task_construct(&auxTask, auxTaskFunction, &auxTaskParams, NULL);
}

/* TX task function. Executed in Task context by TI-RTOS when the scheduler starts. */
static void auxTaskFunction(UArg arg0, UArg arg1)
{
    static int_fast32_t led_ligth=0;
    static int_fast32_t pair_wait=0;

    /* Enter main TX loop */
    while(1)
    {
        /* Wait for a button press */
        Semaphore_pend(auxSemaphoreHandle, BIOS_WAIT_FOREVER);
        switch(send_msg){
        case SEND_OPEN_DOOR:
                        send_msg = 0;
                        sendCrypedPack(0,0x01);//发送函数只能在发送线程里  否则会死机
                        break;
        case SEND_PAIR:
            pair_wait=0;led_ligth=0;
                      while(1)
                      {
                          if(get_wir_status() == WIR_STATUS_NOMAL)
                          { //pair success
                              set_wir_status(WIR_STATUS_NOMAL);
                              break;
                          }
                          Task_sleep((uint32_t)5000);//50ms
                          led_ligth ++ ;
                          if(led_ligth>10)
                          {
                              led_ligth=0;
                              PIN_setOutputValue(ledPinHandle, Board_STATUS_PIN_LED0, !PIN_getOutputValue(Board_STATUS_PIN_LED0));
                              pair_wait++;

                              if(pair_wait>60)
                              {
                                  //pair fail
                                  set_wir_status(WIR_STATUS_NOMAL);
                                    break;
                              }
                          }

                      }
                      checkPair();
            break;
        default :break;
        }

    }
}


/*
 *  ======== watchdogCallback ========
 *  Watchdog interrupt callback function.
 */
void watchdogCallback(uintptr_t unused)
{
    /* Clear watchdog interrupt flag */
    Watchdog_clear(watchdogHandle);

    /* Insert timeout handling code here. */
}

/*
 *  ======== main ========
 */
int main(void)
{    Watchdog_Params params;
    /* Call driver init functions. */
    Board_initGeneral();
    Display_init();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    Assert_isTrue(ledPinHandle != NULL, NULL);

    /* Open Button pins */
    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFunction) != 0) {
        /* Error registering button callback function */
        while(1);
    }

    Watchdog_init();
    /* Create and enable a Watchdog with resets disabled */
    Watchdog_Params_init(&params);
    params.callbackFxn = (Watchdog_Callback)watchdogCallback;
    params.resetMode = Watchdog_RESET_ON;
    watchdogHandle = Watchdog_open(Board_WATCHDOG0, &params);
    if (watchdogHandle == NULL) {
        /* Error opening Watchdog */
        while (1);
    }

    readLockData();
    Assert_isTrue(buttonPinHandle != NULL, NULL);
    macAddress = *((uint64_t *)(FCFG1_BASE + FCFG1_O_MAC_15_4_0));
    setLogicMac( macAddress);
    /* Initialize task */
    auxTaskInit();
    protocol_rf_TaskInit();
    protocol_uart_TaskInit();
    /* Initialize task */
    rxTaskInit();
    uartTaskInit();
    /* Start BIOS */
    BIOS_start();

    return (0);
}
