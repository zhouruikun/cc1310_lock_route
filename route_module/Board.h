/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
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

#ifndef __BOARD_H
#define __BOARD_H

#define Board_CC1310_LAUNCHXL

#ifdef __cplusplus
extern "C" {
#endif

#include "CC1310_LAUNCHXL.h"

#define Board_initGeneral()     CC1310_LAUNCHXL_initGeneral()
#define Board_shutDownExtFlash() CC1310_LAUNCHXL_shutDownExtFlash()
#define Board_wakeUpExtFlash() CC1310_LAUNCHXL_wakeUpExtFlash()

// user define
#define  Board_STATUS_PIN_LED0          IOID_9
#define  Board_OPEN_LOCK_IN_PIN         IOID_6
#define  Board_OPEN_LOCK_OUT_PIN        IOID_5
#define  Board_BAUD_PIN                 IOID_8
#define  Board_SET_PIN                  IOID_7
#define  Board_DE485_PIN                IOID_4
/* These #defines allow us to reuse TI-RTOS across other device families */

#define Board_NVSINTERNAL       CC1310_LAUNCHXL_NVSCC26XX0
#define Board_NVSEXTERNAL       CC1310_LAUNCHXL_NVSSPI25X0

//
#define Board_UART0             CC1310_LAUNCHXL_UART0
//
#define Board_WATCHDOG0         CC1310_LAUNCHXL_WATCHDOG0
//
///* Board specific I2C addresses */
//#define Board_TMP_ADDR          (0x40)
//#define Board_SENSORS_BP_TMP_ADDR Board_TMP_ADDR

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
