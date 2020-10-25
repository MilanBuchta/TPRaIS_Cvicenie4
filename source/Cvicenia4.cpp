/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Cvicenia4.cpp
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"

#include "fsl_pit.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

enum States {
	READING_VALUE,
	START_TIMER
};

#define TIME_OF_READ 78U //104*0.75

volatile States currentState = START_TIMER;
volatile int bitIndex = 0;
volatile int inByte = 0;
volatile bool newByte = false;

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

extern "C" void PIT_IRQHandler() {
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	PIT_StopTimer(PIT, kPIT_Chnl_0);
	if(currentState == READING_VALUE) {
		auto bit = GPIO_ReadPinInput(BOARD_INITPINS_RX_PIN_GPIO,BOARD_INITPINS_RX_PIN_PIN);
		inByte |= (bit << bitIndex);
		bitIndex++;
	}
    currentState=START_TIMER;
}

extern "C" void PORTA_IRQHandler() {
	PORT_ClearPinsInterruptFlags(BOARD_INITPINS_RX_PIN_PORT, BOARD_INITPINS_RX_PIN_PIN_MASK);

	switch(currentState) {
		case START_TIMER:
			currentState = READING_VALUE;
			PIT_StartTimer(PIT, kPIT_Chnl_0);
		case READING_VALUE:
			__asm volatile ("nop");
		break;
	}

	if(bitIndex == 8) {
		newByte = true;
	}
}

int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

	/* Structure of initialize PIT */
	pit_config_t pitConfig;

	PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);
	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0,
			USEC_TO_COUNT(TIME_OF_READ, CLOCK_GetBusClkFreq()));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ (PIT_IRQn);
    EnableIRQ(PORTA_IRQn);
    PRINTF("Hello World\n");

    //PORT_SetPinInterruptConfig(BOARD_INITPINS_RX_PIN_PORT, BOARD_INITPINS_RX_PIN_PIN, kPORT_InterruptEitherEdge);

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        while(!newByte) {
        	__asm volatile ("nop");
        }

        uint8_t tmpByte = inByte;
        newByte = false;
        bitIndex = 0;
        inByte = 0;
        PRINTF("TEST %x \n", tmpByte);
    }
    return 0 ;
}
