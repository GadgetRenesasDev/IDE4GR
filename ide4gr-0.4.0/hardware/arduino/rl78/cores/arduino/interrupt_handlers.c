﻿/***********************************************************************/
/*  													               */
/*      PROJECT NAME :  RLduino78                                      */
/*      FILE         :  interrupt_handlers.c                           */
/*      DESCRIPTION  :  Interrupt Handler                              */
/*      CPU SERIES   :  RL78 - G13                                     */
/*      CPU TYPE     :  R5F100LE                                       */
/*  													               */
/*      This file is generated by e2studio.                        */
/*  													               */
/***********************************************************************/                                                                       
                                                                                   
#include "interrupt_handlers.h"
#include "RLduino78_mcu_depend.h"

void servo_interrupt() __attribute__((weak));
void servo_interrupt() {}
void iica0_interrupt() __attribute__((weak));
void iica0_interrupt() {}


//0x4
void INT_WDTI (void) {
	}
//0x6
void INT_LVI (void) {
	}
//0x8
void INT_P0 (void) {
#if EXTERNAL_INTERRUPT_0 == 0
	external_interrupt_0();
#elif EXTERNAL_INTERRUPT_1 == 0
	external_interrupt_1();
#endif
	}
//0xA
void INT_P1 (void) {
#if EXTERNAL_INTERRUPT_0 == 1
	external_interrupt_0();
#elif EXTERNAL_INTERRUPT_1 == 1
	external_interrupt_1();
#endif
	}
//0xC
void INT_P2 (void) {
#if EXTERNAL_INTERRUPT_0 == 2
	external_interrupt_0();
#elif EXTERNAL_INTERRUPT_1 == 2
	external_interrupt_1();
#endif
	}
//0xE
void INT_P3 (void) {
#if EXTERNAL_INTERRUPT_0 == 3
	external_interrupt_0();
#elif EXTERNAL_INTERRUPT_1 == 3
	external_interrupt_1();
#endif
	}
//0x10
void INT_P4 (void) {
#if EXTERNAL_INTERRUPT_0 == 4
	external_interrupt_0();
#elif EXTERNAL_INTERRUPT_1 == 4
	external_interrupt_1();
#endif
	}
//0x12
void INT_P5 (void) {
#if EXTERNAL_INTERRUPT_0 == 5
	external_interrupt_0();
#elif EXTERNAL_INTERRUPT_1 == 5
	external_interrupt_1();
#endif
	}
//0x14
void INT_ST2 (void) {
#if UART_CHANNEL == 2
	uart_tx_interrupt();
#endif
#if UART2_CHANNEL == 2
	uart2_tx_interrupt();
#endif
	}
//0x16
void INT_SR2 (void) {
#if UART_CHANNEL == 2
	uart_rx_interrupt();
#endif
#if UART2_CHANNEL == 2
	uart2_rx_interrupt();
#endif
	}
//0x18
void INT_SRE2 (void) {
#if UART_CHANNEL == 2
	uart_error_interrupt();
#endif
#if UART2_CHANNEL == 2
	uart2_error_interrupt();
#endif
	}
//0x1A
void INT_DMA0 (void) {
	}
//0x1C
void INT_DMA1 (void) {
	}
//0x1E
void INT_ST0 (void) {
#if UART_CHANNEL == 0
	uart_tx_interrupt();
#endif
    }
//0x20
void INT_SR0 (void) {
#if UART_CHANNEL == 0
	uart_rx_interrupt();
#endif
	}
//0x22
void INT_TM01H (void) {
#if UART_CHANNEL == 0
	uart_error_interrupt();
#endif
	}
//0x24
void INT_ST1 (void) {
#if UART1_CHANNEL == 1
	uart1_tx_interrupt();
#endif
	}
//0x26
void INT_SR1 (void) {
#if UART1_CHANNEL == 1
	uart1_rx_interrupt();
#endif
	}
//0x28
void INT_TM03H (void) {
#if UART1_CHANNEL == 1
	uart1_error_interrupt();
#endif
	}
//0x2A
void INT_IICA0 (void) {
	iica0_interrupt();
	}
//0x2C
void INT_TM00 (void) {
	}
//0x2E
void INT_TM01 (void) {
#if SERVO_CHANNEL == 1
	servo_interrupt();
#endif
#if TONE_TIMER == 1
	tone_interrupt();
#endif
	}
//0x30
void INT_TM02 (void) {
#if TONE_TIMER == 2
	tone_interrupt();
#endif
	}
//0x32
void INT_TM03 (void) {
#if TONE_TIMER == 3
	tone_interrupt();
#endif
	}
//0x34
void INT_AD (void) {
	adc_interrupt();
	}
//0x36
void INT_RTC (void) {
	rtc_interrupt();
	}
//0x38
void INT_IT (void) {
	interval_timer();
	}
//0x3A
void INT_KR (void) {
	}
//0x42
void INT_TM04 (void) {
#if SERVO_CHANNEL == 4
	servo_interrupt();
#endif
#if TONE_TIMER == 4
	tone_interrupt();
#endif
	}
//0x44
void INT_TM05 (void) {
#if SERVO_CHANNEL == 5
	servo_interrupt();
#endif
#if TONE_TIMER == 5
	tone_interrupt();
#endif
	tm05_interrupt();
	}
//0x46
void INT_TM06 (void) {
#if TONE_TIMER == 6
	tone_interrupt();
#endif
#if defined(REL_GR_KURUMI)
	tm06_interrupt();
#endif
	}
//0x48
void INT_TM07 (void) {
#if TONE_TIMER == 7
	tone_interrupt();
#endif
	}
//0x4A
void INT_P6 (void) {
	}
//0x4C
void INT_P7 (void) {
	}
//0x4E
void INT_P8 (void) {
	}
//0x50
void INT_P9 (void) {
	}
//0x52
void INT_P10 (void) {
	}
//0x54
void INT_P11 (void) {
	}
//0x5E
void INT_MD (void) {
	}
//0x62
void INT_FL (void) {
	}
//0x7E
void INT_BRK_I (void) {
	}
