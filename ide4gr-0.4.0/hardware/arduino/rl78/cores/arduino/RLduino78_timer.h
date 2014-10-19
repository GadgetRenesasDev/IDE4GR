﻿/***************************************************************************
 *
 * PURPOSE
 *   RLduino78 framework clock / interval timer header file.
 *
 * TARGET DEVICE
 *   RL78/G13
 *
 * AUTHOR
 *   Renesas Solutions Corp.
 *
 * $Date:: 2013-03-18 09:25:41 +0900#$
 *
 ***************************************************************************
 * Copyright (C) 2012 Renesas Electronics. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * See file LICENSE.txt for further informations on licensing terms.
 ***************************************************************************/
/**
 * @file  RLduino78_timer.h
 * @brief RLduino78フレームワーク クロック/インターバル・タイマ・ヘッダ・ファイル
 */
#ifndef RLDUINO78_TIMER_H
#define RLDUINO78_TIMER_H
/***************************************************************************/
/*    Include MCU depend defines.                                          */
/***************************************************************************/


/***************************************************************************/
/*    Interrupt handler                                                    */
/***************************************************************************/


/***************************************************************************/
/*    Include Header Files                                                 */
/***************************************************************************/


/***************************************************************************/
/*    Macro Definitions                                                    */
/***************************************************************************/


/***************************************************************************/
/*    Type  Definitions                                                    */
/***************************************************************************/


/***************************************************************************/
/*    Function prototypes                                                  */
/***************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
void init_system_clock();
void setup_main_system_clock();
void setup_sub_system_clock();
void init_interval_timer();
void start_interval_timer();
void stop_interval_timer();
void start_micro_seconds_timer();
void stop_micro_seconds_timer();
#ifdef __cplusplus
};
#endif


/***************************************************************************/
/*    Global Variables                                                     */
/***************************************************************************/


/***************************************************************************/
/*    Local Variables                                                      */
/***************************************************************************/


/***************************************************************************/
/*    Global Routines                                                      */
/***************************************************************************/


/***************************************************************************/
/*    Local Routines                                                       */
/***************************************************************************/


/***************************************************************************/
/* End of module                                                           */
/***************************************************************************/
#endif /* RLDUINO78_TIMER_H */
