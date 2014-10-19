/***************************************************************************
 *
 * PURPOSE
 *   RLduino78 framework interrupt header file.
 *
 * TARGET DEVICE
 *   RL78/G13
 *
 * AUTHOR
 *   Renesas Solutions Corp.
 *
 * $Date:: 2012-12-18 17:02:26 +0900#$
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
 * @file  pgmspace.h
 * @brief RLduino78フレームワーク PGMSPACEヘッダ・ファイル
 */
#ifndef PGMSPACE_H
#define PGMSPACE_H
/***************************************************************************/
/*    Include Header Files                                                 */
/***************************************************************************/


/***************************************************************************/
/*    Macro Definitions                                                    */
/***************************************************************************/
#ifndef PGM_P
#define PGM_P const prog_char *
#endif

#define PROGMEM

#define pgm_read_byte(address_short)    (*address_short)

#define PSTR(s) ((const char *)(s))


/***************************************************************************/
/*    Type  Definitions                                                    */
/***************************************************************************/
typedef char prog_char;


/***************************************************************************/
/*    Function prototypes                                                  */
/***************************************************************************/


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
#endif /* PGMSPACE_H */
