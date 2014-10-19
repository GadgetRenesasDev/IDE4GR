/*
  SNShield.cpp - Sensor Network Shield library
  Copyright (c) 2014 Yuuki Okamiya.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 18 March 2014 by yuuki.okamiya.yn@renesas.com
*/

#ifndef SNShield_h
#define SNShield_h

#include <Arduino.h>
#include "utility/MMA8452Q.h"
#include "utility/XBee.h"

int R_SAIC_Create( void );
#define SAIC_RST 2
#define SAIC_CS 4
#define XBEE_SLEEPRQ 6

#define EXT_VOLTAGE 3.3
#define TEMP_AMPLIFIER 3.5
#define TEMP_COEFFICIENT (EXT_VOLTAGE/1024*1000/TEMP_AMPLIFIER)
#define LUX_AMPLIFIER 32.7
#define LUX_COEFFICIENT (EXT_VOLTAGE/1024*1000/LUX_AMPLIFIER)

typedef struct
{
    uint8_t address;
    uint8_t data;
} smartanalog_t ;

class SNShield
{
    private:
    public:
        float temp;
        float lux;
        float accx;
        float accy;
        float accz;
        void begin(void);
        void getAll();
        float getTemp();
        float getLux();
        float getAccx();
        float getAccy();
        float getAccz();
};


#endif //SNShield_h
