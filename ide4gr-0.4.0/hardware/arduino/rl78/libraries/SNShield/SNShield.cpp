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

#include <arduino.h>
#include <snshield.h>
#include <spi.h>
#include <MMA8452Q.h>

smartanalog_t gp_smartanalog_data[] =
{
/*    { 0x0b, 0x0d },
    { 0x12, 0x02 },
    { 0x00, 0x22 },  //CONFIG1 SW1n, SW2n
    { 0x01, 0x06 },  //CONFIG2 SW3n, SW0n
    { 0x03, 0x88 },  //MPX1
    { 0x04, 0x00 },  //MPX2
    { 0x05, 0x21 },
    { 0x06, 0x01 },  //Gain Conf1 10.9db@1 3.5 times
    { 0x07, 0x0c },  //Gain Conf2 30.3db@7 32.7 times
    { 0x08, 0x00 },  //Gain Conf3 6db@0h 36db@fh
    { 0x09, 0x00 },  //AOMC
    { 0x0a, 0x00 },
    { 0x0c, 0x08 },  //DACRC
    { 0x0d, 0x00 },  //DAC1C
    { 0x0e, 0x00 },  //DAC2C
    { 0x0f, 0x00 },  //DAC3C
    { 0x10, 0x00 },  //DAC4C
    { 0x11, 0xbb },  //PC1: On/Off for DAC, AMP
    { 0x12, 0x02 },  //
    { 0xff, 0x00 }
*/	
    { 0x0b, 0x0d },
    { 0x12, 0x02 },
    { 0x00, 0x00 },  //CONFIG1 SW1n, SW2n
    { 0x01, 0x00 },  //CONFIG2 SW3n, SW0n
    { 0x03, 0xff },  //MPX1
    { 0x04, 0x00 },  //MPX2
    { 0x05, 0x21 },
    { 0x06, 0x01 },  //Gain Conf1 10.9db@1 3.5 times
    { 0x07, 0x0c },  //Gain Conf2 30.3db@7 32.7 times
    { 0x08, 0x00 },  //Gain Conf3 6db@0h 36db@fh
    { 0x09, 0x00 },  //AOMC
    { 0x0a, 0x00 },
    { 0x0c, 0x08 },  //DACRC
    { 0x0d, 0x00 },  //DAC1C
    { 0x0e, 0x00 },  //DAC2C
    { 0x0f, 0x00 },  //DAC3C
    { 0x10, 0x00 },  //DAC4C
    { 0x11, 0x08 },  //PC1: On/Off for DAC, AMP
    { 0x12, 0x02 },  //
    { 0xff, 0x00 }
};

void R_SAIC_Write( smartanalog_t *p_saic_data) 
{
    unsigned long txd, rxd;
    unsigned char uc, dat, adrs;

    // SPI初期化
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2 );
    SPI.setDataMode(SPI_MODE3 );    // 立ち下がりサンプリング

    // SAICへのレジスタ設定
    while (p_saic_data->address != 0xff)
    {
        pinMode( SAIC_CS, OUTPUT );   
        digitalWrite( SAIC_CS, LOW );

        delay(1);

        adrs = (p_saic_data->address & 0x7f) | 0x80;   /* 0x80 is data write mode */
        SPI.transfer(adrs);               /* send SAIC Address data  */

        dat = p_saic_data->data;
        SPI.transfer(dat);               /* send SAIC Address data  */

        delay(1);
        digitalWrite( SAIC_CS, HIGH );

        delay(1);
        p_saic_data++;
    }
}
int R_SAIC_Create( void ) 
{
    // reset for SAIC
    pinMode( SAIC_RST, OUTPUT );   
    digitalWrite( SAIC_RST, 0 );
    delay( 1 );
    digitalWrite( SAIC_RST, 1 );
    delay( 1 );

    R_SAIC_Write( gp_smartanalog_data);

    return 0;    
}



void SNShield::begin(){
    R_SAIC_Create();
    if (!initMMA8452()){
        Serial.println("fail to initialize MMA8452Q");
    }
//    pinMode(XBEE_SLEEPRQ, OUTPUT);
//    digitalWrite(XBEE_SLEEPRQ, HIGH);
}
    
float SNShield::getTemp(){
    float t=0;
    for(int i = 0; i< 5;i++){
        t = t + (analogRead(A0) * TEMP_COEFFICIENT - 500)/10;
    }
    t = t / 5;
    return t;
}
float SNShield::getLux(){
    float t=0;
    for(int i = 0; i< 5;i++){
       t = t + analogRead(A1) * LUX_COEFFICIENT ;
    }
    t = t / 5;
    return t;
}

void SNShield::getAll(){
    float acc[3];
    this->temp = getTemp();
    this->lux = getLux();
    readAccelData(acc);
    this->accx = acc[0];
    this->accy = acc[1];
    this->accz = acc[2];
}

float SNShield::getAccx(){
    float acc[3];
    readAccelData(acc);
    this->accx = acc[0];
    return acc[0];
}
float SNShield::getAccy(){
    float acc[3];
    readAccelData(acc);
    this->accy = acc[1];
    return acc[1];
}
float SNShield::getAccz(){
    float acc[3];
    readAccelData(acc);
    this->accz = acc[2];
    return acc[2];
}

