/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 * Modified 28 February 2013 by masahiko.nagata.cj@renesas.com
 */

#include "pins_arduino.h"
#include "SPI.h"

SPIClass SPI;

void SPIClass::begin() {
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  pinMode(MOSI, OUTPUT);
  pinMode(SS, OUTPUT);

  digitalWrite(SCK, HIGH);
  digitalWrite(MOSI, HIGH);
  digitalWrite(SS, HIGH);

  if (SPI_SAUxEN == 0) {
#ifdef WORKAROUND_READ_MODIFY_WRITE
    SBI2(SFR2_PER0, SFR2_BIT_SAUxEN);  // クロック供給開始
#else
    SPI_SAUxEN = 1;                    // クロック供給開始
#endif
    NOP();
    NOP();
    NOP();
    NOP();
    SPI_SPSx = 0x0001;                 // 動作クロック設定
  }

#ifdef WORKAROUND_READ_MODIFY_WRITE
  SPI_STx      |= SPI_CHx;             // シリアル通信停止
  SBI(SFR_MKxx,  SFR_BIT_CSIMKxx);     // 割り込み処理禁止
  CBI(SFR_IFxx,  SFR_BIT_CSIIFxx);     // 割り込み要求フラグをクリア
  CBI(SFR_PR1xx, SFR_BIT_CSIPR1xx);    // 割り込み優先順位の設定
  CBI(SFR_PR0xx, SFR_BIT_CSIPR0xx);
#else
  SPI_STx      |= SPI_CHx;             // シリアル通信停止
  SPI_CSIMKxx   = 1;                   // 割り込み処理禁止
  SPI_CSIIFxx   = 0;                   // 割り込み要求フラグをクリア
  SPI_CSIPR1xx  = 0;                   // 割り込み優先順位の設定
  SPI_CSIPR0xx  = 0;
#endif
  SPI_SIRxx     = 0x0007;              // エラーフラグをクリア
  SPI_SMRxx     = 0x0020;              // モード設定
  SPI_SCRxx     = 0xF007;              // シリアル通信動作設定
  SPI_SDRxx     = SPI_CLOCK_DIV4 << 9; // 動作クロックの分周設定

  start();
}

void SPIClass::end() {
  stop();

#ifdef WORKAROUND_READ_MODIFY_WRITE
    CBI2(SFR2_PER0, SFR2_BIT_SAUxEN);  // クロック供給停止
#else
    SPI_SAUxEN = 0;                    // クロック供給停止
#endif
}

void SPIClass::setBitOrder(uint8_t bitOrder)
{
  stop();

  if(bitOrder == LSBFIRST) {
    SPI_SCRxx |= (1 << 7);
  } else {
    SPI_SCRxx &= ~(1 << 7);
  }

  start();
}

void SPIClass::setDataMode(uint8_t mode)
{
  stop();

  SPI_SCRxx = (SPI_SCRxx & ~SPI_MODE_MASK) | ((uint16_t)mode << 8);

  start();
}

void SPIClass::setClockDivider(uint8_t rate)
{
  stop();

  SPI_SDRxx = rate << 9;

  start();
}

void SPIClass::start()
{
#ifdef WORKAROUND_READ_MODIFY_WRITE
  CBI(SFR_IFxx,   SFR_BIT_CSIIFxx); // 割り込み要求フラグをクリア
  SPI_SOx     |= SPI_CHx << 8; // シリアル出力バッファ設定
  SPI_SOx     &= ~SPI_CHx;
  SPI_SOEx    |= SPI_CHx;      // シリアル出力許可
  SPI_SSx     |= SPI_CHx;      // シリアル通信開始
#else
  SPI_CSIIFxx  = 0;            // 割り込み要求フラグをクリア
  SPI_SOx     |= SPI_CHx << 8; // シリアル出力バッファ設定
  SPI_SOx     &= ~SPI_CHx;
  SPI_SOEx    |= SPI_CHx;      // シリアル出力許可
  SPI_SSx     |= SPI_CHx;      // シリアル通信開始
#endif
}

void SPIClass::stop()
{
  SPI_STx  |= SPI_CHx;         // シリアル通信停止
  SPI_SOEx &= ~SPI_CHx;        // シリアル出力停止
}
