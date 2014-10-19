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

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <stdio.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <RLduino78_mcu_depend.h>

#define SPI_CLOCK_DIV4   0x01
#define SPI_CLOCK_DIV16  0x07
#define SPI_CLOCK_DIV64  0x1F
#define SPI_CLOCK_DIV128 0x3F
#define SPI_CLOCK_DIV2   0x00
#define SPI_CLOCK_DIV8   0x03
#define SPI_CLOCK_DIV32  0x0F

#define SPI_MODE0 0x30
#define SPI_MODE1 0x10
#define SPI_MODE2 0x20
#define SPI_MODE3 0x00

#define SPI_MODE_MASK 0x3000  // DAP = bit 13, CKP = bit 12 on SCR

class SPIClass {
public:
  inline static byte transfer(byte _data);

  // SPI Configuration methods

  inline static void attachInterrupt();
  inline static void detachInterrupt(); // Default

  static void begin(); // Default
  static void end();

  static void setBitOrder(uint8_t);
  static void setDataMode(uint8_t);
  static void setClockDivider(uint8_t);
private:
  static void start();
  static void stop();
};

extern SPIClass SPI;

byte SPIClass::transfer(byte _data) {
  SPI_SIOxx = _data;
  while (!SPI_CSIIFxx);
#ifdef WORKAROUND_READ_MODIFY_WRITE
  CBI(SFR_IFxx, SFR_BIT_CSIIFxx);
#else
  SPI_CSIIFxx = 0;
#endif
  return SPI_SIOxx;
}

void SPIClass::attachInterrupt() {
#ifdef WORKAROUND_READ_MODIFY_WRITE
  CBI(SFR_IFxx, SFR_BIT_CSIIFxx);
  CBI(SFR_MKxx, SFR_BIT_CSIMKxx);
#else
  SPI_CSIIFxx = 0;
  SPI_CSIMKxx = 0;
#endif
}

void SPIClass::detachInterrupt() {
#ifdef WORKAROUND_READ_MODIFY_WRITE
  CBI(SFR_IFxx, SFR_BIT_CSIIFxx);
  SBI(SFR_MKxx, SFR_BIT_CSIMKxx);
#else
  SPI_CSIIFxx = 0;
  SPI_CSIMKxx = 1;
#endif
}

#endif
