// Radiuino.h : cabeçalhos das bibliotecas

// Mais informações em www.radiuino.cc
// Copyright (c) 2011
// Author: Pedro Henrique Gomes e Omar C. Branquinho
// Versão 1.0: 12/09/2011

// Este arquivo é parte da plataforma Radiuino
// Este programa é um software livre; você pode redistribui-lo e/ou modifica-lo dentro dos termos da Licença Pública Geral Menor GNU 
// como publicada pela Fundação do Software Livre (FSF); na versão 2 da Licença, ou (na sua opnião) qualquer futura versão.
// Este programa é distribuido na esperança que possa ser  util, mas SEM NENHUMA GARANTIA; sem uma garantia implicita 
// de ADEQUAÇÂO a qualquer MERCADO ou APLICAÇÃO EM PARTICULAR. Veja a Licença Pública Geral Menor GNU para maiores detalhes.
// Você deve ter recebido uma cópia da Licença Pública Geral Menor GNU junto com este programa, se não, escreva para a Fundação 
// do Software Livre(FSF) Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
    
// This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version. This library 
// is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
// or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details. You should have received a copy 
// of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, 
// Fifth Floor, Boston, MA  02110-1301  USA

#ifndef RADIUINO_H
#define RADIUINO_H 1

#include <avr/pgmspace.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

// Pin definitions
#define GDO0       2 // Used for pooling the RF received data

/* *******************************************************************
	INCLUDES FROM CC1101.h
	**************************************************************** */

#ifndef _CC1101_H
#define _CC1101_H 1

// Flag definitions
#define OK 0
#define ERR -1 
#define NOTHING 1
#define MODIFIED 2

#define CC1101_PA_TABLESIZE 8

// CC2500/CC1100/CC1101 STROBE, CONTROL AND STATUS REGISTER
#define CC1101_IOCFG2       0x00        // GDO2 output pin configuration
#define CC1101_IOCFG1       0x01        // GDO1 output pin configuration
#define CC1101_IOCFG0		0x02        // GDO0 output pin configuration
#define CC1101_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1        0x04        // Sync word, high byte
#define CC1101_SYNC0        0x05        // Sync word, low byte
#define CC1101_PKTLEN       0x06        // Packet length
#define CC1101_PKTCTRL1     0x07        // Packet automation control
#define CC1101_PKTCTRL0     0x08        // Packet automation control
#define CC1101_ADDR         0x09        // Device address
#define CC1101_CHANNR       0x0A        // Channel number
#define CC1101_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC1101_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC1101_FREQ2        0x0D        // Frequency control word, high byte
#define CC1101_FREQ1        0x0E        // Frequency control word, middle byte
#define CC1101_FREQ0        0x0F        // Frequency control word, low byte
#define CC1101_MDMCFG4      0x10        // Modem configuration
#define CC1101_MDMCFG3      0x11        // Modem configuration
#define CC1101_MDMCFG2      0x12        // Modem configuration
#define CC1101_MDMCFG1      0x13        // Modem configuration
#define CC1101_MDMCFG0      0x14        // Modem configuration
#define CC1101_DEVIATN      0x15        // Modem deviation setting
#define CC1101_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CC1101_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CC1101_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CC1101_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CC1101_BSCFG        0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B        // AGC control
#define CC1101_AGCCTRL1     0x1C        // AGC control
#define CC1101_AGCCTRL0     0x1D        // AGC control
#define CC1101_WOREVT1      0x1E        // High byte Event 0 timeout
#define CC1101_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CC1101_WORCTRL      0x20        // Wake On Radio control
#define CC1101_FREND1       0x21        // Front end RX configuration
#define CC1101_FREND0       0x22        // Front end TX configuration
#define CC1101_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27        // RC oscillator configuration
#define CC1101_RCCTRL0      0x28        // RC oscillator configuration
#define CC1101_FSTEST       0x29        // Frequency synthesizer calibration control
#define CC1101_PTEST        0x2A        // Production test
#define CC1101_AGCTEST      0x2B        // AGC test
#define CC1101_TEST2        0x2C        // Various test settings
#define CC1101_TEST1        0x2D        // Various test settings
#define CC1101_TEST0        0x2E        // Various test settings

// Strobe commands
#define CC1101_SRES         0x30        // Reset chip.
#define CC1101_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CC1101_SXOFF        0x32        // Turn off crystal oscillator.
#define CC1101_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CC1101_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CC1101_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CC1101_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST      0x3C        // Reset real time clock.
#define CC1101_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // bytes for simpler software.
// Status registers (read & burst)
#define CC1101_PARTNUM      (0x30 | 0xc0)
#define CC1101_VERSION      (0x31 | 0xc0)
#define CC1101_FREQEST      (0x32 | 0xc0)
#define CC1101_LQI          (0x33 | 0xc0)
#define CC1101_RSSI         (0x34 | 0xc0)
#define CC1101_MARCSTATE    (0x35 | 0xc0)
#define CC1101_WORTIME1     (0x36 | 0xc0)
#define CC1101_WORTIME0     (0x37 | 0xc0)
#define CC1101_PKTSTATUS    (0x38 | 0xc0)
#define CC1101_VCO_VC_DAC   (0x39 | 0xc0)
#define CC1101_TXBYTES      (0x3A | 0xc0)
#define CC1101_RXBYTES      (0x3B | 0xc0)

#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F
#define CC1101_RXFIFO       0x3F

#define CC1101_FIFO_SIZE    0x40 // 64 bytes
#define BUFFLEN	CC1101_FIFO_SIZE

#define CC1101_PACKT_LEN   (CC1101_FIFO_SIZE - 3) // see section 15.3 of the datasheet

#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11
#define SS_PIN    10

class CC1101
{
  public:
    CC1101(void);
    byte Read(byte addr, byte* data);
    byte ReadBurst(byte addr, byte* dataPtr, byte dataCount);
    byte Write(byte addr, byte dat);
    byte WriteBurst(byte addr, const byte* dataPtr, byte dataCount);
    byte Strobe(byte addr);

    //power on reset as discribed in  27.1 of cc1100 datasheet
    void PowerOnStartUp();

    //configure registers of cc1100 making it work in specific mode
    void Setup(byte configId);
    void ReadSetup(void);
    // set power amplification using a table
    void setPA(byte configId, byte paIndex );

};

extern CC1101 cc1101;

#endif // _CC1101_H

/* *******************************************************************
	INCLUDES FROM CC1101Cfg.h
	**************************************************************** */

#ifndef _CC1101CFG_H
#define _CC1101CFG_H

#define CC1101_NR_OF_REGISTERS 35
// list the registers in the same order as CCxRegisterSettings
// stored in progmem to save on RAM
static const byte CC1101_registers[CC1101_NR_OF_REGISTERS] PROGMEM ={

    CC1101_IOCFG2    	,
    CC1101_IOCFG0    	,
    CC1101_FIFOTHR   	,
    CC1101_PKTLEN    	,
    CC1101_PKTCTRL1  	,
    CC1101_PKTCTRL0  	,
    CC1101_ADDR   		,
    CC1101_CHANNR    	,
    CC1101_FSCTRL1 		,
    CC1101_FSCTRL0		,
    CC1101_FREQ2    	,
    CC1101_FREQ1    	,
    CC1101_FREQ0    	,
    CC1101_MDMCFG4  	,
    CC1101_MDMCFG3   	,
    CC1101_MDMCFG2   	,
    CC1101_MDMCFG1   	,
    CC1101_MDMCFG0   	,
    CC1101_DEVIATN   	,
    CC1101_MCSM0    	,
    CC1101_FOCCFG    	,
    CC1101_BSCFG    	,
    CC1101_AGCCTRL2  	,
    CC1101_AGCCTRL1  	,
    CC1101_AGCCTRL0  	,
    CC1101_FREND1    	,
    CC1101_FREND0    	,
    CC1101_FSCAL3    	,
    CC1101_FSCAL2    	,
    CC1101_FSCAL1    	,
    CC1101_FSCAL0    	,
    CC1101_FSTEST    	,
    CC1101_TEST2    	,
    CC1101_TEST1    	,
    CC1101_TEST0    	,
};

#define CC1101_NR_OF_CONFIGS 1

// configuration for CC1101 generated by TI's SmartRf studio
// stored in progmem to save on RAM
extern const byte CC1101_registerSettings[CC1101_NR_OF_CONFIGS][CC1101_NR_OF_REGISTERS];

// PATABLE (dBm output power)

// stored in progmem to save on RAM
extern const byte CC1101_paTable[CC1101_NR_OF_CONFIGS][CC1101_PA_TABLESIZE];

#endif // _CC1101CFG_H

/* *******************************************************************
	INCLUDES FROM Debug.h
	**************************************************************** */

#ifndef _DEBUG_H
#define _DEBUG_H 1

#ifdef DEBUG
    #define DEBUGPRINT( X ) \
    Serial.print( __FUNCTION__ ); \
    Serial.print( ": " ); \
    Serial.println( X );
#else
    #define DEBUGPRINT( X )
#endif

#endif // _DEBUG_H

/* *******************************************************************
	INCLUDES FROM TimerOne.h
	**************************************************************** */

#ifndef _TIMERONE_H
#define _TIMERONE_H 1

#include <avr/io.h>
#include <avr/interrupt.h>

#define RESOLUTION 65536    // Timer1 is 16 bit

class TimerOne
{
  public:
  
    // properties
    unsigned int pwmPeriod;
    unsigned char clockSelectBits;

    // methods
    void initialize(long microseconds=1000000);
    void start();
    void stop();
    void restart();
	unsigned long read();
    void pwm(char pin, int duty, long microseconds=-1);
    void disablePwm(char pin);
    void attachInterrupt(void (*isr)(), long microseconds=-1);
    void detachInterrupt();
    void setPeriod(long microseconds);
    void setPwmDuty(char pin, int duty);
    void (*isrCallback)();
};

extern TimerOne Timer1;

#endif // _TIMERONE_H

#endif // _RADIUINO_H
