//  CCx.cpp  Class to control the chipcon CCxxxx series transceivers
//  see http://focus.ti.com/lit/ds/symlink/cc1101.pdf for details on the CC1101

//  Copyright (c) 2010 Hans Klunder <hans.klunder (at) bigfoot.com>
//  Author: Hans Klunder, based on the original Rfbee v1.0 firmware by Seeedstudio
//  Version: May 22, 2010
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <SPI.h>
#include "RADIUINO4APP.h"

//---------- constructor ----------------------------------------------------

CC1101::CC1101(){}

// Power On Reset as described in  19.1.2 of cc1100 datasheet, tried APOR as described in 19.1.1 but that did not work :-(
void CC1101::PowerOnStartUp()
{
  DEBUGPRINT() 
  
  SPI.begin();
  SPI.setDataMode((1 << SPR1) | (1 << SPR0));//SPICLK=CPU/64

  // start manual Power On Reset
  digitalWrite(SS_PIN,HIGH);
  delayMicroseconds(1);

  digitalWrite(SS_PIN,LOW);  
  delayMicroseconds(10);
  
  digitalWrite(SS_PIN,HIGH);  
  delayMicroseconds(41);

  digitalWrite(SS_PIN,LOW);    
  
  // wait for MISO to go low
  while(digitalRead(MISO_PIN));
  
  SPI.transfer(CC1101_SRES);
 
  DEBUGPRINT("Waiting for CC1101 to complete POR") 

  // wait for MISO to go low
  while(digitalRead(MISO_PIN));
  
  digitalWrite(SS_PIN,HIGH);  
  
  DEBUGPRINT("CC1101 POR complete") 
}

byte CC1101::Read(byte addr,byte* data)
{
  DEBUGPRINT() 
  byte result;
 
  digitalWrite(SS_PIN,LOW);   
  //Spi.slaveSelect(LOW);
  // wait for MISO to go low
  while(digitalRead(MISO_PIN));
  
  result=SPI.transfer(addr | 0x80);
  *data=SPI.transfer(0);

  digitalWrite(SS_PIN,HIGH);  

  return result;
}

byte CC1101::ReadBurst(byte addr, byte* dataPtr, byte size)
{
  DEBUGPRINT() 
  byte result;
  
  digitalWrite(SS_PIN,LOW);  
 
  // wait for MISO to go low
  while(digitalRead(MISO_PIN));
  
  result=SPI.transfer(addr | 0xc0);

  while(size)
  {
    *dataPtr++ = SPI.transfer(0);
    size--;
  }
  
  digitalWrite(SS_PIN,HIGH);  
  
  return result;
}

byte CC1101::Write(byte addr, byte dat)
{
  DEBUGPRINT() 
  byte result;
  
  digitalWrite(SS_PIN,LOW);  

  // wait for MISO to go low
  while(digitalRead(MISO_PIN));
  
  result=SPI.transfer(addr);
  result=SPI.transfer(dat);

  digitalWrite(SS_PIN,HIGH); 
  
  return result;
}

byte CC1101::WriteBurst(byte addr, const byte* dataPtr, byte size)
{
  DEBUGPRINT() 
  byte result;
  
  digitalWrite(SS_PIN,LOW);  

  // wait for MISO to go low
  while(digitalRead(MISO_PIN));
  
  result=SPI.transfer(addr | 0x40);

  while(size)
  {
    result = SPI.transfer(*dataPtr++);
    size--;
  }
  
  digitalWrite(SS_PIN,HIGH);  
  
  return result;
}

byte CC1101::Strobe(byte addr)
{
  DEBUGPRINT() 
  byte result;

  digitalWrite(SS_PIN,LOW);  

  // wait for MISO to go low
  while(digitalRead(MISO_PIN));
  
  result=SPI.transfer(addr);
  
  digitalWrite(SS_PIN,HIGH);  
  
  return result;
}

//configure registers of cc1100 making it work in specific mode
void CC1101::Setup(byte configId)
{
  DEBUGPRINT() 
  byte reg;
  byte val;
  if (configId < CC1101_NR_OF_CONFIGS)
      for(byte i = 0; i< CC1101_NR_OF_REGISTERS; i++){
        reg=pgm_read_byte(&CC1101_registers[i]);
        val=pgm_read_byte(&CC1101_registerSettings[configId][i]);//read flash data no problem
        byte temp = Write(reg,val);
        //Serial.print(temp,HEX);
        //Serial.print(" ");
      }
}


// to aid debugging
//#ifdef DEBUG
void CC1101::ReadSetup()
{
  DEBUGPRINT()
  byte reg;
  byte value;
  for(byte i = 0; i< CC1101_NR_OF_REGISTERS; i++){
    reg=pgm_read_byte(&CC1101_registers[i]);
    Read(reg,&value);
    Serial.print(reg,HEX);
    Serial.print(':');
    Serial.println(value,HEX);
  }
}
//#endif

void CC1101::setPA(byte configId,byte paIndex)
{
  DEBUGPRINT()
  byte PAval=pgm_read_byte(&CC1101_paTable[configId][paIndex]);
  cc1101.Write(CC1101_PATABLE,PAval);
}

//---------- preinstantiate CCx object --------------------------------------

CC1101 cc1101 = CC1101();

