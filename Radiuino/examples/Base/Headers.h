// Header.h : cabecalhos de classes

// Mais informacoes acesse www.radiuino.cc
// Copyright (c) 2015
// Author: Pedro Henrique Gomes, Omar C. Branquinho, Tiago T. Ganselli, Debora M. Ferreira, Guilherme Lopes da Silva, Raphael Montali de Assumpçao.
// Versao 2.3: 18/05/2017

// Este arquivo e parte da plataforma Radiuino
// Este programa e um software livre; voce pode redistribui-lo e/ou modifica-lo dentro dos termos da Licenca Publica Geral Menor GNU 
// como publicada pela Fundacao do Software Livre (FSF); na versao 2 da Licenca, ou (na sua opniao) qualquer futura versao.
// Este programa e distribuido na esperanca que possa ser  util, mas SEM NENHUMA GARANTIA; sem uma garantia implicita 
// de ADEQUACAO a qualquer MERCADO ou APLICACAO EM PARTICULAR. Veja a Licenca Publica Geral Menor GNU para maiores detalhes.
// Voce deve ter recebido uma copia da Licenca Publica Geral Menor GNU junto com este programa, se nao, escreva para a Fundacao 
// do Software Livre(FSF) Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
    
// This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version. This library 
// is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
// or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details. You should have received a copy 
// of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, 
// Fifth Floor, Boston, MA  02110-1301  USA

#ifndef HEADERS_H
#define HEADERS_H 1

/**
 * Estrutura do pacote a ser transmitido e recebido. O pacote possui 52 bytes, sendo 4 bytes de cabecalho de camada Fisica (Phy), 4 bytes de cabecalho
 * de camada de Controle de Acesso ao Meio (MAC), 4 bytes de cabecalho de camada de Rede (Net), 4 bytes de cabecalho de camada de Transporte (Transp) e
 * e 4 bytes de cabecalho de camada de Aplicação (App). Os 36 bytes restantes sao reservados para payload de AD e IO.
 */
typedef struct 
{  
  /* cabecalho da camada Fisica (Phy) */
  byte PhyHdr[4];
  
  /* cabecalho da camada de Controle de Acesso ao Meio (MAC) */
  byte MACHdr[4];
  
  /* cabecalho da camada de Rede (Net) */
  byte NetHdr[4];
  
  /* cabecalho da camada de Transporte (Transp) */
  byte TranspHdr[4];

  /* Bytes o payload de AD */
  byte AD0[3];
  byte AD1[3];
  byte AD2[3];
  byte AD3[3];
  byte AD4[3];
  byte AD5[3];  
  
  /* Bytes do payload de IO */
  byte IO0[3];
  byte IO1[3];
  byte IO2[3];
  byte IO3[3];
  byte IO4[3];
  byte IO5[3];
    
} packet;

packet g_pkt;

/**
 * Classe de camada de Rede
 */
class NET
{
  public:
    NET(void);
    inline void initialize(void);
    inline void send(packet * pkt);
    inline void receive(packet * pkt);

    byte my_addr;
  private:
    
};

/* Objeto de acesso a classe da camada de Rede */
extern NET Net;

/**
 * Classe de camada de Controle de Acesso ao Meio
 */
class MAC
{
  public:
    MAC(void);
    inline void initialize(void);
    inline void send(packet * pkt);
    inline void receive(packet * pkt);

  private:
    
};

/* Objeto de acesso a classe da camada de Controle de Acesso ao Meio */
extern MAC Mac;

/**
 * Classe de camada Fisica
 */
#define BUFFLEN  CC1101_PACKT_LEN
byte serialData[BUFFLEN + 1];

// para fazer led piscar
//#ifndef _PHY_H
//#define _PHY_H 1
#define SET_RX_LED(state)	if(RxLedBlink) digitalWrite(6, state);
#define SET_TX_LED(state)	if(TxLedBlink) digitalWrite(4, state);

class PHY
{
  public:
    PHY(void);
    inline void initialize();
    inline void send(packet * pkt);
    inline int receive(packet * pkt);
    
    void sendSerial(packet * pkt);
    void receiveSerial(void);
    
    byte txFifoFree(void);
    void setChannel(byte channel);
    void setPower(byte power);
    void setFreqOffset(byte freq_offset);
    void setRadio(int set_radio);
    
    boolean carrierSense(void);
    
    byte power;                 /* Potencia */
    byte channel;               /* Canal */
    byte freq_offset;           /* Offset de frequencia */
    int serial_baudrate;        /* Serial baudrate */
    int set_radio;        /* Ajusta o tipo de rádio */
	
	// adiciona led piscar  
    bool RxLedBlink;
    bool TxLedBlink;
  
  private:
    int initCC1101Config(void);
    void configWatchdog(int time);
    void strobe_idle_wait(void);
};

/* Objeto de acesso a classe da camada Fisica */
extern PHY Phy;

/* Configuracao de registradores do CC1101. Obtidos atraves do SmartRF Studio 7 */
// Address Config = No address check 
// Base Frequency = 915.000000 
// CRC Autoflush = false 
// CRC Enable = true 
// Carrier Frequency = 915.000000 
// Channel Number = 0 
// Channel Spacing = 124.969482 
// Data Format = Normal mode 
// Data Rate = 38.3835 
// Deviation = 41.259766 
// Device Address = 0 
// Manchester Enable = false 
// Modulated = true 
// Modulation Format = 2-FSK 
// PA Ramping = false 
// Packet Length = 52 
// Packet Length Mode = Fixed packet length mode. Length configured in PKTLEN register 
// Preamble Count = 4 
// RX Filter BW = 162.500000 
// Sync Word Qualifier Mode = 30/32 sync word bits detected 
// TX Power = 0 
// Whitening = false

 
const byte CC1101_registerSettings[CC1101_NR_OF_CONFIGS][CC1101_NR_OF_REGISTERS] PROGMEM = {
{ 
  0x04,  // IOCFG2  GDO2 Output Pin Configuration
  0x07,  // IOCFG0  GDO0 Output Pin Configuration
  0x47,  // FIFOTHR  RX FIFO and TX FIFO Thresholds
  0x34,  // PKTLEN  Packet Length
  0x04,  // PKTCTRL1  Packet Automation Control
  0x04,  // PKTCTRL0  Packet Automation Control
  0x00,  // ADDR  Device Address
  0x00,  // CHANNR  Channel Number
  0x06,  // FSCTRL1  Frequency Synthesizer Control
  0x00,  // FSCTRL0  Frequency Synthesizer Control
  0x23,  // FREQ2  Frequency Control Word, High Byte
  0x31,  // FREQ1  Frequency Control Word, Middle Byte
  0x3B,  // FREQ0  Frequency Control Word, Low Byte
  0x9A,  // MDMCFG4  Modem Configuration
  0x83,  // MDMCFG3  Modem Configuration
  0x03,  // MDMCFG2  Modem Configuration
  0xA2,  // MDMCFG1  Modem Configuration
  0x3B,  // MDMCFG0  Modem Configuration
  0x45,  // DEVIATN  Modem Deviation Setting
  0x18,  // MCSM0  Main Radio Control State Machine Configuration
  0x16,  // FOCCFG  Frequency Offset Compensation Configuration
  0x6C,  // BSCFG  Bit Synchronization Configuration
  0x43,  // AGCCTRL2  AGC Control
  0x40,  // AGCCTRL1  AGC Control
  0x91,  // AGCCTRL0  AGC Control
  0x56,  // FREND1  Front End RX Configuration
  0x10,  // FREND0  Front End TX Configuration
  0xE9,  // FSCAL3  Frequency Synthesizer Calibration
  0x2A,  // FSCAL2  Frequency Synthesizer Calibration
  0x00,  // FSCAL1  Frequency Synthesizer Calibration
  0x1F,  // FSCAL0  Frequency Synthesizer Calibration
  0x59,  // FSTEST  Frequency Synthesizer Calibration Control
  0x81,  // TEST2  Various Test Settings
  0x35,  // TEST1  Various Test Settings
  0x09,  // TEST0  Various Test Settings
}
};

const byte CC1101_paTable[CC1101_NR_OF_CONFIGS][CC1101_PA_TABLESIZE] PROGMEM ={
// -30  -20   -15  -10    0   5    7   10
  {0x03,0x0E,0x1E,0x27,0x8E,0x84,0xCC,0xC3},    // Configuracao 0
};

#endif
