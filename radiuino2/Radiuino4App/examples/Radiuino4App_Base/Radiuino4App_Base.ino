// Radiuino4App_Base : firmware para o nó Base da rede

// Mais informações em www.radiuino.cc
// Copyright (c) 2011
// Author: Pedro Henrique Gomes e Omar C. Branquinho
// Versão 1.0: 15/12/2011

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

#include <RADIUINO4APP.h>
#include <EEPROM.h>
#include <SPI.h>

#include "Headers.h"

byte int_rx = 0;                   /* Inicialização da interrupção de recepção de pacotes - O hardware gera uma interrupção no começo que não deve ser tratada */
byte int_buff = 0;                 /* Inicialização da interrupção de buffer overflow - O hardware gera uma interrupção no começo que não deve ser tratada */

/**
 * Configura o Arduino. É executado uma única vez no início do firmware.
 */
void setup()
{ 
  
  /* Variáveis de configuração do firmware */ 
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  //                    ENDEREÇO DA BASE
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  // Endereço ou ID da base, que pode ser escolhido de 0 a 255. Só pode existir um endereço por base presente na rede
  my_addr = 0;               /* Endereço */
  
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  //                    AJUSTE DE POTÊNCIA
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  // A potência de transmissão pode ser escolhida entre 8 valores possíveis (0 a 7). Abaixo está a tabela que relaciona o número com a potência de transmissão.
  // Em geral a potência é ajustada para o máximo de 7 (10 dBm). Este valor só muda para distâncias pequenas entre base e sensor.
  // -30  -20   -15  -10    0    5    7   10    - Potência em dBm
  //   0    1     2    3    4    5    6    7    - Número que deve ser colocado na potência
  power = 7;                 /* Potência */
  
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  //                    ESCOLHA DO CANAL DE COMUNICAÇÃO
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  // Pode ser escolhido de 0 a 65. Os canais estão espaçados de 125kHz na faixa de 915 a 923.125 MHz
  channel = 0;               /* Canal */
  
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  //                    AJUSTE DE OFFSET
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  // O rádio usa um cristal que pode apresentar uma diferença de frequência que deve ser compensada. Este número está escrino no BE900
  freq_offset = 0x13;        /* Offset de frequencia */
  
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  //                    TAXA DA SERIAL
  //+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
  // Quando o sensor está ligado no programador é possível que ele envie dados pela serial. É útil para debug de código.
  serial_baudrate = 9600;    /* Serial baudrate */ 

  /************ 0 - BE900/RFBee || 1 - BE990***************/
  set_radio = 1;
  
  /* Inicialização da camada Física */
  Phy.initialize();

  /* Anexa as funções de interrupção de recepção de pacotes e de estouro de buffer de recepção */
  attachInterrupt(0, IntReceiveData, RISING);
  attachInterrupt(1, IntBufferOverflow, RISING); 
  pinMode(GDO0, INPUT);

  /* Inicializa o Timer1 e configura o período para 1 segundo */
  Timer1.initialize(1000000);
  
  /* Anexa a função de interrupção de estouro do Timer1 */
  Timer1.attachInterrupt(IntTimer1);  
  
  /* Escreve mensagem de inicialização */
  Serial.print("Radiuino! Base");
  
}

/**
 * Laço de execução do Arduino. É executado continuamente.
 */
void loop()
{
  /* Aguarda pela recepção de comandos da porta serial */
  if (Serial.available() > 0)
  { 
    Phy.receiveSerial();
  }
}

/**
 * Trata o pacote recebido pela rede sem fio.
 */
void IntReceiveData()
{ 
  /* Se for a primeira vez que a interrupção é executada */
  if (int_rx == 0) 
  {
    int_rx = 1;
    return;
  }

  /* Reenvia toda informação recebida da rede para o PC */
  if ( digitalRead(GDO0) == HIGH ) {
    
    /* Recebe os dados do RF */
    if (Phy.receive_base(&g_pkt) == ERR)
      return;
    
    /* Reenvia pela porta serial */
    Phy.sendSerial(&g_pkt);
  }
  
  return;
}

/**
 * Trata o estouro do buffer de recepção.
 */
void IntBufferOverflow() 
{
  /* Se for a primeira vez que a interrupção é executada */
  if (int_buff == 0) {
    int_buff = 1;
    return;
  }

  /* Esvazia o buffer de recepção e vai para o estado de RX */  
  cc1101.Strobe(CC1101_SFRX);
  cc1101.Strobe(CC1101_SRX);
  
  return;  
}

/**
 * Trata o estouro do período do Timer1.
 */
void IntTimer1()
{  
  /* Aqui podem ser colocadas tarefas periódicas. O período do Timer1 é de 1 segundo por padrão.
   * Mas pode ser mudado no setup() */  
  
  return;
}

/**
 * Interrupção de WatchDog. É executada sempre que o contador de WatchDog é estourado
 */
ISR(WDT_vect) {
 
}
