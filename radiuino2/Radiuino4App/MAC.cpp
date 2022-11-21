// MAC : classe da camada de Controle de Acesso ao Meio

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

#include "RADIUINO4APP.h"

/**
 * Construtor da camada de Controle de Acesso ao Meio.
 */
MAC::MAC()
{
}

/**
 * Inicializa a camada de Controle de Acesso ao Meio.
 */
void MAC::initialize(void) 
{
  time_to_sleep = -1;
}

/**
 * Envia o pacote para a camada inferior
 */
void MAC::send(packet * pkt) 
{
  unsigned long starttime = millis();

  /* Aguarda enquanto o canal está ocupado. Espera no máximo 100 ms */
  while(Phy.carrierSense() && ((millis() - starttime) < 100));
  
  /* Envia para a camada inferior */
  Phy.send(pkt);
  
  return;  
}

/**
 * Recebe o pacote da camada inferior
 */
void MAC::receive(packet * pkt) 
{
  /* Se a mensagem é do tipo SLEEP */
  if (pkt->MACHdr[0] == SLEEP_MSG) {
    
    /* Calcula o tempo total para dormir */
    time_to_sleep = 256 * (pkt->MACHdr[1] & 0x7F) + pkt->MACHdr[2];
    
    /* Retorna um pacote de SLEEP ACK para o emissor */
    pkt->MACHdr[0] = SLEEP_ACK;

    /* Troca os endereços de Origem e Destino */
    Net.swapAddresses(pkt);
    
    /* Send SLEEP ACK packet */
    Phy.send(pkt);
  }
  else {
    Net.receive(pkt);
  }
  
  return;  
}

/* Instanciação do objeto de acesso à classe da camada de Controle de Acesso ao Meio */
MAC Mac = MAC();





