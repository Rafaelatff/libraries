// NET : classe da camada de Rede

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
 * Construtor da camada de Rede.
 */
NET::NET()
{
}

/**
 * Inicializa a camada de Controle de Acesso ao Meio.
 */
void NET::initialize(void) 
{
}

/**
 * Realiza a troca de endereços Origem e Destino
 */
void NET::swapAddresses(packet * pkt) 
{
  /* Troca os endereços de destino e origem para a retranmissão dos pacotes */
  pkt->NetHdr[0] = pkt->NetHdr[2];
  pkt->NetHdr[2] = my_addr;
}

/**
 * Envia o pacote para a camada inferior
 */
void NET::send(packet * pkt) 
{
  /* Envia para a camada inferior */
  Mac.send(pkt);
  
  return;  
}

/**
 * Recebe o pacote da camada inferior
 */
void NET::receive(packet * pkt) 
{      
  /* Troca os endereços de destino e origem para a retranmissão dos pacotes */
  pkt->NetHdr[0] = pkt->NetHdr[2];
  pkt->NetHdr[2] = my_addr;
  
  /* Envia para a camada superior */
  Transp.receive(pkt);
  
  return;  
}

/* Instanciação do objeto de acesso à classe da camada de Rede */
NET Net = NET();
