// Phy : classe da camada Física

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
 * Construtor da camada Física.
 */
PHY::PHY()
{  
}

/**
 * Inicializa a camada Física.
 */
void PHY::initialize(void)
{
  /* Configurando o serial baudrate */
  Serial.begin(serial_baudrate);
  
  //* Inicializa o transceptor CC1101 */
  cc1101.PowerOnStartUp();
  
  /* Inicializa a configuração do transceptor CC1101 */
  initCC1101Config();
  
  /* Configura o canal a ser usada */
  setChannel(channel);

  /* Configura a potência a ser usada */
  setPower(power);

  /* Configura o offset de frequencia */
  setFreqOffset(freq_offset);
  
   /*Configura o tipo de rádio*/
  setRadio(set_radio);

  /* Configura o timeout do WatchDog para 1 segundo */
  configWatchdog(6);
}

/**
 * Recebe dados da porta serial.
 */
void PHY::receiveSerial(void) 
{ 
  byte len;             /* Tamanho do dado recebido pela porta serial */
  byte fifoSize = 0;    /* Tamanho do espaço livre no FIFO de TX */
 
  static byte pos = 0;  /* Total de bytes recebidos pela porta serial */
  
  /* Le a porta serial e incrementa o total de bytes recebidos */
  len = Serial.available() + pos;
  
  /* Processa no máximo BUFFLEN bytes */
  if (len > BUFFLEN ) 
  {
    len = BUFFLEN;
  }
  
  /* Verifica quanto espaço temos no FIFO de TX */
  fifoSize = Phy.txFifoFree();   /* O fifoSize deve ter o número de bytes atualmeente livre no FIFO de TX */
  
  /* Reinicializa as variáveis e sai da função */
  if ( fifoSize <= 0)
  {
    Serial.flush();
    pos = 0;
    return;
  }
  
  /* Evita estourar o FIFO de TX */
  if (len > fifoSize) 
  {
    len = fifoSize;  
  }
    
  /* Finalmente escreve os dados lidos da serial no FIFO de TX */
  for (byte i = pos; i < len; i++)
  {
    serialData[i] = Serial.read();  /* serialData é o nosso buffer global */
  }
    
  /* Atraso de 1 milissegundo */
  delayMicroseconds(1000);
  
  /* Verifica se existem mais dados para receber */
  if ((Serial.available() > 0)  && (len < CC1101_PACKT_LEN))
  {
    pos = len;  /* Mantem a quantidade de bytes já recebidos e espera pela próxima entrada nessa função */
    return;
  }
  
  if (len == sizeof(packet))
  {     
    /* Envia a mensagem recebida pelo RF */
    Phy.send((packet *)serialData);
    /* O buffer da serial está livre novamente */
    pos = 0; 
  }
  else 
  {
    Serial.flush();
    pos = 0;
    return;
  }
}

/**
 * Transmite dados pela porta serial.
 */
void PHY::sendSerial(packet * pkt) 
{ 
  /* Escreve o pacote inteiro na porta serial */
  Serial.write((byte *)pkt, sizeof(packet)); 
}

/**
 * Le o espaço disponível no FIFO de TX.
 */
byte PHY::txFifoFree(void) 
{
  byte size;

  cc1101.Read(CC1101_TXBYTES, &size);
  
  /* Trata um possível underflow to FIFO de TX */
  if (size >= 64)
  {
    cc1101.Strobe(CC1101_SFTX);
    cc1101.Read(CC1101_TXBYTES,&size);
  }
  
  return (CC1101_FIFO_SIZE - size);
}

/**
 * Ajusta o canal a ser usado.
 */
void PHY::setChannel(byte channel)
{
  cc1101.Write(CC1101_CHANNR, channel);
}

/**
 * Ajusta a potência a ser usada.
 */
void PHY::setPower(byte power)
{
  cc1101.setPA(0, power);
}

/**
 * Ajusta o offset de frequencia.
 */
void PHY::setFreqOffset(byte freq_offset)
{
  cc1101.Write(CC1101_FSCTRL0, freq_offset);
}

/* Ajusta o offset de frequencia.*/
void PHY::setRadio(int set_radio){
  
  if(set_radio == 1){

    /*configura pinos do PA e LNA*/
    pinMode (14, OUTPUT); // pino PC0 ligado ao PA - tx
    pinMode (15, OUTPUT); // pino PC1 ligado ao LNA - rx  
  
    /*Deixa LNA ligado*/
    digitalWrite (14, LOW); // desliga PA
    digitalWrite (15, HIGH);  // liga LNA
  
  }
  
}

/**
 * Inicializa a configuração do CC1101.
 */
int PHY::initCC1101Config(void)
{
  /* Carrega a primeira configuração (pode ser inserida mais de uma configuração no código) */
  cc1101.Setup(0);
  
  /* Configura o endereço do rádio */
  cc1101.Write(CC1101_ADDR, my_addr);

  /* Configura a potência para a máxima possível (7) */
  cc1101.setPA(0, 7);
  
  /* Coloca o CC1101 no estado de RX */
  cc1101.Strobe(CC1101_SIDLE);
  delay(1);
  cc1101.Write(CC1101_MCSM1 ,   0x0F );
  cc1101.Strobe(CC1101_SFTX);
  cc1101.Strobe(CC1101_SFRX);
  cc1101.Strobe(CC1101_SRX);
    
  return OK;
}

/**
 * Entra em estado de Low Power mode
 */
void PHY::lowPowerOn(void){

  /* Colocando o CC1101 no estado de SLEEP */
  cc1101.Strobe(CC1101_SIDLE);
  cc1101.Strobe(CC1101_SPWD);

  /* Desabilita o conversor AD */
  _SFR_BYTE(ADCSRA) &= ~_BV(ADEN);
  
  /* Configura o modo Sleep do ATMega */  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  cli(); sleep_enable();          /* Habilita o bit de Sleep no registrador MCUCR */
  
  /* Desliga os módulos do ATMega */
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_usart0_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();

  /* Coloca o ATMega em modo Sleep efetivamente */
  sei(); sleep_mode();

  /**** O PROGRAMA CONTINUA AQUI DEPOIS DE ACORDAR ****/
  sleep_disable();         /* Desabilita o bit de Sleep */
    
  /* Habilita o conversor AD */
  _SFR_BYTE(ADCSRA) |= _BV(ADEN);
  
  /* Liga todos os modulos do ATMega */
  power_all_enable();

}

/**
 * Retorna o status de presença de portadora no canal (Carrier Sense)
 * return   1 se o canal está ocupado, 0 se o canal está livre
 */
boolean PHY::carrierSense(void)
{
  byte cs;

  /* O status da portadora é lido no registrador PKTSTATUS */  
  cc1101.Read(CC1101_PKTSTATUS,&cs);
  /* O bit de Carrier Sense é o bit 6 */
  cs &= 0x40;
  
  if (cs)
    return true;
  else
    return false;
    
}

/**
 * Configura o valor do timeout do WatchDog
 * 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms, 6=1sec, 7=2sec, 8=4sec, 9=8sec
 */
void PHY::configWatchdog(int time) {

  byte value;

  if (time > 9 ) time = 9;
  value = time & 7;
  if (time > 7) value |= (1<<5);
  value |= (1<<WDCE);

  /* Habilita a interrupção de WatchDog no Satus Register */
  MCUSR &= ~(1<<WDRF);
  
  /* Configura as flags do registrador de WatchDog */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  
  /* Configura o valor do timeout */
  WDTCSR = value;
  
  /* Habilita a interrupção do WatchDog */
  WDTCSR |= _BV(WDIE);

}

/**
 *    Envia comando para o CC1101 entrar no estado IDLE e espera a execução dele
 */
void PHY::strobe_idle_wait( void )
{
  byte current_state;
  
  /* Strobe to IDLE state */
  cc1101.Strobe(CC1101_SIDLE);
  
  do 
  {
    /* Read current state */	
    cc1101.Read( CC1101_MARCSTATE, &current_state);
      
  /* Until in IDLE state */
  } while ( current_state != 0x01 );
}

/**
 * Envia dados pelo RF.
 */
void PHY::send(packet * pkt)
{  
  byte *txData = (byte *)pkt;
  
  if(set_radio == 1){
    
    // pino PC1 ligado ao LNA - rx
    digitalWrite (15, LOW);  // pino PC1 ligado ao LNA - rx
  
    // pino PC0 ligado ao PA - tx  
    digitalWrite (14, HIGH); // pino PC0 ligado ao PA - tx  
    delay(1);
  
  }
    
  /* Coloca o CC1101 no estado IDLE */
  cc1101.Strobe(CC1101_SIDLE);
  
  /* Escreve uma rajada com os dados a serem transmitidos */
  cc1101.WriteBurst(CC1101_TXFIFO, txData, sizeof(packet));
   
  /* Vai para o estado TX */
  cc1101.Strobe(CC1101_STX);
  
  /* Aguarda enquanto todos os bytes estão sendo transmitidos */
  while(1)
  {
    byte size;
        cc1101.Read(CC1101_TXBYTES, &size);
    if( size == 0 )
    {
      break;
    }
    else
    {
      cc1101.Strobe(CC1101_STX);
    }
  }
  
  if(set_radio == 1){
    
    // desliga PA - tx 
    digitalWrite (14, LOW); // pino PC0 ligado ao PA - tx

    // liga  LNA - rx
    digitalWrite (15, HIGH);  // pino PC1 ligado ao LNA - rx
  
  }
}

/**
 * Recebe dados pelo RF.
 */
int PHY::receive(packet * pkt){

  byte rxBytes, rxBytesVerify, rssi, lqi, current_state;
    
  /* Desliga o LED Verde */
  digitalWrite(IO2_PIN, HIGH);

  /* Garante que o pacote terminou de ser recebido - deve ler duas vezes a mesma quantidade de bytes */
  cc1101.Read(CC1101_RXBYTES, &rxBytesVerify);
  do
  {
    rxBytes = rxBytesVerify;
    cc1101.Read(CC1101_RXBYTES, &rxBytesVerify);  
  } while (rxBytes != rxBytesVerify);

  /* Checagem de sanidade para ver se o buffer não está vazio */
  if (rxBytes == 0)
  {
    cc1101.Strobe(CC1101_SRX);
	/* Desliga o LED Verde */
	digitalWrite(IO2_PIN, LOW);
    return ERR;
  }

  /* Verifica se chegaram todos os bytes */
  if ((rxBytes != (sizeof(packet) +  2)|| (rxBytes & 0x80))) 
  {
     /* Flush o FIFO de recepção. Deve ir para IDLE para fazer isso. */
     strobe_idle_wait();
     cc1101.Strobe( CC1101_SFRX );
     cc1101.Strobe( CC1101_SRX );
     /* Desliga o LED Verde */
	 digitalWrite(IO2_PIN, LOW);
     return ERR;
  }

  /* Le uma rajada de dados do FIFO de RX */
  cc1101.ReadBurst(CC1101_RXFIFO, (byte *)pkt, sizeof(packet));
  
  /* Le o byte de RSSI */
  cc1101.Read(CC1101_RXFIFO, &rssi);

  /* Le o byte de LQI */
  cc1101.Read(CC1101_RXFIFO, &lqi);
  
  /* Verifica se o endereço destino é o meu - o pacote é para mim */
  if (pkt->NetHdr[0] != my_addr)
  {
	/* Desliga o LED Verde */
	digitalWrite(IO2_PIN, LOW);
    return ERR;
  }

  /* Trata um possível overflow do FIFO de RX */
  if (!(lqi & 0x80))
  {
    strobe_idle_wait();            /* Descarta o FIFO de RX */
    cc1101.Strobe( CC1101_SFRX );
    cc1101.Strobe( CC1101_SRX );
    /* Desliga o LED Verde */
    digitalWrite(IO2_PIN, LOW);
    return ERR;
  }
  
  /* Coloca a informação de RSSI e LQI no cabeçalho da camada física */
  pkt->PhyHdr[0] = rssi;
  pkt->PhyHdr[1] = lqi;

  /* Envia o pacote para a camada superior */
  Mac.receive(pkt);
  
  /* Espera que o modulo retorne para o estado de RX */
  do
  {
    cc1101.Read(CC1101_MARCSTATE, &current_state);
  } while ( current_state != 0x0D );
  
  /* Desliga o LED Verde */
  digitalWrite(IO2_PIN, LOW);

  return OK;
}

/**
 * Recebe dados pelo RF.
 */
int PHY::receive_base(packet * pkt){

  byte rxBytes, rxBytesVerify, rssi, lqi, current_state;
    
  /* Garante que o pacote terminou de ser recebido - deve ler duas vezes a mesma quantidade de bytes */
  cc1101.Read(CC1101_RXBYTES, &rxBytesVerify);
  do
  {
    rxBytes = rxBytesVerify;
    cc1101.Read(CC1101_RXBYTES, &rxBytesVerify);  
  } while (rxBytes != rxBytesVerify);

  /* Checagem de sanidade para ver se o buffer não está vazio */
  if (rxBytes == 0)
  {
    cc1101.Strobe(CC1101_SRX);
    return ERR;
  }

  /* Verifica se chegaram todos os bytes */
  if ((rxBytes != (sizeof(packet) +  2)|| (rxBytes & 0x80))) 
  {
     /* Flush o FIFO de recepção. Deve ir para IDLE para fazer isso. */
     strobe_idle_wait();
     cc1101.Strobe( CC1101_SFRX );
     cc1101.Strobe( CC1101_SRX );
     
     return ERR;
  }

  /* Le uma rajada de dados do FIFO de RX */
  cc1101.ReadBurst(CC1101_RXFIFO, (byte *)pkt, sizeof(packet));
  
  /* Le o byte de RSSI */
  cc1101.Read(CC1101_RXFIFO, &rssi);

  /* Le o byte de LQI */
  cc1101.Read(CC1101_RXFIFO, &lqi);
  
  /* Verifica se o endereço destino é o meu - o pacote é para mim */
  if (pkt->NetHdr[0] != my_addr)
  {
    return ERR;
  }

  /* Trata um possível overflow do FIFO de RX */
  if (!(lqi & 0x80))
  {
    strobe_idle_wait();            /* Descarta o FIFO de RX */
    cc1101.Strobe( CC1101_SFRX );
    cc1101.Strobe( CC1101_SRX );
    return ERR;
  }
  
  /* Coloca a informação de RSSI e LQI no cabeçalho da camada física */
  pkt->PhyHdr[2] = rssi;
  pkt->PhyHdr[3] = lqi;
     
  /* Espera que o modulo retorne para o estado de RX */
  do
  {
    cc1101.Read(CC1101_MARCSTATE, &current_state);
  } while ( current_state != 0x0D );
  
  return OK;
}

/* Instanciação do objeto de acesso à classe da camada Física */
PHY Phy = PHY();
