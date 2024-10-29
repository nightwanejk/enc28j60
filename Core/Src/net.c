/*
 * net.c
 *
 *  Created on: Sep 24, 2024
 *      Author: user
 */

#include "net.h"

uint8_t net_buf[ENC28J60_MAXFRAME];
uint8_t ipaddr[4] = IP_ADDR;
extern uint8_t macaddr[6];
char str1[60] = {0};

void net_init(void)
{
	enc28j60_init();
}

uint8_t arp_read(enc28j60_frame_ptr *frame, uint16_t len)
{
	uint8_t res = 0;
	arp_msg_ptr *msg = (void*)(frame->data);

	if (len>=sizeof(arp_msg_ptr))

	{

	  if ((msg->net_tp==ARP_ETH)&&(msg->proto_tp==ARP_IP))

	  {

	    if ((msg->op==ARP_REQUEST)&&(!memcmp(msg->ipaddr_dst,ipaddr,4)))

	    {
	    	sprintf(str1,"requestrnmac_src %02X:%02X:%02X:%02X:%02X:%02Xrn",

	    	    msg->macaddr_src[0],msg->macaddr_src[1],msg->macaddr_src[2],msg->macaddr_src[3],msg->macaddr_src[4],msg->macaddr_src[5]);


	    	  sprintf(str1,"ip_src %d.%d.%d.%drn",

	    	    msg->ipaddr_src[0],msg->ipaddr_src[1],msg->ipaddr_src[2],msg->ipaddr_src[3]);


	    	  sprintf(str1,"nmac_dst %02X:%02X:%02X:%02X:%02X:%02Xrn",

	    	    msg->macaddr_dst[0],msg->macaddr_dst[1],msg->macaddr_dst[2],msg->macaddr_dst[3],msg->macaddr_dst[4],msg->macaddr_dst[5]);


	    	  sprintf(str1,"ip_dst %d.%d.%d.%drn",

	    	    msg->ipaddr_dst[0],msg->ipaddr_dst[1],msg->ipaddr_dst[2],msg->ipaddr_dst[3]);


	    	  res=1;
	    }

	  }

	  return res;

	}
}

void eth_read(enc28j60_frame_ptr *frame, uint16_t len)
{
	if(frame->type==ETH_ARP)
	  {
	    sprintf(str1,"%02X:%02X:%02X:%02X:%02X:%02X-%02X:%02X:%02X:%02X:%02X:%02X; %d; arprn",
	    frame->addr_src[0],frame->addr_src[1],frame->addr_src[2],frame->addr_src[3],frame->addr_src[4],frame->addr_src[5],
	    frame->addr_dest[0],frame->addr_dest[1],frame->addr_dest[2],frame->addr_dest[3],frame->addr_dest[4],frame->addr_dest[5],
	    len);
	    if(arp_read(frame,len-sizeof(enc28j60_frame_ptr)))

	    {

	    	arp_send(frame);

	    }
	  }
	  else if(frame->type==ETH_IP)
	  {
	    sprintf(str1,"%02X:%02X:%02X:%02X:%02X:%02X-%02X:%02X:%02X:%02X:%02X:%02X; %d; iprn",
	    frame->addr_src[0],frame->addr_src[1],frame->addr_src[2],frame->addr_src[3],frame->addr_src[4],frame->addr_src[5],
	    frame->addr_dest[0],frame->addr_dest[1],frame->addr_dest[2],frame->addr_dest[3],frame->addr_dest[4],frame->addr_dest[5],
	    len);
	  }
    // Если это ICMP (ping), обрабатываем его
    if (ip_read(frame,len - sizeof(enc28j60_frame_ptr)))
    {
        return;
    }

}

void enc28j60_packetSend(uint8_t *buf,uint16_t buflen)

{

	while(enc28j60_readop(ENC28J60_READ_CTRL_REG,ECON1)&ECON1_TXRTS)

	{

	  if(enc28j60_readRegByte(EIR)& EIR_TXERIF)

	  {

	    enc28j60_writeop(ENC28J60_BIT_FIELD_SET,ECON1,ECON1_TXRST);

	    enc28j60_writeop(ENC28J60_BIT_FIELD_CLR,ECON1,ECON1_TXRST);
	    enc28j60_writeReg(EWRPT,TXSTART_INIT);

	    enc28j60_writeReg(ETXND,TXSTART_INIT+buflen);

	    enc28j60_writeBuf(1,(uint8_t*)"x00");

	    enc28j60_writeBuf(buflen,buf);

	    enc28j60_writeop(ENC28J60_BIT_FIELD_SET,ECON1,ECON1_TXRTS);
	  }

	}

}

void eth_send(enc28j60_frame_ptr *frame, uint16_t len)

{

  memcpy(frame->addr_dest,frame->addr_src,6);

  memcpy(frame->addr_src,macaddr,6);

  enc28j60_packetSend((void*)frame,len + sizeof(enc28j60_frame_ptr));

}

void arp_send(enc28j60_frame_ptr *frame)
{
	  arp_msg_ptr *msg = (void*)frame->data;

	  msg->op = ARP_REPLY;
	  memcpy(msg->macaddr_dst,msg->macaddr_src,6);

	  memcpy(msg->macaddr_src,macaddr,6);
	  memcpy(msg->ipaddr_dst,msg->ipaddr_src,4);

	  memcpy(msg->ipaddr_src,ipaddr,4);
	  eth_send(frame,sizeof(arp_msg_ptr));
}

void enc28j60_writeBuf(uint16_t len,uint8_t* data)
{
  SS_SELECT();
  SPI_SendByte(ENC28J60_WRITE_BUF_MEM);
  while(len--)
  SPI_SendByte(*data++);
  SS_DESELECT();
}

uint8_t ip_read(enc28j60_frame_ptr *frame, uint16_t len)
{
    ip_pkt_ptr *ip = (void*)(frame->data);

    if (ip->proto == IP_ICMP && !memcmp(ip->ipaddr_dst, ipaddr, 4))
    {
        icmp_pkt_ptr *icmp = (void*)(ip->data);

        if (icmp->type == ICMP_ECHO)
        {
            sprintf(str1, "ping request from %d.%d.%d.%d\n",
                ip->ipaddr_src[0], ip->ipaddr_src[1], ip->ipaddr_src[2], ip->ipaddr_src[3]);

            // Отправляем ICMP Echo Reply
            icmp_send(frame, len - sizeof(ip_pkt_ptr));

            return 1;
        }
    }

    return 0;
}

void icmp_send(enc28j60_frame_ptr *frame, uint16_t len)
{
    ip_pkt_ptr *ip = (void*)(frame->data);
    icmp_pkt_ptr *icmp = (void*)(ip->data);

    // Меняем тип на Echo Reply (значение 0)
    icmp->type = ICMP_ECHOREPLY;

    // Перемещаем исходные и целевые IP-адреса
    memcpy(ip->ipaddr_dst, ip->ipaddr_src, 4);
    memcpy(ip->ipaddr_src, ipaddr, 4);

    // Обновляем ICMP контрольную сумму
    icmp->chksum = 0;
    icmp->chksum = ip_checksum((uint8_t*)icmp, len);

    // Отправляем пакет через Ethernet
    eth_send(frame, len + sizeof(ip_pkt_ptr));
}


void net_pool(void)
{
	uint16_t len;
	enc28j60_frame_ptr *frame = (void*)net_buf;
	while ((len=enc28j60_packetReceive(net_buf,sizeof(net_buf)))>0)
	  {
	    eth_read(frame,len);
	  }
}

