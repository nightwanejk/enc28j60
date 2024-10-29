/*
 * net.h
 *
 *  Created on: Sep 24, 2024
 *      Author: user
 */

#ifndef INC_NET_H_
#define INC_NET_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "enc28j60.h"

void net_init(void);
void net_pool(void);


typedef struct enc28j60_frame
{
	uint8_t addr_dest[6];
	uint8_t addr_src[6];
	uint16_t type;
	uint8_t data[];
} enc28j60_frame_ptr;

typedef struct arp_msg{

  uint16_t net_tp;

  uint16_t proto_tp;

  uint8_t macaddr_len;

  uint8_t ipaddr_len;

  uint16_t op;

  uint8_t macaddr_src[6];

  uint8_t ipaddr_src[4];

  uint8_t macaddr_dst[6];

  uint8_t ipaddr_dst[4];

} arp_msg_ptr;

// Структура IP-заголовка
typedef struct {
    uint8_t ver_hdr_len;  // Версия и длина заголовка
    uint8_t tos;          // Тип сервиса
    uint16_t len;         // Общая длина
    uint16_t id;          // Идентификатор пакета
    uint16_t flags_frag_offset; // Флаги и смещение фрагмента
    uint8_t ttl;          // Время жизни
    uint8_t proto;        // Протокол (1 = ICMP)
    uint16_t chksum;      // Контрольная сумма
    uint8_t ipaddr_src[4]; // Исходный IP-адрес
    uint8_t ipaddr_dst[4]; // Целевой IP-адрес
    uint8_t data[];       // Данные пакета (начиная с заголовка ICMP)
} ip_pkt_ptr;

// Структура ICMP-заголовка
typedef struct {
    uint8_t type;         // Тип сообщения (Echo Request = 8, Echo Reply = 0)
    uint8_t code;         // Код (для Echo Request и Echo Reply всегда 0)
    uint16_t chksum;      // Контрольная сумма
    uint16_t id;          // Идентификатор пакета
    uint16_t seq;         // Номер последовательности
    uint8_t data[];       // Данные пакета (Echo-запроса/ответа)
} icmp_pkt_ptr;

void arp_send(enc28j60_frame_ptr *frame);
uint8_t ip_read(enc28j60_frame_ptr *frame, uint16_t len);
void icmp_send(enc28j60_frame_ptr *frame, uint16_t len);
uint16_t ip_checksum(uint8_t *buf, uint16_t len); // Добавьте эту функцию

#define be16toword(a) ((((a)>>8)&0xff)|(((a)<<8)&0xff00))

#define ETH_ARP be16toword(0x0806)
#define ETH_IP be16toword(0x0800)

#define ARP_ETH be16toword(0x0001)
#define ARP_IP be16toword(0x0800)
#define ARP_REQUEST be16toword(1)
#define ARP_REPLY be16toword(2)

#define IP_ICMP 1         // Протокол ICMP (используется в IP заголовке)
#define ICMP_ECHO 8       // Тип ICMP Echo Request
#define ICMP_ECHOREPLY 0  // Тип ICMP Echo Reply


#define IP_ADDR {192,168,1,197}

#endif /* INC_NET_H_ */
