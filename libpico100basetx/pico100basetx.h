#ifndef _PICO100BASETX_H
#define _PICO100BASETX_H

#define ntohs(x)	__builtin_bswap16(x)
#define htons(x)	__builtin_bswap16(x)
#define htonl(x)	__builtin_bswap32(x)
#define IP_ADDR(a, b, c, d)	(((a & 0xff) << 24) | ((b & 0xff) << 16) | ((c & 0xff) << 8) | (d & 0xff))

#define MAC_ADDR_LEN	6

/* Ethernet II frame with UDP/IP */
struct eth_hdr
{
	uint8_t  daddr[MAC_ADDR_LEN];	/* destination MAC address */
	uint8_t  saddr[MAC_ADDR_LEN];	/* source MAC address */
	uint16_t type;			/* ethertype field */
} __attribute__ ((__packed__));

struct ip_hdr
{
	uint8_t  ihl:4;
	uint8_t  version:4;
	uint8_t  tos;
	uint16_t tot_len;
	uint16_t id;
	uint16_t frag_off;
	uint8_t  ttl;
	uint8_t  protocol;
	uint16_t chksum;
	uint32_t saddr;
	uint32_t daddr;
} __attribute__ ((__packed__));

struct udp_hdr
{
	uint16_t src_port;
	uint16_t dst_port;
	uint16_t len;
	uint16_t chksum;
} __attribute__ ((__packed__));

#define IPV4_ETHERTYPE	0x0800
#define IPPROTO_UDP	17

#define ETH_HEADER_LEN	14
#define IP_HEADER_LEN	20
#define UDP_HEADER_LEN	8
#define MAX_MTU		1500
#define INFO_HEADER_LEN	4

#define MAX_UDP_LEN	(MAX_MTU - IP_HEADER_LEN - UDP_HEADER_LEN)
#define MAX_PAYLOAD_LEN	(MAX_UDP_LEN - INFO_HEADER_LEN)
#define UDP_PORT_BASE	42101

#define STATUS_OVERFLOW	(1 << 4)

struct udp_frame
{
	uint16_t _pad_align;			/* padding to align for 32-bit DMA input to payload buffer */
	struct eth_hdr eth;
	struct ip_hdr ip;
	struct udp_hdr udp;
	uint8_t info_hdr[INFO_HEADER_LEN];	/* our custom info header at beginning of UDP payload data */
	uint8_t data[MAX_PAYLOAD_LEN];
} __attribute__ ((__packed__));

#define RBUF_MAX_DATA_LEN	(MAX_PAYLOAD_LEN)
#define RBUF_DEFAULT_NUM	4

#define MAX_STREAMS		4

void pico100basetx_start(void);
void pico100basetx_update_head(int stream_id, int head);
int pico100basetx_set_destination(uint16_t stream_id, uint64_t dest_mac, uint32_t dest_ip, uint16_t dest_port);
void pico100basetx_set_host_address(uint64_t mac, uint32_t ip);
void pico100basetx_enable_udp_checksum(bool on);
int pico100basetx_add_stream(uint16_t stream_id, uint16_t format, uint32_t samplerate, uint length, uint slices, struct udp_frame *ringbuf);
int pico100basetx_remove_stream(uint16_t stream_id);
void pico100basetx_init(int base_pin);
#endif
