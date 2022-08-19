//
// FILE:    sntpWiFiUdp.cpp
// TITLE:   ntp program via UDP
//
// Created by Y.J. Lu (yijenlu@yahoo.com)
// Date: Jul 24, 2022

#include <LWiFi.h>
#include "WiFiUdp.h"
#include "UARTClass.h"
#include "sntp.h"

#define SNTP_ERR_KOD           1
#define SNTP_MSG_LEN           48

#define SNTP_OFFSET_LI_VN_MODE      0
#define SNTP_LI_MASK                0xC0
#define SNTP_LI_NO_WARNING          (0x00 << 6)
#define SNTP_LI_LAST_MINUTE_61_SEC  (0x01 << 6)
#define SNTP_LI_LAST_MINUTE_59_SEC  (0x02 << 6)
#define SNTP_LI_ALARM_CONDITION     (0x03 << 6) /* (clock not synchronized) */

#define SNTP_VERSION_MASK           0x38
#define SNTP_VERSION                (4/* NTP Version 4*/<<3)

#define SNTP_MODE_MASK              0x07
#define SNTP_MODE_CLIENT            0x03
#define SNTP_MODE_SERVER            0x04
#define SNTP_MODE_BROADCAST         0x05

#define SNTP_OFFSET_STRATUM         1
#define SNTP_STRATUM_KOD            0x00

#define SNTP_OFFSET_ORIGINATE_TIME  24
#define SNTP_OFFSET_RECEIVE_TIME    32
#define SNTP_OFFSET_TRANSMIT_TIME   40

/* Number of seconds between 1970 and Feb 7, 2036 06:28:16 UTC (epoch 1) */
#define DIFF_SEC_1970_2036          ((u32_t)2085978496L)

#ifndef SNTP_SET_SYSTEM_TIME_NTP
# ifdef SNTP_SET_SYSTEM_TIME_US
#  define SNTP_SET_SYSTEM_TIME_NTP(s, f) \
    SNTP_SET_SYSTEM_TIME_US((u32_t)((s) + DIFF_SEC_1970_2036), SNTP_FRAC_TO_US(f))
# else
#  define SNTP_SET_SYSTEM_TIME_NTP(s, f) \
    SNTP_SET_SYSTEM_TIME((u32_t)((s) + DIFF_SEC_1970_2036))
# endif
#endif /* !SNTP_SET_SYSTEM_TIME_NTP */

/* Start offset of the timestamps to extract from the SNTP packet */
#define SNTP_OFFSET_TIMESTAMPS  (SNTP_OFFSET_TRANSMIT_TIME + 8 - sizeof(struct sntp_timestamps))

struct sntp_time {
  u32_t sec;
  u32_t frac;
};

struct sntp_timestamps {
#if SNTP_COMP_ROUNDTRIP || SNTP_CHECK_RESPONSE >= 2
  struct sntp_time orig;
  struct sntp_time recv;
#endif
  struct sntp_time xmit;
};

PACK_STRUCT_BEGIN
struct sntp_msg {
  PACK_STRUCT_FLD_8(u8_t  li_vn_mode);
  PACK_STRUCT_FLD_8(u8_t  stratum);
  PACK_STRUCT_FLD_8(u8_t  poll);
  PACK_STRUCT_FLD_8(u8_t  precision);
  PACK_STRUCT_FIELD(u32_t root_delay);
  PACK_STRUCT_FIELD(u32_t root_dispersion);
  PACK_STRUCT_FIELD(u32_t reference_identifier);
  PACK_STRUCT_FIELD(u32_t reference_timestamp[2]);
  PACK_STRUCT_FIELD(u32_t originate_timestamp[2]);
  PACK_STRUCT_FIELD(u32_t receive_timestamp[2]);
  PACK_STRUCT_FIELD(u32_t transmit_timestamp[2]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

WiFiUDP ntpClient;
uint8_t sntpBuf[SNTP_MSG_LEN];
uint8_t sntpRxBuf[SNTP_MSG_LEN*2];
u32_t sntp_timestamp;

void sntp_initialize_request(struct sntp_msg *req)
{
  memset(req, 0, SNTP_MSG_LEN);
  req->li_vn_mode = SNTP_LI_NO_WARNING | SNTP_VERSION | SNTP_MODE_CLIENT;

#if SNTP_CHECK_RESPONSE >= 2 || SNTP_COMP_ROUNDTRIP
  {
    s32_t secs;
    u32_t sec, frac;
    /* Get the transmit timestamp */
    SNTP_GET_SYSTEM_TIME_NTP(secs, frac);
    sec  = lwip_htonl((u32_t)secs);
    frac = lwip_htonl(frac);

# if SNTP_CHECK_RESPONSE >= 2
    sntp_last_timestamp_sent.sec  = sec;
    sntp_last_timestamp_sent.frac = frac;
# endif
    req->transmit_timestamp[0] = sec;
    req->transmit_timestamp[1] = frac;
  }
#endif /* SNTP_CHECK_RESPONSE >= 2 || SNTP_COMP_ROUNDTRIP */
}

void sntp_process(const struct sntp_timestamps *timestamps)
{
  s32_t sec;
  u32_t frac;

  sec  = (s32_t)lwip_ntohl(timestamps->xmit.sec);
  frac = lwip_ntohl(timestamps->xmit.frac);

#if SNTP_COMP_ROUNDTRIP
  {
    s32_t dest_sec;
    u32_t dest_frac;
    u32_t step_sec;

    /* Get the destination time stamp, i.e. the current system time */
    SNTP_GET_SYSTEM_TIME_NTP(dest_sec, dest_frac);

    step_sec = (dest_sec < sec) ? ((u32_t)sec - (u32_t)dest_sec)
               : ((u32_t)dest_sec - (u32_t)sec);
    /* In order to avoid overflows, skip the compensation if the clock step
     * is larger than about 34 years. */
    if ((step_sec >> 30) == 0) {
      s64_t t1, t2, t3, t4;

      t4 = SNTP_SEC_FRAC_TO_S64(dest_sec, dest_frac);
      t3 = SNTP_SEC_FRAC_TO_S64(sec, frac);
      t1 = SNTP_TIMESTAMP_TO_S64(timestamps->orig);
      t2 = SNTP_TIMESTAMP_TO_S64(timestamps->recv);
      /* Clock offset calculation according to RFC 4330 */
      t4 += ((t2 - t1) + (t3 - t4)) / 2;

      sec  = (s32_t)((u64_t)t4 >> 32);
      frac = (u32_t)((u64_t)t4);
    }
  }
#endif /* SNTP_COMP_ROUNDTRIP */
sntp_timestamp = sec + DIFF_SEC_1970_2036;
  Serial.print("sntp_process: "); Serial.println(sntp_timestamp);
  SNTP_SET_SYSTEM_TIME_NTP(sec, frac);
}

void sntp_recv(unsigned char* buffer, size_t len)
{
	struct sntp_timestamps timestamps;
	u8_t mode, stratum;
	err_t err;

	/* process the response */
	if (len == SNTP_MSG_LEN) {
    mode = buffer[SNTP_OFFSET_LI_VN_MODE] & SNTP_MODE_MASK;
    /* if this is a SNTP response... */
    if (mode == SNTP_MODE_SERVER) {
		  stratum = buffer[SNTP_OFFSET_STRATUM];

      if (stratum == SNTP_STRATUM_KOD) {
    		/* Kiss-of-death packet. Use another server or increase UPDATE_DELAY. */
    		err = SNTP_ERR_KOD;
    		Serial.println("sntp_recv: Received Kiss-of-Death");
      } else {
        memcpy(&timestamps, &buffer[SNTP_OFFSET_TIMESTAMPS], sizeof(timestamps));
  			err = ERR_OK;
      }
    } else {
      Serial.println("sntp_recv: Invalid mode in response");
      /* wait for correct response */
      err = ERR_TIMEOUT;
    }
  } else {
    Serial.println("sntp_recv: Invalid packet length");
  }

  if (err == ERR_OK) {
    /* correct packet received: process it it */
    sntp_process(&timestamps);
  } else if (err == SNTP_ERR_KOD) {
    /* KOD errors are only processed in case of an explicit poll response */
      /* Kiss-of-death packet. Use another server or increase UPDATE_DELAY. */
  } else {
    /* ignore any broken packet, poll mode: retry after timeout to avoid flooding */
  }
}


void sntp_init(IPAddress ntpIp)
{
  int txLen;
  struct sntp_msg *sntpmsg = (struct sntp_msg *)sntpBuf;
  
  if( !ntpClient.beginPacket(ntpIp, SNTP_PORT) ) Serial.println("beginPacket error!!!!");

  sntp_initialize_request(sntpmsg);
  txLen = ntpClient.write((const char*)sntpmsg, sizeof(struct sntp_msg));
  Serial.print("Sending request:"); Serial.println(txLen);

  if( !ntpClient.endPacket() ) Serial.println("endPacket error!!!!");
}

int sntp_recvfrom(void)
{
  int rxLen = 0;

  if( !ntpClient.parsePacket() ) Serial.println("parsePacket error!!!!");
  else {
    while( ntpClient.available() )
    {
      rxLen = ntpClient.read(sntpRxBuf, SNTP_MSG_LEN*2);
      if( rxLen > 0 ) {
        sntp_recv(sntpRxBuf, rxLen);
      }
    }
  }
  return rxLen;
}

