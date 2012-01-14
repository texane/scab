#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "scab.h"
#include "serial.h"


/* little endian related routines
 */

static inline uint16_t read_uint16(uint8_t* s)
{
  return *(uint16_t*)s;
}

static inline uint32_t read_uint32(uint8_t* s)
{
  return *(uint32_t*)s;
}

static inline void write_uint16(uint8_t* s, uint16_t x)
{
  *(uint16_t*)s = x;
}

static inline void write_uint32(uint8_t* s, uint32_t x)
{
  *(uint32_t*)s = x;
}


/* protocol helpers
 */

static int read_status(scab_handle_t* h, uint8_t* status)
{
  uint8_t buf[SCAB_CMD_SIZE];

  if (serial_readn(h, buf, SCAB_CMD_SIZE)) return -1;
  if (buf[0] != SCAB_CMD_STATUS) return -1;
  *status = buf[1];

  return 0;
}

static int write_cmd_read_status(scab_handle_t* h, const uint8_t* buf)
{
  uint8_t status;  

  if (serial_writen(h, buf, SCAB_CMD_SIZE)) return -1;
  if (read_status(h, &status)) return -1;
  return status == SCAB_STATUS_SUCCESS ? 0 : -1;
}


/* exported programming interface
 */

int scab_open(scab_handle_t* h, const char* devname)
{
  static const serial_conf_t conf = { 38400, 8, SERIAL_PARITY_DISABLED, 1 };

  if (serial_open(h, devname) == -1)
  {
    printf("serial_open(%s) == -1\n", devname);
    return -1;
  }

  if (serial_set_conf(h, &conf) == -1)
  {
    printf("serial_set_conf() == -1\n");
    serial_close(h);
    return -1;
  }

  /* flush residual */

  return 0;
}


int scab_close(scab_handle_t* h)
{
  serial_close(h);
  return 0;
}


int scab_sync_serial(scab_handle_t* h)
{
  /* read until no more byte avail */

  uint8_t buf[128];
  ssize_t res;

  while (1)
  {
    errno = 0;
    res = read(h->fd, buf, sizeof(buf));

    if (res <= 0)
    {
      if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) break ;
      return -1;
    }

    /* something read, continue */
  }

  return 0;
}


int scab_read_frame(scab_handle_t* h, uint16_t* sid, uint8_t* buf)
{
  /* todo: tmp buffer not needed */
  uint8_t scab_buf[SCAB_CMD_SIZE];

  if (serial_readn(h, scab_buf, SCAB_CMD_SIZE)) return -1;

  if (scab_buf[0] != SCAB_CMD_FRAME) return -1;

  if (sid != NULL) *sid = read_uint16(scab_buf + 1);

#define CAN_PAYLOAD_SIZE 8
  memcpy(buf, scab_buf + 3, CAN_PAYLOAD_SIZE);

  return 0;
}


int scab_write_frame(scab_handle_t* h, uint16_t sid, const uint8_t* buf)
{
  /* todo: tmp buffer not needed */
  uint8_t scab_buf[SCAB_CMD_SIZE];

  scab_buf[0] = SCAB_CMD_FRAME;
  write_uint16(scab_buf + 1, sid);
  memcpy(scab_buf + 3, buf, CAN_PAYLOAD_SIZE);

  return serial_writen(h, scab_buf, SCAB_CMD_SIZE);
}


/* enable, disable packet forwarding
 */

static int set_bridge(scab_handle_t* h, uint8_t is_enabled)
{
  uint8_t scab_buf[SCAB_CMD_SIZE];

  scab_buf[0] = SCAB_CMD_ENABLE;
  scab_buf[1] = is_enabled;

  return write_cmd_read_status(h, scab_buf);
}

int scab_enable_bridge(scab_handle_t* h)
{
  return set_bridge(h, 1);
}

int scab_disable_bridge(scab_handle_t* h)
{
  return set_bridge(h, 0);
}


/* CAN filter commands
 */

int scab_set_can_filter(scab_handle_t* h, uint16_t m, uint16_t v)
{
  /* m the filter mask */
  /* v the filter value */

  uint8_t scab_buf[SCAB_CMD_SIZE];
  scab_buf[0] = SCAB_CMD_SET_CAN_FILTER;
  write_uint16(scab_buf + 1, m);
  write_uint16(scab_buf + 3, v);
  return write_cmd_read_status(h, scab_buf);
}


int scab_clear_can_filter(scab_handle_t* h)
{
  uint8_t scab_buf[SCAB_CMD_SIZE];
  scab_buf[0] = SCAB_CMD_CLEAR_CAN_FILTER;
  return write_cmd_read_status(h, scab_buf);
}
