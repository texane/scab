#ifndef SCAB_API_H_INCLUDED
# define SCAB_API_H_INCLUDED


/* exported API */

#include <stdint.h>

typedef void* scab_handle_t;

int scab_open(scab_handle_t**, const char*);
int scab_close(scab_handle_t*);
int scab_sync_serial(scab_handle_t*);
int scab_read_frame(scab_handle_t*, uint16_t*, uint8_t*);
int scab_write_frame(scab_handle_t*, uint16_t, const uint8_t*);
int scab_enable_bridge(scab_handle_t*);
int scab_disable_bridge(scab_handle_t*);
int scab_set_can_filter(scab_handle_t*, uint16_t, uint16_t);
int scab_clear_can_filter(scab_handle_t*);
int scab_get_handle_fd(scab_handle_t*);


#endif /* ! SCAB_API_H_INCLUDED */
