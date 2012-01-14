#ifndef COMMON_H_INCLUDED
# define COMMON_H_INCLUDED


/* common to host and device */

#define SCAB_CMD_FRAME 0
#define SCAB_CMD_SYNC 1
#define SCAB_CMD_ENABLE 2
#define SCAB_CMD_SET_CAN_TIMES 3
#define SCAB_CMD_SET_CAN_FILTER 4
#define SCAB_CMD_CLEAR_CAN_FILTER 5
#define SCAB_CMD_STATUS 6
/* must be the last one */
#define SCAB_CMD_INVALID 7

/* SCAB_STATUS_XXX */
#define SCAB_STATUS_SUCCESS 0
#define SCAB_STATUS_FAILURE 1
#define SCAB_STATUS_INVALID 2

/* command buffer fixed size, in bytes */
/* largest command: cmd + can_sid + can_payload */
#define SCAB_CMD_SIZE (1 + 2 + 8)


#endif /* ! COMMON_H_INCLUDED */
