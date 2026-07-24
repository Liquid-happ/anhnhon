#ifndef COMMON_ERR_H_
#define COMMON_ERR_H_

typedef enum {
    ERR_OK = 0,
    ERR_FAIL = -1,
    ERR_TIMEOUT = -2,
    ERR_INVALID_PARAM = -3,
    ERR_BUSY = -4,
    ERR_CRC = -5,
    ERR_BUFF_FULL = -6,
    ERR_BUFF_EMPTY = -7
} err_t;

#endif /* COMMON_ERR_H_ */
