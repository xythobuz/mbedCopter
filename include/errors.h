#ifndef _ERRORS_H
#define _ERRORS_H

#define NUM_ERRORS 8

#define SUCCESS 0
#define ERR_ARGUMENT 1
#define ERR_ACC_READ 2
#define ERR_ACC_WRITE 3
#define ERR_GYRO_WRITE 4
#define ERR_GYRO_READ 5
#define ERR_ALT_WRITE 6
#define ERR_ALT_READ 7

const char *getErrorString(int error);

#endif