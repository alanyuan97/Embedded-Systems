#ifndef _messages_h_
#define  _messages_h_

#include "mbed.h"
#include "rtos.h"
#include "decode.h"

typedef struct{
    uint64_t data;
    double velocity;
    uint64_t key;
    int typenames; //1 for nounce, 2 for hash rate
}message;

void myprint();

#endif