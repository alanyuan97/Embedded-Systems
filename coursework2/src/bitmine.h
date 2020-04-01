#ifndef _bitmine_h_
#define  _bitmine_h_
#include "mbed.h"
#include "./Crypto/hash/SHA256.h"

void compute();

extern Mutex newkey_mutex;
extern uint64_t newkey = 0;
void CountHash();

#endif