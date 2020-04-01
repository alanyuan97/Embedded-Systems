#ifndef _decode_h_
#define  _decode_h_
#include "bitcoin.h"
#include "messages.h"

extern volatile float target_vel;
extern volatile float target_rot;
extern volatile bool vel_enter = false; 
extern volatile bool val_enter = false; 
extern volatile bool tune_enter = false;

extern char infobuffer[128];
extern uint8_t TUNES[16] = {0};
extern uint8_t tune_idx;

extern RawSerial pc;

void serialISR()；
void decode();
void decode_tune();
void dumpMes(int type, uint64_t datain)

#endif