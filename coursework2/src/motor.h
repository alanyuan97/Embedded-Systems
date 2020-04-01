#ifndef _motor_h_
#define  _motor_h_
#include "mbed.h"
#include "messages.h"
#include "decode.h"
#include "rtos.h

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Test outputs
#define TP0pin D4
#define TP1pin D13
#define TP2pin A2

extern void motorCtrlTick();
extern inline int8_t readRotorState();
extern void motorOut(int8_t driveState);
extern int8_t motorHome();
extern void motorISR();
extern void CheckState();
extern void pos_control();
extern void vel_control();
extern void melody_tune_control();
extern void motorCtrlFn();
extern void posprint();
extern Mutex newtune_mutex;
#endif