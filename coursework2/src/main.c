#include "mbed.h"
#include "messages.h"
#include "bitmine.h"
#include "decode.h"
#include "motor.h"
#include "Crypto.h"
#include "rtos.h"
#include <stdio.h>

Thread print_thread;
Thread decode_thread(osPriorityNormal,1024);
Thread motorCtrlT(osPriorityHigh,1024);

int main() {

    Ticker timing;
    Ticker position;
    MotorPWM.period(0.002f);
    MotorPWM.write(1.0f); //

    //Initialise the serial port
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();


    pc.printf("Rotor origin: %x\n\r",orState);

    CheckState();

    timing.attach(&CountHash, 1.0);
    position.attach (&posprint,2);

    //compute_thread.start(callback(compute));
    print_thread.start(callback(myprint));
    decode_thread.start(callback(decode));
    motorCtrlT.start(callback(motorCtrlFn));
    
    while (1) {
        // Protect key assignment from thread interrupt
        newkey_mutex.lock();
        *key = newkey;
        newkey_mutex.unlock();
        //Compute Hash
        compute();
        *nonce = *nonce + 1;
        hash_counter = hash_counter + 1;
    }
    // measurement();
}