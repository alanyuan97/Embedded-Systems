#include "decode.h"
#include "bitcoin.h"
#include "motor.h"

RawSerial pc(SERIAL_TX, SERIAL_RX);
Mail<uint8_t, 8> inCharQ;

Mutex newkey_mutex;
uint64_t newkey = 0;

void serialISR(){
    uint8_t* newChar = inCharQ.alloc();
    *newChar = pc.getc(); 
    inCharQ.put(newChar);
}

void decode_tune(){
    int i;
    char first_token;
    char op_token;
    int8_t tmp_tune;
    uint8_t tune_counter;

    for(i=1;i<128;i++){
        tune_counter = 0;
        // reached end of buffer
        if (infobuffer[i] == '\0'){
            // do
            break;
        }
        else{
            first_token = infobuffer[i];
            // Found tune
            if (first_token>'A' && first_token<'G') {
                tune_counter = tune_counter<15 ? tune_counter++ : tune_counter=0;
                tmp_tune=(first_token-'A')*2+1;
                op_token = infobuffer[i+1];

                if (op_token>='0' && op_token<='9'){
                    TUNES[tune_counter] = (op_token-'0')<<4 | tmp_tune;
                }
                else{
                    if(op_token == '#'){
                        tmp_tune++;
                    }
                    else if (op_token == '^'){
                        tmp_tune--;
                    }
                    TUNES[tune_counter] = (op_token-'0')<<4 | tmp_tune;
                }
            }
        }
            
    }
}
void decode(){
    pc.attach(&serialISR); 
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t *newChar = (uint8_t*)newEvent.value.p; 

        if(*newChar!='\r'){
            infobuffer[counter] = *newChar;
            counter>=128 ? counter =0: counter++;
        }

        else{
            infobuffer[counter] =  '\0';
            switch(infobuffer[0]){
                // Key
                case('K'):
                    newkey_mutex.lock(); 
                    sscanf(infobuffer,"K%llx", &newkey);
                    dumpMes(5,newkey); // type 5
                    newkey_mutex.unlock();
                    wait(0.1);
                    break;
                // Velocity
                case('V'):
                    vel_enter = true; 
                    sscanf(infobuffer,"V%f",&target_vel);
                    integral_Es=0;
                    // dump the target TODO
                    break;
                    // Rotation numbers
                case('R'):
                    newkey_mutex.lock(); 
                    val_enter = true; 
                    sscanf(infobuffer,"R%f",&target_rot);
                    tmp_rot = (float)motorPosition/6; 
                    newkey_mutex.unlock(); 
                    // dump the target TODO
                    break;
                case('T'):
                    tune_enter = true;
                    newtune_mutex.lock();
                    decode_tune();
                    newtune_mutex.unlock();
                    break;
                default:
                    // Do nothing
                    // clear_buffer();
                    break;
            }
            counter = 0;
        }       
        inCharQ.free(newChar);
    }
}

void dumpMes(int type, uint64_t datain){
    newkey_mutex.lock();
    message *mail = mail_box.alloc();
    mail->key = datain;
    mail->typenames = type;
    mail_box.put(mail);
    newkey_mutex.unlock();
}