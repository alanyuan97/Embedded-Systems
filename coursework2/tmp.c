#include "mbed.h"
#include "./Crypto/hash/SHA256.h"
#include "rtos.h"
#include "Thread.h"
#include "Callback.h"

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
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

const float YVELMAX=0.8f;
//Phase lead to make motor spin
int8_t lead = -2;  //2 for forwards, -2 for backwards
int8_t orState = 0;
volatile uint32_t hash_counter=0;
volatile int32_t motorPosition=0; 

uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64, 0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73, 0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E, 0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20, 0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20, 0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key = (uint64_t*)&sequence[48];
uint64_t* nonce = (uint64_t*)&sequence[56];

const int tuneTable[]={
1000000/416,   //0 |G#/A~
1000000/440,   //1 |A
1000000/467,   //2 |A#/B~
1000000/494,   //3 |B
1000000/2000,  //4 | UNDEFINED
1000000/262,   //5 |C
1000000/278,   //6 |C#/D~
1000000/294,   //7 |D
1000000/312,   //8 |D#/E~
1000000/330,   //9 |E
1000000/2000,  //A| UNDEFINED
1000000/349,   //B |F
1000000/370,   //C |F#/G~
1000000/392,   //D |G
1000000/416,   //E |G#/A~
};

char infobuffer[128];
uint8_t TUNES[16] = {0};
uint8_t tune_idx;
int counter = 0;
bool movement_dir;

typedef struct{
    uint64_t data;
    double velocity;
    uint64_t key;
    int typenames; //1 for nounce, 2 for hash rate
}message;

Mail<message, 16> mail_box;
//Thread compute_thread;
Thread print_thread;
Thread decode_thread(osPriorityNormal,1024);
Thread motorCtrlT(osPriorityHigh,1024);

Mutex newkey_mutex;
Mutex newtune_mutex;
uint64_t newkey = 0;
Mail<uint8_t, 8> inCharQ;
volatile float current_velocity =0;
volatile float target_vel;
volatile float target_rot;
volatile float y_velocity=0;
volatile float y_rotation=0;
volatile bool vel_enter = false; 
volatile bool val_enter = false; 
volatile bool tune_enter = false;
Timeout time_tune;
RawSerial pc(SERIAL_TX, SERIAL_RX);
//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

DigitalOut TP1(TP1pin);
PwmOut MotorPWM(PWMpin);

void motorCtrlTick(){ 
     motorCtrlT.signal_set(0x1); 
}

float new_rot;
float old_rot;
float tmp_rot = 0;
float Er = 0; 
float d_Er = 0;
float p_Er = 0;
const float kpr=0.01;
const float kdr=0.05;

float Es = 0;
float integral_Es = 0; 
const float kps=0.01;
const float kis=0.0035;
const float integral_Es_Max = 800.0;

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}


//Set a given drive state
void motorOut(int8_t driveState){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L=1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04)  L2L=1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L=1;
    if (driveOut & 0x20) L3H = 0;
}



void pos_control(){ 

    if(target_rot == 0){
        y_rotation = 0;
    }
    else{
        if (target_rot <= 0){
            lead = -2;  
        }
        else{
            lead = 2;   
        }

        Er = abs(target_rot) - abs(new_rot - tmp_rot);
        d_Er = Er - p_Er;
        y_rotation = kpr * Er + kdr * d_Er;
        p_Er = Er; 

        if(y_rotation < 0){
            lead = -1*lead; 
            y_rotation = 0;
            }
        
        if(y_rotation>YVELMAX){
            y_rotation = YVELMAX;
        }
    }
}

void vel_control(){

    if(target_vel == 0){
        y_velocity = 0;
    }
    else{

        Es = abs(target_vel) - abs(current_velocity);

        integral_Es = integral_Es + Es * 0.1; 

        if(integral_Es > integral_Es_Max){
            integral_Es = 800;
        }
        if(integral_Es < -integral_Es_Max){
            integral_Es = -800;
        }
        y_velocity = (kps*Es+kis*integral_Es);

        if(y_velocity < 0){
            lead = -1*lead; 
            y_velocity = abs(y_velocity);
        }

        if(y_velocity>YVELMAX){
            y_velocity = YVELMAX;
        }
    }
}

void melody_tune_control(){
    uint8_t reading = tuneTable[tune_idx];
    uint8_t duration = reading>>4;

    MotorPWM.period_us(tuneTable[reading&0xf]);

    time_tune.attach_us(&melody_tune_control, 125000*duration);
    if (duration==0 || tune_idx==15 ){
        tune_idx = 0;
    }else{
        tune_idx++;
    }
}


void motorCtrlFn(){
    Ticker motorCtrlTicker; 
    motorCtrlTicker.attach_us(&motorCtrlTick,100000); 
    old_rot=0;
    new_rot=0;
    int32_t vel_counter=0;
    Timer test;
    float exe_time;

    while(1){
        motorCtrlT.signal_wait(0x1); // wait for thread signal
        // test.start();
        new_rot = (float)motorPosition/6.0;
        current_velocity = (float)(new_rot-old_rot)*10; //100ms per call means 0.1s per call
        old_rot = new_rot;
        vel_counter++;

        pos_control();
        vel_control();

        pc.printf("current velocity %f, Er %f, y velocity %f, y rotation %f\n\r", current_velocity, Er, y_velocity, y_rotation);
        if(val_enter && !vel_enter){
            motorOut((readRotorState()-orState+lead+6)%6);
            //每次input新的rotation之前都drive一遍motor
            MotorPWM.write(y_rotation);
        }
        // else if(vel_enter && !val_enter){
        //     MotorPWM.write(y_velocity);
        // }
        else if(vel_enter && val_enter){
            motorOut((readRotorState()-orState+lead+6)%6);
            //always take the minimum
            if(y_velocity<y_rotation){
                    MotorPWM.write(y_velocity);
            }
            else{
                    MotorPWM.write(y_rotation);
            }
            // test.stop();
            // exe_time = test.read();
            // pc.printf("Execution time: %f\n\r",exe_time);
            // test.reset();
        }

        if (tune_enter){
            // test.start();
            tune_idx = 0;
            newtune_mutex.lock();
            melody_tune_control();
            newtune_mutex.unlock();
            // test.stop();
            // exe_time = test.read();
            // pc.printf("Execution time: %f\n\r",exe_time);
            // test.reset();
            }

        if (vel_counter == 9){
            message *mail = mail_box.alloc();
            mail->velocity = current_velocity;
            mail->typenames = 4;
            mail_box.put(mail);
            vel_counter = 0;
            }
    }
}


//Basic synchronisation routine
int8_t motorHome(){
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    return readRotorState();
}


// ISR to handle the updating of the motor position
void motorISR(){
    static int8_t oldRotorState = 0;
    int8_t rotorState = readRotorState();

    int8_t tmpState = (rotorState-orState+lead+6)%6;

    motorOut(tmpState);
    if (rotorState == 4 && oldRotorState == 3) TP1 = !TP1;
    if(rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++;
    else motorPosition += (rotorState - oldRotorState);
    oldRotorState = rotorState;
}

void CheckState(){
        I1.rise(&motorISR);
        I2.rise(&motorISR);
        I3.rise(&motorISR);
        I1.fall(&motorISR);
        I2.fall(&motorISR);
        I3.fall(&motorISR);
}

void CountHash(){
  message *mail = mail_box.alloc();
  mail->data = hash_counter;
  mail->typenames = 2;
  mail_box.put(mail);
  hash_counter = 0;
}


void myprint(){
    while (true){
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            message *mail = (message*)evt.value.p;

            switch(mail->typenames){
                // Type 1: Found Hash, Print nonce
                case(1):
//                    pc.printf("\n Found Hash_nonce: 0x%x\n\r", mail->data);
                    break;
                // Type 2: Hash rate    
                case(2):
//                    pc.printf("\n Hash-rate: %d\n\r", mail->data);
                    break;
                // Type 3: Abs positon of rotor 
                case(3):
//                    pc.printf("\n Position: %d\n\r", mail->data);
                    break;
                case(4):
//                    pc.printf("\n Velocity: %f\n\r", mail->velocity);
                    break;
                case(5):
//                    pc.printf("\n key found: %llx\n\r", mail->key);
                    break;
            }
        mail_box.free(mail);
        }
    }
}



void compute(){
    uint8_t hash1[32];
    SHA256::computeHash(hash1, sequence, 64);
    if ((hash1[0]==0) && (hash1[1]==0)){
            message *mail = mail_box.alloc();
            mail->data = *nonce;
            mail->typenames = 1;
            mail_box.put(mail);
    }

}

void serialISR(){
    uint8_t* newChar = inCharQ.alloc();
    *newChar = pc.getc(); 
    inCharQ.put(newChar);
}

void dumpMes(int type, uint64_t datain){
    newkey_mutex.lock();
    message *mail = mail_box.alloc();
    mail->key = datain;
    mail->typenames = type;
    mail_box.put(mail);
    newkey_mutex.unlock();
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

void posprint(){
        message *mail = mail_box.alloc();
        mail->data = motorPosition;
        mail->typenames = 3;
        mail_box.put(mail);
}

// void measurement(){
//     Timer test;
//     float exe_time;
//     test.start();
//     float start=test.read();
//     for(int i=0; i<4999; i++){
//         compute(); 
//         *nonce = *nonce + 1;
//         hash_counter = hash_counter + 1;
//     }
//     exe_time = test.read()-start;
//     pc.printf("Execution time: %f\n\r",exe_time);
// }


//Main
int main() {

    Ticker timing;
    Ticker position;
    MotorPWM.period(0.002f);
    MotorPWM.write(1.0f); //设置motor为最快转速  

    //Initialise the serial port
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();
    //motorHome里面会有一次motorOut把motor每次reset之后都放在“state0”里面，
    //每个motor的“state0“都不同，我们的是4，
    //wait两秒等motor到达它的位置，否则会直接走interrupt然后motor就一直转了

    pc.printf("Rotor origin: %x\n\r",orState);

    CheckState();
    //motor到了位置之后，走interrupt

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

