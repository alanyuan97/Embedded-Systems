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
const int8_t kp=25;
const int8_t kd=20;
const uint32_t YVELMAX=2000;
//Phase lead to make motor spin
int8_t lead = -2;  //2 for forwards, -2 for backwards
int8_t orState = 0;
volatile uint32_t hash_counter=0;
volatile int32_t motorPosition=0; 

uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64, 0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73, 0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E, 0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20, 0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20, 0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key = (uint64_t*)&sequence[48];
uint64_t* nonce = (uint64_t*)&sequence[56];
char infobuffer[18];
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
Thread motorCtrlT(osPriorityNormal,1024);

Mutex newkey_mutex;
uint64_t newkey = 0;
Mail<uint8_t, 8> inCharQ;
volatile double current_velocity =0;
double target_vel=30;
double target_rot=0;
volatile double y_velocity=0;
volatile double y_rotation=0;


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

void motorCtrlFn(){
    Ticker motorCtrlTicker; 
    motorCtrlTicker.attach_us(&motorCtrlTick,100000); 
    double oldpos=0;
    double newpos=0;
    double Er = 0; 
    double d_Er = 0;
    double p_Er = 0; 
    int32_t vel_counter=0;



    while(1){
        motorCtrlT.signal_wait(0x1); // wait for thread signal
        newpos = (double)motorPosition/6;
        current_velocity += (double)(newpos-oldpos); //unit in radians. times 10 because 100ms per call means 0.1s per call
        oldpos = newpos;
        vel_counter++;

        if (vel_counter == 10){
            if(target_vel == 0){
                y_velocity = YVELMAX;
            }
            else{
                if (target_vel <= 0){
                    lead = -2;   
                }
                else{
                    lead = 2;   
                }

            y_velocity = kp*(target_vel-abs(current_velocity));
            MotorPWM.write(y_velocity/0.002);

             
            message *mail = mail_box.alloc();
            mail->velocity = current_velocity;
            mail->typenames = 4;
            mail_box.put(mail);
            current_velocity = 0;
            vel_counter = 0;
            }

            Er = target_rot - newpos;
            d_Er = Er - p_Er;
            y_rotation = kp * Er + kd * d_Er;
            p_Er = Er; 

        }
    }
}




//Set a given drive state
void motorOut(int8_t driveState, double y_velocity1){

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

    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine
int8_t motorHome(){
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0,1000);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}


// ISR to handle the updating of the motor position
void motorISR(){
    static int8_t oldRotorState = 0;
    int8_t rotorState = readRotorState();

    int8_t tmpState = (rotorState-orState+lead+6)%6;

    motorOut(tmpState,y_velocity);
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
                    pc.printf("\n Found Hash_nonce: 0x%x\n\r", mail->data);
                    break;
                // Type 2: Hash rate    
                case(2):
                    pc.printf("\n Hash-rate: %d\n\r", mail->data);
                    break;
                // Type 3: Abs positon of rotor 
                case(3):
                    pc.printf("\n Position: %d\n\r", mail->data);
                    break;
                case(4):
                    pc.printf("\n Velocity: %f\n\r", mail->velocity);
                    break;
                case(5):
                    pc.printf("\n key found: %llx\n\r", mail->key);
                    break;
            }
            // pc.printf("\ntype: 0x%d\n\r", mail->typenames);
            // pc.printf("\ndata: 0x%d\n\r", mail->data);

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

void decode(){
    pc.attach(&serialISR); 
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t *newChar = (uint8_t*)newEvent.value.p; 

        if(*newChar!='\r'){
            infobuffer[counter] = *newChar;
            counter>=17 ? counter =0: counter++;
        // continue
        //    pc.printf("\n key: %c\n\r", infobuffer[counter]);
        //    pc.printf("counter %d\n\r", counter);
        }

        else{
            infobuffer[counter] =  '\0';
            switch(infobuffer[0]){
                // Rotation numbers
                case('R'):
                    sscanf(infobuffer,"R%f",&target_rot);
                    // dump the target TODO
                    break;
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
                    sscanf(infobuffer,"V%f",&target_vel);
                    // dump the target TODO
                    break;
            }
            counter = 0;
        }       
        inCharQ.free(newChar);
    }
}

void posprint (){
        message *mail = mail_box.alloc();
        mail->data = motorPosition;
        mail->typenames = 3;
        mail_box.put(mail);
}

//Main
int main() {
    // int8_t orState = 0;    //Rotot offset at motor state 0
    // int8_t intState = 0;
    // int8_t intStateOld = 0;

    Ticker timing;
    Ticker position;

    const int32_t PWM_PRD = 2000;
    MotorPWM.period_us(PWM_PRD);
    MotorPWM.pulsewidth_us(PWM_PRD);

    //Initialise the serial port
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    MotorPWM.pulsewidth_us(PWM_PRD/2);
    //Poll the rotor state and set the motor outputs accordingly to spin the motor

    timing.attach(&CountHash, 10.0);
    position.attach (&posprint,2);

    CheckState();
    //compute_thread.start(callback(compute));
    print_thread.start(callback(myprint));
    decode_thread.start(callback(decode));
    motorCtrlT.start(callback(motorCtrlFn));
    
    while (1) {
        // Protect key assignment from thread interrupt
        newkey_mutex.lock();
        *key = newkey;
        newkey_mutex.unlock();
        // Compute Hash
        compute();
        *nonce = *nonce + 1;
        hash_counter+=1;
        //decode(); 
    }
}
