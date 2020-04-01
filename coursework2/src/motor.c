#include "motor.h"

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


int counter = 0;
bool movement_dir;

volatile float current_velocity =0;
volatile float y_velocity=0;
volatile float y_rotation=0;
volatile int32_t motorPosition=0; 

float new_rot;
float old_rot;
float tmp_rot = 0;
float Er = 0; 
float d_Er = 0;
float p_Er = 0;
const float kpr=0.006;
const float kdr=0.02;

float Es = 0;
float integral_Es = 0; 
const float kps=0.05;
const float kis=0.00095;
const float integral_Es_Max = 800.0;

Timeout time_tune;

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
            y_rotation = -y_rotation; 
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

        integral_Es = integral_Es + Es / 0.1; 

        if(integral_Es > integral_Es_Max){
            integral_Es = 800;
        }
        if(integral_Es < -integral_Es_Max){
            integral_Es = -800;
        }
        y_velocity = (kps*Es+kis*integral_Es);

        if(y_velocity < 0){
            y_velocity = 0;
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
    float avg=0;

    while(1){
        motorCtrlT.signal_wait(0x1); // wait for thread signal
        // test.start();
        new_rot = (float)motorPosition/6.0;
        current_velocity = (float)(new_rot-old_rot)*10; //100ms per call means 0.1s per call
        old_rot = new_rot;
        vel_counter++;

        pos_control();
        vel_control();
        avg=avg+current_velocity; 
        
        //  pc.printf("current velocity %f, Er %f, y velocity %f, y rotation %f\n\r", current_velocity, Er, y_velocity, y_rotation);
        if(val_enter && !vel_enter){
            motorOut((readRotorState()-orState+lead+6)%6);
            MotorPWM.write(y_rotation);
        }
        // else if(vel_enter && !val_enter){
        //     MotorPWM.write(y_velocity);
        // }
        else if(vel_enter && val_enter){
            motorOut((readRotorState()-orState+lead+6)%6);
            //always take the minimum

            // if(y_velocity<y_rotation){
            //         MotorPWM.write(y_velocity);
            // }
            // else{
            //         MotorPWM.write(y_rotation);
            // }
            if(Er >= 0){
                if(y_velocity>y_rotation){
                     if (y_rotation < 0.5) { // stop early and avoid undershooting
                        MotorPWM.write(0.5f);
                    }
                    else{
                        MotorPWM.write(y_rotation);
                    }

                    if ((abs(p_Er) <= 0.5) && (p_Er == Er)) {
                        MotorPWM.write(0);
                    }
                }
                else{
                        MotorPWM.write(y_velocity);
                }
            }
            else{
                if(y_velocity<y_rotation){
                    if (y_rotation < 0.5) { // avoid overshooting
                        MotorPWM.write(0.5f);
                    }
                    else{
                        MotorPWM.write(y_rotation);
                    }

                    if ((abs(p_Er) <= 0.5) && (p_Er == Er)) {
                        MotorPWM.write(0);
                    }
                }
                else{
                        MotorPWM.write(y_velocity);
                }
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
            mail->velocity = avg/9;
            mail->typenames = 4;
            mail_box.put(mail);
            vel_counter = 0;
            avg = 0; 
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

void posprint(){
        message *mail = mail_box.alloc();
        mail->data = motorPosition;
        mail->typenames = 3;
        mail_box.put(mail);
}