#include "messages.h"

Mail<message, 16> mail_box;

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
        mail_box.free(mail);
        }
    }
}

