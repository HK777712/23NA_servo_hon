#include "mbed.h"
#include <cstdint>


PwmOut servo_R(PA_7);//migi oku
PwmOut servo_L(PB_1); //hidari_serovo temae


RawCAN      can_robo( PB_12,  PB_13, 1000000); //robomas
RawCAN      can_main( PA_11,PA_12, 1000000); //maincan

DigitalOut sig(D13);

CircularBuffer<CANMessage, 32> queue;
CircularBuffer<CANMessage, 32> queue2; //main_can

int motor_torqu[2] = {0};

struct C610Data{
    unsigned ID;
    int32_t counts;
    int32_t rpm;
    int32_t current;
    int32_t targetRPM;
    int32_t averageRPM;
    int16_t torqu;
};

struct C610Data M_yoko;
struct C610Data M_tate;



char servo_mode = 0xff;
char kubi_mode[2] = {0x80,0x80}; //yoko, tate

uint16_t kubi_send_data[2];


void canListen(){
    CANMessage Rcvmsg;
    if (can_robo.read(Rcvmsg)){
        queue.push(Rcvmsg);
    }
}

void canmainListen() {
    CANMessage mainmsg;

    if (can_main.read(mainmsg)) {
        queue2.push(mainmsg);
    }
}

void datachange(unsigned ID, struct C610Data *C610, CANMessage *msg){
    if(ID == msg->id){
        C610->counts = uint16_t((msg->data[0] << 8) | msg->data[1]);
        C610->rpm = int16_t((msg->data[2] << 8) | msg->data[3]);
        C610->current = int16_t((msg->data[4] << 8) | msg->data[5]);
        // printf("%d %d %d\n",C610->counts, C610->rpm, C610->current);
    }
}

void TorqueToBytes(uint16_t torqu, unsigned char *upper, unsigned char *lower){
    *upper = (torqu >> 8) & 0xFF;
    *lower = torqu & 0XFF;
}

void sendData(const int32_t torqu0, const int32_t torqu1){
    int16_t t0,t1;
    if(torqu0>32000){
        t0 = 32000;
    }else if(torqu0<-32000){
        t0 = -32000;
    }else{
        t0 = torqu0;
    }
    if(torqu1>32000){
        t1 = 32000;
    }else if(torqu1<-32000){
        t1 = -32000;
    }else{
        t1 = torqu1;
    }

    CANMessage msg;
    msg.id = 0x200;
    TorqueToBytes(t0, &msg.data[0], &msg.data[1]);
    TorqueToBytes(t1, &msg.data[2], &msg.data[3]);
    for(int i=4; i<8; i++){
        msg.data[i] = 0x00;
    }
    can_robo.write(msg);
}

// main() runs in its own thread in the OS
int main()
{
    sig = 0;
    servo_R.pulsewidth_us(2500);
    servo_L.pulsewidth_us(500);

    
    can_main.mode(CAN::Normal);
    can_main.attach(&canmainListen, CAN::RxIrq);
    can_robo.attach(&canListen, CAN::RxIrq);

    M_yoko.ID = 0x201;
    M_tate.ID = 0x202;

    uint32_t counter=0;

    CANMessage Rxmsg;
    CANMessage servomsg;
    while (true) {
        while(!queue.empty()){
            queue.pop(Rxmsg);
            datachange(M_yoko.ID, &M_yoko, &Rxmsg);
            datachange(M_tate.ID, &M_tate, &Rxmsg);
        }
        while(!queue2.empty()){
            queue2.pop(servomsg);
            if(servomsg.id == 0x010){
                servo_mode = servomsg.data[4];
            }else if(servomsg.id == 0x000){
                kubi_mode[0] = servomsg.data[0];
                kubi_mode[1] = servomsg.data[1];
            }
        }
        if(servo_mode == 0xff){ //agaru
            servo_R.pulsewidth_us(1100);
            servo_L.pulsewidth_us(1900);

        }else if(servo_mode == 0x00){ //sagaru
            servo_R.pulsewidth_us(2500);
            servo_L.pulsewidth_us(500); //period
        }
        if(kubi_mode[0] == 0xff){
            kubi_send_data[0] = 5000;
        }else if(kubi_mode[0] == 0x00){
            kubi_send_data[0] = -1000;
        }else if(kubi_mode[0] == 0x80){
            kubi_send_data[0] = 0;
        }
        if(kubi_mode[1] == 0xff){
            kubi_send_data[1] = 2000;
        }else if(kubi_mode[1] == 0x00){
            kubi_send_data[1] = -2000;
        }else if(kubi_mode[1] == 0x80){
            kubi_send_data[1] = 0;
        }
        sendData(kubi_send_data[1], kubi_send_data[0]);
        if(counter > 100){
            printf("%x,  ",servo_mode);
            printf("%d,  ",kubi_send_data[0]);
            printf("%d\n",kubi_send_data[1]);
            counter = 0;
        }else{
            counter++;
        }
        ThisThread::sleep_for(1ms);
        
    }
}
