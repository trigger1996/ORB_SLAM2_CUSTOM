#include "include/udp_socket.h"

/**
 *  https://www.cnblogs.com/skyfsm/p/6287787.html?utm_source=itdadao&utm_medium=referral
 *
 */
__udp_socket skt;

void* udp_sender_func(void *arg) {

    while (true) {
/*
        if (skt.is_Updated()) {
            skt.send_Data();
            skt.set_Sent();
            usleep(1000);
        }
*/
    }

}

int __udp_socket::init() {

    int  pthread_stat = 0;
    char *args = NULL;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd < 0) {
        printf("create socket fail!\n");
        return -1;
    }

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVER_PORT);
    serv_addr.sin_addr.s_addr = inet_addr(SERVER_IP);


    //
    pthread_stat = pthread_create(&udp_sender, NULL, udp_sender_func, args);

    return pthread_stat;

}

int __udp_socket::send_Data() {


    sprintf(Tx, "roll: %f pitch: %f yaw: %f x: %f y: %f z: %f  ", roll, pitch, yaw, x, y, z);
    sendto(fd, Tx, strlen(Tx), 0, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    return 0;
}

int __udp_socket::send_Data(double roll_in, double pitch_in, double yaw_in,
                            double x_in,    double y_in,     double z_in) {

    set_Attitude(roll_in, pitch_in, yaw_in);
    set_PosVector(x_in, y_in, z_in);

    return send_Data();
}
