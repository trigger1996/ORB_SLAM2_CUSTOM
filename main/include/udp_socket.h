#ifndef __UDP_SOCKET_ORBSLAM_H
#define __UDP_SOCKET_ORBSLAM_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

#include <pthread.h>

#define SERVER_PORT 8888
#define SERVER_IP "127.0.0.1"

#define Tx_Len 256

class __udp_socket {
public:
    __udp_socket() {
        memset(Tx, 0, sizeof(char) * Tx_Len);
        fd = -1;

        roll = pitch = yaw = 0.;
        x    = y     = z   = 0.;

    }

    void set_Attitude(double roll_in, double pitch_in, double yaw_in) {
        roll  = roll_in;
        pitch = pitch_in;
        yaw   = yaw_in;
    }

    void set_PosVector(double x_in, double y_in, double z_in) {
        x = x_in;
        y = y_in;
        z = z_in;
    }

    int init();

    int send_Data();

    int send_Data(double roll_in, double pitch_in, double yaw_in,
                  double x_in,    double y_in,     double z_in);

private:

    // UDP
    double roll, pitch, yaw;
    double x,    y,     z;

    char Tx[Tx_Len];

    int fd;
    struct sockaddr_in serv_addr;

    // 发送线程
    pthread_t udp_sender;

};

extern __udp_socket skt;


#endif	// __UDP_SOCKET_ORBSLAM_H
