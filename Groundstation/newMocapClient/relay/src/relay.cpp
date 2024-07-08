#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
//#include <common/mavlink.h>
//#include "mavlink/common/mavlink.h"
#include <iostream>
// to use the std:min function
#include <algorithm>
#include "pi-protocol.h"
#include "pi-messages.h"

#include <stdlib.h>
#include <arpa/inet.h>
#include <math.h>

#define OPTITRACK_PORT 5005
#define SETPOINT_PORT 5006
#define MAX_BUFFER_SIZE 1024

// hypersimple on-demand status updates similar to dd
// https://en.wikipedia.org/wiki/C_signal_handling
// idea: when sending SIGUSR1 -- print messages
//       when sending SIGUSR2 -- print statistics
// TODO: fork this program into two threads so that the main program isn't 
//       interrupted for stats/msgs. Neglect membar's, because we dont care if 
//       sometimes messages are garbled during the print and this hopefully 
//       wont occur too often
#include <signal.h>
#include <stdlib.h>

static void catch_function(int signo) {
    switch(signo) {
        case SIGUSR1:
            piPrintMsgs(&printf);
            break;
        case SIGUSR2:
            piPrintStats(&printf);
            break;
    }
}

int openSerialPort(const char* port, int baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("openSerialPort: Unable to open port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    options.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

int openUdpPort(const uint16_t port) {
    int sockfd;
    struct sockaddr_in server_addr;

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Make socket non-blocking
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    memset(&server_addr, 0, sizeof(server_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    // Bind socket
    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    return sockfd;
}

int serialPortFd; // needs to be global to be used inside serialWriter

void serialWriter(uint8_t byte)
{
    //printf("%d", serialPortFd);
    write(serialPortFd, &byte, 1);
}

float ntohf(float in) {
    uint32_t dummy;
    dummy = ntohl(*(uint32_t*)&(in));
    return *(float*)(&dummy);
}

typedef struct pose_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
} pose_t;

typedef struct pose_der_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float wx;
    float wy;
    float wz;
} pose_der_t;

static pi_parse_states_t piParseStates = {0};

int main(int argc, char** argv) {
    const char* serialPort = "/dev/ttyAMA0";
    //int baudrate = B57600;
    //const char* serialPort = "/tmp/ttyS2";
    //int baudrate = B500000; //todo: make commandline argument
    int baudrate = B921600; //todo: make commandline argument

    serialPortFd = openSerialPort(serialPort, baudrate);
    if (serialPortFd == -1) {
        return 1;
    }

    // register signal handlers (https://en.wikipedia.org/wiki/C_signal_handling)
    if (signal(SIGUSR1, catch_function) == SIG_ERR) {
        fputs("An error occurred while setting a signal handler.\n", stderr);
        return EXIT_FAILURE;
    }
    if (signal(SIGUSR2, catch_function) == SIG_ERR) {
        fputs("An error occurred while setting a signal handler.\n", stderr);
        return EXIT_FAILURE;
    }

    // TODO: make sure PI_MAX_PACKET_LEN is less than PAGE_SIZE (4096)
    uint8_t buffer[PI_MAX_PACKET_LEN];

    // open udps
    int optitrackFd = openUdpPort(OPTITRACK_PORT);
    printf("Server listening on port %d for Optitrack...\n", OPTITRACK_PORT);
    static constexpr size_t OPTITRACK_BUFFER_SIZE = sizeof(unsigned int) + sizeof(pose_t) + sizeof(pose_der_t);
    uint8_t optitrackBuffer[OPTITRACK_BUFFER_SIZE];

    int setpointFd = openUdpPort(SETPOINT_PORT);
    printf("Server listening on port %d for Setpoints...\n", SETPOINT_PORT);
    uint8_t setpointBuffer[PI_MSG_POS_SETPOINT_PAYLOAD_LEN];

    while (true) {
        // Q1: doesnt this add a lot of delay? Because the buffer is only filled once
        // PI_MAX_PACKET_LENGTH is read?

        // Q2: Is the UART TX buffer even big enough?
        // answer, seems like the PL011 buffer is 32byte deep. termios wraps it
        // in a PAGE_SIZE deep buffer (4096bytes), so this is fine at least
        ssize_t numBytes = read(serialPortFd, buffer, PI_MAX_PACKET_LEN);
        bool newMessage = false;
        if (numBytes) {
            for (int i=0; i < numBytes; i++)
                if (piParse(&piParseStates, buffer[i]) == PI_MSG_IMU_ID)
                    newMessage = true;
        }

        if ((piMsgImuRxState == PI_MSG_RX_STATE_NONE) || (!newMessage)) {
            // cannot go on, no time information to timestamp gps msgs, or setpoints
            usleep(500); // sleep 0.5ms, this is bound to miss some gyro messages, but what gives
            continue;
        }

        newMessage = false;

        pose_t pose;
        pose_der_t pose_der;

        //int bytes_received = recvfrom(sockfd, (char*)(piMsgFakeGpsRx)+PI_MSG_PAYLOAD_OFFSET, PI_MSG_FAKE_GPS_PAYLOAD_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int bytes_received = recvfrom(optitrackFd, (uint8_t *)(optitrackBuffer), OPTITRACK_BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &client_addr_len);
        memcpy((uint8_t *)(&pose), optitrackBuffer+sizeof(unsigned int), sizeof(pose_t));
        memcpy((uint8_t *)(&pose_der), optitrackBuffer+sizeof(unsigned int)+sizeof(pose_t), sizeof(pose_der_t));

        if (bytes_received > 0) {
            //pose.timeUs = __builtin_bswap64(pose.timeUs);
            //printf("Received optitrack for time %ld at IMU time %d\n", pose.timeUs, piMsgImuRx->time_ms);
            /*
            pose.x = ntohf(pose.x);
            pose.y = ntohf(pose.y);
            pose.z = ntohf(pose.z);
            pose.qx = ntohf(pose.qx);
            pose.qy = ntohf(pose.qy);
            pose.qz = ntohf(pose.qz);
            pose.qw = ntohf(pose.qw);

            pose_der.x = ntohf(pose_der.x);
            pose_der.y = ntohf(pose_der.y);
            pose_der.z = ntohf(pose_der.z);
            pose_der.wx = ntohf(pose_der.wx);
            pose_der.wy = ntohf(pose_der.wy);
            pose_der.wz = ntohf(pose_der.wz);
            */

            piMsgFakeGpsTx.time_ms = piMsgImuRx->time_ms;
            static constexpr double CYBERZOO_LAT = 51.99071002805145;
            static constexpr double CYBERZOO_LON = 4.376727452462819;
            static constexpr double RE = 6378137.;
            piMsgFakeGpsTx.lat = (int) 1e7 * 
                (CYBERZOO_LAT + 180. / M_PI * (pose.y / RE));
            piMsgFakeGpsTx.lon = (int) 1e7 *
                (CYBERZOO_LON + 180 / M_PI * (pose.x / RE) / cos(CYBERZOO_LON * M_PI / 180.));
            piMsgFakeGpsTx.altCm = (int) (pose.z * 100.f);
            piMsgFakeGpsTx.hdop =  (short) 150;
            piMsgFakeGpsTx.groundSpeed = (short) (hypotf(pose_der.x, pose_der.y) * 100.f);
            piMsgFakeGpsTx.groundCourse = (short) (1800.f * atan2(pose_der.y, pose_der.x) / M_PI);
            piMsgFakeGpsTx.numSat = 8;
            piSendMsg(&piMsgFakeGpsTx, &serialWriter);

            piMsgExternalPoseTx.time_ms = piMsgImuRx->time_ms;
            piMsgExternalPoseTx.enu_x = pose.x;
            piMsgExternalPoseTx.enu_y = pose.y;
            piMsgExternalPoseTx.enu_z = pose.z;
            piMsgExternalPoseTx.enu_xd = pose_der.x;
            piMsgExternalPoseTx.enu_yd = pose_der.y;
            piMsgExternalPoseTx.enu_zd = pose_der.z;
            piMsgExternalPoseTx.body_qi = pose.qw;
            piMsgExternalPoseTx.body_qx = pose.qx;
            piMsgExternalPoseTx.body_qy = pose.qy;
            piMsgExternalPoseTx.body_qz = pose.qz;
            piSendMsg(&piMsgExternalPoseTx, &serialWriter);
        }
        //else {
            //printf("No optitrack bytes received at %ld at IMU time %d\n", pose.timeUs, piMsgImuRx->time_ms);
        //}

        // ---- setpoints ----
        pi_POS_SETPOINT_t msgPosSetpoint;
        bytes_received = recvfrom(setpointFd, (uint8_t *)(setpointBuffer), PI_MSG_POS_SETPOINT_PAYLOAD_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);
#define PI_MSG_PAYLOAD_OFFSET (PI_MSG_ID_BYTES + PI_MSG_PAYLOAD_LEN_BYTES)
        memcpy((uint8_t *)(&msgPosSetpoint)+PI_MSG_PAYLOAD_OFFSET, setpointBuffer, PI_MSG_POS_SETPOINT_PAYLOAD_LEN);
        //if (bytes_received == -1) {
        //    perror("recvfrom failed");
        //    exit(EXIT_FAILURE);
        //}

        //bool udpReceived = false;
        //if (bytes_received > 0) {
        //    printf
        //    for (int i=0; i < bytes_received; i++)
        //        updReceived |= (piParse(udpbuffer[i]) == PI_MSG_FAKE_GPS_ID);
        //}

        if (bytes_received > 0) {
            /*
            piPrintMsgs(&printf);
            printf("Hello: ");
            for (int j = 0; j < bytes_received; j++) {
                printf(" 0x%x", *(uint8_t*)(piMsgFakeGpsRx+PI_MSG_PAYLOAD_OFFSET+j));
            }
            */
            piMsgPosSetpointTx.time_ms = piMsgImuRx->time_ms;

            piMsgPosSetpointTx.enu_x = ntohf(msgPosSetpoint.enu_x);
            piMsgPosSetpointTx.enu_y = ntohf(msgPosSetpoint.enu_y);
            piMsgPosSetpointTx.enu_z = ntohf(msgPosSetpoint.enu_z);
            piMsgPosSetpointTx.enu_xd = ntohf(msgPosSetpoint.enu_xd);
            piMsgPosSetpointTx.enu_yd = ntohf(msgPosSetpoint.enu_yd);
            piMsgPosSetpointTx.enu_zd = ntohf(msgPosSetpoint.enu_zd);
            piMsgPosSetpointTx.yaw = ntohf(msgPosSetpoint.yaw);

            piSendMsg(&piMsgPosSetpointTx, &serialWriter);
        }
    }

    close(serialPortFd);
    close(optitrackFd);
    close(setpointFd);

    return 0;
}
