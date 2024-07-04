#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <fstream>
#include <sys/time.h>

#include <chrono>
#include <ctime>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

//#include <common/mavlink.h>
//#include "mavlink/common/mavlink.h"
#include <iostream>
// to use the std:min function
#include <algorithm>
#include "pi-protocol.h"
#include "pi-messages.h"

#include <stdlib.h>
#include <arpa/inet.h>

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

// functions for writing IMU message to csv file
FILE* createImuLogFile() {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[100];
    sprintf(filename, "/home/pi/logs/imu-%d-%d-%d-%d-%d-%d.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    // convert to std::string
    std::string filename_str(filename);
    // if /home/pi/logs doesnt exists we make it
    std::ifstream file_stream(filename_str.c_str());

    if (!file_stream.good()) {
        system("mkdir -p /home/pi/logs");
    }
    
    FILE* file = fopen(filename, "w");
    printf("Logging IMU data to %s\n", filename);
    return file;
}

void writeImuToCsvHeader(FILE* file) {
    // time_ms_rec is the time the message was received
    // time_ms is the time the message was created
    fprintf(file, "time_ms_rec,time_ms_msg,roll,pitch,yaw,x,y,z\n");
}

void writeImuToCsv(pi_IMU_t* imuMsg, FILE* file) {
    // get current time ms
    auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    
    uint32_t time_ms_rec = (uint32_t)millisec_since_epoch;
    fprintf(file, "%u,%u,%f,%f,%f,%f,%f,%f\n", time_ms_rec, imuMsg->time_ms, imuMsg->roll, imuMsg->pitch, imuMsg->yaw, imuMsg->x, imuMsg->y, imuMsg->z);
}

int main(int argc, char** argv) {
    const char* serialPort = "/dev/ttyAMA0";
    //int baudrate = B57600;
    //const char* serialPort = "/dev/ttyS0";
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

#define OPTITRACK_BUFFER_SIZE (PI_MSG_FAKE_GPS_PAYLOAD_LEN + PI_MSG_EXTERNAL_POSE_PAYLOAD_LEN)
    int optitrackFd = openUdpPort(OPTITRACK_PORT);
    printf("Server listening on port %d for Optitrack...\n", OPTITRACK_PORT);
    uint8_t optitrackBuffer[OPTITRACK_BUFFER_SIZE];

    int setpointFd = openUdpPort(SETPOINT_PORT);
    printf("Server listening on port %d for Setpoints...\n", SETPOINT_PORT);
    uint8_t setpointBuffer[PI_MSG_POS_SETPOINT_PAYLOAD_LEN];

    // create imu log file and write to header
    //FILE* imuLogFile = createImuLogFile();
    //writeImuToCsvHeader(imuLogFile);

    while (true) {
        // Q1: doesnt this add a lot of delay? Because the buffer is only filled once
        // PI_MAX_PACKET_LENGTH is read?

        // Q2: Is the UART TX buffer even big enough?
        // answer, seems like the PL011 buffer is 32byte deep. termios wraps it
        // in a PAGE_SIZE deep buffer (4096bytes), so this is fine at least
        ssize_t numBytes = read(serialPortFd, buffer, PI_MAX_PACKET_LEN);
        //uint8_t res;
        if (numBytes) {
            for (int i=0; i < numBytes; i++){
                //res = piParse(buffer[i]);
                piParse(buffer[i]);

                //if (res == PI_MSG_IMU_ID) {
                //    // write to csv
                //    // if message parse successful then write to csv
                //    writeImuToCsv(piMsgImuRx, imuLogFile);
                //}
            }
        }



        pi_FAKE_GPS_t msgFakeGps;
        pi_EXTERNAL_POSE_t msgExternalPose;
#define PI_MSG_PAYLOAD_OFFSET (PI_MSG_ID_BYTES + PI_MSG_PAYLOAD_LEN_BYTES)
        //int bytes_received = recvfrom(sockfd, (char*)(piMsgFakeGpsRx)+PI_MSG_PAYLOAD_OFFSET, PI_MSG_FAKE_GPS_PAYLOAD_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int bytes_received = recvfrom(optitrackFd, (uint8_t *)(optitrackBuffer), OPTITRACK_BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &client_addr_len);
        memcpy((uint8_t *)(&msgFakeGps)+PI_MSG_PAYLOAD_OFFSET, optitrackBuffer, PI_MSG_FAKE_GPS_PAYLOAD_LEN);
        memcpy((uint8_t *)(&msgExternalPose)+PI_MSG_PAYLOAD_OFFSET, optitrackBuffer+PI_MSG_FAKE_GPS_PAYLOAD_LEN, PI_MSG_EXTERNAL_POSE_PAYLOAD_LEN);
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
            piMsgFakeGpsTx.time_ms = piMsgImuRx->time_ms;
            piMsgFakeGpsTx.lat = ntohl(msgFakeGps.lat);
            piMsgFakeGpsTx.lon = ntohl(msgFakeGps.lon);
            piMsgFakeGpsTx.altCm = ntohl(msgFakeGps.altCm);
            piMsgFakeGpsTx.hdop =  ntohs(msgFakeGps.hdop);
            piMsgFakeGpsTx.groundSpeed = ntohs(msgFakeGps.groundSpeed);
            piMsgFakeGpsTx.groundCourse = ntohs(msgFakeGps.groundCourse);
            piMsgFakeGpsTx.numSat = msgFakeGps.numSat;
            piSendMsg(&piMsgFakeGpsTx, &serialWriter);

            piMsgExternalPoseTx.time_ms = piMsgImuRx->time_ms;
            piMsgExternalPoseTx.enu_x = ntohf(msgExternalPose.enu_x);
            piMsgExternalPoseTx.enu_y = ntohf(msgExternalPose.enu_y);
            piMsgExternalPoseTx.enu_z = ntohf(msgExternalPose.enu_z);
            piMsgExternalPoseTx.enu_xd = ntohf(msgExternalPose.enu_xd);
            piMsgExternalPoseTx.enu_yd = ntohf(msgExternalPose.enu_yd);
            piMsgExternalPoseTx.enu_zd = ntohf(msgExternalPose.enu_zd);
            piMsgExternalPoseTx.body_qi = ntohf(msgExternalPose.body_qi);
            piMsgExternalPoseTx.body_qx = ntohf(msgExternalPose.body_qx);
            piMsgExternalPoseTx.body_qy = ntohf(msgExternalPose.body_qy);
            piMsgExternalPoseTx.body_qz = ntohf(msgExternalPose.body_qz);
            piSendMsg(&piMsgExternalPoseTx, &serialWriter);
        }


        // ---- setpoints ----
        pi_POS_SETPOINT_t msgPosSetpoint;
        bytes_received = recvfrom(setpointFd, (uint8_t *)(setpointBuffer), PI_MSG_POS_SETPOINT_PAYLOAD_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);
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
    //fclose(imuLogFile);

    return 0;
}
