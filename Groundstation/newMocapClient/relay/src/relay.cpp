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
#define KEYBOARD_PORT 5007
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


#ifdef ORIN
// STUFF FOR ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <thread>
#include <chrono>

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex);

    // get time in ms
    static uint32_t start_s = msg->header.stamp.sec;
    // difference of first time and current time
    uint32_t diff_s = msg->header.stamp.sec - start_s;
    // diff to ms
    uint32_t time_ms = diff_s * 1000 + msg->header.stamp.nanosec / 1e6;

    // VIO_POSE MESSAGE
    piMsgVioPoseTx.time_ms = time_ms;
    piMsgVioPoseTx.x = msg->pose.pose.position.x;
    piMsgVioPoseTx.y = msg->pose.pose.position.y;
    piMsgVioPoseTx.z = msg->pose.pose.position.z;
    piMsgVioPoseTx.vx = msg->twist.twist.linear.x;
    piMsgVioPoseTx.vy = msg->twist.twist.linear.y;
    piMsgVioPoseTx.vz = msg->twist.twist.linear.z;
    piMsgVioPoseTx.qw = msg->pose.pose.orientation.w;
    piMsgVioPoseTx.qx = msg->pose.pose.orientation.x;
    piMsgVioPoseTx.qy = msg->pose.pose.orientation.y;
    piMsgVioPoseTx.qz = msg->pose.pose.orientation.z;
    piMsgVioPoseTx.p = msg->twist.twist.angular.x;
    piMsgVioPoseTx.q = msg->twist.twist.angular.y;
    piMsgVioPoseTx.r = msg->twist.twist.angular.z;

    // print content of piMsgVioPoseTx
    printf("VIO_POSE: time_ms: %u, x: %f, y: %f, z: %f, vx: %f, vy: %f, vz: %f, qw: %f, qx: %f, qy: %f, qz: %f\n", piMsgVioPoseTx.time_ms, piMsgVioPoseTx.x, piMsgVioPoseTx.y, piMsgVioPoseTx.z, piMsgVioPoseTx.vx, piMsgVioPoseTx.vy, piMsgVioPoseTx.vz, piMsgVioPoseTx.qw, piMsgVioPoseTx.qx, piMsgVioPoseTx.qy, piMsgVioPoseTx.qz);

    piSendMsg(&piMsgVioPoseTx, &serialWriter);
}
#endif


int main(int argc, char** argv) {
    // enable logging if the first argument is -l
    bool logging = (argc == 2) && !strcmp(argv[1], "-l"); 

#ifdef ORIN
    const char* serialPort = "/dev/ttyTHS1";
#else
    const char* serialPort = "/dev/ttyAMA0";
#endif
    int baudrate = B921600;

    serialPortFd = openSerialPort(serialPort, baudrate);
    if (serialPortFd == -1) {
        return EXIT_FAILURE;
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

    uint8_t piBuffer[PI_MAX_PACKET_LEN];

    // open udps
    int optitrackFd = openUdpPort(OPTITRACK_PORT);
    printf("Server listening on port %d for Optitrack...\n", OPTITRACK_PORT);
    static constexpr size_t OPTITRACK_BUFFER_SIZE = sizeof(unsigned int) + sizeof(pose_t) + sizeof(pose_der_t);
    uint8_t optitrackBuffer[OPTITRACK_BUFFER_SIZE];

    int setpointFd = openUdpPort(SETPOINT_PORT);
    printf("Server listening on port %d for Setpoints...\n", SETPOINT_PORT);
    uint8_t setpointBuffer[PI_MSG_POS_SETPOINT_PAYLOAD_LEN];

    // create imu log file and write to header
    FILE* imuLogFile = NULL;
    if (logging) {
        imuLogFile = createImuLogFile();
        writeImuToCsvHeader(imuLogFile);
    }

#ifdef ORIN
    // ROS2 stuff for subscribing to odometry messages from ZED camera
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("zed_odometry_listener");

    auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
        "/zed/zed_node/odom", 10, odomCallback);

    // Start a separate thread for ROS2 spinning
    std::thread ros_thread([&node]() {
        rclcpp::spin(node);
    });
    // --------------------------------------------------------------------
#endif

    while (true) {
        // Q1: doesnt this add a lot of delay? Because the buffer is only filled once
        // PI_MAX_PACKET_LENGTH is read?

        // Q2: Is the UART TX buffer even big enough?
        // answer, seems like the PL011 buffer is 32byte deep. termios wraps it
        // in a PAGE_SIZE deep buffer (4096bytes), so this is fine at least
        ssize_t numBytes = read(serialPortFd, piBuffer, PI_MAX_PACKET_LEN);
        bool newMessage = false;
        if (numBytes) {
            for (int i=0; i < numBytes; i++) {
                if (piParse(&piParseStates, piBuffer[i]) == PI_MSG_IMU_ID) {
                    newMessage = true;
                    if (logging) {
                        writeImuToCsv(piMsgImuRx, imuLogFile);
                    }
                }
            }
        }

        if ((piMsgImuRxState == PI_MSG_RX_STATE_NONE) || (!newMessage)) {
            // cannot go on, no time information to timestamp gps msgs, or setpoints
            usleep(250); // reduce CPU load a bit
            continue;
        } else {
            newMessage = false;
        }

        // 
        pose_t pose;
        pose_der_t pose_der;

        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int optitrackBytes = recvfrom(optitrackFd, (uint8_t *)(optitrackBuffer), OPTITRACK_BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (optitrackBytes > 0) {
            // we have optitrack copy from the buffer into the structs
            memcpy((uint8_t *)(&pose), optitrackBuffer+sizeof(unsigned int), sizeof(pose_t));
            memcpy((uint8_t *)(&pose_der), optitrackBuffer+sizeof(unsigned int)+sizeof(pose_t), sizeof(pose_der_t));

            // proceed to send Fake GPS and External Pose
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

        // ---- setpoints ----
        pi_POS_SETPOINT_t msgPosSetpoint;
        int setpointBytes = recvfrom(setpointFd, (uint8_t *)(setpointBuffer), PI_MSG_POS_SETPOINT_PAYLOAD_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (setpointBytes > 0) {
#define PI_MSG_PAYLOAD_OFFSET (PI_MSG_ID_BYTES + PI_MSG_PAYLOAD_LEN_BYTES)
            memcpy((uint8_t *)(&msgPosSetpoint)+PI_MSG_PAYLOAD_OFFSET, setpointBuffer, PI_MSG_POS_SETPOINT_PAYLOAD_LEN);
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

        // ---- keystrokes ----
        pi_POS_SETPOINT_t msgPosSetpoint;
        int setpointBytes = recvfrom(setpointFd, (uint8_t *)(setpointBuffer), PI_MSG_POS_SETPOINT_PAYLOAD_LEN, 0, (struct sockaddr *)&client_addr, &client_addr_len);

        if (setpointBytes > 0) {
#define PI_MSG_PAYLOAD_OFFSET (PI_MSG_ID_BYTES + PI_MSG_PAYLOAD_LEN_BYTES)
            memcpy((uint8_t *)(&msgPosSetpoint)+PI_MSG_PAYLOAD_OFFSET, setpointBuffer, PI_MSG_POS_SETPOINT_PAYLOAD_LEN);
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
