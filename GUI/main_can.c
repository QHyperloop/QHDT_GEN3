



#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>



// Sensor Request
#define MASTER_REQUEST 0x0000FF01 // 01 is first follower

// State extended IDs
#define INIT 0x00001000
#define FAULT 0x00001100
#define SAFE_TO_APPROACH 0x00001200
#define COAST 0x00001300
#define BRAKE 0x00001400
#define CRAWL 0x00001500
#define TRACK 0x00001600
#define LAUNCH 0x0000FF00
#define READY 0x00001700

unsigned int current_state;


#define FOLLOWERS 3;

// For storing confirm responses from sending states
int state_responses[FOLLOWERS][2];  

// For storing all the sensor data after master request
int sensors[FOLLOWERS][4][20];


int create_socket(const char* ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    int enable_canfd = 1;

    // Create a socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Error while opening socket");
        return -1;
    }

    // Specify interface name
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    // Bind socket to network interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -1;
    }

    // Enable BRS
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        perror("Error while enabling CAN FD for BRS");
        return -1;
    }

    return s;
}

// Function to send a state and wait for confirm
void send_state(int s, int current_state) {

    struct canfd_frame frame;

    if (current_state == READY){
        frame.can_id = current_state | CAN_EFF_FLAG;
        frame.can_dlc = 4;
        frame.data[0] = 0x00;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
    } else {
        frame.can_id = current_state | CAN_EFF_FLAG;
        frame.can_dlc = 1;
        frame.data[0] = 0x00;
    }

    if (write(s, &frame, sizeof(frame)) != sizeof(frame)){
        perror("Error in sending CAN frame");
    }

    read_state_responses(s);

}

void read_state_responses(int s) {

    struct canfd_frame frame_received;
    int responses_received = 0;
    int nbytes;

    printf("waiting for Can frame");

    while (responses_received < FOLLOWERS){

        nbytes = read(s, &frame_received, sizeof(struct can_frame));

        if (nbytes == sizeof(struct can_frame)) {

            printf("Recieved response\n");
            state_responses[responses_received][0] = frame_received.can_id;
            state_responses[responses_received][1] = frame_received.data[0];
            printf("Stored frame");

            responses_received++;

        }
    }   
}

// Master sensor request function
void master_request(int s) {

    unsigned int request = MASTER_REQUEST;

    struct canfd_frame frame;

    for (int i = 0; i < FOLLOWERS; i++){

        frame.can_id = request | CAN_EFF_FLAG; // extended id flag
        frame.can_dlc = 1;
        frame.data[0] = 0x00;
   
        request++; // increment to get all followers

        if (write(s, &frame, sizeof(frame)) != sizeof(frame)){
            perror("Error in sending CAN frame");
        }

        // logic to read incoming packets and update global array
        
        struct canfd_frame frame_received;
        int responses_received = 0;
        int nbytes;

        printf("waiting for Can frame");

       
        for (int j = 0; j < 4; j++ ){    

            nbytes = read(s, &frame_received, sizeof(struct can_frame));
            
            if (nbytes == sizeof(struct can_frame)) {

                printf("Recieved response\n");

                sensors[i][j][0] = frame_received.can_id;       // first index will be id
                
                for (int k = 0; k < frame_received.can_dlc; k++) {              
                    sensors[i][j][k+1] = frame_received.data[k];    // follower | sensor (packet) | data 
                }
                
                printf("Stored frame");

            }
        }
    } 
}

int main() {

    int can0 = create_socket("can0"); // States
    int can1 = create_socket("can1"); // Sensor data
    int can2 = create_socket("can2"); // Internal

    // Initial READY command
    current_state = READY;
    send_state(can0, current_state);

    master_request(can1);

    


    close(can0);
    close(can1);
    close(can2);
}