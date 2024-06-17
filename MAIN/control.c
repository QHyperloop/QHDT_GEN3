#include "control.h"

/* MAIN ######################################################################################################################################*/

/*CAN #######################################################################################################################################*/
uint32_t M_RPM;
uint16_t M_CURR;


int can0;

/*CAN #######################################################################################################################################*/

// Function to create a socket (CAN bus)
int create_socket(const char *ifname)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    int enable_canfd = 1;

    // Create a socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        perror("Error while opening socket");
        return -1;
    }

    // Specify interface name
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    // Bind socket to network interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        return -1;
    }

    // Enable BRS
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        perror("Error while enabling CAN FD");
        return -1;
    }

    return s;
}

void send_curr_state()
{
    struct canfd_frame frame;
    frame.can_id = 0x00001000;
    frame.len = 1;
    frame.data[0] = (uint8_t)Curr_State;

    if (write(can0, &frame, sizeof(frame)) != sizeof(frame))
    {
        perror("Error in sending CAN frame");
    }else{
         printf("Sent CAN FD frame with ID 0x%08x and data %02x\n",frame.can_id, frame.data[0]);
    }
}

// Function to read the responses and store data into state_responses array
void read_can_responses()
{

    struct canfd_frame frame;
    int nbytes;

    nbytes = read(can0, &frame, sizeof(struct canfd_frame));
    if (nbytes < 0)
    {
        perror("read");
        return;
    }

    if (nbytes < sizeof(struct canfd_frame))
    {
        fprintf(stderr, "read: incomplete CAN frame\n");
        return;
    }
    printf("Recieved response\n"); // to help with testing

    if (frame.can_id == (IMD_ID | (CAN_EFF_FLAG & 0xE)))
    {
        if ((frame.data[0] & 0x40) == 0x40)
        {
            if ((frame.data[0] & 0x03) == 0b10)
            {
                ISO_STATE = 0xF0; // set LED to yellow
                yellowstatus(1);
                printf("iso state warning");
                return;
            }
            else if ((frame.data[0] & 0x03) == 0b11)
            {
                ISO_STATE = 0xFF; // set led to red
                Curr_State = FAULT;
                return;
            }
            else
            {
                ISO_STATE = 0x00;
                return;
            }
        }
    }

    if ((frame.can_id | IMD_ID | (CAN_EFF_FLAG & 0xFF)) == ESC_ID)
    {
        if ((frame.can_id | IMD_ID | (CAN_EFF_FLAG & 0xFF00)) == 0x9000)
        {
            M_RPM = frame.data[3] << 24 | frame.data[2] << 16 | frame.data[1] << 8 | frame.data[0];
            M_CURR = (frame.data[5] << 8 | frame.data[4]) * 10;
            return;
        }
    }
}

int msg_wait()
{
    fd_set readfds;
    struct timeval timeout;
    int ret;

    FD_ZERO(&readfds);
    FD_SET(can0, &readfds);

    timeout.tv_usec = 100;

    ret = select(can0 + 1, &readfds, NULL, NULL, &timeout);

    if (ret < 0)
    {
        perror("select");
        return -1;
    }
    else if (ret == 0)
    {
        // Timeout
        return 0;
    }
    else
    {
        if (FD_ISSET(can0, &readfds))
        {
            // New message received
            
            return 1;
        }
    }

    return 0;
}

/* MAIN ######################################################################################################################################*/

uint8_t Run_State(PodState state)
{
    //error_handler status = OK;

    switch (state)
    {
    case INIT: // Auto state
        printf("INIT\n");

        if (TEMP_INIT() != TEMP_INIT_SUCCESS)
        {
            printf("TEMP_INIT FAIL\n");
            // status = 1;
        }
        else
        {
            printf("TEMP_INIT Success\n");
        }

        pid_init();
        printf("PID_INIT\n");
        // pump_control(1);

        Curr_State = SAFE_TO_APPROACH;
        return 0;
        break;
    case FAULT: // auto/manual state
        printf("FAULT\n");
        HV_off();
        yellowstatus(0);
        greenstatus(0);
        redstatus(1);
        // pump_control(0);
        brake_state(0);

        return 0;
        break;
    case SAFE_TO_APPROACH: // manual state
        printf("SAFE_TO_APPROACH\n");
        HV_off();
        yellowstatus(0);
        greenstatus(0);
        brake_state(1);

        return 0;
        break;
    case READY: // manual state
        printf("READY\n");
        precharge();
        yellowstatus(1);
        brake_state(0);
        return 0;
        break;
    case LAUNCH: // manual state
        printf("LAUNCH\n");
        yellowstatus(0);
        greenstatus(1);

        return 0;
        break;
    case COAST: // auto state
        printf("COAST\n");
        yellowstatus(0);
        greenstatus(1);

        return 0;
        break;
    case BRAKE: // auto state
        printf("BRAKE\n");
        yellowstatus(0);
        greenstatus(1);
        brake_state(1);

        return 0;
        break;
    case CRAWL: // auto state
        printf("CRAWL\n");
        yellowstatus(0);
        greenstatus(1);
        brake_state(0);

        return 0;
        break;
    case TRACK: // manual state
        printf("TRACk\n");
        HV_off();
        yellowstatus(0);
        greenstatus(0);
        brake_state(0);

        return 0;
        break;
    default:
        Curr_State = FAULT;
        // invalid state
        return 1;
        break;
    }
}

  
