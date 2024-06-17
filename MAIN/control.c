#include "control.h"

/* MAIN ######################################################################################################################################*/


/*ESC #######################################################################################################################################*/

uint8_t current[4];
double previous_curr = 0.0;
uint8_t dist_prev;

typedef struct
{
    double kp;             // Proportional gain
    double ki;             // Integral gain
    double kd;             // Derivative gain
    double previous_error; // Previous error value
    double integral;       // Integral value
} PIDController;

PIDController pid;

int run[][2] = {
    {0, 100}, {1, 100}, {2, 100}, {3, 100}, {4, 100}, {5, 100}, {6, 100}, {7, 100}, {8, 100}, {9, 100}, {10, 100}, {11, 100}, {12, 100}, {13, 100}, {14, 100}, {15, 100}, {16, 100}, {17, 100}, {18, 100}, {19, 100}, {20, 100}, {21, 100}, {22, 100}, {23, 100}, {24, 100}, {25, 100}, {26, 100}, {27, 100}, {28, 100}, {29, 100}, {30, 100}, {31, 100}, {32, 100}, {33, 100}, {34, 100}, {35, 100}, {36, 100}, {37, 100}, {38, 100}, {39, 100}, {40, 100}, {41, 100}, {42, 100}, {43, 100}, {44, 100}, {45, 100}, {46, 100}, {47, 100}, {48, 100}, {49, 100}, {50, 100}, {51, -999}};

/*CAN #######################################################################################################################################*/
uint32_t M_RPM;
uint16_t M_CURR;


#define ESC_ID 0x65
#define IMD_ID 0xA100101
#define BMD_ID 0x99

int can0;

/*RELAY #####################################################################################################################################*/
uint16_t RelayStates;

#define MCP23017_ADDRESS 0x20
#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
#define MCP23017_OLATA 0x0A
#define MCP23017_OLATB 0x1A

int mcp[2];
#define I2C_BUS 0
#define I2C_FLAGS 0

#define RELAY1 0x01  // PD15 RED LED
#define RELAY2 0x02  // PD14 YELLOW LED
#define RELAY3 0x04  // PD13 GREEN LED
#define RELAY4 0x08  // PD12 BRAKE RELEASE
#define RELAY5 0x10  // PD11 PUMP 1
#define RELAY6 0x20  // PD10 PUMP 2
#define RELAY7 0x40  // PD9
#define RELAY8 0x80  // PD8
#define RELAY9 0x01  // PE7
#define RELAY10 0x02 // PE8
#define RELAY11 0x04 // PE9 MOTOR 0 LOWSIDE
#define RELAY12 0x08 // PE10 MOTOR 0 RESISTOR
#define RELAY13 0x10 // PE11 MOTOR 0 HIGHSIDE
#define RELAY14 0x20 // PE13 MOTOR 1 LOWSIDE
#define RELAY15 0x40 // PE14 MOTOR 1 RESISTOR
#define RELAY16 0x80 // PE15 MOTOR 1 HIGHSIDE

uint8_t RelayStatesA = 0x00;
uint8_t RelayStatesB = 0x00;

/*TEMP ######################################################################################################################################*/

#define ADS1115_ID0 0x48
#define ADS1115_ID1 0x49
#define MCP9600_ID0 0x67
#define MCP9600_ID1 0x66
#define MCP9600_ID2 0x65
#define MCP9600_ID3 0x60
#define MCP_CONFIG 0x06
#define MCP_THERMO 0x05
#define ADS_CONFIG 0x01
#define ADS_CONVER 0x00
#define ADS_CON1 0xC583
#define ADS_CON2 0xD583
#define ADS_CON3 0xE583
#define ADS_CON4 0xF583

float TEMP_ADS[8];
float TEMP_MCP[4];

int MCP[4];
int ADS[2];
int temp;
#define I2C_BUS 0
#define I2C_FLAGS 0

#define ADC0_1_MAX_TEMP 250 // 25.0 = 250, 25.3 = 253
#define ADC0_1_MIN_TEMP 0
#define ADC0_2_MAX_TEMP 250
#define ADC0_2_MIN_TEMP 0
#define ADC0_3_MAX_TEMP 250
#define ADC0_3_MIN_TEMP 0
#define ADC0_4_MAX_TEMP 250
#define ADC0_4_MIN_TEMP 0

#define ADC1_1_MAX_TEMP 250 // 25.0 = 250, 25.3 = 253
#define ADC1_1_MIN_TEMP 0
#define ADC1_2_MAX_TEMP 250
#define ADC1_2_MIN_TEMP 0
#define ADC1_3_MAX_TEMP 250
#define ADC1_3_MIN_TEMP 0
#define ADC1_4_MAX_TEMP 250
#define ADC1_4_MIN_TEMP 0

#define MCP_0_MAX_TEMP 250
#define MCP_0_MIN_TEMP 0
#define MCP_1_MAX_TEMP 250
#define MCP_1_MIN_TEMP 0
#define MCP_2_MAX_TEMP 250
#define MCP_2_MIN_TEMP 0
#define MCP_3_MAX_TEMP 250
#define MCP_3_MIN_TEMP 0

#define ADC5_2_MAX_PRESSURE 120.0f
#define ADC5_2_MIN_PRESSURE 60.0f

/*############################################################################################################################################*/

void HV_on()
{

    i2cWriteByteData(mcp[0], MCP23017_OLATB, RelayStatesB ^ RELAY10 ^ RELAY11 ^ RELAY15 ^ RELAY16);
    RelayStatesB = RelayStatesB ^ RELAY10 ^ RELAY11 ^ RELAY15 ^ RELAY16;
}

void precharge()
{
    // high side on, low side precharge resitor on wait 500ms then low side main on, resistor off

    i2cWriteByteData(mcp[0], MCP23017_OLATB, RelayStatesB ^ RELAY9 ^ RELAY10 ^ RELAY14 ^ RELAY15);
    RelayStates = RelayStatesB ^ RELAY9 ^ RELAY10 ^ RELAY14 ^ RELAY15;
    clock_t start_time = clock();
    while (clock() < start_time + 500)
        ;
    HV_on();
}

void HV_off()
{

    i2cWriteByteData(mcp[0], MCP23017_OLATB, RelayStatesB ^ RELAY9 ^ RELAY11 ^ RELAY14 ^ RELAY16);

    RelayStatesB = RelayStatesB ^ RELAY9 ^ RELAY11 ^ RELAY14 ^ RELAY16;
}

void redstatus(uint8_t state)
{
    if (state == 1)
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY1);
        RelayStatesA = RelayStatesA ^ RELAY1;
    }
    else
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY1);
        RelayStatesA = RelayStatesA ^ RELAY1;
    }
}

void yellowstatus(uint8_t state)
{
    if (state == 1)
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY2);
        RelayStatesA = RelayStatesA ^ RELAY2;
    }
    else
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY2);
        RelayStatesA = RelayStatesA ^ RELAY2;
    }
}

void greenstatus(uint8_t state)
{
    if (state == 1)
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY3);
        RelayStatesA = RelayStatesA ^ RELAY3;
    }
    else
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY3);
        RelayStatesA = RelayStatesA ^ RELAY3;
    }
}

void brake_state(uint8_t state)
{
    if (state == 1)
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY4);
        RelayStatesA = RelayStatesA ^ RELAY4;
    }
    else
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY4);
        RelayStatesA = RelayStatesA ^ RELAY4;
    }
}

void pump_control(uint8_t state)
{
    if (state == 1)
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY5 ^ RELAY6);
        RelayStatesA = RelayStatesA ^ RELAY5 ^ RELAY6;
    }
    else
    {
        i2cWriteByteData(mcp[0], MCP23017_OLATA, RelayStatesA ^ RELAY5 ^ RELAY6);
        RelayStatesA = RelayStatesA ^ RELAY5 ^ RELAY6;
    }
}

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

/*ESC #######################################################################################################################################*/

void PID_Init(PIDController *pid, double kp, double ki, double kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0;
    pid->integral = 0;
}

double PID_Compute(PIDController *pid, double set_rpm, double actual_rpm, double dt)
{
    double error = set_rpm - actual_rpm;
    pid->integral += error * dt;
    double derivative = (error - pid->previous_error) / dt;
    double output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->previous_error = error;
    return output;
}

void pid_init()
{

    double kp = 1.0;  // Proportional gain
    double ki = 0.1;  // Integral gain
    double kd = 0.01; // Derivative gain

    PID_Init(&pid, kp, ki, kd);

    return;
}

void update_esc(int actual_rpm, double dist_curr)
{
    double dt = dist_curr - dist_prev;
    dist_prev = dist_curr;
    int setpoint_rpm = run[(int)round(dist_curr)][1];
    double new_curr = previous_curr + PID_Compute(&pid, setpoint_rpm, actual_rpm, dt);
    previous_curr = new_curr;
    new_curr = new_curr * 10;

    current[0] = (uint8_t)new_curr & 0xFF;
    current[1] = (uint8_t)new_curr >> 8 & 0xFF;
    current[2] = (uint8_t)new_curr >> 16 & 0xFF;
    current[3] = (uint8_t)new_curr >> 24 & 0xFF;
    set_esc_curr(current);
};

/*RELAY #####################################################################################################################################*/

void relay_init()
{

    mcp[0] = i2cOpen(I2C_BUS, MCP23017_ADDRESS | 0b000, I2C_FLAGS);
    i2cWriteByteData(mcp[0], MCP23017_IODIRA, 0x00);
    i2cWriteByteData(mcp[0], MCP23017_IODIRB, 0x00);
    i2cWriteByteData(mcp[0], MCP23017_OLATA, 0x00);
    i2cWriteByteData(mcp[0], MCP23017_OLATB, 0x00);
}
// PORT MATCHES LETTER

/*TEMP ######################################################################################################################################*/

error_handler TEMP_INIT()
{
    MCP[0] = i2cOpen(I2C_BUS, MCP9600_ID0, I2C_FLAGS);
    MCP[1] = i2cOpen(I2C_BUS, MCP9600_ID1, I2C_FLAGS);
    // MCP[2] = i2cOpen(I2C_BUS, MCP9600_ID2, I2C_FLAGS);
    // MCP[3] = i2cOpen(I2C_BUS, MCP9600_ID3, I2C_FLAGS);
    ADS[0] = i2cOpen(I2C_BUS, ADS1115_ID0, I2C_FLAGS);
    // ADS[1] = i2cOpen(I2C_BUS, ADS1115_ID1, I2C_FLAGS);

    if (i2cWriteByteData(MCP[0], MCP_CONFIG, 0b00100000) < 0)
    {
        return MCP_CONFIG_FAIL;
    }
    if (i2cWriteByteData(MCP[0], MCP_THERMO, 0b00000000) < 0)
    {
        return MCP_CONFIG_FAIL;
    }
    if (i2cWriteByteData(MCP[1], MCP_CONFIG, 0b00100000) < 0)
    {
        return MCP_CONFIG_FAIL;
    }
    if (i2cWriteByteData(MCP[1], MCP_THERMO, 0b00000000) < 0)
    {
        return MCP_CONFIG_FAIL;
    }
    /*
       if(i2cWriteByteData(MCP[2], MCP_CONFIG, 0b00100000)<0){
            return MCP_CONFIG_FAIL;
        }
        if(i2cWriteByteData(MCP[2], MCP_THERMO, 0b00000000)<0){
            return MCP_CONFIG_FAIL;
        }
        if(i2cWriteByteData(MCP[3], MCP_CONFIG, 0b00100000)<0){
            return MCP_CONFIG_FAIL;
        }
        if(i2cWriteByteData(MCP[3], MCP_THERMO, 0b00000000)<0){
            return MCP_CONFIG_FAIL;
        }
    */
    if (i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON1) < 0)
    {
        return ADS_CONFIG_FAIL;
    }
    if (i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON2) < 0)
    {
        return ADS_CONFIG_FAIL;
    }
    if (i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON3) < 0)
    {
        return ADS_CONFIG_FAIL;
    }
    if (i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON4) < 0)
    {
        return ADS_CONFIG_FAIL;
    }
    /*
        if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON1)<0){
            return ADS_CONFIG_FAIL;
        }
        if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON2)<0){
            return ADS_CONFIG_FAIL;
        }
        if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON3)<0){
            return ADS_CONFIG_FAIL;
        }
        if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON4)<0){
            return ADS_CONFIG_FAIL;
        }
    */
    return TEMP_INIT_SUCCESS;
}

error_handler UPDATE_TEMP()
{
    if (i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON1) < 0)
    {
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[0] = i2cReadWordData(ADS[0], ADS_CONVER) / (2 ^ 16);

    if (i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON2) < 0)
    {
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[1] = i2cReadWordData(ADS[0], ADS_CONVER) / (2 ^ 16);

    if (i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON3) < 0)
    {
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[2] = i2cReadWordData(ADS[0], ADS_CONVER) / (2 ^ 16);

    if (i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON4) < 0)
    {
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[3] = i2cReadWordData(ADS[0], ADS_CONVER) / (2 ^ 16);
    /*
    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON1)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[4] =  i2cReadWordData(ADS[1], ADS_CONVER)/(2^16);

    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON2)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[5] =  i2cReadWordData(ADS[1], ADS_CONVER)/(2^16);

    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON3)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[6] =  i2cReadWordData(ADS[1], ADS_CONVER)/(2^16);

    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON4)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[7] =  i2cReadWordData(ADS[1], ADS_CONVER)/(2^16);
   */

    temp = i2cReadWordData(MCP[0], 0x00);

    TEMP_MCP[0] = ((((uint8_t)temp & 0xFF00) >> 8) * 16.0 + ((uint8_t)temp & 0xFF) / 16.0);

    temp = i2cReadWordData(MCP[1], 0x00);

    TEMP_MCP[1] = ((((uint8_t)temp & 0xFF00) >> 8) * 16.0 + ((uint8_t)temp & 0xFF) / 16.0);
    /*
        temp = i2cReadWordData(MCP[2],0x00);

        TEMP_MCP[2] = ((((uint8_t)temp & 0xFF00)>>8)*16.0 + ((uint8_t)temp & 0xFF)/16.0);

        temp = i2cReadWordData(MCP[3],0x00);

        TEMP_MCP[3] = ((((uint8_t)temp & 0xFF00)>>8)*16.0 + ((uint8_t)temp & 0xFF)/16.0);
    */
    return TEMP_OK;
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

  
