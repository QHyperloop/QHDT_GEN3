#include "main.h"

/* MAIN ######################################################################################################################################*/
int Fault_Flag = 0;
uint8_t ISO_STATE;
PodState Curr_State = INIT;
int sensor_flag = 0;
int response_flag = 0;
double sensors_data[6];
int success_or_fail = 0;

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
uint8_t ISO_STATE;

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

void set_esc_curr(uint8_t curr[])
{
    struct canfd_frame frame;
    frame.can_id = 0x100 | ESC_ID | CAN_EFF_FLAG;
    frame.len = 8;
    frame.data[0] = curr[0];
    frame.data[1] = curr[1];
    frame.data[2] = curr[2];
    frame.data[3] = curr[3];
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(can0, &frame, sizeof(frame)) != sizeof(frame))
    {
        perror("Error in sending CAN frame");
    }
}

void IMD_REQ_ISO()
{
    struct canfd_frame frame;
    frame.can_id = IMD_ID | CAN_EFF_FLAG;
    frame.len = 1;
    frame.data[0] = 0xE0;
    if (write(can0, &frame, sizeof(frame)) != sizeof(frame))
    {
        perror("Error in sending CAN frame");
    }
}

// Function to read the responses and store data into state_responses array
void read_state_responses()
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
// Function to map state strings to state IDs
unsigned int map_state_to_id(const char *state)
{
    if (strcmp(state, "INIT") == 0)
        return INIT;
    else if (strcmp(state, "FAULT") == 0)
        return FAULT;
    else if (strcmp(state, "SAFE_TO_APPROACH") == 0)
        return SAFE_TO_APPROACH;
    else if (strcmp(state, "COAST") == 0)
        return COAST;
    else if (strcmp(state, "BRAKE") == 0)
        return BRAKE;
    else if (strcmp(state, "CRAWL") == 0)
        return CRAWL;
    else if (strcmp(state, "TRACK") == 0)
        return TRACK;
    else if (strcmp(state, "LAUNCH") == 0)
        return LAUNCH;
    else if (strcmp(state, "READY") == 0)
        return READY;
    else
        return 0;
}

char *serialize_sensors()
{
    json_object *jroot = json_object_new_object();
    json_object *jsensor_temps = json_object_new_array();
    json_object *jsensor_pressures = json_object_new_array();

    // Serialize temperature sensors data
    for (int i = 0; i < 4; i++)
    {
        json_object_array_add(jsensor_temps, json_object_new_double(sensors_data[i]));
    }

    // Serialize pressure sensors data
    for (int i = 4; i < 6; i++)
    {
        json_object_array_add(jsensor_pressures, json_object_new_double(sensors_data[i]));
    }

    json_object_object_add(jroot, "temperature_sensors", jsensor_temps);
    json_object_object_add(jroot, "pressure_sensors", jsensor_pressures);

    const char *json_str = json_object_to_json_string(jroot);
    char *result = strdup(json_str);
    json_object_put(jroot);
    return result;
}

// Callback function. Caleld by libwebsockets whenever an event occurs
int callback_websockets(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{
    switch (reason)
    {
    case LWS_CALLBACK_RECEIVE:
        // Copy the received message into the state variable
        // Data is pointed to by in and has length len

        printf("Receievd state: %s\n", (char *)in);

        // state variable
        char state[20];
        strncpy(state, (char *)in, sizeof(state) - 1);
        state[sizeof(state) - 1] = '\0';

        // Use this if you rather have the extended ID
        // unsigned int state_id = map_state_to_id((char*) in);
        //printf("States: ");
       // puts(state);
        //printf("\n");
        // DO WHATEVER ACTION WITH STATE HERE

        // If send is a success then set this to 1 if not leave at 0
        success_or_fail = 1;
        printf("hit \n");
        // ON A SUCCESSFUL SEND SET FLAG AND CALL WRITE
        response_flag = 1;
        lws_callback_on_writable(wsi);

        break;

    // Send array data to frontend in JSON format
    case LWS_CALLBACK_CLIENT_WRITEABLE:
        if (sensor_flag)
        {
            char *out = NULL;
            printf("5\n");
            char *json_data = serialize_sensors();
            printf("6\n");
            out = (char *)malloc(sizeof(char)*(LWS_SEND_BUFFER_PRE_PADDING + strlen(json_data) + LWS_SEND_BUFFER_POST_PADDING));
            memcpy (out + LWS_SEND_BUFFER_PRE_PADDING, json_data, strlen(json_data) );

            lws_write(wsi,(unsigned char*) out + LWS_SEND_BUFFER_PRE_PADDING, strlen(json_data), LWS_WRITE_TEXT);
            free(out);

            sensor_flag = 0;
        }
        if (response_flag)
        {
            char *response = "Fail";
            char *out = NULL;
            if (success_or_fail)
            {
                printf("Success\n");
                response = "Success";
            }
            else
            {
                printf("Fail\n");
                response = "Fail";
            }
            
            out = (char *)malloc(sizeof(char)*(LWS_SEND_BUFFER_PRE_PADDING + strlen(response) + LWS_SEND_BUFFER_POST_PADDING));
            memcpy (out + LWS_SEND_BUFFER_PRE_PADDING, response, strlen(response) );
            lws_write(wsi,(unsigned char*) out + LWS_SEND_BUFFER_PRE_PADDING, strlen(response), LWS_WRITE_TEXT);
            free(out);
           
            response_flag = 0;
            success_or_fail = 0;
        }
        break;

    case LWS_CALLBACK_CLIENT_ESTABLISHED:
        printf("Connection established\n");
        lws_callback_on_writable(wsi);
        break;

    case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
        printf("Connection failed\n");
        break;

    case LWS_CALLBACK_CLIENT_CLOSED:
        printf("Connection closed\n");
        break;

    default:
        break;
    }
    return 0;
}

// Protocols Array, specifies protocols supported by the client
static struct lws_protocols protocols[] = {
    {
        "default-protocol",
        callback_websockets,
        0,
        512, // Adjust the protocol buffer size as needed
    },
    {NULL, NULL, 0, 0} /* terminator */
};

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

int main(void)
{
   /* if (gpioInitialise() < 0)
    {
        printf("GPIO INIT FAIL\n");
        Curr_State = FAULT;
    }
    else
    {
        printf("GPIO INIT SUCCESS\n");
    }

    // init can
    can0 = create_socket("can0");
    printf("CAN INIT SUCCESS\n");
*/
    // WebSocket initialization
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof info);
    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;

    struct lws_context *context = lws_create_context(&info);
    if (context == NULL)
    {
        printf("LWS INIT FAILED\n");
        fprintf(stderr, "lws init failed\n");
        return -1;
    }else{
        printf("LWS INIT SUCCESS\n");
    }

    // Connecting to the server
    struct lws_client_connect_info connect_info = {0};
    connect_info.context = context;
    connect_info.address = "raspberrypi.local";
    connect_info.port = 3000;
    connect_info.path = "/";
    connect_info.protocol = "";
    connect_info.local_protocol_name = protocols[0].name;
    connect_info.host = lws_canonical_hostname(context);

    struct lws *wsi = lws_client_connect_via_info(&connect_info);
    if (wsi == NULL)
    {
        fprintf(stderr, "Connection failed\n");
        printf("LWS Connection Failed\n");
        lws_context_destroy(context);
        return -1;
    }
    else
    {
        printf("LWS Connected\n");
    }

    time_t last_request_time = time(NULL);
    lws_set_log_level(LLL_ERR | LLL_WARN | LLL_NOTICE | LLL_INFO | LLL_DEBUG, NULL);
    // i2c init

    //Fault_Flag = Run_State(Curr_State);
    printf("INIT_COMPLETE\n");

    while (1)
    {
        printf("1\n");
        int n = lws_service(context, 1000);
        printf("n: %d",n);
        printf("2\n");
        if (n < 0)
        {
            printf("LWS ERROR OCCURRED\n");
            fprintf(stderr, "Error occurred\n");
            break;
        }
        printf("3\n");

        // write array sensor data every 10 seconds
        if (time(NULL) - last_request_time >= 10)
        {
            printf("Data Sent\n");
            last_request_time = time(NULL);
            sensor_flag = 1;
            lws_callback_on_writable(wsi);
        }
        printf("4\n");
/*
        if (msg_wait() < 0)
        {
            printf("CAN Error\n");
        }
        Fault_Flag = Run_State(Curr_State);
        if (Fault_Flag != 0)
        {
            printf("CReMy SHits");
            Curr_State = FAULT;
        }*/
        
    }
    
    close(can0);
    lws_context_destroy(context);
    return 0;
}
