#include "main.h"

volatile uint8_t Fault_Flag;
uint8_t ISO_STATE;
PodState Curr_State = INIT;


int sensor_flag = 0;
int response_flag = 0;

// Function to map state strings to state IDs
unsigned int map_state_to_id(const char* state) {
    if (strcmp(state, "INIT") == 0) return INIT;
    else if (strcmp(state, "FAULT") == 0) return FAULT;
    else if (strcmp(state, "SAFE_TO_APPROACH") == 0) return SAFE_TO_APPROACH;
    else if (strcmp(state, "COAST") == 0) return COAST;
    else if (strcmp(state, "BRAKE") == 0) return BRAKE;
    else if (strcmp(state, "CRAWL") == 0) return CRAWL;
    else if (strcmp(state, "TRACK") == 0) return TRACK;
    else if (strcmp(state, "LAUNCH") == 0) return LAUNCH;
    else if (strcmp(state, "READY") == 0) return READY;
    else return 0;
}

char* serialize_sensors() {
    json_object *jroot = json_object_new_object();
    json_object *jsensor_temps = json_object_new_array();
    json_object *jsensor_pressures = json_object_new_array();

    // Serialize temperature sensors data
    for (int i = 0; i < 4; i++) {
        json_object_array_add(jsensor_temps, json_object_new_double(sensors_data[i]));
    }

    // Serialize pressure sensors data
    for (int i = 4; i < 6; i++) {
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
static int callback_websockets(const struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len) {
    switch (reason) {
        case LWS_CALLBACK_CLIENT_RECEIVE:
            // Copy the received message into the state variable
            // Data is pointed to by in and has length len

            printf("Receievd state: %s\n", (char*) in);
            
            // state variable 
            char state[20];
            strncpy(state, (char*) in, sizeof(state) - 1);
            state[sizeof(state) - 1] = '\0';
            
            // Use this if you rather have the extended ID
            //unsigned int state_id = map_state_to_id((char*) in);

            // DO WHATEVER ACTION WITH STATE HERE 
            

            // If send is a success then set this to 1 if not leave at 0
            int success_or_fail = 0;

            //ON A SUCCESSFUL SEND SET FLAG AND CALL WRITE
            response_flag = 1;
            lws_callback_on_writable(wsi);


            break;

        // Send array data to frontend in JSON format
        case LWS_CALLBACK_CLIENT_WRITEABLE:
            if (sensor_flag){
                char *json_data = serialize_sensors();
                lws_write(wsi, (unsigned char *) json_data, strlen(json_data), LWS_WRITE_TEXT);
                free(json_data);
                sensor_flag = 0;
            }    
            if (response_flag){
                int success_or_fail = 0;
                if (success_or_fail){
                    response = "Success";
                }else{
                    response = "Fail";
                }
                lws_write(wsi, (unsigned char *) response, strlen(response), LWS_WRITE_TEXT);
                response_flag = 0;
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
static const struct lws_protocols protocols[] = {
    {
        "default-protocol",
        callback_websockets,
        0,
        512, // Adjust the protocol buffer size as needed
    },
    { NULL, NULL, 0, 0 } /* terminator */
};


uint8_t Run_State(PodState state) {
	error_handler status = OK;

    switch (state) {
        case INIT: //Auto state
        	
        	if(TEMP_INIT() != TEMP_INIT_SUCCESS){
        		return 1;
        	}
        	can_init();
			pid_init();
        	//pump_control(1);
        	
        	Curr_State = SAFE_TO_APPROACH;
        	return status;
            break;
        case FAULT: // auto/manual state
        	HV_off();
        	yellowstatus(0);
        	greenstatus(0);
        	redstatus(1);
        	//pump_control(0);
        	brake_state(0);

        	return status;
            break;
        case SAFE_TO_APPROACH: //manual state
        	HV_off();
        	yellowstatus(0);
        	greenstatus(0);
        	brake_state(1);

        	return status;
            break;
        case READY: //manual state
        	precharge();
        	yellowstatus(1);
        	brake_state(0);
        	return status;
            break;
        case LAUNCH: //manual state
        	yellowstatus(0);
        	greenstatus(1);

        	return status;
            break;
        case COAST: //auto state
        	yellowstatus(0);
        	greenstatus(1);

        	return status;
            break;
        case BRAKE: //auto state
        	yellowstatus(0);
        	greenstatus(1);
        	brake_state(1);

        	return status;
            break;
        case CRAWL: //auto state
        	yellowstatus(0);
        	greenstatus(1);
        	brake_state(0);

        	return status;
            break;
        case TRACK: //manual state
        	HV_off();
        	yellowstatus(0);
        	greenstatus(0);
        	brake_state(0);

        	return status;
            break;
        default:
        	Curr_State = FAULT;
        	//invalid state
            return 1;
            break;
    }
}






int main(void){
	// WebSocket initialization
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof info);
    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;

    struct lws_context *context = lws_create_context(&info);
    if (context == NULL) {
        fprintf(stderr, "lws init failed\n");
        return -1;
    }

    // Connecting to the server
    struct lws_client_connect_info connect_info = {0};
    connect_info.context = context;
    connect_info.address = "localhost";
    connect_info.port = 3000;
    connect_info.path = "/";
    connect_info.protocol = protocols[0].name;
    connect_info.host = lws_canonical_hostname(context)

    struct lws *wsi = lws_client_connect_via_info(&connect_info);
    if (wsi == NULL) {
        fprintf(stderr, "Connection failed\n");
        lws_context_destroy(context);
        return -1;
    }




	time_t last_request_time = time(NULL);

    //i2c init
    if (gpioInitialise() < 0)
    {
        Curr_State = FAULT;
    }

    Fault_Flag = Run_State(Curr_State);
  
  	while (1){
		int n = lws_service(context, 1000);
    	if (n < 0) { 
        	fprintf(stderr, "Error occurred\n");
        	break;
    	}

    	// write array sensor data every 10 seconds
    	if (time(NULL) - last_request_time >= 10) {
        	last_request_time = time(NULL);
        	sensor_flag = 1;
        	lws_callback_on_writable(wsi);
    	}
    



		if(msg_wait() < 0){
			printf("CAN Error");
		}
		Fault_Flag = Run_State(Curr_State);
		if(Fault_Flag != 0){
			Curr_State = FAULT;
		}
	
  		}
	close(can0);
	lws_context_destroy(context);
	return 0;
}
