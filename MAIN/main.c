#include "control.h"


#define MESSAGE_INTERVAL 2 * LWS_USEC_PER_SEC

static int interrupted;
static struct lws *client_wsi = NULL;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;


PodState Curr_State = INIT;
double sensors_data[6] = {0};
int Fault_Flag = 0;
uint8_t ISO_STATE;
int sensor_flag = 0;
int response_flag = 0;
int success_or_fail = 0;


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

static void send_message(struct lws *wsi)
{
    char *json_data = serialize_sensors();
    size_t msg_len = strlen(json_data);
    unsigned char buf[LWS_PRE + msg_len];
    memset(buf, 0, sizeof(buf));
    memcpy(&buf[LWS_PRE], json_data, msg_len);
    lws_write(wsi, &buf[LWS_PRE], msg_len, LWS_WRITE_TEXT);
}

static int callback_websocket(struct lws *wsi, enum lws_callback_reasons reason,
                              void *user, void *in, size_t len)
{
    switch (reason)
    {
    case LWS_CALLBACK_CLIENT_ESTABLISHED:
        printf("Client connected\n");
        lws_set_timer_usecs(wsi, MESSAGE_INTERVAL);
        break;
    case LWS_CALLBACK_CLIENT_RECEIVE:
        pthread_mutex_lock(&lock);
        printf("Received: %.*s\n", (int)len, (char *)in);
        if (strcmp((char *)in, "INIT") == 0)
            Curr_State = INIT;
        else if (strcmp((char *)in, "FAULT") == 0)
            Curr_State = FAULT;
        else if (strcmp((char *)in, "SAFE_TO_APPROACH") == 0)
            Curr_State = SAFE_TO_APPROACH;
        else if (strcmp((char *)in, "COAST") == 0)
            Curr_State = COAST;
        else if (strcmp((char *)in, "BRAKE") == 0)
            Curr_State = BRAKE;
        else if (strcmp((char *)in, "CRAWL") == 0)
            Curr_State = CRAWL;
        else if (strcmp((char *)in, "TRACK") == 0)
            Curr_State = TRACK;
        else if (strcmp((char *)in, "LAUNCH") == 0)
            Curr_State = LAUNCH;
        else if (strcmp((char *)in, "READY") == 0)
            Curr_State = READY;
        else
            Curr_State = FAULT;
        pthread_mutex_unlock(&lock);
        break;
    case LWS_CALLBACK_CLIENT_CLOSED:
        printf("Client disconnected\n");
        interrupted = 1;
        break;
    case LWS_CALLBACK_CLIENT_WRITEABLE:
        // We do nothing here
        break;
    case LWS_CALLBACK_TIMER:
        send_message(wsi);
        lws_set_timer_usecs(wsi, MESSAGE_INTERVAL); // Reset the timer
        break;
    default:
        printf("Callback reason: %d\n", reason);
        break;
    }
    return 0;
}

void *websocket_thread(void *arg)
{
    struct lws_context *context = (struct lws_context *)arg;

    while (!interrupted)
    {
        pthread_mutex_lock(&lock);
        lws_service(context, 1000);
        pthread_mutex_unlock(&lock);
    }

    return NULL;
}

int main(int argc, char **argv)
{
    /*if (gpioInitialise() < 0)
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

    // i2c init

    Fault_Flag = Run_State(Curr_State);
    printf("INIT_COMPLETE\n");*/

    struct lws_context_creation_info info;
    struct lws_client_connect_info ccinfo = {0};
    struct lws_context *context;
    pthread_t thread_id;

    memset(&info, 0, sizeof info);
    info.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
    info.port = CONTEXT_PORT_NO_LISTEN; /* No server ports */

    struct lws_protocols protocols[] = {
        {
            "example-protocol",
            callback_websocket,
            0,
            4096,
        },
        {NULL, NULL, 0, 0} /* terminator */
    };
    info.protocols = protocols;

    context = lws_create_context(&info);
    if (!context)
    {
        fprintf(stderr, "lws_create_context failed\n");
        return -1;
    }

    ccinfo.context = context;
    ccinfo.address = "raspberrypi.local";
    ccinfo.port = 3000;
    ccinfo.path = "/";
    ccinfo.host = lws_canonical_hostname(context);
    ccinfo.origin = "origin";
    ccinfo.protocol = protocols[0].name;
    ccinfo.pwsi = &client_wsi;

    if (!lws_client_connect_via_info(&ccinfo))
    {
        fprintf(stderr, "Client connection failed\n");
        lws_context_destroy(context);
        return -1;
    }

    // Create a thread for the libwebsockets event loop
    if (pthread_create(&thread_id, NULL, websocket_thread, context))
    {
        fprintf(stderr, "Error creating thread\n");
        lws_context_destroy(context);
        return -1;
    }

    // Main application logic can run here
    while (!interrupted)
    {
        printf("state: %d", Curr_State);
        sleep(1);
        /*if (msg_wait() < 0)
        {
            printf("CAN Error\n");
        }
        Fault_Flag = Run_State(Curr_State);
        if (Fault_Flag != 0)
        {
            printf("CReMy SHits");
            Curr_State = FAULT;
        } */
    }
    // close(can0);

    // Wait for the websocket thread to finish
    pthread_join(thread_id, NULL);

    lws_context_destroy(context);

    return 0;
}
