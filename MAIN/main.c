#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <libwebsockets.h>

static int interrupted;
static struct lws *client_wsi = NULL;
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

static int callback_websocket(struct lws *wsi, enum lws_callback_reasons reason,
                              void *user, void *in, size_t len) {
    switch (reason) {
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            printf("Client connected\n");
            // Allocate space for the message with LWS_PRE padding
            {
                char *msg = "Hello, WebSocket!";
                size_t msg_len = strlen(msg);
                unsigned char buf[LWS_PRE + msg_len];
                memset(buf, 0, sizeof(buf));
                memcpy(&buf[LWS_PRE], msg, msg_len);
                lws_write(wsi, &buf[LWS_PRE], msg_len, LWS_WRITE_TEXT);
            }
            break;
        case LWS_CALLBACK_CLIENT_RECEIVE:
            printf("Received: %.*s\n", (int)len, (char *)in);
            break;
        case LWS_CALLBACK_CLIENT_CLOSED:
            printf("Client disconnected\n");
            interrupted = 1;
            break;
        case LWS_CALLBACK_CLIENT_WRITEABLE:
            // Here you can queue more messages to send if needed
            break;
        default:
            printf("Callback reason: %d\n", reason);
            break;
    }
    return 0;
}

void *websocket_thread(void *arg) {
    struct lws_context *context = (struct lws_context *)arg;

    while (!interrupted) {
        pthread_mutex_lock(&lock);
        lws_service(context, 1000);
        pthread_mutex_unlock(&lock);
    }

    return NULL;
}

int main(int argc, char **argv) {
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
    if (!context) {
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

    if (!lws_client_connect_via_info(&ccinfo)) {
        fprintf(stderr, "Client connection failed\n");
        lws_context_destroy(context);
        return -1;
    }

    // Create a thread for the libwebsockets event loop
    if (pthread_create(&thread_id, NULL, websocket_thread, context)) {
        fprintf(stderr, "Error creating thread\n");
        lws_context_destroy(context);
        return -1;
    }

    // Main application logic can run here
    while (!interrupted) {
        // Simulate doing something useful
        printf("Main thread working...\n");
        sleep(1);
    }

    // Wait for the websocket thread to finish
    pthread_join(thread_id, NULL);

    lws_context_destroy(context);

    return 0;
}
