/*
 * Copyright (C) Ericsson AB 2009-2010
 * Copyright 2006, The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Torgny Johansson <torgny.johansson@ericsson.com>
 *         Rickard Bellini <rickard.bellini@ericsson.com>
 */

#include <errno.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdlib.h>

#include "log.h"
#include "gpsctrl/gps_ctrl.h"
#include "gpsctrl/pgps.h"
#include "mbm_service_handler.h"

#define FDNAME "/data/data/mbmservice"
#define MBMSERVICE_BACKLOG 10

/* the following defines must match the MSG_* fields in the mbm service handler apk */
#define MSG_AIRPLANE_MODE            "AIRPLANE_MODE"
#define MSG_EXTRA_NETWORK_INFO       "EXTRA_NETWORK_INFO"
#define MSG_EXTRA_OTHER_NETWORK_INFO "EXTRA_OTHER_NETWORK_INFO"
#define MSG_EXTRA_NO_CONNECTIVITY    "EXTRA_NO_CONNECTIVITY"
#define MSG_BACKGROUND_DATA_SETTING  "BACKGROUND_DATA_SETTING"
#define MSG_MOBILE_DATA_ALLOWED      "MOBILE_DATA_ALLOWED"
#define MSG_ROAMING_ALLOWED          "ROAMING_ALLOWED"
#define MSG_ANY_DATA_STATE           "ANY_DATA_STATE"
#define MSG_APN_INFO                 "APN_INFO"
#define MSG_NO_APN_DEFINED           "NO_APN_DEFINED"
#define MSG_OPERATOR_INFO            "OPERATOR_INFO"
#define MSG_PGPS_DATA                "MSG_PGPS_DATA"


#define MAX_APN_LENGTH                  255
#define MAX_USERNAME_LENGTH             255
#define MAX_PASSWORD_LENGTH             255
#define MAX_PATH_LENGTH                 255
#define MAX_AUTHTYPE_LENGTH              10

#define APN_INFO_DELIMITER              '\n'

typedef struct {
    int socket;
    int isInitialized;
    int roaming_data_allowed;
    int background_data_allowed;
    int data_enabled;
} ServiceContext;

static ServiceContext global_service_context;
pthread_t socket_thread;

static int initializeServiceContext(void)
{
    ServiceContext *context;

    context = &global_service_context;
    memset(context, 0, sizeof(ServiceContext));

    context->roaming_data_allowed = 0;
    context->background_data_allowed = 0;
    context->data_enabled = 0;
    context->isInitialized = 1;

    LOGI("Initialized new gps ctrl context");

    return 0;
}

/* get the current context */
static ServiceContext* get_service_context(void)
{
    if (!global_service_context.isInitialized)
        LOGE("Service context not initialized. Possible problems ahead!");
    return &global_service_context;
}

static int internal_init(void)
{
    struct sockaddr_un server;
    int sock;
    int err;
    LOGD("%s", __FUNCTION__);

     /* Initialise socket for GPSService message reception */
    unlink(FDNAME);
    sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0) {
        LOGE("opening gpsservice stream socket: %s", strerror(errno));
        return -1;
    }

    LOGD("%s, socket=%d", __FUNCTION__, sock);

    server.sun_family = AF_UNIX;
    strcpy(server.sun_path, FDNAME);

    if (bind(sock, (struct sockaddr *) &server, sizeof(struct sockaddr_un))) {
        LOGE("binding stream socket: %s", strerror(errno));
        return -1;
    }

    if (listen(sock, MBMSERVICE_BACKLOG) == -1 ) {
        LOGE("listen stream socket: %s", strerror(errno));
        return -1;
    }

    LOGD("Socket has name: %s, id:%d", server.sun_path, sock);

    err = chmod(FDNAME,
                S_IRUSR | S_IWUSR |
                S_IRGRP | S_IWGRP |
                S_IROTH | S_IWOTH);
    if (err < 0)
        LOGE("Error setting permissions of socket");

    return sock;
}

void service_handler_send_message(char cmd, char *data)
{
    ServiceContext *context = get_service_context();
    int ret;

    LOGD("%s, enter, %d", __FUNCTION__, context->socket);

    ret = write(context->socket, &cmd, 1);
    if (ret < 0)
        LOGE("Error sending to mbm service: %s", strerror(errno));

    cmd = (char) strlen(data);
    write(context->socket, &cmd, 1);
    if (ret < 0)
        LOGE("Error sending to mbm service");

    write(context->socket, data, strlen(data));
    if (ret < 0)
        LOGE("Error sending to mbm service");

    LOGD("%s, exit", __FUNCTION__);
}

static int parse_info(char *dst, char *src, char* tag, int max_len)
{
    char *idx1;
    char *idx2;

    idx1 = strstr(src, tag);
    if (!idx1)
        return -1;
    idx1 = idx1 + strlen(tag);
    idx2 = strchr(idx1, APN_INFO_DELIMITER);
    if (!idx2)
        return -1;
    if (idx2 - idx1 > max_len - 1)
        return -1;
    strncpy(dst, idx1, idx2 - idx1);
    dst[idx2 - idx1] = '\0';

    return 0;
}

static void parse_message(char *msg)
{
    ServiceContext *context = get_service_context();
    char *data = NULL;
    char *idx = strchr(msg, ':');

    if(idx)
        asprintf(&data, "%s", idx + 1);
    else
        asprintf(&data, "%s", "");

    if (strstr(msg, MSG_BACKGROUND_DATA_SETTING)) {
        LOGD("Background data allowed: %s", data);
        if (strstr(data, "true")) {
            context->background_data_allowed = 1;
            gpsctrl_set_background_data_allowed(1);
        } else {
            context->background_data_allowed = 0;
            gpsctrl_set_background_data_allowed(0);
        }
    } else if (strstr(msg, MSG_MOBILE_DATA_ALLOWED)) {
        LOGD("Data enabled: %s", data);
        if (strstr(data, "true")) {
            context->data_enabled = 1;
            gpsctrl_set_data_enabled(1);
        } else {
            context->data_enabled = 0;
            gpsctrl_set_data_enabled(0);
        }
    } else if (strstr(msg, MSG_ROAMING_ALLOWED)) {
        LOGD("Roaming data allowed: %s", data);
        if (strstr(data, "true")) {
            context->roaming_data_allowed = 1;
            gpsctrl_set_data_roaming_allowed(1);
        } else {
            context->roaming_data_allowed = 0;
            gpsctrl_set_data_roaming_allowed(0);
        }
    } else if (strstr(msg, MSG_APN_INFO)) {
        char apn[MAX_APN_LENGTH];
        char username[MAX_USERNAME_LENGTH];
        char password[MAX_PASSWORD_LENGTH];
        char auth[MAX_AUTHTYPE_LENGTH];

        LOGD("%s: %s", MSG_APN_INFO, data);

        if (0 != parse_info(apn, data, "apn=", MAX_APN_LENGTH))
            return;
        if (0 != parse_info(username, data, "user=", MAX_USERNAME_LENGTH))
            return;
        if (0 != parse_info(password, data, "pass=", MAX_PASSWORD_LENGTH))
            return;
        if (0 != parse_info(auth, data, "authtype=", MAX_AUTHTYPE_LENGTH))
            return;

        gpsctrl_set_apn_info(apn, username, password, auth);
    } else if (strstr(msg, MSG_PGPS_DATA)) {
        char path[MAX_PATH_LENGTH];
        char s_id[10];
        int id;

        LOGD("%s: %s", MSG_PGPS_DATA, data);

        if (strstr(data, "failed"))
            onPgpsDataFailed();
        else {
            if (0 != parse_info(s_id, data, "id=", MAX_PATH_LENGTH)) {
                LOGD("FAILED TO PARSE ID");
                onPgpsDataFailed();
                return;
            }
            if (0 != parse_info(path, data, "path=", MAX_PATH_LENGTH)) {
                LOGD("FAILED TO PARSE PATH");
                onPgpsDataFailed();
                return;
            }

            id = atoi(s_id);
            pgps_read_data(id, path);
        }
    } else {
        LOGD("Unknown message from mbm service received");
    }
}

static void* socket_loop(void* arg)
{
    int msgsock, rval, ret, socket;
    char buf[1024];
    int len;
    char lsb, msb;
    ServiceContext *context = get_service_context();
    (void) arg;

    if ((ret = internal_init()) < 0) {
        LOGE("Error initializing socket");
        return NULL;
    } else
        socket = ret;

    LOGD("%s, socket=%d", __FUNCTION__, socket);

    while(1) {
        msgsock = accept(socket, 0, 0);
        context->socket = msgsock;
        LOGD("%s, msgsock=%d", __FUNCTION__, msgsock);

        if (msgsock == -1) {
            LOGE("socket accept: %s, exiting service loop", strerror(errno) );
            close(socket);
            return NULL;
        } else {
            do {
                memset(buf, 0, 1024);
                len = lsb = msb = 0;
                if ((rval = read(msgsock, &msb, 1)) < 0)
                        LOGE("reading stream length: %s", strerror(errno));
                if ((rval = read(msgsock, &lsb, 1)) < 0)
                        LOGE("reading stream length: %s", strerror(errno));

                len |= msb;
                len = len << 8;
                len |= lsb;
                if ((rval = read(msgsock, buf, len)) < 0)
                    LOGE("reading stream message: %s", strerror(errno) );
                else if (rval > 0)
                    parse_message(buf);
            } while (rval > 0);
        }
        close(msgsock);
    }

    LOGD("%s exiting", __FUNCTION__);
    return NULL;
}

int service_handler_init(void)
{
    int ret;

    if (initializeServiceContext()) {
        LOGE("Initialize service context failed!");
        return -1;
    }

    ret = pthread_create(&socket_thread, NULL, socket_loop, NULL);
    if (ret < 0) {
        LOGE("%s error creating socket thread", __FUNCTION__);
        return -1;
    }

    return 0;
}

int service_handler_stop(void)
{
    ServiceContext *context = get_service_context();

    LOGD("%s", __FUNCTION__);

    service_handler_send_message(CMD_SERVICE_QUIT, "");

    close(context->socket);

    return 0;
}
