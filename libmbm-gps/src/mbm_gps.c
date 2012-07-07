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
 */

#include <errno.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <poll.h>
#include <termios.h>
#include <math.h>
#include <sys/socket.h>
#include <pthread.h>
#include <cutils/properties.h>
#include <hardware/gps.h>

#include "gpsctrl/gps_ctrl.h"
#include "gpsctrl/nmeachannel.h"
#include "nmea_reader.h"
#include "mbm_service_handler.h"
#include "version.h"
#include "log.h"

/* Just check this file */
#ifdef __GNUC__
#pragma GCC diagnostic warning "-pedantic"
#endif

#define MAX_AT_RESPONSE (8 * 1024)
#define TIMEOUT_POLL 200
#define EMRDY_POLL_INTERVAL 1000 /* needs to be a multiple of TIMEOUT_POLL */
#define TIMEOUT_EMRDY 10000 /* needs to be a multiple of EMRDY_POLL_INTERVAL */

#define SUPLNI_VERIFY_ALLOW 0
#define SUPLNI_VERIFY_DENY 1
#define SUPLNI_NOTIFY 2
#define SUPLNI_NOTIFY_DENIED 3

#define DEFAULT_NMEA_PORT "/dev/ttyACM2"
#define DEFAULT_CTRL_PORT "/dev/bus/usb/002/049"

#define PROP_SUPL "SUPL"
#define PROP_STANDALONE "STANDALONE"
#define PROP_PGPS "PGPS"

#define SINGLE_SHOT_INTERVAL 9999

#define CLEANUP_REQUESTED -10
#define DEVICE_NOT_READY -11

enum {
    CMD_STATUS_CB = 0,
    CMD_AGPS_STATUS_CB,
    CMD_NI_CB,
    CMD_DEV_LOST,
    CMD_QUIT
};

typedef struct {
    int device_state;
    NmeaReader reader[1];
    GpsCtrlSuplConfig supl_config;
    gps_status_callback status_callback;
    gps_create_thread create_thread_callback;
    AGpsCallbacks agps_callbacks;
    AGpsStatus agps_status;

    int gps_started;
    int gps_should_start;
    int gps_initiated;
    int clear_flag;
    int cleanup_requested;

    int pref_mode;
    int allow_uncert;
    int enable_ni;

    GpsStatus gps_status;
    AGpsType type;

    int ril_connected;
    int ril_roaming;
    int ril_type;

    gps_ni_notify_callback ni_callback;
    GpsCtrlSuplNiRequest current_ni_request;
    GpsNiNotification notification;

    int control_fd[2];

    pthread_t main_thread;
    pthread_mutex_t mutex;
    pthread_mutex_t cleanup_mutex;
    pthread_cond_t cleanup_cond;
}GpsContext;

GpsContext global_context;

static void add_pending_command(char cmd);
static void nmea_received(char *line);
static int mbm_gps_start(void);
static void main_loop(void *arg);

static GpsContext* get_gps_context(void)
{
    return &global_context;
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       GPS-NI I N T E R F A C E                        *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

void mbm_gps_ni_request(GpsCtrlSuplNiRequest *ni_request)
{
    GpsContext *context = get_gps_context();

    LOGD("%s: enter", __FUNCTION__);

    if (ni_request == NULL) {
        LOGE("%s: ni_request = NULL", __FUNCTION__);
        return;
    }

    context->current_ni_request.message_id = ni_request->message_id;
    context->current_ni_request.message_type = ni_request->message_type;

    context->notification.notification_id = ni_request->message_id;
    context->notification.ni_type = GPS_NI_TYPE_UMTS_SUPL;
    context->notification.timeout = 30;
    context->notification.requestor_id_encoding = GPS_ENC_SUPL_UCS2;
    context->notification.text_encoding = GPS_ENC_SUPL_UCS2;

    LOGD("%s: notification filled with id:%d, type:%d, id_encoding:%d, text_encoding:%d", __FUNCTION__, context->notification.notification_id, context->notification.ni_type, context->notification.requestor_id_encoding, context->notification.text_encoding);

    snprintf(context->notification.requestor_id, GPS_NI_SHORT_STRING_MAXLEN,
             "%s", ni_request->requestor_id_text);

    LOGD("%s: requestor id filled with: %s", __FUNCTION__,
         context->notification.requestor_id);

    snprintf(context->notification.text, GPS_NI_LONG_STRING_MAXLEN,
             "%s", ni_request->client_name_text);

    LOGD("%s: text filled with: %s", __FUNCTION__,
         context->notification.text);

    switch (ni_request->message_type) {
    case SUPLNI_VERIFY_ALLOW:
        context->notification.notify_flags =
            GPS_NI_NEED_VERIFY | GPS_NI_NEED_NOTIFY;
        context->notification.default_response = GPS_NI_RESPONSE_ACCEPT;
        LOGD("%s: SUPL_VERIFY_ALLOW", __FUNCTION__);
        break;
    case SUPLNI_VERIFY_DENY:
        context->notification.notify_flags =
            GPS_NI_NEED_VERIFY | GPS_NI_NEED_NOTIFY;
        context->notification.default_response = GPS_NI_RESPONSE_DENY;
        LOGD("%s: SUPL_VERIFY_DENY", __FUNCTION__);
        break;
    case SUPLNI_NOTIFY:
        context->notification.notify_flags = GPS_NI_NEED_NOTIFY;
        context->notification.default_response = GPS_NI_RESPONSE_ACCEPT;
        LOGD("%s: SUPL_NOTIFY", __FUNCTION__);
        break;
    case SUPLNI_NOTIFY_DENIED:
        context->notification.notify_flags = GPS_NI_NEED_NOTIFY;
        context->notification.default_response = GPS_NI_RESPONSE_DENY;
        LOGD("%s: SUPL_NOTIFY_DENIED", __FUNCTION__);
        break;
    default:
        LOGD("%s: unknown request", __FUNCTION__);
        break;
    }

    add_pending_command(CMD_NI_CB);
    LOGD("%s: exit", __FUNCTION__);
}

void mbm_gps_ni_init(GpsNiCallbacks * callbacks)
{
    GpsContext *context = get_gps_context();

    LOGD("%s: enter", __FUNCTION__);

    if (callbacks == NULL) {
        LOGE("%s: callbacks = NULL", __FUNCTION__);
        return;
    }

    context->ni_callback = callbacks->notify_cb;

    /* todo register for callback */
    
    LOGD("%s: exit", __FUNCTION__);

}

void mbm_gps_ni_respond(int notif_id, GpsUserResponseType user_response)
{
    int allow;
    GpsContext *context = get_gps_context();

    LOGD("%s: enter notif_id=%i", __FUNCTION__, notif_id);

    if (notif_id != context->current_ni_request.message_id) {
        LOGD("Mismatch in notification ids. Ignoring the response.");
        return;
    }

    switch (user_response) {
    case GPS_NI_RESPONSE_ACCEPT:
        allow = 1;
        break;
    case GPS_NI_RESPONSE_DENY:
        allow = 0;
        break;
    case GPS_NI_RESPONSE_NORESP:
        if (context->current_ni_request.message_type == SUPLNI_VERIFY_DENY)
            allow = 0;
        else
            allow = 1;
        break;
    default:
        allow = 0;
        break;
    }

    gpsctrl_supl_ni_reply(&context->current_ni_request, allow);
    LOGD("%s: exit", __FUNCTION__);
}

/**
 * Extended interface for Network-initiated (NI) support.
 */
static const GpsNiInterface mbmGpsNiInterface = {
    sizeof(GpsNiInterface),
    /** Registers the callbacks for HAL to use. */
    mbm_gps_ni_init,

    /** Sends a response to HAL. */
    mbm_gps_ni_respond,
};

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       AGPS I N T E R F A C E                          *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

void mbm_agps_init(AGpsCallbacks * callbacks)
{
    GpsContext *context = get_gps_context();

    LOGD("%s: enter", __FUNCTION__);

    if (callbacks == NULL) {
        LOGE("%s: callbacks = NULL", __FUNCTION__);
        return;
    }
    context->agps_callbacks = *callbacks;

    LOGD("%s: exit", __FUNCTION__);
}

/* Function is called from java API when AGPS status
   is changed with GPS_REQUEST_AGPS_DATA_CONN
   DONT update AGPS status here!
*/
int mbm_agps_data_conn_open(const char *apn)
{
    LOGD("%s: enter, apn: %s", __FUNCTION__, apn);

    LOGD("%s: exit", __FUNCTION__);
    return 0;
}

/* Function is called from java API when AGPS status
   is changed with GPS_RELEASE_AGPS_DATA_CONN
   DONT update AGPS status here!
*/
int mbm_agps_data_conn_closed(void)
{
    LOGD("%s: enter", __FUNCTION__);

    LOGD("%s: exit", __FUNCTION__);
    return 0;
}

/* Function is called from java API when AGPS status
   is changed with GPS_RELEASE_AGPS_DATA_CONN
   DONT update AGPS status here!
*/
int mbm_agps_data_conn_failed(void)
{
    LOGD("%s: enter", __FUNCTION__);

    LOGD("%s: exit", __FUNCTION__);
    return 0;
}

int mbm_agps_set_server(AGpsType type, const char *hostname, int port)
{
    LOGD("%s: enter hostname: %s %d %s", __FUNCTION__, hostname, port,
         type == AGPS_TYPE_SUPL ? "SUPL" : "C2K");

    /* This is strange we get C2K if in flight mode?!?!? */

    /* store the server to be used in the supl config */
    gpsctrl_set_supl_server((char *)hostname);
    
    LOGD("%s: exit", __FUNCTION__);
    return 0;
}

/** Extended interface for AGPS support. */
static const AGpsInterface mbmAGpsInterface = {
    sizeof(AGpsInterface),
    /**
     * Opens the AGPS interface and provides the callback routines
     * to the implemenation of this interface.
     */
    mbm_agps_init,
    /**
     * Notifies that a data connection is available and sets 
     * the name of the APN to be used for SUPL.
     */
    mbm_agps_data_conn_open,
    /**
     * Notifies that the AGPS data connection has been closed.
     */
    mbm_agps_data_conn_closed,
    /**
     * Notifies that a data connection is not available for AGPS. 
     */
    mbm_agps_data_conn_failed,
    /**
     * Sets the hostname and port for the AGPS server.
     */
    mbm_agps_set_server,
};


/*
  AGpsRil callbacks:
  agps_ril_request_set_id request_setid;
  agps_ril_request_ref_loc request_refloc;
  gps_create_thread create_thread_cb;

*/
void mbm_agpsril_init(AGpsRilCallbacks * callbacks)
{
    ENTER;
    if (callbacks == NULL)
        LOGE("callback null");
    EXIT;

}

static const char *agps_refid_str(int type)
{
    switch (type) {
    case AGPS_REF_LOCATION_TYPE_GSM_CELLID:
        return "AGPS_REF_LOCATION_TYPE_GSM_CELLID";
    case AGPS_REF_LOCATION_TYPE_UMTS_CELLID:
        return "AGPS_REF_LOCATION_TYPE_UMTS_CELLID";
    case AGPS_REG_LOCATION_TYPE_MAC:
        return "AGPS_REG_LOCATION_TYPE_MAC";
    default:
        return "UNKNOWN AGPS_REF_LOCATION";
    }
}

void
mbm_agpsril_set_ref_location(const AGpsRefLocation * agps_reflocation,
                             size_t sz_struct)
{
    ENTER;
    if (sz_struct != sizeof(AGpsRefLocation))
        LOGE("AGpsRefLocation struct incorrect size");

    LOGD("AGpsRefLocation.type=%s sie=%d",
         agps_refid_str(agps_reflocation->type), sz_struct);

    EXIT;
}

static const char *agps_setid_str(int type)
{
    switch (type) {
    case AGPS_SETID_TYPE_NONE:
        return "AGPS_SETID_TYPE_NONE";
    case AGPS_SETID_TYPE_IMSI:
        return "AGPS_SETID_TYPE_IMSI";
    case AGPS_SETID_TYPE_MSISDN:
        return "AGPS_SETID_TYPE_MSISDN";
    default:
        return "UNKNOWN AGPS_SETID_TYPE";
    }
}

void mbm_agpsril_set_set_id(AGpsSetIDType type, const char *setid)
{
    ENTER;
    LOGD("type=%s setid=%s", agps_setid_str(type), setid);
    EXIT;
}

void mbm_agpsril_ni_message(uint8_t * msg, size_t len)
{
    ENTER;
    LOGD("msg@%p len=%d", msg, len);
    EXIT;
}


void
mbm_agpsril_update_network_state(int connected, int type, int roaming,
                                 const char *extra_info)
{
    GpsContext *context = get_gps_context();
    ENTER;
    LOGD("connected=%i type=%i roaming=%i, extra_info=%s",
         connected, type, roaming, extra_info);
    context->ril_connected = connected;
    context->ril_roaming = roaming;
    context->ril_type = type;
    if (context->gps_initiated) {
        gpsctrl_set_is_roaming(roaming);
        gpsctrl_set_is_connected(connected);
    }
    EXIT;

}


static void mbm_agpsril_update_network_availability (int avaiable, const char* apn) {
    LOGI("%s", __func__);
}

/* Not implemented just for debug*/
static const AGpsRilInterface mbmAGpsRilInterface = {
    sizeof(AGpsRilInterface),
    /**
     * Opens the AGPS interface and provides the callback routines
     * to the implemenation of this interface.
     */
    mbm_agpsril_init,
    /**
     * Sets the reference location.
     */
    mbm_agpsril_set_ref_location,
    /**
     * Sets the set ID.
     */
    mbm_agpsril_set_set_id,
    /**
     * Send network initiated message.
     */
    mbm_agpsril_ni_message,
    /**
     * Notify GPS of network status changes.
     * These parameters match values in the android.net.NetworkInfo class.
     */
    mbm_agpsril_update_network_state,

    /**
    * Notify GPS of network status changes.
    * These parameters match values in the android.net.NetworkInfo class.
    */
    mbm_agpsril_update_network_availability
};

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       M A I N  I N T E R F A C E                      *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/
static int jul_days(struct tm tm_day)
{
    return
        367 * (tm_day.tm_year + 1900) -
        floor(7 *
              (floor((tm_day.tm_mon + 10) / 12) +
               (tm_day.tm_year + 1900)) / 4) -
        floor(3 *
              (floor
               ((floor((tm_day.tm_mon + 10) / 12) +
                 (tm_day.tm_year + 1899)) / 100) + 1) / 4) +
        floor(275 * (tm_day.tm_mon + 1) / 9) + tm_day.tm_mday + 1721028 -
        2400000;
}

static void utc_to_gps(const time_t time, int *tow, int *week)
{
    struct tm tm_utc;
    struct tm tm_gps;
    int day, days_cnt;

    if (tow == NULL || week == NULL) {
        LOGE("%s: tow/week null", __FUNCTION__);
        return;
    }
    gmtime_r(&time, &tm_utc);
    tm_gps.tm_year = 80;
    tm_gps.tm_mon = 0;
    tm_gps.tm_mday = 6;

    days_cnt = jul_days(tm_utc) - jul_days(tm_gps);
    day = days_cnt % 7;
    *week = floor(days_cnt / 7);
    *tow = (day * 86400) + ((tm_utc.tm_hour * 60) +
                            tm_utc.tm_min) * 60 + tm_utc.tm_sec;
}

/* set the command to be handled from the main loop */
static void add_pending_command(char cmd)
{
    GpsContext *context = get_gps_context();
    write(context->control_fd[0], &cmd, 1);
}

static void nmea_received(char *line)
{
    GpsContext *context = get_gps_context();

    if (line == NULL) {
        LOGE("%s: line null", __FUNCTION__);
        return;
    }

    LOGD("%s: %s", __FUNCTION__, (char *) line);

    if ((strstr(line, "$GP") != NULL) && (strlen(line) > 3))
        nmea_reader_add(context->reader, (char *) line);
}

static int epoll_register(int epoll_fd, int fd)
{
    struct epoll_event ev;
    int ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev);
    }
    while (ret < 0 && errno == EINTR);

    return ret;
}

static int epoll_deregister(int epoll_fd, int fd)
{
    int ret;

    do {
        ret = epoll_ctl(epoll_fd, EPOLL_CTL_DEL, fd, NULL);
    }
    while (ret < 0 && errno == EINTR);

    return ret;
}

static void onATReaderClosed(void)
{
    LOGI("AT channel closed\n");
    add_pending_command(CMD_DEV_LOST);
}

static int safe_read(int fd, char *buf, int count)
{
    return read(fd, buf, count);
}

/**
 * Wait until the module reports EMRDY
 */
static int wait_for_emrdy (int fd, int timeout)
{
    struct pollfd fds[1];
    char start[MAX_AT_RESPONSE];
    int n, len, time;
    GpsContext *context = get_gps_context();
    char *at_emrdy = "AT*EMRDY?\r\n";

    LOGI("%s: waiting for EMRDY...", __FUNCTION__);

    fds[0].fd = fd;
    fds[0].events = POLLIN;

    time = 0;
    while (1) {
        /* dump EMRDY? on the line every EMRDY_POLL_INTERVAL to poll if no unsolicited is received
         * don't care if it doesn't work
         */
        if ((time % EMRDY_POLL_INTERVAL) == 0)
            write(fd, at_emrdy, strlen(at_emrdy));

        n = poll(fds, 1, TIMEOUT_POLL);

        if (n < 0) {
            LOGE("%s: Poll error", __FUNCTION__);
            return -1;
        } else if ((n > 0) && (fds[0].revents & (POLLIN | POLLERR))) {
            LOGD("%s, got an event", __FUNCTION__);
            memset(start, 0, MAX_AT_RESPONSE);
            len = safe_read(fd, start, MAX_AT_RESPONSE - 1);

            if (start == NULL)
                LOGI("%s: Eiii empty string", __FUNCTION__);
            else if (strstr(start, "EMRDY: 1") == NULL)
                LOGI("%s: Eiii this was not EMRDY: %s", __FUNCTION__, start);
            else
                break;

            /* sleep a while to not increase the time variable to quickly */
            usleep(TIMEOUT_POLL * 1000);
        }

        if (context->cleanup_requested) {
            LOGD("%s, aborting because of cleanup requested", __FUNCTION__);
            return CLEANUP_REQUESTED;
        } else if (time >= timeout) {
            LOGE("%s: timeout, go ahead anyway(might work)...", __FUNCTION__);
            return 0;
        }

        time += TIMEOUT_POLL;
    }

    if (context->cleanup_requested) {
        LOGD("%s: Got EMRDY but aborting because of cleanup requested", __FUNCTION__);
        return CLEANUP_REQUESTED;
    } else {
        LOGI("%s: Got EMRDY", __FUNCTION__);
        return 0;
    }
}

/* open ctrl device */
int open_ctrl_device(void)
{
    int ctrl_fd;
    char *ctrl_dev = gpsctrl_get_ctrl_device();
    GpsContext *context = get_gps_context();

    LOGD("%s trying to open device", __FUNCTION__);
    while (1) {
        if (context->cleanup_requested) {
            LOGD("%s, aborting because of cleanup", __FUNCTION__);
            return CLEANUP_REQUESTED;
        }
        ctrl_fd = open(ctrl_dev, O_RDWR | O_NONBLOCK);
        if (ctrl_fd < 0) {
            LOGD("%s, ctrl_fd < 0, dev:%s, error:%s", __FUNCTION__, ctrl_dev, strerror(errno));
            LOGD("Trying again");
            sleep(1);
        } else
            break;
    }

    if (strstr(ctrl_dev, "/dev/ttyA")) {
        struct termios ios;
        LOGD("%s, flushing device", __FUNCTION__);
        /* Clear the struct and then call cfmakeraw*/
        tcflush(ctrl_fd, TCIOFLUSH);
        tcgetattr(ctrl_fd, &ios);
        memset(&ios, 0, sizeof(struct termios));
        cfmakeraw(&ios);
        /* OK now we are in a known state, set what we want*/
        ios.c_cflag |= CRTSCTS;
        /* ios.c_cc[VMIN]  = 0; */
        /* ios.c_cc[VTIME] = 1; */
        ios.c_cflag |= CS8;
        tcsetattr(ctrl_fd, TCSANOW, &ios);
        tcflush(ctrl_fd, TCIOFLUSH);
        tcsetattr(ctrl_fd, TCSANOW, &ios);
        tcflush(ctrl_fd, TCIOFLUSH);
        tcflush(ctrl_fd, TCIOFLUSH);
        cfsetospeed(&ios, B115200);
        cfsetispeed(&ios, B115200);
        tcsetattr(ctrl_fd, TCSANOW, &ios);

        fcntl(ctrl_fd, F_SETFL, 0);
    }

    if (wait_for_emrdy(ctrl_fd, TIMEOUT_EMRDY) == CLEANUP_REQUESTED)
        return CLEANUP_REQUESTED;
    else
        return ctrl_fd;
}


/* this loop is needed to be able to run callbacks in
 * the correct thread, created with the create_thread callback
 */
static void main_loop(void *arg)
{
    int epoll_fd = epoll_create(1);
    char cmd = 255;
    char nmea[MAX_NMEA_LENGTH];
    int ret;
    int ctrl_fd;
    int nmea_fd = -1;
    int device_lost;
    GpsContext *context = get_gps_context();
    int control_fd = context->control_fd[1];
    (void) arg;

    LOGD("Starting main loop");

    while (1) {
        device_lost = 0;
        ctrl_fd = open_ctrl_device();
        if (ctrl_fd == CLEANUP_REQUESTED)
            goto exit;
        else if (ctrl_fd < 0) {
            LOGE("Error opening ctrl device");
            goto error;
        }

        ret = gpsctrl_open(ctrl_fd, onATReaderClosed);
        if (ret < 0) {
            LOGE("Error opening ctrl device");
            goto error;
        }

        context->gps_status.status = GPS_STATUS_ENGINE_ON;
        add_pending_command(CMD_STATUS_CB);

        gpsctrl_set_device_is_ready(1);

        gpsctrl_set_position_mode(context->pref_mode, 1);

        ret = gpsctrl_init_supl(context->allow_uncert, context->enable_ni);
        if (ret < 0)
            LOGE("Error initing supl");

        ret = gpsctrl_init_pgps();
        if (ret < 0)
            LOGE("Error initing pgps");

        /* temporarily removed because of issues with MbmService */
        /*
        LOGD("Requesting status from MbmService");
        service_handler_send_message(CMD_SEND_ALL_INFO, "");
        */

        pthread_mutex_lock(&context->mutex);
        if (context->gps_should_start || context->gps_started) {
            LOGD("%s, restoring gps state to started", __FUNCTION__);
            mbm_gps_start();
        }
        pthread_mutex_unlock(&context->mutex);

        nmea_fd = gpsctrl_get_nmea_fd();

        epoll_register(epoll_fd, control_fd);
        epoll_register(epoll_fd, nmea_fd);

        while (!device_lost) {
            struct epoll_event event;
            int nevents;

            nevents = epoll_wait(epoll_fd, &event, 1, -1);
            if (nevents < 0 && errno != EINTR) {
                LOGE("epoll_wait() unexpected error: %s", strerror(errno));
                continue;
            }

            if ((event.events & (EPOLLERR | EPOLLHUP)) != 0) {
                LOGE("EPOLLERR or EPOLLHUP after epoll_wait()!");
                if (event.data.fd == nmea_fd) {
                    LOGD("Device lost. Will try to recover.");
                    break;
                }
                goto error;
            }

            if ((event.events & EPOLLIN) != 0) {
                int fd = event.data.fd;

                if (fd == control_fd) {

                    do {
                        ret = read(fd, &cmd, 1);
                    }
                    while (ret < 0 && errno == EINTR);

                    switch (cmd) {
                        LOGD("%s cmd %d", __FUNCTION__, (int) cmd);
                    case CMD_STATUS_CB:
                        context->status_callback(&context->gps_status);
                        break;
                    case CMD_AGPS_STATUS_CB:
                        context->agps_callbacks.status_cb(&context->agps_status);
                        break;
                    case CMD_NI_CB:
                        LOGD("%s: CMD_NI_CB", __FUNCTION__);
                        context->ni_callback(&context->notification);
                        LOGD("%s: CMD_NI_CB sent", __FUNCTION__);
                        break;
                    case CMD_DEV_LOST:
                        LOGD("%s: CMD_DEV_LOST, will try to recover.", __FUNCTION__);
                        device_lost = 1;
                        break;
                    case CMD_QUIT:
                        goto exit;
                        break;
                    default:
                        break;
                    }
                } else if (fd == nmea_fd) {
                    nmea_read(nmea_fd, nmea);
                    nmea_received(nmea);
                } else {
                    LOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }

        gpsctrl_set_device_is_ready(0);
        gpsctrl_cleanup();
        epoll_deregister(epoll_fd, control_fd);
        epoll_deregister(epoll_fd, nmea_fd);

        context->gps_status.status = GPS_STATUS_ENGINE_OFF;
        add_pending_command(CMD_STATUS_CB);
    }

  error:
    LOGE("main loop terminated unexpectedly!");
  exit:
    pthread_mutex_lock(&context->cleanup_mutex);
    epoll_deregister(epoll_fd, control_fd);
    epoll_deregister(epoll_fd, nmea_fd);
    LOGD("Main loop quitting");
    pthread_cond_signal(&context->cleanup_cond);
    pthread_mutex_unlock(&context->cleanup_mutex);
}

static void get_properties(void)
{
    char prop[PROPERTY_VALUE_MAX];
    char nmea_dev[PROPERTY_VALUE_MAX];
    char ctrl_dev[PROPERTY_VALUE_MAX];
    int len;
    GpsContext *context = get_gps_context();

    LOGD("%s: enter", __FUNCTION__);

    len = property_get("mbm.gps.config.gps_pref_mode", prop, "");
    if (strstr(prop, PROP_SUPL)) {
        LOGD("Setting preferred agps mode to SUPL");
        context->pref_mode = MODE_SUPL;
    } else if (strstr(prop, PROP_PGPS)) {
        LOGD("Setting preferred agps mode to PGPS");
        context->pref_mode = MODE_PGPS;
    } else if (strstr(prop, PROP_STANDALONE)) {
        LOGD("Setting preferred agps mode to STANDALONE");
        context->pref_mode = MODE_STAND_ALONE;
    } else {
        LOGD("Setting preferred agps mode to PGPS (prop %s)",
             prop);
        context->pref_mode = MODE_PGPS;
    }

    len = property_get("mbm.gps.config.supl.enable_ni", prop, "no");
    if (strstr(prop, "yes")) {
        LOGD("Enabling network initiated requests");
        context->enable_ni = 1;
    } else {
        LOGD("Disabling network initiated requests");
        context->enable_ni = 0;
    }

    len = property_get("mbm.gps.config.supl.uncert", prop, "no");
    if (strstr(prop, "yes")) {
        LOGD("Allowing unknown certificates");
        context->allow_uncert = 1;
    } else {
        LOGD("Not allowing unknown certificates");
        context->allow_uncert = 0;
    }

    len = property_get("mbm.gps.config.gps_ctrl", ctrl_dev, "");
    if (len)
        LOGD("Using gps ctrl device: %s", ctrl_dev);
    else {
        LOGD("No gps ctrl device set, using the default instead.");
        snprintf(ctrl_dev, PROPERTY_VALUE_MAX, "%s", DEFAULT_CTRL_PORT);
    }

    len = property_get("mbm.gps.config.gps_nmea", nmea_dev, "");
    if (len)
        LOGD("Using gps nmea device: %s", nmea_dev);
    else {
        LOGD("No gps nmea device set, using the default instead.");
        snprintf(nmea_dev, PROPERTY_VALUE_MAX, "%s", DEFAULT_NMEA_PORT);
    }

    gpsctrl_set_devices(ctrl_dev, nmea_dev);
}

static int mbm_gps_init(GpsCallbacks * callbacks)
{
    int ret;
    GpsContext *context = get_gps_context();
    pthread_mutexattr_t mutex_attr;

    LOGD("MBM-GPS version: %s", MBM_GPS_VERSION);

    context->gps_started = 0;
    context->gps_initiated = 0;
    context->clear_flag = CLEAR_AIDING_DATA_NONE;
    context->cleanup_requested = 0;
    context->gps_should_start = 0;

    if (callbacks == NULL) {
        LOGE("%s, callbacks null", __FUNCTION__);
        return -1;
    }

    if (callbacks->set_capabilities_cb) {
        callbacks->set_capabilities_cb(GPS_CAPABILITY_SCHEDULING |
                                       GPS_CAPABILITY_MSB |
                                       GPS_CAPABILITY_MSA |
                                       GPS_CAPABILITY_SINGLE_SHOT);
    } else {
        LOGE("%s capabilities_cb is null", __FUNCTION__);
    }

    context->agps_status.size = sizeof(AGpsStatus);

    nmea_reader_init(context->reader);

    context->control_fd[0] = -1;
    context->control_fd[1] = -1;
    if (socketpair(AF_LOCAL, SOCK_STREAM, 0, context->control_fd) < 0) {
        LOGE("could not create thread control socket pair: %s",
             strerror(errno));
        return -1;
    }

    context->status_callback = callbacks->status_cb;

    context->create_thread_callback = callbacks->create_thread_cb;

    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&context->mutex, &mutex_attr);

    pthread_mutex_init(&context->cleanup_mutex, NULL);
    pthread_cond_init(&context->cleanup_cond, NULL);

    /* initialize mbm service handler */
    LOGD("%s pre init service handler", __FUNCTION__);
    if (service_handler_init() < 0)
        LOGD("%s, error initializing service handler", __FUNCTION__);
    LOGD("%s post init service handler", __FUNCTION__);

    ret = gpsctrl_init();
    if (ret < 0)
        LOGE("Error initing gpsctrl lib");

    get_properties();
    gpsctrl_set_supl_ni_callback(mbm_gps_ni_request);
    if (ret < 0) {
        LOGE("Error setting devices");
        return -1;
    }

    /* main thread must be started prior to setting nmea_reader callbacks */
    context->main_thread = context->create_thread_callback("mbm_main_thread",
                                                     main_loop, NULL);

    nmea_reader_set_callbacks(context->reader, callbacks);

    context->gps_initiated = 1;

    LOGD("%s: exit", __FUNCTION__);
    return 0;
}

static void mbm_gps_cleanup(void)
{
    GpsContext *context = get_gps_context();

    LOGD("%s: enter", __FUNCTION__);

    context->gps_initiated = 0;

    LOGD("%s, waiting for main thread to exit", __FUNCTION__);
    pthread_mutex_lock(&context->cleanup_mutex);

    add_pending_command(CMD_QUIT);
    context->cleanup_requested = 1;

    pthread_cond_wait(&context->cleanup_cond, &context->cleanup_mutex);

    LOGD("%s, stopping service handler", __FUNCTION__);
    service_handler_stop();

    LOGD("%s, cleanup gps ctrl", __FUNCTION__);
    gpsctrl_cleanup();

    pthread_mutex_unlock(&context->cleanup_mutex);

    pthread_mutex_destroy(&context->mutex);
    pthread_mutex_destroy(&context->cleanup_mutex);
    pthread_cond_destroy(&context->cleanup_cond);

    LOGD("%s: exit", __FUNCTION__);
}


static int mbm_gps_start(void)
{
    int err;
    GpsContext *context = get_gps_context();

    LOGD("%s: enter", __FUNCTION__);

    pthread_mutex_lock(&context->mutex);

    if (!gpsctrl_get_device_is_ready()) {
        LOGD("%s, device is not ready. Deferring start of gps.", __FUNCTION__);
        context->gps_should_start = 1;
        pthread_mutex_unlock(&context->mutex);
        return 0;
    }

    err = gpsctrl_start();
    if (err < 0) {
        LOGE("Error starting gps");
        pthread_mutex_unlock(&context->mutex);
        return -1;
    }

    context->gps_started = 1;

    context->gps_status.status = GPS_STATUS_SESSION_BEGIN;
    add_pending_command(CMD_STATUS_CB);

    LOGD("%s: exit 0", __FUNCTION__);
    pthread_mutex_unlock(&context->mutex);
    return 0;
}

static int mbm_gps_stop(void)
{
    int err;
    GpsContext *context = get_gps_context();
    LOGD("%s: enter", __FUNCTION__);

    pthread_mutex_lock(&context->mutex);

    if (!gpsctrl_get_device_is_ready()) {
        LOGD("%s, device not ready.", __FUNCTION__);
    } else {
        err = gpsctrl_stop();
        if (err < 0)
            LOGE("Error stopping gps");
    }

    context->gps_started = 0;
    context->gps_should_start = 0;

    context->gps_status.status = GPS_STATUS_SESSION_END;
    add_pending_command(CMD_STATUS_CB);

    if (context->clear_flag > CLEAR_AIDING_DATA_NONE) {
        LOGD("%s: Executing deferred operation of deleting aiding data", __FUNCTION__);
        gpsctrl_delete_aiding_data(context->clear_flag);
        context->clear_flag = CLEAR_AIDING_DATA_NONE;
    }

    LOGD("%s: exit 0", __FUNCTION__);
    pthread_mutex_unlock(&context->mutex);
    return 0;
}


/* Not implemented just debug*/
static int
mbm_gps_inject_time(GpsUtcTime time, int64_t timeReference,
                    int uncertainty)
{
    char buff[100];
    int tow, week;
    time_t s_time;
    (void) timeReference;
    (void) uncertainty;

    LOGD("%s: enter", __FUNCTION__);

    s_time = time / 1000;
    utc_to_gps(s_time, &tow, &week);
    memset(buff, 0, 100);
    snprintf(buff, 98, "AT*E2GPSTIME=%d,%d\r", tow, week);
    LOGD("%s: %s", __FUNCTION__, buff);

    LOGD("%s: exit 0", __FUNCTION__);
    return 0;
}

/** Injects current location from another location provider
 *  (typically cell ID).
 *  latitude and longitude are measured in degrees
 *  expected accuracy is measured in meters
 */
static int
mbm_gps_inject_location(double latitude, double longitude, float accuracy)
{

    LOGD("%s: lat = %f , lon = %f , acc = %f", __FUNCTION__, latitude,
         longitude, accuracy);
    return 0;
}

static void mbm_gps_delete_aiding_data(GpsAidingData flags)
{
    GpsContext *context = get_gps_context();
    int clear_flag = CLEAR_AIDING_DATA_NONE;

    LOGD("%s: enter", __FUNCTION__);

    /* Support exist for EPHEMERIS and ALMANAC only
     * where only EPHEMERIS can be cleared by it self.
     */
    if ((GPS_DELETE_EPHEMERIS | GPS_DELETE_ALMANAC) ==
        ((GPS_DELETE_EPHEMERIS | GPS_DELETE_ALMANAC) & flags))
        clear_flag = CLEAR_AIDING_DATA_ALL;
    else if (GPS_DELETE_EPHEMERIS == (GPS_DELETE_EPHEMERIS & flags))
        clear_flag = CLEAR_AIDING_DATA_EPH_ONLY;
    else
        LOGI("Parameters not supported for deleting");

    if (!context->gps_started)
        gpsctrl_delete_aiding_data(clear_flag);
    else {
        LOGD("%s, Deferring operation of deleting aiding data until gps is stopped", __FUNCTION__);
        context->clear_flag = clear_flag;
    }

    LOGD("%s: exit 0", __FUNCTION__);
}

static char *get_mode_name(int mode)
{
    switch (mode) {
    case GPS_POSITION_MODE_STANDALONE:
        return "GPS_POSITION_MODE_STANDALONE";
    case GPS_POSITION_MODE_MS_BASED:
        return "GPS_POSITION_MODE_MS_BASED";
    case GPS_POSITION_MODE_MS_ASSISTED:
        return "GPS_POSITION_MODE_MS_ASSISTED";
    default:
        return "UNKNOWN MODE";
    }
}

static int
mbm_gps_set_position_mode(GpsPositionMode mode,
                          GpsPositionRecurrence recurrence,
                          uint32_t min_interval,
                          uint32_t preferred_accuracy,
                          uint32_t preferred_time)
{
    GpsContext *context = get_gps_context();
    int new_mode = MODE_STAND_ALONE;
    int interval;
    (void) preferred_accuracy;

    LOGD("%s:enter  %s min_interval = %d recurrence=%d pref=%d",
         __FUNCTION__, get_mode_name(mode), min_interval, recurrence,
         preferred_time);

    switch (mode) {
    case GPS_POSITION_MODE_MS_ASSISTED:
    case GPS_POSITION_MODE_MS_BASED:
        if (context->pref_mode == MODE_SUPL)
            new_mode = MODE_SUPL;
        else if (context->pref_mode == MODE_STAND_ALONE)
            /* override Androids choice since stand alone
             * has been explicitly selected via a property
             */
            new_mode = MODE_STAND_ALONE;
        else
            new_mode = MODE_PGPS;
        break;
    case GPS_POSITION_MODE_STANDALONE:
        new_mode = MODE_STAND_ALONE;
        break;
    default:
        break;
    }

    if (recurrence == GPS_POSITION_RECURRENCE_SINGLE || min_interval == SINGLE_SHOT_INTERVAL)
        interval = 0;
    else if (min_interval < 1000)
        interval = 1;
    else
        interval = min_interval / 1000;

    gpsctrl_set_position_mode(new_mode, interval);

    LOGD("%s: exit 0", __FUNCTION__);
    return 0;
}

static const void *mbm_gps_get_extension(const char *name)
{
    LOGD("%s: enter name=%s", __FUNCTION__, name);

    if (name == NULL)
        return NULL;

    LOGD("%s, querying %s", __FUNCTION__, name);

    if (!strncmp(name, AGPS_INTERFACE, 10)) {
        LOGD("%s: exit &mbmAGpsInterface", __FUNCTION__);
        return &mbmAGpsInterface;
    }

    if (!strncmp(name, AGPS_RIL_INTERFACE, 10)) {
        LOGD("%s: exit &mbmAGpsRilInterface", __FUNCTION__);
        return &mbmAGpsRilInterface;
    }

    if (!strncmp(name, GPS_NI_INTERFACE, 10)) {
        LOGD("%s: exit &mbmGpsNiInterface", __FUNCTION__);
        return &mbmGpsNiInterface;
    }

    LOGD("%s: exit NULL", __FUNCTION__);
    return NULL;
}

/** Represents the standard GPS interface. */
static const GpsInterface mbmGpsInterface = {
    sizeof(GpsInterface),
        /**
	 * Opens the interface and provides the callback routines
	 * to the implemenation of this interface.
	 */
    mbm_gps_init,

        /** Starts navigating. */
    mbm_gps_start,

        /** Stops navigating. */
    mbm_gps_stop,

        /** Closes the interface. */
    mbm_gps_cleanup,

        /** Injects the current time. */
    mbm_gps_inject_time,

        /** Injects current location from another location provider
	 *  (typically cell ID).
	 *  latitude and longitude are measured in degrees
	 *  expected accuracy is measured in meters
	 */
    mbm_gps_inject_location,

        /**
	 * Specifies that the next call to start will not use the
	 * information defined in the flags. GPS_DELETE_ALL is passed for
	 * a cold start.
	 */

    mbm_gps_delete_aiding_data,
        /**
	 * fix_frequency represents the time between fixes in seconds.
	 * Set fix_frequency to zero for a single-shot fix.
	 */
    mbm_gps_set_position_mode,

        /** Get a pointer to extension information. */
    mbm_gps_get_extension,
};

const GpsInterface *gps_get_hardware_interface(void)
{
    LOGD("gps_get_hardware_interface");
    return &mbmGpsInterface;
}

/* This is for Gingerbread */
const GpsInterface *mbm_get_gps_interface(struct gps_device_t *dev)
{
    (void) dev;
    LOGD("mbm_gps_get_hardware_interface");
    return &mbmGpsInterface;
}

static int
mbm_open_gps(const struct hw_module_t *module,
             char const *name, struct hw_device_t **device)
{
    struct gps_device_t *dev;
    (void) name;

    LOGD("%s: enter", __FUNCTION__);

    if (module == NULL) {
        LOGE("%s: module null", __FUNCTION__);
        return -1;
    }

    dev = malloc(sizeof(struct gps_device_t));
    if (!dev) {
        LOGE("%s: malloc fail", __FUNCTION__);
        return -1;
    }
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t *) module;
    dev->get_gps_interface = mbm_get_gps_interface;

    *device = (struct hw_device_t *) dev;

    LOGD("%s: exit", __FUNCTION__);
    return 0;
}

static struct hw_module_methods_t mbm_gps_module_methods = {
    .open = mbm_open_gps
};

const struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "MBM GPS",
    .author = "Ericsson",
    .methods = &mbm_gps_module_methods,
};
