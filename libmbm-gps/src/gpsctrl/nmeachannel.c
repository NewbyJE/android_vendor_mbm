/* Ericsson libgpsctrl
 *
 * Copyright (C) Ericsson AB 2011
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
 *
 * Author: Torgny Johansson <torgny.johansson@ericsson.com>
 *
 */

#include <errno.h>
#include <fcntl.h>
#include <termios.h>

#include "nmeachannel.h"

#define LOG_TAG "libgpsctrl-nmea"
#include "../log.h"

int nmea_read (int fd, char *nmea)
{
    int ret;

    do {
        ret = read(fd, nmea, MAX_NMEA_LENGTH);
    }
    while (ret < 0 && errno == EINTR);

    /* remove \r\n */
    if (strncmp(&nmea[ret - 2], "\r\n", 2)) {
        nmea[0] = '\0';
        return -1;
    }
    nmea[ret - 2] = '\0';

    return 0;
}

static int writeline (int fd, const char *s)
{
    size_t cur = 0;
    size_t len;
    ssize_t written;
    char *cmd = NULL;

    LOGD("NMEA(%d)> %s\n", fd, s);

    len = asprintf(&cmd, "%s\r\n", s);

    /* The main string. */
    while (cur < len) {
        do {
            written = write (fd, cmd + cur, len - cur);
        } while (written < 0 && (errno == EINTR || errno == EAGAIN));

        if (written < 0) {
            free(cmd);
            return -1;
        }

        cur += written;
    }

    free(cmd);

    return 0;
}

int nmea_activate_port (int nmea_fd)
{
    int ret;

    ret = writeline(nmea_fd, "AT*E2GPSNPD");
    if (ret < 0) {
        LOGE("%s, error setting up port for nmea data", __FUNCTION__);
        return -1;
    }

    return 0;
}

int nmea_open (char *dev)
{
    int ret;
    int nmea_fd;
    struct termios ios;

    LOGD("%s", __FUNCTION__);

    nmea_fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (nmea_fd < 0) {
        LOGE("%s, nmea_fd < 0", __FUNCTION__);
        return -1;
    }

	tcflush(nmea_fd, TCIOFLUSH);
	tcgetattr(nmea_fd, &ios);
	memset(&ios, 0, sizeof(struct termios));
	ios.c_cflag |= CRTSCTS;
	ios.c_lflag |= ICANON;
	ios.c_cflag |= CS8;
	tcsetattr(nmea_fd, TCSANOW, &ios);
	tcflush(nmea_fd, TCIOFLUSH);
	tcsetattr(nmea_fd, TCSANOW, &ios);
	tcflush(nmea_fd, TCIOFLUSH);
	tcflush(nmea_fd, TCIOFLUSH);
	cfsetospeed(&ios, B115200);
	cfsetispeed(&ios, B115200);
	tcsetattr(nmea_fd, TCSANOW, &ios);

    /* setup port for receiving nmea data */
    ret = writeline(nmea_fd, "AT");
    if (ret < 0) {
        LOGE("%s, error setting up port for nmea data", __FUNCTION__);
        return -1;
    }

    LOGD("%s, pausing to let nmea port settle", __FUNCTION__);
    sleep(1);

    return nmea_fd;
}

void nmea_close (int fd)
{
    LOGD("%s", __FUNCTION__);
    close(fd);
}
