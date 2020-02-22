/*
 * This file is part of the Dronecode Camera Manager
 *
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
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
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "log.h"
#include "serial.h"

#define DEFAULT_BUF_SIZE 1024

SerialConnection::SerialConnection()
    : _read_cb([](const struct buffer &buf) {})
    , _write_buf{0, nullptr}
{
}

SerialConnection::~SerialConnection()
{
    free(_write_buf.data);
    close();
}

int SerialConnection::write(const struct buffer &buf)
{
    int r;
    r = _do_write(buf);
    if (r >= 0)
        return r;

    if (_write_buf.data) {
        delete[] _write_buf.data;
        log_warning("Error in writing packet.");
    }

    // write when possible
    _write_buf = {buf.len, new uint8_t[buf.len]};
    memcpy(_write_buf.data, buf.data, buf.len);
    monitor_write(true);

    return 0;
}

void SerialConnection::set_read_callback(std::function<void(const struct buffer &buf)> cb)
{
    _read_cb = cb;
}

bool SerialConnection::_can_read()
{
    struct buffer read_buf {
        DEFAULT_BUF_SIZE, new uint8_t[DEFAULT_BUF_SIZE]
    };

    int r = _do_read(read_buf);

    if (r == -EAGAIN) {
        log_debug("Read packet failed. Trying again.");
        delete[] read_buf.data;
        return true;
    }
    if (r < 0) {
        log_debug("Read failed. Droping packet.");
        delete[] read_buf.data;
        return false;
    }

    if (r > 0) {
        read_buf.len = (unsigned int)r;
        _read_cb(read_buf);
    }

    delete[] read_buf.data;
    return true;
}

bool SerialConnection::_can_write()
{
    int r;

    if (!_write_buf.data)
        return false;

    r = _do_write(_write_buf);
    if (r < 0) {
        log_error("Write package failed. Droping packet.");
    }

    delete[] _write_buf.data;
    _write_buf = {0, nullptr};
    return false;
}

void SerialConnection::close()
{
    if (_fd >= 0)
        ::close(_fd);
    _fd = -1;
}

int SerialConnection::open(struct serial_port sp)
{
    _serial_port = sp;

    // wait for serial port to be initialized
    while(access(_serial_port.port_addr, F_OK) == -1) {
        sleep(2);
    }

    _fd = ::open(_serial_port.port_addr, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (_fd == -1) {
        log_error("Could not create socket (%m)");
        return -1;
    }

    struct termios tc;
    bzero(&tc, sizeof(tc));

    if (tcgetattr(_fd, &tc) != 0) {
        log_error("tcgetattr failed: (%m)");
        close();
        return -1;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    tc.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
    tc.c_cflag |= CS8;

    tc.c_cc[VMIN] = 0; // We are ok with 0 bytes.
    tc.c_cc[VTIME] = 10; // Timeout after 1 second.

    tc.c_cflag |= CLOCAL; // Without this a write() blocks indefinitely.

    if (cfsetispeed(&tc, B57600) != 0) {
        log_error("cfsetispeed failed: (%m)");
        close();
        return -1;
    }

    if (cfsetospeed(&tc, B57600) != 0) {
        log_error("cfsetospeed failed: (%m)");
        close();
        return -1;
    }

    if (tcsetattr(_fd, TCSANOW, &tc) != 0) {
        log_error("cfsetospeed failed: (%m)");
        close();
        return -1;
    }


    log_info("Open serial [%d]", _fd);

    monitor_read(true);
    return _fd;
}

int SerialConnection::_do_read(const struct buffer &buf)
{
    ssize_t r = read(_fd, buf.data, buf.len);
    if (r < -1) {
        log_error("read failed: (%m)");
        return -1;
    }
    return r;
}

int SerialConnection::_do_write(const struct buffer &buf)
{
    if (_fd < 0) {
        log_error("Trying to write to an invalid _fd");
        return -EINVAL;
    }

    ssize_t r = ::write(_fd, buf.data, buf.len);
    if (r == -1) {
        log_error("Error writing (%m)");
        return -1;
    };

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t)buf.len) {
        log_debug("Discarding packet, incomplete write %zd but len=%u", r, buf.len);
    }

    log_debug("Serial: [%d] wrote %zd bytes", _fd, r);

    return r;
}