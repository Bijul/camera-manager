#pragma once

#include "pollable.h"
#include "util.h"

struct serial_port {
    char* port_addr;
    int baud_rate;
};

class SerialConnection : public Pollable {
    public:
        SerialConnection();
        virtual ~SerialConnection();

        int write(const struct buffer &buf);
        void set_read_callback(
            std::function<void(const struct buffer &buf)> cb);

        int open(struct serial_port sp);
        void close();

    protected:
        bool _can_read();
        bool _can_write();
        int _do_write(const struct buffer &buf);
        int _do_read(const struct buffer &buf);

    private:
        std::function<void(const struct buffer &buf)> _read_cb;
        struct buffer _write_buf;
        struct serial_port _serial_port {};
};
