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
#pragma once

#include <map>
#include <mavlink.h>
#include <memory>
#include <vector>

#include "CameraComponent.h"
#include "conf_file.h"
#include "socket.h"
#include "serial.h"

typedef struct image_callback {
    int comp_id;             /* Component ID */
} image_callback_t;

class MavlinkServer {
public:
    MavlinkServer(const ConfFile &conf);
    ~MavlinkServer();
    void start();
    void stop();
    int addCameraComponent(CameraComponent *camComp);
    void removeCameraComponent(CameraComponent *camComp);
    CameraComponent *getCameraComponent(int compID);

private:
    bool _is_running;
    struct serial_port _serial_port;
    unsigned int _timeout_handler;
    SerialConnection _serial;
    bool _is_sys_id_found;
    int _system_id;
    int _comp_id;
    std::map<int, CameraComponent *> compIdToObj;

    void _message_received(const struct buffer &buf);
    void _handle_mavlink_message(mavlink_message_t *msg);
    void _handle_request_camera_information(mavlink_command_long_t &cmd);
    void _handle_request_camera_settings(mavlink_command_long_t &cmd);
    void _handle_request_storage_information(mavlink_command_long_t &cmd);
    void _handle_set_camera_mode(mavlink_command_long_t &cmd);
    void _handle_image_start_capture(mavlink_command_long_t &cmd);
    void _handle_image_stop_capture(mavlink_command_long_t &cmd);
    void _handle_video_start_capture(mavlink_command_long_t &cmd);
    void _handle_video_stop_capture(mavlink_command_long_t &cmd);
    void _image_captured_cb(image_callback_t cb_data, int result, int seq_num);
    void _handle_request_camera_capture_status(mavlink_command_long_t &cmd);
    void _handle_param_ext_request_read(mavlink_message_t *msg);
    void _handle_param_ext_request_list(mavlink_message_t *msg);
    void _handle_param_ext_set(mavlink_message_t *msg);
    void _handle_reset_camera_settings(mavlink_command_long_t &cmd);
    void _handle_heartbeat(mavlink_message_t *msg);
    bool _send_camera_capture_status(int compid);
    bool _send_mavlink_message(mavlink_message_t &msg);
    void _send_ack(int cmd, int comp_id, bool success);
#if 0
    const Stream::FrameSize *_find_best_frame_size(Stream &s, uint32_t w, uint32_t v);
#endif
    friend bool _heartbeat_cb(void *data);

    CameraParameters::Mode mav2dcmCameraMode(uint32_t mode);
    uint32_t dcm2mavCameraMode(CameraParameters::Mode mode);
};
