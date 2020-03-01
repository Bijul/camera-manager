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
#include <assert.h>
#include <sstream>
#include <unistd.h>
#include <vector>

#include "CameraParameters.h"
#include "ImageCaptureRaspistill.h"

#include "log.h"

#define DEFAULT_IMAGE_FILE_FORMAT CameraParameters::IMAGE_FILE_JPEG
#define DEFAULT_FILE_PATH "/tmp/"
#define V4L2_DEVICE_PREFIX "/dev/"

int ImageCaptureRaspistill::imgCount = 0;

ImageCaptureRaspistill::ImageCaptureRaspistill(std::shared_ptr<CameraDevice> camDev)
    : mCamDev(camDev)
    , mState(STATE_IDLE)
    , mWidth(0)
    , mHeight(0)
    , mFormat(DEFAULT_IMAGE_FILE_FORMAT)
    , mInterval(0)
    , mPath(DEFAULT_FILE_PATH)
    , mResultCB(nullptr)
{

    mCamDev->getSize(mCamWidth, mCamHeight);
    mCamDev->getPixelFormat(mCamPixFormat);
}

ImageCaptureRaspistill::ImageCaptureRaspistill(std::shared_ptr<CameraDevice> camDev,
                                 struct ImageSettings &imgSetting)
    : mCamDev(camDev)
    , mState(STATE_IDLE)
    , mWidth(imgSetting.width)
    , mHeight(imgSetting.height)
    , mFormat(imgSetting.fileFormat)
    , mInterval(0)
    , mPath(DEFAULT_FILE_PATH)
    , mResultCB(nullptr)
{
    log_info("%s Device:%s with settings", __func__, mCamDev->getDeviceId().c_str());

    mCamDev->getSize(mCamWidth, mCamHeight);
    mCamDev->getPixelFormat(mCamPixFormat);
}

ImageCaptureRaspistill::~ImageCaptureRaspistill()
{
    stop();
}

int ImageCaptureRaspistill::init()
{
    log_info("%s::%s", typeid(this).name(), __func__);

    if (getState() != STATE_IDLE) {
        log_error("Invalid State : %d", getState());
        return -1;
    }

    setState(STATE_INIT);
    return 0;
}

int ImageCaptureRaspistill::uninit()
{
    log_info("%s::%s", typeid(this).name(), __func__);

    if (getState() != STATE_INIT && getState() != STATE_ERROR) {
        log_error("Invalid State : %d", getState());
        return -1;
    }

    setState(STATE_IDLE);
    return 0;
}

int ImageCaptureRaspistill::start(int interval, int count, std::function<void(int result, int seq_num)> cb)
{
    int ret = 0;
    log_info("%s::%s interval:%d count:%d", typeid(this).name(), __func__, interval, count);
    // Invalid Arguments
    // Either the capture is count based or interval based or count with interval
    if (count <= 0 && interval <= 0) {
        log_error("Invalid Parameters");
        return 1;
    }

    // check & set state
    if (getState() != STATE_INIT) {
        log_error("Invalid State : %d", getState());
        return -1;
    }

    mResultCB = cb;
    mInterval = interval;
    setState(STATE_RUN);

    if (count == 1) {
        // There will be no stop call
        ret = click();
        setState(STATE_INIT);
        if (mResultCB)
            mResultCB(ret, 1);
    } else {
        // create a thread to capture images
        mThread = std::thread(&ImageCaptureRaspistill::captureThread, this, count);
    }

    return 0;
}

int ImageCaptureRaspistill::stop()
{
    log_info("%s::%s", typeid(this).name(), __func__);

    setState(STATE_INIT);

    if (mThread.joinable())
        mThread.join();

    return 0;
}

int ImageCaptureRaspistill::setState(int state)
{
    int ret = 0;
    log_debug("%s : %d", __func__, state);

    if (mState == state)
        return 0;

    if (state == STATE_ERROR) {
        mState = state;
        return 0;
    }

    switch (mState) {
    case STATE_IDLE:
        if (state == STATE_INIT)
            mState = state;
        break;
    case STATE_INIT:
        if (state == STATE_IDLE || state == STATE_RUN)
            mState = state;
        break;
    case STATE_RUN:
        if (state == STATE_INIT)
            mState = state;
        break;
    case STATE_ERROR:
        log_info("In Error State");
        // Free up resources, restart?
        if (state == STATE_IDLE)
            mState = state;
        break;
    default:
        break;
    }

    if (mState != state) {
        ret = -1;
        log_error("InValid State Transition");
    }

    return ret;
}

int ImageCaptureRaspistill::getState()
{
    return mState;
}

int ImageCaptureRaspistill::setResolution(int imgWidth, int imgHeight)
{
    mWidth = imgWidth;
    mHeight = imgHeight;

    return 0;
}

int ImageCaptureRaspistill::setInterval(int interval)
{
    if (interval < 0)
        return -1;

    mInterval = interval;

    return 0;
}

int ImageCaptureRaspistill::getInterval()
{
    return mInterval;
}

int ImageCaptureRaspistill::setFormat(CameraParameters::IMAGE_FILE_FORMAT imgFormat)
{
    if (imgFormat <= CameraParameters::IMAGE_FILE_MIN
        || imgFormat >= CameraParameters::IMAGE_FILE_MAX) {
        log_error("Invalid Pixel format");
        return 1;
    }

    mFormat = imgFormat;

    return 0;
}

int ImageCaptureRaspistill::setLocation(const std::string imgPath)
{
    // TODO::Check if the path is writeable/valid
    log_debug("%s:%s", __func__, imgPath.c_str());
    mPath = imgPath;

    return 0;
}

void ImageCaptureRaspistill::captureThread(int num)
{
    log_debug("captureThread num:%d int:%d", num, mInterval);
    int ret = -1;
    int count = num;
    int seq_num = 0;
    while (mState == STATE_RUN) {
        ret = click();
        if (getState() != STATE_RUN)
            continue;

        seq_num++;
        if (mResultCB)
            mResultCB(ret, seq_num);

        if (ret) {
            log_error("Error in Image Capture");
            setState(STATE_ERROR);
            continue;
        }

        // Check if the capture is periodic or count(w/wo interval) based
        if (count <= 0) {
            if (mInterval > 0)
                sleep(mInterval);
            else
                break;
        } else {
            log_debug("Current Count : %d", count);
            count--;
            if (count == 0)
                setState(STATE_INIT);
            else {
                if (mInterval > 0)
                    sleep(mInterval);
            }
        }
    }
}

int ImageCaptureRaspistill::click()
{
    log_debug("%s", __func__);

    int ret = 0;
    std::string ext = getImgExt(mFormat);
    if (ext.empty())
        return 0;

    std::stringstream ss;
    ss << "raspistill -t 1 -o " << mPath + "img_" << std::to_string(++imgCount) << "." + ext;
    ret = system(ss.str().c_str());
    return ret;
}

std::string ImageCaptureRaspistill::getImgExt(int format)
{
    switch (format) {
    case CameraParameters::IMAGE_FILE_JPEG:
        return "jpg";
    case CameraParameters::IMAGE_FILE_PNG:
        return "png";
    case CameraParameters::IMAGE_FILE_RAW:
    default:
        return "raw";
    }
}
