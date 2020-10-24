/*
 *             Copyright 2020 by Morgan
 *
 * This software BSD-new. See the included COPYING file for details.
 *
 * License: BSD-new
 * ==============================================================================
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the \<organization\> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "zusbcontext.h"
#include "zusbeventthread.h"
#include "zdebug.h"

#include <assert.h>
#include <thread>
#include <chrono>

/*** ---------------------------==*+*+*==---------------------------------- ***/
ZUSBContext::ZUSBContext()
    : usb_context_owned(true),
      usb_event_thread(nullptr), start_ref_count(0)
{
    int res = libusb_init(&usb_context);
    if ( res != 0 ) {
        zCritical("FATAL: failed to initialize USB: %d",res);
        usb_context_owned = false;
    }
}

ZUSBContext::ZUSBContext(ZUSBContext* _usb_context)
    : usb_context(_usb_context->getUSBContext()),
      usb_context_owned(false),
      usb_event_thread(nullptr),
      start_ref_count(0)
{

}

ZUSBContext::~ZUSBContext()
{
    assert(usb_event_thread == nullptr);
    if (usb_context_owned) libusb_exit(usb_context);
}

void ZUSBContext::startRef()
{
    std::lock_guard<std::mutex> lock(mutex);
    if ( start_ref_count == 0 ) {
        assert(usb_event_thread == nullptr);
        usb_event_thread = new ZUSBEventThread(usb_context);
        usb_event_thread->start();

        /* Wait for USB thread to start */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    start_ref_count ++;
}

void ZUSBContext::stopUnRef()
{
    std::lock_guard<std::mutex> lock(mutex);
    start_ref_count --;
    assert(start_ref_count >= 0);
    if ( start_ref_count == 0 ) {
        Q_ASSERT(usb_event_thread != nullptr);
        usb_event_thread->waitAndDelete();
        usb_event_thread = nullptr;
    }
}

bool ZUSBContext::handleEvents(int& completed)
{
    if (libusb_handle_events_completed(usb_context, &completed) < 0)
        return false;

    return true;
}

bool ZUSBContext::handleEvents(timeval& time_val, int& completed)
{
    int res;
    if ( (res = libusb_handle_events_timeout_completed(usb_context, &time_val, &completed)) < 0) {
        return false;
    }

    return true;
}

std::string ZUSBContext::translateLibUSBErrorCode(int error_code)
{
    QString error_text;

    switch(error_code) {
    case LIBUSB_SUCCESS:
        error_text = "Success (no error).";
        break;

    case LIBUSB_ERROR_IO:
        error_text = "Input/output error.";
        break;

    case LIBUSB_ERROR_INVALID_PARAM:
        error_text = "Invalid parameter.";
        break;

    case LIBUSB_ERROR_ACCESS:
        error_text = "Access denied (insufficient permissions).";
        break;

    case LIBUSB_ERROR_NO_DEVICE:
        error_text = "No such device (it may have been disconnected).";
        break;

    case LIBUSB_ERROR_NOT_FOUND:
        error_text = "Entity not found.";
        break;

    case LIBUSB_ERROR_BUSY:
        error_text = "Resource busy.";
        break;

    case LIBUSB_ERROR_TIMEOUT:
        error_text = "Operation timed out.";
        break;

    case LIBUSB_ERROR_OVERFLOW:
        error_text = "Overflow.";
        break;

    case LIBUSB_ERROR_PIPE:
        error_text = "Pipe error.";
        break;

    case LIBUSB_ERROR_INTERRUPTED:
        error_text = "System call interrupted (perhaps due to signal).";
        break;

    case LIBUSB_ERROR_NO_MEM:
        error_text = "Insufficient memory.";
        break;

    case LIBUSB_ERROR_NOT_SUPPORTED:
        error_text = "Operation not supported or unimplemented on this platform.";
        break;

    case LIBUSB_ERROR_OTHER:
        error_text = "Other error.";
        break;

    default:
        error_text = "Unknown error.";
    }

    return error_text;
}

