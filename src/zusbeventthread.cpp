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

#include "zusbeventthread.h"
#include "zdebug.h"
#include "assert.h"

ZUSBEventThread::ZUSBEventThread(libusb_context* _usb_context)
    : running(0), usb_context(_usb_context),
      usb_event_thread(nullptr)
{
    /* no op */
}

void ZUSBEventThread::waitAndDelete()
{
    running = 1;
    usb_event_thread->join();
    usb_event_thread.reset();
}

void ZUSBEventThread::run()
{
    const int timeout_in_ms = 100;

    zDebug("USB event thread started");
    while(!running) {
        timeval time_val;
        time_val.tv_sec = timeout_in_ms / 1000;
        time_val.tv_usec = (timeout_in_ms % 1000) * 1000;

        // libusb_handle_events_timeout(usb_context,&time_val);
        libusb_handle_events_timeout_completed(usb_context, &time_val, &running);
    }
    zDebug("**** USB event thread ended");
}

void ZUSBEventThread::start()
{
    assert(usb_event_thread == nullptr);
    usb_event_thread.reset(new std::thread(&ZUSBEventThread::run,this));
}
