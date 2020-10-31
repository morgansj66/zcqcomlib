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

#ifndef ZLINDRIVER_H_
#define ZLINDRIVER_H_

#include "zrefcountingobjbase.h"
#include <string>

class ZLINChannel;
class ZLINDriver : public ZRefCountingObjBase {
protected:
    ZLINDriver(const std::string& _name, const std::string& _description)
    : driver_priority_order(50), name(_name), description(_description)
    {
        /* no op */
    }

public:
    virtual const std::string getObjectText() const = 0;
    virtual int getNumberOfChannels() = 0;
    virtual const std::string getChannelName(int channel_index) = 0;
    virtual const std::string getChannelDeviceDescription(int channel_index) {
        ZUNUSED(channel_index)
        /* Optionally implemented */
        return std::string();
    }

    int getDriverPriorityOrder() const {
        return driver_priority_order;
    }

    virtual ZLINChannel* getChannel(int channel_index) = 0;

    /**
     * @return true if device list has changed since last time
     */
    virtual bool enumerateDevices() {
        /* Optionally implemented */
        return false;
    }

    virtual void driverRef() { /* Optionally implemented */ }

    virtual void driverUnref() { /* Optionally implemented */ }

protected:
    void setDriverPriorityOrder(int _driver_priority_order) {
        driver_priority_order = _driver_priority_order;
    }

private:
    int driver_priority_order;

    std::string name;
    std::string description;
};

#endif /* VXLINDRIVER_H_ */
