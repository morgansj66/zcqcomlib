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

#include "zzenolindriver.h"
#include "zzenocandriver.h"
// #include "zlindriverfactory.h"

ZZenoLINDriver::ZZenoLINDriver(ZZenoCANDriver *_zeno_can_driver)
: ZLINDriver("zeno-lin-driver","Zuragon Zeno USB LIN driver"),
  zeno_can_driver(_zeno_can_driver)
{
    setDriverPriorityOrder(1);
    // ZLINDriverFactory::instance()->addPluginInstance(this);
}

const std::string ZZenoLINDriver::getObjectText() const
{
    return "Zeno LIN interface driver";
}

int ZZenoLINDriver::getNumberOfChannels()
{
    int channel_count = 0;
    for ( auto device : device_list ) {
        channel_count += device->getLINChannelCount();
    }

    return channel_count;
}

const std::string ZZenoLINDriver::getChannelName(int channel_index)
{
    ZZenoLINChannel* lin_channel = static_cast<ZZenoLINChannel*>(getChannel(channel_index));
    if ( lin_channel == nullptr ) return std::string();
    return lin_channel->getObjectText();
}

ZLINChannel* ZZenoLINDriver::getChannel(int channel_index)
{
    int channel_offset = 0;
    for(auto device : device_list) {
        int index = channel_index - channel_offset;
        int channel_count = device->getCANChannelCount();

        if ( index >= 0 && index < channel_count) {
            return device->getLINChannel(index).cast<ZZenoLINChannel>();
        }
        channel_offset += channel_count;
    }
    return nullptr;
}

bool ZZenoLINDriver::enumerateDevices()
{
    std::lock_guard<std::mutex> lock(driver_mutex);

    bool device_list_has_changed = ( new_device_list != device_list );
    enumerateLINChannels();

    return device_list_has_changed;
}

void ZZenoLINDriver::driverRef()
{
    zeno_can_driver->driverRef();
}

void ZZenoLINDriver::driverUnref()
{
    zeno_can_driver->driverUnref();
}

void ZZenoLINDriver::updateDeviceList(std::vector<ZRef<ZZenoUSBDevice> > _new_device_list)
{
    // QMutexLocker lock(&driver_mutex);
    bool dispatch_channel_list_updated = false;

    driver_mutex.lock();
    new_device_list = _new_device_list;

    enumerateLINChannels();
    dispatch_channel_list_updated = true;

    driver_mutex.unlock();

    // if (dispatch_channel_list_updated ) {
    //    ZLINDriverFactory::instance()->dispatchChannelListUpdated();
    // }
}

void ZZenoLINDriver::enumerateLINChannels()
{
    device_list = new_device_list;
    new_device_list.clear();
}
