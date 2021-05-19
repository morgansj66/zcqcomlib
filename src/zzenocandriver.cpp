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

#include "zzenocandriver.h"
#include "zzenolindriver.h"
#include "zusbcontext.h"
#include "zdebug.h"
#include <map>

ZZenoCANDriver::ZZenoCANDriver()
: ZCANDriver("zeno-can-driver", "Zuragon Zeno USB CAN driver"),
  usb_context(new ZUSBContext()),
  enumerate_pending(false), driver_ref_count(0),
  lin_driver(nullptr)
{
    setDriverPriorityOrder(0);
    init();
}

ZZenoCANDriver::~ZZenoCANDriver()
{
}

const std::string ZZenoCANDriver::getObjectText() const
{
    return "Zuragon Zeno USB CAN interface driver";
}

int ZZenoCANDriver::getNumberOfChannels()
{
    int channel_count = 0;
    for ( auto device : device_list ) {
        channel_count += device->getCANChannelCount();
    }

    return channel_count;
}

const std::string ZZenoCANDriver::getChannelName(int channel_index)
{
    ZRef<ZZenoCANChannel> can_channel = static_cast<ZZenoCANChannel*>(getChannel(channel_index));
    if ( can_channel == nullptr ) return std::string();
    return can_channel->getObjectText();
}

ZCANChannel* ZZenoCANDriver::getChannel(int channel_index)
{
    int channel_offset = 0;
    for(auto device : device_list) {
        int index = channel_index - channel_offset;
        int channel_count = device->getCANChannelCount();
        if ( index >= 0 && index < channel_count) {
            return device->getCANChannel(index).get();
        }
        channel_offset += channel_count;
    }
    return nullptr;
}

bool ZZenoCANDriver::enumerateDevices()
{
    bool channel_list_updated;

    driver_mutex.lock();
    channel_list_updated = enumerateDevicesUnlocked();
    driver_mutex.unlock();

    return channel_list_updated;
}

bool ZZenoCANDriver::enumerateDevicesUnlocked()
{
    if ( driver_ref_count != 0 ) {
        zInfo("can't enumerate CAN devices now, driver is busy");
        enumerate_pending = true;
        return false;
    }

    std::vector<ZRef<ZZenoUSBDevice> > new_zeno_can_device_list;
    std::vector<ZRef<ZZenoUSBDevice> > new_zeno_lin_device_list;

    /* Discover devices */
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(usb_context->getUSBContext(), &list);

    if (cnt < 0) {
        zCritical("failed to enumerate USB devices");
        return false;
    }

    unsigned int device_index = 0;
    bool device_list_updated = false;
    for (int i = 0; i < cnt; i++) {
        libusb_device *device = list[i];

        libusb_device_descriptor descriptor;
        libusb_get_device_descriptor(device, &descriptor);
        // qDebug() << "Zeno  USB: " << i << " vendor " << descriptor.idVendor << " product " << descriptor.idProduct;

        std::map<int,std::string>::iterator k;
        if (descriptor.idVendor == ZURAGON_VENDOR_ID &&
            (k = product_id_table.find(descriptor.idProduct)) != product_id_table.end()) {

            ZRef<ZZenoUSBDevice> zeno_usb_device;
            zeno_usb_device = new ZZenoUSBDevice(this,i, device, k->second);

            if ( device_index < device_list.size() ) {
                /* Check if channel is already at the same index */
                ZRef<ZZenoUSBDevice> __zeno_usb_device = device_list[device_index];
                if ( !__zeno_usb_device->isDeviceGoneOrDisconnected() &&
                     __zeno_usb_device->getSerialNumber() == zeno_usb_device->getSerialNumber() ) {
                    /* Assume channel already enumerated, don't change */
                    zeno_usb_device = __zeno_usb_device;
                } else {
                    /* New device */
                    device_list_updated = true;
                }
            } else {
                /* New device */
                device_list_updated = true;
            }

            new_zeno_can_device_list.push_back(zeno_usb_device);
            if (zeno_usb_device->getLINChannelCount() > 0 ) {
                new_zeno_lin_device_list.push_back(zeno_usb_device);
            }
            device_index ++;
        }
    }

    if ( !new_zeno_can_device_list.empty() ) {
        if ( lin_driver == nullptr ) {
            lin_driver = new ZZenoLINDriver(this);
        }
        lin_driver->updateDeviceList(new_zeno_can_device_list);
    }
    else {
        if ( lin_driver != nullptr ) {
            lin_driver = nullptr;

            // VxLINDriverFactory::instance()->dispatchChannelListUpdated();
        }
    }

    if ( device_list.size() != new_zeno_can_device_list.size() ) {
        /* New devices added at the end */
        device_list_updated = true;
    }

    device_list = new_zeno_can_device_list;
    device_list.shrink_to_fit();
    libusb_free_device_list(list,1);

    return device_list_updated;
}

void ZZenoCANDriver::driverRef()
{
    std::lock_guard<std::mutex> lock(driver_mutex);
    driver_ref_count++;
}

void ZZenoCANDriver::driverUnref()
{
    bool channel_list_updated = false;

    driver_mutex.lock();
    assert(driver_ref_count >= 0);

    driver_ref_count--;

    // bool channels_enumerated = false;
    if ( driver_ref_count == 0 && enumerate_pending ) {
        channel_list_updated = enumerateDevicesUnlocked();
        // channels_enumerated = true;
        if ( driver_ref_count == 0) channel_list_updated = true;
    }
    driver_mutex.unlock();

    // if ( channel_list_updated ) VxCANDriverFactory::instance()->dispatchChannelListUpdated();
}

std::vector<ZRef<ZZenoUSBDevice> > ZZenoCANDriver::getZenoDeviceList() const
{
    return device_list;
}

void ZZenoCANDriver::init()
{
    product_id_table[ZENO_CANUNO_PRODUCT_ID]          = "Zeno CANuno";
    product_id_table[ZENO_CANQUATRO_PRODUCT_ID]       = "Zeno CANquatro";
    product_id_table[ZENO_CANQUATRO_MPCIE_PRODUCT_ID] = "Zeno CANquatro mPCIe";

    ZZenoCANDriver::enumerateDevices();
}
