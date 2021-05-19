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

#ifndef ZZENOCANDRIVER_H_
#define ZZENOCANDRIVER_H_

#include "zcandriver.h"
#include "zzenousbdevice.h"
#include "libusb.h"

#include <map>
#include <vector>
#include <mutex>

#define ZURAGON_VENDOR_ID               0x84d8
#define ZENO_CANUNO_PRODUCT_ID          0x14
#define ZENO_CANQUATRO_PRODUCT_ID       0x15
#define ZENO_CANQUATRO_MPCIE_PRODUCT_ID 0x16

class ZUSBContext;
class ZZenoLINDriver;
class ZZenoCANDriver : public ZCANDriver {
public:
    ZZenoCANDriver();
    ~ZZenoCANDriver() override;

    const std::string getObjectText() const override;
    int getNumberOfChannels() override;
    const std::string getChannelName(int channel_index) override;
    ZCANChannel* getChannel(int channel_index) override;
    bool enumerateDevices() override;

    void driverRef() override;
    void driverUnref() override;

    ZUSBContext* getUSBContext() const {
        return usb_context;
    }

    ZZenoLINDriver* getZenoLINDriver() const {
        return lin_driver;
    }

    std::vector<ZRef<ZZenoUSBDevice> > getZenoDeviceList() const;

private:
    void init();
    bool enumerateDevicesUnlocked();

    ZUSBContext* usb_context;
    std::map<int,std::string> product_id_table;
    std::vector<ZRef<ZZenoUSBDevice> > device_list;

    bool enumerate_pending;
    int driver_ref_count;
    mutable std::mutex driver_mutex;

    /* Zeno LIN */
    ZRef<ZZenoLINDriver> lin_driver;
};

#endif /* ZZENOCANDRIVER_H_ */
