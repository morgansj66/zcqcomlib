/******
 ***
 **
 **  8P d8P
 **  P d8P  8888 8888 888,8,  ,"Y88b  e88 888  e88 88e  888 8e
 **   d8P d 8888 8888 888 "  "8" 888 d888 888 d888 888b 888 88b
 **  d8P d8 Y888 888P 888    ,ee 888 Y888 888 Y888 888P 888 888
 ** d8P d88  "88 88"  888    "88 888  "88 888  "88 88"  888 888
 **                                    ,  88P
 **                                   "8",P"
 **
 ** Copyright Zuragon Ltd (R)
 **
 ** This Software Development Kit (SDK) is Subject to the payment of the
 ** applicable license fees and have been granted to you on a non-exclusive,
 ** non-transferable basis to use according to Zuragon General Terms 2014.
 ** Zuragon Technologies Ltd reserves any and all rights not expressly
 ** granted to you.
 **
 ***
 *****/

#ifndef VXZENOLINDRIVER_H
#define VXZENOLINDRIVER_H

#include "lin/vxlindriver.h"
#include "vxzenousbdevice.h"

#include <QMutex>

class VxZenoCANDriver;
class VxZenoLINDriver : public VxLINDriver {
    Q_OBJECT
public:
    VxZenoLINDriver(VxZenoCANDriver* _zeno_can_driver);

    const QString getObjectText() const override;

    int getNumberOfChannels() override;

    const QString getChannelName(int channel_index) override;

    VxLINChannel* getChannel(int channel_index) override;

    bool enumerateDevices() override;

    void driverRef() override;

    void driverUnref() override;

    void updateDeviceList(QList<VxReference<VxZenoUSBDevice> > new_device_list);


private:
    void enumerateLINChannels();

    QList<VxReference<VxZenoUSBDevice> > device_list;
    QList<VxReference<VxZenoUSBDevice> > new_device_list;

    QMutex driver_mutex;
    VxZenoCANDriver* zeno_can_driver;
};

#endif /* VXZENOLINDRIVER_H */
