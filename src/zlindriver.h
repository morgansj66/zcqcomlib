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

#ifndef VXLINDRIVER_H_
#define VXLINDRIVER_H_

#include "vxplugindescriptor.h"

class VxLINChannel;
class VX_EXPORT VxLINDriver : public VxPluginDescriptor {
    Q_OBJECT
protected:
    VxLINDriver(const QString& _name, const QString& _description)
    : VxPluginDescriptor(_name,_description), driver_priority_order(50)
    {
        /* no op */
    }

public:
    virtual const QString getObjectText() const = 0;
    virtual int getNumberOfChannels() = 0;
    virtual const QString getChannelName(int channel_index) = 0;
    virtual const QString getChannelDeviceDescription(int channel_index) {
        Q_UNUSED(channel_index)
        /* Optionally implemented */
        return QString();
    }

    int getDriverPriorityOrder() const {
        return driver_priority_order;
    }

    virtual VxLINChannel* getChannel(int channel_index) = 0;

    /**
     * @return true if device list has changed since last time
     */
    virtual bool enumerateDevices() {
        /* Optionally implemented */
        return false;
    }

    Q_INVOKABLE virtual void driverRef() { /* Optionally implemented */ }

    Q_INVOKABLE virtual void driverUnref() { /* Optionally implemented */ }

protected:
    void setDriverPriorityOrder(int _driver_priority_order) {
        driver_priority_order = _driver_priority_order;
    }

private:
    int driver_priority_order;
};

#endif /* VXLINDRIVER_H_ */
