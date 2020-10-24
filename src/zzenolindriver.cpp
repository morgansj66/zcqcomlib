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

#include "vxzenolindriver.h"
#include "vxzenocandriver.h"
#include "lin/vxlindriverfactory.h"

VxZenoLINDriver::VxZenoLINDriver(VxZenoCANDriver *_zeno_can_driver)
: VxLINDriver("zeno-lin-driver","Zuragon Zeno USB LIN driver"),
  zeno_can_driver(_zeno_can_driver)
{
    setDriverPriorityOrder(1);
    VxLINDriverFactory::instance()->addPluginInstance(this);
}

const QString VxZenoLINDriver::getObjectText() const
{
    return "Zeno LIN interface driver";
}

int VxZenoLINDriver::getNumberOfChannels()
{
    int channel_count = 0;
    for ( int i = 0; i < device_list.size(); ++i ) {
         channel_count += device_list[i]->getLINChannelCount();
    }

    return channel_count;
}

const QString VxZenoLINDriver::getChannelName(int channel_index)
{
    VxLINChannel* lin_channel = getChannel(channel_index);
    if ( lin_channel == nullptr ) return QString();
    return lin_channel->getObjectText();
}

VxLINChannel* VxZenoLINDriver::getChannel(int channel_index)
{
    int channel_offset = 0;
    for ( int i = 0; i < device_list.size(); ++i ) {
        int index = channel_index - channel_offset;
        int channel_count = device_list[i]->getLINChannelCount();
        if ( index >= 0 && index < channel_count) {
            return device_list[i]->getLINChannel(index).get();
        }
        channel_offset += channel_count;
    }
    return nullptr;
}

bool VxZenoLINDriver::enumerateDevices()
{
    QMutexLocker lock(&driver_mutex);

    bool device_list_has_changed = ( new_device_list != device_list );
    enumerateLINChannels();

    return device_list_has_changed;
}

void VxZenoLINDriver::driverRef()
{
    zeno_can_driver->driverRef();
}

void VxZenoLINDriver::driverUnref()
{
    zeno_can_driver->driverUnref();
}

void VxZenoLINDriver::updateDeviceList(QList<VxReference<VxZenoUSBDevice> > _new_device_list)
{
    // QMutexLocker lock(&driver_mutex);
    bool dispatch_channel_list_updated = false;

    driver_mutex.lock();
    new_device_list = _new_device_list;

    enumerateLINChannels();
    dispatch_channel_list_updated = true;

    driver_mutex.unlock();

    if (dispatch_channel_list_updated ) {
        VxLINDriverFactory::instance()->dispatchChannelListUpdated();
    }
}

void VxZenoLINDriver::enumerateLINChannels()
{
    device_list = new_device_list;
    new_device_list.clear();
}
