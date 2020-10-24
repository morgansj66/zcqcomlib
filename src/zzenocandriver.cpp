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

#include "vxzenocandriver.h"
#include "vxzenolindriver.h"
#include "can/vxcandriverfactory.h"
#include "lin/vxlindriverfactory.h"
#include "vxusbcontext.h"

#include <QtPlugin>
#include <QtDebug>

VxZenoCANDriver::VxZenoCANDriver()
: VxCANDriver("zeno-can-driver", "Zuragon Zeno USB CAN driver"),
  usb_context(new VxUSBContext(this)),
  enumerate_pending(false), driver_ref_count(0),
  lin_driver(nullptr)
{
    setDriverPriorityOrder(0);
    init();
}

VxZenoCANDriver::~VxZenoCANDriver()
{
    if (lin_driver != nullptr) delete lin_driver;
}

const QString VxZenoCANDriver::getObjectText() const
{
    return "Zuragon Zeno USB CAN interface driver";
}

int VxZenoCANDriver::getNumberOfChannels()
{
    int channel_count = 0;
    for ( int i = 0; i < device_list.size(); ++i ) {
         channel_count += device_list[i]->getCANChannelCount();
    }

    return channel_count;
}

const QString VxZenoCANDriver::getChannelName(int channel_index)
{
    VxCANChannel* can_channel = getChannel(channel_index);
    if ( can_channel == nullptr ) return QString();
    return can_channel->getObjectText();
}

VxCANChannel* VxZenoCANDriver::getChannel(int channel_index)
{
    int channel_offset = 0;
    for ( int i = 0; i < device_list.size(); ++i ) {
        int index = channel_index - channel_offset;
        int channel_count = device_list[i]->getCANChannelCount();
        if ( index >= 0 && index < channel_count) {
            return device_list[i]->getCANChannel(index).get();
        }
        channel_offset += channel_count;
    }
    return nullptr;
}

bool VxZenoCANDriver::enumerateDevices()
{
    bool channel_list_updated;

    driver_mutex.lock();
    channel_list_updated = enumerateDevicesUnlocked();
    driver_mutex.unlock();

    return channel_list_updated;
}

bool VxZenoCANDriver::enumerateDevicesUnlocked()
{
    if ( driver_ref_count != 0 ) {
        qDebug() << __PRETTY_FUNCTION__ << ": can't enumerate CAN devices now, driver is busy";
        enumerate_pending = true;
        return false;
    }

    QList<VxReference<VxZenoUSBDevice> > new_zeno_can_device_list;
    QList<VxReference<VxZenoUSBDevice> > new_zeno_lin_device_list;

    /* Discover devices */
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(usb_context->getUSBContext(), &list);

    if (cnt < 0) {
        qCritical() << "FATAL: failed to enumerate USB devices";
        return false;
    }

    int device_index = 0;
    bool device_list_updated = false;
    for (int i = 0; i < cnt; i++) {
        libusb_device *device = list[i];

        libusb_device_descriptor descriptor;
        libusb_get_device_descriptor(device, &descriptor);
        // qDebug() << "Zeno  USB: " << i << " vendor " << descriptor.idVendor << " product " << descriptor.idProduct;

        QHash<int,QString>::iterator k;
        if (descriptor.idVendor == ZURAGON_VENDOR_ID &&
            (k = product_id_table.find(descriptor.idProduct)) != product_id_table.end()) {

            VxReference<VxZenoUSBDevice> zeno_usb_device;
            zeno_usb_device = new VxZenoUSBDevice(this,i, device, k.value());

            if ( device_index < device_list.size() ) {
                /* Check if channel is already at the same index */
                VxReference<VxZenoUSBDevice> __zeno_usb_device = device_list[device_index];
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

            new_zeno_can_device_list.append(zeno_usb_device);
            if (zeno_usb_device->getLINChannelCount() > 0 ) {
                new_zeno_lin_device_list.append(zeno_usb_device);
            }
            device_index ++;
        }
    }

    if ( !new_zeno_can_device_list.isEmpty() ) {
        if ( lin_driver == nullptr ) {
            lin_driver = new VxZenoLINDriver(this);
        }
        lin_driver->updateDeviceList(new_zeno_can_device_list);
    }
    else {
        if ( lin_driver != nullptr ) {
            delete lin_driver;
            lin_driver = nullptr;

            VxLINDriverFactory::instance()->dispatchChannelListUpdated();
        }
    }

    if ( device_list.size() != new_zeno_can_device_list.size() ) {
        /* New devices added at the end */
        device_list_updated = true;
    }

    device_list = new_zeno_can_device_list;
    libusb_free_device_list(list,1);

    return device_list_updated;
}

void VxZenoCANDriver::driverRef()
{
    QMutexLocker lock(&driver_mutex);
    driver_ref_count++;
}

void VxZenoCANDriver::driverUnref()
{
    bool channel_list_updated = false;

    driver_mutex.lock();
    Q_ASSERT(driver_ref_count >= 0);

    driver_ref_count--;

    // bool channels_enumerated = false;
    if ( driver_ref_count == 0 && enumerate_pending ) {
        channel_list_updated = enumerateDevicesUnlocked();
        // channels_enumerated = true;
        if ( driver_ref_count == 0) channel_list_updated = true;
    }
    driver_mutex.unlock();

    if ( channel_list_updated ) VxCANDriverFactory::instance()->dispatchChannelListUpdated();
}

QList<VxReference<VxZenoUSBDevice> > VxZenoCANDriver::getZenoDeviceList() const
{
    return device_list;
}

void VxZenoCANDriver::init()
{
    product_id_table.insert(ZENO_CANUNO_PRODUCT_ID, "Zeno CANuno");
    product_id_table.insert(ZENO_CANQUATRO_PRODUCT_ID,"Zeno CANquatro");

    enumerateDevices();
}
