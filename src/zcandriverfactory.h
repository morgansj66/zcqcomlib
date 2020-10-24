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

#ifndef VXCANDRIVERFACTORY_H_
#define VXCANDRIVERFACTORY_H_

#include "vxpluginloader.h"
#include "can/vxcandriver.h"
#include <QList>

class VxCANDriver;
class VxCANChannel;
class VX_EXPORT VxCANDriverFactory : public VxPluginLoader {
    Q_OBJECT
private:
    VxCANDriverFactory();

    static VxCANDriverFactory* __the_instance;

public:
    static VxCANDriverFactory* instance();

    QList<VxCANDriver*> getDriverList();
    QList<VxReference<VxCANChannel> > getChannelList();
    QList<VxReference<VxCANChannel> > getCanFDChannelList();
    void enumerateDevices();
    void dispatchChannelListUpdated();

signals:
    void channelListUpdated();

private slots:
    void onDeviceAddedRemoved();
};
#endif /* VXCANDRIVERFACTORY_H_ */
