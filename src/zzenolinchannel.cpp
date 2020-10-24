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

#include "vxzenolinchannel.h"
#include "vxzenousbdevice.h"
#include "vxzenolindriver.h"
#include "zenocan.h"

#include <QThread>
#include <QtDebug>
/*** ---------------------------==*+*+*==---------------------------------- ***/
quint8 linCalculatePIDParity(quint8 pid)
{
    /* Check PID parity */
    union {
        struct {
            unsigned ID0: 1;
            unsigned ID1: 1;
            unsigned ID2: 1;
            unsigned ID3: 1;
            unsigned ID4: 1;
            unsigned ID5: 1;
            unsigned P0: 1;
            unsigned P1: 1;
        } b;
        uint8_t raw_PID;
    } PID;

    PID.raw_PID = pid & 0x3F;
    pid = PID.raw_PID;
    PID.b.P0 =  PID.b.ID0 ^ PID.b.ID1;
    PID.b.P0 =  PID.b.P0  ^ PID.b.ID2;
    PID.b.P0 =  PID.b.P0  ^ PID.b.ID4;
    PID.b.P1 =  PID.b.ID1 ^ PID.b.ID3;
    PID.b.P1 =  PID.b.P1  ^ PID.b.ID4;
    PID.b.P1 =  PID.b.P1  ^ PID.b.ID5;
    PID.b.P1 = ~PID.b.P1;

    return PID.raw_PID;
}

quint8 linCalculateClassicCRC(quint8* data, quint8 dlc)
{
    quint16 __crc = 0;

    for (int i = 0; i < dlc; i++){
        __crc = __crc + data[i];
        if(__crc > 0xFF)
            __crc -= 0xFF;
    }
    __crc = ~__crc;

    return __crc & 0xff;
}

quint8 linCalculateEnhancedCRC(quint8 pid, quint8* data, quint8 dlc)
{
    quint16 __crc = pid;

    for (int i = 0; i < dlc; i++){
        __crc = __crc + data[i];
        if(__crc > 0xFF)
            __crc -= 0xFF;
    }
    __crc = ~__crc;

    return __crc & 0xff;
}

/*** ---------------------------==*+*+*==---------------------------------- ***/
VxZenoLINChannel::VxZenoLINChannel(int _channel_index, int _display_index,
                                   VxZenoUSBDevice *_usb_can_device)
    : channel_index(_channel_index),
      display_index(_display_index),
      is_master(false),
      auto_baud_set(false),
      zeno_usb_device(_usb_can_device),
      usb_display_name(zeno_usb_device->getObjectText()),
      serial_number(zeno_usb_device->getSerialNumber()),
      rx_message_fifo(32),
      tx_pending(false)
{
    connect(zeno_usb_device, &VxZenoUSBDevice::destroyed, [this]() {
        zeno_usb_device = nullptr;
    });
}

VxZenoLINChannel::~VxZenoLINChannel()
{
    if (is_open) close();
}

const QString VxZenoLINChannel::getObjectText() const
{
    QString channel_name;
    channel_name = QString("%1 LIN%2 (%3)").arg(usb_display_name).arg(display_index).arg(serial_number);
    return channel_name;
}

const QString VxZenoLINChannel::getLastErrorText()
{
    return last_error_text;
}

bool VxZenoLINChannel::open(bool master)
{
    is_open.ref();

    if (is_open.load() > 1) {
        is_open.deref();
        last_error_text = QString("LIN Channel %1 is already open").arg(channel_index);
        return false;
    }

    if (!zeno_usb_device->open()) {
        is_open.deref();
        return false;
    }

    ZenoLinOpen cmd;
    ZenoOpenResponse reply;

    memset(&cmd,0,sizeof(ZenoLinOpen));
    cmd.h.cmd_id = ZENO_CMD_LIN_OPEN;
    cmd.channel = quint8(channel_index);
    cmd.is_master = master;
    cmd.base_clock_divisor = quint8(zeno_usb_device->getClockResolution() / 1000);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed to open Zeno LIN channel: " << last_error_text;
        close();
        return false;
    }

    is_master = master;
    tx_pending = false;

    return true;
}

bool VxZenoLINChannel::close()
{
    busOff();

    ZenoClose cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoClose));
    cmd.h.cmd_id = ZENO_CMD_LIN_CLOSE;
    cmd.channel = quint8(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed to close Zeno LIN channel: " << last_error_text;
    }

    is_master = false;
    auto_baud_set = false;

    zeno_usb_device->close();
    is_open.deref();

    return true;
}

quint32 VxZenoLINChannel::getCapabilites()
{
    quint32 capabilities;

    capabilities =
            BusStatistics |
            GenerateError |
            GenerateOverload |
            TxAcknowledge;

    return capabilities;
}

bool VxZenoLINChannel::busOn()
{
    if (!checkOpen()) return false;

    ZenoBusOn cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOn));
    cmd.h.cmd_id = ZENO_CMD_LIN_BUS_ON;
    cmd.channel = quint8(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed to go bus on: " << last_error_text;
        return false;
    }

    return true;
}

bool VxZenoLINChannel::busOff()
{
    if (!checkOpen()) return false;

    ZenoBusOff cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOff));
    cmd.h.cmd_id = ZENO_CMD_LIN_BUS_OFF;
    cmd.channel = quint8(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed to go bus off: " << last_error_text;
        return false;
    }

    return true;
}

bool VxZenoLINChannel::setBusParameters(int bitrate)
{
    if (!checkOpen()) return false;

    if (bitrate == 0 && is_master) {
        last_error_text = "ERROR: Lin-Ch " + QString::number(channel_index) + " auto-baud is only supported for LIN slaves";
        return false;
    }
    ZenoLinBitrate cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoLinBitrate));
    cmd.h.cmd_id = ZEMO_CMD_LIN_SET_BITRATE;
    cmd.channel = quint8(channel_index);
    cmd.bitrate = quint16(bitrate);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed to set bitrate " << last_error_text;
        return false;
    }

    auto_baud_set = (bitrate == 0) ? true : false;

    return true;
}

bool VxZenoLINChannel::setSlaveResponse(quint8 pid, const quint8 *data,
                                        quint8 data_length, quint32 tx_flags)
{
    if (!checkOpen()) return false;

    ZenoTxLINRequest cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoTxLINRequest));
    cmd.h.cmd_id = ZENO_CMD_LIN_UPDATE_MESSAGE;
    cmd.channel = quint8(channel_index);
    cmd.pid = pid & 0x3f;
    cmd.dlc = qMin(data_length, quint8(8));
    memcpy(cmd.data, data, cmd.dlc);
    cmd.flags = tx_flags;

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed to update slave response " << last_error_text;
        return false;
    }

    return true;
}

bool VxZenoLINChannel::clearSlaveResponse(quint8 pid)
{
    if (!checkOpen()) return false;

    ZenoLinClearMessage cmd;
    ZenoResponse reply;

    cmd.h.cmd_id = ZENO_CMD_LIN_CLEAR_MESSAGE;
    cmd.channel = quint8(channel_index);
    cmd.pid = pid & 0x3f;

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed to clear slave response " << last_error_text;
        return false;
    }

    return true;
}

bool VxZenoLINChannel::clearSlaveResponses()
{
    if (!checkOpen()) return false;

    ZenoLinClearMessage cmd;
    ZenoResponse reply;

    cmd.h.cmd_id = ZENO_CMD_LIN_CLEAR_ALL_MESSAGES;
    cmd.channel = quint8(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed to clear slave responses " << last_error_text;
        return false;
    }

    return true;
}

VxLINChannel::ReadResult VxZenoLINChannel::readWait(quint8 &pid,
                                                    unsigned char *msg,
                                                    quint8 &data_length,
                                                    quint32 &flags,
                                                    qint64 &timestamp_in_us,
                                                    int timeout_in_ms)
{
    rx_message_fifo_mutex.lock();
    if ( rx_message_fifo.isEmpty()) {

        /* Wait for RX FIFO */
        rx_message_fifo_cond.wait(&rx_message_fifo_mutex, ulong(timeout_in_ms));

        /* If RX FIFO is still empty */
        if ( rx_message_fifo.isEmpty() ) {
            // onReadTimeoutCheck();

            rx_message_fifo_mutex.unlock();
            return ReadTimeout;
        }
    }

    ZenoLINMessage rx = rx_message_fifo.read();
    rx_message_fifo_mutex.unlock();

    pid = rx.pid & 0x3f;
    flags = rx.flags;

    if ( linCalculatePIDParity(rx.pid) != rx.pid ) {
        flags |= ParityError;
        flags |= ErrorFrame;
        if (auto_baud_set) resetAutoBaud();
    }

    // Q_ASSERT(rx.dlc <= 8);
    data_length = qMin(rx.dlc,quint8(8));
    timestamp_in_us = qint64(rx.timestamp_end / 70);
    memcpy(msg, rx.data, sizeof(rx.data));

    /* Calcuate and check CRC */
    if ( !(rx.flags & ZenoLINNoData) ) {
        /* First check with LIN 2.x enhanced CRC */
        quint8 __crc = linCalculateEnhancedCRC(rx.pid, rx.data, data_length);
        if ( __crc != rx.checksum ) {
            /* Fall over to LIN 1.x classic CRC */
            __crc = linCalculateClassicCRC(rx.data, data_length);
            if ( __crc == rx.checksum ) {
                flags |= ClassicChecksum;
            }
            else {
                flags |= ChecksumError;
                flags |= ErrorFrame;
                if (auto_baud_set) resetAutoBaud();
            }
        }
    }

    return ReadStatusOK;
}

VxLINChannel::SendResult VxZenoLINChannel::send(quint8 id,
                                                const quint8 *msg,
                                                quint8 data_length,
                                                quint32 flags,
                                                int timeout_in_ms)
{
    if (!checkOpen()) return SendError;

    tx_message_mutex.lock();
    if (!waitForTX(timeout_in_ms)) {
        qDebug() << "LIN send timeout";
        tx_message_mutex.unlock();
        return SendTimeout;
    }

    ZenoTxLINRequest cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoLinBitrate));
    cmd.h.cmd_id = ZENO_CMD_LIN_TX_MESSAGE;
    cmd.channel = quint8(channel_index);

    cmd.pid = id;
    cmd.dlc = qMin(data_length, quint8(8));
    cmd.flags = flags & 0xff;
    cmd.is_master_request = false;
    memcpy(cmd.data, msg, cmd.dlc);

    tx_pending = true;
    tx_message = cmd;
    tx_message_mutex.unlock();
    // cmd.bitrate = quint16(bitrate);


    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed transmit " << last_error_text;

        tx_message_mutex.lock();
        tx_pending = false;
        tx_message_cond.wakeOne();
        tx_message_mutex.unlock();

        return SendError;
    }

    return SendStatusOK;
}

VxLINChannel::SendResult VxZenoLINChannel::sendMasterRequest(quint8 pid,
                                                             quint32 flags)
{
    if (!checkOpen()) return SendError;

    tx_message_mutex.lock();
    if (!waitForTX(100)) {
        tx_message_mutex.unlock();

        return SendTimeout;
    }

    ZenoTxLINRequest cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoLinBitrate));
    cmd.h.cmd_id = ZENO_CMD_LIN_TX_MESSAGE;
    cmd.channel = quint8(channel_index);

    cmd.pid = pid;
    cmd.dlc = 0;
    cmd.flags = flags;
    cmd.is_master_request = true;

    tx_pending = true;
    tx_message = cmd;
    tx_message_mutex.unlock();

    qDebug() << "Send master request" << channel_index;

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed transmit " << last_error_text;

        tx_message_mutex.lock();
        tx_pending = false;
        tx_message_cond.wakeOne();
        tx_message_mutex.unlock();

        return SendError;
    }

    return SendError;
}

VxLINChannel::SendResult VxZenoLINChannel::sendWakeup(quint32 flags)
{
    Q_UNUSED(flags)
    /* TODO: implement me ... */

    return SendError;
}

quint64 VxZenoLINChannel::getSerialNumber()
{
    return serial_number;
}

VxLINDriver *VxZenoLINChannel::getLINDriver() const
{
    return zeno_usb_device->getZenoLINDriver();
}

void VxZenoLINChannel::queueMessage(ZenoLINMessage &message)
{
    QMutexLocker lock(&rx_message_fifo_mutex);

    if ( rx_message_fifo.available() == 0 ) {
        qDebug() << "ZenoLIN-Ch" << channel_index << "Zeno: RX buffer overflow" << rx_message_fifo.count() << rx_message_fifo.isEmpty();
        return;
    }

    rx_message_fifo_cond.wakeOne();
    rx_message_fifo.write(message);
}

void VxZenoLINChannel::txAck(ZenoTxLINRequestAck &tx_ack)
{
    QMutexLocker lock(&tx_message_mutex);
    if ( tx_pending ) {
        if ( !tx_message.is_master_request ) {
            ZenoLINMessage rx;

            rx.h = tx_ack.h;
            rx.pid = linCalculatePIDParity(tx_message.pid);
            rx.flags = tx_ack.flags | ZenoLINTxMsgAcknowledge;
            rx.dlc = tx_message.dlc;
            memcpy(rx.data, tx_message.data, sizeof(rx.data));

            if ( rx.flags | ClassicChecksum ) {
                rx.checksum = linCalculateClassicCRC(rx.data, qMin(quint8(8), rx.dlc));
            }
            else {
                rx.checksum = linCalculateEnhancedCRC(rx.pid, rx.data, qMin(quint8(8), rx.dlc));
            }

            rx.timestamp_end = tx_ack.timestamp_end;
            rx.timestamp_start = tx_ack.timestamp_start;

            tx_message_mutex.unlock();
            queueMessage(rx);
            tx_message_mutex.lock();
        }

        tx_pending = false;
        tx_message_cond.wakeOne();
    }
}

bool VxZenoLINChannel::checkOpen()
{
    if (is_open.load() == 0) {
        last_error_text = QString("LIN Channel %1 is not open").arg(channel_index);
        return false;
    }

    return true;
}

bool VxZenoLINChannel::waitForTX(int timeout_in_ms)
{
    if ( tx_pending ) {
        tx_message_cond.wait(&tx_message_mutex, ulong(timeout_in_ms));
    }

    return !tx_pending;
}

bool VxZenoLINChannel::resetAutoBaud()
{
    if (!checkOpen()) return false;

    ZenoLinResetAutoBaud cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoLinResetAutoBaud));
    cmd.h.cmd_id = ZENO_CMD_LIN_RESET_AUTO_BAUD;
    cmd.channel = quint8(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) LIN-Ch" << channel_index << " failed to go bus on: " << last_error_text;
        return false;
    }

    return true;
}
