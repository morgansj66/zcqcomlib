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
#include "zzenocanchannel.h"
#include "zzenousbdevice.h"
#include "zzenocandriver.h"
#include "zdebug.h"

ZZenoCANChannel::ZZenoCANChannel(int _channel_index,
                                   ZZenoUSBDevice* _usb_can_device)
    : channel_index(_channel_index),
      is_open(false),is_canfd_mode(false),
      initial_timer_adjustment_done(false),
      usb_can_device(_usb_can_device),
      tx_request_count(0), tx_next_trans_id(0),
      max_outstanding_tx_requests(31),
      rx_message_fifo(2048),
      tx_message_fifo(1024),
      bus_active_bit_count(0),
      last_measure_time_in_us(0),
      current_bitrate(0),
      // tx_id(0),
      open_start_ref_timestamp_in_us(0),
      usb_display_name(usb_can_device->getObjectText()),
      serial_number(usb_can_device->getSerialNumber()),
      base_clock_divisor(1)
{
    connect(usb_can_device, &VxZenoUSBDevice::destroyed, [this]() {
        usb_can_device = nullptr;
    });

    memset(&canfd_msg_p1, 0 , sizeof(canfd_msg_p1));
    memset(&canfd_msg_p2, 0 , sizeof(canfd_msg_p2));
}

ZZenoCANChannel::~ZZenoCANChannel()
{
    if (is_open) close();
}

const std::string ZZenoCANChannel::getObjectText() const
{
    QString channel_name;
    channel_name = QString("%1 Ch%2 (%3)").arg(usb_display_name).arg(channel_index+1).arg(serial_number);
    return channel_name;
}

const std::string ZZenoCANChannel::getLastErrorText()
{
    return last_error_text;
}

bool ZZenoCANChannel::open(int open_flags)
{
    is_open.ref();

    if (is_open.load() > 1) {
        is_open.deref();
        last_error_text = QString("CAN Channel %1 is already open").arg(channel_index);
        return false;
    }

    if ( open_flags & VxCANFlags::SharedMode ) {
        last_error_text = QStringLiteral("Shared mode not supported on this CAN channel");
        return false;
    }

    flushRxFifo();
    flushTxFifo();

    if (!usb_can_device->open()) {
        is_open.deref();
        return false;
    }


    ZenoOpen cmd;
    ZenoOpenResponse reply;

    memset(&cmd,0,sizeof(ZenoOpen));
    cmd.h.cmd_id = ZENO_CMD_OPEN;
    cmd.channel = quint8(channel_index);
    cmd.base_clock_divisor = quint8(usb_can_device->getClockResolution() / 1000);

    if ( open_flags & VxCANFlags::CanFD ) {
        cmd.can_fd_mode = 1;
        is_canfd_mode = true;
    }
    else is_canfd_mode = false;

    if ( open_flags & VxCANFlags::CanFDNonISO ) {
        cmd.can_fd_non_iso = 1;
    }

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed to open Zeno CAN channel: " << last_error_text;
        close();
        return false;
    }

    last_device_timestamp_in_us = 0;
    max_outstanding_tx_requests = reply.max_pending_tx_msgs;
    open_start_ref_timestamp_in_us = qint64(reply.clock_start_ref);
    initial_timer_adjustment_done = 0;
    base_clock_divisor = qMax(uint(reply.base_clock_divisor),1u);

    qDebug() << "Zeno - max outstanding TX:" << reply.max_pending_tx_msgs << "Base clock divisor:" << base_clock_divisor;

    return true;
}

bool ZZenoCANChannel::close()
{
    busOff();

    ZenoClose cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoClose));
    cmd.h.cmd_id = ZENO_CMD_CLOSE;
    cmd.channel = quint8(channel_index);

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed to close Zeno CAN channel: " << last_error_text;
    }

    is_canfd_mode = false;
    current_bitrate = 0;
    usb_can_device->close();
    is_open.deref();

    return true;
}

uint32_t ZZenoCANChannel::getCapabilites()
{
    quint32 capabilities =
            GenerateError |
            ErrorCounters |
            BusStatistics |
            ErrorCounters |
            ExtendedCAN   |
            TxRequest     |
            TxAcknowledge;

    if ( channel_index < 4) {
        capabilities |= CanFD | CanFDNonISO;
    }

    return capabilities;
}

bool ZZenoCANChannel::busOn()
{
    qDebug() << "ZenoCAN-Ch" << channel_index << "Bus On";
    if (!checkOpen()) return false;

    ZenoBusOn cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOn));
    cmd.h.cmd_id = ZENO_CMD_BUS_ON;
    cmd.channel = quint8(channel_index);

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed to go bus on: " << last_error_text;
        return false;
    }

    /* Bus load statistic */
    last_measure_time_in_us = VxTimerBase::getCurrentTimeInUs();
    bus_active_bit_count = 0;

    return true;
}

bool ZZenoCANChannel::busOff()
{
    qDebug() << "ZenoCAN-Ch" << channel_index  << "Bus Off";
    if (!checkOpen()) return false;

    ZenoBusOff cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBitTiming));
    cmd.h.cmd_id = ZENO_CMD_BUS_OFF;
    cmd.channel = quint8(channel_index);

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed to go bus off: " << last_error_text;
        return false;
    }

    return true;
}

bool ZZenoCANChannel::setBusParameters(int bitrate, int sample_point, int sjw)
{
    Q_UNUSED(sample_point)
    Q_UNUSED(sjw)

    if (!checkOpen()) return false;

    ZenoBitTiming cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBitTiming));
    cmd.h.cmd_id = ZENO_CMD_SET_BIT_TIMING;
    cmd.channel = quint8(channel_index);

    switch (bitrate) {
    case 1000000:
        cmd.brp   = 0;
        cmd.tseg1 = 30;
        cmd.tseg2 = 7;
        cmd.sjw   = 7;

        if ( channel_index >= 4) {
            /* NOTE: 1mbit is not supported on the Zeno CANquatro 42x
             * due to the internal clock of 70Mhz, there no possible way
             * to configure the ECAN to 1mbit */
            last_error_text =  "1Mbit bitrate is not supported on this channel";
            return false;
        }

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0002;
        cmd.cicfg2 = 0x02A0;
        break;

    case 500000:
        cmd.brp   = 0;
        cmd.tseg1 = 62;
        cmd.tseg2 = 15;
        cmd.sjw   = 15;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0006;
        cmd.cicfg2 = 0x01A1;
        break;

    case 250000:
        cmd.brp   = 0;
        cmd.tseg1 = 126;
        cmd.tseg2 = 31;
        cmd.sjw   = 31;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x000D;
        cmd.cicfg2 = 0x01A1;
        break;

    case 125000:
        cmd.brp   = 0;
        cmd.tseg1 = 254;
        cmd.tseg2 = 63;
        cmd.sjw   = 63;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x001B;
        cmd.cicfg2 = 0x01A1;
        break;

    case 100000:
        cmd.brp   = 14;
        cmd.tseg1 = 16;
        cmd.tseg2 = 7;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0022;
        cmd.cicfg2 = 0x01A1;
        break;

    case 83333:
    case 83000:
        cmd.brp   = 18;
        cmd.tseg1 = 16;
        cmd.tseg2 = 6;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0029;
        cmd.cicfg2 = 0x01A1;
        break;

    case 62000:
        cmd.brp   = 26;
        cmd.tseg1 = 15;
        cmd.tseg2 = 6;
        cmd.sjw   = 3;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x002E;
        cmd.cicfg2 = 0x02A9;
        break;

    case 50000:
        cmd.brp   = 30;
        cmd.tseg1 = 16;
        cmd.tseg2 = 7;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x0031;
        cmd.cicfg2 = 0x01B3;
        break;

    case 33333:
        cmd.brp   = 46;
        cmd.tseg1 = 16;
        cmd.tseg2 = 7;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 1;
        cmd.cicfg1 = 0x003D;
        cmd.cicfg2 = 0x04BA;
        break;

    case 10000:
        cmd.brp   = 156;
        cmd.tseg1 = 16;
        cmd.tseg2 = 7;
        cmd.sjw   = 0;

        /* ECAN1 and ECAN2 */
        cmd.cancks = 0;
        cmd.cicfg1 = 0x00FF;
        cmd.cicfg2 = 0x07BF;
        break;

    default:
        last_error_text =  QString("Unsupported bitrate %1").arg(bitrate);
        return false;
    }

    qDebug()  << "ZenoCAN-Ch" << channel_index  << "Bus params" << cmd.brp << cmd.tseg1 << cmd.tseg2 << cmd.sjw;

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed set bit timings: " << last_error_text;
        return false;
    }

    current_bitrate = bitrate;

    return true;
}

bool ZZenoCANChannel::setBusParametersFd(int bitrate, int sample_point, int sjw)
{
    Q_UNUSED(sample_point)
    Q_UNUSED(sjw)

    if (!checkOpen()) return false;

    ZenoBitTiming cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBitTiming));
    cmd.h.cmd_id = ZENO_CMD_SET_DATA_BIT_TIMING;
    cmd.channel = quint8(channel_index);

    switch (bitrate) {
    case 1000000:
        cmd.brp = 0;
        cmd.tseg1 = 30;
        cmd.tseg2 = 7;
        cmd.sjw = 7;
        // SSP
        cmd.tdc_offset = 31;
        cmd.tdc_value = 0;
        break;
    case 2000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 14;
        cmd.tseg2 = 3;
        cmd.sjw = 3;
        // SSP
        cmd.tdc_offset = 15;
        cmd.tdc_value = 0;
        break;
    case 3000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 8;
        cmd.tseg2 = 2;
        cmd.sjw = 2;
        // SSP
        cmd.tdc_offset = 9;
        cmd.tdc_value = 0;
        break;
    case 4000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 6;
        cmd.tseg2 = 1;
        cmd.sjw = 1;
        // SSP
        cmd.tdc_offset = 7;
        cmd.tdc_value = 0;
        break;
    case 5000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 4;
        cmd.tseg2 = 1;
        cmd.sjw = 1;
        // SSP
        cmd.tdc_offset = 5;
        cmd.tdc_value = 0;
        break;
    case 6700000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 3;
        cmd.tseg2 = 0;
        cmd.sjw = 0;
        // SSP
        cmd.tdc_offset = 4;
        cmd.tdc_value = 0;
        break;
    case 8000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 2;
        cmd.tseg2 = 0;
        cmd.sjw = 0;
        // SSP
        cmd.tdc_offset = 3;
        cmd.tdc_value = 1;
        break;
    case 10000000:
        // Data BR
        cmd.brp = 0;
        cmd.tseg1 = 1;
        cmd.tseg2 = 0;
        cmd.sjw = 0;
        // SSP
        cmd.tdc_offset = 2;
        cmd.tdc_value = 0;
        break;

    case 500000:
        cmd.brp = 1;
        cmd.tseg1 = 30;
        cmd.tseg2 = 7;
        cmd.sjw = 7;
        // SSP
        cmd.tdc_offset = 31;
        cmd.tdc_value = 0;
        cmd.tdc_ssp_mode_off = 1;
        break;
    case 833000:
        cmd.brp = 1;
        cmd.tseg1 = 17;
        cmd.tseg2 = 4;
        cmd.sjw = 4;
        // SSP
        cmd.tdc_offset = 18;
        cmd.tdc_value = 0;
        cmd.tdc_ssp_mode_off = 1;
        break;
    case 1500000:
        cmd.brp = 0;
        cmd.tseg1 = 18;
        cmd.tseg2 = 5;
        cmd.sjw = 5;
        // SSP
        cmd.tdc_offset = 19;
        cmd.tdc_value = 0;
        break;
    default:
        last_error_text =  QString("Unsupported data-bitrate %1").arg(bitrate);
        return false;
    }

    qDebug()  << "ZenoCAN-Ch" << channel_index  << "Bus params" << cmd.brp << cmd.tseg1 << cmd.tseg2 << cmd.sjw;

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed set data bit timings: " << last_error_text;
        return false;
    }

    return true;
}

bool ZZenoCANChannel::setDriverMode(VxCANFlags::DriverMode driver_mode)
{
    Q_UNUSED(driver_mode)

    /* TODO: implement me .... */
    return true;
}

VxCANFlags::ReadResult ZZenoCANChannel::readWait(long& id, unsigned char* msg,
                                                  unsigned int& dlc,
                                                  unsigned int& flags,
                                                  qint64& driver_timestmap_in_us,
                                                  qint64& appl_timestamp_in_us,
                                                  int timeout_in_ms)
{
    rx_message_fifo_mutex.lock();
    if ( rx_message_fifo.isEmpty()) {

        /* Wait for RX FIFO */
        rx_message_fifo_cond.wait(&rx_message_fifo_mutex, ulong(timeout_in_ms));

        /* If RX FIFO is still empty */
        if ( rx_message_fifo.isEmpty() ) {
            onReadTimeoutCheck();

            rx_message_fifo_mutex.unlock();
            return ReadTimeout;
        }
    }

    FifoRxCANMessage rx = rx_message_fifo.read();
    rx_message_fifo_mutex.unlock();

    flags = 0;
    id = long(rx.id);
    int msg_bit_count = 0;
    if ( rx.flags & ZenoCANFlagExtended ) {
        flags |= Extended;
        msg_bit_count = 38 + 25;
    } else if ( rx.flags & ZenoCANFlagStandard ) {
        flags |= Standard;
        msg_bit_count = 19 + 25;
    }

    if (rx.flags & ZenoCANErrorFrame ) {
        flags |= ErrorFrame | InternalFrame;
        flags |= (rx.flags & ZenoCANErrorMask);
    }
    if (rx.flags & ZenoCANErrorHWOverrun)
        flags |= ErrorHWOverrun;
    // if (rx.flags & LOG_MESSAGE_FLAG_NERR)
    //    flag |= NError;
    //if (rx.flags & LOG_MESSAGE_FLAG_WAKEUP)
    //    flag |= Wakeup;
    //if (rx.flags & LOG_MESSAGE_FLAG_REMOTE_FRAME)
    //    flag |= Rtr;
    if (rx.flags & ZenoCANFlagTxAck)
        flags |= TxMsgAcknowledge;

    if ( is_canfd_mode ) {
        if (rx.flags & ZenoCANFlagFD)
            flags |= CanFDFrame;

        if (rx.flags & ZenoCANFlagFDBRS)
            flags |= CanFDBitrateSwitch;
        //TODO: Check flag CanFDESI
    }
    // Q_ASSERT(rx.dlc <= 8);

    dlc = qMin(uint(rx.dlc),uint(is_canfd_mode ? 64 : 8));
    msg_bit_count += dlc * 8;
    bus_active_bit_count += msg_bit_count;

    driver_timestmap_in_us = qint64(rx.timestamp / base_clock_divisor);

    // qint64 device_time;
    // getDeviceTimeInUs(device_time);
    // qDebug() << "DS" << driver_timestmap_in_us << device_time << (device_time-driver_timestmap_in_us);

    appl_timestamp_in_us = caluclateTimeStamp(driver_timestmap_in_us);
    memcpy(msg, rx.data, rx.dlc);

    return ReadStatusOK;
}

VxCANFlags::SendResult ZZenoCANChannel::send(const long id,
                                              const unsigned char* msg,
                                              const unsigned int dlc,
                                              const unsigned int flags,
                                              int timeout_in_ms)
{

    if (!checkOpen()) return SendError;
    if (is_canfd_mode && (flags &  VxCANChannel::CanFDFrame))
        return sendFD(id,msg,dlc,flags,timeout_in_ms);
    // qDebug() << "ZENO_CMD_CAN20_TX_REQUEST" << hex << id;;

    ZenoTxCAN20Request request;
    request.h.cmd_id = ZENO_CMD_CAN20_TX_REQUEST;
    request.channel = quint8(channel_index);
    request.h.transaction_id = tx_next_trans_id & 0x7f;
    tx_next_trans_id ++;

    request.id = quint32(id);
    request.dlc = dlc & 0x0F;
    request.flags = 0;

    if (flags & Extended) {
        request.flags |= ZenoCANFlagExtended;
    } else if (flags & Standard ){
        request.flags |= ZenoCANFlagStandard;
    } else {
        return SendInvalidParam;
    }

    memcpy(request.data, msg, 8);

    tx_message_fifo_mutex.lock();
    tx_request_count ++;
    if ( tx_request_count >= int(max_outstanding_tx_requests) ) {
        if (timeout_in_ms > 0) {
            if ( !waitForSpaceInTxFifo(timeout_in_ms) ) {
                last_error_text = "Timeout request waiting for space in transmit buffer";
                tx_request_count --;
                tx_message_fifo_mutex.unlock();
                return SendTimeout;
            }
        } else {
            last_error_text ="Transmit request buffer overflow";
            tx_request_count --;
            tx_message_fifo_mutex.unlock();
            return TransmitBufferOveflow;
        }
    }

    if (! usb_can_device->queueTxRequest(zenoRequest(request), timeout_in_ms)) {
        last_error_text = usb_can_device->getLastErrorText();
        tx_request_count --;
        tx_message_fifo_mutex.unlock();

        return SendError;
    }

    // qDebug() << " enqueue" << (tx_next_trans_id & 0x7f) << tx_request_count;
    // qDebug() << " enqueue " << request.cmdIOPSeq.transId;

    FifoTxCANMessage* tx_request = tx_message_fifo.writePtr();
    tx_request->id = quint32(id);
    tx_request->flags = request.flags;
    tx_request->transaction_id = request.h.transaction_id;
    tx_request->dlc = request.dlc;
    memcpy(tx_request->data, msg, 8);

    tx_message_fifo.write();
    tx_message_fifo_mutex.unlock();

    return SendStatusOK;
}

VxCANFlags::SendResult ZZenoCANChannel::sendFD(const long id,
                                                const unsigned char* msg,
                                                const unsigned int dlc,
                                                const unsigned int flags,
                                                int timeout_in_ms)
{
    bool dlc_valid = false;
    if ( dlc <= 8 ) {
        dlc_valid = true;
    } else {
        switch(dlc) {
        case 12:
        case 16:
        case 20:
        case 24:
        case 32:
        case 48:
        case 64:
            dlc_valid = true;
            break;
        default:
            dlc_valid = false;
        }
    }

    if (!dlc_valid) {
        last_error_text = "Invalid parameter, allowed data length can be one of the following 0-8, 12, 16, 20, 24, 32, 48, 64.";
        return SendInvalidParam;
    }

    ZenoTxCANFDRequestP1 p1_request;
    p1_request.h.cmd_id = ZENO_CMD_CANFD_P1_TX_REQUEST;
    p1_request.channel = quint8(channel_index);
    p1_request.h.transaction_id = tx_next_trans_id & 0x7f;
    tx_next_trans_id ++;

    p1_request.id = quint32(id);
    p1_request.dlc = quint8(dlc);
    p1_request.flags = 0;

    if (flags & Extended) {
        p1_request.flags |= ZenoCANFlagExtended;
    } else if (flags & Standard ){
        p1_request.flags |= ZenoCANFlagStandard;
    } else {
        return SendInvalidParam;
    }
    if ( flags & CanFDBitrateSwitch ) {
        p1_request.flags |= ZenoCANFlagFDBRS;
    }

    memcpy(p1_request.data, msg, qMin(dlc, uint(20)));

    tx_message_fifo_mutex.lock();
    tx_request_count ++;
    if ( tx_request_count >= int(max_outstanding_tx_requests) ) {
        if (timeout_in_ms > 0) {
            if ( !waitForSpaceInTxFifo(timeout_in_ms) ) {
                last_error_text = "Timeout request waiting for space in transmit buffer";
                tx_request_count --;
                tx_message_fifo_mutex.unlock();
                return SendTimeout;
            }
        } else {
            last_error_text ="Transmit request buffer overflow";
            tx_request_count --;
            tx_message_fifo_mutex.unlock();
            return TransmitBufferOveflow;
        }
    }

    if (! usb_can_device->queueTxRequest(zenoRequest(p1_request), timeout_in_ms)) {
        last_error_text = usb_can_device->getLastErrorText();
        tx_request_count --;
        tx_message_fifo_mutex.unlock();

        return SendError;
    }

    if ( dlc > 20 ) {
        ZenoTxCANFDRequestP2 p2_request;
        p2_request.h.cmd_id = ZENO_CMD_CANFD_P2_TX_REQUEST;
        p2_request.channel = quint8(channel_index);
        p2_request.h.transaction_id = p1_request.h.transaction_id;
        p2_request.dlc = p1_request.dlc;

        memcpy(p2_request.data, msg + 20, qMin(dlc-20, uint(28)));

        if (! usb_can_device->queueTxRequest(zenoRequest(p2_request), timeout_in_ms)) {
            last_error_text = usb_can_device->getLastErrorText();
            tx_request_count --;
            tx_message_fifo_mutex.unlock();

            return SendError;
        }

        if ( dlc > 48 ) {
            ZenoTxCANFDRequestP3 p3_request;
            p3_request.h.cmd_id = ZENO_CMD_CANFD_P3_TX_REQUEST;
            p3_request.channel = quint8(channel_index);
            p3_request.h.transaction_id = p1_request.h.transaction_id;
            p3_request.dlc = p1_request.dlc;

            memcpy(p3_request.data, msg + 48, qMin(dlc-48, uint(16)));

            if (! usb_can_device->queueTxRequest(zenoRequest(p3_request), timeout_in_ms)) {
                last_error_text = usb_can_device->getLastErrorText();
                tx_request_count --;
                tx_message_fifo_mutex.unlock();

                return SendError;
            }
        }
    }

    // qDebug() << " enqueue" << (tx_next_trans_id & 0x7f) << tx_request_count;
    // qDebug() << " enqueue " << request.cmdIOPSeq.transId;
    // tx_message_fifo.write(request);
    FifoTxCANMessage* tx_request = tx_message_fifo.writePtr();
    tx_request->id = quint32(id);
    tx_request->flags = p1_request.flags;
    tx_request->transaction_id = p1_request.h.transaction_id;
    tx_request->dlc = p1_request.dlc;
    memcpy(tx_request->data, msg, qMin(dlc, uint(64)));

    tx_message_fifo.write();
    tx_message_fifo_mutex.unlock();

    return SendStatusOK;
}

bool ZZenoCANChannel::adjustTimeDrift(bool initial_adjustment)
{
    if (!initial_adjustment && usb_can_device->isInitClockCalibrationDone()) {
        /* Just do a read device clock - so variables are updated */
        getDeviceClock();
        return false;
    }

    if (!adjustDeviceTimeDrift(initial_adjustment)) return false;
    if (initial_adjustment) initial_timer_adjustment_done.ref();

    return true;
}

quint64 ZZenoCANChannel::getSerialNumber()
{
    quint64 unique_device_nr = serial_number;
    unique_device_nr <<= 8;
    unique_device_nr |= (channel_index & 0xff);

    return serial_number;
}

int ZZenoCANChannel::getBusLoad()
{
    qint64 t0 = VxTimerBase::getCurrentTimeInUs();
    qint64 delta_time_in_us = t0 - last_measure_time_in_us;
    delta_time_in_us = qMax(delta_time_in_us, qint64(1));

    // qDebug() << " busload delta time " << delta_time_in_ms << " active " << bus_active_bit_count << " rate " << bitrate;

    qint64 busload = (bus_active_bit_count * 100000000) / (current_bitrate * delta_time_in_us);
    last_measure_time_in_us = t0;
    bus_active_bit_count = 0;
    if ( busload > 100 ) busload = 100;

    return int(busload);
}

void ZZenoCANChannel::queueMessage(ZenoCAN20Message& message)
{
    QMutexLocker lock(&rx_message_fifo_mutex);

    if ( rx_message_fifo.available() == 0 ) {
        qDebug() << "ZenoCAN-Ch" << channel_index << "Zeno: RX buffer overflow" << rx_message_fifo.count() << rx_message_fifo.isEmpty();
        return;
    }

    rx_message_fifo_cond.wakeOne();

    FifoRxCANMessage* rx_message = rx_message_fifo.writePtr();
    rx_message->timestamp = message.timestamp | (quint64(message.timestamp_msb) << 32);
    rx_message->id        = message.id;
    rx_message->flags     = message.flags;
    rx_message->dlc       = message.dlc;
    memcpy(rx_message->data, message.data,8);

    rx_message_fifo.write();
}

void ZZenoCANChannel::queueMessageCANFDP1(ZenoCANFDMessageP1 &message_p1)
{
    // QByteArray b = QByteArray::fromRawData(reinterpret_cast<char*>(message_p1.data), int(18));
    // qDebug() << "P1 data" << b.toHex('-');

    if ( message_p1.dlc <= 18 ) {
        QMutexLocker lock(&rx_message_fifo_mutex);

        if ( rx_message_fifo.available() == 0 ) {
            qDebug() << "ZenoCAN-Ch" << channel_index << "P1 - Zeno: RX buffer overflow" << rx_message_fifo.count() << rx_message_fifo.isEmpty();
            return;
        }

        rx_message_fifo_cond.wakeOne();

        FifoRxCANMessage* rx_message = rx_message_fifo.writePtr();
        rx_message->timestamp = message_p1.timestamp;
        rx_message->id        = message_p1.id;
        rx_message->flags     = message_p1.flags;
        rx_message->dlc       = message_p1.dlc;
        memcpy(rx_message->data, message_p1.data, message_p1.dlc);

        rx_message_fifo.write();
    }
    else {
        canfd_msg_p1 = message_p1;
    }
}

void ZZenoCANChannel::queueMessageCANFDP2(ZenoCANFDMessageP2 &message_p2)
{
    // QByteArray b = QByteArray::fromRawData(reinterpret_cast<char*>(message_p2.data), int(28));
    // qDebug() << "P2 data" << b.toHex('-') << (canfd_msg_p1.dlc - 18) << canfd_msg_p1.dlc;

    if ( canfd_msg_p1.dlc <= 46 ) {
        QMutexLocker lock(&rx_message_fifo_mutex);

        if ( rx_message_fifo.available() == 0 ) {
            qDebug() << "ZenoCAN-Ch" << channel_index << "P2 - Zeno: RX buffer overflow" << rx_message_fifo.count() << rx_message_fifo.isEmpty();
            return;
        }

        rx_message_fifo_cond.wakeOne();

        FifoRxCANMessage* rx_message = rx_message_fifo.writePtr();
        rx_message->timestamp = canfd_msg_p1.timestamp;
        rx_message->id        = canfd_msg_p1.id;
        rx_message->flags     = canfd_msg_p1.flags;
        rx_message->dlc       = canfd_msg_p1.dlc;

        memcpy(rx_message->data,      canfd_msg_p1.data, 18);
        memcpy(rx_message->data + 18, message_p2.data,   canfd_msg_p1.dlc - 18);
        rx_message_fifo.write();
    }
    else {
        canfd_msg_p2 = message_p2;
    }
}

void ZZenoCANChannel::queueMessageCANFDP3(ZenoCANFDMessageP3 &message_p3)
{
    // QByteArray b = QByteArray::fromRawData(reinterpret_cast<char*>(message_p3.data), int(18));
    // qDebug() << "P3 data" << b.toHex('-');

    if ( canfd_msg_p1.dlc < 46 ) {

        memset(&canfd_msg_p1, 0 , sizeof(canfd_msg_p1));
        memset(&canfd_msg_p2, 0 , sizeof(canfd_msg_p2));
        return;
    }

    QMutexLocker lock(&rx_message_fifo_mutex);

    if ( rx_message_fifo.available() == 0 ) {
        qDebug() << "ZenoCAN-Ch" << channel_index << "P3 - Zeno: RX buffer overflow" << rx_message_fifo.count() << rx_message_fifo.isEmpty();
        return;
    }

    rx_message_fifo_cond.wakeOne();

    FifoRxCANMessage* rx_message = rx_message_fifo.writePtr();
    rx_message->timestamp = canfd_msg_p1.timestamp;
    rx_message->id        = canfd_msg_p1.id;
    rx_message->flags     = canfd_msg_p1.flags;
    rx_message->dlc       = canfd_msg_p1.dlc;

    memcpy(rx_message->data,      canfd_msg_p1.data, 18);
    memcpy(rx_message->data + 18, canfd_msg_p2.data, 28);
    memcpy(rx_message->data + 46, message_p3.data,   canfd_msg_p1.dlc - 46);

    rx_message_fifo.write();

    memset(&canfd_msg_p1, 0 , sizeof(canfd_msg_p1));
    memset(&canfd_msg_p2, 0 , sizeof(canfd_msg_p2));
}

void ZZenoCANChannel::txAck(ZenoTxCANRequestAck& tx_ack)
{
    tx_message_fifo_mutex.lock();
    tx_message_fifo_cond.wakeOne();

    // static int count = 0;

    int tx_count = int(tx_message_fifo.count());

    for (int i = 0; i < tx_count ; ++i ) {
        FifoTxCANMessage m = tx_message_fifo.read();

        if ( m.transaction_id == tx_ack.trans_id )  {
            FifoRxCANMessage message;

            message.id = m.id;
            message.dlc = m.dlc;
            message.flags = quint8(m.flags | ZenoCANFlagTxAck);
            // message.dlc = tx_ack.dlc;
            message.timestamp = tx_ack.timestamp | (quint64(tx_ack.timestamp_msb) << 32);
            memcpy(message.data, m.data, m.dlc);

            // tx_message_fifo.removeAt(i);
            tx_request_count --;
            if ( tx_request_count == 0 ) {
            //      qDebug() << "ZenoCAN-Ch" << channel_index  << "TX done" << tx_ack.trans_id; //  << count;
            }
            Q_ASSERT(tx_request_count >= 0);

            tx_message_fifo_mutex.unlock();

            if ( tx_ack.flags & ZenoCANErrorFrame ) {
                qDebug() << "ZenoCAN-Ch" << channel_index  << "TX failed, remove pending TX";
                flushTxFifo();
                return;
            }

            rx_message_fifo_mutex.lock();

            if ( rx_message_fifo.available() == 0 ) {
                qDebug() << "Zeno: RX buffer overflow (TxACK)" << rx_message_fifo.count() << rx_message_fifo.isEmpty();
                rx_message_fifo_mutex.unlock();
                return;
            }
            rx_message_fifo.write(message);
            rx_message_fifo_cond.wakeOne();

            rx_message_fifo_mutex.unlock();

            return;
        }

        qDebug() << "ZenoCAN-Ch" << channel_index << "*** Writing back to TX fifo" << m.transaction_id << tx_ack.trans_id << hex << m.id;
        tx_message_fifo.write(m);
    }

    tx_message_fifo_mutex.unlock();
    qCritical()  << "ZenoCAN-Ch" << channel_index  << "-- TX ack with transId: " << tx_ack.trans_id  << " not queued";
}

bool ZZenoCANChannel::getZenoDeviceTimeInUs(uint64_t& timestamp_in_us)
{
    if (!checkOpen()) return false;

    ZenoReadClock cmd;
    ZenoReadClockResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOn));
    cmd.h.cmd_id = ZENO_CMD_READ_CLOCK;
    cmd.channel = channel_index;

    if (!usb_can_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = usb_can_device->getLastErrorText();
        qCritical() << "ERROR(ZenoUSB) Ch" << channel_index << " failed to read Zeno device clock: " << last_error_text;
        return false;
    }

    timestamp_in_us = reply.clock_value;
    timestamp_in_us /= reply.divisor;

    return true;
}

bool ZZenoCANChannel::getDeviceTimeInUs(qint64& timestamp_in_us)
{
    if (!checkOpen()) return false;

    /* Read the clock from the Zeno device */
    if (!getZenoDeviceTimeInUs(timestamp_in_us)) return false;
    adjustDeviceTimerWrapAround(timestamp_in_us);

    return true;
}

void ZZenoCANChannel::synchToTimer(VxTimerBase* timer)
{
    VxGenericPacketDevice::synchToTimerOffset(timer);
}

uint64_t ZZenoCANChannel::getDeviceClock()
{
    if (!checkOpen()) return false;

    qint64 t;
    /* Read the clock from the Zeno device */
    if (!getZenoDeviceTimeInUs(t)) return false;

    timer_adjust_mutex.lock();
    adjustDeviceTimerWrapAround(t);
    t = calculateTimeDrift(t);
    timer_adjust_mutex.unlock();

    return t;
}

ZCANDriver* ZZenoCANChannel::getCANDriver() const
{
    return usb_can_device->getZenoDriver();
}

void ZZenoCANChannel::flushRxFifo()
{
    QMutexLocker lock(&tx_message_fifo_mutex);
    rx_message_fifo.clear();
}

void ZZenoCANChannel::flushTxFifo()
{
    QMutexLocker lock(&tx_message_fifo_mutex);
    tx_message_fifo.clear();
    tx_request_count = 0;
    tx_next_trans_id = 0;

    /* TODO: send flush CMD to Zeno */
}

bool ZZenoCANChannel::checkOpen()
{
    if (is_open.load() == 0) {
        last_error_text = QString("CAN Channel %1 is not open").arg(channel_index);
        return false;
    }

    return true;
}

bool ZZenoCANChannel::waitForSpaceInTxFifo(int& timeout_in_ms)
{
    VxReference<VxTimer> tx_timeout_timer = new VxTimer();
    int time_left_in_ms = timeout_in_ms;
    while ( time_left_in_ms > 0 &&
            tx_message_fifo.count() >=  max_outstanding_tx_requests) {
        bool wait_success;

        wait_success = tx_message_fifo_cond.wait(&tx_message_fifo_mutex, time_left_in_ms);
        // qDebug()  << "ZenoCAN-Ch" << channel_index  << "wait for TX space" << tx_message_fifo.count() << tx_ack_received << wait_success;

        time_left_in_ms -= tx_timeout_timer->elapsedInMs();

        if (!wait_success) break;
    }

    timeout_in_ms = time_left_in_ms;

    return tx_message_fifo.count() < max_outstanding_tx_requests;
}
