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

#include "zzenolinchannel.h"
#include "zzenousbdevice.h"
#include "zzenolindriver.h"
#include "zenocan.h"
#include "zdebug.h"
#include <string.h>
#include <algorithm>

/*** ---------------------------==*+*+*==---------------------------------- ***/
uint8_t linCalculatePIDParity(uint8_t pid)
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

uint8_t linCalculateClassicCRC(uint8_t* data, uint8_t dlc)
{
    uint16_t __crc = 0;

    for (int i = 0; i < dlc; i++){
        __crc = __crc + data[i];
        if(__crc > 0xFF)
            __crc -= 0xFF;
    }
    __crc = ~__crc;

    return __crc & 0xff;
}

uint8_t linCalculateEnhancedCRC(uint8_t pid, uint8_t* data, uint8_t dlc)
{
    uint16_t __crc = pid;

    for (int i = 0; i < dlc; i++){
        __crc = __crc + data[i];
        if(__crc > 0xFF)
            __crc -= 0xFF;
    }
    __crc = ~__crc;

    return __crc & 0xff;
}

/*** ---------------------------==*+*+*==---------------------------------- ***/
ZZenoLINChannel::ZZenoLINChannel(int _channel_index, int _display_index,
                                 ZZenoUSBDevice *_usb_can_device)
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
    // connect(zeno_usb_device, &ZZenoUSBDevice::destroyed, [this]() {
    //     zeno_usb_device = nullptr;
    // });
}

ZZenoLINChannel::~ZZenoLINChannel()
{
    if (is_open) close();
}

const std::string ZZenoLINChannel::getObjectText() const
{
    std::string channel_name = usb_display_name + ' ';

    channel_name += "LIN" + std::to_string(display_index) + ' ';
    channel_name += '(' + std::to_string(serial_number) + ')';

    return channel_name;
}

const std::string ZZenoLINChannel::getLastErrorText()
{
    return last_error_text;
}

bool ZZenoLINChannel::open(bool master)
{
    is_open++;

    if (is_open.load() > 1) {
        is_open--;
        last_error_text = "LIN Channel " + std::to_string(channel_index+1) + " is already open";
        return false;
    }

    if (!zeno_usb_device->open()) {
        is_open--;
        return false;
    }

    ZenoLinOpen cmd;
    ZenoOpenResponse reply;

    memset(&cmd,0,sizeof(ZenoLinOpen));
    cmd.h.cmd_id = ZENO_CMD_LIN_OPEN;
    cmd.channel = uint8_t(channel_index);
    cmd.is_master = master;
    cmd.base_clock_divisor = uint8_t(zeno_usb_device->getClockResolution() / 1000);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to open Zeno LIN channel: %s", display_index, last_error_text.c_str());
        close();
        return false;
    }

    is_master = master;
    tx_pending = false;

    return true;
}

bool ZZenoLINChannel::close()
{
    busOff();

    ZenoClose cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoClose));
    cmd.h.cmd_id = ZENO_CMD_LIN_CLOSE;
    cmd.channel = uint8_t(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to close Zeno LIN channel: %s", display_index, last_error_text.c_str());
    }

    is_master = false;
    auto_baud_set = false;

    zeno_usb_device->close();
    is_open--;

    return true;
}

uint32_t ZZenoLINChannel::getCapabilites()
{
    uint32_t capabilities;

    capabilities =
            BusStatistics |
            GenerateError |
            GenerateOverload |
            TxAcknowledge;

    return capabilities;
}

bool ZZenoLINChannel::busOn()
{
    if (!checkOpen()) return false;

    ZenoBusOn cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOn));
    cmd.h.cmd_id = ZENO_CMD_LIN_BUS_ON;
    cmd.channel = uint8_t(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to go bus-on: %s", display_index, last_error_text.c_str());
        return false;
    }

    return true;
}

bool ZZenoLINChannel::busOff()
{
    if (!checkOpen()) return false;

    ZenoBusOff cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOff));
    cmd.h.cmd_id = ZENO_CMD_LIN_BUS_OFF;
    cmd.channel = uint8_t(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to go bus-off: %s", display_index, last_error_text.c_str());
        return false;
    }

    return true;
}

bool ZZenoLINChannel::setBusParameters(int bitrate)
{
    if (!checkOpen()) return false;

    if (bitrate == 0 && is_master) {
        last_error_text = "ERROR: LIN" + std::to_string(display_index) + " auto-baud is only supported for LIN slaves";
        return false;
    }
    ZenoLinBitrate cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoLinBitrate));
    cmd.h.cmd_id = ZEMO_CMD_LIN_SET_BITRATE;
    cmd.channel = uint8_t(channel_index);
    cmd.bitrate = uint16_t(bitrate);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to set bitrate: %s", display_index, last_error_text.c_str());
        return false;
    }

    auto_baud_set = (bitrate == 0) ? true : false;

    return true;
}

bool ZZenoLINChannel::setSlaveResponse(uint8_t pid, const uint8_t *data,
                                        uint8_t data_length, uint32_t tx_flags)
{
    if (!checkOpen()) return false;

    ZenoTxLINRequest cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoTxLINRequest));
    cmd.h.cmd_id = ZENO_CMD_LIN_UPDATE_MESSAGE;
    cmd.channel = uint8_t(channel_index);
    cmd.pid = pid & 0x3f;
    cmd.dlc = std::min(data_length, uint8_t(8));
    memcpy(cmd.data, data, cmd.dlc);
    cmd.flags = tx_flags;

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to update slave response: %s", display_index, last_error_text.c_str());
        return false;
    }

    return true;
}

bool ZZenoLINChannel::clearSlaveResponse(uint8_t pid)
{
    if (!checkOpen()) return false;

    ZenoLinClearMessage cmd;
    ZenoResponse reply;

    cmd.h.cmd_id = ZENO_CMD_LIN_CLEAR_MESSAGE;
    cmd.channel = uint8_t(channel_index);
    cmd.pid = pid & 0x3f;

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to update slave response: %s", display_index, last_error_text.c_str());
        return false;
    }

    return true;
}

bool ZZenoLINChannel::clearSlaveResponses()
{
    if (!checkOpen()) return false;

    ZenoLinClearMessage cmd;
    ZenoResponse reply;

    cmd.h.cmd_id = ZENO_CMD_LIN_CLEAR_ALL_MESSAGES;
    cmd.channel = uint8_t(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to clear slave response: %s", display_index, last_error_text.c_str());
        return false;
    }

    return true;
}

bool ZZenoLINChannel::readFromRXFifo(ZenoLINMessage& rx, uint64_t timeout_in_ms)
{
    std::unique_lock<std::mutex> lock(rx_message_fifo_mutex);

    zDebug("rx_message_fifo.isEmpty() %d - %Ld", rx_message_fifo.isEmpty(), timeout_in_ms);
    if ( rx_message_fifo.isEmpty()) {
        std::chrono::milliseconds timeout(timeout_in_ms);

        /* Wait for RX FIFO */
        rx_message_fifo_cond.wait_for(lock,timeout);

        /* If RX FIFO is still empty */
        if ( rx_message_fifo.isEmpty() ) {
            rx_message_fifo_mutex.unlock();
            return false;
        }
    }

    rx = rx_message_fifo.read();

    return true;
}

ZLINChannel::ReadResult ZZenoLINChannel::readWait(uint8_t& pid, uint8_t *msg,
                                                  uint8_t& data_length,
                                                  uint32_t& flags,
                                                  uint64_t& timestamp_in_us,
                                                  int timeout_in_ms)
{
    ZenoLINMessage rx;
    if (!readFromRXFifo(rx, uint64_t(timeout_in_ms))) return ReadTimeout;

    pid = rx.pid & 0x3f;
    flags = rx.flags;

    if ( linCalculatePIDParity(rx.pid) != rx.pid ) {
        flags |= ParityError;
        flags |= ErrorFrame;
        if (auto_baud_set) resetAutoBaud();
    }

    // Q_ASSERT(rx.dlc <= 8);
    data_length = std::min(rx.dlc,uint8_t(8));
    timestamp_in_us = uint64_t(rx.timestamp_end / 70);
    memcpy(msg, rx.data, sizeof(rx.data));

    /* Calcuate and check CRC */
    if ( !(rx.flags & ZenoLINNoData) ) {
        /* First check with LIN 2.x enhanced CRC */
        uint8_t __crc = linCalculateEnhancedCRC(rx.pid, rx.data, data_length);
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

ZLINChannel::SendResult ZZenoLINChannel::send(uint8_t id,
                                                const uint8_t *msg,
                                                uint8_t data_length,
                                                uint32_t flags,
                                                int timeout_in_ms)
{
    if (!checkOpen()) return SendError;

    ZenoTxLINRequest cmd;
    ZenoResponse reply;
    {
        std::unique_lock<std::mutex> lock(tx_message_mutex);
        if (!waitForTX(lock,timeout_in_ms)) {
            zDebug("LIN send timeout");
            tx_message_mutex.unlock();
            return SendTimeout;
        }

        memset(&cmd,0,sizeof(ZenoLinBitrate));
        cmd.h.cmd_id = ZENO_CMD_LIN_TX_MESSAGE;
        cmd.channel = uint8_t(channel_index);

        cmd.pid = id;
        cmd.dlc = std::min(data_length, uint8_t(8));
        cmd.flags = flags & 0xff;
        cmd.is_master_request = false;
        memcpy(cmd.data, msg, cmd.dlc);

        tx_pending = true;
        tx_message = cmd;
    }
    // cmd.bitrate = uint16_t(bitrate);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d transmit failed: %s", display_index, last_error_text.c_str());

        tx_message_mutex.lock();
        tx_pending = false;
        tx_message_cond.notify_one();
        tx_message_mutex.unlock();

        return SendError;
    }

    return SendStatusOK;
}

ZLINChannel::SendResult ZZenoLINChannel::sendMasterRequest(uint8_t pid,
                                                             uint32_t flags)
{
    if (!checkOpen()) return SendError;

    ZenoTxLINRequest cmd;
    ZenoResponse reply;
    {
        std::unique_lock<std::mutex> lock(tx_message_mutex);
        if (!waitForTX(lock,100)) {
            tx_message_mutex.unlock();

            return SendTimeout;
        }

        memset(&cmd,0,sizeof(ZenoLinBitrate));
        cmd.h.cmd_id = ZENO_CMD_LIN_TX_MESSAGE;
        cmd.channel = uint8_t(channel_index);

        cmd.pid = pid;
        cmd.dlc = 0;
        cmd.flags = flags;
        cmd.is_master_request = true;

        tx_pending = true;
        tx_message = cmd;
    }
    zDebug("LIN%d Send master request", display_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d master request failed: %s", display_index, last_error_text.c_str());

        tx_message_mutex.lock();
        tx_pending = false;
        tx_message_cond.notify_one();
        tx_message_mutex.unlock();

        return SendError;
    }

    return SendError;
}

ZLINChannel::SendResult ZZenoLINChannel::sendWakeup(uint32_t flags)
{
    ZUNUSED(flags)
    /* TODO: implement me ... */

    return SendError;
}

uint64_t ZZenoLINChannel::getSerialNumber()
{
    return serial_number;
}

ZLINDriver *ZZenoLINChannel::getLINDriver() const
{
    return zeno_usb_device->getZenoLINDriver();
}

void ZZenoLINChannel::queueMessage(ZenoLINMessage &message)
{
    std::lock_guard<std::mutex> lock(rx_message_fifo_mutex);

    if ( rx_message_fifo.available() == 0 ) {
        zDebug("(ZenoUSB) LIN%d Zeno: RX buffer overflow: %d - %d", channel_index+1,rx_message_fifo.count(), rx_message_fifo.isEmpty());
        return;
    }

    rx_message_fifo_cond.notify_one();
    rx_message_fifo.write(message);
}

void ZZenoLINChannel::txAck(ZenoTxLINRequestAck &tx_ack)
{
    std::lock_guard<std::mutex> lock(tx_message_mutex);

    if ( tx_pending ) {
        if ( !tx_message.is_master_request ) {
            ZenoLINMessage rx;

            rx.h = tx_ack.h;
            rx.pid = linCalculatePIDParity(tx_message.pid);
            rx.flags = tx_ack.flags | ZenoLINTxMsgAcknowledge;
            rx.dlc = tx_message.dlc;
            memcpy(rx.data, tx_message.data, sizeof(rx.data));

            if ( rx.flags | ClassicChecksum ) {
                rx.checksum = linCalculateClassicCRC(rx.data, std::min(uint8_t(8), rx.dlc));
            }
            else {
                rx.checksum = linCalculateEnhancedCRC(rx.pid, rx.data, std::min(uint8_t(8), rx.dlc));
            }

            rx.timestamp_end = tx_ack.timestamp_end;
            rx.timestamp_start = tx_ack.timestamp_start;

            tx_message_mutex.unlock();
            queueMessage(rx);
            tx_message_mutex.lock();
        }

        tx_pending = false;
        tx_message_cond.notify_one();
    }
}

bool ZZenoLINChannel::checkOpen()
{
    if (is_open.load() == 0) {
        last_error_text = "LIN" + std::to_string(display_index) + " channel is not open";
        return false;
    }

    return true;
}

bool ZZenoLINChannel::waitForTX(std::unique_lock<std::mutex>& lock, int timeout_in_ms)
{
    if ( tx_pending ) {
        std::chrono::milliseconds timeout(timeout_in_ms);
        tx_message_cond.wait_for(lock, timeout);
    }

    return !tx_pending;
}

bool ZZenoLINChannel::resetAutoBaud()
{
    if (!checkOpen()) return false;

    ZenoLinResetAutoBaud cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoLinResetAutoBaud));
    cmd.h.cmd_id = ZENO_CMD_LIN_RESET_AUTO_BAUD;
    cmd.channel = uint8_t(channel_index);

    if (!zeno_usb_device->sendAndWhaitReply(zenoRequest(cmd), zenoReply(reply))) {
        last_error_text = zeno_usb_device->getLastErrorText();
        zCritical("(ZenoUSB) LIN%d failed to reset auto-baud: %s", display_index, last_error_text.c_str());

        return false;
    }

    return true;
}
