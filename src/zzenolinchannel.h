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

#ifndef ZZENOLINCHANNEL_H
#define ZZENOLINCHANNEL_H

#include "zlinchannel.h"
#include "zenocan.h"
#include "zring.h"

#include <QWaitCondition>
#include <QMutex>

class VxZenoUSBDevice;
class VxZenoLINChannel : public VxLINChannel {
    Q_OBJECT
public:
    VxZenoLINChannel(int _channel_index, int _display_index, VxZenoUSBDevice* _usb_can_device);
    ~VxZenoLINChannel() override;

    const QString getObjectText() const override;

    const QString getLastErrorText() override;

    bool open(bool master) override;

    bool close() override;

    quint32 getCapabilites() override;

    bool busOn() override;

    bool busOff() override;

    bool setBusParameters(int bitrate) override;

    bool setSlaveResponse(quint8 pid,
                          const quint8 *data,
                          quint8 data_length,
                          quint32 tx_flags) override;

    bool clearSlaveResponse(quint8 id) override;

    bool clearSlaveResponses(void) override;

    ReadResult readWait(quint8& pid, unsigned char *msg,
                        quint8& data_length, quint32& flags,
                        qint64& timestamp_in_us,
                        int timeout_in_ms) override;
    // We should possibly add a pointer to an extened message info struct.
    SendResult send(quint8 id,
                    const quint8 *msg,
                    quint8 data_length,
                    quint32 flags,
                    int timeout_in_ms) override;

    SendResult sendMasterRequest(quint8 id, quint32 flags) override;
    SendResult sendWakeup(quint32 flags) override;

    quint64 getSerialNumber() override;

    VxLINDriver* getLINDriver() const override;

    void queueMessage(ZenoLINMessage& message);
    void txAck(ZenoTxLINRequestAck& tx_ack);

private:
    bool checkOpen();
    bool waitForTX(int timeout_in_ms);
    bool resetAutoBaud();

    int channel_index;
    int display_index;
    QAtomicInt is_open;
    bool is_master;
    bool auto_baud_set;
    VxZenoUSBDevice* zeno_usb_device;

    QString last_error_text;
    QString usb_display_name;
    quint64 serial_number;

    /* RX logic */
    QMutex rx_message_fifo_mutex;
    QWaitCondition rx_message_fifo_cond;
    VxRing<ZenoLINMessage> rx_message_fifo;

    /* TX logic */
    QMutex tx_message_mutex;
    QWaitCondition tx_message_cond;
    bool tx_pending;
    ZenoTxLINRequest tx_message;

};

#endif /* VXZENOLINCHANNEL_H */
