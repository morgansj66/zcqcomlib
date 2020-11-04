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

#include <condition_variable>
#include <mutex>

class ZZenoUSBDevice;
class ZZenoLINChannel : public ZLINChannel {
public:
    ZZenoLINChannel(int _channel_index, int _display_index, ZZenoUSBDevice* _usb_can_device);
    ~ZZenoLINChannel() override;

    const std::string getObjectText() const;

    const std::string getLastErrorText() override;

    bool open(bool master) override;

    bool close() override;

    uint32_t getCapabilites() override;

    bool busOn() override;

    bool busOff() override;

    bool setBusParameters(int bitrate) override;

    bool setSlaveResponse(uint8_t pid,
                          const uint8_t *data,
                          uint8_t data_length,
                          uint32_t tx_flags) override;

    bool clearSlaveResponse(uint8_t id) override;

    bool clearSlaveResponses(void) override;

    ReadResult readWait(uint8_t& pid, uint8_t *msg,
                        uint8_t& data_length, uint32_t& flags,
                        uint64_t& timestamp_in_us,
                        int timeout_in_ms) override;
    // We should possibly add a pointer to an extened message info struct.
    SendResult send(uint8_t id,
                    const uint8_t *msg,
                    uint8_t data_length,
                    uint32_t flags,
                    int timeout_in_ms) override;

    SendResult sendMasterRequest(uint8_t id, uint32_t flags) override;
    SendResult sendWakeup(uint32_t flags) override;

    uint64_t getSerialNumber() override;

    ZLINDriver* getLINDriver() const override;

    void queueMessage(ZenoLINMessage& message);
    void txAck(ZenoTxLINRequestAck& tx_ack);

private:
    bool checkOpen();
    bool waitForTX(std::unique_lock<std::mutex>& lock, int timeout_in_ms);
    bool resetAutoBaud();
    bool readFromRXFifo(ZenoLINMessage& rx, uint64_t timeout_in_ms);

    int channel_index;
    int display_index;
    std::atomic<int> is_open;
    bool is_master;
    bool auto_baud_set;
    ZZenoUSBDevice* zeno_usb_device;

    std::string last_error_text;
    std::string usb_display_name;
    uint64_t serial_number;

    /* RX logic */
    std::mutex rx_message_fifo_mutex;
    std::condition_variable rx_message_fifo_cond;
    ZRing<ZenoLINMessage> rx_message_fifo;

    /* TX logic */
    std::mutex tx_message_mutex;
    std::condition_variable tx_message_cond;
    bool tx_pending;
    ZenoTxLINRequest tx_message;
};

#endif /* ZZENOLINCHANNEL_H */
