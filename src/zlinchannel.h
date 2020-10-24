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

#ifndef ZLINCHANNEL_H_
#define ZLINCHANNEL_H_

#include "zrefcountingobjbase.h"
#include <string>

class ZLINDriver;
class ZLINChannel : public ZRefCountingObjBase {
protected:
    ZLINChannel() { }

public:
    enum CapabilitesMask {
        BusStatistics    = 0x00000002L,
        GenerateError    = 0x00000010L,
        GenerateOverload = 0x00000020L,
        TxRequest        = 0x00000040L,
        TxAcknowledge    = 0x00000080L,
        Virtual          = 0x00010000L,
        Simulated        = 0x00020000L,
        Remote           = 0x00040000L
        // Master Slave Spy ExtendedFrames
    };

    enum MessageFlagsMask {
        ParityError       = 0x0001, // Rx: parity error (the identifier)
        ChecksumError     = 0x0002, // Rx: checksum error
        NoData            = 0x0004, // Rx: header only
        BitError          = 0x0008, // Tx: transmitted 1, got 0 or vice versa
        TxSlaveResponse   = 0x0010, // Rx: echo of a slave response we transmitted
        ErrorFrame        = 0x0020, // Error frame
        TxMsgAcknowledge  = 0x0040,
        TxMsgRequest      = 0x0080,
        ErrorHWOverrun    = 0x0200, // Rx: LIN interface overrun
        ErrorSWOverrun    = 0x0400, // Rx: receive queue overrun
        SynchError        = 0x0800, // Synch error
        WakeUp            = 0x1000, // A wake up frame was received
        ClassicChecksum   = 0x2000, // Rx or Tx

    };

    enum ReadResult {
        ReadStatusOK = 0,
        ReadTimeout = -1,
        ReadError = -2
    };

    enum SendResult {
        SendStatusOK = 0,
        SendTimeout = -1,
        TransmitBufferOveflow = -2,
        SendInvalidParam = -3,
        SendError = -4,
        SendSlaveError = -5 // Only a master can transmit a message
    };

    virtual const std::string getDeviceDescription() const {
        /* Optionally implemented */
        return std::string();
    }

    virtual const std::string getLastErrorText() = 0;
    virtual bool open(bool master) = 0;
    virtual bool close() = 0;
    virtual uint32_t getCapabilites() = 0;
    virtual bool busOn() = 0;
    virtual bool busOff() = 0;
    virtual bool setBusParameters(int bitrate) = 0;
    virtual bool setSlaveResponse(uint8_t id,
                                  const uint8_t *data,
                                  uint8_t data_length,
                                  uint32_t tx_flags) = 0;
    virtual bool clearSlaveResponse(uint8_t id) = 0;
    virtual bool clearSlaveResponses(void) = 0;
    virtual ReadResult readWait(uint8_t& id, uint8_t *msg,
                                uint8_t& data_length, uint32_t& flags,
                                uint64_t& timestamp_in_us,
                                int timeout_in_ms) = 0;
    // We should possibly add a pointer to an extened message info struct.
    virtual SendResult send(uint8_t id,
                            const uint8_t *msg,
                            uint8_t data_length,
                            uint32_t flags,
                            int timeout_in_ms) = 0;
    virtual SendResult sendMasterRequest(uint8_t id, uint32_t flags) = 0;
    virtual SendResult sendWakeup(uint32_t flags) = 0;
    virtual uint64_t getSerialNumber() {
        /* Optionally implemented */
        return uint64_t(-1);
    }

    virtual ZLINDriver* getLINDriver() const = 0;
};

#endif /* VXLINCHANNEL_H_ */
