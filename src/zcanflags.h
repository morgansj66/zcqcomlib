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

#ifndef ZCANFLAGS_H
#define ZCANFLAGS_H

#include "zglobal.h"

class ZCANFlags {
public:
    enum MessageFlagsMask {
        Rtr                 = 0x00001,
        Standard            = 0x00002,
        Extended            = 0x00004,
        Wakeup              = 0x00008,
        NError              = 0x00010,
        ErrorFrame          = 0x00020,
        TxMsgAcknowledge    = 0x00040,
        TxMsgRequest        = 0x00080,
        ErrorMask           = 0x0ff00,
        ErrorHWOverrun      = 0x00200,
        ErrorSWOverrun      = 0x00400,
        ErrorStuff          = 0x00800,
        ErrorForm           = 0x01000,
        ErrorCRC            = 0x02000,
        ErrorBIT0           = 0x04000,
        ErrorBIT1           = 0x08000,
        InternalFrame       = 0x10000,
        ISO15765ExtAddr     = 0x20000,
        ISO15765UnknownType = 0x40000,
        CanFDFrame          = 0x80000,
        CanFDBitrateSwitch  = 0x100000,
        CanFDESI            = 0x200000
    };

    enum CapabilitesMask {
        ExtendedCAN      = 0x00000001L,
        BusStatistics    = 0x00000002L,
        ErrorCounters    = 0x00000004L,
        CanDiagnostics   = 0x00000008L,
        GenerateError    = 0x00000010L,
        GenerateOverload = 0x00000020L,
        TxRequest        = 0x00000040L,
        TxAcknowledge    = 0x00000080L,
        Virtual          = 0x00010000L,
        Simulated        = 0x00020000L,
        Remote           = 0x00040000L,
        CanFD            = 0x00080000L,
        CanFDNonISO      = 0x00100000L,
        SharedMode       = 0x80000000L
    };

    enum DriverMode {
        Off = 0,
        Silent = 1,
        Normal = 4,
        SelfReception = 8
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
        SendError = -4
    };
};

#endif /* ZCANFLAGS_H */
