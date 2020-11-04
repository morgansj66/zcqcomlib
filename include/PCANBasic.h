/*
 *   Copyright 2020 by Morgan
 *
 * This software BSD-new. See the included COPYING file for details.
 *
 * License: BSD-new
 * ==============================================================================
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *     * Neither the name of the \<organization\> nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
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

#ifndef PCANBASIC_H
#define PCANBASIC_H

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#include <Windows.h>
#define ZTEXT(x) ZTEXT(x)
#else
#include <stdint.h>
#define ZTEXT(x) x
typedef uint8_t BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef uint64_t UINT64;
typedef char* LPSTR;
#endif

#ifdef __cplusplus
extern "C" {
#define Z_DEFAULT_ARG =0
#else
#define Z_DEFAULT_ARG
#endif

#define PCAN_NONEBUS 0x00U
#define PCAN_ISABUS1 0x21U
#define PCAN_ISABUS2 0x22U
#define PCAN_ISABUS3 0x23U
#define PCAN_ISABUS4 0x24U
#define PCAN_ISABUS5 0x25U
#define PCAN_ISABUS6 0x26U
#define PCAN_ISABUS7 0x27U
#define PCAN_ISABUS8 0x28U
#define PCAN_DNGBUS1 0x31U
#define PCAN_PCIBUS1 0x41U
#define PCAN_PCIBUS2 0x42U
#define PCAN_PCIBUS3 0x43U
#define PCAN_PCIBUS4 0x44U
#define PCAN_PCIBUS5 0x45U
#define PCAN_PCIBUS6 0x46U
#define PCAN_PCIBUS7 0x47U
#define PCAN_PCIBUS8 0x48U
#define PCAN_PCIBUS9 0x409U
#define PCAN_PCIBUS10 0x40AU
#define PCAN_PCIBUS11 0x40BU
#define PCAN_PCIBUS12 0x40CU
#define PCAN_PCIBUS13 0x40DU
#define PCAN_PCIBUS14 0x40EU
#define PCAN_PCIBUS15 0x40FU
#define PCAN_PCIBUS16 0x410U
#define PCAN_USBBUS1 0x51U
#define PCAN_USBBUS2 0x52U
#define PCAN_USBBUS3 0x53U
#define PCAN_USBBUS4 0x54U
#define PCAN_USBBUS5 0x55U
#define PCAN_USBBUS6 0x56U
#define PCAN_USBBUS7 0x57U
#define PCAN_USBBUS8 0x58U
#define PCAN_USBBUS9 0x509U
#define PCAN_USBBUS10 0x50AU
#define PCAN_USBBUS11 0x50BU
#define PCAN_USBBUS12 0x50CU
#define PCAN_USBBUS13 0x50DU
#define PCAN_USBBUS14 0x50EU
#define PCAN_USBBUS15 0x50FU
#define PCAN_USBBUS16 0x510U
#define PCAN_PCCBUS1 0x61U
#define PCAN_PCCBUS2 0x62U
#define PCAN_LANBUS1 0x801U
#define PCAN_LANBUS2 0x802U
#define PCAN_LANBUS3 0x803U
#define PCAN_LANBUS4 0x804U
#define PCAN_LANBUS5 0x805U
#define PCAN_LANBUS6 0x806U
#define PCAN_LANBUS7 0x807U
#define PCAN_LANBUS8 0x808U
#define PCAN_LANBUS9 0x809U
#define PCAN_LANBUS10 0x80AU
#define PCAN_LANBUS11 0x80BU
#define PCAN_LANBUS12 0x80CU
#define PCAN_LANBUS13 0x80DU
#define PCAN_LANBUS14 0x80EU
#define PCAN_LANBUS15 0x80FU
#define PCAN_LANBUS16 0x810U
#define PCAN_ERROR_OK 0x00000U
#define PCAN_ERROR_XMTFULL 0x00001U
#define PCAN_ERROR_OVERRUN 0x00002U
#define PCAN_ERROR_BUSLIGHT 0x00004U
#define PCAN_ERROR_BUSHEAVY 0x00008U
#define PCAN_ERROR_BUSWARNING PCAN_ERROR_BUSHEAVY
#define PCAN_ERROR_BUSPASSIVE 0x40000U
#define PCAN_ERROR_BUSOFF 0x00010U
#define PCAN_ERROR_ANYBUSERR (PCAN_ERROR_BUSWARNING | PCAN_ERROR_BUSLIGHT | PCAN_ERROR_BUSHEAVY | PCAN_ERROR_BUSOFF | PCAN_ERROR_BUSPASSIVE)
#define PCAN_ERROR_QRCVEMPTY 0x00020U
#define PCAN_ERROR_QOVERRUN 0x00040U
#define PCAN_ERROR_QXMTFULL 0x00080U
#define PCAN_ERROR_REGTEST 0x00100U
#define PCAN_ERROR_NODRIVER 0x00200U
#define PCAN_ERROR_HWINUSE 0x00400U
#define PCAN_ERROR_NETINUSE 0x00800U
#define PCAN_ERROR_ILLHW 0x01400U
#define PCAN_ERROR_ILLNET 0x01800U
#define PCAN_ERROR_ILLCLIENT 0x01C00U
#define PCAN_ERROR_ILLHANDLE (PCAN_ERROR_ILLHW | PCAN_ERROR_ILLNET | PCAN_ERROR_ILLCLIENT)
#define PCAN_ERROR_RESOURCE 0x02000U
#define PCAN_ERROR_ILLPARAMTYPE 0x04000U
#define PCAN_ERROR_ILLPARAMVAL 0x08000U
#define PCAN_ERROR_UNKNOWN 0x10000U
#define PCAN_ERROR_ILLDATA 0x20000U
#define PCAN_ERROR_CAUTION 0x2000000U
#define PCAN_ERROR_INITIALIZE 0x4000000U
#define PCAN_ERROR_ILLOPERATION 0x8000000U
#define PCAN_NONE 0x00U
#define PCAN_PEAKCAN 0x01U
#define PCAN_ISA 0x02U
#define PCAN_DNG 0x03U
#define PCAN_PCI 0x04U
#define PCAN_USB 0x05U
#define PCAN_PCC 0x06U
#define PCAN_VIRTUAL   0x07U
#define PCAN_LAN 0x08U
#define PCAN_DEVICE_NUMBER 0x01U
#define PCAN_5VOLTS_POWER 0x02U
#define PCAN_RECEIVE_EVENT 0x03U
#define PCAN_MESSAGE_FILTER 0x04U
#define PCAN_API_VERSION 0x05U
#define PCAN_CHANNEL_VERSION 0x06U
#define PCAN_BUSOFF_AUTORESET 0x07U
#define PCAN_LISTEN_ONLY 0x08U
#define PCAN_LOG_LOCATION 0x09U
#define PCAN_LOG_STATUS 0x0AU
#define PCAN_LOG_CONFIGURE 0x0BU
#define PCAN_LOG_TEXT 0x0CU
#define PCAN_CHANNEL_CONDITION 0x0DU
#define PCAN_HARDWARE_NAME 0x0EU
#define PCAN_RECEIVE_STATUS 0x0FU
#define PCAN_CONTROLLER_NUMBER 0x10U
#define PCAN_TRACE_LOCATION 0x11U
#define PCAN_TRACE_STATUS 0x12U
#define PCAN_TRACE_SIZE 0x13U
#define PCAN_TRACE_CONFIGURE 0x14U
#define PCAN_CHANNEL_IDENTIFYING 0x15U
#define PCAN_CHANNEL_FEATURES 0x16U
#define PCAN_BITRATE_ADAPTING 0x17U
#define PCAN_BITRATE_INFO 0x18U
#define PCAN_BITRATE_INFO_FD 0x19U
#define PCAN_BUSSPEED_NOMINAL 0x1AU
#define PCAN_BUSSPEED_DATA 0x1BU
#define PCAN_IP_ADDRESS 0x1CU
#define PCAN_PARAMETER_OFF 0x00U
#define PCAN_PARAMETER_ON 0x01U
#define PCAN_FILTER_CLOSE 0x00U
#define PCAN_FILTER_OPEN 0x01U
#define PCAN_FILTER_CUSTOM 0x02U
#define PCAN_CHANNEL_UNAVAILABLE 0x00U
#define PCAN_CHANNEL_AVAILABLE 0x01U
#define PCAN_CHANNEL_OCCUPIED 0x02U
#define PCAN_CHANNEL_PCANVIEW (PCAN_CHANNEL_AVAILABLE |  PCAN_CHANNEL_OCCUPIED)
#define LOG_FUNCTION_DEFAULT 0x00U
#define LOG_FUNCTION_ENTRY 0x01U
#define LOG_FUNCTION_PARAMETERS 0x02U
#define LOG_FUNCTION_LEAVE 0x04U
#define LOG_FUNCTION_WRITE 0x08U
#define LOG_FUNCTION_READ  0x10U
#define LOG_FUNCTION_ALL 0xFFFFU
#define TRACE_FILE_SINGLE 0x00U
#define TRACE_FILE_SEGMENTED 0x01U
#define TRACE_FILE_DATE 0x02U
#define TRACE_FILE_TIME 0x04U
#define TRACE_FILE_OVERWRITE 0x80U
#define FEATURE_FD_CAPABLE 0x01U
#define PCAN_MESSAGE_STANDARD 0x00U
#define PCAN_MESSAGE_RTR 0x01U
#define PCAN_MESSAGE_EXTENDED 0x02U
#define PCAN_MESSAGE_FD 0x04U
#define PCAN_MESSAGE_BRS 0x08U
#define PCAN_MESSAGE_ESI 0x10U
#define PCAN_MESSAGE_STATUS 0x80U
#define PCAN_MODE_STANDARD PCAN_MESSAGE_STANDARD
#define PCAN_MODE_EXTENDED PCAN_MESSAGE_EXTENDED
#define PCAN_BAUD_1M 0x0014U
#define PCAN_BAUD_800K 0x0016U
#define PCAN_BAUD_500K 0x001CU
#define PCAN_BAUD_250K 0x011CU
#define PCAN_BAUD_125K 0x031CU
#define PCAN_BAUD_100K 0x432FU
#define PCAN_BAUD_95K 0xC34EU
#define PCAN_BAUD_83K 0x852BU
#define PCAN_BAUD_50K 0x472FU
#define PCAN_BAUD_47K 0x1414U
#define PCAN_BAUD_33K 0x8B2FU
#define PCAN_BAUD_20K 0x532FU
#define PCAN_BAUD_10K 0x672FU
#define PCAN_BAUD_5K  0x7F7FU
#define PCAN_BR_CLOCK ZTEXT("f_clock")
#define PCAN_BR_CLOCK_MHZ ZTEXT("f_clock_mhz")
#define PCAN_BR_NOM_BRP ZTEXT("nom_brp")
#define PCAN_BR_NOM_TSEG1 ZTEXT("nom_tseg1")
#define PCAN_BR_NOM_TSEG2 ZTEXT("nom_tseg2")
#define PCAN_BR_NOM_SJW  ZTEXT("nom_sjw")
#define PCAN_BR_NOM_SAMPLE ZTEXT("nom_sam")
#define PCAN_BR_DATA_BRP ZTEXT("data_brp")
#define PCAN_BR_DATA_TSEG1 ZTEXT("data_tseg1")
#define PCAN_BR_DATA_TSEG2 ZTEXT("data_tseg2")
#define PCAN_BR_DATA_SJW ZTEXT("data_sjw")
#define PCAN_BR_DATA_SAMPLE ZTEXT("data_ssp_offset")
#define PCAN_TYPE_ISA 0x01U
#define PCAN_TYPE_ISA_SJA 0x09U
#define PCAN_TYPE_ISA_PHYTEC 0x04U
#define PCAN_TYPE_DNG 0x02U
#define PCAN_TYPE_DNG_EPP 0x03U
#define PCAN_TYPE_DNG_SJA 0x05U
#define PCAN_TYPE_DNG_SJA_EPP 0x06U
#define TPCANHandle WORD
#define TPCANStatus DWORD
#define TPCANParameter BYTE
#define TPCANDevice BYTE
#define TPCANMessageType BYTE
#define TPCANType BYTE
#define TPCANMode BYTE
#define TPCANBaudrate WORD
#define TPCANBitrateFD LPSTR
#define TPCANTimestampFD UINT64

typedef struct tagTPCANMsg {
    DWORD ID;
    TPCANMessageType MSGTYPE;
    BYTE LEN;
    BYTE DATA[8];
} TPCANMsg;

typedef struct tagTPCANTimestamp {
    DWORD millis;
    WORD millis_overflow;
    WORD micros;
} TPCANTimestamp;

typedef struct tagTPCANMsgFD {
    DWORD ID;
    TPCANMessageType MSGTYPE;
    BYTE DLC;
    BYTE DATA[64];
} TPCANMsgFD;

TPCANStatus CAN_Initialize(TPCANHandle channel_handle, TPCANBaudrate baud_rate, TPCANType can_hw_type Z_DEFAULT_ARG, DWORD io_port Z_DEFAULT_ARG, WORD interrupt Z_DEFAULT_ARG);

TPCANStatus CAN_InitializeFD(TPCANHandle channel_handle, TPCANBitrateFD baud_rate);

TPCANStatus CAN_Uninitialize(TPCANHandle channel_handle);

TPCANStatus CAN_Reset(TPCANHandle channel_handle);

TPCANStatus CAN_GetStatus(TPCANHandle channel_handle);

TPCANStatus CAN_Read(TPCANHandle channel_handle, TPCANMsg* can_msg, TPCANTimestamp* timestamp);

TPCANStatus CAN_ReadFD(TPCANHandle channel_handle, TPCANMsgFD* can_msg, TPCANTimestampFD *timestamp);

TPCANStatus CAN_Write(TPCANHandle channel_handle, TPCANMsg* can_msg);

TPCANStatus CAN_WriteFD(TPCANHandle channel_handle, TPCANMsgFD* can_msg);

TPCANStatus CAN_FilterMessages(TPCANHandle channel_handle, DWORD from_can_id, DWORD to_can_id, TPCANMode can_mode);

TPCANStatus CAN_GetValue(TPCANHandle channel_handle, TPCANParameter can_parameter, void* buffer, DWORD buffer_length);

TPCANStatus CAN_SetValue(TPCANHandle channel_handle, TPCANParameter Parameter, void* Buffer, DWORD BufferLength);

TPCANStatus CAN_GetErrorText(TPCANStatus Error, WORD Language, LPSTR Buffer);

#ifdef __cplusplus
}
#endif

#endif /* PCANBASIC_H */
