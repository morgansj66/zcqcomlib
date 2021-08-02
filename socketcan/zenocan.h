// SPDX-License-Identifier: GPL-2.0
/* SocketCAN driver for the Zuragon CANquatro 
 * CAN and CAN FD USB devices
 *
 * Copyright(C) 2021 Zuragon LTd - www.zuragon.com
 */

#ifndef ZENOCAN_H
#define ZENOCAN_H

#include <linux/types.h>

#define ZENO_CMD_SIZE 32
#define ZENO_EXT_CMD_SIZE 64

#define zenoRequest(c) (ZenoCmd*)(&c)
#define zenoReply(c)   (ZenoResponse*)(&c)

enum ZenoCmdID {
    ZENO_CMD_RESET,
    ZENO_CMD_INFO,
    ZENO_CMD_OPEN,
    ZENO_CMD_CLOSE,
    ZENO_CMD_RESPONSE,
    ZENO_CMD_BUS_ON,
    ZENO_CMD_BUS_OFF,
    ZENO_CMD_SET_BIT_TIMING,

    ZENO_CMD_CAN20_TX_REQUEST,
    ZENO_CMD_CANFD_TX_REQUEST,

    ZENO_CMD_READ_CLOCK,
    ZENO_DEBUG,
    
    ZENO_CMD_CAN_TX_ACK,
    ZENO_CMD_CAN_RX,
    
    ZEMO_CMD_START_CLOCK_INT,
    ZEMO_CMD_STOP_CLOCK_INT,
    ZENO_CLOCK_INFO_INT,
    
    ZENO_CMD_CANFD_P1_RX,
    ZENO_CMD_CANFD_P2_RX,
    ZENO_CMD_CANFD_P3_RX,

    ZENO_CMD_CANFD_P1_TX_REQUEST,
    ZENO_CMD_CANFD_P2_TX_REQUEST,
    ZENO_CMD_CANFD_P3_TX_REQUEST,
    
    ZENO_CMD_READ_CAN_CHIP_CLOCK,
    
    /* LIN commands */
    ZENO_CMD_LIN_OPEN,
    ZENO_CMD_LIN_CLOSE,
    ZEMO_CMD_LIN_SET_BITRATE,
    ZENO_CMD_LIN_BUS_ON,
    ZENO_CMD_LIN_BUS_OFF,
    ZENO_CMD_LIN_UPDATE_MESSAGE,
    ZENO_CMD_LIN_CLEAR_MESSAGE,
    ZENO_CMD_LIN_CLEAR_ALL_MESSAGES,
    ZENO_CMD_LIN_TX_MESSAGE,
    ZENO_CMD_LIN_TX_ACK,
    ZENO_CMD_LIN_RX_MESSAGE,
    
    ZENO_CMD_RESERVED1,
    ZENO_CMD_RESERVED2,
    ZENO_CMD_RESERVED3,
    ZENO_CMD_RESERVED4,
    ZENO_CMD_RESERVED5,
    ZENO_CMD_RESERVED6,
    ZENO_CMD_RESERVED7,

    /* CAN FD */
    ZENO_CMD_SET_DATA_BIT_TIMING,

    /* LIN command */
    ZENO_CMD_LIN_RESET_AUTO_BAUD,

    /* CAN operation mode */
    ZENO_CMD_SET_OP_MODE,
};

enum ZenoOpModeID {
    ZENO_NORMAL_CAN20_ONLY_MODE,
    ZENO_NORMAL_CANFD_MODE,
    ZENO_SILENT_MODE,
};

#define ZenoCAPExtendedCAN      0x00000001
#define ZenoCAPBusStatistics    0x00000002
#define ZenoCAPErrorCounters    0x00000004
#define ZenoCAPCanDiagnostics   0x00000008
#define ZenoCAPGenerateError    0x00000010
#define ZenoCAPGenerateOverload 0x00000020
#define ZenoCAPTxRequest        0x00000040
#define ZenoCAPTxAcknowledge    0x00000080
#define ZenoCAPRemote           0x00040000
#define ZenoCAPCanFD            0x00080000
#define ZenoCAPCanFDNonISO      0x00100000

#define ZenoCANFlagRTR          0x00001
#define ZenoCANFlagStandard     0x00002
#define ZenoCANFlagExtended     0x00004
#define ZenoCANFlagFD           0x00008
#define ZenoCANFlagFDBRS        0x00010
#define ZenoCANFlagFDNext       0x00010
#define ZenoCANErrorFrame       0x00020
#define ZenoCANFlagTxAck        0x00040
#define ZenoCANErrorHWOverrun   0x00080
#define ZenoCANErrorMask        0x0ff00
#define ZenoCANErrorStuff       0x00800
#define ZenoCANErrorForm        0x01000
#define ZenoCANErrorCRC         0x02000
#define ZenoCANErrorBIT0        0x04000
#define ZenoCANErrorBIT1        0x08000

/* LIN flags */
#define ZenoLINParityError       0x0001  // Rx: parity error (the identifier)
#define ZenoLINChecksumError     0x0002  // Rx: checksum error
#define ZenoLINNoData            0x0004  // Rx: header only
#define ZenoLINBitError          0x0008  // Tx: transmitted 1, got 0 or vice versa
#define ZenoLINTxMsgAcknowledge  0x0040

/* LIN TX flags */
#define ZenoLINEnhancedCRC       0x0080

#define ZenoCANStatusOpNormal    0x01
#define ZenoCANStatusOpConf      0x02
#define ZenoCANStatusTXPassive   0x03
#define ZenoCANStatusRXPassive   0x04
#define ZenoCANStatusTXWarning   0x05
#define ZenoCANStatusRXWarning   0x06
#define ZenoCANStatusErrorBusOff 0x07
#define ZenoCANInvalidMessage    0x08


#pragma pack(push, 1)

typedef struct {
    u8 cmd_id;
    u8 transaction_id;
} ZenoHeader;

typedef struct {
    ZenoHeader h;

    /* Up to 30 bytes payload */
    u8 cmd_payload[30];
} ZenoCmd;

typedef struct {
    ZenoHeader h;

    /* Up to 14 bytes payload */
    u8 cmd_payload[14];
} ZenoIntCmd;

typedef struct {
    ZenoHeader h;
    
    u32 clock_value_t0;
    u64 clock_value_t1;
    u8 clock_divisor;
    u8 usb_overflow_count;
} ZenoIntClockInfoCmd;

typedef struct {
    ZenoHeader h;
    u8 response_cmd_id;
    u8 cmd_result_code;

    u8 response_payload[28];
} ZenoResponse;

typedef struct {
    ZenoHeader h;
    u8 response_cmd_id;
    u8 cmd_result_code;

    u32 capabilities;

    u32 fw_version;
    u32 serial_number;

    u8 can_channel_count;
    u8 lin_channel_count;

    u8 hw_revision;
    u8 type_code;

    u16 clock_resolution; /* In Khz */
    
    /* Fill up up to 32 bytes */
    u8 unused[10];
} ZenoInfoResponse;

typedef struct {
    ZenoHeader h;
    u8 channel;
    
    /* Fill up up to 32 bytes */
    u8 unused[29];
} ZenoReadClock;

typedef struct {
    ZenoHeader h;
    u8 response_cmd_id;
    u8 cmd_result_code;
    
    u64 clock_value;
    u32 clock_wrap_around_count;
    u8 read_count;
    u8 divisor;
    
    u8 unused[14];
} ZenoReadClockResponse;

typedef struct {
    ZenoHeader h;
    u8 channel;
    u8 base_clock_divisor;
    u8 can_fd_mode;
    u8 can_fd_non_iso;

    /* Fill up up to 32 bytes */
    u8 unused[28];
} ZenoOpen;

typedef struct {
    ZenoHeader h;
    u8 channel;
    u8 base_clock_divisor;
    u8 is_master;
    
    /* Fill up up to 32 bytes */
    u8 unused[27];
} ZenoLinOpen;

typedef struct {
    ZenoHeader h;
    u8 response_cmd_id;
    u8 cmd_result_code;
    
    u64 clock_start_ref;
    u16 max_pending_tx_msgs;

    u8 base_clock_divisor;

    /* Fill up up to 32 bytes */
    u8 unused[17];
} ZenoOpenResponse;

typedef struct {
    ZenoHeader h;
    u8 channel;

    /* Fill up up to 32 bytes */
    u8 unused[29];
} ZenoClose;

typedef struct {
    ZenoHeader h;
    u8 channel;

    /* Fill up up to 32 bytes */
    u8 unused[29];
} ZenoBusOn;

typedef struct {
    ZenoHeader h;
    u8 channel;

    /* Fill up up to 32 bytes */
    u8 unused[29];
} ZenoBusOff;

typedef struct {
    ZenoHeader h;
    u8 channel;

    u8 brp;
    u8 tseg1;
    u8 tseg2;
    u8 sjw;

    /* For ECAN1 and ECAN2 */
    u8  cancks; /* 1: FCAN = 2 * Fp - 0: FCAN = Fp */
    u16 cicfg1;
    u16 cicfg2;
    
    /* CAN FD - Transmitter delay compensation */
    u8 tdc_offset;
    u8 tdc_value;
    u8 tdc_ssp_mode_off;

    /* Fill up up to 32 bytes */
    u8 unused[17];
} ZenoBitTiming;

typedef struct {
    ZenoHeader h;
    u8 channel;
    u8 unused1;
    
    u16 bitrate;

    /* Fill up up to 32 bytes */
    u8 unused2[26];
} ZenoLinBitrate;

typedef struct {
    ZenoHeader h;
    u8 channel;
    u8 unused1;

    /* Fill up up to 32 bytes */
    u8 unused2[28];
} ZenoLinResetAutoBaud;

typedef struct {
    ZenoHeader h;
    u8 channel;
    u8 op_mode;

    /* Fill up up to 32 bytes */
    u8 unused[29];
} ZenoOpMode;

typedef struct {
    ZenoHeader h;
    
    u8 trans_id;
    u8 flags;
    
    u32 id;
    u32 timestamp;
    
    u8 data[8];

    u8 dlc;
    u8 channel;

    u32 timestamp_msb;

    /* Fill up up to 32 bytes */
    u8 unused2[6];
} ZenoCAN20Message;

typedef struct {
    ZenoHeader h;

    u8 trans_id;
    u8 flags;

    u8 pid;
    u8 checksum;
    
    u64 timestamp_start;
    u64 timestamp_end;
    
    u8 data[8];

    u8 dlc;
    u8 channel;
 } ZenoLINMessage;

typedef struct {
    ZenoHeader h;

    u8 trans_id;
    u8 flags;

    u32 id;
    u32 timestamp;

    u8 data[18];

    u8 dlc;
    u8 channel;
} ZenoCANFDMessageP1;

typedef struct {
    ZenoHeader h;
    
    u8 trans_id;
    u8 channel;

    u8 data[28];
} ZenoCANFDMessageP2;

typedef struct {
    ZenoHeader h;

    u8 trans_id;
    u8 channel;

    u8 data[18];

    /* Fill up up to 32 bytes */
    u8 unused[10];
} ZenoCANFDMessageP3;

typedef struct {
    ZenoHeader h;

    u32 id;
    u32 flags;

    u8 dlc;
    u8 channel;

    u8 data[8];

    /* Fill up up to 32 bytes */
    u8 unused2[12];
} ZenoTxCAN20Request;

typedef struct {
    ZenoHeader h;

    u8 pid;
    u8 is_master_request;
    u32 flags;

    u8 dlc;
    u8 channel;

    u8 data[8];

    /* Fill up up to 32 bytes */
    u8 unused2[14];
} ZenoTxLINRequest;

typedef struct {
    ZenoHeader h;

    u32 id;
    u32 flags;

    u8 dlc;
    u8 channel;

    u8 data[20];
} ZenoTxCANFDRequestP1;

typedef struct {
    ZenoHeader h;

    u8 dlc;
    u8 channel;

    u8 data[28];
} ZenoTxCANFDRequestP2;

typedef struct {
    ZenoHeader h;

    u8 dlc;
    u8 channel;

    u8 data[16];
} ZenoTxCANFDRequestP3;

typedef struct {
    ZenoHeader h;
    u8 trans_id;
    u8 flags;
    
    u32 id;
    u32 timestamp;

    u8 dlc;
    u8 channel;
    
    u32 timestamp_msb;

    /* Fill up up to 32 bytes */    
    u8 unused2[10];
} ZenoTxCANRequestAck;

typedef struct {
    ZenoHeader h;
    u8 trans_id;
    u8 flags;
    
    u64 timestamp_start;
    u64 timestamp_end;
    u8 channel;
    
    /* Fill up up to 32 bytes */    
    u8 unused2[11];
} ZenoTxLINRequestAck;

typedef struct {
    ZenoHeader h;

} ZenoTxAck;

typedef struct {
    ZenoHeader h;

    /*  Debug msg */
    u8 debug_msg[30];
} ZenoDebug;

typedef struct {
    ZenoHeader h;

    /* Fill up up to 32 bytes */
    u8 unused[30];
} ZenoStartStopClockInt;

typedef struct {
    ZenoHeader h;
    u8 channel;
    u8 pid;
    
    /* Fill up up to 32 bytes */
    u8 unused[28];
} ZenoLinClearMessage;

typedef struct {
    ZenoHeader h;
    u8 channel;
    
    /* Fill up up to 32 bytes */
    u8 unused[29];
} ZenoLinClearAllMessages;

#pragma pack(pop)

#endif /* ZENOCAN_H */
