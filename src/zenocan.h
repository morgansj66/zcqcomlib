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

#ifndef ZENOCAN_H
#define ZENOCAN_H

#if defined(__XC16__) || defined(__XC8__)
#include <xc.h>
typedef uint8_t quint8;
typedef uint16_t quint16;
typedef uint32_t quint32;
typedef uint64_t quint64;
#else
#include <QtGlobal>
#endif

#define ZENO_CMD_SIZE 32
#define ZENO_EXT_CMD_SIZE 64

#define zenoRequest(c) reinterpret_cast<ZenoCmd*>(&c)
#define zenoReply(c)   reinterpret_cast<ZenoResponse*>(&c)

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
    
    /* FW update */
    ZENO_CMD_FLASH_START,
    ZENO_CMD_FLASH_READ_ROW_TO_BUFFER,
    ZENO_CMD_FLASH_READ_ROW_FROM_BUFFER,
    ZENO_CMD_FLASH_WRITE_TO_BUFFER,
    ZENO_CMD_FLASH_ERASE_ROW,
    ZENO_CMD_FLASH_WRITE_ROW_FROM_BUFFER,
    ZENO_CMD_FLASH_FINISH,

    /* CAN FD */
    ZENO_CMD_SET_DATA_BIT_TIMING,

    /* LIN command */
    ZENO_CMD_LIN_RESET_AUTO_BAUD,
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
    quint8 cmd_id;
    quint8 transaction_id;
} ZenoHeader;

typedef struct {
    ZenoHeader h;

    /* Up to 30 bytes payload */
    quint8 cmd_payload[30];
} ZenoCmd;

typedef struct {
    ZenoHeader h;

    /* Up to 14 bytes payload */
    quint8 cmd_payload[14];
} ZenoIntCmd;

typedef struct {
    ZenoHeader h;
    
    quint32 clock_value_t0;
    quint64 clock_value_t1;
    quint8 clock_divisor;
    quint8 usb_overflow_count;
} ZenoIntClockInfoCmd;

typedef struct {
    ZenoHeader h;
    quint8 response_cmd_id;
    quint8 cmd_result_code;

    quint8 response_payload[28];
} ZenoResponse;

typedef struct {
    ZenoHeader h;
    quint8 response_cmd_id;
    quint8 cmd_result_code;

    quint32 capabilities;

    quint32 fw_version;
    quint32 serial_number;

    quint8 can_channel_count;
    quint8 lin_channel_count;

    quint8 hw_revision;
    quint8 type_code;

    quint16 clock_resolution; /* In Khz */
    
    /* Fill up up to 32 bytes */
    quint8 unused[10];
} ZenoInfoResponse;

typedef struct {
    ZenoHeader h;
    quint8 channel;
    
    /* Fill up up to 32 bytes */
    quint8 unused[29];
} ZenoReadClock;

typedef struct {
    ZenoHeader h;
    quint8 response_cmd_id;
    quint8 cmd_result_code;
    
    quint64 clock_value;
    quint32 clock_wrap_around_count;
    quint8 read_count;
    quint8 divisor;
    
    quint8 unused[10];
} ZenoReadClockResponse;


typedef struct {
    ZenoHeader h;
    quint8 channel;
    quint8 base_clock_divisor;
    quint8 can_fd_mode;
    quint8 can_fd_non_iso;

    /* Fill up up to 32 bytes */
    quint8 unused[28];
} ZenoOpen;

typedef struct {
    ZenoHeader h;
    quint8 channel;
    quint8 base_clock_divisor;
    quint8 is_master;
    
    /* Fill up up to 32 bytes */
    quint8 unused[27];
} ZenoLinOpen;

typedef struct {
    ZenoHeader h;
    quint8 response_cmd_id;
    quint8 cmd_result_code;
    
    quint64 clock_start_ref;
    quint16 max_pending_tx_msgs;

    quint8 base_clock_divisor;

    /* Fill up up to 32 bytes */
    quint8 unused[17];
} ZenoOpenResponse;

typedef struct {
    ZenoHeader h;
    quint8 channel;

    /* Fill up up to 32 bytes */
    quint8 unused[29];
} ZenoClose;

typedef struct {
    ZenoHeader h;
    quint8 channel;

    /* Fill up up to 32 bytes */
    quint8 unused[29];
} ZenoBusOn;

typedef struct {
    ZenoHeader h;
    quint8 channel;

    /* Fill up up to 32 bytes */
    quint8 unused[29];
} ZenoBusOff;

typedef struct {
    ZenoHeader h;
    quint8 channel;

    quint8 brp;
    quint8 tseg1;
    quint8 tseg2;
    quint8 sjw;

    /* For ECAN1 and ECAN2 */
    quint8  cancks; /* 1: FCAN = 2 * Fp - 0: FCAN = Fp */
    quint16 cicfg1;
    quint16 cicfg2;
    
    /* CAN FD - Transmitter delay compensation */
    quint8 tdc_offset;
    quint8 tdc_value;
    quint8 tdc_ssp_mode_off;

    /* Fill up up to 32 bytes */
    quint8 unused[17];
} ZenoBitTiming;

typedef struct {
    ZenoHeader h;
    quint8 channel;
    quint8 unused1;
    
    quint16 bitrate;

    /* Fill up up to 32 bytes */
    quint8 unused2[26];    
} ZenoLinBitrate;

typedef struct {
    ZenoHeader h;
    quint8 channel;
    quint8 unused1;

    /* Fill up up to 32 bytes */
    quint8 unused2[28];
} ZenoLinResetAutoBaud;

typedef struct {
    ZenoHeader h;
    quint8 channel;
    quint8 op_mode;

    /* Fill up up to 32 bytes */
    quint8 unused[29];
} ZenoOpMode;

typedef struct {
    ZenoHeader h;
    
    quint8 trans_id;
    quint8 flags;
    
    quint32 id;
    quint32 timestamp;
    
    quint8 data[8];

    quint8 dlc;
    quint8 channel;

    quint32 timestamp_msb;

    /* Fill up up to 32 bytes */
    quint8 unused2[6];
} ZenoCAN20Message;

typedef struct {
    ZenoHeader h;

    quint8 trans_id;
    quint8 flags;

    quint8 pid;
    quint8 checksum;
    
    quint64 timestamp_start;
    quint64 timestamp_end;
    
    quint8 data[8];

    quint8 dlc;
    quint8 channel;
 } ZenoLINMessage;

typedef struct {
    ZenoHeader h;

    quint8 trans_id;
    quint8 flags;

    quint32 id;
    quint32 timestamp;

    quint8 data[18];

    quint8 dlc;
    quint8 channel;
} ZenoCANFDMessageP1;

typedef struct {
    ZenoHeader h;
    
    quint8 trans_id;
    quint8 channel;

    quint8 data[28];
} ZenoCANFDMessageP2;

typedef struct {
    ZenoHeader h;

    quint8 trans_id;
    quint8 channel;

    quint8 data[18];

    /* Fill up up to 32 bytes */
    quint8 unused[10];
} ZenoCANFDMessageP3;

typedef struct {
    ZenoHeader h;

    quint32 id;
    quint32 flags;

    quint8 dlc;
    quint8 channel;

    quint8 data[8];

    /* Fill up up to 32 bytes */
    quint8 unused2[12];
} ZenoTxCAN20Request;

typedef struct {
    ZenoHeader h;

    quint8 pid;
    quint8 is_master_request;
    quint32 flags;

    quint8 dlc;
    quint8 channel;

    quint8 data[8];

    /* Fill up up to 32 bytes */
    quint8 unused2[14];
} ZenoTxLINRequest;

typedef struct {
    ZenoHeader h;

    quint32 id;
    quint32 flags;

    quint8 dlc;
    quint8 channel;

    quint8 data[20];
} ZenoTxCANFDRequestP1;

typedef struct {
    ZenoHeader h;

    quint8 dlc;
    quint8 channel;

    quint8 data[28];
} ZenoTxCANFDRequestP2;

typedef struct {
    ZenoHeader h;

    quint8 dlc;
    quint8 channel;

    quint8 data[16];
} ZenoTxCANFDRequestP3;

typedef struct {
    ZenoHeader h;
    quint8 trans_id;
    quint8 flags;
    
    quint32 id;
    quint32 timestamp;

    quint8 dlc;
    quint8 channel;
    
    quint32 timestamp_msb;

    /* Fill up up to 32 bytes */    
    quint8 unused2[10];
} ZenoTxCANRequestAck;

typedef struct {
    ZenoHeader h;
    quint8 trans_id;
    quint8 flags;
    
    quint64 timestamp_start;
    quint64 timestamp_end;
    quint8 channel;
    
    /* Fill up up to 32 bytes */    
    quint8 unused2[11];       
} ZenoTxLINRequestAck;


typedef struct {
    ZenoHeader h;

} ZenoTxAck;

typedef struct {
    ZenoHeader h;

    /*  Debug msg */
    quint8 debug_msg[30];
} ZenoDebug;

typedef struct {
    ZenoHeader h;

    /* Fill up up to 32 bytes */
    quint8 unused[30];
} ZenoStartStopClockInt;

typedef struct {
    ZenoHeader h;
    quint8 channel;
    quint8 pid;
    
    /* Fill up up to 32 bytes */
    quint8 unused[28];
} ZenoLinClearMessage;

typedef struct {
    ZenoHeader h;
    quint8 channel;
    
    /* Fill up up to 32 bytes */
    quint8 unused[29];
} ZenoLinClearAllMessages;

typedef struct {
    ZenoHeader h;
    quint16 start_code;
    
    /* Fill up up to 32 bytes */
    quint8 unused[28];
} ZenoFlashStart;

typedef struct {
    ZenoHeader h;
    quint16 finish_code;
    quint16 from_table;
    quint16 page_count;

    /* Fill up up to 32 bytes */
    quint8 unused[24];
} ZenoFlashFinish;

typedef struct {
    ZenoHeader h;
    quint16 page;
    quint16 offset;
    
    /* Fill up up to 32 bytes */
    quint8 unused[26];
} ZenoFlashReadRowToBuffer;

typedef struct {
    ZenoHeader h;
    quint16 offset;    

    /* Fill up up to 32 bytes */
    quint8 unused[28];
} ZenoFlashReadFromBuffer;

typedef struct {
    ZenoHeader h;
    quint8 data[30];
} ZenoFlashReadFromBufferResponse;

typedef struct {
    ZenoHeader h;
    quint16 offset;    
    quint8 data[28];
} ZenoFlashWriteToBuffer;

typedef struct {
    ZenoHeader h;
    quint16 page;
    quint16 offset;
    
    /* Fill up up to 32 bytes */
    quint8 unused[26];
} ZenoFlashEraseRow;

typedef struct {
    ZenoHeader h;
    quint16 page;
    quint16 offset;
    
    /* Fill up up to 32 bytes */
    quint8 unused[26];
} ZenoFlashWriteRowFromBuffer;

#pragma pack(pop)

#endif /* ZENOCAN_H */
