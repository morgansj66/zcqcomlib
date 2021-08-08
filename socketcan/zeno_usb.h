// SPDX-License-Identifier: GPL-2.0
/* SocketCAN driver for the Zuragon CANquatro 
 * CAN and CAN FD USB devices
 *
 * Copyright(C) 2021 Zuragon LTd - www.zuragon.com
 */

#ifndef ZENO_USB_H
#define ZENO_USB_H

#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/usb.h>

#include <linux/can.h>
#include <linux/can/dev.h>

#include "zenocan.h"

#define ZURAGON_VENDOR_ID               0x84d8
#define ZENO_CANQUATRO_USB_ID           0x15
#define ZENO_CANQUATRO_MPCIE_USB_ID     0x16

#define ZENO_USB_TIMEOUT		  1000 /* msecs */
#define ZENO_USB_RX_BUFFER_SIZE   4096
#define ZENO_USB_INT_BUFFER_SIZE  16
#define ZENO_USB_MAX_NET_DEVICES  8
#define ZENO_USB_MAX_RX_URBS      6

struct zeno_usb
{
	struct usb_device *udev;
	struct usb_interface *intf;
    struct usb_interface *clock_int_intf;
	struct zeno_usb_net_priv *nets[ZENO_USB_MAX_NET_DEVICES];

    struct usb_endpoint_descriptor *bulk_in, *bulk_out;
    struct usb_endpoint_descriptor *clock_int_in;
	struct usb_anchor rx_anchor;

    u32 firmware_version;
    u32 capabilities;
    u32 serial_number;
    unsigned int nr_of_can_channels;
    unsigned int nr_of_lin_channels;
    unsigned int system_clock_resolution;
    bool device_disconnected;
    
    /* CAN CHannel 1 - 4 */
    unsigned int clock_freq_ch14;
	const struct can_bittiming_const * bittiming_const_ch14;
	const struct can_bittiming_const * data_bittiming_const_ch14;
    unsigned int max_pending_tx_ch14;
    
    /* CAN CHannel 5 - 6 */
    unsigned int clock_freq_ch56;
    const struct can_bittiming_const * bittiming_const_ch56;
    unsigned int max_pending_tx_ch56;
    
    int zeno_response_cmd_id;
    ZenoResponse* zeno_response;
    bool reply_received;
    struct completion cmd_reply_comp;
    
    /* Clock INT RX */
    void *rx_int_buf;
	dma_addr_t rx_int_buf_dma;
    ktime_t t2_clock_start_ref;
    ktime_t drift_time;
    ktime_t last_device_time;
    // double drift_factor;
    int init_calibrate_count;
    unsigned int last_usb_overflow_count;
    
    /* RX */
    bool rx_init_done;
    void *rxbuf[ZENO_USB_MAX_RX_URBS];
	dma_addr_t rxbuf_dma[ZENO_USB_MAX_RX_URBS];
};

struct zeno_tx_message
{
    struct zeno_usb_net_priv *net;
	int transaction_id;
	int dlc;
};

struct zeno_usb_net_priv
{
	struct can_priv can;
	struct can_berr_counter bec;

	struct zeno_usb *dev;
	struct net_device *netdev;

    bool is_open;
    bool bitrate_is_set;
    bool data_bitrate_is_set;
    
	int channel;

    ZenoCANFDMessageP1 canfd_p1;
    bool canfd_p1_received;
    ZenoCANFDMessageP2 canfd_p2;
    bool canfd_p2_received;
    
    s64 open_start_ref_timestamp_in_us;
    unsigned int base_clock_divisor;
    
    struct usb_anchor tx_anchor;

    int next_transaction_id;
    int tx_fifo_size;

    spinlock_t tx_fifo_lock;
    int tx_read_i;
    int tx_write_i;    
    struct zeno_tx_message tx_fifo[];
};

int zeno_cq_setup_endpoints(struct zeno_usb *dev);
int zeno_cq_setup_clock_int_endpoints(struct zeno_usb *dev);
int zeno_cq_device_reset(struct zeno_usb *dev);
int zeno_cq_get_device_info(struct zeno_usb *dev);
int zeno_cq_start_clock_int(struct zeno_usb *dev);
int zeno_cq_stop_clock_int(struct zeno_usb *dev);
int zeno_cq_open(struct zeno_usb_net_priv *net);
int zeno_cq_close(struct zeno_usb_net_priv *net);
int zeno_cq_start_bus_on(struct zeno_usb_net_priv *net);
int zeno_cq_stop_bus_off(struct zeno_usb_net_priv *net);
int zeno_cq_set_mode(struct net_device *netdev, enum can_mode mode);
int zeno_cq_set_opt_mode(const struct zeno_usb_net_priv *priv);
int zeno_cq_set_bittiming(struct net_device *netdev);
int zeno_cq_set_data_bittiming(struct net_device *netdev);
int zeno_cq_get_berr_counter(const struct net_device *netdev, struct can_berr_counter *bec);    
void zeno_cq_read_bulk_callback(struct zeno_usb *dev,void *in_buffer, int bytes_transferred);
void *zeno_cq_canfd_frame_to_cmd(struct zeno_usb_net_priv *priv,
                                 const struct sk_buff *skb, int *frame_len,
                                 int *cmd_len, int* transaction_id);
void *zeno_cq_can_frame_to_cmd(struct zeno_usb_net_priv *priv,
                               const struct sk_buff *skb, int *frame_len,
                               int *cmd_len, int* transaction_id);

/* Zeno proc fs */
int zeno_procfs_init(void);
void zeno_procfs_cleanup(void);
int zeno_procfs_register_dev(struct zeno_usb* dev);
int zeno_procfs_remove_dev(struct zeno_usb* dev);

#endif /*  ZENO_USB_H */
