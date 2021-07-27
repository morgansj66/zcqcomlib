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

#define ZENO_USB_TIMEOUT		  1000 /* msecs */
#define ZENO_USB_RX_BUFFER_SIZE   4096
#define ZENO_USB_MAX_NET_DEVICES  8
#define ZENO_USB_MAX_RX_URBS      6
struct zeno_usb
{
	struct usb_device *udev;
	struct usb_interface *intf;
	struct zeno_usb_net_priv *nets[ZENO_USB_MAX_NET_DEVICES];

    struct usb_endpoint_descriptor *bulk_in, *bulk_out;
	struct usb_anchor rx_anchor;
    
    u32 firmware_version;
    u32 capabilities;
    u32 serial_number;
    unsigned int nr_of_can_channels;
    unsigned int nr_of_lin_channels;
    unsigned int system_clock_resolution;

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

    /* RX */
    bool rx_init_done;
    void *rxbuf[ZENO_USB_MAX_RX_URBS];
	dma_addr_t rxbuf_dma[ZENO_USB_MAX_RX_URBS];
};

struct zeno_tx_message
{
    struct zeno_usb_net_priv *priv;
	int transaction_id;
	int dlc;
};

struct zeno_usb_net_priv
{
	struct can_priv can;
	struct can_berr_counter bec;

	struct zeno_usb *dev;
	struct net_device *netdev;
	int channel;

    struct usb_anchor tx_anchor;

    int tx_fifo_size;

    spinlock_t tx_fifo_lock;
    int tx_read_i;
    int tx_write_i;    
    struct zeno_tx_message tx_fifo[];
};

int zeno_cq_setup_endpoints(struct zeno_usb *dev);
int zeno_cq_device_reset(struct zeno_usb *dev);
int zeno_cq_get_device_info(struct zeno_usb *dev);
int zeno_cq_set_mode(struct net_device *netdev, enum can_mode mode);
int zeno_cq_set_bittiming(struct net_device *netdev);
int zeno_cq_set_data_bittiming(struct net_device *netdev);
int zeno_cq_get_berr_counter(const struct net_device *netdev, struct can_berr_counter *bec);
    

#endif /*  ZENO_USB_H */
