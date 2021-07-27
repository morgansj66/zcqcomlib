// SPDX-License-Identifier: GPL-2.0
/* SocketCAN driver for the Zuragon CANquatro 
 * CAN and CAN FD USB devices
 *
 * Copyright(C) 2021 Zuragon LTd - www.zuragon.com
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/if.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/usb.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>

#include "zeno_usb.h"

#define ZENO_CQ_TX_FIFO_SIZE_CH14 31
#define ZENO_CQ_TX_FIFO_SIZE_CH56 7

static const struct can_bittiming_const zeno_cq_bittiming_const_ch14 = {
	.name = "zeno_cq_ch1-4",
	.tseg1_min = 2,
	.tseg1_max = 256,
	.tseg2_min = 1,
	.tseg2_max = 256,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static const struct can_bittiming_const zeno_cq_data_bittiming_const_ch14 = {
	.name = "zeno_cq_fd_ch1-4",
	.tseg1_min = 1,
	.tseg1_max = 32,
	.tseg2_min = 1,
	.tseg2_max = 16,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static const struct can_bittiming_const zeno_cq_bittiming_const_ch56 = {
	.name = "zeno_cq_ch5-6",
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

static int zeno_usb_send_cmd(const struct zeno_usb *dev, ZenoCmd* cmd)
{
	int actual_len; /* Not used */
    int err;

    ZenoCmd* __cmd;

    __cmd = kmalloc(sizeof(ZenoCmd), GFP_KERNEL);
	if (!__cmd)
		return -ENOMEM;
    *__cmd = *cmd;
    
	err = usb_bulk_msg(dev->udev,
                       usb_sndbulkpipe(dev->udev,
                                       dev->bulk_out->bEndpointAddress),
                       __cmd, sizeof(ZenoCmd), &actual_len, ZENO_USB_TIMEOUT);

    kfree(__cmd);

    return err;
}

static void zeno_cq_handle_command(struct zeno_usb *dev, ZenoCmd* zeno_cmd)
{
    switch(zeno_cmd->h.cmd_id) {
    case ZENO_CMD_CAN_TX_ACK: {
        // ZenoTxCANRequestAck* zeno_tx_ack = (ZenoTxCANRequestAck*)zeno_cmd;
        break;
    }
    case ZENO_CMD_CAN_RX: {
        break;
    }
    case ZENO_CMD_CANFD_P1_RX: {
        // ZenoCANFDMessageP1* zeno_canfd_msg_p1 = (ZenoCANFDMessageP1*)zeno_cmd;
        break;
    }
    case ZENO_CMD_CANFD_P2_RX: {
        // ZenoCANFDMessageP2* zeno_canfd_msg_p2 = (ZenoCANFDMessageP2*)zeno_cmd;
        break;
    }
    case ZENO_CMD_CANFD_P3_RX: {
        // ZenoCANFDMessageP3* zeno_canfd_msg_p3 = (ZenoCANFDMessageP3*)zeno_cmd;
        break;
    }
    case ZENO_CMD_LIN_RX_MESSAGE: {
        // ZenoLINMessage* zeno_lin_msg = (ZenoLINMessage*)zeno_cmd;
        break;
    }
    case ZENO_CMD_LIN_TX_ACK: {
        // ZenoTxLINRequestAck* zeno_lin_tx_ack = (ZenoTxLINRequestAck*)zeno_cmd;
        break;
    }
    case ZENO_DEBUG: {
        ZenoDebug* zeno_debug = (ZenoDebug*)zeno_cmd;
        char debug_msg[31];
        int i, debug_msg_len = 30;
        
        for (i = 0; i < 30; ++i) {
            if (zeno_debug->debug_msg[i] == 0) {
                debug_msg_len = i;
                break;
            }
            debug_msg[i] = zeno_debug->debug_msg[i];
        }
        debug_msg[debug_msg_len] = '\0';
        
        printk(KERN_DEBUG "ZenoDBG: %s\n", debug_msg);
        break;
    }
    default:
        break;
    }
}

static void zeno_cq_handle_incomming_data(struct zeno_usb *dev, void* in_buffer,
                                         int bytes_transferred)
{
    ZenoCmd* cmd;
    int offset = 0;
    bytes_transferred -= ZENO_CMD_SIZE;

    while(offset <= bytes_transferred) {
        cmd = (ZenoCmd*)(in_buffer + offset);

        if (cmd->h.cmd_id == ZENO_CMD_RESPONSE) {
            ZenoResponse* reply = zenoReply(*cmd);

            if (dev->zeno_response != NULL &&
                dev->zeno_response_cmd_id == reply->response_cmd_id) {
                *dev->zeno_response = *reply;
                dev->reply_received = true;
                break;
            }
        }
        else zeno_cq_handle_command(dev,cmd);
            
        offset += ZENO_CMD_SIZE;
    }


    while(offset <= bytes_transferred) {
        cmd = (ZenoCmd*)(in_buffer + offset);
        zeno_cq_handle_command(dev,cmd);

        offset += ZENO_CMD_SIZE;
    }
}

static int zeno_cq_wait_for_reply_before_init(struct zeno_usb *dev)
{
    int err;
	void *buf;
	int actual_len;
	unsigned long to = jiffies + msecs_to_jiffies(ZENO_USB_TIMEOUT);

	buf = kzalloc(ZENO_USB_RX_BUFFER_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

    do {
        err = usb_bulk_msg(dev->udev,
                           usb_rcvbulkpipe(dev->udev,
                                           dev->bulk_in->bEndpointAddress),
                           buf, ZENO_USB_RX_BUFFER_SIZE, &actual_len, ZENO_USB_TIMEOUT);
        if (err)
            goto end;;

        zeno_cq_handle_incomming_data(dev,buf,actual_len);
        
    } while (!dev->reply_received && time_before(jiffies, to));

    if (!dev->reply_received)
        err = -EINVAL;

 end:
    kfree(buf);

    return err;
}

static int zeno_cq_wait_for_reply(struct zeno_usb *dev)
{
    return 0;
}

static int zeno_cq_send_cmd_and_wait_reply(struct zeno_usb *dev,
                                            ZenoCmd* cmd, ZenoResponse* reply)
{
    int err;

    dev->zeno_response_cmd_id = cmd->h.cmd_id;
    dev->zeno_response = reply;
    dev->reply_received = false;
    
    err = zeno_usb_send_cmd(dev, cmd);
    if (err)
        return err;

    if (!dev->rx_init_done)
        err = zeno_cq_wait_for_reply_before_init(dev);
    else
        err = zeno_cq_wait_for_reply(dev);

    return err;
}
    
int zeno_cq_setup_endpoints(struct zeno_usb *dev)
{
	const struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;

	iface_desc = dev->intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in && usb_endpoint_is_bulk_in(endpoint))
			dev->bulk_in = endpoint;

		if (!dev->bulk_out && usb_endpoint_is_bulk_out(endpoint))
			dev->bulk_out = endpoint;

		/* use first bulk endpoint for in and out */
		if (dev->bulk_in && dev->bulk_out)
			return 0;
	}

	return -ENODEV;    
}

int zeno_cq_device_reset(struct zeno_usb *dev)
{
    int err;
    ZenoCmd cmd;
    ZenoResponse reply;
    
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZENO_CMD_RESET;

    err = zeno_cq_send_cmd_and_wait_reply(dev,&cmd,&reply);
    if (err)
        return err;
    
    return 0;
}

int zeno_cq_get_device_info(struct zeno_usb *dev)
{
    int err;
    ZenoCmd cmd;
    ZenoInfoResponse info;
    
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZENO_CMD_RESET;

    err = zeno_cq_send_cmd_and_wait_reply(dev,&cmd,zenoReply(info));
    if (err)
        return err;


    printk(KERN_DEBUG "Zeno Capabilities: %08x\n", info.capabilities);
    printk(KERN_DEBUG "       fw_version: %x.%x.%x\n",
           (info.fw_version >> 24) & 0xff,
           (info.fw_version >> 16) & 0xff,
            info.fw_version        & 0xff);
    printk(KERN_DEBUG "        serial-nr: %x\n", info.serial_number);;
    printk(KERN_DEBUG "          clock @: %dMhz\n", info.clock_resolution / 1000);
    printk(KERN_DEBUG "CAN channel count: %d\n",info.can_channel_count);
    printk(KERN_DEBUG "LIN channel count: %d\n",info.lin_channel_count);

    dev->system_clock_resolution = info.clock_resolution;
    dev->serial_number = info.serial_number;
    dev->firmware_version = info.fw_version;

        
    dev->nr_of_can_channels = info.can_channel_count;
    dev->nr_of_lin_channels = info.lin_channel_count;
    dev->max_pending_tx_ch14 = ZENO_CQ_TX_FIFO_SIZE_CH14;
        
    dev->clock_freq_ch14 = info.clock_resolution * 1000;
    dev->bittiming_const_ch14 = &zeno_cq_bittiming_const_ch14;
    dev->data_bittiming_const_ch14 = &zeno_cq_data_bittiming_const_ch14;

    dev->clock_freq_ch56 = (dev->clock_freq_ch14 * 7) / 4;
    dev->bittiming_const_ch56 = &zeno_cq_bittiming_const_ch56;
    dev->max_pending_tx_ch56 = ZENO_CQ_TX_FIFO_SIZE_CH56;
    
    return 0;
}

int zeno_cq_set_mode(struct net_device *netdev, enum can_mode mode)
{
    return 0;
}

int zeno_cq_set_bittiming(struct net_device *netdev)
{
    return 0;
}

int zeno_cq_set_data_bittiming(struct net_device *netdev)
{
    return 0;    
}

int zeno_cq_get_berr_counter(const struct net_device *netdev, struct can_berr_counter *bec)
{
    return 0;
}
