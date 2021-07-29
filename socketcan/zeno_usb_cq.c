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
#define ZENO_CQ_DRIFT_MAX_ADJUST  (250 * 1000)
#define ZENO_CQ_DRIFT_MULTIPLIER  0x8000000

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
    printk(KERN_DEBUG "Zeno %d\n", zeno_cmd->h.cmd_id);
    
    switch(zeno_cmd->h.cmd_id) {
    case ZENO_CMD_CAN_TX_ACK: {
        printk(KERN_DEBUG "Zeno CAN Tx\n");
        break;
    }
    case ZENO_CMD_CAN_RX: {
        printk(KERN_DEBUG "Zeno CAN Rx\n");
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
static void zeno_cq_adjust_time_drift(struct zeno_usb* dev, ktime_t device_time,
                                      ktime_t new_drift_time, ktime_t max_adjust)
{
    ktime_t diff = dev->drift_time - new_drift_time;    
    ktime_t adjust;

    adjust = (diff < 0) ? -diff : diff;
    adjust = min(adjust, max_adjust);

    if (diff > 0)
        dev->drift_time -= adjust;
    else
        dev->drift_time += adjust;

    dev->last_device_time = device_time;
    // if ( device_time != 0 ) {
    //    kernel_fpu_begin();
    //    dev->drift_factor = (double)dev->drift_time / (double)device_time;
    //    kernel_fpu_end();
    // }
}

static void zeno_cq_handle_int_command(struct zeno_usb *dev, ZenoIntCmd* zeno_int_cmd)
{
    switch(zeno_int_cmd->h.cmd_id) {
    case ZENO_CLOCK_INFO_INT:  {
        ZenoIntClockInfoCmd* clock_info;
        ktime_t host_t0, zeno_clock, drift;
        
        if (dev->init_calibrate_count > 5) {
            /* NOTE: the first 2 events may be bogus */

            dev->init_calibrate_count--;
            if (dev->init_calibrate_count == 5)
                dev->t2_clock_start_ref = ktime_get();
            return;
        }
                
        host_t0 = ktime_get() - dev->t2_clock_start_ref;
        
        clock_info = (ZenoIntClockInfoCmd*)zeno_int_cmd;
        dev->last_usb_overflow_count += clock_info->usb_overflow_count;

        zeno_clock = (clock_info->clock_value_t1 / clock_info->clock_divisor) * 1000;
        drift = zeno_clock - host_t0;

        if (dev->init_calibrate_count > 0) {
            dev->init_calibrate_count--;
            dev->drift_time += drift;

            if ( dev->init_calibrate_count == 0 ) {
                dev->drift_time /= 5;
                printk(KERN_DEBUG "Zeno Avg drift time: %lld us\n", dev->drift_time / 1000);
                dev->t2_clock_start_ref -= dev->drift_time;
                dev->drift_time = 0;
            }
        }
        else {
            zeno_cq_adjust_time_drift(dev, zeno_clock, drift, ZENO_CQ_DRIFT_MAX_ADJUST);
        }
    }
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

void zeno_cq_read_bulk_callback(struct zeno_usb *dev,void *in_buffer, int bytes_transferred)
{
    ZenoCmd* cmd;
    int offset = 0;
    bytes_transferred -= ZENO_CMD_SIZE;

    printk(KERN_DEBUG "Zeno bulk CK: %d\n", bytes_transferred);
    while(offset <= bytes_transferred) {
        cmd = (ZenoCmd*)(in_buffer + offset);

        if (cmd->h.cmd_id == ZENO_CMD_RESPONSE) {
            ZenoResponse* reply = zenoReply(*cmd);

            if (dev->zeno_response != NULL &&
                dev->zeno_response_cmd_id == reply->response_cmd_id) {
                *dev->zeno_response = *reply;
                dev->reply_received = true;

                complete(&dev->cmd_reply_comp);
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
            goto end;

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
    if (!wait_for_completion_timeout(&dev->cmd_reply_comp,
                                     msecs_to_jiffies(ZENO_USB_TIMEOUT)))
		return -ETIMEDOUT;

    if (!dev->reply_received)
        return -EINVAL;
    
    return 0;
}

static int zeno_cq_send_cmd_and_wait_reply(struct zeno_usb *dev,
                                            ZenoCmd* cmd, ZenoResponse* reply)
{
    int err;

    dev->zeno_response_cmd_id = cmd->h.cmd_id;
    dev->zeno_response = reply;
    dev->reply_received = false;

    if (dev->rx_init_done)
        init_completion(&dev->cmd_reply_comp);
    
    err = zeno_usb_send_cmd(dev, cmd);
    if (err) {
        dev->zeno_response = NULL;
        return err;
    }

    if (!dev->rx_init_done)
        err = zeno_cq_wait_for_reply_before_init(dev);
    else
        err = zeno_cq_wait_for_reply(dev);

    dev->zeno_response = NULL;
    
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

int zeno_cq_setup_clock_int_endpoints(struct zeno_usb *dev)
{
	const struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;

	iface_desc = dev->clock_int_intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
        endpoint = &iface_desc->endpoint[i].desc;

        if (!dev->clock_int_in && usb_endpoint_is_int_in(endpoint))
            dev->clock_int_in = endpoint;

        if (dev->clock_int_in)
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
    cmd.h.cmd_id = ZENO_CMD_INFO;

    err = zeno_cq_send_cmd_and_wait_reply(dev,&cmd,zenoReply(info));
    if (err)
        return err;


    printk(KERN_DEBUG "Zeno Capabilities: %08x\n", info.capabilities);
    printk(KERN_DEBUG "       fw_version: %x.%x.%x\n",
           (info.fw_version >> 16) & 0xff,
           (info.fw_version >>  8) & 0xff,
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

static void zeno_cq_int_callback(struct urb *urb)
{
	struct zeno_usb *dev = urb->context;
    int err;
    
    switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		return;
	default:
		dev_info(&dev->intf->dev, "Rx INT URB aborted (%d)\n", urb->status);
		goto resubmit_urb;
	}

    zeno_cq_handle_int_command(dev, (ZenoIntCmd*)dev->rx_int_buf);
 resubmit_urb:
    usb_fill_int_urb(urb, dev->udev,
                     usb_rcvintpipe(dev->udev,
                                    dev->clock_int_in->bEndpointAddress),
                     urb->transfer_buffer, ZENO_USB_INT_BUFFER_SIZE,
                     zeno_cq_int_callback, dev, dev->clock_int_in->bInterval);
    
	err = usb_submit_urb(urb, GFP_ATOMIC);
    if (err) {
		dev_err(&dev->intf->dev,
                "Failed resubmitting INT urb: %d\n", err);
	}
}

int zeno_cq_start_clock_int(struct zeno_usb *dev)
{
    struct urb *urb = NULL;
    u8 *buf = NULL;
    dma_addr_t buf_dma;
    int err;
    ZenoCmd cmd;
    ZenoResponse reply;
    
    urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!urb)
        return -ENOMEM;

    buf = usb_alloc_coherent(dev->udev, ZENO_USB_INT_BUFFER_SIZE,
                             GFP_KERNEL, &buf_dma);
    if (!buf) {
        dev_warn(&dev->clock_int_intf->dev,
				 "No memory left for USB buffer\n");
        usb_free_urb(urb);
        return -ENOMEM;
    }

    usb_fill_int_urb(urb, dev->udev,
                     usb_rcvintpipe(dev->udev,
                                    dev->clock_int_in->bEndpointAddress),
                     buf, ZENO_USB_INT_BUFFER_SIZE,
                     zeno_cq_int_callback, dev,
                     dev->clock_int_in->bInterval);

    urb->transfer_dma = buf_dma;
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    usb_anchor_urb(urb, &dev->rx_anchor);

    err = usb_submit_urb(urb, GFP_KERNEL);
    if (err) {
        usb_unanchor_urb(urb);
        usb_free_coherent(dev->udev,
                          ZENO_USB_INT_BUFFER_SIZE, buf,
                          buf_dma);
        usb_free_urb(urb);
        return err;
    }

    dev->rx_int_buf = buf;
    dev->rx_int_buf_dma = buf_dma;

    usb_free_urb(urb);

    err = zeno_cq_stop_clock_int(dev);
    if (err)
        return err;
   
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZEMO_CMD_START_CLOCK_INT;

    printk(KERN_DEBUG "Sending Start-Clock INT cmd");
    dev->init_calibrate_count = 7;
    err = zeno_cq_send_cmd_and_wait_reply(dev,&cmd,&reply);

    if (err) {
        zeno_cq_stop_clock_int(dev);
        return err;
    }

    return 0;
}

int zeno_cq_stop_clock_int(struct zeno_usb *dev)
{
    int err;
    ZenoCmd cmd;
    ZenoResponse reply;
    
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.cmd_id = ZEMO_CMD_STOP_CLOCK_INT;

    err = zeno_cq_send_cmd_and_wait_reply(dev,&cmd,&reply);
    if (err)
        return err;

    return 0;
}

int zeno_cq_open(struct zeno_usb_net_priv *net)
{
    int err;
    ZenoOpen cmd;
    ZenoOpenResponse reply;

    if ((net->can.ctrlmode &
         (CAN_CTRLMODE_FD | CAN_CTRLMODE_FD_NON_ISO)) ==
	    CAN_CTRLMODE_FD_NON_ISO) {
		netdev_warn(net->netdev,
                    "CTRLMODE_FD shall be on if CTRLMODE_FD_NON_ISO is on\n");
		return -EINVAL;
	}

    memset(&cmd,0,sizeof(ZenoOpen));
    cmd.h.cmd_id = ZENO_CMD_OPEN;
    cmd.channel = (u8)net->channel;
    cmd.base_clock_divisor = (u8)(net->dev->system_clock_resolution / 1000);
    
    if (net->can.ctrlmode & CAN_CTRLMODE_FD)
        cmd.can_fd_mode = 1;
    
    if (net->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO)
        cmd.can_fd_non_iso = 1;
    
    err = zeno_cq_send_cmd_and_wait_reply(net->dev,zenoRequest(cmd),zenoReply(reply));
    if (err)
        return err;

    return 0;
}

int zeno_cq_close(struct zeno_usb_net_priv *net)
{
    int err;
    ZenoClose cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoClose));
    cmd.h.cmd_id = ZENO_CMD_CLOSE;
    cmd.channel = (u8)net->channel;

    err = zeno_cq_send_cmd_and_wait_reply(net->dev,zenoRequest(cmd),zenoReply(reply));
    if (err)
        return err;
    
    return 0;
}

int zeno_cq_start_bus_on(struct zeno_usb_net_priv *net)
{
    int err;
    ZenoBusOn cmd;
    ZenoResponse reply;

    printk("Zeno Bus on\n");
    
    memset(&cmd,0,sizeof(ZenoBusOn));
    cmd.h.cmd_id = ZENO_CMD_BUS_ON;
    cmd.channel = (u8)net->channel;

    err = zeno_cq_send_cmd_and_wait_reply(net->dev,zenoRequest(cmd),zenoReply(reply));
    if (err)
        return err;

    return 0;
}

int zeno_cq_stop_bus_off(struct zeno_usb_net_priv *net)
{
    int err;
    ZenoBusOn cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBusOn));
    cmd.h.cmd_id = ZENO_CMD_BUS_OFF;
    cmd.channel = (u8)net->channel;

    printk(KERN_DEBUG "Zeno Buss off\n");
    err = zeno_cq_send_cmd_and_wait_reply(net->dev,zenoRequest(cmd),zenoReply(reply));
    if (err)
        return err;

    printk(KERN_DEBUG "Zeno Buss off done\n");
    return 0;
}

int zeno_cq_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct zeno_usb_net_priv *net = netdev_priv(netdev);
	int err;

	switch (mode) {
	case CAN_MODE_START:
		err = zeno_cq_start_bus_on(net);
		if (err)
			return err;
		break;
	default:
		return -EOPNOTSUPP;
	}

    return 0;
}

int zeno_cq_set_opt_mode(const struct zeno_usb_net_priv *net)
{
    int err;
    ZenoOpMode cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoOpMode));
    cmd.h.cmd_id = ZENO_CMD_SET_OP_MODE;
    cmd.channel = (u8)net->channel;

    if (net->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		cmd.op_mode = ZENO_SILENT_MODE;
	else
        cmd.op_mode = ZENO_NORMAL_CAN20_ONLY_MODE;
		// cmd.op_mode = is_canfd_mode ? ZENO_NORMAL_CANFD_MODE : ZENO_NORMAL_CAN20_ONLY_MODE;

    printk("Zeno opt mode: %d\n", cmd.op_mode);
    
    err = zeno_cq_send_cmd_and_wait_reply(net->dev,zenoRequest(cmd),zenoReply(reply));
    if (err)
        return err;

    return 0;
}

int zeno_cq_set_bittiming(struct net_device *netdev)
{
    int err;
    struct zeno_usb_net_priv *net = netdev_priv(netdev);
	struct can_bittiming *bt = &net->can.bittiming;

    ZenoBitTiming cmd;
    ZenoResponse reply;

    memset(&cmd,0,sizeof(ZenoBitTiming));
    cmd.h.cmd_id = ZENO_CMD_SET_BIT_TIMING;
    cmd.channel = (u8)net->channel;

    printk(KERN_DEBUG "Zeno: set_bittimg\n");
    
    if (net->channel >=0 && net->channel < 4) {
        /* Channel 1-4 */
        cmd.brp   = bt->brp-1;
        cmd.tseg1 = (bt->prop_seg + bt->phase_seg1) -1;
        cmd.tseg2 = bt->phase_seg2-1;
        cmd.sjw   = bt->sjw-1;
    }
    else {
        /* CHannel 5-6 */
        if ( bt->bitrate == 1000000 ) {
            /* NOTE: 10kpbs is not supported on the Zeno CANquatro 42x CAN5 and CAN6
             * due to the internal clock of 70Mhz, there no possible way
             * to configure the ECAN to 1mbit */
            netdev_err(netdev, "1Mbit bitrate is not supported on channel: %d", net->channel);
            return -EINVAL;
        }

        if ( bt->bitrate == 10000 ) {
            /* NOTE: 10kpbs is not supported on the Zeno CANquatro 42x CAN5 and CAN6
             * due to the internal clock of 70Mhz, there no possible way
             * to configure the ECAN to 1mbit */
            netdev_err(netdev, "10kpbs bitrate is not supported on channel: %d", net->channel);
            return -EINVAL;
        }

        cmd.cicfg1 = (u16)((bt->brp-1) & 0x3f) | (((bt->sjw-1) & 0x3) << 6);
        cmd.cicfg2 = (u16)((bt->prop_seg-1) & 0x7) | (((bt->phase_seg1-1) & 0x7) <<3) | (1 << 7) |
            (((bt->phase_seg1-1) & 0x7) << 8);
    }
    
    err = zeno_cq_send_cmd_and_wait_reply(net->dev,zenoRequest(cmd),zenoReply(reply));
    if (err)
        return err;
    
    return 0;
}

int zeno_cq_set_data_bittiming(struct net_device *netdev)
{
    int err;
    ZenoBitTiming cmd;
    ZenoResponse reply;
    struct zeno_usb_net_priv *net = netdev_priv(netdev);
    struct can_bittiming *bt = &net->can.data_bittiming;
 
    memset(&cmd,0,sizeof(ZenoBitTiming));
    cmd.h.cmd_id = ZENO_CMD_SET_DATA_BIT_TIMING;
    cmd.channel = (u8)net->channel;

    cmd.brp   = bt->brp;
    cmd.tseg1 = bt->prop_seg + bt->phase_seg1;
    cmd.tseg2 = bt->phase_seg2;
    cmd.sjw   = bt->sjw-1;

    switch (bt->bitrate) {
    case 1000000:
        cmd.tdc_offset = 31;
        cmd.tdc_value = 0;
        break;
        
    case 2000000:
        cmd.tdc_offset = 15;
        cmd.tdc_value = 0;
        break;
        
    case 3000000:
        cmd.tdc_offset = 9;
        cmd.tdc_value = 0;
        break;
        
    case 4000000:
        cmd.tdc_offset = 7;
        cmd.tdc_value = 0;
        break;
        
    case 5000000:
        cmd.tdc_offset = 5;
        cmd.tdc_value = 0;
        break;
        
    case 6700000:
        cmd.tdc_offset = 4;
        cmd.tdc_value = 0;
        break;
        
    case 8000000:
        cmd.tdc_offset = 3;
        cmd.tdc_value = 1;
        break;
        
    case 10000000:
        cmd.tdc_offset = 2;
        cmd.tdc_value = 0;
        break;

    case 500000:
        cmd.tdc_offset = 31;
        cmd.tdc_value = 0;
        cmd.tdc_ssp_mode_off = 1;
        break;
        
    case 833000:
        cmd.tdc_offset = 18;
        cmd.tdc_value = 0;
        cmd.tdc_ssp_mode_off = 1;
        break;
        
    case 1500000:
        cmd.tdc_offset = 19;
        cmd.tdc_value = 0;
        break;
        
    default:
        netdev_err(netdev, "Unsupported data-bitrate: %d", bt->bitrate);
        return -EINVAL;
    }
    
    err = zeno_cq_send_cmd_and_wait_reply(net->dev,zenoRequest(cmd),zenoReply(reply));
    if (err)
        return err;

    return 0;
}

int zeno_cq_get_berr_counter(const struct net_device *netdev, struct can_berr_counter *bec)
{
    struct zeno_usb_net_priv *net = netdev_priv(netdev);
    *bec = net->bec;
    
    return 0;
}
