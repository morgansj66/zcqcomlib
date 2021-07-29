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

#define ZURAGON_VENDOR_ID               0x84d8
#define ZENO_CANQUATRO_USB_ID           0x15
#define ZENO_CANQUATRO_MPCIE_USB_ID     0x16

static const struct usb_device_id zeno_usb_table[] = {
	/* Leaf USB product IDs */
	{ USB_DEVICE(ZURAGON_VENDOR_ID, ZENO_CANQUATRO_USB_ID) },
	{ USB_DEVICE(ZURAGON_VENDOR_ID, ZENO_CANQUATRO_MPCIE_USB_ID) },
    { }
};

static void zeno_usb_read_bulk_callback(struct urb *urb)
{
	struct zeno_usb *dev = urb->context;
	int err;
	unsigned int i;
    
	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -EPIPE:
	case -EPROTO:
	case -ESHUTDOWN:
		return;
	default:
		dev_info(&dev->intf->dev, "Rx URB aborted (%d)\n", urb->status);
		goto resubmit_urb;
	}
    printk(KERN_DEBUG "Zeno RCB: %d\n", urb->actual_length);
	zeno_cq_read_bulk_callback(dev, urb->transfer_buffer,
                               urb->actual_length);
    
 resubmit_urb:
	usb_fill_bulk_urb(urb, dev->udev,
                      usb_rcvbulkpipe(dev->udev,
                                      dev->bulk_in->bEndpointAddress),
                      urb->transfer_buffer, ZENO_USB_RX_BUFFER_SIZE,
                      zeno_usb_read_bulk_callback, dev);
    
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err == -ENODEV) {
		for (i = 0; i < dev->nr_of_can_channels; i++) {
			if (!dev->nets[i])
				continue;
            
			netif_device_detach(dev->nets[i]->netdev);
		}
	} else if (err) {
		dev_err(&dev->intf->dev,
                "Failed resubmitting read bulk urb: %d\n", err);
	}
}

static int zeno_usb_setup_rx_urbs(struct zeno_usb *dev)
{
    int i, err = 0;

	if (dev->rx_init_done)
		return 0;

	for (i = 0; i < ZENO_USB_MAX_RX_URBS; i++) {
        struct urb *urb = NULL;
		u8 *buf = NULL;
		dma_addr_t buf_dma;
        
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			err = -ENOMEM;
			break;
		}
        
		buf = usb_alloc_coherent(dev->udev, ZENO_USB_RX_BUFFER_SIZE,
                                 GFP_KERNEL, &buf_dma);
		if (!buf) {
			dev_warn(&dev->intf->dev,
                     "No memory left for USB buffer\n");
			usb_free_urb(urb);
			err = -ENOMEM;
			break;
		}

        usb_fill_bulk_urb(urb, dev->udev,
                          usb_rcvbulkpipe(dev->udev,
                                          dev->bulk_in->bEndpointAddress),
                          buf, ZENO_USB_RX_BUFFER_SIZE,
                          zeno_usb_read_bulk_callback, dev);
        
		urb->transfer_dma = buf_dma;
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(urb, &dev->rx_anchor);

		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			usb_unanchor_urb(urb);
			usb_free_coherent(dev->udev,
                              ZENO_USB_RX_BUFFER_SIZE, buf,
                              buf_dma);
			usb_free_urb(urb);
			break;
		}
        
		dev->rxbuf[i] = buf;
		dev->rxbuf_dma[i] = buf_dma;
        
		usb_free_urb(urb);        
    }
    
    if (i == 0) {
        dev_warn(&dev->intf->dev, "Cannot setup read URBs, error %d\n",
                 err);
		return err;
	} else if (i < ZENO_USB_MAX_RX_URBS) {
		dev_warn(&dev->intf->dev, "RX performances may be slow\n");
	}
    
    dev->rx_init_done = true;

	return 0;
}

static int zeno_usb_open(struct net_device *netdev)
{
	struct zeno_usb_net_priv *priv = netdev_priv(netdev);
	struct zeno_usb *dev = priv->dev;
	int err;

	err = open_candev(netdev);
	if (err)
		return err;

	err = zeno_usb_setup_rx_urbs(dev);
	if (err)
		goto error;

    err = zeno_cq_open(priv);
	if (err)
		goto error;

	err = zeno_cq_set_opt_mode(priv);
	if (err)
		goto error;

	err = zeno_cq_start_bus_on(priv);
	if (err) {
		netdev_warn(netdev, "Cannot start device, error %d\n", err);
		goto error;
	}

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

error:
	close_candev(netdev);
	return err;
}

static void zeno_usb_reset_tx_fifo(struct zeno_usb_net_priv *priv)
{
    int i;
    
    priv->tx_read_i = priv->tx_write_i = 0;

	for (i = 0; i < priv->tx_fifo_size; i++)
		priv->tx_fifo[i].transaction_id = 0;
}

/* This method might sleep. Do not call it in the atomic context
 * of URB completions.
 */
static void zeno_usb_unlink_tx_urbs(struct zeno_usb_net_priv *priv)
{
	usb_kill_anchored_urbs(&priv->tx_anchor);
	zeno_usb_reset_tx_fifo(priv);    
}

static int zeno_usb_close(struct net_device *netdev)
{
	struct zeno_usb_net_priv *priv = netdev_priv(netdev);
	int err;

	netif_stop_queue(netdev);

    if (!priv->dev->device_disconnected) {
        err = zeno_cq_stop_bus_off(priv);
        if (err)
            netdev_warn(netdev, "Cannot stop device, error %d\n", err);

        err = zeno_cq_close(priv);
        if (err)
            netdev_warn(netdev, "Cannot close device, error %d\n", err);
    }

	/* reset tx contexts */
	zeno_usb_unlink_tx_urbs(priv);

	priv->can.state = CAN_STATE_STOPPED;
	close_candev(priv->netdev);

	return 0;
}

static netdev_tx_t zeno_usb_start_xmit(struct sk_buff *skb,
                                       struct net_device *netdev)
{
    int ret;
    ret = NETDEV_TX_OK;
    return ret;
}

static const struct net_device_ops zeno_usb_netdev_ops = {
	.ndo_open = zeno_usb_open,
	.ndo_stop = zeno_usb_close,
	.ndo_start_xmit = zeno_usb_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static void zeno_usb_unlink_all_urbs(struct zeno_usb *dev)
{
	int i;

	usb_kill_anchored_urbs(&dev->rx_anchor);

	for (i = 0; i < ZENO_USB_MAX_RX_URBS; i++)
		usb_free_coherent(dev->udev, ZENO_USB_RX_BUFFER_SIZE,
				  dev->rxbuf[i], dev->rxbuf_dma[i]);

    dev->rx_init_done = false;
    
	for (i = 0; i < dev->nr_of_can_channels; i++) {
		struct zeno_usb_net_priv *priv = dev->nets[i];

		if (priv)
			zeno_usb_unlink_tx_urbs(priv);
	}
}

static void zeno_usb_remove_interfaces(struct zeno_usb *dev)
{
    int i;
    
    for (i = 0; i < dev->nr_of_can_channels; i++) {
		if (!dev->nets[i])
			continue;

		unregister_candev(dev->nets[i]->netdev);
	}

	zeno_usb_unlink_all_urbs(dev);

	for (i = 0; i < dev->nr_of_can_channels; i++) {
		if (!dev->nets[i])
			continue;

		free_candev(dev->nets[i]->netdev);
	}

}

static int zeno_usb_init_one(struct zeno_usb *dev,
                             const struct usb_device_id *id, int channel)
{
	struct net_device *netdev;
	struct zeno_usb_net_priv *priv;
	int err;

    int tx_fifo_size;

    if ( channel >= 0 && channel <= 3)
        tx_fifo_size = dev->max_pending_tx_ch14;
    else
        tx_fifo_size = dev->max_pending_tx_ch56;

    netdev = alloc_candev(struct_size(priv, tx_fifo, tx_fifo_size + 1),
                          tx_fifo_size);
	if (!netdev) {
		dev_err(&dev->intf->dev, "Cannot alloc candev\n");
		return -ENOMEM;
	}

	priv = netdev_priv(netdev);

    priv->tx_fifo_size = tx_fifo_size + 1;

	init_usb_anchor(&priv->tx_anchor);
	priv->can.ctrlmode_supported = 0;

	priv->dev = dev;
	priv->netdev = netdev;
	priv->channel = channel;

	spin_lock_init(&priv->tx_fifo_lock);
    priv->tx_read_i = priv->tx_write_i = 0;

	priv->can.state = CAN_STATE_STOPPED;
    priv->can.ctrlmode_supported |= CAN_CTRLMODE_LISTENONLY;
    
    if (channel >= 0 && channel <= 3) {
        /* Channel 1-4 - CAN FD */
        priv->can.ctrlmode_supported |= CAN_CTRLMODE_FD | CAN_CTRLMODE_FD_NON_ISO;
        priv->can.clock.freq = dev->clock_freq_ch14;
        priv->can.bittiming_const = dev->bittiming_const_ch14;
        priv->can.data_bittiming_const = dev->data_bittiming_const_ch14;
		priv->can.do_set_data_bittiming = zeno_cq_set_data_bittiming;
    } else {
        /* CHannel 5-6 ECAN1 and ECAN2 */
        priv->can.clock.freq = dev->clock_freq_ch56;
        priv->can.bittiming_const = dev->bittiming_const_ch56;
    }

	priv->can.do_set_bittiming = zeno_cq_set_bittiming;
	priv->can.do_set_mode = zeno_cq_set_mode;
    priv->can.do_get_berr_counter = zeno_cq_get_berr_counter;

	netdev->flags |= IFF_ECHO;

	netdev->netdev_ops = &zeno_usb_netdev_ops;

	SET_NETDEV_DEV(netdev, &dev->intf->dev);
	netdev->dev_id = channel;

	dev->nets[channel] = priv;

	err = register_candev(netdev);
	if (err) {
		dev_err(&dev->intf->dev, "Failed to register CAN device\n");
		free_candev(netdev);
		dev->nets[channel] = NULL;
		return err;
	}

	netdev_dbg(netdev, "device registered\n");

	return 0;
}

static int zeno_usb_setup_clock_int_inf(struct zeno_usb *dev)
{
    int err;
    printk(KERN_DEBUG "Zeno-USB setting CLOCK int\n");

    err = zeno_cq_setup_clock_int_endpoints(dev);
    if (err) {
		dev_err(&dev->clock_int_intf->dev, "Cannot get usb INT endpoint");
		return err;
    }

    err = zeno_cq_start_clock_int(dev);
    if (err) {
		dev_err(&dev->clock_int_intf->dev, "Failed to start CLOCK INT");
		return err;
    }
    
    return 0;
}

static int zeno_usb_probe(struct usb_interface *intf,
                          const struct usb_device_id *id)
{
    struct zeno_usb *dev;
	int err;
    int ifnum;
	int i;
    struct usb_device* udev;
    
    if (intf->num_altsetting < 1)
        return -ENODEV;

    ifnum = intf->altsetting[0].desc.bInterfaceNumber;

    printk(KERN_DEBUG "Zeno-USB interface nr: %d - %p\n", ifnum, intf);    
    udev = interface_to_usbdev(intf);
    dev = dev_get_drvdata(&udev->dev);

    if (dev == NULL) {
        printk(KERN_DEBUG "Zeno-USB allocating device data\n");
        dev = devm_kzalloc(&intf->dev, sizeof(*dev), GFP_KERNEL);
        if (!dev)
            return -ENOMEM;
        dev_set_drvdata(&udev->dev, dev);
    }

	usb_set_intfdata(intf, dev);

    if (ifnum == 1) {
        dev->clock_int_intf = intf;
        return zeno_usb_setup_clock_int_inf(dev);
    }

    dev->udev = udev;
	dev->intf = intf;
    
	err = zeno_cq_setup_endpoints(dev);
	if (err) {
		dev_err(&intf->dev, "Cannot get usb endpoint(s)");
		return err;
	}

	init_usb_anchor(&dev->rx_anchor);

    err = zeno_cq_device_reset(dev);
    if (err) {
        dev_err(&intf->dev,
			"Failed to reset device, error %d\n", err);
        return err;
    }


    err = zeno_cq_get_device_info(dev);
    if (err) {
		dev_err(&intf->dev,
			"Failed to get device info, error %d\n", err);
		return err;
	}

    if (dev->nr_of_can_channels > ZENO_USB_MAX_NET_DEVICES) {
		dev_err(&intf->dev,
			"Device is not support, nr of CAN channels %d\n", dev->nr_of_can_channels);
        dev->nr_of_can_channels = 0;
		return err;
    }
    
    for (i = 0; i < dev->nr_of_can_channels; i++) {
		err = zeno_usb_init_one(dev, id, i);
		if (err) {
			zeno_usb_remove_interfaces(dev);
			return err;
		}
	}

    return 0;    
}

static void zeno_usb_disconnect(struct usb_interface *intf)
{
	struct zeno_usb *dev = usb_get_intfdata(intf);
    struct usb_device* udev;
    printk("Zeno-USB disconnect %p\n", intf);

    if (!dev || dev->clock_int_intf == intf) return;

    dev->device_disconnected = true;
    udev = interface_to_usbdev(intf);
	zeno_usb_remove_interfaces(dev);

	usb_set_intfdata(intf, NULL);
    usb_set_intfdata(dev->clock_int_intf, NULL);
    dev_set_drvdata(&udev->dev, NULL);
}

static struct usb_driver zeno_usb_driver = {
	.name = "zeno_usb",
	.probe = zeno_usb_probe,
	.disconnect = zeno_usb_disconnect,
	.id_table = zeno_usb_table,
};

module_usb_driver(zeno_usb_driver);

MODULE_AUTHOR("Zuragon LTd <support@zuragon.com>");
MODULE_DESCRIPTION("USB driver for Zeno CANquatro devices");
MODULE_LICENSE("GPL v2");
