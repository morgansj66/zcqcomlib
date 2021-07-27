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

static int zeno_usb_open(struct net_device *netdev)
{
    return 0;
}

static int zeno_usb_close(struct net_device *netdev)
{
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

static void zeno_usb_unlink_all_urbs(struct zeno_usb *dev)
{
	int i;

	usb_kill_anchored_urbs(&dev->rx_anchor);

	for (i = 0; i < ZENO_USB_MAX_RX_URBS; i++)
		usb_free_coherent(dev->udev, ZENO_USB_RX_BUFFER_SIZE,
				  dev->rxbuf[i], dev->rxbuf_dma[i]);

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
        priv->can.ctrlmode_supported |= CAN_CTRLMODE_FD;
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

static int zeno_usb_probe(struct usb_interface *intf,
                          const struct usb_device_id *id)
{
    struct zeno_usb *dev;
	int err;
	int i;

	dev = devm_kzalloc(&intf->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->intf = intf;

	err = zeno_cq_setup_endpoints(dev);
	if (err) {
		dev_err(&intf->dev, "Cannot get usb endpoint(s)");
		return err;
	}

    dev->udev = interface_to_usbdev(intf);

	init_usb_anchor(&dev->rx_anchor);

	usb_set_intfdata(intf, dev);

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

	usb_set_intfdata(intf, NULL);

	if (!dev)
		return;

	zeno_usb_remove_interfaces(dev);
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
