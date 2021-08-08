// SPDX-License-Identifier: GPL-2.0
/* SocketCAN driver for the Zuragon CANquatro 
 * CAN and CAN FD USB devices
 *
 * Copyright(C) 2021 Zuragon LTd - www.zuragon.com
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>   
#include <linux/proc_fs.h>

#include "zeno_usb.h"

#define MAX_NR_OF_ZENO_DEVICES 32

static struct proc_dir_entry *zeno_procfs_entry = NULL;

struct zeno_device_procfs_entry {
    int nr_of_zeno_devs;
    struct zeno_usb* zeno_device_list[MAX_NR_OF_ZENO_DEVICES];
};

static struct zeno_device_procfs_entry zeno_device_procfs_entry_list;

static int zeno_procfs_show(struct seq_file *m, void *v)
{
    int i,j;
    struct zeno_device_procfs_entry* d = &zeno_device_procfs_entry_list;
    // seq_printf(m,"Zeno devices\n");

    for(i = 0; i < d->nr_of_zeno_devs; ++i) {
        struct zeno_usb *dev = d->zeno_device_list[i];
        switch(dev->udev->descriptor.idProduct) {
        case ZENO_CANQUATRO_USB_ID:
            seq_printf(m, "dev%02d: \"Zeno CANquatro\" ", i);
            break;
        case ZENO_CANQUATRO_MPCIE_USB_ID:
            seq_printf(m, "dev%02d: \"Zeno CANquatro mPCIe\" ", i);
            break;
        default:
            seq_printf(m, "dev%02d: \"Zeno unknown\" ",i);
        }

        seq_printf(m, "cap:%08x fw:%08x serial:%08x clock:%d devpath:\"%s\"\n",
                   dev->capabilities,
                   dev->firmware_version,
                   dev->serial_number,
                   dev->system_clock_resolution,
                   dev->udev->devpath);
                   
        for(j = 0; j < dev->nr_of_can_channels; ++j) {
            struct zeno_usb_net_priv* net = dev->nets[j];
            seq_printf(m, "\t%s ---\n", net->can.dev->name);
        }

        for(j = 0; j < dev->nr_of_lin_channels; ++j) {
            seq_printf(m, "\tlin%d ---\n",j);
        }
    }
    return 0;
}

static ssize_t zeno_procfs_write(struct file *file, const char __user *ubuf,size_t count, loff_t *ppos) 
{
	printk(KERN_DEBUG "zenodev - write handler\n");
	return -1;
}

static int zeno_procfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, zeno_procfs_show, NULL);
}

static struct proc_ops zeno_procfs_ops = 
{
    .proc_open = zeno_procfs_open,
	.proc_read = seq_read,
    .proc_lseek = seq_lseek,
	.proc_write = zeno_procfs_write,
    .proc_release = single_release,
};

int zeno_procfs_register_dev(struct zeno_usb* dev)
{
    if (zeno_device_procfs_entry_list.nr_of_zeno_devs >= MAX_NR_OF_ZENO_DEVICES)
        return -ENOMEM;

    zeno_device_procfs_entry_list.zeno_device_list[zeno_device_procfs_entry_list.nr_of_zeno_devs] = dev;

    zeno_device_procfs_entry_list.nr_of_zeno_devs++;
    
    return 0;
}

int zeno_procfs_remove_dev(struct zeno_usb* dev)
{
    int i;
    int remove_index = -1;
    struct zeno_device_procfs_entry* d = &zeno_device_procfs_entry_list;
    
    for(i = 0; i < d->nr_of_zeno_devs; ++i) {
        if (d->zeno_device_list[i] == dev) {
            remove_index = i;
            break;
        }
    }

    if (remove_index != -1) {
        for(i = remove_index+1; i < d->nr_of_zeno_devs; ++i) {
            d->zeno_device_list[i-1] = d->zeno_device_list[i];
        }
        d->nr_of_zeno_devs--;
    }
    
    return 0;
}

int zeno_procfs_init(void)
{
    memset(&zeno_device_procfs_entry_list,0,
           sizeof(zeno_device_procfs_entry_list));
    
    zeno_procfs_entry = proc_create("zenodev",0666,NULL,&zeno_procfs_ops);
    
    return 0;
}

void zeno_procfs_cleanup(void)
{
    if (zeno_procfs_entry != NULL) {
        proc_remove(zeno_procfs_entry);
        zeno_procfs_entry = NULL;
    }
}
