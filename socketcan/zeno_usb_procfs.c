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
static struct proc_dir_entry *zeno_dir = NULL;

struct zeno_device_procfs_entry {
    int nr_of_zeno_devs;
    struct {
        struct zeno_usb* dev;
        struct proc_dir_entry* proc_dev_entry;
    } zeno_dev_list[MAX_NR_OF_ZENO_DEVICES];    
};

static struct zeno_device_procfs_entry zeno_device_procfs_entry_list;

static int zeno_procfs_show(struct seq_file *m, void *v)
{
    int i,j;
    struct zeno_device_procfs_entry* d = &zeno_device_procfs_entry_list;
    // seq_printf(m,"Zeno devices\n");

    for(i = 0; i < d->nr_of_zeno_devs; ++i) {
        struct zeno_usb *dev = d->zeno_dev_list[i].dev;
        switch(dev->udev->descriptor.idProduct) {
        case ZENO_CANQUATRO_USB_ID:
            seq_printf(m, "dev%02d#Zeno CANquatro#", i);
            break;
        case ZENO_CANQUATRO_MPCIE_USB_ID:
            seq_printf(m, "dev%02d#Zeno CANquatro mPCIe#", i);
            break;
        default:
            seq_printf(m, "dev%02d#Zeno unknown#",i);
        }

        seq_printf(m, "cap:%08x#fw:%08x#serial:%08x#clock:%d#devpath:%s\n",
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

static int zeno_procfs_dev_show(struct seq_file *m, void *v)
{
    struct inode *inode = m->private;
    void* data = PDE_DATA( inode );
    int device_index =(int)(long)data;
    struct zeno_device_procfs_entry* d = &zeno_device_procfs_entry_list;
    struct zeno_usb *dev;
    u64 device_clock, drift_clock;
    s64 time_drift;
    int err;
    
    /* just for sanity, should never happen */
    if (device_index < 0 || device_index >= MAX_NR_OF_ZENO_DEVICES) {
        return -1;
    }
    
    dev = d->zeno_dev_list[device_index].dev;

    err = zeno_cq_get_device_clock(dev, &device_clock, &drift_clock, &time_drift);
    if (err)
        return -1;

    seq_write(m, &device_clock, sizeof(device_clock));
    seq_write(m, &drift_clock, sizeof(drift_clock));
    seq_write(m, &time_drift, sizeof(time_drift));
    seq_write(m, &dev->t2_clock_start_ref, sizeof(dev->t2_clock_start_ref));

    return 0;
}

static ssize_t zeno_procfs_dev_write(struct file *file, const char __user *ubuf,size_t count, loff_t *ppos) 
{
	printk(KERN_DEBUG "zenodev - write handler\n");
	return -1;
}

static int zeno_procfs_dev_open(struct inode *inode, struct file *file)
{
	return single_open(file, zeno_procfs_dev_show, inode);
}

static struct proc_ops zeno_procfs_dev_ops = 
{
    .proc_open = zeno_procfs_dev_open,
	.proc_read = seq_read,
    .proc_lseek = seq_lseek,
	.proc_write = zeno_procfs_dev_write,
    .proc_release = single_release,
};

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

static void zeno_procfs_register_dev_entry(int device_index)
{
    struct zeno_device_procfs_entry* d = &zeno_device_procfs_entry_list;
    struct proc_dir_entry* entry;
    char sbuf[16];
    long di;
    
    snprintf(sbuf,sizeof(sbuf),"dev%02d",device_index);

    di = device_index;
    entry = proc_create_data(sbuf,0666,zeno_dir,&zeno_procfs_dev_ops,(void*)di);

    d->zeno_dev_list[device_index].proc_dev_entry = entry;
}

int zeno_procfs_register_dev(struct zeno_usb* dev)
{
    struct zeno_device_procfs_entry* d = &zeno_device_procfs_entry_list;
    
    if (zeno_device_procfs_entry_list.nr_of_zeno_devs >= MAX_NR_OF_ZENO_DEVICES)
        return -ENOMEM;
    
    d->zeno_dev_list[d->nr_of_zeno_devs].dev = dev;
    zeno_procfs_register_dev_entry(d->nr_of_zeno_devs);
            
    zeno_device_procfs_entry_list.nr_of_zeno_devs++;
    
    return 0;
}

int zeno_procfs_remove_dev(struct zeno_usb* dev)
{
    int i;
    int remove_index = -1;
    struct zeno_device_procfs_entry* d = &zeno_device_procfs_entry_list;
    
    for(i = 0; i < d->nr_of_zeno_devs; ++i) {
        if (d->zeno_dev_list[i].dev == dev) {
            remove_index = i;
            break;
        }
    }

    if (remove_index != -1) {
        struct proc_dir_entry* e = d->zeno_dev_list[remove_index].proc_dev_entry;
        if (e != NULL)
            proc_remove(e);
        
        for(i = remove_index+1; i < d->nr_of_zeno_devs; ++i) {
            int new_i = i-1;
            d->zeno_dev_list[new_i] = d->zeno_dev_list[i];
            
            e = d->zeno_dev_list[new_i].proc_dev_entry;
            if (e != NULL)
                proc_remove(e);

            zeno_procfs_register_dev_entry(new_i);
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
    zeno_dir = proc_mkdir("zeno",NULL);
    
    return 0;
}

void zeno_procfs_cleanup(void)
{
    if (zeno_procfs_entry != NULL) {
        proc_remove(zeno_procfs_entry);
        zeno_procfs_entry = NULL;
    }

    if (zeno_dir != NULL) {
        proc_remove(zeno_dir);
        zeno_dir = NULL;
    }
}
