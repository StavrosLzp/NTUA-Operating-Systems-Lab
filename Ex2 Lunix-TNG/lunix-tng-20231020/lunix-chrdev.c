/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
 * Stavros Lazopoulos <el20843@mail.ntua.gr>
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
/*
 * Declare a prototype so we can define the "unused" attribute and keep
 * the compiler happy. This function is not yet used, because this helpcode
 * is a stub.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	debug("enetering\n");
	/* ? */
	sensor = state->sensor;
	if (state->buf_timestamp < sensor->msr_data[state->type]->last_update)
		return 1;
	/* The following return is bogus, just for the stub to compile */
	return 0; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	/**/
	uint32_t raw_data;
	uint32_t timestamp;
	long cooked_data;
	unsigned long flags;

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	WARN_ON(!(sensor = state->sensor));
	spin_lock_irqsave(&sensor->lock, flags);
	raw_data = sensor->msr_data[state->type]->values[0];
	timestamp = sensor->msr_data[state->type]->last_update;
	spin_unlock_irqrestore(&sensor->lock, flags);
	/* ? */
	
  /* Why use spinlocks? See LDD3, p. 119 */

  /*
	 * Any new data available?
   */
  if(!(state->buf_timestamp < timestamp))
	 	return -EAGAIN;
	/* ? */

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	if (state->type == BATT)
		cooked_data = lookup_voltage[raw_data];
	else if (state->type == TEMP)
		cooked_data = lookup_temperature[raw_data];
	else if (state->type == LIGHT)
		cooked_data = lookup_light[raw_data];
	else 
		goto out;



	state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ,
			"%ld.%03ld ", cooked_data/1000, cooked_data%1000);
	state->buf_timestamp = timestamp;

	/* ? */

out:
	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	struct lunix_chrdev_state_struct *chrdev_state;
	unsigned int sensor_no, sensor_type;
	/* ? */

	int ret;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	sensor_no = iminor(inode) / 8;
	sensor_type = iminor(inode) % 8;

	/* Allocate a new Lunix character device private state structure */
	if(!(chrdev_state = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL))){
		debug("failed to allocate memory for device private state\n");
		ret = -ENOMEM;
		goto out;
	}
	chrdev_state->type = sensor_type;
	chrdev_state->sensor = &lunix_sensors[sensor_no];
	chrdev_state->buf_lim = 0;
	chrdev_state->buf_timestamp = 0;
	sema_init(&chrdev_state->lock, 1);
	chrdev_state->nonblocking = 0; //(filp->f_flags & O_NONBLOCK) ? 1 : 0;

	filp->private_data =chrdev_state;

	debug("chrdev_state initialized successfully");	

	/* ? */
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	kfree(filp->private_data);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret, cached_bytes;
	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock? */
	if (down_interruptible(&state->lock))
		return -ERESTARTSYS;
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			/* The process needs to sleep */
			up(&state->lock);
			if (state->nonblocking)
				return -EAGAIN;
			/*wait_event_interruptible(queue, condition) wait queue until */
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
				return -ERESTARTSYS; /* signal: tell the fs layer to handle it */

			if (down_interruptible(&state->lock))
				return -ERESTARTSYS;
			/* See LDD3, page 153 for a hint */
		}
	}

	/* End of file */
	if (!state->buf_lim){
		ret = 0 ;
		goto out;
	}
	/* ? */
	
	/* Determine the number of cached bytes to copy to userspace */
	cached_bytes = state->buf_lim - *f_pos;
	if (cached_bytes < cnt)
		cnt = cached_bytes;

	if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)){
		ret = -EFAULT;
		goto out;
	}
	ret = cnt;
	*f_pos += cnt;
	/* ? */

	/* Auto-rewind on EOF mode? */
	if (*f_pos >= state->buf_lim){
		*f_pos = 0;
	}
	/* ? */
out:
	/* Unlock? */
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
  .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* int register_chrdev_region(dev_t first, unsigned int count, char *name);*/
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix");

	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	/* int cdev_add(struct cdev *dev, dev_t num, unsigned int count); */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);

	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
