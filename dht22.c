/*
 * Copyright 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implicit warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define DHT22_DEVICE_NAME "dht22"
#define DHT22_NUM_DEVICES 1
#define DHT22_MODULE_NAME "dht22"

#define DHT22_MAX_TIMESTAMPS 43
#define DHT22_PULSE_BOUNDARY 105

/*
 * gpiopin module paramter configures which GPIO pin the module uses.
 */
static int gpiopin = 4;
module_param(gpiopin, int, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(gpiopin, "GPIO pin that the DHT22 is connected to");

/*
 * read_delay_sec module paramter sets duration between sensor reads.
 */
static int read_delay_sec = 60;
module_param(read_delay_sec, int, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(read_delay_sec, "Delay between each sensor read");

/*
 * initial_delay_sec module paramter sets duration to the first sensor read.
 */
static int initial_delay_sec = 3;
module_param(initial_delay_sec, int, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(initial_delay_sec, "Time before first sensor read");

/*
 * struct dht22_state - All relevant sensor state.
 * @irq: Interrupt used to detect falling edge on sensor GPIO pin.
 * @num_edges: Number of detected edges during a sensor read.
 * @timestamps: Timestamps of detected edges.
 * @bytes: Decoded transmitted data from a sensor read.
 * @read_timestamp: Timestamps of latest sensor read.
 * @read_timespec: Timespec of latest sensor read.
 * @temperature: Most recently read temperature (times ten).
 * @humidity: Most recently read humidity percentage (times ten).
 */
struct dht22_state {
	int irq;

	int num_edges;
	ktime_t timestamps[DHT22_MAX_TIMESTAMPS];
	u8 bytes[5];

	ktime_t read_timestamp;
	struct timespec64 read_timespec;
	int temperature;
	int humidity;
};

/*
 * sensor_state is an instance of struct dht22_state.
 * It may only be accessed when holding sensor_mutex.
 */
static struct dht22_state sensor_state;
static DEFINE_MUTEX(sensor_mutex);  /* Protects sensor_state. */

/*
 * We set up an interrupt to trigger a falling edge (high to low) on the
 * GPIO pin. During a sensor read, there are in total 43 falling edges:
 * three during the setup phase and then one for each transmitted bit of
 * information. The transmission of each bit starts with the signal going
 * low for 50 µs. Then signal goes high for 26 µs when 0 is transmitted;
 * signal goes high for 70 µs when 1 is transmitted. We count the time
 * between the falling edges and use 105µs as the boundary between reading
 * a 0 and reading a 1. The threshold is derived from measurements: the
 * long cycles are usually around 125µs while the short cycles vary between
 * 70µs and 95µs.
 */

/*
 * s_handle_edge() - process interrupt due to falling edge on GPIO pin.
 * @irq: Then IRQ number. Unused.
 * @dev_id: The device identifier. Unused.
 *
 * Records the time stamp of a falling edge (high to low) on the DHT22
 * sensor pin and fills in the corresponding bit in the bytes buffer. The
 * code assumes that the bytes buffer was zeroed out before starting the
 * sensor read.
 *
 * Return: IRQ_HANDLED
 */
static irqreturn_t s_handle_edge(int irq, void *dev_id)
{
	ktime_t now, last;
	int width;
	unsigned long flags;
	local_irq_save(flags);
	if (sensor_state.num_edges <= 0) goto irq_handled;
	if (sensor_state.num_edges >= DHT22_MAX_TIMESTAMPS) goto irq_handled;
	now = ktime_get();
	last = sensor_state.timestamps[sensor_state.num_edges - 1];
	width = ktime_to_us(now - last);
	/* Start storing timestamps after the long start pulse happened. */
	if (sensor_state.num_edges == 1 && width < 1000) goto irq_handled;
	sensor_state.timestamps[sensor_state.num_edges++] = now;
	/*
	 * The first falling edge that is the end of a tramsitted bit occurs
	 * at index 3 in the array of time stamps. Since we zeroed out the
	 * bytes array before starting the read, we only have to update bytes
	 * when we read a 1 (encoded as a long pulse) from the sensor.
	 */
	if (sensor_state.num_edges >= 4 && width > DHT22_PULSE_BOUNDARY) {
		int sequence_index = sensor_state.num_edges - 4;
		int byte_index = sequence_index / 8;
		int bit_index = 7 - (sequence_index & 7);
		sensor_state.bytes[byte_index] |= 1 << bit_index;
	}
 irq_handled:
	local_irq_restore(flags);
	return IRQ_HANDLED;
}

/*
 * try_read_sensor() - try reading data from the DHT22 sensor.
 *
 * The DHT22 sensor can be read at most once every two seconds, so if the
 * sensor has recently been read we simply keep the old reading and return.
 *
 * Otherwise, we tell the sensor that we want to execute a new sensor read.
 * The protocol for starting a sensor read is to first pull the GPIO pin
 * low for at least 1ms (we pull it low for 1.5ms) and then pull the pin
 * high and wait for the sensor to respond with an 80µs low pulse followed
 * by an 80µs high pulse. After that initial response, the sensor sends 40
 * pulses whose widths encode the actual sensor data. The pulses are
 * recorded by our interrupt handler; we simply wait for 10ms so that the
 * read cycle finishes and then process the data in sensor_state.
 *
 * Once the sensor data has been read, we check that the right number of
 * bits have been read and that the checksum is correct. If those checks
 * pass, we update read_timestamp, humidity and temperature fields in
 * sensor_state with the newly read data.
 *
 * Context: May only be called when holding sensor_mutex.
 * Return: 0 on success; -EIO on error.
 */
static int try_read_sensor(void)
{
	ktime_t new_timestamp = ktime_get();
	s64 timestamp_diff =
		ktime_to_ms(new_timestamp - sensor_state.read_timestamp);
	u8 sum;
	if (timestamp_diff < 2000) {
		printk(KERN_ERR DHT22_MODULE_NAME ": sensor read too soon\n");
		return -EBUSY;
	}
	sensor_state.timestamps[0] = new_timestamp;
	sensor_state.num_edges = 1;
	memset(sensor_state.bytes, 0, sizeof sensor_state.bytes);

	if (gpio_direction_output(gpiopin, 0)) {
		printk(KERN_ERR DHT22_MODULE_NAME
		       ": gpio_direction_output failed\n");
		return -EIO;
	}
	udelay(1500);
	gpio_set_value(gpiopin, 1);
	if (gpio_direction_input(gpiopin)) {
		printk(KERN_ERR DHT22_MODULE_NAME
		       ": gpio_direction_input failed\n");
		return -EIO;
	}
	mdelay(20);  /* Read cycle takes less than 6ms. */

	if (sensor_state.num_edges != DHT22_MAX_TIMESTAMPS) {
		printk(KERN_ERR DHT22_MODULE_NAME
		       ": only detected %d bits\n",
		       sensor_state.num_edges);
		return -EIO;
	}
	sum = sensor_state.bytes[0] + sensor_state.bytes[1] +
	  sensor_state.bytes[2] + sensor_state.bytes[3];
	if (sum != sensor_state.bytes[4]) {
		printk(KERN_ERR DHT22_MODULE_NAME
		       ": sensor checksum mismatch\n");
		return -EIO;
	}

	ktime_get_real_ts64(&sensor_state.read_timespec);
	sensor_state.read_timestamp =
		sensor_state.timestamps[DHT22_MAX_TIMESTAMPS - 1];
	sensor_state.humidity =	(sensor_state.bytes[0] * 256 +
				 sensor_state.bytes[1]);
	sensor_state.temperature = ((sensor_state.bytes[2] & 0x7F) * 256 +
				    sensor_state.bytes[3]);
	if (sensor_state.bytes[2] & 0x80) {
		sensor_state.temperature = -sensor_state.temperature;
	}
	return 0;
}

#ifdef DHT22_DEBUG

/*
 * When compiled with -DDHT22_DEBUG, the module logs debug
 * information to the kernel log.
 *
 * print_debug_info() prints sensor_state to kernel log.
 *
 * Context: May only be called when holding sensor_mutex.
 */
static void print_debug_info(void)
{
	int i;
	ktime_t last, first, now;
	u8 sum;
	printk(KERN_DEBUG DHT22_MODULE_NAME ": using gpio pin %d\n", gpiopin);
	printk(KERN_DEBUG DHT22_MODULE_NAME ": detected %d edges\n",
	       sensor_state.num_edges);
	first = last = sensor_state.timestamps[0];
	for (i = 1; i < sensor_state.num_edges; i++) {
		now = sensor_state.timestamps[i];
		if (i < 3) {
			printk(KERN_DEBUG DHT22_MODULE_NAME
			       ": pulse[%2d]  == %5lldµs\n",
			       i, ktime_to_us(now - last));
		} else {
			printk(KERN_DEBUG DHT22_MODULE_NAME
			       ": pulse[%2d]  == %5lldµs  ==>  %1d\n",
			       i, ktime_to_us(now - last),
			       ktime_to_us(now - last) > DHT22_PULSE_BOUNDARY
			       ? 1 : 0);
		}
		last = now;
	}
	if (sensor_state.num_edges != DHT22_MAX_TIMESTAMPS) {
		return;
	}
	sum = sensor_state.bytes[0] + sensor_state.bytes[1] +
	  sensor_state.bytes[2] + sensor_state.bytes[3];
	printk(KERN_DEBUG DHT22_MODULE_NAME ": duration   == %5lldµs\n",
	       ktime_to_us(last - first));
	printk(KERN_DEBUG DHT22_MODULE_NAME ": read bytes ==  %*phC\n",
	       5, sensor_state.bytes);
	if (sum != sensor_state.bytes[4]) {
		printk(KERN_DEBUG DHT22_MODULE_NAME
		       ": checksum NOT ok: %x != %x\n",
		       sum, sensor_state.bytes[4]);
		return;
	}
	printk(KERN_DEBUG DHT22_MODULE_NAME ": checksum ok\n");
	printk(KERN_DEBUG DHT22_MODULE_NAME ": humidity ==%3d.%d%%\n",
	       sensor_state.humidity / 10, sensor_state.humidity % 10);
	printk(KERN_DEBUG DHT22_MODULE_NAME ": temperature ==%3d.%d°C\n",
	       sensor_state.temperature / 10, sensor_state.temperature % 10);
	printk(KERN_DEBUG DHT22_MODULE_NAME ": ktime_get() == %lldns\n",
	       sensor_state.read_timestamp);
	printk(KERN_DEBUG DHT22_MODULE_NAME ": timeofday == %llds\n",
	       sensor_state.read_timespec.tv_sec);
}
#endif



/*
 * We use a read-only character device to return sensor data in
 * human-readable form to users. The device returns one line of text: a
 * comma separated list of read timestamp (in seconds since the epoch),
 * humidity (as a percentage) and temperature (in degrees Celsius).
 */

/*
 * struct dht22_output - used to store human-readable sensor output.
 * @buf: The buffer.
 * @ptr: Pointer into @buf used when copying data to user space.
 */
struct dht22_output {
	char buf[30];
	char* ptr;
};

/*
 * device_open() - called when a process opens our device.
 *
 * Fail if the sensor has not been read at all or if the last 10 attempts
 * to read the sensor failed.
 *
 * Otherwise allocate a buffer for the text output and put a text
 * representation of the most recent sensor reading into the buffer.
 * A pointer to the output struct is saved in filp->private_data.
 */
static int device_open(struct inode *inode, struct file *filp)
{
	struct dht22_output* out;
	int error = 0;
	mutex_lock(&sensor_mutex);
	if (sensor_state.read_timespec.tv_sec == 0) {
		printk(KERN_ERR DHT22_MODULE_NAME ": device not ready\n");
		error = -EIO;
		goto unlock_and_return;
	}
	if (ktime_to_ms(ktime_get() - sensor_state.read_timestamp)
	    > 10000 * read_delay_sec) {
		printk(KERN_ERR DHT22_MODULE_NAME
		       ": too many recent read errors\n");
		error = -EIO;
		goto unlock_and_return;
	}
	out = kmalloc(sizeof(struct dht22_output),	GFP_KERNEL);
	if (!out) {
		printk(KERN_ALERT DHT22_MODULE_NAME
		       ": no memory for chrdev buffer\n");
		error = -ENOMEM;
		goto unlock_and_return;
	}
	if(sizeof out->buf <= snprintf(out->buf, sizeof out->buf,
				       "%lld,%d.%d,%d.%d\n",
				       sensor_state.read_timespec.tv_sec,
				       sensor_state.humidity / 10,
				       sensor_state.humidity % 10,
				       sensor_state.temperature / 10,
				       sensor_state.temperature % 10)) {
		kfree(out);
		printk(KERN_ALERT DHT22_MODULE_NAME
		       ": chrdev buffer overflow\n");
		error = -ENOMEM;
		goto unlock_and_return;
	}
	out->ptr = out->buf;
	filp->private_data = out;
 unlock_and_return:
	mutex_unlock(&sensor_mutex);
	return error;
}

/*
 * device_release() - called when a process closes our device.
 *
 * Free buffer memory allocated in device_open().
 *
 * Return: 0.
 */
static int device_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

/*
 * device_read() - called when a process reads from our device.
 * @filp: Contains pointer to sensor state in private_data field.
 * @user_buffer: User space buffer for returned text.
 * @len: Size of @user_buffer.
 * @offset: Unused.
 *
 * If there is no more data to return, just return a zero. Otherwise, copy
 * as much data as possible from our internal buffer to the user space buffer.
 *
 * Return: Number of bytes copied into @user_buffer.
 *         Return-value 0 indicates that all data has been copied.
 */
static ssize_t device_read(struct file *filp, char *user_buffer, size_t len,
			   loff_t *offset)
{
	struct dht22_output* out = filp->private_data;
	int bytes_read = 0;
	if (!filp->private_data) {
		printk(KERN_ALERT DHT22_MODULE_NAME
		       ": chrdev buffer not allocated\n");
		return -ENOMEM;
	}
	while (len && *out->ptr) {
		/* We use put_user() to copy from kernel data to user data. */
		put_user(*(out->ptr++), user_buffer++);
		len--;
		bytes_read++;
	}
	return bytes_read;
}

static const struct file_operations dht22_cdev_fops = {
	.owner = THIS_MODULE,
	.read = device_read,
	.open = device_open,
	.release = device_release
};

static dev_t dht22_dev;
static dev_t dht22_dev_0;
static int dht22_dev_major;
static struct cdev dht22_cdev;
static struct class *dht22_class;
static struct device *dht22_device;



/*
 * We use a hrtimer to trigger regular reads from the sensor.
 * The timer is configured when the kernel module is loaded.
 */

static struct hrtimer timer;

static enum hrtimer_restart hrtimer_handler(struct hrtimer *timer)
{
	ktime_t interval = ktime_set(read_delay_sec, 0);
	mutex_lock(&sensor_mutex);
	try_read_sensor();
#ifdef DHT22_DEBUG
	print_debug_info();
#endif
	mutex_unlock(&sensor_mutex);
	hrtimer_forward(timer, ktime_get(), interval);
	return HRTIMER_RESTART;
}



/**
 * m_init() - initialize the DHT22 module as it is loaded.
 *
 * Validate configuration parameters, connect to GPIO pin, initialize GPIO
 * pin to high, set up interrupt to trigger activity on the GPIO pin, create
 * character device used to pass human-readable data back to users, set up
 * timer that triggers sensor reads.
 */
int __init m_init(void)
{
	int error = 0;
	printk(KERN_INFO DHT22_MODULE_NAME ": hello, world\n");

	if (read_delay_sec < 3) {
		printk(KERN_ALERT DHT22_MODULE_NAME
		       ": read delay less than 3s\n");
		return -EINVAL;
	}
	if (!gpio_is_valid(gpiopin)){
		printk(KERN_ALERT DHT22_MODULE_NAME ": invalid GPIO pin\n");
		return -EINVAL;
	}
	if (gpio_request(gpiopin, DHT22_MODULE_NAME) < 0) {
		printk(KERN_ALERT DHT22_MODULE_NAME ": gpio_request failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO DHT22_MODULE_NAME ": GPIO pin %d\n", gpiopin);
	if (gpio_direction_input(gpiopin)) {
		printk(KERN_ERR DHT22_MODULE_NAME
		       ": initial gpio_direction_input failed\n");
		return -ENODEV;
	}
	sensor_state.irq = gpio_to_irq(gpiopin);
	if (sensor_state.irq < 0) {
		printk(KERN_ALERT DHT22_MODULE_NAME ": gpio_to_irq failed\n");
		error = -EBUSY;
		goto gpio_to_irq_failed;
	}
	if (request_irq(sensor_state.irq, s_handle_edge, IRQF_TRIGGER_FALLING,
			DHT22_MODULE_NAME, NULL) < 0) {
		printk(KERN_ALERT DHT22_MODULE_NAME ": request_irq failed\n");
		error = -EBUSY;
		goto request_irq_failed;
	}
	printk(KERN_INFO DHT22_MODULE_NAME
	       ": interrupt %d\n", sensor_state.irq);

	if ((error = alloc_chrdev_region(&dht22_dev, 0, DHT22_NUM_DEVICES,
					 DHT22_DEVICE_NAME)) < 0) {
		printk(KERN_ALERT DHT22_MODULE_NAME
		       ": alloc_chrdev_region failed\n");
		goto alloc_chrdev_failed;
	}
	dht22_dev_major = MAJOR(dht22_dev);
	dht22_dev_0 = MKDEV(dht22_dev_major, 0);
	cdev_init(&dht22_cdev, &dht22_cdev_fops);
	if ((error = cdev_add(&dht22_cdev, dht22_dev_0, 1)) < 0) {
		printk(KERN_ALERT DHT22_MODULE_NAME ": cdev_add failed\n");
		goto cdev_add_failed;
	}
	printk(KERN_INFO DHT22_MODULE_NAME ": device %d created\n",
	       dht22_dev_major);

	dht22_class = class_create(THIS_MODULE, DHT22_MODULE_NAME);
	if (IS_ERR(dht22_class)) {
		printk(KERN_ALERT DHT22_MODULE_NAME ": class_create failed\n");
		error = PTR_ERR(dht22_class);
		goto class_create_failed;
	}
	dht22_device = device_create(dht22_class, NULL, dht22_dev_0,
				     NULL, DHT22_DEVICE_NAME);
	if (IS_ERR(dht22_device)) {
		printk(KERN_ALERT DHT22_MODULE_NAME
		       ": device_create failed\n");
		error = PTR_ERR(dht22_device);
		goto device_create_failed;
	}

	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = hrtimer_handler;
	hrtimer_start(&timer, ktime_set(initial_delay_sec, 0),
		      HRTIMER_MODE_REL);
	return 0;

device_create_failed:
	class_destroy(dht22_class);
class_create_failed:
	cdev_del(&dht22_cdev);
cdev_add_failed:
	unregister_chrdev_region(dht22_dev, DHT22_NUM_DEVICES);
alloc_chrdev_failed:
	free_irq(sensor_state.irq, NULL);
request_irq_failed:
gpio_to_irq_failed:
	gpio_free(gpiopin);
	return error;
}

/**
 * m_cleanup() - clean up before the DHT22 module is unloaded.
 */
void __exit m_cleanup(void)
{
	hrtimer_cancel(&timer);
	device_destroy(dht22_class, dht22_dev_0);
	class_destroy(dht22_class);
	cdev_del(&dht22_cdev);
	unregister_chrdev_region(dht22_dev, DHT22_NUM_DEVICES);
	free_irq(sensor_state.irq, NULL);
	gpio_free(gpiopin);
	printk(KERN_INFO DHT22_MODULE_NAME ": goodbye, world\n");
}

module_init(m_init);
module_exit(m_cleanup);

MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Lars Engebretsen <lars@engebretsen.ch>");
MODULE_DESCRIPTION("DHT22 Driver");
