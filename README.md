# Raspberry Pi Kernel Module for DHT22 Sensor

Package dht22 implements a Raspberry Pi kernel module for the DHT22 temperature
and humidity sensor. The implementation is intended to be an example of how
a kernel module could be written. It has no ambitions to be production-grade.
In particular, this is not an officially supported Google product.

## Pre-requisites

To build the kernel module, you need to first install the following
packages:

```
sudo apt install build-essential raspberrypi-kernel-headers
```

To learn more about Linux device drivers, see [Linux Device Drivers, Third
Edition](https://lwn.net/Kernel/LDD3/). To learn more about Linux kernel
modules in general, see [The Linux Kernel Module Programming
Guide](https://www.tldp.org/LDP/lkmpg/2.6/). To learn more about the DHT22
sensor, see the [data sheet](http://www.electrodragon.com/w/AM2302).

## Building and testing

To build, simply run `make`. You can switch debug logging on and off by
editing the `Makefile`. You can see these logs by running `dmesg`.

To load the module into the kernel, run:

```
sudo insmod dht22.ko
```

Once the module has loaded, you can examine the data read by the sensor
by running `cat /dev/dht22`. That returns three values: the unix timestamp,
the humidity, the temperature. To gather these values, say, every ten minutes,
add the following like to crontab:

```
*/10 * * * * cat /dev/dht22 >> /home/pi/data.csv
```

To unload the module from the kernel, run:

```
sudo rmmod dht22.ko
```

## Implementation

The module reads data from the sensor when the user opens the `/dev/dht22`
device. To have the device accessible by regular users, one must add the udev
rules file `99-dht22.rules` to the directory `/etc/udev/rules.d/`.

Since the sensor, as per the datasheet, can only be read once every two seconds
the kernel module returns -EBUSY if the user attempts to read the sensor too
soon. If the sensor read fails for some reason—usually due to the checksum
validation not passing—the kernel module returns -EIO.

A succesul read from the `/dev/dht22` device returns three comma-separated
numbers: the timestamp of the most recent successful sensor read, the relative
humidity as a percentage, and the temperature in degrees Celcius.

The protocol for starting a sensor read is to first pull the GPIO pin low for
at least 1ms and then pull the pin high and wait for the sensor to respond with
an 80µs low pulse followed by an 80µs high pulse. After that initial response,
the sensor sends 40 pulses whose widths encode the actual sensor data. The
pulses are recorded by an interrupt handler; we simply wait for the read cycle
to finish and then process the data that the interrupt handler gathered.

The interrupt handler triggers on a falling edge (high to low) on the GPIO
pin. During a sensor read, there are in total 42 falling edges: two during the
setup phase and then one for each transmitted bit of information. The
transmission of each bit starts with the signal going low for 50 µs. Then
signal goes high for 26 µs when 0 is transmitted; signal goes high for 70 µs
when 1 is transmitted. We count the time between the falling edges and use
110µs as the boundary between reading a 0 and reading a 1. The threshold is
derived from measurements: the long cycles are usually around 125µs while the
short cycles vary between 70µs and 95µs.
