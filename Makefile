MODULE = dht22
# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

ifeq ($(DEBUG),y)
 ccflags-y := -O -g -DDHT22_DEBUG
endif

obj-m := ${MODULE}.o

module_upload=${MODULE}.ko
gpio_pin=4

compile:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean

info: compile
	modinfo ${module_upload}

test: compile
	sudo dmesg -C
	sudo insmod ${module_upload} gpiopin=${gpio_pin}
	sudo rmmod ${module_upload}
	dmesg
