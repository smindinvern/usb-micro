#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <libusb.h>

int main(int argc, char** argv)
{
	uint16_t vid = 0, pid = 0;

	if (libusb_init(NULL) < 0) {
		return -1;
	}

	libusb_device_handle* handle;
	libusb_device* dev;
	uint8_t bus, port_path[8];
	struct libusb_config_descriptor* conf_desc;
	const struct libusb_endpoint_descriptor* endpoint;
	struct libusb_device_descriptor dev_desc;

	printf("opening device %04x:%04x\n", vid, pid);
	handle = libusb_open_device_with_vid_pid(NULL, vid, pid);
	if (!handle) {
		printf("failed\n");
		goto error;
	}

	dev = libusb_get_device(handle);
	bus = libusb_get_bus_number(dev);

	int j;
	int i = libusb_get_port_numbers(dev, port_path, sizeof(port_path));
	if (i > 0) {
		printf("\nDevice properties:\n");
		printf("\tbus number: %d\n", bus);
		printf("\tport path: %d", port_path[0]);
		for (j = 1; j < i; j++) {
			printf("->%d", port_path[j]);
		}
		printf(" (from root hub)\n");
	}
	i = libusb_get_device_speed(dev);
	if (i < 0 || i > 4) {
		i = 0;
	}

	const char* speed_names[] = { "Unknown", "1.5 Mbit/s (USB LowSpeed)", "12 Mbit/s (USB FullSpeed)",
				      "480 Mbit/s (USB HighSpeed)", "5000 Mbit/s (USB SuperSpeed)" };
	printf("\tspeed: %s\n", speed_names[i]);

	printf("Reading device descriptor:\n");
	if (libusb_get_device_descriptor(dev, &dev_desc) < 0) {
		libusb_close(handle);
		goto error;
	}
	printf("\tlength: %d\n", dev_desc.bLength);
	printf("\tdevice class: %d\n", dev_desc.bDeviceClass);
	printf("\tVID:PID: %04X:%04X\n", dev_desc.idVendor, dev_desc.idProduct);
	printf("\n");

	libusb_set_auto_detach_kernel_driver(handle, 1);

	printf("Claiming interface 0\n");
	if (libusb_claim_interface(handle, 0) != LIBUSB_SUCCESS) {
		printf("failed\n");
		libusb_close(handle);
		goto error;
	}

	unsigned char request[] = "Testing loopback functionality with test USB device."
		"Herp derp derpy derp derp, herp.  This is test incantation 2.  HERP.  Oh ya, and derp.";
	unsigned char* buffer = malloc(4 + sizeof(request));
	if (!buffer) {
		libusb_release_interface(handle, 0);
		libusb_close(handle);
		printf("OOM\n");
		goto error;
	}
	uint16_t buf_size = sizeof(request);
	memcpy(buffer, &buf_size, 2);
	memcpy(buffer + 2, request, buf_size);
	buf_size += 2;
	int size;
	if (libusb_bulk_transfer(handle, 0x02, buffer, buf_size, &size, 0) < 0) {
		libusb_release_interface(handle, 0);		
		libusb_close(handle);
		printf("failed\n");
		goto error;
	}
	printf("Wrote %d bytes to the device\n", size);
	char rx_buffer[sizeof(request)];
	if (libusb_bulk_transfer(handle, 0x81, rx_buffer, sizeof(rx_buffer), &size, 0) < 0) {
		libusb_release_interface(handle, 0);
		libusb_close(handle);
		printf("failed\n");
		goto error;
	}
	printf("Read %d bytes from the device\n", size);
	printf("\t\"%s\"\n", rx_buffer);

	printf("Releasing interface 0\n");
	libusb_release_interface(handle, 0);
	
	printf("Closing device\n");
	libusb_close(handle);

	libusb_exit(NULL);
	return 0;
	
error:
	libusb_exit(NULL);
	return -1;
}
