## USBUART
 A cross-platform library for reading/wring data via USB-UART adapters

### Introduction

`USBUART` is a LIBUSB based library that implements a relay from USB-UART 
converter’s endpoints to a pair of I/O resources, either appointed by given 
file descriptors, or created inside the library with pipe[(2)](http://linux.die.net/man/2/pipe).

User application may then use standard I/O operations for reading and writing data.
USBUART Library provides API for three languages - C++, C and Java.

See C/C++ API description in [usbuart.h](usbuart_8h.html) and Android API in 

### Usage with C++

	// Instantiate a context
	context ctx;
	
	// Attach USB via a pipe channel
	channel chnl;
	ctx.pipe(device_id{0x067b,0x0609},chnl,_115200_8N1n);
	
	//Run loop in one thread
	while(ctx.loop(10) >= -error_t::no_channel);
	
	//Read/write data in other thread(s)
	char buff[256];
	read(chnl.fd_read, buff, sizeof(buff));
	
	//or use non-blocking I/O in the loop body

### Usage on Android

	// Instantiate a context
	ctx = new UsbUartContext();
	
	// Start a thread running event loop 
	new Thread(ctx).start();
	
	// Obtain permission to the USB device
	UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
	usbManager.requestPermission(device, PendingIntent.getBroadcast(...));
	
	// Open device
	UsbDeviceConnection connection = usbManager.openDevice(device);
	
	// Create pipe channel
	Channel channel = ctx.pipe(connection, 0, EIA_TIA_232_Info._115200_8N1n());
	
	// Open streams
	InputStream input = channel.getInputStream();	
	OutputStream output = channel.getOutputStream();	
	
	// Perfrom I/O operations with the streams

### Termux Support for USB-Serial Piping (e.g., for Klipper)

This functionality allows a USB-serial device connected to an Android phone to be accessed by applications running within a Termux environment, such as Klipper. It works by creating a named pipe (FIFO) in Termux that acts as a bridge to the actual USB-serial device.

**Key Component:**

The core of this feature is the `termux_uart_bridge` utility, found in the `examples/` directory.

**Prerequisites for Building:**

*   A Termux environment on your Android device.
*   Required packages installed in Termux: `clang`, `libusb` (which provides `libusb-1.0-dev` equivalent functionality in Termux), `pkg-config`, and `make`.
    ```bash
    pkg install clang libusb make pkg-config
    ```

**Prerequisites for Running:**

*   The Termux:API application must be installed on your Android device (this provides the `termux-usb` utility).
*   A USB-to-serial adapter (e.g., CH340, CP210x, FTDI) connected to the Android device via a USB OTG adapter if necessary.

**Building `termux_uart_bridge`:**

1.  Follow the main "Building" instructions below to clone the `usbuart` repository and its `libusb` submodule (steps 1-5, ensuring `libusb` is configured and built for your host system if you are cross-compiling, or just built within Termux).
2.  Build the main `libusbuart.so` shared library:
    ```bash
    make all 
    ```
    (or simply `make`)
3.  Build the `termux_uart_bridge` utility:
    ```bash
    make termux_uart_bridge
    ```
    The binary `termux_uart_bridge` will be created in the root directory of the project.

**Running `termux_uart_bridge`:**

1.  Identify your USB device: Open Termux and run `termux-usb -l`. This will list connected USB devices. Note the system path for your USB-serial adapter (e.g., `/dev/bus/usb/001/002`).
2.  Execute the bridge utility from the root of the `usbuart` project directory:
    ```bash
    ./termux_uart_bridge /dev/bus/usb/001/002 /data/data/com.termux/files/usr/tmp/klipper_serial_pipe
    ```
    *   Replace `/dev/bus/usb/001/002` with the actual device path obtained in the previous step.
    *   Replace `/data/data/com.termux/files/usr/tmp/klipper_serial_pipe` with your desired full path for the named pipe. Using a path within Termux's accessible file system (like `/data/data/com.termux/files/usr/tmp/`) is recommended.

    The program will first use `termux-usb` to request USB device permission. If granted, `termux-usb` will re-execute `termux_uart_bridge` with a file descriptor for the USB device. The bridge will then create the named pipe and start relaying data.

**Connecting Klipper:**

1.  In your Klipper `printer.cfg` file, configure the `[mcu]` section to use the named pipe:
    ```ini
    [mcu]
    serial: /data/data/com.termux/files/usr/tmp/klipper_serial_pipe
    # baud: 250000 # Or your MCU's configured baud rate. See notes below.
    ```
2.  Ensure Klipper (specifically the `klippy` service) is started *after* `termux_uart_bridge` has successfully created the named pipe and is running.

**Important Disclaimer:**

> Due to limitations in the automated development environment, the `termux_uart_bridge` binary could not be compiled or fully tested by the automated agent. Users should build and test this utility carefully in their own Termux environment.

**Troubleshooting/Notes:**

*   The `termux_uart_bridge` program must remain running in a Termux session (e.g., using `tmux` or by running it in the foreground of a dedicated Termux session) for the named pipe to function.
*   Serial communication parameters (baud rate, data bits, parity, stop bits) are currently hardcoded in `examples/termux_uart_bridge.cpp` (defaulting to 115200 baud, 8 data bits, no parity, 1 stop bit). If your USB-serial device or target MCU requires different settings, you will need to modify the `serial_params` variable in `termux_uart_bridge.cpp` and recompile the utility. Future enhancements might include command-line arguments for these settings.

### Building 

1. Get USBUART library sources

		git clone https://github.com/hutorny/usbuart.git

2. Get libusb sources

		cd usbuart
		git clone https://github.com/libusb/libusb.git

3. Change directory to libusb

		cd libusb

4. Configure and build libusb

		./autogen.sh --disable-udev
		make

5. Change directory to usbuart 

		cd ..

6. Make USBUART

		make

### Building for Android	

1. Get USBUART library sources

		git clone https://github.com/hutorny/usbuart.git

2. Get Android-tuned libusb fork 

		cd usbuart
		git clone https://github.com/hutorny/libusb.git

3. Download the latest NDK from:
   http://developer.android.com/tools/sdk/ndk/index.html

4. Extract the NDK.

5. Open a shell and make sure there exist an NDK global variable
   set to the directory where you extracted the NDK.

6. Change directory to usbuart/libusb

7. Configure libusb 

		./android/autogen.sh --enable-system_log

8. Change directory to usbuart

		cd ..

9. Build libary and modules 

		ant debug
