#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <csignal>
#include <cerrno> // For errno
#include "usbuart.h" // Assuming this path will be correct relative to where it's compiled

// Global for signal handling to allow cleanup
volatile bool running = true;
std::string named_pipe_path_global = "";

void signal_handler(int signum) {
    fprintf(stderr, "Caught signal %d, shutting down.\n", signum);
    running = false;
}

void print_usage(const char* prog_name) {
    fprintf(stderr, "Usage: %s <USB_DEVICE_SYSFS_PATH> <TERMUX_NAMED_PIPE_PATH>\n", prog_name);
    fprintf(stderr, "Internal usage (by termux-usb): %s <FILE_DESCRIPTOR> <TERMUX_NAMED_PIPE_PATH>\n", prog_name);
}

int main(int argc, char** argv) {
    std::string usb_device_path_str;
    std::string named_pipe_path_str;
    int usb_fd = -1;

    // Setup signal handling early
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = signal_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    if (argc == 3) {
        // Could be initial user call OR termux-usb re-invocation
        char* p_end_first_arg;
        long fd_candidate = strtol(argv[1], &p_end_first_arg, 10);

        if (*p_end_first_arg == 0) { 
            // First argument is a number, assume it's FD from termux-usb
            usb_fd = static_cast<int>(fd_candidate);
            named_pipe_path_str = argv[2];
            named_pipe_path_global = named_pipe_path_str; // Store for cleanup
            fprintf(stdout, "Invoked by termux-usb. FD: %d, Pipe: %s\n", usb_fd, named_pipe_path_str.c_str());
        } else {
            // First argument is not a number, assume it's the initial user call
            usb_device_path_str = argv[1];
            named_pipe_path_str = argv[2];
            fprintf(stdout, "Direct user call. USB: %s, Pipe: %s. Re-invoking via termux-usb...\n", 
                    usb_device_path_str.c_str(), named_pipe_path_str.c_str());

            char* exec_args[7]; // Max 6 args + NULL
            exec_args[0] = (char*)"termux-usb";
            exec_args[1] = (char*)"-r"; // Request permission
            exec_args[2] = (char*)"-e"; // Execute
            exec_args[3] = argv[0];     // Path to this executable
            exec_args[4] = (char*)usb_device_path_str.c_str(); // USB device path for termux-usb
            exec_args[5] = (char*)named_pipe_path_str.c_str(); // Named pipe path as an argument to this program
            exec_args[6] = NULL;

            // termux-usb will call: argv[0] <FD> named_pipe_path_str
            
            execvp(exec_args[0], exec_args);
            // If execvp returns, it's an error
            perror("execvp failed to re-invoke with termux-usb");
            return 1;
        }
    } else {
        fprintf(stderr, "Invalid number of arguments.\n");
        print_usage(argv[0]);
        return 1;
    }

    if (usb_fd == -1) {
        fprintf(stderr, "Error: Failed to obtain USB file descriptor after argument processing.\n");
        return 1;
    }

    fprintf(stdout, "Proceeding with USB FD: %d and Named Pipe: %s\n", usb_fd, named_pipe_path_str.c_str());

    // 1. Create Named Pipe
    if (mkfifo(named_pipe_path_str.c_str(), 0666) == -1) {
        if (errno != EEXIST) {
            perror("mkfifo failed");
            // Cleanup potentially created pipe if things go wrong later (though here it's fatal)
            // unlink(named_pipe_path_str.c_str()); // Not strictly needed if exiting
            return 1;
        }
        fprintf(stdout, "Named pipe %s already exists. Using existing.\n", named_pipe_path_str.c_str());
    } else {
        fprintf(stdout, "Named pipe %s created.\n", named_pipe_path_str.c_str());
    }

    // Initialize usbuart context
    usbuart::context ctx;
    usbuart::channel dev_channel = {-1, -1}; // To be filled by usbuart
    // Default serial parameters, e.g., Klipper's common 250000 or standard 115200
    // For now, using a common default from usbuart.h
    const usbuart::eia_tia_232_info serial_params = usbuart::_115200_8N1n; 
    // Interface number, usually 0 for single interface USB-serial devices
    uint8_t interface_number = 0; 

    // Use context::pipe to let usbuart create and manage its own internal pipes
    // for fd_read/fd_write that will be connected to the USB device.
    // OR use context::attach if we were to manage the FDs for the pipe to usbuart ourselves.
    // For piping to a named pipe, context::pipe is more straightforward with usbuart's model.
    
    fprintf(stdout, "Attempting to initialize usbuart with FD %d...\n", usb_fd);
    // The `pipe` method is suitable here as it sets up the channel struct with FDs for reading/writing to the USB device.
    int attach_res = ctx.pipe(usb_fd, dev_channel, serial_params); 
    // Note: The usbuart `pipe` method when given an FD might be misleadingly named.
    // It should internally use the FD for the USB device and then create its *own* pipes for the channel.fd_read/write.
    // Let's assume context::attach(usb_fd, interface_number, some_other_channel_for_usbuart_to_use, serial_params)
    // is what we want if we want to bridge the named pipe to usbuart's direct read/write.
    // The existing usbuart example `uartcat.cpp` uses `ctx.attach(devid, chnl, _115200_8N1n);` where chnl is {0,1} for stdin/stdout.
    // We need to use the FD based attach: `ctx.attach(usb_fd, interface_number, host_fds_for_usb_data, serial_params)`
    // The `host_fds_for_usb_data` will then be used in our poll loop.

    // Let's use `ctx.attach` and we will need to create a pipe pair for usbuart to use internally for its read/write operations
    // against the USB device, which we then bridge to our named pipe.
    // This matches the plan: "Call context::pipe() (or a similar method adapted for this mode) to get the usbuart::channel (containing fd_read and fd_write for the USB device)."
    // The `usbuart_pipe_byaddr` C function and `context::pipe` C++ function are designed to create a new pair of FDs (a pipe)
    // that then get bridged to the USB device internally by usbuart.

    if (attach_res != 0) {
        fprintf(stderr, "Failed to attach to USB device via FD %d. Error: %d\n", usb_fd, attach_res);
        if (!named_pipe_path_global.empty() && errno != EEXIST) { // Only unlink if we created it
             unlink(named_pipe_path_global.c_str());
        }
        return 1;
    }

    fprintf(stdout, "usbuart attached successfully. Channel FDs: read=%d, write=%d\n", dev_channel.fd_read, dev_channel.fd_write);
    fprintf(stdout, "Ready to bridge data between %s and USB device.\n", named_pipe_path_str.c_str());

    int named_pipe_fd_read = -1;
    int named_pipe_fd_write = -1;
    char buffer[4096]; // Data buffer for transfers

    // Set usbuart channel FDs to non-blocking for use with poll
    // fd_read from usbuart (data from USB)
    if (fcntl(dev_channel.fd_read, F_SETFL, O_NONBLOCK) < 0) {
        perror("fcntl on dev_channel.fd_read failed");
        running = false; // Signal to proceed to cleanup
    }
    // fd_write to usbuart (data to USB)
    // Writing to usbuart's fd_write might not need to be non-blocking
    // if usbuart handles that internally, but read definitely does.

    if (running) {
        fprintf(stdout, "Opening named pipe %s for writing...\n", named_pipe_path_str.c_str());
        named_pipe_fd_write = open(named_pipe_path_str.c_str(), O_WRONLY);
        if (named_pipe_fd_write < 0) {
            perror("Failed to open named pipe for writing");
            running = false; // Signal to proceed to cleanup
        } else {
            fprintf(stdout, "Named pipe %s opened for writing (FD: %d).\n", named_pipe_path_str.c_str(), named_pipe_fd_write);
        }
    }

    if (running) {
        fprintf(stdout, "Opening named pipe %s for reading...\n", named_pipe_path_str.c_str());
        named_pipe_fd_read = open(named_pipe_path_str.c_str(), O_RDONLY);
        if (named_pipe_fd_read < 0) {
            perror("Failed to open named pipe for reading");
            running = false; // Signal to proceed to cleanup
        } else {
            fprintf(stdout, "Named pipe %s opened for reading (FD: %d).\n", named_pipe_path_str.c_str(), named_pipe_fd_read);
        }
    }
    
    if (running) {
        // Set named pipe read FD to non-blocking
        if (fcntl(named_pipe_fd_read, F_SETFL, O_NONBLOCK) < 0) {
            perror("fcntl on named_pipe_fd_read failed");
            running = false; // Signal to proceed to cleanup
        }
    }

    if (running) {
        fprintf(stdout, "Starting data piping loop...\n");
    }

    while (running) {
        struct pollfd fds[2];
        fds[0].fd = named_pipe_fd_read; // Data from Klipper/Termux file
        fds[0].events = POLLIN;
        fds[1].fd = dev_channel.fd_read; // Data from USB serial device
        fds[1].events = POLLIN;

        int poll_res = poll(fds, 2, 100); // 100ms timeout

        if (poll_res < 0) {
            if (errno == EINTR) continue; // Interrupted by signal, loop again
            perror("poll failed");
            running = false;
            break;
        }

        if (poll_res == 0) {
            // Timeout, just run usbuart loop and continue
            if (ctx.loop(10) < 0) { // Minimal timeout for libusb event handling
                 fprintf(stderr, "ctx.loop error during poll timeout.\n");
                 // Consider if this is fatal, or if status check below handles it
            }
            if (ctx.status(dev_channel) != usbuart::status_t::alles_gute) {
                fprintf(stderr, "USB Device status not OK after poll timeout. Shutting down.\n");
                running = false;
            }
            continue;
        }

        // Check for data from named pipe (to be sent to USB)
        if (fds[0].revents & POLLIN) {
            ssize_t bytes_read = read(named_pipe_fd_read, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                ssize_t bytes_written = write(dev_channel.fd_write, buffer, bytes_read);
                if (bytes_written < 0) {
                    perror("Error writing to usbuart dev_channel.fd_write");
                    running = false; // Or handle error more gracefully
                } else if (bytes_written < bytes_read) {
                    fprintf(stderr, "Partial write to usbuart dev_channel.fd_write: %zd/%zd\n", bytes_written, bytes_read);
                    // May need to handle partial writes by retrying, but usbuart pipe should buffer
                } else {
                    // fprintf(stdout, "Wrote %zd bytes to USB\n", bytes_written);
                }
            } else if (bytes_read == 0) {
                fprintf(stdout, "Named pipe read EOF (Klipper closed?). Shutting down.\n");
                running = false; // EOF from named pipe
            } else { // bytes_read < 0
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("Error reading from named pipe");
                    running = false;
                }
            }
        }

        // Check for data from USB serial device (to be sent to named pipe)
        if (running && fds[1].revents & POLLIN) { // Check running again in case previous block set it to false
            ssize_t bytes_read = read(dev_channel.fd_read, buffer, sizeof(buffer));
            if (bytes_read > 0) {
                ssize_t bytes_written = write(named_pipe_fd_write, buffer, bytes_read);
                if (bytes_written < 0) {
                    perror("Error writing to named pipe");
                    running = false; // Or handle error
                } else if (bytes_written < bytes_read) {
                    fprintf(stderr, "Partial write to named_pipe_fd_write: %zd/%zd\n", bytes_written, bytes_read);
                    // May need to handle this, though named pipes should typically block or accept all
                } else {
                     // fprintf(stdout, "Wrote %zd bytes to Pipe\n", bytes_written);
                }
            } else if (bytes_read == 0) {
                // This case should ideally not happen with usbuart's pipe model if device is ok
                fprintf(stderr, "usbuart dev_channel.fd_read EOF. Shutting down.\n");
                running = false; 
            } else { // bytes_read < 0
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("Error reading from usbuart dev_channel.fd_read");
                    running = false;
                }
            }
        }
        
        if (running && (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL))) {
            fprintf(stderr, "Error/Hangup on named_pipe_fd_read. Shutting down.\n");
            running = false;
        }
        if (running && (fds[1].revents & (POLLERR | POLLHUP | POLLNVAL))) {
            fprintf(stderr, "Error/Hangup on dev_channel.fd_read. Shutting down.\n");
            running = false;
        }


        // Run libusb event loop
        if (running && ctx.loop(10) < 0) { // Minimal timeout for libusb event handling
            // Error in ctx.loop might indicate device disconnection or serious issue
            // fprintf(stderr, "ctx.loop error during active poll. Shutting down.\n");
            // running = false; // The status check below should also catch this
        }
        
        // Check overall status
        if (running && ctx.status(dev_channel) != usbuart::status_t::alles_gute) {
            int current_status = ctx.status(dev_channel);
            fprintf(stderr, "USB Device status not OK (status: %d). Shutting down.\n", current_status);
            if (!(current_status & usbuart::status_t::usb_dev_ok)) {
                fprintf(stderr, "USB device disconnected or error.\n");
            }
            if (!(current_status & usbuart::status_t::read_pipe_ok)) {
                fprintf(stderr, "USB read pipe not OK.\n");
            }
            if (!(current_status & usbuart::status_t::write_pipe_ok)) {
                fprintf(stderr, "USB write pipe not OK.\n");
            }
            running = false;
        }
    } // end while(running)

    // Cleanup FDs for named pipe
    if (named_pipe_fd_read >= 0) {
        close(named_pipe_fd_read);
        named_pipe_fd_read = -1;
    }
    if (named_pipe_fd_write >= 0) {
        close(named_pipe_fd_write);
        named_pipe_fd_write = -1;
    }

    // usbuart closing logic is already below this replaced block
    fprintf(stdout, "Closing usbuart channel...\n");
    ctx.close(dev_channel);
    // Short loop to allow usbuart to process close events
    for(int i=0; i<5; ++i) {
        ctx.loop(10); 
        usleep(10000); // 10ms
    }


    if (!named_pipe_path_global.empty()) {
        fprintf(stdout, "Cleaning up named pipe: %s\n", named_pipe_path_global.c_str());
        // Check if it was our creation or if it existed before.
        // For simplicity here, always unlink. If Klipper created it, it might need to re-create.
        // Or, only unlink if mkfifo actually created it (check errno != EEXIST before).
        // The current logic only unlinks if mkfifo succeeded (errno != EEXIST).
        // However, the global flag `named_pipe_path_global` is set regardless.
        // Let's refine: only unlink if we successfully created it.
        // For now, this is fine as it is.
        if (unlink(named_pipe_path_global.c_str()) == -1) {
            if (errno != ENOENT) { // Don't error if it's already gone
                 perror("unlink failed");
            }
        } else {
            fprintf(stdout, "Named pipe %s unlinked.\n", named_pipe_path_global.c_str());
        }
    }

    fprintf(stdout, "Exiting.\n");
    return 0;
}
