// termux_uart_bridge.cpp
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstdlib>      // EXIT_FAILURE, EXIT_SUCCESS
#include <cstring>      // strerror, memset
#include <fcntl.h>      // open, O_RDONLY, O_WRONLY
#include <unistd.h>     // read, write, close, unlink, STDERR_FILENO (for safer signal write)
#include <sys/stat.h>   // mkfifo, mode_t
#include <sys/poll.h>   // poll, pollfd
#include <cerrno>       // errno
#include <stdexcept>    // For std::stoi exceptions

#include "usbuart.h" // Your provided header

const char* PIPE_NAME = "./serial_pipe";
std::atomic<bool> quit_flag(false);

void signal_handler(int signum) {
    // Minimal, async-signal-safe action: set the flag.
    // Optionally, write a message using async-signal-safe functions.
    // const char msg[] = "Signal received, shutting down...\n";
    // write(STDERR_FILENO, msg, sizeof(msg) - 1);
    quit_flag.store(true);
}

// Helper function to write all data, handling short writes and EINTR
bool write_all(int fd, const char* buf, ssize_t len) {
    ssize_t written = 0;
    while (written < len) {
        ssize_t ret = write(fd, buf + written, len - written);
        if (ret < 0) {
            if (errno == EINTR) continue; // Interrupted by signal, retry
            std::cerr << "write_all error: " << strerror(errno) << " on fd " << fd << std::endl;
            return false;
        }
        if (ret == 0 && len > 0) { // Typically means pipe/socket closed by other end
            std::cerr << "write_all error: wrote 0 bytes on fd " << fd << " (likely closed by peer)" << std::endl;
            return false;
        }
        written += ret;
    }
    return true;
}

int main(int argc, char* argv[]) {
    // 1. Get Termux USB FD from command line argument
    if (argc < 2) {
        std::cerr << "Error: No USB file descriptor provided as argument." << std::endl;
        std::cerr << "Usage: This program is intended to be run via termux-usb:" << std::endl;
        std::cerr << "  termux-usb -e ./termux_uart_bridge -r /dev/bus/usb/XXX/YYY" << std::endl;
        std::cerr << "where ./termux_uart_bridge is this executable, and /dev/bus/usb/XXX/YYY is the USB device path." << std::endl;
        return EXIT_FAILURE;
    }

    const char* usb_fd_arg_str = argv[1];
    int usb_dev_fd = -1;
    try {
        usb_dev_fd = std::stoi(usb_fd_arg_str);
    } catch (const std::invalid_argument& ia) {
        std::cerr << "Error: Invalid USB file descriptor argument '" << usb_fd_arg_str << "'. Not a number." << std::endl;
        return EXIT_FAILURE;
    } catch (const std::out_of_range& oor) {
        std::cerr << "Error: USB file descriptor argument '" << usb_fd_arg_str << "' out of range." << std::endl;
        return EXIT_FAILURE;
    }

    if (usb_dev_fd < 0) { 
        std::cerr << "Error: Invalid USB file descriptor value parsed: " << usb_dev_fd << " (must be non-negative)." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Using USB device FD: " << usb_dev_fd << " (from argv[1])" << std::endl;

    // 2. Setup signal handling
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    // sa.sa_flags = SA_RESTART; // To auto-restart some syscalls; not strictly needed here as we handle EINTR
    if (sigaction(SIGINT, &sa, nullptr) == -1) {
        perror("Error setting SIGINT handler");
        return EXIT_FAILURE;
    }
    if (sigaction(SIGTERM, &sa, nullptr) == -1) {
        perror("Error setting SIGTERM handler");
        return EXIT_FAILURE;
    }
    // Optional: Ignore SIGPIPE if you prefer to handle EPIPE errors from write() directly
    // signal(SIGPIPE, SIG_IGN);

    // 3. UART parameters
    usbuart::eia_tia_232_info uart_params = usbuart::_115200_8N1n;

    // 4. Initialize usbuart context
    usbuart::context& uart_ctx = usbuart::context::instance();
    // usbuart::context::setloglevel(usbuart::loglevel_t::debug); // Uncomment for verbose logging

    usbuart::channel termux_usb_channel = usbuart::bad_channel;

    // IMPORTANT NOTE on usbuart::context::pipe with an fd:
    // The provided usbuart.h has the C++ declaration:
    //   int pipe(int fd, channel& ch, const eia_tia_232_info& pi) noexcept;
    // This declaration does NOT take an interface number (ifc) directly.
    // However, its Doxygen comment *does* mention "@param ifc".
    // The usbuart library implementation for this function will need to determine
    // the USB interface (e.g., defaulting to 0, or auto-detecting).
    // If your specific USB device requires a non-default interface number, AND
    // this `pipe` overload does not correctly select it, you might encounter issues.
    // In such a case:
    //   a) Check if your local `usbuart.h` or library version has an overload
    //      `pipe(int fd, uint8_t ifc, channel& ch, ...)` and use that.
    //   b) Or, you might need to use `usbuart::context::attach` instead:
    //      `int my_pipes_to_usb[2], my_pipes_from_usb[2];`
    //      `pipe(my_pipes_to_usb); pipe(my_pipes_from_usb);`
    //      `usbuart::channel custom_channel = {my_pipes_from_usb[0], my_pipes_to_usb[1]};`
    //      `uint8_t ifc_num = 0; // Or your required interface`
    //      `uart_ctx.attach(usb_dev_fd, ifc_num, custom_channel, uart_params);`
    //      Then, adapt the poll loop to use `my_pipes_to_usb[0]` for reading from serial_pipe
    //      and `my_pipes_from_usb[1]` for writing to serial_pipe.
    //
    // Adhering strictly to the provided C++ header declaration:
    int usbuart_ret = uart_ctx.pipe(usb_dev_fd, termux_usb_channel, uart_params);

    if (usbuart_ret != +usbuart::error_t::success) {
        std::cerr << "Error: usbuart_ctx.pipe failed with code: " << usbuart_ret << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "USB UART channel opened: usbuart_read_fd=" << termux_usb_channel.fd_read 
              << ", usbuart_write_fd=" << termux_usb_channel.fd_write << std::endl;

    // 5. Create and open named pipe
    if (mkfifo(PIPE_NAME, 0666) < 0) {
        if (errno != EEXIST) {
            std::cerr << "Error: mkfifo(\"" << PIPE_NAME << "\") failed: " << strerror(errno) << std::endl;
            uart_ctx.close(termux_usb_channel);
            return EXIT_FAILURE;
        }
        std::cout << "Named pipe \"" << PIPE_NAME << "\" already exists. Using it." << std::endl;
    } else {
        std::cout << "Named pipe \"" << PIPE_NAME << "\" created." << std::endl;
    }

    std::cout << "Waiting for a writer to connect to " << PIPE_NAME << " (e.g., 'cat > " << PIPE_NAME << "')..." << std::endl;
    int pipe_user_read_fd = open(PIPE_NAME, O_RDONLY);
    if (pipe_user_read_fd < 0) {
        std::cerr << "Error: open(\"" << PIPE_NAME << "\", O_RDONLY) failed: " << strerror(errno) << std::endl;
        uart_ctx.close(termux_usb_channel);
        unlink(PIPE_NAME); 
        return EXIT_FAILURE;
    }
    std::cout << PIPE_NAME << " opened for reading from user." << std::endl;

    std::cout << "Waiting for a reader to connect to " << PIPE_NAME << " (e.g., 'cat " << PIPE_NAME << "')..." << std::endl;
    int pipe_user_write_fd = open(PIPE_NAME, O_WRONLY);
    if (pipe_user_write_fd < 0) {
        std::cerr << "Error: open(\"" << PIPE_NAME << "\", O_WRONLY) failed: " << strerror(errno) << std::endl;
        close(pipe_user_read_fd);
        uart_ctx.close(termux_usb_channel);
        unlink(PIPE_NAME);
        return EXIT_FAILURE;
    }
    std::cout << PIPE_NAME << " opened for writing to user." << std::endl;
    std::cout << "Bridge is active. Press Ctrl+C to exit." << std::endl;

    // 6. Start usbuart_loop in a thread
    std::thread uart_loop_thread([&uart_ctx]() { // Removed unused termux_usb_channel capture
        std::cout << "usbuart_loop thread started." << std::endl;
        int loop_ret = uart_ctx.loop(-1); // Blocks until event, error, or all channels closed
        std::cout << "usbuart_loop thread finished with code: " << loop_ret << std::endl;
        if (loop_ret != +usbuart::error_t::success && loop_ret != +usbuart::error_t::no_channels) {
            std::cerr << "Warning: usbuart_loop exited with error code: " << loop_ret << std::endl;
            // quit_flag.store(true); // Optionally, trigger main shutdown if USB loop fails
        }
    });

    // 7. Main relay loop using poll
    struct pollfd fds_to_poll[2];
    char buffer[4096];

    fds_to_poll[0].fd = pipe_user_read_fd;
    fds_to_poll[0].events = POLLIN;
    fds_to_poll[1].fd = termux_usb_channel.fd_read; 
    fds_to_poll[1].events = POLLIN;

    bool pipe_user_read_eof = false;
    bool termux_usb_read_eof = false;

    while (!quit_flag.load()) {
        struct pollfd current_poll_set[2];
        int nfds_this_poll = 0;

        if (!pipe_user_read_eof) {
            current_poll_set[nfds_this_poll++] = fds_to_poll[0];
        }
        if (!termux_usb_read_eof) {
            current_poll_set[nfds_this_poll++] = fds_to_poll[1];
        }
        
        if (nfds_this_poll == 0) {
            std::cout << "Both data sources are EOF. Exiting relay loop." << std::endl;
            quit_flag.store(true); // Ensure outer loop/cleanup is triggered
            break; 
        }

        int poll_ret = poll(current_poll_set, nfds_this_poll, 250); // 250ms timeout

        if (poll_ret < 0) {
            if (errno == EINTR) { 
                if (quit_flag.load()) break; // Signal handler set quit_flag
                continue; 
            }
            perror("poll error");
            quit_flag.store(true); 
            break;
        }
        if (poll_ret == 0) { // Timeout: check quit_flag and continue
            // Optionally, check uart_ctx.status() here for proactive error detection
            // (see thought process for an example snippet)
            continue;
        }

        for (int i = 0; i < nfds_this_poll; ++i) {
            if (quit_flag.load()) break; // Check before processing each FD

            int current_fd = current_poll_set[i].fd;
            short revents = current_poll_set[i].revents;

            if (revents & (POLLERR | POLLNVAL)) {
                std::cerr << "Error event (POLLERR/POLLNVAL) on polled FD " << current_fd << std::endl;
                if (current_fd == fds_to_poll[0].fd) pipe_user_read_eof = true;
                if (current_fd == fds_to_poll[1].fd) {
                     termux_usb_read_eof = true;
                     if (pipe_user_write_fd != -1) { close(pipe_user_write_fd); pipe_user_write_fd = -1; }
                }
                continue;
            }

            if (revents & (POLLIN | POLLHUP)) {
                ssize_t bytes_read = read(current_fd, buffer, sizeof(buffer));
                if (bytes_read < 0) {
                    if (errno == EINTR) {
                         if (quit_flag.load()) break;
                         continue;
                    }
                    std::cerr << "read error on fd " << current_fd << ": " << strerror(errno) << std::endl;
                    if (current_fd == fds_to_poll[0].fd) pipe_user_read_eof = true;
                    if (current_fd == fds_to_poll[1].fd) {
                         termux_usb_read_eof = true;
                         if (pipe_user_write_fd != -1) { close(pipe_user_write_fd); pipe_user_write_fd = -1; }
                    }
                } else if (bytes_read == 0 || (revents & POLLHUP)) { 
                    std::cout << "EOF/HUP on fd " << current_fd << std::endl;
                    if (current_fd == fds_to_poll[0].fd) { 
                        pipe_user_read_eof = true;
                    }
                    if (current_fd == fds_to_poll[1].fd) { 
                        termux_usb_read_eof = true;
                        if (pipe_user_write_fd != -1) {
                           close(pipe_user_write_fd);
                           pipe_user_write_fd = -1; 
                        }
                    }
                } else { // Data read
                    if (current_fd == fds_to_poll[0].fd) { // Data from user_pipe -> to USB
                        if (termux_usb_channel.fd_write != -1) {
                           if (!write_all(termux_usb_channel.fd_write, buffer, bytes_read)) {
                                std::cerr << "Failed to write all data to USB channel. Shutting down." << std::endl;
                                quit_flag.store(true);
                            }
                        } else {
                            std::cerr << "User pipe data received, but USB write channel fd_write is invalid. Data dropped." << std::endl;
                            pipe_user_read_eof = true; // Stop reading if can't write
                        }
                    } else if (current_fd == fds_to_poll[1].fd) { // Data from USB -> to user_pipe
                        if (pipe_user_write_fd != -1) { 
                            if (!write_all(pipe_user_write_fd, buffer, bytes_read)) {
                                std::cerr << "Failed to write all data to named pipe. Shutting down." << std::endl;
                                quit_flag.store(true);
                            }
                        } else {
                            std::cerr << "USB data received, but pipe_user_write_fd already closed. Data dropped." << std::endl;
                        }
                    }
                }
            } // end if POLLIN | POLLHUP
        } // end for nfds_this_poll
    } // end while !quit_flag
    std::cout << "Relay poll loop finished." << std::endl;

    // 8. Cleanup
    std::cout << "Initiating cleanup..." << std::endl;

    if (pipe_user_read_fd >= 0) {
        close(pipe_user_read_fd);
        pipe_user_read_fd = -1;
    }
    if (pipe_user_write_fd >= 0) { 
        close(pipe_user_write_fd);
        pipe_user_write_fd = -1;
    }

    if (termux_usb_channel.fd_read != -1 || termux_usb_channel.fd_write != -1) {
        std::cout << "Closing USB UART channel..." << std::endl;
        uart_ctx.close(termux_usb_channel);
        termux_usb_channel = usbuart::bad_channel;
    }

    if (uart_loop_thread.joinable()) {
        std::cout << "Waiting for usbuart_loop thread to join..." << std::endl;
        uart_loop_thread.join();
        std::cout << "usbuart_loop thread joined." << std::endl;
    }

    std::cout << "Unlinking named pipe: " << PIPE_NAME << std::endl;
    if (unlink(PIPE_NAME) < 0) {
        if (errno != ENOENT) { // Don't warn if it just doesn't exist
            std::cerr << "Warning: unlink(\"" << PIPE_NAME << "\") failed: " << strerror(errno) << std::endl;
        }
    }

    std::cout << "Cleanup complete. Exiting." << std::endl;
    return EXIT_SUCCESS;
}
