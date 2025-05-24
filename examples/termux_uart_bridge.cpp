// termux_uart_bridge.cpp
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstdlib>      // getenv, atoi, EXIT_FAILURE, EXIT_SUCCESS
#include <cstring>      // strerror
#include <fcntl.h>      // open, O_RDONLY, O_WRONLY, O_NONBLOCK
#include <unistd.h>     // read, write, close, unlink
#include <sys/stat.h>   // mkfifo, mode_t
#include <sys/poll.h>   // poll, pollfd
#include <cerrno>       // errno

#include "usbuart.h" // Your provided header

const char* PIPE_NAME = "./serial_pipe";
std::atomic<bool> quit_flag(false);

void signal_handler(int signum) {
    std::cerr << "Signal " << signum << " received, initiating shutdown..." << std::endl;
    quit_flag.store(true);
}

// Helper function to write all data, handling short writes and EINTR
bool write_all(int fd, const char* buf, ssize_t len) {
    ssize_t written = 0;
    while (written < len) {
        ssize_t ret = write(fd, buf + written, len - written);
        if (ret < 0) {
            if (errno == EINTR) continue;
            std::cerr << "write_all error: " << strerror(errno) << " on fd " << fd << std::endl;
            return false;
        }
        if (ret == 0 && len > 0) { // Should not happen for blocking write unless fd is strange
            std::cerr << "write_all error: wrote 0 bytes on fd " << fd << std::endl;
            return false;
        }
        written += ret;
    }
    return true;
}

int main(int argc, char* argv[]) {
    // 1. Get Termux USB FD
    const char* termux_usb_fd_str = getenv("TERMUX_USB_FD");
    if (!termux_usb_fd_str) {
        std::cerr << "Error: TERMUX_USB_FD environment variable not set." << std::endl;
        std::cerr << "Run this executable via: termux-usb -e termux_uart_bridge -r /dev/bus/usb/XXX/YYY" << std::endl;
        return EXIT_FAILURE;
    }
    int usb_dev_fd = atoi(termux_usb_fd_str);
    if (usb_dev_fd <= 0) { // FDs are positive integers, typically >= 3 for passed FDs
        std::cerr << "Error: Invalid TERMUX_USB_FD value: " << termux_usb_fd_str << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Using USB device FD: " << usb_dev_fd << std::endl;

    // 2. Setup signal handling
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; // No SA_RESTART, so syscalls might be interrupted
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // 3. UART parameters (default, could be command-line args)
    usbuart::eia_tia_232_info uart_params = usbuart::_115200_8N1n;
    uint8_t usb_ifc_num = 0; // Common default for USB-serial adapters

    // 4. Initialize usbuart context
    usbuart::context& uart_ctx = usbuart::context::instance(); // Get singleton
    // usbuart::context::setloglevel(usbuart::loglevel_t::debug); // Optional: for more verbose logging

    usbuart::channel termux_usb_channel = usbuart::bad_channel;
    int usbuart_ret = uart_ctx.pipe(usb_dev_fd, usb_ifc_num, termux_usb_channel, uart_params);

    if (usbuart_ret != +usbuart::error_t::success) {
        std::cerr << "Error: usbuart_ctx.pipe failed with code: " << usbuart_ret << std::endl;
        // Note: The original TERMUX_USB_FD (usb_dev_fd) is managed by termux-usb,
        // we should not close it here. The usbuart library will handle its copy.
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

    // Open named pipe for reading (data from user -> to USB)
    // This will block until a writer opens the other end.
    std::cout << "Waiting for a writer to connect to " << PIPE_NAME << "..." << std::endl;
    int pipe_user_read_fd = open(PIPE_NAME, O_RDONLY);
    if (pipe_user_read_fd < 0) {
        std::cerr << "Error: open(\"" << PIPE_NAME << "\", O_RDONLY) failed: " << strerror(errno) << std::endl;
        uart_ctx.close(termux_usb_channel);
        unlink(PIPE_NAME); // Attempt to clean up
        return EXIT_FAILURE;
    }
    std::cout << PIPE_NAME << " opened for reading from user." << std::endl;

    // Open named pipe for writing (data from USB -> to user)
    // This will block until a reader opens the other end.
    std::cout << "Waiting for a reader to connect to " << PIPE_NAME << "..." << std::endl;
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
    std::cout << "In other Termux sessions, run:" << std::endl;
    std::cout << "  cat > " << PIPE_NAME << "  # To send data to USB device" << std::endl;
    std::cout << "  cat " << PIPE_NAME << "    # To receive data from USB device" << std::endl;


    // 6. Start usbuart_loop in a thread
    std::thread uart_loop_thread([&uart_ctx]() {
        std::cout << "usbuart_loop thread started." << std::endl;
        int loop_ret = uart_ctx.loop(-1); // Loop indefinitely until no channels or error
        // This loop should exit when uart_ctx.close() is called on its channel or if an error occurs.
        std::cout << "usbuart_loop thread finished with code: " << loop_ret << std::endl;
    });

    // 7. Main relay loop using poll
    struct pollfd fds[2];
    char buffer[4096]; // Buffer for relaying data

    // FD for reading from named pipe (user input -> USB)
    fds[0].fd = pipe_user_read_fd;
    fds[0].events = POLLIN;

    // FD for reading from usbuart (USB input -> user output)
    fds[1].fd = termux_usb_channel.fd_read;
    fds[1].events = POLLIN;

    bool pipe_user_read_eof = false;
    bool termux_usb_read_eof = false;

    while (!quit_flag.load()) {
        if (pipe_user_read_eof && termux_usb_read_eof) {
            std::cout << "Both sides EOF. Exiting relay loop." << std::endl;
            break; // Both sides are done
        }

        struct pollfd current_fds[2];
        int nfds_to_poll = 0;

        if (!pipe_user_read_eof) {
            current_fds[nfds_to_poll++] = fds[0];
        }
        if (!termux_usb_read_eof) {
            current_fds[nfds_to_poll++] = fds[1];
        }
        
        if (nfds_to_poll == 0) break; // Should be caught by the check above, but for safety

        int poll_ret = poll(current_fds, nfds_to_poll, 250); // Timeout in ms to check quit_flag

        if (poll_ret < 0) {
            if (errno == EINTR) continue; // Interrupted by signal, check quit_flag
            std::cerr << "poll error: " << strerror(errno) << std::endl;
            quit_flag.store(true); // Trigger shutdown on poll error
            break;
        }
        if (poll_ret == 0) { // Timeout
            continue;
        }

        // Check which FDs are ready by iterating through the ones we polled
        for (int i = 0; i < nfds_to_poll; ++i) {
            if (current_fds[i].revents == 0) continue; // No events for this FD

            if (current_fds[i].revents & (POLLERR | POLLNVAL)) {
                std::cerr << "Error event on polled FD " << current_fds[i].fd << std::endl;
                if (current_fds[i].fd == fds[0].fd) pipe_user_read_eof = true;
                if (current_fds[i].fd == fds[1].fd) termux_usb_read_eof = true;
                continue;
            }

            if (current_fds[i].revents & (POLLIN | POLLHUP)) {
                ssize_t bytes_read = read(current_fds[i].fd, buffer, sizeof(buffer));
                if (bytes_read < 0) {
                    if (errno == EINTR) continue;
                    std::cerr << "read error on fd " << current_fds[i].fd << ": " << strerror(errno) << std::endl;
                    if (current_fds[i].fd == fds[0].fd) pipe_user_read_eof = true;
                    if (current_fds[i].fd == fds[1].fd) termux_usb_read_eof = true;
                } else if (bytes_read == 0 || (current_fds[i].revents & POLLHUP)) { // EOF or HUP
                    std::cout << "EOF/HUP on fd " << current_fds[i].fd << std::endl;
                    if (current_fds[i].fd == fds[0].fd) { // User input pipe closed
                        pipe_user_read_eof = true;
                        // Optionally, signal EOF to USB write side if usbuart supports it (e.g., shutdown WR)
                        // For now, usbuart_close will handle it during cleanup.
                        // Or, if we want to close just the write part to USB:
                        // if(termux_usb_channel.fd_write != -1) close(termux_usb_channel.fd_write);
                        // termux_usb_channel.fd_write = -1; // Mark as closed
                        // However, usbuart library manages these FDs; direct close might be bad.
                    }
                    if (current_fds[i].fd == fds[1].fd) { // USB read pipe closed
                        termux_usb_read_eof = true;
                        // Close our write end to the user pipe to signal EOF
                        if (pipe_user_write_fd != -1) {
                           close(pipe_user_write_fd);
                           pipe_user_write_fd = -1; // Mark as closed by us
                        }
                    }
                } else { // Data read
                    if (current_fds[i].fd == fds[0].fd) { // Data from user_pipe -> to USB
                        // std::cout << "Relaying " << bytes_read << " bytes from pipe to USB" << std::endl;
                        if (!write_all(termux_usb_channel.fd_write, buffer, bytes_read)) {
                            std::cerr << "Failed to write all data to USB channel. Shutting down." << std::endl;
                            quit_flag.store(true);
                        }
                    } else if (current_fds[i].fd == fds[1].fd) { // Data from USB -> to user_pipe
                        // std::cout << "Relaying " << bytes_read << " bytes from USB to pipe" << std::endl;
                        if (pipe_user_write_fd != -1) { // Check if not already closed
                            if (!write_all(pipe_user_write_fd, buffer, bytes_read)) {
                                std::cerr << "Failed to write all data to named pipe. Shutting down." << std::endl;
                                quit_flag.store(true);
                            }
                        } else {
                            std::cerr << "USB data received, but pipe_user_write_fd already closed." << std::endl;
                        }
                    }
                }
            }
             if (quit_flag.load()) break; // check after each FD processing
        }
    }
    std::cout << "Relay poll loop finished." << std::endl;

    // 8. Cleanup
    std::cout << "Initiating cleanup..." << std::endl;
    quit_flag.store(true); // Ensure quit flag is set for other parts

    // Close local ends of the named pipe first
    if (pipe_user_read_fd >= 0) {
        close(pipe_user_read_fd);
        pipe_user_read_fd = -1;
    }
    if (pipe_user_write_fd >= 0) { // Might be already closed if termux_usb_read_eof caused it
        close(pipe_user_write_fd);
        pipe_user_write_fd = -1;
    }

    // Close the usbuart channel. This should signal the usbuart_loop to stop processing for this channel.
    if (termux_usb_channel.fd_read != -1 || termux_usb_channel.fd_write != -1) {
        std::cout << "Closing USB UART channel..." << std::endl;
        uart_ctx.close(termux_usb_channel);
        // The FDs within termux_usb_channel are now closed by the library
        termux_usb_channel = usbuart::bad_channel;
    }

    // Wait for the usbuart_loop thread to finish
    if (uart_loop_thread.joinable()) {
        std::cout << "Waiting for usbuart_loop thread to join..." << std::endl;
        uart_loop_thread.join();
        std::cout << "usbuart_loop thread joined." << std::endl;
    }

    // Remove the named pipe
    std::cout << "Unlinking named pipe: " << PIPE_NAME << std::endl;
    if (unlink(PIPE_NAME) < 0) {
        std::cerr << "Warning: unlink(\"" << PIPE_NAME << "\") failed: " << strerror(errno) << std::endl;
    }

    // The original usb_dev_fd (from TERMUX_USB_FD) is managed by termux-usb process.
    // We don't close it.

    std::cout << "Cleanup complete. Exiting." << std::endl;
    return EXIT_SUCCESS;
}
