// cp210x.h
#ifndef USBUART_CP210X_H_
#define USBUART_CP210X_H_

#include "usbuart.hpp" // Should include the main internal header for usbuart

namespace usbuart {
namespace driver {

// CP210x Vendor IDs and Product IDs (Add more as needed)
// Silicon Labs
#define CP210X_VID_SILABS 0x10C4
#define CP210X_PID_CP2101 0xEA60 // Generic CP2101, CP2102, CP2103, CP2104
#define CP210X_PID_CP2102 CP210X_PID_CP2101
#define CP210X_PID_CP2103 CP210X_PID_CP2101
#define CP210X_PID_CP2104 CP210X_PID_CP2101
#define CP210X_PID_CP2105 0xEA70 // Dual UART
#define CP210X_PID_CP2108 0xEA71 // Quad UART

// Other vendors might use CP210x chips with their own PIDs
// Example (fictional)
// #define CP210X_PID_MYBOARD 0x1234


// CP210x Control Request Codes
#define CP210X_REQTYPE_HOST_TO_DEVICE 0x40 // bmRequestType for sending
#define CP210X_REQTYPE_DEVICE_TO_HOST 0xC0 // bmRequestType for receiving

#define CP210X_IFC_ENABLE       0x00
#define CP210X_SET_BAUDDIV      0x01 // Older name, now use SET_BAUDRATE
#define CP210X_SET_BAUDRATE     0x01
#define CP210X_GET_BAUDRATE     0x02
#define CP210X_SET_LINE_CTL     0x03
#define CP210X_GET_LINE_CTL     0x04
#define CP210X_SET_MHS          0x07 // Set Modem Handshake State (DTR/RTS)
#define CP210X_GET_MDMSTS       0x08 // Get Modem Status (CTS/DSR/etc.)
#define CP210X_SET_XON          0x09
#define CP210X_SET_XOFF         0x0A
#define CP210X_PURGE            0x12
#define CP210X_SET_FLOW         0x13
#define CP210X_GET_FLOW         0x14
// Other requests exist for GPIOs, etc., not covered here

// CP210X_IFC_ENABLE wValue
#define CP210X_UART_DISABLE     0x0000
#define CP210X_UART_ENABLE      0x0001

// CP210X_SET_LINE_CTL wValue bit masks/values
// Stop Bits (Bits 0-1)
#define CP210X_LINE_CTL_STOP_BITS_1   (0 << 0)
#define CP210X_LINE_CTL_STOP_BITS_1_5 (1 << 0)
#define CP210X_LINE_CTL_STOP_BITS_2   (2 << 0)
// Parity (Bits 2-4)
#define CP210X_LINE_CTL_PARITY_NONE   (0 << 2)
#define CP210X_LINE_CTL_PARITY_ODD    (1 << 2)
#define CP210X_LINE_CTL_PARITY_EVEN   (2 << 2)
#define CP210X_LINE_CTL_PARITY_MARK   (3 << 2)
#define CP210X_LINE_CTL_PARITY_SPACE  (4 << 2)
// Data Bits (Bits 8-11), actual value is (databits << 8)
#define CP210X_LINE_CTL_DATA_BITS_5   (5 << 8)
#define CP210X_LINE_CTL_DATA_BITS_6   (6 << 8)
#define CP210X_LINE_CTL_DATA_BITS_7   (7 << 8)
#define CP210X_LINE_CTL_DATA_BITS_8   (8 << 8)

// CP210X_PURGE wValue bit masks
#define CP210X_PURGE_RX         0x0001
#define CP210X_PURGE_TX         0x0002


class cp210x : public driver {
public:
    cp210x(libusb_device_handle* dev, const ifc_desc& ifc, const eia_tia_232_info& pi);
    ~cp210x() override;

    void setup(const eia_tia_232_info& pi) override throw(error_t);
    void reset() override throw(error_t);
    void sendbreak(bool on) override throw(error_t); // CP210x might not support break directly, needs investigation

    // Callbacks are default from base driver class if no special handling needed
    // void read_callback(libusb_transfer* transfer, size_t& pos) override;
    // void write_callback(libusb_transfer* transfer) override;

    // CP210x typically uses interface 0
    static const uint8_t default_interface = 0;

private:
    int ctrl_transfer(uint8_t bmRequestType, uint8_t bRequest,
                      uint16_t wValue, uint16_t wIndex,
                      unsigned char* data, uint16_t wLength,
                      unsigned int timeout) throw(error_t);

    void set_uart_enable(bool enable) throw(error_t);
    void set_baudrate(uint32_t baudrate) throw(error_t);
    void set_line_control(const eia_tia_232_info& pi) throw(error_t);
    void set_flow_control(const eia_tia_232_info& pi) throw(error_t); // Basic RTS/CTS
    void purge_fifos(uint16_t direction) throw(error_t);
    void set_mhs(bool dtr, bool rts) throw(error_t); // Helper for DTR/RTS


    // Store the current settings
    eia_tia_232_info current_params;
    // Interface number (usually 0 for CP210x single port)
    uint8_t interface_number;
};


class cp210x_factory : public factory {
public:
    cp210x_factory() noexcept;
    driver* create(libusb_device_handle* dev, uint8_t ifcnum) const override throw(error_t);
};

} // namespace driver
} // namespace usbuart

#endif // USBUART_CP210X_H_
