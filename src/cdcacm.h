// cdcacm.h
#ifndef USBUART_CDCACM_H_
#define USBUART_CDCACM_H_

#include "usbuart.hpp"

namespace usbuart {
namespace driver {

// CDC Class Codes
#define USB_CLASS_CDC                   0x02
#define USB_CDC_SUBCLASS_ACM            0x02
#define USB_CLASS_CDC_DATA              0x0A

// CDC ACM Request Codes (sent to Communication Class Interface)
#define CDC_SET_LINE_CODING             0x20
#define CDC_GET_LINE_CODING             0x21
#define CDC_SET_CONTROL_LINE_STATE      0x22
#define CDC_SEND_BREAK                  0x23

// CDC bmRequestType
#define CDC_REQTYPE_HOST_TO_DEVICE      0x21 // Class, Interface, Host-to-Device
#define CDC_REQTYPE_DEVICE_TO_HOST      0xA1 // Class, Interface, Device-to-Host

// Line Coding Structure (7 bytes)
struct cdc_line_coding {
    uint32_t dwDTERate;     // Baud rate
    uint8_t  bCharFormat;   // Stop bits (0=1, 1=1.5, 2=2)
    uint8_t  bParityType;   // Parity (0=None, 1=Odd, 2=Even, 3=Mark, 4=Space)
    uint8_t  bDataBits;     // Data bits (5, 6, 7, 8, or 16)
} __attribute__((packed));


class cdcacm : public driver {
public:
    // Note: ifc_desc here refers to the Data Class Interface (DCI)
    // The comm_ifc_num is the Communication Class Interface (CCI) number
    cdcacm(libusb_device_handle* dev, const ifc_desc& dci_ifc_desc, uint8_t cci_ifc_num, const eia_tia_232_info& pi);
    ~cdcacm() override;

    void setup(const eia_tia_232_info& pi) override throw(error_t);
    void reset() override throw(error_t); // CDC-ACM doesn't have a specific reset, re-init
    void sendbreak(bool on) override throw(error_t);

private:
    int ctrl_transfer(uint8_t bmRequestType, uint8_t bRequest,
                      uint16_t wValue, uint16_t wIndex,
                      unsigned char* data, uint16_t wLength,
                      unsigned int timeout) throw(error_t);

    void set_line_coding(const eia_tia_232_info& pi) throw(error_t);
    void set_control_line_state(bool dtr, bool rts) throw(error_t);

    eia_tia_232_info current_params;
    uint8_t communication_if_num; // Interface number of the CCI
    // Data interface number is stored in the base class's ifc.id
    
    bool dtr_state;
    bool rts_state;
};


class cdcacm_factory : public factory {
public:
    cdcacm_factory() noexcept;
    driver* create(libusb_device_handle* dev, uint8_t ifcnum_hint) const override throw(error_t);
    // ifcnum_hint could be either CCI or DCI ifc number, factory needs to figure it out.
};

} // namespace driver
} // namespace usbuart

#endif // USBUART_CDCACM_H_
