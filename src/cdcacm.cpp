// cdcacm.cpp
#include <cstring> // For memcpy
#include <stdexcept>
#include <algorithm> // For std::min
#include <libusb.h>
#include "usbuart.hpp"
#include "cdcacm.h"

namespace usbuart {
namespace driver {

static cdcacm_factory _cdcacm_factory_instance;

cdcacm_factory::cdcacm_factory() noexcept : factory() {
    log.i(__, "CDC-ACM driver factory registered");
}

driver* cdcacm_factory::create(libusb_device_handle* devh, uint8_t ifcnum_hint) const throw(error_t) {
    libusb_device* dev = libusb_get_device(devh);
    libusb_config_descriptor *config_desc = nullptr;
    int ret = libusb_get_active_config_descriptor(dev, &config_desc);
    if (ret < 0 || !config_desc) {
        log.e(__, "CDC-ACM: Failed to get active config descriptor: %s", libusb_error_name(ret));
        return nullptr;
    }

    int cci_ifc_num = -1; // Communication Class Interface number
    int dci_ifc_num = -1; // Data Class Interface number
    ifc_desc drv_dci_desc; // usbuart's ifc_desc for the DCI

    // Iterate through interfaces to find a pair of CCI and DCI
    // A common pattern is an Interface Association Descriptor (IAD) grouping them.
    // Or, DCI often immediately follows CCI.
    for (uint8_t i = 0; i < config_desc->bNumInterfaces; ++i) {
        const libusb_interface* interface = &config_desc->interface[i];
        if (interface->num_altsetting > 0) {
            const libusb_interface_descriptor* if_desc = &interface->altsetting[0];

            if (if_desc->bInterfaceClass == USB_CLASS_CDC &&
                if_desc->bInterfaceSubClass == USB_CDC_SUBCLASS_ACM) {
                cci_ifc_num = if_desc->bInterfaceNumber;
                log.d(__, "CDC-ACM: Found potential CCI at ifc %d", cci_ifc_num);

                // Now look for the associated DCI
                // DCI usually follows CCI immediately or is specified by a functional descriptor in CCI's extra data.
                // For simplicity, we'll assume DCI is cci_ifc_num + 1 if not explicitly linked or ifcnum_hint is DCI
                
                // Try to find DCI:
                // 1. If cci_ifc_num + 1 exists and is a DCI
                // 2. Or if ifcnum_hint itself is a DCI and we found a compatible CCI
                int potential_dci_num = cci_ifc_num + 1;
                if (ifcnum_hint == potential_dci_num || ifcnum_hint == cci_ifc_num) { // if hint matches either part of a pair
                    // Check if cci_ifc_num + 1 is a valid DCI
                     if ( (uint8_t)(cci_ifc_num + 1) < config_desc->bNumInterfaces) {
                        const libusb_interface* next_interface = &config_desc->interface[cci_ifc_num + 1];
                        if (next_interface->num_altsetting > 0) {
                            const libusb_interface_descriptor* next_if_desc = &next_interface->altsetting[0];
                            if (next_if_desc->bInterfaceClass == USB_CLASS_CDC_DATA) {
                                dci_ifc_num = next_if_desc->bInterfaceNumber;
                                log.d(__, "CDC-ACM: Found DCI ifc %d associated with CCI ifc %d", dci_ifc_num, cci_ifc_num);
                            }
                        }
                    }
                }
                // More robust: parse CDC Functional Descriptors within CCI's extra data
                // to find the bDataInterface field if present. This is complex.

                if (dci_ifc_num != -1) { // Found a CCI/DCI pair
                    // Populate drv_dci_desc with DCI's endpoint info
                    const libusb_interface_descriptor* dci_actual_if_desc = &config_desc->interface[dci_ifc_num].altsetting[0];
                    uint8_t ep_in = 0, ep_out = 0;
                    uint16_t in_max_packet = 0, out_max_packet = 0;

                    for (int k = 0; k < dci_actual_if_desc->bNumEndpoints; ++k) {
                        const libusb_endpoint_descriptor* ep = &dci_actual_if_desc->endpoint[k];
                        if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK) {
                            if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                                ep_in = ep->bEndpointAddress;
                                in_max_packet = ep->wMaxPacketSize;
                            } else {
                                ep_out = ep->bEndpointAddress;
                                out_max_packet = ep->wMaxPacketSize;
                            }
                        }
                    }
                    if (ep_in && ep_out) {
                        drv_dci_desc.id = dci_ifc_num;
                        drv_dci_desc.ep_bulk_in = ep_in;
                        drv_dci_desc.ep_bulk_out = ep_out;
                        drv_dci_desc.chunk_size = std::min(in_max_packet, out_max_packet);
                        if (drv_dci_desc.chunk_size == 0) drv_dci_desc.chunk_size = 64; // Default

                        log.i(__, "CDC-ACM: Valid pair: CCI=%d, DCI=%d (EP_IN:0x%02X, EP_OUT:0x%02X, Chunk:%d)",
                              cci_ifc_num, drv_dci_desc.id, drv_dci_desc.ep_bulk_in, drv_dci_desc.ep_bulk_out, drv_dci_desc.chunk_size);
                        goto found_pair; // Exit loops
                    } else {
                        dci_ifc_num = -1; // Invalid DCI, reset
                    }
                }
                // If DCI wasn't cci_ifc_num + 1, reset cci_ifc_num to continue search for other CCIs
                if (dci_ifc_num == -1) cci_ifc_num = -1;

            } // end if USB_CLASS_CDC
        } // end if num_altsetting
    } // end for interfaces

found_pair:
    libusb_free_config_descriptor(config_desc);

    if (cci_ifc_num == -1 || dci_ifc_num == -1) {
        log.w(__, "CDC-ACM: Could not find a suitable CCI/DCI pair (hint: %d)", ifcnum_hint);
        return nullptr;
    }

    // Claim both interfaces
    ret = libusb_claim_interface(devh, cci_ifc_num);
    if (ret < 0) {
        log.e(__, "CDC-ACM: Failed to claim CCI %d: %s", cci_ifc_num, libusb_error_name(ret));
        return nullptr;
    }
    log.i(__, "CDC-ACM: CCI %d claimed", cci_ifc_num);

    ret = libusb_claim_interface(devh, dci_ifc_num);
    if (ret < 0) {
        log.e(__, "CDC-ACM: Failed to claim DCI %d: %s", dci_ifc_num, libusb_error_name(ret));
        libusb_release_interface(devh, cci_ifc_num); // Release CCI
        return nullptr;
    }
    log.i(__, "CDC-ACM: DCI %d claimed", dci_ifc_num);

    eia_tia_232_info initial_pi = {115200, 8, parity_t::none, stop_bits_t::one, flow_control_t::none_};
    cdcacm* instance = nullptr;
    try {
        instance = new cdcacm(devh, drv_dci_desc, cci_ifc_num, initial_pi);
    } catch (const error_t& err) {
        log.e(__, "CDC-ACM: Constructor failed with error_t %d", +err);
        libusb_release_interface(devh, dci_ifc_num);
        libusb_release_interface(devh, cci_ifc_num);
        return nullptr;
    } catch (const std::exception& e) {
        log.e(__, "CDC-ACM: Constructor failed with exception: %s", e.what());
        libusb_release_interface(devh, dci_ifc_num);
        libusb_release_interface(devh, cci_ifc_num);
        return nullptr;
    }
    return instance;
}


cdcacm::cdcacm(libusb_device_handle* dev, const ifc_desc& dci_ifc_desc, uint8_t cci_ifc_num, const eia_tia_232_info& pi)
    : driver(dev, dci_ifc_desc), current_params(pi), communication_if_num(cci_ifc_num), dtr_state(false), rts_state(false) {
    log.i(__, "CDC-ACM driver instance created (CCI:%d, DCI:%d)", communication_if_num, dci_ifc_desc.id);
}

cdcacm::~cdcacm() {
    log.i(__, "CDC-ACM driver instance (CCI:%d, DCI:%d) destroying", communication_if_num, ifc.id);
    // Try to clear DTR/RTS on exit
    try {
        set_control_line_state(false, false);
    } catch (...) { /* ignore */ }

    int ret_dci = libusb_release_interface(devh, ifc.id); // Release DCI
    int ret_cci = libusb_release_interface(devh, communication_if_num); // Release CCI
    if (ret_dci < 0) log.e(__, "CDC-ACM: Failed to release DCI %d: %s", ifc.id, libusb_error_name(ret_dci));
    else log.i(__, "CDC-ACM: DCI %d released", ifc.id);
    if (ret_cci < 0) log.e(__, "CDC-ACM: Failed to release CCI %d: %s", communication_if_num, libusb_error_name(ret_cci));
    else log.i(__, "CDC-ACM: CCI %d released", communication_if_num);
}

int cdcacm::ctrl_transfer(uint8_t bmRequestType, uint8_t bRequest,
                           uint16_t wValue, uint16_t wIndex, // wIndex is CCI number here
                           unsigned char* data, uint16_t wLength,
                           unsigned int timeout) throw(error_t) {
    int ret = libusb_control_transfer(devh, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout);
    if (ret < 0) {
        log.e(__, "CDC-ACM: Control transfer failed (req:0x%02X, val:0x%04X, idx:0x%04X): %s (%d)",
              bRequest, wValue, wIndex, libusb_error_name(ret), ret);
        if (ret == LIBUSB_ERROR_PIPE) throw error_t::control_error;
        if (ret == LIBUSB_ERROR_TIMEOUT) throw error_t::usb_error;
        if (ret == LIBUSB_ERROR_NO_DEVICE) throw error_t::no_device;
        throw error_t::control_error;
    }
    return ret;
}

void cdcacm::set_line_coding(const eia_tia_232_info& pi) throw(error_t) {
    cdc_line_coding coding;
    coding.dwDTERate = htole32(pi.baudrate); // Ensure little-endian

    switch (pi.stopbits) {
        case stop_bits_t::one:   coding.bCharFormat = 0; break;
        case stop_bits_t::_1_5:  coding.bCharFormat = 1; break;
        case stop_bits_t::two:   coding.bCharFormat = 2; break;
        default: throw error_t::invalid_param;
    }

    switch (pi.parity) {
        case parity_t::none:  coding.bParityType = 0; break;
        case parity_t::odd:   coding.bParityType = 1; break;
        case parity_t::even:  coding.bParityType = 2; break;
        case parity_t::mark:  coding.bParityType = 3; break;
        case parity_t::space: coding.bParityType = 4; break;
        default: throw error_t::invalid_param;
    }

    coding.bDataBits = pi.databits;
    if (pi.databits < 5 || (pi.databits > 8 && pi.databits != 16)) { // CDC supports 5,6,7,8,16
        throw error_t::invalid_param;
    }

    log.d(__, "CDC-ACM: Setting Line Coding (CCI:%d) - Baud:%u, Stop:%d, Parity:%d, Data:%d",
          communication_if_num, pi.baudrate, coding.bCharFormat, coding.bParityType, coding.bDataBits);

    ctrl_transfer(CDC_REQTYPE_HOST_TO_DEVICE, CDC_SET_LINE_CODING,
                  0x0000, this->communication_if_num,
                  reinterpret_cast<unsigned char*>(&coding), sizeof(coding), 1000);
}

void cdcacm::set_control_line_state(bool dtr, bool rts) throw(error_t) {
    uint16_t control_value = 0;
    if (dtr) control_value |= 0x0001; // DTR present
    if (rts) control_value |= 0x0002; // RTS present (Carrier Control for ACM)

    log.d(__, "CDC-ACM: Setting Control Line State (CCI:%d) to 0x%04X (DTR:%d, RTS:%d)",
          communication_if_num, control_value, dtr, rts);

    ctrl_transfer(CDC_REQTYPE_HOST_TO_DEVICE, CDC_SET_CONTROL_LINE_STATE,
                  control_value, this->communication_if_num,
                  nullptr, 0, 1000);
    this->dtr_state = dtr;
    this->rts_state = rts;
}

void cdcacm::setup(const eia_tia_232_info& pi) throw(error_t) {
    log.i(__, "CDC-ACM: Setup called (CCI:%d) (Baud:%u, Data:%d, P:%d, S:%d, F:%d)",
        communication_if_num, pi.baudrate, pi.databits, static_cast<int>(pi.parity),
        static_cast<int>(pi.stopbits), static_cast<int>(pi.flowcontrol));

    set_line_coding(pi);

    // Set DTR and RTS.
    // For CDC-ACM, hardware flow control (RTS/CTS) is often implicit or handled by the device
    // based on its capabilities and SET_LINE_CODING. Explicit RTS toggle via SET_CONTROL_LINE_STATE
    // is more about modem control signals than direct flow control toggling for some devices.
    // However, many simple ACM devices use RTS from SET_CONTROL_LINE_STATE for enabling receive.
    bool dtr = true; // Typically assert DTR
    bool rts = true; // Typically assert RTS, especially if no hw flow control or if it means "ready to receive"
    
    if (pi.flowcontrol == flow_control_t::rts_cts) {
        // For CDC-ACM, actual RTS/CTS flow control is usually managed by the device
        // internally if it supports it. The host just ensures data lines are "active".
        // Some devices might require RTS to be asserted by host to enable its receiver.
        // No specific CDC request to "enable RTS/CTS flow control mode" like in vendor-specific drivers.
        // It's assumed if the device supports it, it does it.
        log.d(__, "CDC-ACM: RTS/CTS flow control requested. DTR/RTS asserted.");
    } else if (pi.flowcontrol == flow_control_t::none_) {
        log.d(__, "CDC-ACM: No flow control. DTR/RTS asserted.");
    } else {
        log.w(__, "CDC-ACM: Flow control mode %d not fully supported/implemented for generic CDC-ACM.", static_cast<int>(pi.flowcontrol));
    }
    
    set_control_line_state(dtr, rts);

    current_params = pi;
    log.i(__, "CDC-ACM: Setup complete (CCI:%d)", communication_if_num);
}

void cdcacm::reset() throw(error_t) {
    log.i(__, "CDC-ACM: Reset called (CCI:%d). Re-applying settings.", communication_if_num);
    // For CDC-ACM, a "reset" typically means re-applying the line coding and control line state.
    // Some devices might react to DTR toggle.
    set_control_line_state(false, false);
    // small delay could be added if necessary, but usbuart is synchronous for setup
    setup(current_params);
}

void cdcacm::sendbreak(bool on) throw(error_t) {
    uint16_t break_duration = on ? 0xFFFF : 0x0000; // 0xFFFF = indefinite, 0 = end
    log.i(__, "CDC-ACM: Send break %s (CCI:%d, duration:0x%04X)",
        (on ? "ON" : "OFF"), communication_if_num, break_duration);

    ctrl_transfer(CDC_REQTYPE_HOST_TO_DEVICE, CDC_SEND_BREAK,
                  break_duration, this->communication_if_num,
                  nullptr, 0, 1000);
}

} // namespace driver
} // namespace usbuart
