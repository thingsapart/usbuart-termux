// cp210x.cpp
#include <cstring> // For memcpy
#include <stdexcept> // For runtime_error for simplicity, usbuart uses error_t
#include <libusb.h>
#include "usbuart.hpp" // Main internal header
#include "cp210x.h"

namespace usbuart {
namespace driver {

// Instantiate the factory so it gets registered
static cp210x_factory _cp210x_factory_instance;

cp210x_factory::cp210x_factory() noexcept : factory() {
    log.i(__, "CP210x driver factory registered");
}

driver* cp210x_factory::create(libusb_device_handle* devh, uint8_t ifcnum) const throw(error_t) {
    libusb_device* dev = libusb_get_device(devh);
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(dev, &desc) < 0) {
        return nullptr; // Could not get descriptor
    }

    bool supported_pid = false;
    if (desc.idVendor == CP210X_VID_SILABS) {
        switch (desc.idProduct) {
            case CP210X_PID_CP2101: // Covers CP2101/2/3/4
            case CP210X_PID_CP2105: // Dual UART (ifcnum might be 0 or 1)
            case CP210X_PID_CP2108: // Quad UART (ifcnum 0-3)
                supported_pid = true;
                break;
            // Add other PIDs if necessary
        }
    }

    if (!supported_pid) {
        return nullptr; // Not a VID/PID we recognize for CP210x
    }

    // CP210x devices typically have one configuration.
    // We need to find the interface with bulk IN and OUT endpoints.
    // Interface number `ifcnum` passed in could be a hint or specific request.
    // For single-port CP210x, it's almost always interface 0.

    libusb_config_descriptor *config_desc = nullptr;
    int ret = libusb_get_active_config_descriptor(dev, &config_desc);
    if (ret < 0 || !config_desc) {
        log.e(__, "CP210x: Failed to get active config descriptor: %s", libusb_error_name(ret));
        return nullptr;
    }

    const libusb_interface* interface = nullptr;
    const libusb_interface_descriptor* interface_desc = nullptr;
    ifc_desc drv_ifc_desc; // usbuart's internal ifc_desc struct

    bool found_interface = false;
    for (int i = 0; i < config_desc->bNumInterfaces; ++i) {
        interface = &config_desc->interface[i];
        if (interface->num_altsetting > 0) { // Should have at least one alt setting
            interface_desc = &interface->altsetting[0]; // Use the first alt setting

            // Check if this interface matches the requested ifcnum
            // AND if it looks like a serial interface (bulk in/out)
            if (interface_desc->bInterfaceNumber == ifcnum) {
                if (interface_desc->bInterfaceClass == LIBUSB_CLASS_VENDOR_SPEC ||
                    interface_desc->bInterfaceClass == 0x00 /* Some might use class 0 */ ) {
                    
                    uint8_t ep_in = 0, ep_out = 0;
                    uint16_t in_max_packet = 0, out_max_packet = 0;

                    for (int k = 0; k < interface_desc->bNumEndpoints; ++k) {
                        const libusb_endpoint_descriptor* ep = &interface_desc->endpoint[k];
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
                        drv_ifc_desc.id = interface_desc->bInterfaceNumber;
                        drv_ifc_desc.ep_bulk_in = ep_in;
                        drv_ifc_desc.ep_bulk_out = ep_out;
                        // CP210x usually has 64-byte max packet size for full speed.
                        // High speed variants might differ.
                        drv_ifc_desc.chunk_size = std::min(in_max_packet, out_max_packet); 
                        if (drv_ifc_desc.chunk_size == 0) drv_ifc_desc.chunk_size = 64; // Default
                        
                        found_interface = true;
                        log.i(__, "CP210x: Found matching interface %d, EP IN:0x%02X, EP OUT:0x%02X, Chunk:%d",
                              drv_ifc_desc.id, drv_ifc_desc.ep_bulk_in, drv_ifc_desc.ep_bulk_out, drv_ifc_desc.chunk_size);
                        break; 
                    }
                }
            }
        }
        if (found_interface) break;
    }

    libusb_free_config_descriptor(config_desc);

    if (!found_interface) {
        log.w(__, "CP210x: Could not find a suitable bulk IN/OUT interface for ifcnum %d", ifcnum);
        return nullptr;
    }
    
    // Try to claim the interface
    ret = libusb_claim_interface(devh, drv_ifc_desc.id);
    if (ret < 0) {
        log.e(__,"CP210x: libusb_claim_interface failed for ifc %d: %s", drv_ifc_desc.id, libusb_error_name(ret));
        // Check if kernel driver is active and try to detach (optional, needs careful handling)
        // if (libusb_kernel_driver_active(devh, drv_ifc_desc.id) == 1) {
        //     log.i(__, "CP210x: Kernel driver active on interface %d. Detaching...", drv_ifc_desc.id);
        //     if (libusb_detach_kernel_driver(devh, drv_ifc_desc.id) == 0) {
        //         log.i(__, "CP210x: Kernel driver detached. Retrying claim...");
        //         ret = libusb_claim_interface(devh, drv_ifc_desc.id);
        //         if (ret < 0) {
        //            log.e(__,"CP210x: libusb_claim_interface failed after detach: %s", libusb_error_name(ret));
        //            return nullptr;
        //         }
        //     } else {
        //         log.e(__, "CP210x: Could not detach kernel driver.");
        //         return nullptr;
        //     }
        // } else {
             return nullptr; // Claim failed, not kernel driver issue or detach failed
        // }
    }
    log.i(__, "CP210x: Interface %d claimed successfully", drv_ifc_desc.id);

    // If we reached here, the device is a CP210x we support and interface is claimed.
    // Initial parameters (can be default, will be set by setup())
    eia_tia_232_info initial_pi = {115200, 8, parity_t::none, stop_bits_t::one, flow_control_t::none_};
    
    cp210x* instance = nullptr;
    try {
        instance = new cp210x(devh, drv_ifc_desc, initial_pi);
    } catch (const error_t& err) {
        log.e(__, "CP210x: Constructor failed with error_t %d", +err);
        libusb_release_interface(devh, drv_ifc_desc.id); // Release claimed interface
        return nullptr;
    } catch (const std::exception& e) {
        log.e(__, "CP210x: Constructor failed with exception: %s", e.what());
        libusb_release_interface(devh, drv_ifc_desc.id);
        return nullptr;
    }
    
    return instance;
}


cp210x::cp210x(libusb_device_handle* dev, const ifc_desc& ifc, const eia_tia_232_info& pi)
    : driver(dev, ifc), current_params(pi), interface_number(ifc.id) {
    log.i(__, "CP210x driver instance created for interface %d", interface_number);
    // Initial setup is typically done via the `setup` call from the context
}

cp210x::~cp210x() {
    log.i(__, "CP210x driver instance for interface %d destroying", interface_number);
    try {
        set_uart_enable(false); // Try to disable UART on close
    } catch (const error_t& e) {
        log.w(__, "CP210x: Failed to disable UART on close: %d", +e);
    } catch (...) {
        log.w(__, "CP210x: Unknown error disabling UART on close");
    }

    int ret = libusb_release_interface(devh, interface_number);
    if (ret < 0) {
        log.e(__,"CP210x: libusb_release_interface failed for ifc %d: %s", interface_number, libusb_error_name(ret));
    } else {
        log.i(__,"CP210x: Interface %d released", interface_number);
    }
}

int cp210x::ctrl_transfer(uint8_t bmRequestType, uint8_t bRequest,
                           uint16_t wValue, uint16_t wIndex,
                           unsigned char* data, uint16_t wLength,
                           unsigned int timeout) throw(error_t) {
    // Ensure wIndex includes the interface number as per some CP210x docs
    // High byte of wIndex for interface number for some requests.
    // For others, it's just wIndex. Here we use the passed wIndex directly,
    // assuming caller sets it correctly (e.g., `this->interface_number` for simple cases).
    int ret = libusb_control_transfer(devh, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout);
    if (ret < 0) {
        log.e(__, "CP210x: Control transfer failed (req:0x%02X, val:0x%04X, idx:0x%04X): %s (%d)",
              bRequest, wValue, wIndex, libusb_error_name(ret), ret);
        if (ret == LIBUSB_ERROR_PIPE) throw error_t::control_error; // Stall
        if (ret == LIBUSB_ERROR_TIMEOUT) throw error_t::usb_error; // Timeout
        if (ret == LIBUSB_ERROR_NO_DEVICE) throw error_t::no_device;
        throw error_t::control_error; // Generic control error
    }
    if (ret != wLength && (bmRequestType & LIBUSB_ENDPOINT_IN)) {
        log.w(__, "CP210x: Control transfer short read (req:0x%02X): expected %d, got %d", bRequest, wLength, ret);
        // This might not always be an error, depends on the request.
    }
    return ret;
}

void cp210x::set_uart_enable(bool enable) throw(error_t) {
    log.d(__, "CP210x: Setting UART enable to %d on ifc %d", enable, interface_number);
    ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, CP210X_IFC_ENABLE,
                  enable ? CP210X_UART_ENABLE : CP210X_UART_DISABLE,
                  this->interface_number, nullptr, 0, 1000);
}

void cp210x::set_baudrate(uint32_t baudrate) throw(error_t) {
    log.d(__, "CP210x: Setting baudrate to %u on ifc %d", baudrate, interface_number);
    // CP210x baud rate is set with a 4-byte little-endian value in the data stage
    unsigned char baud_data[4];
    baud_data[0] = baudrate & 0xFF;
    baud_data[1] = (baudrate >> 8) & 0xFF;
    baud_data[2] = (baudrate >> 16) & 0xFF;
    baud_data[3] = (baudrate >> 24) & 0xFF;

    ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, CP210X_SET_BAUDRATE,
                  0x0000, // wValue is not used directly for baud rate value for this req
                  this->interface_number, // wIndex specifies the interface
                  baud_data, sizeof(baud_data), 1000);
}

void cp210x::set_line_control(const eia_tia_232_info& pi) throw(error_t) {
    uint16_t line_ctl_word = 0;

    // Data bits (bits 8-11)
    if (pi.databits >= 5 && pi.databits <= 8) {
        line_ctl_word |= (pi.databits << 8);
    } else {
        log.e(__, "CP210x: Invalid data bits: %d", pi.databits);
        throw error_t::invalid_param;
    }

    // Parity (bits 2-4)
    switch (pi.parity) {
        case parity_t::none: line_ctl_word |= CP210X_LINE_CTL_PARITY_NONE; break;
        case parity_t::odd:  line_ctl_word |= CP210X_LINE_CTL_PARITY_ODD;  break;
        case parity_t::even: line_ctl_word |= CP210X_LINE_CTL_PARITY_EVEN; break;
        case parity_t::mark: line_ctl_word |= CP210X_LINE_CTL_PARITY_MARK; break;
        case parity_t::space:line_ctl_word |= CP210X_LINE_CTL_PARITY_SPACE;break;
        default: throw error_t::invalid_param;
    }

    // Stop bits (bits 0-1)
    switch (pi.stopbits) {
        case stop_bits_t::one:  line_ctl_word |= CP210X_LINE_CTL_STOP_BITS_1;   break;
        case stop_bits_t::_1_5: line_ctl_word |= CP210X_LINE_CTL_STOP_BITS_1_5; break; // CP210x supports 1.5 for 5 data bits
        case stop_bits_t::two:  line_ctl_word |= CP210X_LINE_CTL_STOP_BITS_2;   break;
        default: throw error_t::invalid_param;
    }
    log.d(__, "CP210x: Setting line control to 0x%04X (D:%d P:%d S:%d) on ifc %d",
          line_ctl_word, pi.databits, static_cast<int>(pi.parity), static_cast<int>(pi.stopbits), interface_number);

    ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, CP210X_SET_LINE_CTL,
                  line_ctl_word, this->interface_number, nullptr, 0, 1000);
}

void cp210x::set_mhs(bool dtr, bool rts) throw(error_t) {
    uint16_t mhs_val = 0;
    // Bit 0: DTR state, Bit 1: DTR mask (1=change)
    // Bit 8: RTS state, Bit 9: RTS mask (1=change)
    mhs_val |= (dtr ? 0x0001 : 0x0000); // DTR value
    mhs_val |= 0x0002;                  // DTR mask (always change)

    mhs_val |= (rts ? 0x0100 : 0x0000); // RTS value
    mhs_val |= 0x0200;                  // RTS mask (always change)

    log.d(__, "CP210x: Setting MHS to 0x%04X (DTR:%d RTS:%d) on ifc %d", mhs_val, dtr, rts, interface_number);
    ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, CP210X_SET_MHS,
                  mhs_val, this->interface_number, nullptr, 0, 1000);
}


void cp210x::set_flow_control(const eia_tia_232_info& pi) throw(error_t) {
    // This is a simplified implementation. CP210X_SET_FLOW is complex.
    // We'll primarily focus on enabling/disabling RTS/CTS.
    // DTR/DSR flow control is also possible but less common for basic bridging.
    // XON/XOFF needs XON/XOFF char settings (CP210X_SET_XON, CP210X_SET_XOFF).

    // Data for CP210X_SET_FLOW (8 bytes)
    // struct {
    //    uint32_t ulControlHandshake; // dword 0
    //    uint32_t ulFlowReplace;      // dword 1
    //    uint16_t usXonLimit;         // word 4
    //    uint16_t usXoffLimit;        // word 5
    // } CP210X_FLOW_CONTROL_CONFIG;
    // AN571 is key here.
    // For RTS_CTS:
    // ulControlHandshake:
    //   Bit 0: DTR Control (0=ignore, 1=CP210x controlled by DTR state)
    //   Bit 1: DSR Handshake (0=ignore, 1=TX stops if DSR inactive)
    //   Bit 2: DSR Sensitivity (0=ignore DSR, 1=DSR state affects DCD output)
    //   Bit 3: CTS Handshake (0=ignore, 1=TX stops if CTS inactive) <--- This one
    //   Bit 4: RTS Control (0=manual, 1=flow control by CP210x) <--- This one
    //   Bit 5: RTS State (0=inactive, 1=active) - only if RTS Control is manual
    // ulFlowReplace:
    //   Bit 0: XON/XOFF TX (0=disable, 1=enable)
    //   Bit 1: XON/XOFF RX (0=disable, 1=enable)
    //   ... and others for error/break char replacement

    unsigned char flow_data[8];
    memset(flow_data, 0, sizeof(flow_data)); // Initialize to all zeros (mostly disabled)

    uint32_t& ulControlHandshake = *reinterpret_cast<uint32_t*>(&flow_data[0]);
    uint32_t& ulFlowReplace = *reinterpret_cast<uint32_t*>(&flow_data[4]);
    // uint16_t& usXonLimit = *reinterpret_cast<uint16_t*>(&flow_data[8]); // if wLength was 12
    // uint16_t& usXoffLimit = *reinterpret_cast<uint16_t*>(&flow_data[10]);

    bool dtr_active = true; // Default DTR to active
    bool rts_active_manual = true; // Default RTS to active if manual

    switch (pi.flowcontrol) {
        case flow_control_t::none_:
            log.d(__, "CP210x: Flow control: None");
            // DTR active, RTS active (manual)
            ulControlHandshake = 0; // All handshake/control off by default
            // We still need to set DTR/RTS lines explicitly if needed
            // set_mhs(true, true); // Keep lines asserted by default
            break;
        case flow_control_t::rts_cts:
            log.d(__, "CP210x: Flow control: RTS/CTS");
            ulControlHandshake |= (1 << 3); // CTS Handshake (TX stops if CTS inactive)
            ulControlHandshake |= (1 << 4); // RTS Control (flow control by CP210x)
            // dtr_active remains true
            // rts_active_manual is irrelevant as RTS is now auto
            break;
        case flow_control_t::dtr_dsr:
            log.d(__, "CP210x: Flow control: DTR/DSR - (Partially supported, DSR input only)");
            // CP210x can stop TX if DSR inactive
            ulControlHandshake |= (1 << 1); // DSR Handshake (TX stops if DSR inactive)
            // DTR is an output, DSR an input.
            // DTR can be controlled manually by host.
            // rts_active_manual remains true
            break;
        case flow_control_t::xon_xoff:
            log.d(__, "CP210x: Flow control: XON/XOFF - (Needs XON/XOFF char setup)");
            // Requires setting XON (0x09) and XOFF (0x0A) chars first.
            // For simplicity, we only enable the flags here.
            ulFlowReplace |= (1 << 0); // XON/XOFF TX enabled
            ulFlowReplace |= (1 << 1); // XON/XOFF RX enabled
            // Typically, XON = 0x11 (DC1), XOFF = 0x13 (DC3)
            // ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, CP210X_SET_XON, 0x0011, this->interface_number, nullptr, 0, 1000);
            // ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, CP210X_SET_XOFF, 0x0013, this->interface_number, nullptr, 0, 1000);
            // Note: XonLimit and XoffLimit are part of a larger structure for SET_CHARS or GET_CHARS,
            // not directly in SET_FLOW's 8-byte payload. This basic setup might be insufficient.
            break;
        default:
            throw error_t::invalid_param;
    }

    log.d(__, "CP210x: Setting flow control (data: 0x%08X %08X) on ifc %d", ulControlHandshake, ulFlowReplace, interface_number);
    ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, CP210X_SET_FLOW,
                  0x0000, // wValue typically 0 for this
                  this->interface_number, flow_data, sizeof(flow_data), 1000);
    
    // After setting flow control, set DTR/RTS lines appropriately
    // If RTS is manual (not rts_cts), set it. DTR is always manual from host perspective unless DTR flow control is used.
    bool rts_is_manual = !(pi.flowcontrol == flow_control_t::rts_cts);
    set_mhs(dtr_active, rts_is_manual ? rts_active_manual : false /* RTS handled by chip in flow control mode */);

}


void cp210x::purge_fifos(uint16_t direction) throw(error_t) {
    log.d(__, "CP210x: Purging FIFOs (direction: 0x%04X) on ifc %d", direction, interface_number);
    ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, CP210X_PURGE,
                  direction, this->interface_number, nullptr, 0, 1000);
}


void cp210x::setup(const eia_tia_232_info& pi) throw(error_t) {
    log.i(__, "CP210x: Setup called for ifc %d (Baud:%u, Data:%d, P:%d, S:%d, F:%d)",
        interface_number, pi.baudrate, pi.databits, static_cast<int>(pi.parity),
        static_cast<int>(pi.stopbits), static_cast<int>(pi.flowcontrol));

    // Sequence from AN571:
    // 1. IFC_ENABLE (Enable UART)
    // 2. SET_BAUDRATE
    // 3. SET_LINE_CTL
    // 4. SET_FLOW_CONTROL (includes MHS if needed, or set MHS separately)
    // (Potentially PURGE)

    set_uart_enable(true); // Enable the UART interface
    set_baudrate(pi.baudrate);
    set_line_control(pi);
    set_flow_control(pi); // This will also handle basic DTR/RTS line states

    // Optionally purge FIFOs after setup
    purge_fifos(CP210X_PURGE_RX | CP210X_PURGE_TX);

    current_params = pi; // Store current settings
    log.i(__, "CP210x: Setup complete for ifc %d", interface_number);
}

void cp210x::reset() throw(error_t) {
    log.i(__, "CP210x: Reset called for ifc %d", interface_number);
    // CP210x doesn't have a specific "UART reset" command like some other chips.
    // Re-applying settings and purging FIFOs is the closest.
    // Some interpretations might involve toggling UART_ENABLE.
    set_uart_enable(false);
    purge_fifos(CP210X_PURGE_RX | CP210X_PURGE_TX); // Purge while disabled
    // Small delay might be good here if supported by usbuart async model
    setup(current_params); // Re-apply current parameters, which includes enabling UART
}

void cp210x::sendbreak(bool on) throw(error_t) {
    // CP210x AN571 mentions SET_BREAK (0x05)
    // bRequest = 0x05 (CP210X_SET_BREAK)
    // wValue = 0x0001 (Break ON), 0x0000 (Break OFF)
    // wIndex = Interface
    log.i(__, "CP210x: Send break %s on ifc %d", (on ? "ON" : "OFF"), interface_number);
    ctrl_transfer(CP210X_REQTYPE_HOST_TO_DEVICE, 0x05 /*CP210X_SET_BREAK*/,
                  on ? 0x0001 : 0x0000,
                  this->interface_number, nullptr, 0, 1000);
}

// Default read_callback and write_callback from base driver class are usually sufficient
// as CP210x uses straightforward bulk transfers without special framing in the data stream.

} // namespace driver
} // namespace usbuart
