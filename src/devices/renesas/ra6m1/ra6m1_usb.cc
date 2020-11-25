/**
 * Copyright 2018 Nickolas T Lloyd <ultrageek.lloyd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ra6m1.hh"

#include "mm.hh"

void ra6m1_usb_set_device_mode()
{
    // 29.2.1: The DCFM bit selects the host or device function of the USBFS.
    // Only change this bit when the DPRPU and DRPD bits are both 0.
    ra6m1_usb_detach__();
    ra6m1_usb_set_device_mode__();
}

void ra6m1_usb_set_host_mode()
{
    // 29.2.1: The DCFM bit selects the host or device function of the USBFS.
    // Only change this bit when the DPRPU and DRPD bits are both 0.
    ra6m1_usb_detach__();
    ra6m1_usb_set_host_mode__();
}

bool ra6m1_enable_usb_device()
{
    if (!ra6m1_usb_enable_usb_clock())
    {
	return false;
    }
    ra6m1_usb_enable_usb__();
    ra6m1_usb_set_device_mode();
    return true;
}

bool ra6m1_usb_is_connected()
{
    return ra6m1_usb_is_connected__();
}

bool ra6m1_usb_attach()
{
    // 29.3.1.3: In device controller mode, confirm that connection to the USB host is made, and then
    // set the SYSCFG.DPRPU bit to 1 and pull up the D+ line.
    if (ra6m1_usb_is_device_mode__() && !ra6m1_usb_is_connected())
    {
	return false;
    }
    ra6m1_usb_attach__();
    return true;
}

void ra6m1_usb_detach()
{
    ra6m1_usb_detach__();
}

void ra6m1_usb_cfifo_select__(
    unsigned char pipe_number,
    bool write_dir,
    bool big_endian,
    bool wide_access,
    bool read_count_mode)
{
    Reg16 cfifosel{ USBFS_CFIFOSEL };
    unsigned short value =
	pipe_number
	| (write_dir ? USBFS_CFIFOSEL_ISEL : 0)
	| (big_endian ? USBFS_FIFOSEL_BIGEND : 0)
	| (wide_access ? USBFS_FIFOSEL_MBW : 0)
	| (read_count_mode ? USBFS_FIFOSEL_RCNT : 0);
    cfifosel = value;
    while ((cfifosel & USBFS_FIFOSEL_CURPIPE_MASK) != pipe_number)
    {
	cfifosel = value;
    }
}

void ra6m1_usb_cfifo_select(
    unsigned char pipe_number,
    bool write_dir,
    bool big_endian,
    bool wide_access,
    bool buffer_pointer_rewind,
    bool read_count_mode)
{
    Reg16 cfifosel{ USBFS_CFIFOSEL };
    ra6m1_usb_cfifo_select__(
	pipe_number,
	write_dir,
	big_endian,
	wide_access,
	read_count_mode);
    while (!ra6m1_usb_cfifo_port_ready__());
    if (buffer_pointer_rewind)
    {
	cfifosel |= USBFS_FIFOSEL_REW;
    }
}

void ra6m1_usb_dfifo_select(
    bool dNfifo,
    unsigned char pipe_number,
    bool big_endian,
    bool wide_access,
    bool buffer_pointer_rewind,
    bool read_count_mode,
    bool transfer_req_enabled,
    bool auto_buffer_clear)
{
    Reg16 dfifosel{ USBFS_DFIFOSEL(dNfifo) };
}

void ra6m1_usb_set_pipe_pid(
    unsigned char pipe_number,
    unsigned char pid)
{
    using namespace UsbPid;
    if (pipe_number == 0)
    {
	ra6m1_usb_set_dcp_pid(pid);
    }
    else
    {
	Reg16 pipectr{ USBFS_PIPECTR(pipe_number) };
	unsigned short temp = pipectr;
	unsigned char current_pid = temp & USBFS_PIPECTR_PID_MASK;
	temp &= ~USBFS_PIPECTR_PID_MASK;
	if (current_pid == NAK && pid == STALL)
	{
	    pipectr = temp | STALL0;
	}
	else if (current_pid == BUF && pid == STALL)
	{
	    pipectr = temp | STALL1;
	}
	else if (current_pid == STALL1 && pid == NAK)
	{
	    pipectr = temp | STALL0;
	    pipectr = temp | NAK;
	}
	else if ((current_pid == STALL0 || current_pid == STALL1) && pid == BUF)
	{
	    pipectr = temp | NAK;
	    pipectr = temp | BUF;
	}
	else
	{
	    pipectr = temp | pid;
	}
    }
}

void ra6m1_usb_cfifo_select_pipe(
    unsigned char pipe_number,
    bool write_dir)
{
    ra6m1_usb_cfifo_select__(
	pipe_number,
	write_dir,
	false,
	false,
	false);
}

bool ra6m1_usb_cfifo_ready(
    unsigned char pipe_number,
    bool write_dir)
{
    ra6m1_usb_cfifo_select_pipe(pipe_number, write_dir);
    return ra6m1_usb_cfifo_port_ready__();
}

void ra6m1_usb_cfifo_clear(
    unsigned char pipe_number,
    bool write_dir)
{
    ra6m1_usb_cfifo_select_pipe(pipe_number, write_dir);
    if (pipe_number != 0)
    {
	// Wait for pipe to become ready
	while (!ra6m1_usb_cfifo_ready(pipe_number, write_dir))
	{
	    ra6m1_usb_nak_pipe(pipe_number);
	    ra6m1_usb_cfifo_clear(pipe_number, write_dir);
	}
    }
    else {
	ra6m1_usb_cfifo_select_pipe(pipe_number, write_dir);
	ra6m1_usb_cfifo_clear__();
    }
}

unsigned short ra6m1_usb_cfifo_rx_size(unsigned char pipe_number)
{
    ra6m1_usb_cfifo_select_pipe(pipe_number, false);
    return ra6m1_usb_cfifo_dtln__();
}

char* ra6m1_usb_cfifo_read(
    unsigned char pipe_number,
    unsigned short& size)
{
    using namespace UsbPid;
    while (!ra6m1_usb_cfifo_ready(pipe_number, false));
    size = ra6m1_usb_cfifo_dtln__();
    if (size == 0)
    {
	// 29.3.7:
	// Data cannot be read when a zero-length packet is received, so use the BCLR bit to clear the
	// buffer.
	ra6m1_usb_cfifo_clear__();
	// Enable reception of next packet.
	ra6m1_usb_set_pipe_pid(pipe_number, BUF);
	// TODO: is this the right thing to do?
	return nullptr;
    }
    else
    {
	char* buffer = new(std::nothrow) char[size];
	if (buffer == nullptr)
	{
	    return nullptr;
	}
	// Read one byte at a time from the FIFO, MBW=0
	for (unsigned int i = 0; i < size; i++)
	{
	    buffer[i] = ra6m1_usb_cfifo_read_byte__();
	}
	// Enable reception of next packet.
	ra6m1_usb_set_pipe_pid(pipe_number, BUF);
	return buffer;
    }
}

void ra6m1_usb_cfifo_write(
    unsigned char pipe_number,
    unsigned short size,
    const char* buffer)
{
    using namespace UsbPid;
    while (!ra6m1_usb_cfifo_ready(pipe_number, true));
    // Enable pipe for transmit.
    ra6m1_usb_set_pipe_pid(pipe_number, BUF);
    for (unsigned int i = 0; i < size; i++)
    {
	ra6m1_usb_cfifo_write_byte__(buffer[i]);
    }
}

void ra6m1_usb_cfifo_write_short_packet(
    unsigned char pipe_number,
    unsigned short size,
    const char* buffer)
{
    ra6m1_usb_cfifo_write(pipe_number, size, buffer);
    // Set BVAL to transmit packet
    ra6m1_usb_cfifo_send_short_packet();
}

void ra6m1_usb_cfifo_write_zlp(unsigned char pipe_number)
{
    using namespace UsbPid;
    while (!ra6m1_usb_cfifo_ready(pipe_number, true));
    // Enable pipe for transmit.
    ra6m1_usb_set_pipe_pid(pipe_number, BUF);
    ra6m1_usb_cfifo_send_zlp();
}

void ra6m1_usb_dcp_complete_transaction()
{
    Reg16 dcpctr{ USBFS_DCPCTR };
    ra6m1_usb_set_dcp_pid(UsbPid::BUF);
    dcpctr |= USBFS_DCPCTR_CCPL;
}

void ra6m1_usb_configure_dcp(
    bool auto_nak,
    unsigned char max_packet_size)
{
    // 29.3.4.1: Do not set the following bits when the CURPIPE bits are set.  These bits in the pipe
    // control registers can only be changed when the selected pipe information is not set in the
    // CURPIPE bits in CFIFOSEL, D0FIFOSEL, and D1FIFOSEL:
    // - Bits in the DCPCFG and DCPMAXP
    // - Bits in PIPECFG, PIPEMAXP and PIPEPERI.
    // To change pipe information, you must set the CURPIPE bits in the port select register to a pipie
    // other than the one to be changed.  For the DCP, the buffer must be cleared using the BCLR bit in
    // the port control register after the pipe information is changed.
    ra6m1_usb_nak_dcp();
    while (ra6m1_usb_dcp_is_busy());
    // Sets ISEL to receiving direction
    ra6m1_usb_cfifo_select_pipe(1, false);
    ra6m1_usb_set_dcp_max_packet_size__(max_packet_size);
    // 29.2.26:
    // In device controller mode, set these bits to 0.
    if (ra6m1_usb_is_device_mode__())
    {
	ra6m1_usb_set_dcp_devsel__(0);
    }
    Reg16 dcpcfg{ USBFS_DCPCFG };
    // Sets DIR to receiving direction
    dcpcfg = auto_nak ? USBFS_DCPCFG_SHTNAK : 0;
    // Sets ISEL to receiving direction
    ra6m1_usb_cfifo_clear(0, false);
    ra6m1_usb_cfifo_clear(0, true);
    // Enable BRDY and BEMP interrupts on DCP.
    Reg16 bempenb{ USBFS_BEMPENB };
    Reg16 brdyenb{ USBFS_BRDYENB };
    bempenb |= 1U;
    brdyenb |= 1U;
}

void ra6m1_usb_configure_pipe(
    unsigned char pipe_number,
    unsigned char endpoint_number,
    bool transmit_pipe,
    bool auto_nak,
    bool double_buffer,
    bool brdy_each_transaction,
    unsigned char transfer_type,
    unsigned short max_packet_size)
{
    // 29.3.4.1
    ra6m1_usb_nak_pipe(pipe_number);
    while (ra6m1_usb_pipe_is_busy(pipe_number));
    // 29.2.29 note 3
    ra6m1_usb_pipe_auto_clear__(pipe_number);
    ra6m1_usb_cfifo_select_pipe(0, false);
    Reg16 pipecfg{ USBFS_PIPECFG };
    pipecfg =
	endpoint_number
	| (transmit_pipe ? USBFS_PIPECFG_DIR : 0)
	| (auto_nak ? USBFS_PIPECFG_SHTNAK : 0)
	| (double_buffer ? USBFS_PIPECFG_DBLB : 0)
	| (!brdy_each_transaction ? USBFS_PIPECFG_BFRE : 0)
	| (transfer_type << USBFS_PIPECFG_TYPE_SHIFT);
    ra6m1_usb_set_pipe_max_packet_size__(
	pipe_number,
	max_packet_size);
    // 29.2.30:
    // In device controller mode, set these bits to 0.
    if (ra6m1_usb_is_device_mode__())
    {
	ra6m1_usb_set_pipe_devsel__(0);
    }
    // If this is a transmit pipe, enable BEMP interrupt.
    // For receive pipes, enable BRDY interrupt.
    Reg16 bempenb{ USBFS_BEMPENB };
    Reg16 brdyenb{ USBFS_BRDYENB };
    if (transmit_pipe)
    {
	bempenb |= (1U << pipe_number);
	brdyenb &= (unsigned short)~(1U << pipe_number);
    }
    else
    {
	brdyenb |= (1U << pipe_number);
	bempenb &= (unsigned short)~(1U << pipe_number);
    }
    // set PID to BUF to enable pipe.
    ra6m1_usb_set_pipe_pid(pipe_number, UsbPid::BUF);
}

bool ra6m1_usb_pipe_buffer_ready(unsigned char pipe_number)
{
    Reg16 pipectr{ (pipe_number == 0) ? USBFS_DCPCTR : USBFS_PIPECTR(pipe_number) };
    return (pipectr & USBFS_PIPECTR_BSTS) != 0;
}

void ra6m1_usb_control_transfer(unsigned char transfer_stage)
{
    Reg16 intsts0{ USBFS_INTSTS0 };
    unsigned short flags = *intsts0;
    switch (transfer_stage)
    {
	using namespace UsbControlTransferStage;
    case ControlReadDataStage:
	if (flags & USBFS_INTSTS0_VALID)
	{
	    usb_setup_token(0);
	}
	break;
    case ControlWriteDataStage:
    case ControlWriteNoDataStatusStage:
	if (flags & USBFS_INTSTS0_VALID)
	{
	    usb_setup_token(0);
	}
	break;
    case IdleOrSetupStage:
    case ControlReadStatusStage:
    case ControlWriteStatusStage:
    case ControlTransferSequenceError:
    default:
	break;
    }
}


extern "C"
{

    void ra6m1_usb_set_address(const unsigned int)
    {
	// Automatically handled by USBFS
    }

    void ra6m1_usb_set_configured(const bool)
    {
	
    }

    void ra6m1_usb_init_usb()
    {
	// 29.4.1
	// Enable USBFS module
	Reg32 mstpcrb{ (0x40047000UL) };
	mstpcrb &= ~(1U << 11);
	// 29.3.1
	// Set to device controller mode.
	while (!ra6m1_enable_usb_device());
	// Assign I/O lines to USBFS.
	// USB_VBUS
	pin_set_peripheral(4, 7, 0b10011);
	// 29.2.36: The PHYSLEW register adjusts the cross point of the driver.
	// In both host and device controller modes, set this register before operating the controller.
	ra6m1_usb_set_phy_cross_point();
	// Configure interrupts
	// 29.4.3: Clear INTSTS0 and INTSTS1 registers
	Reg16 intsts0{ USBFS_INTSTS0 };
	intsts0 = 0;
	Reg16 intsts1{ USBFS_INTSTS1 };
	intsts1 = 0;
	// Disable interrupts on all pipes until they are individually configured.
	Reg16 brdyenb{ USBFS_BRDYENB };
	brdyenb = 0;
	Reg16 nrdyenb{ USBFS_NRDYENB };
	nrdyenb = 0;
	Reg16 bempenb{ USBFS_BEMPENB };
	bempenb = 0;
	// Enable the interrupts we care about.
	Reg16 intenb0{ USBFS_INTENB0 };
	intenb0 =
	    USBFS_INTENB0_BRDYE
	    | USBFS_INTENB0_BEMPE
	    | USBFS_INTENB0_CTRE
	    | USBFS_INTENB0_DVSE
	    | USBFS_INTENB0_RSME;
	icu_link_event_to_irq(0x61U, 0);
    }

    void ra6m1_usb_ep_isr(unsigned int)
    {

    }

    void ra6m1_usb_enter_default_state()
    {

    }

    void ra6m1_usb_enable()
    {
	while (!ra6m1_usb_attach());
    }
}
