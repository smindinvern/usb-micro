#ifndef RA6M1_USB_HH_
#define RA6M1_USB_HH_

#include "Registers.hh"
#include "arm.hh"
#include "usb.hh"

#define RA6M1_USBFS_BASE        (0x40090000UL)

// 29.2.1: System Configuration Register
#define USBFS_SYSCFG            RA6M1_USBFS_BASE
// USBFS Operation Enable
#define USBFS_SYSCFG_USBE       (1U << 0)
// D+ Line Resistor Control
#define USBFS_SYSCFG_DPRPU      (1U << 4)
// D+/D- Line Resistor Control
#define USBFS_SYSCFG_DRPD       (1U << 5)
// Controller Function Select
#define USBFS_SYSCFG_DCFM       (1U << 6)
#define USBFS_SYSCFG_HOST       USBFS_SYSCFG_DCFM
// USB Clock Enable
// Note: After writing 1 to the SCKE bit, read it to confirm that it is set to 1.
#define USBFS_SYSCFG_SCKE       (1U << 10)

inline bool ra6m1_usb_enable_usb_clock()
{
    Reg16 syscfg{ USBFS_SYSCFG };
    syscfg |= USBFS_SYSCFG_SCKE;
    return (syscfg & USBFS_SYSCFG_SCKE) != 0;
}

inline void ra6m1_usb_enable_usb__()
{
    Reg16 syscfg{ USBFS_SYSCFG };
    syscfg |= USBFS_SYSCFG_USBE;
}

inline void ra6m1_usb_attach__()
{
    Reg16 syscfg{ USBFS_SYSCFG };
    syscfg |= USBFS_SYSCFG_DPRPU;
}

inline void ra6m1_usb_detach__()
{
    Reg16 syscfg{ USBFS_SYSCFG };
    syscfg &= (unsigned short)~(USBFS_SYSCFG_DPRPU | USBFS_SYSCFG_DRPD);
}

inline void ra6m1_usb_set_device_mode__()
{
    Reg16 syscfg{ USBFS_SYSCFG };
    syscfg &= (unsigned short)~USBFS_SYSCFG_HOST;
}

inline void ra6m1_usb_set_host_mode__()
{
    Reg16 syscfg{ USBFS_SYSCFG };
    unsigned short temp = syscfg;
    temp |= USBFS_SYSCFG_HOST;
    syscfg = temp;
    temp |= USBFS_SYSCFG_DRPD;
    syscfg = temp;
}

inline bool ra6m1_usb_is_host_mode__()
{
    Reg16 syscfg{ USBFS_SYSCFG };
    return (syscfg & USBFS_SYSCFG_HOST) != 0;
}

inline bool ra6m1_usb_is_device_mode__()
{
    return !ra6m1_usb_is_host_mode__();
}

void ra6m1_usb_set_device_mode();
void ra6m1_usb_set_host_mode();
bool ra6m1_enable_usb_device();

// 29.2.2: System Configuration Status Register 0
#define USBFS_SYSSTS0         (RA6M1_USBFS_BASE + 0x4UL)
// USB Data Line Status Monitor
namespace UsbDataLineStatus
{
    enum
    {
	SE0     = 0b00,
	J_State = 0b01,
	K_State = 0b10,
	SE1     = 0b11
    };
}

// External ID0 Input Pin Monitor
#define USBFS_SYSSTS0_IDMON  (1U << 2)
// Active Monitor When the Host Controller Is Selected
#define USBFS_SYSSTS0_SOFEA  (1U << 5)
// USB Host Sequencer Status Monitor
#define USBFS_SYSSTS00_HTACT (1U << 6)
// External USB_OVRCURA/USB_OVRCURB Input Pin Monitor
namespace UsbOverCurrentMonitorStatus
{
    enum
    {
	UsbOverCurrentA = (1U << 15),
	UsbOverCurrentB = (1U << 14)
    };
}

inline bool ra6m1_usb_is_over_current()
{
    using namespace UsbOverCurrentMonitorStatus;
    Reg16 syssts0{ USBFS_SYSSTS0 };
    return (syssts0 & (UsbOverCurrentA | UsbOverCurrentB)) != 0;
}

// 29.2.3: Device State Control Register 0
#define USBFS_DVSTCTR0  (RA6M1_USBFS_BASE + 0x8UL)
// USB Bus Reset Status
namespace UsbHostBusResetStatus
{
    enum
    {
	Indeterminate = 0b000,
	LowSpeed      = 0b001,
	FullSpeed     = 0b010
    };

#define USBFS_DVSTCTR0_RESET_BIT  (1U << 2)
    
    inline bool reset_in_progress()
    {
	Reg16 dvstctr0{ USBFS_DVSTCTR0 };
	return (dvstctr0 & USBFS_DVSTCTR0_RESET_BIT) != 0;
    }

#undef USBFS_DVSTCTR0_RESET_BIT
}

namespace UsbDeviceBusResetStatus
{
    enum
    {
	Indeterminate              = 0b000,
	ResetInProgress            = 0b001,
	ResetInProgressOrFullSpeed = 0b010
    };

    inline bool reset_in_progress()
    {
	Reg16 dvstctr0{ USBFS_DVSTCTR0 };
	return (dvstctr0 & (ResetInProgress | ResetInProgressOrFullSpeed)) != 0;
    }
}

// USB Bus Enable
// In device controller mode, always set this bit to 0.
#define USBFS_DVSTCTR0_UACT    (1U << 4)
// Resume Output
// Always set this bit to 0 in device controller mode.
#define USBFS_DVSTCTR0_RESUME  (1U << 5)
// USB Bus Reset Output
// Always set this bit to 0 in device controller mode.
#define USBFS_DVSTCTR0_USBRST  (1U << 6)
// Wakeup Detection Enable
// Always set this bit to 0 in device controller mode.
#define USBFS_DVSTCTR0_RWUPE   (1U << 7)
// Wakeup Output
// Always set this bit to 0 in host controller mode.
#define USBFS_DVSTCTR0_WKUP    (1U << 8)
// USB_VBUSEN Output Pin Control
#define USBFS_DVSTCTR0_VBUSEN  (1U << 9)
// USB_EXICEN Output Pin Control
#define USBFS_DVSTCTR0_EXICEN  (1U << 10)
// Host Negotiation Protocol (HNP) Control
#define USBFS_DVSTCTR0_HNPBTOA (1U << 11)

// 29.2.4: CFIFO Port Register, D0FIFO Port Register, D1FIFO Port Register
// CFIFO Port Register
// When MBW = 1
#define USBFS_CFIFO   (RA6M1_USBFS_BASE + 0x14UL)
// When MBW = 0
#define USBFS_CFIFOL  USBFS_CFIFO

inline unsigned char ra6m1_usb_cfifo_read_byte__()
{
    Reg8 cfifol{ USBFS_CFIFOL };
    return *cfifol;
}

inline void ra6m1_usb_cfifo_write_byte__(const unsigned char byte)
{
    Reg8 cfifol{ USBFS_CFIFOL };
    cfifol = byte;
}

inline unsigned short ra6m1_usb_cfifo_read_hword__()
{
    Reg16 cfifo{ USBFS_CFIFO };
    return *cfifo;
}

inline void ra6m1_usb_cfifo_write_hword__(const unsigned short hword)
{
    Reg16 cfifo{ USBFS_CFIFO };
    cfifo = hword;
}

// D0FIFO Port Register
// When MBW = 1
#define USBFS_D0FIFO  (RA6M1_USBFS_BASE + 0x18UL)
// When MBW = 0
#define USBFS_D0FIFOL USBFS_D0FIFO

// D1FIFO Port Register
// When MBW = 1
#define USBFS_D1FIFO  (RA6M1_USBFS_BASE + 0x1CUL)
// When MBW = 0
#define USBFS_D1FIFOL USBFS_D1FIFO

/**
 * Three FIFO ports are available:
 * - CFIFO
 * - D0FIFO
 * - D1FIFO
 *
 * Each FIFO port is configured with:
 * - A port register that handles the reading of data from the FIFO buffer and writing of
 * data to the FIFO buffer.
 * - A port select register that selects the pipe assigned to the FIFO port.
 * - A port control register.
 *
 * Each FIFO port has the following constraints:
 * - Access to the FIFO buffer for DCP control transfers is through the CFIFO port.
 * - Access to the FIFO buffer for DMA or DTC transfers is through the D0FIFO or D1FIFO port.
 * - The D0FIFO and D1FIFO ports can also be accessed by the CPU.
 * - When using functions specific to the FIFO port, such as the DMA or DTC transfer function,
 *   you cannot change the pipe number selected in the CURPIPE bits of the port select register.
 * - Registers configuring a FIFO port do not affect other FIFO ports.
 * - The same pipe must not be assigned to two or more FIFO ports.
 * - There are two FIFO buffer states, one giving access rights to the CPU and the other to the
 *   serial interface engine (SIE).  When the SIE has access rights, the FIFO buffer cannot be
 *   accessed by the CPU.
 *
 * FIFOPORT[15:0] bits
 * When the FIFOPORT bits are accessed, the USBFS reads the received data from the FIFO buffer
 * or writes the transmit data to the FIFO buffer.
 * The FIFO port register can be accessed only when the FRDY bit in the associated port control
 * register is 1.  The valid bits in the FIFO port register depend on the MBW and BIGEND settings
 * in the port select register.
 */

// 29.2.5: CFIFO Port Select Register, D0FIFO Port Select Register, D1FIFO Port Select Register
#define USBFS_CFIFOSEL  (RA6M1_USBFS_BASE + 0x20UL)
// Only valid for n=0,1
#define USBFS_DFIFOSEL(n) (RA6M1_USBFS_BASE + 0x28UL + (((unsigned int)(n) * 0x4UL)))

// FIFO Port Access Pipe Specification
// The CURPIPE bits specify the pipe number to use for reading or writing data through the FIFO port.
// After writing to these bits, read them to check that the written value agrees with the read value
// before proceeding to the next process.  Do not set the same pipe number to the CURPIPE bits in
// CFIFOSEL, D0FIFOSEL, and D1FIFOSEL.
// During FIFO buffer access, the current access setting is maintained until the access is complete,
// even if software attempts to change the CURPIPE setting.
#define USBFS_FIFOSEL_CURPIPE_MASK  (0xFU)
// CFIFO Port Access Direction When DCP is Selected
#define USBFS_CFIFOSEL_ISEL  (1U << 5)
// FIFO Port Endian Control
#define USBFS_FIFOSEL_BIGEND (1U << 8)
// FIFO Port Access Bit Width
// When the selected pipe is receiving, after a write to these bits starts a data read from the
// FIFO buffer, do not change the bits until all of the data is read.  Set the CURPIPE and MBW bits
// simultaneously.  When reading the FIFO buffer, read using the access size that is set in MBW.
// When the selected pipe is transmitting, the bit width cannot be changed from 8-bit to 16-bit while
// data is being written to the FIFO buffer.
// An odd number of bytes can also be written through byte access control even when 16-bit width
// is selected.
#define USBFS_FIFOSEL_MBW    (1U << 10)
// Buffer Pointer Rewind
// When the selected pipe is receiving, setting this bit to 1 while the FIFO buffer is being read
// allows re-reading of the FIFO buffer from the first data.  In double buffering, this setting
// enables re-reading of the currently-read FIFO buffer plane from the first entry.
// Do not set this bit to 1 while simultaneously changing the CURPIPE bits.  Before setting the bit
// to 1, be sure to check that the FRDY bit is 1.
// To rewrite to the FIFO buffer from the first data for the transmitting pipe, use the BCLR bit.
#define USBFS_FIFOSEL_REW    (1U << 14)
// Read Count Mode
// Specifies the read mode for the value in the CFIFOCTR.DTLN bit.  When accessing DnFIFO with the
// PIPECFG.BFRE bit set to 1, set the RCNT bit to 0.
#define USBFS_FIFOSEL_RCNT   (1U << 15)

// DMA/DTC Transfer Request Enable
// To enable DMA or DTC transfer requests, set this bit to 1 after setting the CURPIPE bits.
// To change the CURPIPE setting, first set this bit to 0.
#define USBFS_DFIFOSEL_DREQE (1U << 12)
// Auto Buffer Memory Clear Mode Accessed after Specified Pipe Data is Read
// When this bit is set to 1, on receiving a zero-length packet while the FIFO buffer assigned
// to the selected pipe is empty, or when reading of a received short packet is complete while
// the PIPECFG.BFRE bit is 1, the USBFS sets the BCLR bit in the FIFO Port Control Register to 1.
// When using the USBFS with the SOFCFG.BRDYM bit set to 1, set this bit to 0.
#define USBFS_DFIFOSEL_DCLRM (1U << 13)

// FIFO Port Access Pipe Specification
void ra6m1_usb_cfifo_select(
    unsigned char pipe_number,
    bool big_endian,
    bool wide_access,
    bool buffer_pointer_rewind,
    bool read_count_mode);

// This is only valid for the Default Control Pipe (DCP)
inline void ra6m1_usb_cfifo_set_direction(bool write)
{
    Reg16 cfifosel{ USBFS_CFIFOSEL };
    // Clear CURPIPE bits.  CURPIPE=0 for DCP.
    unsigned short temp = cfifosel & ~(USBFS_FIFOSEL_CURPIPE_MASK | USBFS_CFIFOSEL_ISEL);
    // Write ISEL.
    if (write)
    {
	temp |= USBFS_CFIFOSEL_ISEL;
    }
    cfifosel = temp;
}

void ra6m1_usb_dfifo_select(
    bool dNfifo,
    unsigned char pipe_number,
    bool big_endian,
    bool wide_access,
    bool buffer_pointer_rewind,
    bool read_count_mode,
    bool transfer_req_enabled,
    bool auto_buffer_clear);

// 29.2.6: CFIFO Port Control Register, D0FIFO Port Control Register, D1FIFO Port Control Register
#define USBFS_CFIFOCTR          (RA6M1_USBFS_BASE + 0x22UL)
// Only valid for n=0,1
#define USBFS_DFIFOCTR(n)       (RA6M1_USBFS_BASE + 0x2AUL + (((unsigned int)(n)) * 0x4UL))
// Receive Data Length
// The DTLN bits indicate the length of the received data.
// While the FIFO buffer is being read, the DTLN bits indicate different values depending on the
// DnFIFOSEL.RCNT bit, as follows:
// - RCNT = 0
//   The USBFS sets the DTLN bits to indicate the length of the received data until the CPU or
//   DMA/DTC has read all of the received data from a single FIFO buffer plane.
//   While the PIPECFG.BFRE bit = 1, the USBFS retains the length of the received data until the
//   BCLR bit is set to 1, even after all data is read.
// - RCNT = 1
//   The USBFS decrements the value indicated in the DTLN bits each time data is read from the FIFO
//   buffer.  The value is decremented by 1 when MBW = 0, and by 2 when MBW = 1.
//   The USBFS sets these bits to 0 when all the data is read from one FIFO buffer plane.  In double
//   buffer mode, if data is received in one FIFO buffer plane before all of the data is read from
//   the other plane, the USBFS sets these bits to indicate the length of the receive data in the
//   former plane when all of the data is read from the latter plane.
#define USBFS_FIFOCTR_DTLN_MASK (0x1FFU)

inline unsigned short ra6m1_usb_cfifo_dtln__()
{
    Reg16 cfifoctr{ USBFS_CFIFOCTR };
    return cfifoctr & USBFS_FIFOCTR_DTLN_MASK;
}

// FIFO Port Ready
// The FRDY bit indicates whether the FIFO port can be accessed by the CPU or DMA/DTC
// In the following cases, the USBFS sets the FRDY bit to 1 but data cannot be read by the FIFO port
// because there is no data to be read:
// - A zero-length packet is received when the FIFO buffer assigned to the selected pipe is empty.
// - A short packet is received and the data is completely read while the PIPECFG.BFRE bit = 1.
// In these cases, set the BCLR bit to 1 to clear the FIFO buffer, and enable transmission and
// and reception of the next data.
#define USBFS_FIFOCTR_FRDY      (1U << 13)
// CPU Buffer Clear
// Set the BCLR bit to 1 to clear the FIFO buffer in the CPU for the selected pipe.
// When double buffer mode is set for the buffer assigned to the selected pipe, the USBFS clears only
// one plane of the FIFO buffer even when both planes are read-enabled.
// When the DCP is the selected pipe, setting the BCLR bit to 1 allows the USBFS to clear the FIFO
// buffer regardless of whether the CPU or SIE has access rights.  To clear the buffer when the SIE
// has access rights, set the DCPCTR.PID bits to 0b00 (NAK response) before setting the BCLR bit to 1.
// When the selected pipe is transmitting, if a 1 is written to the BVAL flag and the BCLR bit
// simultaneously, the USBFS clears the data that is already written, enabling the transmission of a
// zero-length packet.
// When the selected pipe is not the DCP, only write 1 to the BCLR bit while the FRDY bit in the FIFO
// Port Control Register is 1 (set by the USBFS).
#define USBFS_FIFOCTR_BCLR      (1U << 14)
// Buffer Memory Valid Flag
// Set the BVAL flag to 1 when data is completely written to the FIFO buffer in the CPU for the pipe
// selected in CURPIPE.  When the selected pipe is transmitting, set the flag in the following cases:
// - To transmit a short-packet, set this flag to 1 after data is written.
// - To transmit a zero-length packet, set this flag to 1 before data is written to the FIFO buffer.
// The USBFS then switches the FIFO buffer from the CPU to the SIE, enabling transmission.
// When data of the maximum packet size is written for the pipe in continuous transfer mode, the USBFS
// sets the BVAL flag to 1 and switches the FIFO buffer from the CPU to the SIE, enabling transmission.
// Only write 1 to the BVAL flag while the FRDY bit is 1 (set by the USBFS).  When the selected pipe
// is receiving, do not set the BVAL flag to 1.
#define USBFS_FIFOCTR_BVAL      (1U << 15)

inline bool ra6m1_usb_cfifo_buffer_valid()
{
    Reg16 cfifoctr{ USBFS_CFIFOCTR };
    return (cfifoctr & USBFS_FIFOCTR_BVAL) != 0;
}

inline void ra6m1_usb_cfifo_send_short_packet()
{
    Reg16 cfifoctr{ USBFS_CFIFOCTR };
    cfifoctr |= USBFS_FIFOCTR_BVAL;
}

inline void ra6m1_usb_cfifo_send_zlp()
{
    Reg16 cfifoctr{ USBFS_CFIFOCTR };
    cfifoctr |= USBFS_FIFOCTR_BVAL | USBFS_FIFOCTR_BCLR;
}

inline bool ra6m1_usb_cfifo_port_ready__()
{
    Reg16 cfifoctr{ USBFS_CFIFOCTR };
    return (cfifoctr & USBFS_FIFOCTR_FRDY) != 0;
}

inline void ra6m1_usb_cfifo_clear__()
{
    Reg16 cfifoctr{ USBFS_CFIFOCTR };
    cfifoctr |= USBFS_FIFOCTR_BCLR;
}

// 29.2.7: Interrupt Enable Register 0
#define USBFS_INTENB0          (RA6M1_USBFS_BASE + 0x30UL)
// Buffer Ready Interrupt Enable
#define USBFS_INTENB0_BRDYE    (1U << 8)
// Buffer Not Ready Response Interrupt Enable
#define USBFS_INTENB0_NRDYE    (1U << 9)
// Buffer Empty Interrupt Enable
#define USBFS_INTENB0_BEMPE    (1U << 10)
// Control Transfer Stage Transition Interrupt Enable
// Note: Can only be set to 1 in device controller mode.
#define USBFS_INTENB0_CTRE     (1U << 11)
// Device State Transition Interrupt Enable
// Note: Can only be set to 1 in device controller mode.
#define USBFS_INTENB0_DVSE     (1U << 12)
// Frame Number Update Interrupt Enable
#define USBFS_INTENB0_SOFE     (1U << 13)
// Resume Interrupt Enable
// Note: Can only be set to 1 in device controller mode.
#define USBFS_INTENB0_RSME     (1U << 14)
// VBUS Interrupt Enable
#define USBFS_INTENB0_VBSE     (1U << 15)

// 29.2.8: Interrupt Enable Register 1
#define USBFS_INTENB1          (RA6M1_USBFS_BASE + 0x32UL)
// Setup Transaction Normal Response Interrupt Enable
#define USBFS_INTENB1_SACKE    (1U << 4)
// Setup Transaction Error Interrupt Enable
#define USBFS_INTENB1_SIGNE    (1U << 5)
// EOF Error Detection Interrupt Enable
#define USBFS_INTENB1_EOFERRE  (1U << 6)
// Connection Detection Interrupt Enable
#define USBFS_INTENB1_ATTCHE   (1U << 11)
// Disconnection Detection Interrupt Enable
#define USBFS_INTENB1_DTCHE    (1U << 12))
// USB Bus Change Interrupt Enable
#define USBFS_INTENB1_BCHGE    (1U << 14)
// Overcurrent Input Change Interrupt Enable
#define USBFS_INTENB1_OVRCRE   (1U << 15)

// 29.2.9: BRDY Interrupt Enable Register
// The BRDYENB register enables or disables the INTSTS0.BRDY bit to be set to 1 when a BRDY interrupt
// is detected for each pipe.
#define USBFS_BRDYENB          (RA6M1_USBFS_BASE + 0x36UL)
// BRDY Interrupt Enable for Pipe N
// When a status flag in the BRDYSTS register is set to 1 and the associated PIPEnBRDYE bit setting in
// the BRDYENB is 1, the INTSTS0.BRDY flag is set to 1.  In this case, if the BRDYE bit in INTENB0
// is 1, the USBFS generates a BRDY interrupt request.
#define USBFS_BRDYENB_PIPEBRDYE(n)  (1U << (n))

// 29.2.10: NRDY Interrupt Enable Register
// The NRDYENB register enables or disables the INTSTS0.NRDY bit to be set to 1 when a NRDY interrupt
// is detected for each pipe.
#define USBFS_NRDYENB          (RA6M1_USBFS_BASE + 0x38UL)
// NRDY Interrupt Enable for Pipe N
#define USBFS_NRDYENB_PIPENRDYE(n)  (1U << (n))

// 29.2.11: BEMP Interrupt Enable Register
#define USBFS_BEMPENB          (RA6M1_USBFS_BASE + 0x3AUL)
// BEMP Interrupt Enable for Pipe N
#define USBFS_BEMPENB_PIPEBEMPE(n)  (1U << (n))

// 29.2.12: SOF Output Configuration Register
#define USBFS_SOFCFG           (RA6M1_USBFS_BASE + 0x3CUL)
// Edge Interrupt Output Status Monitor
// Note: Confirm that this bit is 0 before stopping the clock supply to the USBFS.
#define USBFS_SOFCFG_EDGESTS   (1U << 4)
// BRDY Interrupt Status Clear Timing
// 0=BRDY flag cleared by software.
// 1=BRDY flag cleared by the USBFS through a data read from the FIFO buffer or data write to the FIFO
// buffer.
#define USBFS_SOFCFG_BRDYM     (1U << 6)
// Transaction-Enabled Time Select
// Note: Confirm that this bit is 0 before stopping the clock supply to the USBFS.
// This bit is only valid in host controller mode.  Set this bit to 0 in device controller mode.
#define USBFS_SOFCFG_TRNENSEL  (1U << 8)

// 29.2.13: Interrupt Status Register 0
#define USBFS_INTSTS0            (RA6M1_USBFS_BASE + 0x40UL)
// Control Transfer Stage
#define USBFS_INTSTS0_CTSQ_MASK  (0b111)

namespace UsbControlTransferStage
{
    enum
    {
	IdleOrSetupStage = 0b000,
	ControlReadDataStage = 0b001,
	ControlReadStatusStage = 0b010,
	ControlWriteDataStage = 0b011,
	ControlWriteStatusStage = 0b100,
	ControlWriteNoDataStatusStage = 0b101,
	ControlTransferSequenceError = 0b110
    };
}

inline unsigned char ra6m1_usb_get_control_transfer_stage()
{
    Reg16 intsts0{ USBFS_INTSTS0 };
    return intsts0 & USBFS_INTSTS0_CTSQ_MASK;
}

// USB Request Reception
// Note: To clear the VBINT, RESM, SOFR, DVST, CTRT, or VALID bit, write 0 only to the bits to be
// cleared.  Write 1 to the other bits.  Do not write 0 to the status bits indicating 0.
// 0=Setup packet not received.
// 1=Setup packet received.
#define USBFS_INTSTS0_VALID      (1U << 3)
// Device State
#define USBFS_INTSTS0_DVSQ_MASK  (0b111U << 4)
#define USBFS_INTSTS0_DVSQ_RESET_MASK  (0b100 << 4)

namespace UsbDeviceState
{
    enum
    {
	PoweredState = 0b000,
	DefaultState = 0b001,
	AddressState = 0b010,
	ConfiguredState = 0b011
    };
}

inline unsigned char ra6m1_usb_get_device_state()
{
    Reg16 intsts0{ USBFS_INTSTS0 };
    return intsts0 & USBFS_INTSTS0_DVSQ_MASK;
}

inline bool ra6m1_usb_device_suspended()
{
    return (ra6m1_usb_get_device_state() & USBFS_INTSTS0_DVSQ_RESET_MASK) != 0;
}

// VBUS Input Status
#define USBFS_INTSTS0_VBSTS    (1U << 7)

inline bool ra6m1_usb_is_connected__()
{
    Reg16 intsts0{ USBFS_INTSTS0 };
    return (intsts0 & USBFS_INTSTS0_VBSTS) != 0;
}

// Buffer Ready Interrupt Status
#define USBFS_INTSTS0_BRDY     (1U << 8)
// Buffer Not Ready Interrupt Status
#define USBFS_INTSTS0_NRDY     (1U << 9)
// Buffer Empty Interrupt Status
#define USBFS_INTSTS0_BEMP     (1U << 10)
// Control Transfer Stage Transition Interrupt Status
// Note: The status of the RESM, DVST, and CTRT bits are changed only in device controller mode.
// Set the associated interrupt enable bits to 0 (disabled) in host controller mode.
// Note: To clear the VBINT, RESM, SOFR, DVST, CTRT, or VALID bit, write 0 only to the bits to be
// cleared.  Write 1 to the other bits.  Do not write 0 to the status bits indicating 0.
#define USBFS_INTSTS0_CTRT    (1U << 11)
// Device State Transition Interrupt Status
// Note: The status of the RESM, DVST, and CTRT bits are changed only in device controller mode.
// Set the associated interrupt enable bits to 0 (disabled) in host controller mode.
// Note: To clear the VBINT, RESM, SOFR, DVST, CTRT, or VALID bit, write 0 only to the bits to be
// cleared.  Write 1 to the other bits.  Do not write 0 to the status bits indicating 0.
#define USBFS_INTSTS0_DVST    (1U << 12)
// Frame Number Refresh Interrupt Status
// Note: To clear the VBINT, RESM, SOFR, DVST, CTRT, or VALID bit, write 0 only to the bits to be
// cleared.  Write 1 to the other bits.  Do not write 0 to the status bits indicating 0.
#define USBFS_INTSTS0_SOFR    (1U << 13)
// Resume Interrupt Status
// Note: The status of the RESM, DVST, and CTRT bits are changed only in device controller mode.
// Set the associated interrupt enable bits to 0 (disabled) in host controller mode.
// Note: To clear the VBINT, RESM, SOFR, DVST, CTRT, or VALID bit, write 0 only to the bits to be
// cleared.  Write 1 to the other bits.  Do not write 0 to the status bits indicating 0.
// Note: The USBFS detects a change in the status indicated in the VBINT and RESM bits even while the
// clock supply is stopped (SCKE bit is 0), and it requests the interrupt when the associated interrupt
// request bit is 1.  Enable the clock supply before clearing the status through software.
#define USBFS_INTSTS0_RESM    (1U << 14)
// VBUS Interrupt Status
// Note: To clear the VBINT, RESM, SOFR, DVST, CTRT, or VALID bit, write 0 only to the bits to be
// cleared.  Write 1 to the other bits.  Do not write 0 to the status bits indicating 0.
// Note: The USBFS detects a change in the status indicated in the VBINT and RESM bits even while the
// clock supply is stopped (SCKE bit is 0), and it requests the interrupt when the associated interrupt
// request bit is 1.  Enable the clock supply before clearing the status through software.
#define USBFS_INTSTS0_VBINT   (1U << 15)

// 29.2.14: Interrupt Status Register 1
// INTSTS1 is used to confirm the status of each interrupt in host controller mode.  Only enable the
// status change interrupts indicated in the bits in INTSTS1 in host controller mode.
// Note: To clear the bits in INTSTS1, write 0 only to the bits to be cleared.  Write 1 to other bits
// except reserved bits.
#define USBFS_INTSTS1         (RA6M1_USBFS_BASE + 0x42UL)
// Setup Transaction Normal Response Interrupt Status
#define USBFS_INTSTS1_SACK    (1U << 4)
// Setup Transaction Error Interrupt Status
#define USBFS_INTSTS1_SIGN    (1U << 5)
// EOF Error Detection Interrupt Status
#define USBFS_INTSTS1_EOFERR  (1U << 6)
// ATTCH Interrupt status
#define USBFS_INTSTS1_ATTCH   (1U << 11)
// USB Disconnection Detection Interrupt Status
#define USBFS_INTSTS1_DTCH    (1U << 12)
// USB Bus Change Interrupt Status
// Note: The USBFS detects a change in the status in the OVRCR or BCHG bit even when the clock supply
// is stopped (SYSCFG.SCKE = 0), and it requests the interrupt when the associated interrupt request
// bit is 1.  Enable the clock supply before clearing the status through software.  No other interrupts
// can be detected while the clock supply is stopped.
#define USBFS_INTSTS1_BCHG    (1U << 14)
// Overcurrent Input Change Interrupt Status
// Note: The USBFS detects a change in the status in the OVRCR or BCHG bit even when the clock supply
// is stopped (SYSCFG.SCKE = 0), and it requests the interrupt when the associated interrupt request
// bit is 1.  Enable the clock supply before clearing the status through software.  No other interrupts
// can be detected while the clock supply is stopped.
#define USBFS_INTSTS1_OVRCR   (1U << 15)

// 29.2.15: BRDY Interrupt Status Register
#define USBFS_BRDYSTS         (RA6M1_USBFS_BASE + 0x46UL)
// BRDY Interrupt Status for Pipe N
// Note: When the SOFCFG.BRDYM bit is set to 0, to clear the status indicated by the bits in BRDYSTS,
// write 0 only to the bits to be cleared.  Write 1 to other bits (except reserved bits).
// Note: When the SOFCFG.BRDYM bit is set to 0, clear BRDY interrupts before accessing the FIFO.
#define USBFS_BRDYSTS_PIPEBRDY(n)  (1U << (n))

// 29.2.16: NRDY Interrupt Status Register
#define USBFS_NRDYSTS         (RA6M1_USBFS_BASE + 0x48UL)
// NRDY Interrupt Status for Pipe N
// Note: To clear the status indicated by the bits in NRDYSTS, write 0 only to the bits to be cleared.
// Write 1 to the other bits (except reserved bits).
#define USBFS_NRDYSTS_PIPENRDY(n)  (1U << (n))

// 29.2.17: BEMP Interrupt Status Register
#define USBFS_BEMPSTS         (RA6M1_USBFS_BASE + 0x4AUL)
// BEMP Interrupt Status for Pipe N
// Note: to clear the status indicated by the bits in BEMPSTS, write 0 only to the bits to be cleared.
// Write 1 to the other bits (except reserved bits).
#define USBFS_BEMPSTS_PIPEBEMP(n)  (1U << (n))

// 29.2.18: Frame Number Register
#define USBFS_FRMNUM          (RA6M1_USBFS_BASE + 0x4CUL)
// Frame Number
#define USBFS_FRMNUM_FRNM_MASK  (0x7FFU)
// Receive Data Error
// Note: To clear the status, write 0 only to the bits to be cleared.  Write 1 to the other bits.
// The CRCE bit is set to 1 when a CRC error or bit stuffing error occurs during isochronous transfer.
// On detecting a CRC error in host controller mode, the USBFS generates an internal NRDY interrupt.
#define USBFS_FRMNUM_CRCE       (1U << 14)
// Overrun/Underrun Detection Status
// Note: To clear the status, write 0 only to the bits to be cleared.  Write 1 to the other bits.
// The OVRN flag is set to 1 when an overrun or underrun error occurs during isochronous transfer.
// In host controller mode, the OVRN bit is set to 1 on any of the following conditions:
// - For a transmitting isochronous pipe, the time to issue an OUT token comes before all of the
//   transmit data is written to the FIFO buffer.
// - For a receiving isochronous pipe, the time to issue an IN token comes when no FIFO buffer planes
//   are empty.
// In device controller mode, the OVRN bit is set to 1 on any of the following conditions:
// - For a transmitting isochronous pipe, the IN token is received before all of the transmit data
//   is written to the FIFO buffer.
// - For a receiving isochronous pipe, the OUT token is received when no FIFO buffer planes are empty.
#define USBFS_FRMNUM_OVRN       (1U << 15)

// 29.2.19: Device State Change Register
#define USBFS_DVCHGR         (RA6M1_USBFS_BASE + 0x4EUL)
// Device State Change
// 0=Writes to USBADDR.STSRECOV and USBADDR.USBADDR bits are disabled.
// 1=Writes to USBADDR.STSRECOV and USBADDR.USBADDR bits are enabled.
#define USBFS_DVCHGR_DVCHG   (1U << 15)

// 29.2.20: USB Address Register
#define USBFS_USBADDR        (RA6M1_USBFS_BASE + 0x50UL)
// USB Address
// In device controller mode, the USBADDR bits indicate the USB address received when the USBFS
// processed a SetAddress request successfully.  The USBFS sets the USBADDR bits to 0 on detecting a
// USB bus reset.
// Writing to these bits is enabled while the DVCHGR.DVCHG bit is set to 1.  On recovering from a USB
// power shut-off, operation can resume from the USB address set before software shut-off.
// In host controller mode, the USBADDR bits are invalid.
#define USBFS_USBADDR_USBADDR_MASK  (0x7FU)
// Status Recovery
// Use the STSRECOV bits to resume the state of the internal sequencer on recovering from USB power
// shut-off.  For details, see 29.3.15.
// Writing to these bits is enabled while the DVCHGR.DVCHG bit is set to 1.
#define USBFS_USBADDR_STSRECOV_MASK (0xFU << 8)

namespace UsbStatusRecovery
{
    enum
    {
	ReturnToFullSpeedDefaultState    = 0b1001,
	ReturnToFullSpeedAddressState    = 0b1010,
	ReturnToFullSpeedConfiguredState = 0b1011,
	ReturnToLowSpeedHost             = 0b0100,
	ReturnToFullSpeedHost            = 0b1000
    };
}

// 29.2.21: USB Request Type Register
// Note: In device controller mode, these bits can be read, but writing to them has no effect.  In host
// controller mode, these bits are both read/write bits.
#define USBFS_USBREQ         (RA6M1_USBFS_BASE + 0x54UL)
// Request Type
#define USBFS_USBREQ_BMREQUESTTYPE_MASK  (0xFFU)
// Request
#define USBFS_USBREQ_BREQUEST_MASK       (0xFF00U)
#define USBFS_USBREQ_BREQUEST_SHIFT      (8)

inline unsigned char ra6m1_usb_get_bmRequestType()
{
    Reg16 usbreq{ USBFS_USBREQ };
    return usbreq & USBFS_USBREQ_BMREQUESTTYPE_MASK;
}

inline unsigned char ra6m1_usb_get_bRequest()
{
    Reg16 usbreq{ USBFS_USBREQ };
    return (usbreq & USBFS_USBREQ_BREQUEST_MASK) >> USBFS_USBREQ_BREQUEST_SHIFT;
}

// 29.2.22: USB Request Value Register
// Note: In device controller mode, these bits can be read, but writing to themm has no effect.  In
// host controller mode, these bits are both read/write bits.
// In device controller mode, USBVAL stores the received wValue value.  In host controller mode, it is
// set to the wValue value to be transmitted.
// USBVAL is initialized by a USB bus reset.
#define USBFS_USBVAL          (RA6M1_USBFS_BASE + 0x56UL)

inline unsigned short ra6m1_usb_get_wValue()
{
    Reg16 usbval{ USBFS_USBVAL };
    return *usbval;
}

// 29.2.23: USB Request Index Register
// Note: In device controller mode, these bits can be read, but writing to themm has no effect.  In
// host controller mode, these bits are both read/write bits.
// USBINDX stores setup requests for control transfers.
// In device controller mode, it stores the received wIndex value.  In host controller mmode, it is
// set to the wIndex value to be transmitted.
// USBINDX is initialized by a USB bus reset.
#define USBFS_USBINDX         (RA6M1_USBFS_BASE + 0x58UL)

inline unsigned short ra6m1_usb_get_wIndex()
{
    Reg16 usbindx{ USBFS_USBINDX };
    return *usbindx;
}

// 29.2.24: USB Request Length Register
// Note: In device controller mode, these bits can be read, but writing to themm has no effect.  In
// host controller mode, these bits are both read/write bits.
// USBLENG stores setup requests for control transfers.
// In device controller mode, it stores the received wLength value.  In host controller mmode, it is
// set to the wLength value to be transmitted.
// USBLENG is initialized by a USB bus reset.
#define USBFS_USBLENG         (RA6M1_USBFS_BASE + 0x5AUL)

inline unsigned short ra6m1_usb_get_wLength()
{
    Reg16 usbleng{ USBFS_USBLENG };
    return *usbleng;
}

// 29.2.25: DCP Configuration Register
#define USBFS_DCPCFG          (RA6M1_USBFS_BASE + 0x5CUL)
void ra6m1_usb_configure_dcp(
    bool auto_nak,
    unsigned char max_packet_size);

// Transfer Direction
// Note: Only set this bit while the PID is NAK.  Before setting this bit after changing the DCPCTR.PID
// bits for the DCP from BUF to NAK, check that the DCPCTR.PBUSY bit is 0.  However, if the PID bits
// are changed to NAK by the USBFS, checking the PBUSY bit through software is not required.
// In host controller mode, the DIR bit sets the transfer direction of the data stage and status stage
// for control transfers.  In device controller mode, set the DIR bit to 0.
#define USBFS_DCPCFG_DIR      (1U << 4)
// Pipe Disabled at End of Transfer
// Note: Only set this bit while the PID is NAK.  Before setting this bit after changing the DCPCTR.PID
// bits for the DCP from BUF to NAK, check that the DCPCTR.PBUSY bit is 0.  However, if the PID bits
// are changed to NAK by the USBFS, checking the PBUSY bit through software is not required.
// The SHTNAK bit specifies whether to change PID to NAK on transfer end when the selected pipe is
// receiving.  It is only valid when the selected pipe is receiving.
// When the SHTNAK bit is 1, the USBFS changes the DCPCTR.PID bits for the DCP to NAK on determining
// that a transfer has ended.  The USBFS determines that the transfer has ended on the following
// conditions:
// - A short packet, including zero-length packet, is successfully received.
#define USBFS_DCPCFG_SHTNAK   (1U << 7)

// 29.2.26: DCP Maximum Packet Size Register
#define USBFS_DCPMAXP         (RA6M1_USBFS_BASE + 0x5EUL)
// Maximum packet size
// Note: Only set the MXPS bits while PID is NAK.  Before setting these bits after changing the
// DCPCTR.PID bits for the DCP from BUF to NAK, check that the DCPCTR.PBUSY bit is 0.  However, if the
// PID bits are changed to NAK by the USBFS, checking the PBUSY bit through software is not required.
// After modifying the MXPS bits and setting the DCP to the CURPIPE bits in the port select register,
// clear the buffer by setting the BCLR bit in the port control register to 1.
// The MXPS bits specify the maximum data payload for the DCP.  The initial value is 64 bytes.  Set the
// bits to a USB 2.0-compiant value.  Do not write to the FIFO buffer or set PID=BUF while MXPS is set
// to 0.
#define USBFS_DCPMAXP_MXPS_MASK  (0x7FU)
// Device Select
// Note: Only set the DEVSEL bits while PID is NAK and the DCPCTR.SUREQ bit is 0.  Before setting these
// bits after changing the DCPCTR.PID bits for the DCP from BUF to NAK, check that the DCPCTR.PBUSY bit
// is 0.  However, if the PID bits are changed to NAK by the USBFS, checking the PBUSY bit through
// software is not required.
// In host controller mode, the DEVSEL bits specify the address of the target peripheral device for a
// control transfer.  Set up the device address in the associated DEVADDn register first, and then set
// these bits to the corresponding value.  To set the DEVSEL bits to 0b0010, for example, first set the
// address in the DEVADD2 register.
// In device controller mode, set these bits to 0b0000.

inline void ra6m1_usb_set_dcp_max_packet_size__(
    unsigned short max_packet_size)
{
    Reg16 dcpmaxp{ USBFS_DCPMAXP };
    unsigned short temp = dcpmaxp & ~USBFS_DCPMAXP_MXPS_MASK;
    temp |= max_packet_size;
    dcpmaxp = temp;
}

#define USBFS_DCPMAXP_DEVSEL_MASK  (0xFU << 12)
#define USBFS_DCPMAXP_DEVSEL_SHIFT (12)

inline void ra6m1_usb_set_dcp_devsel__(
    unsigned char devsel)
{
    Reg16 dcpmaxp{ USBFS_DCPMAXP };
    unsigned short temp = dcpmaxp & ~USBFS_DCPMAXP_DEVSEL_MASK;
    temp |= devsel << USBFS_DCPMAXP_DEVSEL_SHIFT;
    dcpmaxp = temp;
}

// 29.2.27: DCP Control Register
#define USBFS_DCPCTR          (RA6M1_USBFS_BASE + 0x60UL)
// Response PID
// The PID bits control the USB response type during control transfers.
// In host controller mmode, to change the PID setting from NAK to BUF:
// - When the transmitting direction is set:
//   + Write all of the transmit data to the FIFO buffer while the DVSTCTR0.UACT bit is 1 and PID is
//     NAK.
//   + Set PID bits to BUF.  The USBFS then executes the OUT transaction.
// - When the receiving direction is set:
//   + Check that the FIFO buffer is empty (or empty the buffer) while the DVSTCTR0.UACT bit is 1 and
//     PID is NAK.
//   + Set the PID bits to BUF.  The USBFS then executes the IN transaction.
// The USBFS changes the PID setting as follows:
// - When the PID bits are set to BUF by software and the USBFS has received data exceeding
//   MaxPacketSize, the USBFS sets the PID to STALL (0b11).
// - When a reception error, such as a CRC error, is detected three times consecutively, the USBFS sets
//   the PID bits to NAK.
// - On receiving the STALL handshake, the USBFS sets PID to STALL (0b11).
// In device controller mode, the USBFS changes the PID setting as follows:
// - On receiving a setup packet, the USBFS sets PID to NAK.  The USBFS then sets the INSTS0.VALID flag
//   to 1, and the PID setting cannot be changed until software clears the VALID flag to 0.
// - When the PID bits are set to BUF by software and the USBFS has received data exceeding
//   MaxPacketSize, the USBFS sets PID to STALL (0b11).
// - On detecting a control transfer sequence error, the USBFS sets PID to STALL (0b1x).
// - On detecting a USB bus reset, the USBFS sets PID to NAK.
// The USBFS does not check the PID setting while processing a SET_ADDRESS request.
// The PID bits are initialized by a USB bus reset.
#define USBFS_DCPCTR_PID_MASK (0b11)

namespace UsbPid
{
    enum
    {
	NAK = 0b00,
	BUF = 0b01,
	STALL0 = 0b10,
	STALL1 = 0b11,
	STALL = 0b11
    };

    inline bool IsStall(unsigned char pid)
    {
	return (pid & 0b10) != 0;
    }
}

inline unsigned char ra6m1_usb_get_dcp_pid()
{
    Reg16 dcpctr{ USBFS_DCPCTR };
    return dcpctr && USBFS_DCPCTR_PID_MASK;
}

inline void ra6m1_usb_set_dcp_pid(unsigned char pid)
{
    Reg16 dcpctr{ USBFS_DCPCTR };
    unsigned short temp = dcpctr & (unsigned short)~USBFS_DCPCTR_PID_MASK;
    dcpctr = temp | pid;
}

inline void ra6m1_usb_nak_dcp()
{
    using namespace UsbPid;
    ra6m1_usb_set_dcp_pid(NAK);
}

// Control Transfer End Enable
// In device controller mmode, setting the CCPL bit to 1 enables the status stage of the control
// transfer to be completed.  When this bit is set to 1 by software while the associated PID bits are
// set to BUF, the USBFS completes the control transfer status stage.
// During control read transfers, the USBFS transmits the ACK handshake in response to the OUT
// transaction from the USB host.  During control write or no-data control transfers, it transmits the
// zero-length packet in response to the IN transaction from the USB host.  On detecting a SET_ADDRESS
// request, the USBFS operates in auto response mode from the setup stage up to the status stage
// completion regardless of the CCPL bit setting.
// The USBFS changes the CCPL bit from 1 to 0 on receiving a new setup packet.  Software cannot write 1
// to the bit while the INTSTS00.VALID bit is 1.  The CCPL bit is initialized by a USB bus reset.
// In host controller mode, always write 0 to the CCPL bit.
#define USBFS_DCPCTR_CCPL     (1U << 2)
// Pipe Busy
// The PBUSY bit indicates whether DCP is used for the transaction when the USBFS changes the PID bits
// from BUF to NAK.  The USBFS changes the PBUSY bit from 0 to 1 at the start of a USB transaction for
// the selected pipe, and changes the PBUSY bit from 1 to 0 on completion of one transaction.
// After PID is set to NAK by software, the value of the PBUSY bit indicates whether changes to the
// pipe settings can proceed.
#define USBFS_DCPCTR_PBUSY    (1U << 5)

inline bool ra6m1_usb_dcp_is_busy()
{
    Reg16 dcpctr{ USBFS_DCPCTR };
    return (dcpctr & USBFS_DCPCTR_PBUSY) != 0;
}

// Sequence Toggle Bit Monitor
// The SQMON bit indicates the expected value of the sequence toggle bit for the next transaction
// during a DCP transfer.
// the USBFS toggles the SQMON bit on successful completion of the transaction.  It does not toggle the
// bit, however, when a DATA_PID mismatch occurs during a transfer in the receiving direction.
// In device controller mode, the USBFS sets the SQMON bit to 1 (specifies DATA1 as the expected value)
// on successful reception of the setup packet.
// In device controller mode, the USBFS does not reference the SQMON bit during IN or OUT transactions
// at the status stage, and it does not toggle the bit on normal completion.
#define USBFS_DCPCTR_SQMON    (1U << 6)
// Sequence Toggle Bit Set
// Note: Only set SQSET and SQCLR bits to 1 while PID is NAK.  Before setting these bits after changing
// the PID bits for the DCP from BUF to NAK, check that the PBUSY bit is 0.  However, if the PID bits
// are changed to NAK by the USBFS, checking the PBUSY bit through software is not required.
// The SQSET bit specifies DATA1 as the expected value of the sequence toggle bit for the next
// transaction during a DCP transfer.
// Do not set the SQCLR and SQSET bits to 1 simultaneously.
#define USBFS_DCPCTR_SQSET    (1U << 7)
// Sequence Toggle Bit Clear
// Note: Only set SQSET and SQCLR bits to 1 while PID is NAK.  Before setting these bits after changing
// the PId bits for the DCP from BUF to NAK, check that the PBUSY bit is 0.  However, if the PID bits
// are changed to NAK by the USBFS, checking the PBUSY bit through software is not required.
// The SQCLR bit specifies DATA0 as the expected value of the sequence toggle bit for the next
// transaction during a DCP transfer.  It is read as 0.
// Do not set the SQCLR and SQSET bits to 1 simultaneously.
#define USBFS_DCPCTR_SQCLR    (1U << 8)
// SUREQ Bit Clear
// In host controller mode, setting the SUREQCLR bit to 1 clears the SUREQ bit to 0.
// In device controller mode, always write 0 to this bit.
#define USBFS_DCPCTR_SUREQCLR (1U << 11)
// Setup Token Transmission
// In host controller mode, setting the SUREQ bit to 1 triggers the USBFS to transmit the setup packet.
// In device controller mode, always write 0 to this bit.
#define USBFS_DCPCTR_SUREQ    (1U << 14)
// Buffer Status
// The BSTS bit indicates the status of access to the DCP FIFO buffer.  The meaning of this bit varies
// as follows depending on the CFIFOSEL.ISEL setting:
// - When ISEL=0, this bit indicates whether receive data can be read from the buffer.
// - When ISEL=1, this bit indicates whether transmit data can be written to the buffer.
#define USBFS_DCPCTR_BSTS     (1U << 15)

// 29.2.28: Pipe Window Select Register
// Set pipes 1 to 9 using the PIPESEL, PIPECFG, PIPEMAXP, PIPEPERI, PIPEnCTR, PIPEnTRE, and PIPEnTRN
// registers (n=0 to 9).
// After selecting the pipe in the PIPESEL register, set the pipe functions in the associated PIPECFG,
// PIPEMAXP, and PIPEPERI registers.  The PIPEnCTR, PIPEnTRE, and PIPEnTRN registers can be set
// independently of the pipe selection in the PIPESEL registers.
#define USBFS_PIPESEL         (RA6M1_USBFS_BASE + 0x64UL)
// Pipe Window Select
// The PIPESEL bits select the pipe number associated with the PIPECFG, PIPEMAXP, and PIPEPERI
// registers used for data writing and reading.  Selecting a pipe number in the PIPESEL bits allows
// writing to and reading from PIPECFG, PIPEMAXP, and PIPEPERI associated with the selected pipe number.
// when PIPESEL=0b0000, 0 is read from all of the bits in PIPECFG, PIPEMAXP, and PIPEPERI.  Writing to
// those bits is invalid.
inline void ra6m1_usb_select_pipe(unsigned char pipe_number)
{
    Reg16 pipesel{ USBFS_PIPESEL };
    pipesel = pipe_number;
}

// 29.2.29: Pipe Configuration Register
// PIPECFG specifies the transfer type, FIFO buffer access direction, and endpoint numbers for pipes 1
// to 9.  It also selects single or double buffer mode, and whether to continue or disable pipe
// operation at the end of transfer.
#define USBFS_PIPECFG         (RA6M1_USBFS_BASE + 0x68UL)
// Endpoint Number
// Note: Only set the TYPE, SHTNAK, and EPNUM bits while PID is NAK.  Before setting this bit after
// changing the PIPEnCTR.PID bits from BUF to NAK, check that the PIPEnCTR.PBUSY bit is 0.  However, if
// the PID bits are changed to NAK by the USBFS, checking the PBUSY bit through software is not
// required.
// The EPNUM bits specify the endpoint number for the selected pipe.  Setting 0b0000 indicates the pipe
// is not used.  Set these bits so that the combination of the DIR and EPNUM settings is different from
// those for other pipes.  The EPNUM bits can be set to 0b0000 for all pipes.
#define USBFS_PIPECFG_EPNUM_MASK  (0xFU)
void ra6m1_usb_configure_pipe(
    unsigned char pipe_number,
    unsigned char endpoint_number,
    bool transmit_pipe,
    bool auto_nak,
    bool double_buffer,
    bool brdy_each_transaction,
    unsigned char transfer_type,
    unsigned short max_packet_size);

inline unsigned char ra6m1_usb_get_pipe_epnum__()
{
    Reg16 pipecfg{ USBFS_PIPECFG };
    return pipecfg & USBFS_PIPECFG_EPNUM_MASK;
}

// Transfer Direction
// Note: Only set the BFRE, DBLB, and DIR bits while PID is NAK and before the pipe is selected in the
// CURPIPE bits in the port select register.  Before setting these bits after changing the PIPEnCTR.PID
// bits from BUF to NAK, check that the PIPEnCTR.PBUSY bit is 0.  However, if the PID bits are changed
// to NAK by the USBFS, checking the PBUSY bit through software is not required.
// Note: To change the BFRE, DBLB, or DIR bits after completing USB communication on the selected pipe,
// in addition to the constraints described above, write 1 and then 0 to the PIPEnCTR.ACLRM bit
// continously through software to clear the FIFO buffer assigned to the selected pipe.
// The DIR bit specifies the transfer direction for the selected pipe.
// When software sets this bit to 0, the USBFS uses the selected pipe for receiving.  When software
// sets this bit to 1, the USBFS uses the selected pipe for transmitting.
#define USBFS_PIPECFG_DIR     (1U << 4)
// Pipe Disabled at End of Transfer
// Note: Only set the TYPE, SHTNAK, and EPNUM bits while PID is NAK.  Before setting this bit after
// changing the PIPEnCTR.PID bits from BUF to NAK, check that the PIPEnCTR.PBUSY bit is 0.  However, if
// the PID bits are changed to NAK by the USBFS, checking the PBUSY bit through software is not
// required.
// The SHTNAK bit specifies whether to change the PIPEnCTR.PID bits to NAK at the end of transfer when
// the selected pipe is set in the receiving direction.  The SHTNAK bit is valid for pipes 1 to 5 in
// the receiving direction.
// When software sets this bit to 1 for a receiving pipe, the USBFS changes the PIPEnCTR.PID bits
// associated with the selected pipe to NAK on determining the transfer end.  The USBFS determines the
// transfer has ended on the following conditions:
// - A short packet (including a zero-length packet) is successfully received.
// - The transaction counter is used and the number of packets specified for the transaction are
//   successfully received.
#define USBFS_PIPECFG_SHTNAK  (1U << 7)
// Double Buffer Mode
// Note: Only set the BFRE, DBLB, and DIR bits while PID is NAK and before the pipe is selected in the
// CURPIPE bits in the port select register.  Before setting these bits after changing the PIPEnCTR.PID
// bits from BUF to NAK, check that the PIPEnCTR.PBUSY bit is 0.  However, if the PID bits are changed
// to NAK by the USBFS, checking the PBUSY bit through software is not required.
// Note: To change the BFRE, DBLB, or DIR bits after completing USB communication on the selected pipe,
// in addition to the constraints described above, write 1 and then 0 to the PIPEnCTR.ACLRM bit
// continously through software to clear the FIFO buffer assigned to the selected pipe.
// The DBLB bit selects either single or double buffer mode for the FIFO buffer used by the selected
// pipe.
// This bit is valid for pipes 1 to 5.
#define USBFS_PIPECFG_DBLB    (1U << 9)
// BRDY Interrupt Operation Specification
// Note: Only set the BFRE, DBLB, and DIR bits while PID is NAK and before the pipe is selected in the
// CURPIPE bits in the port select register.  Before setting these bits after changing the PIPEnCTR.PID
// bits from BUF to NAK, check that the PIPEnCTR.PBUSY bit is 0.  However, if the PID bits are changed
// to NAK by the USBFS, checking the PBUSY bit through software is not required.
// Note: To change the BFRE, DBLB, or DIR bits after completing USB communication on the selected pipe,
// in addition to the constraints described above, write 1 and then 0 to the PIPEnCTR.ACLRM bit
// continously through software to clear the FIFO buffer assigned to the selected pipe.
// The BFRE bit specifies the BRDY interrupt generation timing from the USBFS to the CPU for the
// selected pipe.
// When software sets the BFRE bit to 1 and the selected pipe is receiving, the USBFS detects the
// transfer completion and generates the BRDY interrupt on reading the packet.
// When a BRDY interrupt is generated with this setting, write 1 to the BCLR bit in the port control
// register with software.  The FIFO buffer assigned to the selected pipe is not enabled for reception
// until 1 is written to the BCLR bit.
// When the BFRE bit is set to 1 by software and the selected pipe is transmitting, the USBFS does not
// generate the BRDY interrupt.  For details, see 29.3.3.1.
#define USBFS_PIPECFG_BFRE    (1U << 10)
// Transfer Type
// Note: Only set the TYPE, SHTNAK, and EPNUM bits while PID is NAK.  Before setting this bit after
// changing the PIPEnCTR.PID bits from BUF to NAK, check that the PIPEnCTR.PBUSY bit is 0.  However, if
// the PID bits are changed to NAK by the USBFS, checking the PBUSY bit through software is not
// required.
// The TYPE bits specify the transfer type for the pipe selected in the PIPESEL.PIPESEL bits.  Before setting
// PID to BUF and starting USB communication on the selected pipe, set the TYPE bits to a value other than 0b00.
#define USBFS_PIPECFG_TYPE_MASK  (0b11U << 14)
#define USBFS_PIPECFG_TYPE_SHIFT (14)

namespace UsbTransferType
{
    enum
    {
	NotUsed = 0b00,
	BulkTransfer = 0b01,
	InterruptTransfer = 0b10,
	IsochronousTransfer = 0b11
    };
}

// 29.2.30: Pipe Maximum Packet Size Register
// PIPEMAXP specifies the maximum packet size for pipes 1 to 9.
#define USBFS_PIPEMAXP        (RA6M1_USBFS_BASE + 0x6CUL)
// Maximum Packet Size
// Note: Only set the MXPS bits while PID is NAK and before the pipe is selected in the CURPIPE bits in the port
// select register.  Before setting these bits after changing the PIPEnCTR.PID bits from BUF to NAK, check that
// the PIPEnCTR.PBUSY bit is 00.  However, if the PID bits are changed to NAK by the USBFS, checking the PBUSY bit
// through software is not required.
// The MXPS bits specify the maximum data payload (maximum packet size) for the selected pipe.
// Set these bits to the appropriate value for each transfer type based on the USB 2.0 specification.  When MXPS=0,
// do not write to the FIFO buffer or set PID to BUF.  These writes have no effect.
#define USBFS_PIPEMAXP_MXPS_MASK  (0x1FFU)

inline void ra6m1_usb_set_pipe_max_packet_size__(
    unsigned char pipe_number,
    unsigned short max_packet_size)
{
    if (pipe_number == 0)
    {
	ra6m1_usb_set_dcp_max_packet_size__(max_packet_size);
    }
    else
    {
	Reg16 pipemaxp{ USBFS_PIPEMAXP };
	ra6m1_usb_select_pipe(pipe_number);
	unsigned short temp = pipemaxp | ~USBFS_PIPEMAXP_MXPS_MASK;
	temp |= max_packet_size;
	pipemaxp = temp;
    }
}

// Device Select
// Note: Only set the DEVSEL bits while PID is NAK.  Before setting these bits after changing the
// PIPEnCTR.PID bits from BUF to NAK, check that the PIPEnCTR.PBUSY bit is 0.  However, if the
// PIPEnCTR.PID bits are changed to NAK by the USBFS, checking the PBUSY bit through software is not
// required.
// In host controller mode, the DEVSEL bits specify the address of the target device for USB
// communication.  Set up the device address in the associated DEVADDn (n=0 to 5) register first, then
// set these bits to the corresponding value.  To set the DEVSEL bits to 0b0010, for example, first set
// the address in the DEVADD2 register.
// In device controller mode, set these bits to 0b0000.

#define USBFS_PIPEMAXP_DEVSEL_MASK  (0xFU << 12)
#define USBFS_PIPEMAXP_DEVSEL_SHIFT (12)

inline void ra6m1_usb_set_pipe_devsel__(
    unsigned char devsel)
{
    Reg16 pipemaxp{ USBFS_PIPEMAXP };
    unsigned short temp = pipemaxp & ~USBFS_PIPEMAXP_DEVSEL_MASK;
    temp |= devsel << USBFS_PIPEMAXP_DEVSEL_SHIFT;
    pipemaxp = temp;
}


// 29.2.31: Pipe Cycle Control Register
// PIPEPERI selects whether the buffer is flushed or not when an interval error occurred during
// isochronous IN transfers, and sets the interval error detection interval for pipes 1 to 9.
#define USBFS_PIPEPERI        (RA6M1_USBFS_BASE + 0x6EUL)
// Interval Error Detection Interval
// Note: Only set the IITV bits while PID is NAK.  Before setting these bits after changing the PID
// bits from BUF to NAK, check that the PBUSY bit is 0.  However, if the PID bits are changed to NAK by
// the USBFS, checking the PBUSY bit through software is not required.
// To change the IITV bits to another value after they are set and USB communication is performed, set
// the PIPEnCTR.PID bits to NAK, then set the PIPEnCTR.ACLRM bit to 1 to initialize the interval timer.
// The IITV bits are not provided for pipes 3 to 5.  Write 0b000 to the bit positions of the IITV bits
// associated with pipes 3 to 5.
#define USBFS_PIPEPERI_IITV_MASK  (0b111)
// Isochronous IN Buffer Flush
// The IFIS bit specifies whether to flush the buffer when the pipe selected in the PIPESEL bits are
// used for isochronous transfers.
// In device controller mode when the selected pipe is for isochronous IN transfers, the USBFS
// automatically clears the FIFO buffer if the USBFS fails to receive the IN token from the USB host
// within the interval set in the IITV bits in terms of frames.
// When double buffering is specified, the USBFS only clears the data in the previously used plane.
// The USBFS clears the FIFO buffer on receiving the SOF packet immediately after the frame in which
// the USBFS expected to receive the IN token.  Even if the SOF packet is corrupted, the FIFO buffer is
// cleared at the time the SOF packet is expected to be received by using the internal interpolation
// function.
// When the host controller function is selected, set this bit to 0.  Set this bit to 0 when the
// selected pipe is not for isochronous transfer.
#define USBFS_PIPEPERI_IFIS   (1U << 12)

// 29.2.32: PIPEn Control Registers
// PIPEnCTR can be set for any pipe selection in the PIPESEL register.
// Note: only valid for n=1 to 9
#define USBFS_PIPECTR(n)      (RA6M1_USBFS_BASE + 0x70UL + (((unsigned int)((n) - 1)) * 2UL))
// Response PID
// The PID bits specify the response type for the next transaction on the selected pipe.
// The default PID setting is NAK.  Change the PID setting to BUF to use the associated pipe for USBFS
// transfer.
// After changing the PID setting from BUF to NAK through software during USBFS communication on the
// selected pipe, check that the PBUSY bit is 1 to determine if USBFS transfer on the selected pipe has
// actually entered the NAK state.  If the USBFS changes the PID bits to NAK, checking the PBUSY bit
// through software is not required.
// The USBFS changes the PIPEnCTR.PID setting in the following cases:
// - The USBFS sets PID to NAK on recognizing completion of the transfer when the pipe is receiving and
//   the PIPECFG.SHTNAK bit for the selected pipe is set to 1 by software.
// - The USBFS sets PID to STALL (0b11) on receiving a data packet with a payload exceeding the maximum
//   packet size of the selected pipe.
// - The USBFS sets PID to NAK on detecting a USB bus reset in device controller mode.
// - The USBFS sets PID to NAK on detecting a reception error such as a CRC error, three consecutive
//   times in host controller mode.
// - The USBFS sets PID to STALL (0b11) on receiving the STALL handshake in host controller mode.
// To specify the response type, set the PID bits as follows:
// - To transition from NAK to STALL, set 0b10.
// - To transition from BUF to STALL, set 0b11.
// - To transition from STALL (0b11) to NAK, set 0b10 and then 0b00.
// - To transition from STALL to BUF, transition to NAK and then BUF.
// See table 29.7 and 29.8.
#define USBFS_PIPECTR_PID_MASK (0b11)

void ra6m1_usb_set_pipe_pid(
    unsigned char pipe_number,
    unsigned char pid);

inline void ra6m1_usb_nak_pipe(unsigned char pipe_number)
{
    using namespace UsbPid;
    if (pipe_number == 0)
    {
	ra6m1_usb_nak_dcp();
    }
    else
    {
	ra6m1_usb_set_pipe_pid(pipe_number, NAK);
    }
}

// Pipe Busy
// The PBUSY bit indicates whether the selected pipe is being used for the current transaction.
// The USBFS changes the PBUSY bit from 0 to 1 at the start of the USBFS transaction for the selected
// pipe, and changes the PBUSY bit from 1 to 0 on completion of the transaction.
// reading the PBUSY bit with software after PID is set to NAK allows you to check whether changing the
// pipe setting is possible.  For details, see 29.3.4.1.
#define USBFS_PIPECTR_PBUSY   (1U << 5)

inline bool ra6m1_usb_pipe_is_busy(unsigned char pipe_number)
{
    if (pipe_number == 0)
    {
	return ra6m1_usb_dcp_is_busy();
    }
    else
    {
	Reg16 pipectr{ USBFS_PIPECTR(pipe_number) };
	ra6m1_usb_select_pipe(pipe_number);
	return (pipectr & USBFS_PIPECTR_PBUSY) != 0;
    }
}

// Sequence Toggle Bit Confirmation
// The SQMON bit indicates the expected value of the sequence toggle bit for the next transaction of
// the selected pipe.
// When the selected pipe is not the isochronous transfer type, the USBFS toggles the SQMON flag on
// successful completion of the transaction.  However, the USBFS does not toggle the SQMON bit when a
// DATA-PID mismatch occurs during transfer in the receiving direction.
#define USBFS_PIPECTR_SQMON   (1U << 6)
// Sequence Toggle Bit Set
// Note: Only set the ATREPM bit or write 1 to the SQCLR or SQSET bit while PID is NAK.  Before setting
// these bits after changing the PId bits from BUF to NAK, check that the PBUSY bit is 0.  However, if
// the PID bits are changed to NAK by the USBFS, checking the PBUSy bit through software is not
// required.
// Setting the SQSET bit to 1 through software causes the USBFS to set DATA1 as the expected value of
// the sequence toggle bit for the next transaction on the selected pipe.  the USBFS sets the SQSET bit
// to 0.
#define USBFS_PIPECTR_SQSET   (1U << 7)
// Sequence Toggle Bit Clear
// Note: Only set the ATREPM bit or write 1 to the SQCLR or SQSET bit while PID is NAK.  Before setting
// these bits after changing the PId bits from BUF to NAK, check that the PBUSY bit is 0.  However, if
// the PID bits are changed to NAK by the USBFS, checking the PBUSY bit through software is not
// required.
// Setting the SQCLR bit to 1 through software causes the USBFS to clear the expected value of the
// sequence toggle bit for the next transaction on the selected pipe to DATA0.  The USBFS sets the
// SQCLR bit to 0.
#define USBFS_PIPECTR_SQCLR   (1U << 8)
// Auto Buffer Clear Mode
// Note: Only set the ACLRM bit while PID is NAK and before the pipe is selected in the CURPIPE bits in
// the port select register.  Before setting these bits after changing the PID bits from BUF to NAK,
// check that the PBUSY bit is 0.  However, if the PID bits are changed to NAK by the USBFS, checking
// the PBUSY bit through software is not required.
// The ACLRM bit enables or disables auto buffer clear mode for the selected pipe.  To completely clear
// the data in the FIFO buffer allocated to the selected pipe, write 1 and then 0 to the ACLRM bit
// continuously.
// See table 29.9.
#define USBFS_PIPECTR_ACLRM   (1U << 9)

inline void ra6m1_usb_pipe_auto_clear__(unsigned char pipe_number)
{
    Reg16 pipectr{ USBFS_PIPECTR(pipe_number) };
    unsigned short temp = pipectr;
    pipectr |= USBFS_PIPECTR_ACLRM;
    wait_n_systicks(1);
    pipectr = temp;
}
// Auto Response Mode
// Note: Only set the ATREPM bit or write 1 to the SQCLR or SQSET bit while PID is NAK.  Before setting
// these bits after changing the PID bits from BUF to NAK, check that the PBUSY bit is 0.  However, if
// the PID bits are changed to NAK by the USBFS, checking the PBUSY bit through software is not
// required.
// The ATREPMM bit enables or disables auto response mode for the selected pipe.
// This bit can be set to 1 in device controller mode when the selected pipe is for bulk transfer.
// When this bit is set to 1, the USBFS responds to the token from the USB host as follows:
// - When the selected pipe is set for bulk IN transfers:
//   + When ATREPM=1 and PID=BUF, the USBFS transmits a zero-length packet in response to the IN token.
//   + The USBFS updates (allows toggling of) the sequence toggle bit (DATA-PID) each time the USBFS
//     receives ACK from the USB host.  In a single transaction, the IN token is received, a
//     zero-length packet is transmitted, and ACK is received.  The USBFS does not generate the BRDY or
//     BEMP interrupt.
// - When the selected pipe is for bulk OUT transfers:
//   + When ATREPM=1 and PID=BUF, the USBFS returns NAK in response to the OUT token and generates an
//     NRDY interrupt.
// For USB communication in auto response mode, set the ATREPM bit to 1 while the FIFO buffer is empty.
// Do not write to the FIFO buffer during USB communication in auto response mode.  When the selected
// pipe uses isochronous transfer, always set this bit to 0.
// In host controller mode, always set the ATREPM bit to 0.
#define USBFS_PIPECTR_ATREPM  (1U << 10)
// Transmit Buffer Monitor
// Note: only valid for pipes 1 to 5.
// The INBUFM bit indicates the FIFO buffer status for the selected pipe in the transmitting direction.
// When the selected pipe is transmitting, the USBFS sets this bit to 1 when the CPU or DMA/DTC
// completes writing data to at least one FIFO buffer plane.
// The USBFS sets the INBUFM bit to 0 when it completes transmission of the data from the FIFO buffer
// plane to which all the data is written.  In double buffer mode, the USBFS sets the INBUFM bit to 0
// when it completes transmission of the data from the two FIFO buffer planes before the CPU or DMA/DTC
// completes writing data to one FIFO buffer plane.
// The INBUFM bit indicates the same value as the BSTS bit when the selected pipe is receiving.
#define USBFS_PIPECTR_INBUFM  (1U << 14)
// Buffer Status
// Note: only valid for pipes 1 to 5.
// The BSTS bit indicates the FIFO buffer status for the selected pipe.
// The meaning of the BSTS bit depends on the PIPECFG.DIR, PIPECFG.BFRE, and DnFIFOSEL.DCLRM settings,
// as shown below:
// DIR value | BFRE value | DCLRM value | BSTS bit function
// ----------------------------------------------------------------------------------------------------
// 0         | 0          | 0           | Sets to 1 when receive data can be read from the FIFO buffer,
//           |            |             | and sets to 0 on completion of data read.
//           |            |             |
//           |            | 1           | Setting prohibited.
//           |            |             |
//           | 1          | 0           | Sets to 1 when receive data can be read from the FIFO buffer,
//           |            |             | and sets to 0 when software sets the BCLR bit in the port
//           |            |             | control register to 1 after the data read is complete.
//           |            |             |
//           |            | 1           | Sets to 1 when receive data can be read from the FIFO buffer,
//           |            |             | and sets to 0 on completion of data read.
//           |            |             |
// 1         | 0          | 0           | Sets to 1 when transmit data can be written to the FIFO
//           |            |             | buffer, and sets to 0 on completion of data write.
//           |            |             |
//           |            | 1           | Setting prohibited.
//           |            |             |
//           | 1          | 0           | Setting prohibited.
//           |            |             |
//           |            | 1           | Setting prohibited.
#define USBFS_PIPECTR_BSTS    (1U << 15)

// 29.2.33: PIPEn Transaction Counter Enable Register
// Note: Set each bit in PIPEnTRE while PID is NAK.  Before setting these bits after changing the PID
// from BUF to NAK, check that the PBUSY bit is 0.  However, if the PID bits are changed to NAK by the
// USBFS, checking the PBUSy bit through software is not required.
// Note: only valid for n=1 to 5.
#define USBFS_PIPETRE(n)      (RA6M1_USBFS_BASE + 0x90UL + (((unsigned int)((n) - 1)) * 4UL))
// Transaction Counter Clear
// When the TRCLR bit is set to 1, the USBFS clears the current value of the transaction counter
// associated with the selected pipe and then sets the TRCLR bit to 0.
#define USBFS_PIPETRE_TRCLR   (1U << 8)
// Transaction Counter Enable
// The TRENB bit enables or disables the transaction counter.
// For receiving pipes, setting the TRENB bit to 1 after setting the total number of the packets to be
// received in the PIPEnTRN.TRNCNT bits through software allows the USBFS to control hardware on having
// received the number of packets equal to the TRNCNT setting, as follows:
// - When the PIPECFG.SHTNAK bit is 1, the USBFS changes the PID bits to NAK for the associated pipe on
//   having received the number of packets equal to the TRNCNT setting.
// - When the PIPECFG.BFRE bit is 1, the USBFS asserts the BRDY interrupt on having received the number
//   of packets equal to the TRNCNT setting and then reading the last received data.
// For transmitting pipes, set the TRENB bit to 0.
// When the transaction counter is not used, set the TRENB bit to 0.  When the transaction counter is
// used, set the TRNCNT bits before setting the TRENB bit to 1.  Set this bit to 1 before receiving the
// first packet to be counted by the transaction counter.
#define USBFS_PIPETRE_TRENB   (1U << 9)

// 29.2.34: PIPEn Transaction Counter Register
// The PIPnTRN registers retain their settings during a USB bus reset.
// Note: only valid for n=1 to 5
#define USBFS_PIPETRN(n)      (RA6M1_USBFS_BASE + 0x92 + (((unsigned int)((n) - 1)) * 4UL))
// Transaction Counter
// The USBFS increments the value of the TRNCNT bits by 1 when all of the following conditions are
// satisfied on receiving the packet:
// - The PIPnTRE.TRENB bit = 1
// - (TRNCNT set value != current counter value + 1) on receiving the packet
// - The payload of the received packet aligns with the PIPEMAXP.MXPS setting.
// The USBFS sets the value of the TRNCNT bits to 00 when any of the following conditions are
// satisfied:
// - All of the following conditions are satisfied:
//   + The PIPEnTRE.TRENB bit = 1
//   + (TRNCNT set value = current counter value + 1) on receiving the packet
//   + The payload of the received packet aligns with the PIPEMAXP.MXPS setting.
// - Both of the following conditions are satisfied:
//   + The PIPEnTRE.TRENB bit = 1
//   + The USBFS received a short packet.
// - Both of the following conditions are satisfied:
//   + The PIPEnTRE.TRENB bit = 1
//   + The PIPEnTRE.TRCLR bit was set to 1 by software.
// For transmitting pipes, set the TRNCNT bits to 0.  When the transaction counter is not used, set the
// TRNCNT bits to 0.
// Setting the number of transactions to be transferred to the TRNCNT bits is only enabled when the
// PIPEnTRE.TRENB bit is 0.  To set the number of transactions to be transferred, set the TRCLR bit to
// 1 to clear the current counter value before setting the TRENB bit to 1.

inline void ra6m1_usb_set_transaction_count(
    unsigned char pipe_number,
    unsigned short count)
{
    Reg16 pipetre{ USBFS_PIPETRE(pipe_number) };
    Reg16 pipetrn{ USBFS_PIPETRN(pipe_number) };
    // Disable transaction counter.
    pipetre = 0;
    // Clear transaction counter.
    pipetre = USBFS_PIPETRE_TRCLR;
    pipetrn = count;
    pipetre = USBFS_PIPETRE_TRENB;
}

// 29.2.35: Device Address n Configuration Register
// In device controller mode, set all bits in this register to 0.

// 29.2.36: PHY Cross Point Adjustment Register
// The PHYSLEW register adjusts the cross point of the driver.  In both host and device controller
// modes, set this register before operating the controller.
#define USBFS_PHYSLEW         (RA6M1_USBFS_BASE + 0xF0UL)
// Driver Cross Point Adjustment 00
// 0: Reserved
// 1: Host or device controller mode
#define USBFS_PHYSLEW_SLEWR00 (1U << 0)
// Driver Cross Point Adjustment 01
// 0: Reserved
// 1: Host or device controller mode
#define USBFS_PHYSLEW_SLEWR01 (1U << 1)
// Driver Cross Point Adjustment 00
// 0: Reserved
// 1: Host or device controller mode
#define USBFS_PHYSLEW_SLEWF00 (1U << 2)
// Driver Cross Point Adjustment 01
// 0: Reserved
// 1: Host or device controller mode
#define USBFS_PHYSLEW_SLEWF01 (1U << 3)

#define USBFS_PHYSLEW_VALUE				\
    ( USBFS_PHYSLEW_SLEWR00				\
      | USBFS_PHYSLEW_SLEWR01				\
      | USBFS_PHYSLEW_SLEWF00 )
inline void ra6m1_usb_set_phy_cross_point()
{
    Reg32 physlew{ USBFS_PHYSLEW };
    physlew = USBFS_PHYSLEW_VALUE;
}

// 29.2.37: Deep Software Standby USB Transceiver Control/Pin Monitor Register
// TODO

// 29.2.38: Deep Software Standby USB Suspend/Resume Interrupt Register
// TODO

void ra6m1_usb_control_transfer(unsigned char transfer_stage);

#ifdef __cplusplus
extern "C" {
#endif
    void ra6m1_usb_set_address(unsigned int);
    void ra6m1_usb_set_configured(bool);
    void ra6m1_usb_init_usb(void);
    void ra6m1_usb_ep_isr(unsigned int);
    void ra6m1_usb_enter_default_state(void);
    void ra6m1_usb_enable(void);
    
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

bool ra6m1_usb_is_connected();
bool ra6m1_usb_attach();
void ra6m1_usb_detach();
bool ra6m1_usb_cfifo_ready(
    unsigned char pipe_number,
    bool write_dir);
void ra6m1_usb_cfifo_clear(
    unsigned char pipe_number,
    bool write_dir);
unsigned short ra6m1_usb_cfifo_rx_size(unsigned char pipe_number);
char* ra6m1_usb_cfifo_read(
    unsigned char pipe_number,
    unsigned short& size);
void ra6m1_usb_cfifo_write(
    unsigned char pipe_number,
    unsigned short size,
    const char* buffer);
void ra6m1_usb_cfifo_write_short_packet(
    unsigned char pipe_number,
    unsigned short size,
    const char* buffer);
void ra6m1_usb_cfifo_write_zlp(unsigned char pipe_number);
bool ra6m1_usb_pipe_buffer_ready(unsigned char pipe_number);
void ra6m1_usb_control_transfer(unsigned char transfer_stage);

struct RA6M1USBEndpointImpl : public USBEndpointImpl
{
private:
    unsigned char ep_num;
    unsigned char pipe_num;
    bool dir;  // false => in, true => out
    USBEndpoint::ep_type ep_type;
    unsigned short max_pack_size;
public:
    virtual void reset()
    {
	// Configure the endpoint.
	if (ep_num == 0)
	{
	    ra6m1_usb_configure_dcp(true, max_pack_size);
	}
	else
	{
	    unsigned char type_code;
	    switch (ep_type)
	    {
		using namespace UsbTransferType;
	    case USBEndpoint::ep_type::isochronous_ep:
		type_code = IsochronousTransfer;
		break;
	    case USBEndpoint::ep_type::bulk_ep:
		type_code = BulkTransfer;
		break;
	    case USBEndpoint::ep_type::interrupt_ep:
		type_code = InterruptTransfer;
		break;
	    default:
		// Error
		return;
	    }
	    ra6m1_usb_configure_pipe(
		pipe_num,
		ep_num,
		!dir,
		true,
		false,
		true,
		type_code,
		max_pack_size);
	}
    }

    virtual void stall()
    {
	ra6m1_usb_set_pipe_pid(pipe_num, UsbPid::STALL);
    }

    virtual void unstall()
    {
	ra6m1_usb_set_pipe_pid(pipe_num, UsbPid::NAK);
    }

    void complete_setup()
    {
	Reg16 dcpctr{ USBFS_DCPCTR };
	ra6m1_usb_set_dcp_pid(UsbPid::BUF);
	dcpctr |= USBFS_DCPCTR_CCPL;
    }

    virtual int send_data(
	const char* buf,
	unsigned int size,
	bool wait)
    {
	if (size > max_pack_size)
	{
	    return -1;
	}
	else if (size == max_pack_size)
	{
	    ra6m1_usb_cfifo_write(pipe_num, size, buf);
	}
	else if (size > 0)
	{
	    ra6m1_usb_cfifo_write_short_packet(pipe_num, size, buf);
	}
	else
	{
	    using namespace UsbControlTransferStage;
	    unsigned int stage = ra6m1_usb_get_control_transfer_stage();
	    if (pipe_num == 0 && (stage == ControlWriteDataStage || stage == ControlWriteNoDataStatusStage))
	    {
		complete_setup();
	    }
	    else
	    {
		ra6m1_usb_cfifo_write_zlp(pipe_num);
	    }
	}
	if (pipe_num == 0)
	{
	    //complete_setup();
	}
	if (wait)
	{
	    Reg16 bempsts{ USBFS_BEMPSTS };
	    while (!(bempsts & 1U));
	}
	return 0;
    }

    virtual char* read_data(unsigned int& size)
    {
	unsigned short size2;
	char* ret = ra6m1_usb_cfifo_read(pipe_num, size2);
	size = size2;
	return ret;
    }

    virtual char* read_setup(unsigned int& size)
    {
	// USB 2.0 9.3: Every setup packet has 8 bytes.
	// The USBFS parses these and stores them in peripheral registers for us.
	// We need to reassemble them into a buffer.
	char* buf = new(std::nothrow) char[8];
	if (buf == nullptr)
	{
	    size = 0;
	    return nullptr;
	}
	size = 8;
	buf[0] = (char)ra6m1_usb_get_bmRequestType();
	buf[1] = (char)ra6m1_usb_get_bRequest();
	unsigned short wValue = ra6m1_usb_get_wValue();
	memcpy(&buf[2], &wValue, 2);
	unsigned short wIndex = ra6m1_usb_get_wIndex();
	memcpy(&buf[4], &wIndex, 2);
	unsigned short wLength = ra6m1_usb_get_wLength();
	memcpy(&buf[6], &wLength, 2);
	// Setup packet received
	// 29.2.27:
	// Clear VALID bit before doing any processing.
	Reg16 intsts0{ USBFS_INTSTS0 };
	intsts0 = (unsigned short)~(USBFS_INTSTS0_VALID);
	return buf;
    }

    RA6M1USBEndpointImpl(
	unsigned char ep_num_,
	unsigned char pipe_num_,
	bool dir_,
	USBEndpoint::ep_type ep_type_,
	unsigned short size)
	: ep_num{ ep_num_ },
	  pipe_num{ pipe_num_ },
	  dir{ dir_ },
	  ep_type{ ep_type_ },
	  max_pack_size{ size }
    {
	reset();
    }

    virtual ~RA6M1USBEndpointImpl()
    {
	// Disable endpoint.
	ra6m1_usb_nak_pipe(pipe_num);
	Reg16 pipecfg{ USBFS_PIPECFG };
	pipecfg = 0;
	Reg16 bempenb{ USBFS_BEMPENB };
	Reg16 brdyenb{ USBFS_BRDYENB };
	bempenb &= ~(1U << pipe_num);
	brdyenb &= ~(1U << pipe_num);
    }
};

struct RA6M1USBInEndpoint : public USBInEndpoint
{
    RA6M1USBInEndpoint(
	unsigned char ep_num_,
	unsigned char pipe_num_,
	USBEndpoint::ep_type ep_type_,
	unsigned short size,
	unsigned char interval)
	: USBInEndpoint(
	    (unsigned char)(ep_num_ | 0x80),
	    ep_type_,
	    size,
	    interval,
	    new(std::nothrow) RA6M1USBEndpointImpl(
		ep_num_,
		pipe_num_,
		false,
		ep_type_,
		size))
	{
	    
	}
    RA6M1USBInEndpoint(const RA6M1USBInEndpoint&) = delete;
    RA6M1USBInEndpoint(RA6M1USBInEndpoint&&) = default;
};

struct RA6M1USBOutEndpoint : public USBOutEndpoint
{
    RA6M1USBOutEndpoint(
	unsigned char ep_num_,
	unsigned char pipe_num_,
	USBEndpoint::ep_type ep_type_,
	unsigned short size,
	unsigned char interval)
	: USBOutEndpoint(
	    ep_num_,
	    ep_type_,
	    size,
	    interval,
	    new(std::nothrow) RA6M1USBEndpointImpl(
		ep_num_,
		pipe_num_,
		true,
		ep_type_,
		size))
	{
	    
	}
    RA6M1USBOutEndpoint(const RA6M1USBOutEndpoint&) = delete;
    RA6M1USBOutEndpoint(RA6M1USBOutEndpoint&&) = default;
};

struct RA6M1USBControlEndpoint : public USBControlEndpoint
{
    RA6M1USBControlEndpoint(
	unsigned short size)
	: USBControlEndpoint(
	    0,
	    size,
	    new(std::nothrow) RA6M1USBEndpointImpl(
		0,
		0,
		true,
		USBEndpoint::ep_type::control_ep,
		size))
	{
	    
	}
    RA6M1USBControlEndpoint(const RA6M1USBControlEndpoint&) = delete;
    RA6M1USBControlEndpoint(RA6M1USBControlEndpoint&&) = default;
};

#endif


#endif // RA6M1_USB_HH_
