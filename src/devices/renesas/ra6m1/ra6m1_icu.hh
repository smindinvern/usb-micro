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

#ifndef RA6M1_ICU_HH_
#define RA6M1_ICU_HH_

/**
 * 14.1: The Interrupt Controller Unit (ICU) controls which event signals are linked
 * to the NVIC, DTC, and DMAC modules.  The ICU also controlls non-maskable interrupts.
 */

void ra6m1_wfi();

#define RA6M1_ICU_BASE        (RA6M1_PERIPHERALS_BASE + 0x6000UL)
// IRQ Control Register i (IRQCRi) (i = 0 to 13)
#define IRQCR(n)              (RA6M1_ICU_BASE + ((unsigned long)(n)))

// IRQi Detection Sense Select
enum IrqDetectionSense
{
    IrqDetectionFallingEdge = 0b00,
    IrqDetectionRisingEdge = 0b01,
    IrqDetectionBothEdges = 0b10
};

#define IRQMD_SHIFT           0

// IRQi Digital Filter Sampling Clock Select
enum IrqFilterSampleClock
{
    PClkB = 0b00,
    PClkBOver8 = 0b01,
    PClkBOver32 = 0b10,
    PClkBOver64 = 0b11
};

#define FCLKSEL_SHIFT         4

// IRQi Digital Filter Enable
#define FLTEN                 (1U << 7)

// Non-Maskable Interrupt Status Register (NMISR)
#define NMISR                 (RA6M1_ICU_BASE + 0x140UL)

// IWDT Underflow/Refresh Error Status Flag
#define IWDTST                (1U << 0)
// WDT Underflow/Refresh Error Status Flag
#define WDTST                 (1U << 1)
// Voltage Monitor 1 Interrupt Status Flag
#define LVD1ST                (1U << 2)
// Voltage Monitor 2 Interrupt Status Flag
#define LVD2ST                (1U << 3)
// Oscillation Stop Detection Interrupt Status Flag
#define OSTST                 (1U << 6)
// NMI Status Flag
#define NMIST		      (1U << 7)
// SRAM Parity Error Interrupt Status Flag
#define RPEST		      (1U << 8)
// SRAM ECC Error Interrupt Status Flag
#define RECCST		      (1U << 9)
// MPU Bus Slave Error Interrupt Status Flag
#define BUSSST		      (1U << 10)
// MPU Bus Master Error Interrupt Status Flag
#define BUSMST		      (1U << 11)
// CPU Stack Pointer Monitor Interrupt Status Flag
#define SPEST		      (1U << 12)

// Non-Maskable Interrupt Enable Register (NMIER)
#define NMIER		      (RA6M1_ICU_BASE + 0x120UL)
// IWDT Underflow/Refresh Error Interrupt Enable
#define IWDTEN		      (1U << 0)
// WDT Underflow/Refresh Error Interrupt Enable
#define WDTEN		      (1U << 1)
// Voltage Monitor 1 Interrupt Enable
#define LVD1EN                (1U << 2)
// Voltage Monitor 2 Interrupt Enable
#define LVD2EN                (1U << 3)
// Oscillation Stop Detection Interrupt Enable
#define OENEN                 (1U << 6)
// NMI Pin Interrupt Enable
#define NMIEN		      (1U << 7)
// SRAM Parity Error Interrupt Enable
#define RPEEN		      (1U << 8)
// SRAM ECC Error Interrupt Enable
#define RECCEN		      (1U << 9)
// MPU Bus Slave Error Interrupt Enable
#define BUSSEN		      (1U << 10)
// MPU Bus Master Error Interrupt Enable
#define BUSMEN		      (1U << 11)
// CPU Stack Pointer Monitor Interrupt Enable
#define SPEEN		      (1U << 12)

// Non-Maskable Interrupt Status Clear Register (NMICLR)
#define NMICLR		      (RA6M1_ICU_BASE + 0x130UL)
// IWDT Clear
#define IWDTCLR		      (1U << 0)
// WDT Clear
#define WDTCLR		      (1U << 1)
// LVD1 Clear
#define LVD1CLR               (1U << 2)
// LVD2 Clear
#define LVD2CLR               (1U << 3)
// OST Clear
#define OCLRCLR               (1U << 6)
// NMI Clear
#define NMICLR_		      (1U << 7)
// SRAM Parity Error Clear
#define RPECLR		      (1U << 8)
// SRAM ECC Error Clear
#define RECCCLR		      (1U << 9)
// MPU Bus Slave Error Clear
#define BUSSCLR		      (1U << 10)
// MPU Bus Master Error Clear
#define BUSMCLR		      (1U << 11)
// SPEST Clear
#define SPECLR		      (1U << 12)

void ra6m1_nmi_clear();
void ra6m1_nmi_clear(unsigned short flags);

// NMI Pin Interrupt Control Register (NMICR)
#define NMICR		      (RA6M1_ICU_BASE + 0x100UL)
// NMI Detection Set
#define NMIMD_FALLING_EDGE    (0U)
#define NMIMD_RISING_EDGE     (1U << 0)
// NMI Digital Filter Sampling Clock Select
#define NFCLKSEL_SHIFT	      4
// NMI Digital Filter Enable
#define NFLTEN		      (1U << 7)

// ICU Event Link Setting Register n (IELSRn) (n = 0 to 95)
#define IELSR(n)	      (RA6M1_ICU_BASE + 0x300UL + (((unsigned long)(n)) * 4UL))

// ICU Event Link Select
#define IELS_MASK             (0x1FFU)
// Interrupt Status Flag
#define IELSR_IR	      (1U << 16)
// DTC Activation Enable
#define DTCE		      (1U << 24)

void icu_link_event_to_irq(
    unsigned short event,
    unsigned char vector);
void icu_link_event_to_dtc(
    unsigned short event,
    unsigned char vector);
void icu_unlink_event(unsigned char vector);
void icu_clear_interrupt(unsigned char vector);

// DMAC Event Link Setting Register n (DELSRn) (n = 0 to 7)
#define DELSR(n)	      (RA6M1_ICU_BASE + 0x280UL + (((unsigned long)(n)) * 4UL))

// DMAC Event Link Select
#define DELS_MASK	      (0x1FFU)
// Interrupt Status Flag for DMAC
#define DELSR_IR	      (1U << 16)

// SYS Event Link Setting Register (SELSR0)
#define SELSR0		      (RA6M1_ICU_BASE + 0x200UL)
// SYS Event Link Select
#define SELS_MASK	      (0x1FFU)

void icu_link_snooze_cancel_event(unsigned short event);
void icu_unlink_snooze_cancel_event();

// Wake Up Interrupt Enable Register (WUPEN)
#define WUPEN		      (RA6M1_ICU_BASE + 0x1A0UL)
// IRQ Interrupt Software Standby Returns Enable
#define IRQWUPEN_MASK	      (0x3FFF)
// IWDT Interrupt Software Standby Returns Enable
#define IWDTWUPEN	      (1U << 16)
// Key Interrupt Software Standby Returns Enable
#define KEYWUPEN	      (1U << 17)
// LVD1 Interrupt Software Standby Returns Enable
#define LVD1WUPEN	      (1U << 18)
// LVD2 Interrupt Software Standby Returns Enable
#define LVD2WUPEN	      (1U << 19)
// ACMPHS0 Interrupt Software Standby Returns Enable
#define ACMPHS00WUPEN	      (1U << 22)
// RTC Alarm Interrupt Software Standby Returns Enable
#define RTCALMWUPEN	      (1U << 24)
// RTC Period Interrupt Software Standby Returns Enable
#define RTCPRDWUPEN	      (1U << 25)
// USBFS Interrupt Software Standby Returns Enable
#define USBFSWUPEN	      (1U << 27)
// AGT1 Underflow Interrupt Software Standby Returns Enable
#define AGT1UDWUPEN	      (1U << 28)
// AGT1 Compare Match A Interrupt Software Standby Returns Enable
#define AGT1CAWUPEN	      (1U << 29)
// AGT1 Compare Match B Interrupt Software Standby Returns Enable
#define AGT1CBWUPEN	      (1U << 30)
// IIC0 Address Match Interrupt Software Standby Returns Enable
#define IIC0WUPEN	      (1U << 31)

// Interrupt Vector Table
template<unsigned int irqN>
inline constexpr unsigned int vector_offset()
{
    static_assert(irqN <= 95, "Only 96 interrupt sources exist");
    return 0x40UL + (4UL * irqN);
}

// 14.3.2: Event Numbers

#define PORT_IRQ_EVENT(n)     ((unsigned long)(n) + 1UL)
template<unsigned int portN>
inline constexpr unsigned int port_irq_event()
{
    static_assert(portN <= 13, "Only 14 port IRQs exist");
    return PORT_IRQ_EVENT(portN);
}

#define DMAC_INT_EVENT(n)     ((unsigned long)(n) + 0x20UL)
template<unsigned int dmacN>
inline constexpr unsigned int dmac_int_event()
{
    static_assert(dmacN <= 7, "Only 8 DMACs exist");
    return DMAC_INT_EVENT(dmacN);
}

#define DTC_COMPLETE_EVENT    (0x29UL)
#define ICU_SNZCANCEL_EVENT   (0x2DUL)
#define FCU_FIFERR_EVENT      (0x30UL)
#define FCU_FRDYI_EVENT	      (0x31UL)
#define LVD_LVD1_EVENT	      (0x38UL)
#define LVD_LVD2_EVENT	      (0x39UL)
#define MOSC_STOP_EVENT	      (0x3BUL)
#define SYSTEM_SNZREQ_EVENT   (0x3CUL)
#define AGT0_AGTI_EVENT	      (0x40UL)
#define AGT0_AGTCMAI_EVENT    (0x41UL)
#define AGT0_AGTCMBI_EVENT    (0x42UL)
#define AGT1_AGTI_EVENT	      (0x43UL)
#define AGT1_AGTCMAI_EVENT    (0x44UL)
#define AGT1_AGTCMBI_EVENT    (0x45UL)
#define IWDT_NMIUNDF_EVENT    (0x46UL)
#define WDT_NMIUNDF_EVENT     (0x47UL)
#define RTC_ALM_EVENT	      (0x48UL)
#define RTC_PRD_EVENT	      (0x49UL)
#define RTC_CUP_EVENT	      (0x4AUL)
#define ADC120_ADI_EVENT      (0x4BUL)
#define ADC120_GBADI_EVENT    (0x4CUL)
#define ADC120_CMPAI_EVENT    (0x4DUL)
#define ADC120_CMPBI_EVENT    (0x4EUL)
#define ADC120_WCMPM_EVENT    (0x4FUL)
#define ADC120_WCMPUM_EVENT   (0x50UL)
#define ADC121_ADI_EVENT      (0x51UL)
#define ADC121_GBADI_EVENT    (0x52UL)
#define ADC121_CMPAI_EVENT    (0x53UL)
#define ADC121_CMPBI_EVENT    (0x53UL)
#define ADC121_WCMPM_EVENT    (0x55UL)
#define ADC121_WCMPUM_EVENT   (0x56UL)
#define ACMP_HS_EVENT(n)      (0x57UL + ((unsigned long)(n))))
#define USBFS_D0FIFO_EVENT    (0x5FUL)
#define USBFS_D1FIFO_EVENT    (0x60UL)
#define USBFS_USBI_EVENT      (0x61UL)
#define USBFS_USBR_EVENT      (0x62UL)
#define IIC0_RXI_EVENT	      (0x63UL)
#define IIC0_TXI_EVENT	      (0x64UL)
#define IIC0_TEI_EVENT	      (0x65UL)
#define IIC0_EEI_EVENT	      (0x66UL)
#define IIC0_WUI_EVENT	      (0x67UL)
#define IIC1_RXI_EVENT	      (0x68UL)
#define IIC1_TXI_EVENT	      (0x69UL)
#define IIC1_TEI_EVENT	      (0x6AUL)
#define IIC1_EEI_EVENT	      (0x6BUL)
#define SSIE0_SSITXI_EVENT    (0x72UL)
#define SSIE0_SSIRXI_EVENT    (0x73UL)
#define SSIE0_SSIF_EVENT      (0x75UL)
#define SRC_IDEI_EVENT	      (0x7AUL)
#define SRC_ODFI_EVENT	      (0x7BUL)
#define SRC_OVFI_EVENT	      (0x7CUL)
#define SRC_UDFI_EVENT	      (0x7DUL)
#define SRC_CEFI_EVENT	      (0x7EUL)
#define CTSU_CTSUWR_EVENT     (0x82UL)
#define CTSU_CTSURD_EVENT     (0x83UL)
#define CTSU_CTSUFN_EVENT     (0x84UL)
#define KEY_INTKR_EVENT	      (0x85UL)
#define DOC_DOPCI_EVENT	      (0x86UL)
#define CAC_FERRI_EVENT	      (0x87UL)
#define CAC_MENDI_EVENT	      (0x88UL)
#define CAC_OVFI_EVENT	      (0x89UL)
#define CAN0_ERS_EVENT	      (0x8AUL)
#define CAN0_RXF_EVENT	      (0x8BUL)
#define CAN0_TXF_EVENT	      (0x8CUL)
#define CAN0_RXM_EVENT	      (0x8DUL)
#define CAN0_TXM_EVENT	      (0x8EUL)
#define CAN1_ERS_EVENT	      (0x8FUL)
#define CAN1_RXF_EVENT	      (0x90UL)
#define CAN1_TXF_EVENT	      (0x91UL)
#define CAN1_RXM_EVENT	      (0x92UL)
#define CAN1_TXM_EVENT	      (0x93UL)
#define IOPORT_GROUP1_EVENT   (0x94UL)
#define IOPORT_GROUP2_EVENT   (0x95UL)
#define IOPORT_GROUP3_EVENT   (0x96UL)
#define IOPORT_GROUP4_EVENT   (0x97UL)
#define ELC_SWEVT0_EVENT      (0x98UL)
#define ELC_SWEVT1_EVENT      (0x99UL)
#define POEG_GROUP0_EVENT     (0x9AUL)
#define POEG_GROUP1_EVENT     (0x9BUL)
#define POEG_GROUP2_EVENT     (0x9CUL)
#define POEG_GROUP3_EVENT     (0x9DUL)
// Note: only valid for timers 0 to 7
#define GPTE_CCMPA_EVENT(n)   (0xB0UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_CCMPB_EVENT(n)   (0xB1UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_CMPC_EVENT(n)    (0xB2UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_CMPD_EVENT(n)    (0xB3UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_CMPE_EVENT(n)    (0xB4UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_CMPF_EVENT(n)    (0xB5UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_OVF_EVENT(n)     (0xB6UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_UDF_EVENT(n)     (0xB7UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_ADTRGA_EVENT(n)  (0xB8UL + (((unsigned long)(n)) * 10UL)))
#define GPTE_ADTRGB_EVENT(n)  (0xB9UL + (((unsigned long)(n)) * 10UL)))
// Note: only valid for timers 8 to 13
#define GPT_CCMPA_EVENT(n)    (0x100UL + (((unsigned long)(n) - 8UL) * 8UL))
#define GPT_CCMPB_EVENT(n)    (0x101UL + (((unsigned long)(n) - 8UL) * 8UL))
#define GPT_CMPC_EVENT(n)     (0x102UL + (((unsigned long)(n) - 8UL) * 8UL))
#define GPT_CMPD_EVENT(n)     (0x103UL + (((unsigned long)(n) - 8UL) * 8UL))
#define GPT_CMPE_EVENT(n)     (0x104UL + (((unsigned long)(n) - 8UL) * 8UL))
#define GPT_CMPF_EVENT(n)     (0x105UL + (((unsigned long)(n) - 8UL) * 8UL))
#define GPT_OVF_EVENT(n)      (0x106UL + (((unsigned long)(n) - 8UL) * 8UL))
#define GPT_UDF_EVENT(n)      (0x107UL + (((unsigned long)(n) - 8UL) * 8UL))

#define GPT_UVWEDGE_EVENT     (0x150UL)

#define SCI0_RXI_EVENT	      (0x174UL)
#define SCI0_TXI_EVENT	      (0x175UL)
#define SCI0_TEI_EVENT	      (0x176UL)
#define SCI0_ERI_EVENT	      (0x177UL)
#define SCI0_AM_EVENT	      (0x178UL)
#define SCI0_RXI_OR_ERI_EVENT (0x179UL)
#define SCI1_RXI_EVENT	      (0x17AUL)
#define SCI1_TXI_EVENT	      (0x17BUL)
#define SCI1_TEI_EVENT	      (0x17CUL)
#define SCI1_ERI_EVENT	      (0x17DUL)
#define SCI1_AM_EVENT	      (0x17EUL)
#define SCI2_RXI_EVENT	      (0x180UL)
#define SCI2_TXI_EVENT	      (0x181UL)
#define SCI2_TEI_EVENT	      (0x182UL)
#define SCI2_ERI_EVENT	      (0x183UL)
#define SCI2_AM_EVENT	      (0x184UL)
#define SCI3_RXI_EVENT	      (0x186UL)
#define SCI3_TXI_EVENT	      (0x187UL)
#define SCI3_TEI_EVENT	      (0x188UL)
#define SCI3_ERI_EVENT	      (0x189UL)
#define SCI3_AM_EVENT	      (0x18AUL)
#define SCI4_RXI_EVENT	      (0x18CUL)
#define SCI4_TXI_EVENT	      (0x18DUL)
#define SCI4_TEI_EVENT	      (0x18EUL)
#define SCI4_ERI_EVENT	      (0x18FUL)
#define SCI4_AM_EVENT	      (0x190UL)
#define SCI8_RXI_EVENT	      (0x1A4UL)
#define SCI8_TXI_EVENT	      (0x1A5UL)
#define SCI8_TEI_EVENT	      (0x1A6UL)
#define SCI8_ERI_EVENT	      (0x1A7UL)
#define SCI8_AM_EVENT	      (0x1A8UL)
#define SCI9_RXI_EVENT	      (0x1AAUL)
#define SCI9_TXI_EVENT	      (0x1ABUL)
#define SCI9_TEI_EVENT	      (0x1ACUL)
#define SCI9_ERI_EVENT	      (0x1ADUL)
#define SCI9_AM_EVENT	      (0x1AEUL)
#define SPI0_SPRI_EVENT	      (0x1BCUL)
#define SPI0_SPTI_EVENT	      (0x1BDUL)
#define SPI0_SPII_EVENT	      (0x1BEUL)
#define SPI0_SPEI_EVENT	      (0x1BFUL)
#define SPI0_SPTEND_EVENT     (0x1C0UL)
#define SPI1_SPRI_EVENT	      (0x1C1UL)
#define SPI1_SPTI_EVENT	      (0x1C2UL)
#define SPI1_SPII_EVENT	      (0x1C3UL)
#define SPI1_SPEI_EVENT	      (0x1C4UL)
#define SPI1_SPTEND_EVENT     (0x1C5UL)
#define QSPI_INTR_EVENT	      (0x1C6UL)
#define SDHI_MMC0_ACCS_EVENT  (0x1C7UL)
#define SDHI_MMC0_SDIO_EVENT  (0x1C8UL)
#define SDHI_MMC0_CARD_EVENT  (0x1C9UL)
#define SDHI_MMC0_ODMSDBREQ_EVENT (0x1CAUL)
#define SDHI_MMC1_ACCS_EVENT  (0x1CBUL)
#define SDHI_MMC1_SDIO_EVENT  (0x1CCUL)
#define SDHI_MMC1_CARD_EVENT  (0x1CDUL)
#define SDHI_MMC1_ODMSDBREQ_EVENT (0x1CEUL)








#endif // RA6M1_ICU_HH_
