#ifndef RA6M1_SPI_HH_
#define RA6M1_SPI_HH_

#include "Registers.hh"

// SPI Control Register
// Only valid for n=0, 1
#define RA6M1_SPCR(n)  (0x40072000UL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spcr[2] =
{
    { RA6M1_SPCR(0) },
    { RA6M1_SPCR(1) }
};
// SPI Mode Select
#define SPMS  (1U << 0)
inline void ra6m1_spi_select_4wire__(unsigned int n)
{
    ra6m1_spcr[n].clear_bits(SPMS);
}
inline void ra6m1_spi_select_clock_synchronous__(unsigned int n)
{
    ra6m1_spcr[n].set_bits(SPMS);
}
// Communications Operating Mode Select
#define TXMD  (1U << 1)
inline void ra6m1_spi_set_full_duplex__(unsigned int n)
{
    ra6m1_spcr[n].clear_bits(TXMD);
}
inline void ra6m1_spi_set_tx_only__(unsigned int n)
{
    ra6m1_spcr[n].set_bits(TXMD);
}
// Mode Fault Error Detection Enable
#define MODFEN  (1U << 2)
inline void ra6m1_spi_set_modfen__(unsigned int n, bool enabled)
{
    ra6m1_spcr[n].set_or_clear(MODFEN, enabled);
}
// SPI Master/Slave Mode Select
#define MSTR  (1U << 3)
inline void ra6m1_spi_set_master__(unsigned int n)
{
    ra6m1_spcr[n].set_bits(MSTR);
}
inline void ra6m1_spi_set_slave__(unsigned int n)
{
    ra6m1_spcr[n].clear_bits(MSTR);
}
// SPI Error Interrupt Enable
#define SPEIE  (1U << 4)
inline void ra6m1_spi_set_error_int_enabled__(unsigned int n, bool enabled)
{
    ra6m1_spcr[n].set_or_clear(SPEIE, enabled);
}
// Transmit Buffer Empty Interrupt Enable
#define SPTIE  (1U << 5)
inline void ra6m1_spi_set_tx_empty_int_enabled__(unsigned int n, bool enabled)
{
    ra6m1_spcr[n].set_or_clear(SPTIE, enabled);
}
// SPI Function Enable
#define SPE  (1U << 6)
inline void ra6m1_spi_set_enabled__(unsigned int n, bool enabled)
{
    ra6m1_spcr[n].set_or_clear(SPE, enabled);
}
// SPI Receive Buffer Full Interrupt Enable
#define SPRIE  (1U << 7)
inline void ra6m1_spi_set_rx_full_int_enabled__(unsigned int n, bool enabled)
{
    ra6m1_spcr[n].set_or_clear(SPRIE, enabled);
}

// SPI Slave Select Polarity Register
// Only valid for n=0, 1
#define RA6M1_SSLP(n)  (0x40072001UL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_sslp[2] =
{
    { RA6M1_SSLP(0) },
    { RA6M1_SSLP(1) }
};
// SSLn Signal Polarity Setting
// Only valid for n=0..3
#define SSLnP(n)  (1U << n)
inline void ra6m1_spi_set_ssl_polarity__(unsigned int n, unsigned int ssl, bool active_high)
{
    ra6m1_sslp[n].set_or_clear(SSLnP(ssl), active_high);
}

// SPI Pin Control Register
#define RA6M1_SPPCR(n)  (0x40072002UL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_sppcr[2] =
{
    { RA6M1_SPPCR(0) },
    { RA6M1_SPPCR(1) }
};
// SPI Loopback
#define SPLP  (1U)
inline void ra6m1_spi_set_loopback_enabled__(unsigned int n, bool enabled)
{
    ra6m1_sppcr[n].set_or_clear(SPLP, enabled);
}
// SPI Loopback 2
#define SPLP2  (1U << 1)
inline void ra6m1_spi_set_loopback2_enabled__(unsigned int n, bool enabled)
{
    ra6m1_sppcr[n].set_or_clear(SPLP2, enabled);
}
// MOSI Idle Fixed Value
#define MOIFV  (1U << 4)
inline void ra6m1_spi_set_mosi_idle_level__(unsigned int n, bool idle_high)
{
    ra6m1_sppcr[n].set_or_clear(MOIFV, idle_high);
}
// MOSI Idle Fixing Enable
#define MOIFE  (1U << 5)
inline void ra6m1_spi_set_mosi_idle_level_fixed__(unsigned int n, bool enabled)
{
    ra6m1_sppcr[n].set_or_clear(MOIFE, enabled);
}

// SPI Status Register
// Only valid for n=0, 1
#define RA6M1_SPSR(n)  (0x40072003UL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spsr[2] =
{
    { RA6M1_SPSR(0) },
    { RA6M1_SPSR(1) }
};
// Overrun Error Flag
#define OVRF  (1U)
inline bool ra6m1_spi_get_overrun_error_flag__(unsigned int n)
{
    return ra6m1_spsr[n].bits_set(OVRF);
}
inline void ra6m1_spi_clear_overrun_error_flag__(unsigned int n)
{
    ra6m1_spsr[n].clear_bits(OVRF);
}
// SPI Idle Flag
#define IDLNF  (1U << 1)
inline bool ra6m1_spi_get_idle_flag__(unsigned int n)
{
    return ra6m1_spsr[n].bits_set(IDLNF);
}
// Mode Fault Error Flag
#define MODF  (1U << 2)
inline bool ra6m1_spi_get_mode_fault_flag__(unsigned int n)
{
    return ra6m1_spsr[n].bits_set(MODF);
}
inline void ra6m1_spi_clear_mode_fault_flag__(unsigned int n)
{
    ra6m1_spsr[n].clear_bits(MODF);
}
// Parity Error Flag
#define PERF  (1U << 3)
inline bool ra6m1_spi_get_parity_error_flag__(unsigned int n)
{
    return ra6m1_spsr[n].bits_set(PERF);
}
inline void ra6m1_spi_clear_parity_error_flag__(unsigned int n)
{
    ra6m1_spsr[n].clear_bits(PERF);
}
// Underrun Error Flag
#define UDRF  (1U << 4)
inline bool ra6m1_spi_get_underrun_error_flag__(unsigned int n)
{
    return ra6m1_spsr[n].bits_set(UDRF);
}
inline void ra6m1_spi_clear_underrun_error_flag__(unsigned int n)
{
    // 34.2.4: Note 2.
    ra6m1_spsr[n].clear_bits(UDRF | MODF);
}
// SPI Transmit Buffer Empty Flag
#define SPTEF  (1U << 5)
inline bool ra6m1_spi_get_tx_empty_flag__(unsigned int n)
{
    return ra6m1_spsr[n].bits_set(SPTEF);
}
// SPI Receive Buffer Full Flag
#define SPRF  (1U << 7)
inline bool ra6m1_spi_get_rx_full_flag__(unsigned int n)
{
    return ra6m1_spsr[n].bits_set(SPRF);
}

// SPI Data Register
// Only valid for n=0, 1
#define RA6M1_SPDR(n)  (0x40072004UL + (((unsigned long)(n)) * 0x100UL))
static Reg32 ra6m1_spdr[2] =
{
    { RA6M1_SPDR(0) },
    { RA6M1_SPDR(1) }
};
static Reg16 ra6m1_spdr_ha[2] =
{
    { RA6M1_SPDR(0) },
    { RA6M1_SPDR(1) }
};
static Reg8 ra6m1_spdr_b[2] =
{
    { RA6M1_SPDR(0) },
    { RA6M1_SPDR(1) }
};

// SPI Sequence Control Register
#define RA6M1_SPSCR(n)  (0x40072008UL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spscr[2] =
{
    { RA6M1_SPSCR(0) },
    { RA6M1_SPSCR(1) }
};
// SPI Sequence Length Specification
inline void ra6m1_spi_set_sequence_length__(unsigned int n, unsigned char len)
{
    ra6m1_spscr[n] = len;
}
inline unsigned char ra6m1_spi_get_sequence_length__(unsigned int n)
{
    return *(ra6m1_spscr[n]);
}

// SPI Sequence Status Register
#define RA6M1_SPSSR(n)  (0x40072009UL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spssr[2] =
{
    { RA6M1_SPSSR(0) },
    { RA6M1_SPSSR(1) }
};
// SPI Command Pointer
inline void ra6m1_spi_set_command_pointer__(unsigned int n, unsigned char cmd)
{
    unsigned char temp = *(ra6m1_spssr[n]);
    temp &= ~0b111;
    ra6m1_spssr[n] = temp | cmd;
}
inline unsigned char ra6m1_spi_get_command_pointer__(unsigned int n)
{
    return (*(ra6m1_spssr[n])) & 0b111;
}
// SPI Error Command
inline void ra6m1_spi_set_error_command__(unsigned int n, unsigned char cmd)
{
    unsigned char temp = *(ra6m1_spssr[n]);
    temp &= ~(0b111 << 4);
    ra6m1_spssr[n] = temp | (cmd << 4);
}
inline unsigned char ra6m1_spi_get_error_command__(unsigned int n)
{
    return ((*(ra6m1_spssr[n])) >> 4) & 0b111;
}

// SPI Bit Rate Register
#define RA6M1_SPBR(n)  (0x4007200AUL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spbr[2] =
{
    { RA6M1_SPBR(0) },
    { RA6M1_SPBR(1) }
};
inline void ra6m1_spi_set_bit_rate_bits__(unsigned int n, unsigned char bits)
{
    ra6m1_spbr[n] = bits;
}
inline unsigned char ra6m1_spi_get_bit_rate_bits__(unsigned int n)
{
    return *(ra6m1_spbr[n]);
}

// SPI Data Control Register
// Only valid for n=0, 1
#define RA6M1_SPDCR(n)  (0x4007200BUL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spdcr[2] =
{
    { RA6M1_SPDCR(0) },
    { RA6M1_SPDCR(1) }
};
// Number of Frames Specification
inline void ra6m1_spi_set_number_of_frames__(unsigned int n, unsigned char frames)
{
    unsigned char temp = *(ra6m1_spdcr[n]);
    temp &= ~0b11;
    ra6m1_spdcr[n] = temp | ((frames - 1) & 0b11);
}
inline unsigned char ra6m1_spi_get_number_of_frames__(unsigned int n)
{
    return ((*(ra6m1_spdcr[n])) & 0b11) + 1;
}
// SPI Receive/Transmit Data Select
#define SPRDTD  (1U << 4)
inline void ra6m1_spi_set_read_buffer__(unsigned int n, bool read_tx)
{
    ra6m1_spdcr[n].set_or_clear(SPRDTD, read_tx);
}
// SPI Word Access/Halfword Access Specification
#define SPLW  (1U << 5)
// SPI Byte Access Specification
#define SPBYT  (1U << 6)
namespace SpiAccessWidth
{
    enum AccessWidth
    {
	Byte,
	HalfWord,
	Word
    };
}
inline void ra6m1_spi_set_access_width__(unsigned int n, SpiAccessWidth::AccessWidth width)
{
    unsigned char bits = 0;
    switch (width)
    {
    case SpiAccessWidth::Byte:
	bits = SPBYT;
	break;
    case SpiAccessWidth::HalfWord:
	bits = 0;
	break;
    case SpiAccessWidth::Word:
	bits = SPLW;
	break;
    }
    unsigned char temp = *(ra6m1_spdcr[n]);
    temp &= ~(SPLW | SPBYT);
    ra6m1_spdcr[n] = temp | (bits << 5);
}
inline bool ra6m1_spi_get_splw__(unsigned int n)
{
    return ra6m1_spdcr[n].bits_set(SPLW);
}
inline bool ra6m1_spi_get_spbyt__(unsigned int n)
{
    return ra6m1_spdcr[n].bits_set(SPBYT);
}
inline SpiAccessWidth::AccessWidth ra6m1_spi_get_access_width__(unsigned int n)
{
    if (ra6m1_spi_get_spbyt__(n))
    {
	return SpiAccessWidth::Byte;
    }
    else if (ra6m1_spi_get_splw__(n))
    {
	return SpiAccessWidth::Word;
    }
    else
    {
	return SpiAccessWidth::HalfWord;
    }
}

// SPI Clock Delay Register
#define RA6M1_SPCKD(n)  (0x4007200CUL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spckd[2] =
{
    { RA6M1_SPCKD(0) },
    { RA6M1_SPCKD(1) }
};
inline void ra6m1_spi_set_clock_delay__(unsigned int n, unsigned char delay)
{
    ra6m1_spckd[n] = (delay - 1) & 0b111;
}
inline unsigned char ra6m1_spi_get_clock_delay__(unsigned int n)
{
    return *(ra6m1_spckd[n]) + 1;
}

// SPI Slave Select Negation Delay Register
#define RA6M1_SSLND(n)  (0x4007200DUL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_sslnd[2] =
{
    { RA6M1_SSLND(0) },
    { RA6M1_SSLND(1) }
};
inline void ra6m1_spi_set_ssl_negation_delay__(unsigned int n, unsigned char delay)
{
    ra6m1_sslnd[n] = (delay - 1) & 0b111;
}
inline unsigned char ra6m1_spi_get_sll_negation_delay__(unsigned int n)
{
    return *(ra6m1_sslnd[n]) + 1;
}

// SPI Next-Access Delay Register
#define RA6M1_SPND(n)  (0x4007200EUL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spnd[2] =
{
    { RA6M1_SPND(0) },
    { RA6M1_SPND(1) }
};
inline void ra6m1_spi_set_next_access_delay__(unsigned int n, unsigned char delay)
{
    ra6m1_spnd[n] = (delay - 1) & 0b111;
}
inline unsigned char ra6m1_spi_get_next_access_delay__(unsigned int n)
{
    return *(ra6m1_spnd[n]) + 1;
}

// SPI Control Register 2
#define RA6M1_SPCR2(n)  (0x4007200FUL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spcr2[2] =
{
    { RA6M1_SPCR2(0) },
    { RA6M1_SPCR2(1) }
};
// Parity Enable
#define SPPE  (1U)
inline void ra6m1_spi_set_parity_enable__(unsigned int n, bool enabled)
{
    ra6m1_spcr2[n].set_or_clear(SPPE, enabled);
}
inline bool ra6m1_spi_get_parity_enable__(unsigned int n)
{
    return ra6m1_spcr2[n].bits_set(SPPE);
}
// Parity Mode
#define SPOE  (1U << 1)
inline void ra6m1_spi_set_parity_mode__(unsigned int n, bool odd_parity)
{
    ra6m1_spcr2[n].set_or_clear(SPOE, odd_parity);
}
inline bool ra6m1_spi_get_odd_parity_mode__(unsigned int n)
{
    return ra6m1_spcr2[n].bits_set(SPOE);
}
// SPI Idle Interrupt Enable
#define SPIIE  (1U << 2)
inline void ra6m1_spi_set_idle_int_enable__(unsigned int n, bool enabled)
{
    ra6m1_spcr2[n].set_or_clear(SPIIE, enabled);
}
inline bool ra6m1_spi_get_idle_int_enable__(unsigned int n, bool enabled)
{
    return ra6m1_spcr2[n].bits_set(SPIIE);
}
// Parity Self-Testing
#define PTE  (1U << 3)
inline void ra6m1_spi_set_parity_self_test__(unsigned int n, bool enabled)
{
    ra6m1_spcr2[n].set_or_clear(PTE, enabled);
}
inline bool ra6m1_spi_get_parity_self_test__(unsigned int n)
{
    return ra6m1_spcr2[n].bits_set(PTE);
}
// RSPCK Auto-Stop Function Enable
#define SCKASE  (1U << 4)
inline void ra6m1_spi_set_rspck_auto_stop__(unsigned int n, bool enabled)
{
    ra6m1_spcr2[n].set_or_clear(SCKASE, enabled);
}
inline bool ra6m1_spi_get_rspck_auto_stop__(unsigned int n)
{
    return ra6m1_spcr2[n].bits_set(SCKASE);
}

// SPI Command Registers 0 to 7
#define RA6M1_SPCMD(n, c)  (0x40072010UL + (((unsigned long)(n)) * 0x100UL) + (((unsigned long)(c)) * 0x2UL))
static Reg16 ra6m1_spcmd[2][8] =
{
    {
	{ RA6M1_SPCMD(0, 0) },
	{ RA6M1_SPCMD(0, 1) },
	{ RA6M1_SPCMD(0, 2) },
	{ RA6M1_SPCMD(0, 3) },
	{ RA6M1_SPCMD(0, 4) },
	{ RA6M1_SPCMD(0, 5) },
	{ RA6M1_SPCMD(0, 6) },
	{ RA6M1_SPCMD(0, 7) }
    },
    {
	{ RA6M1_SPCMD(1, 0) },
	{ RA6M1_SPCMD(1, 1) },
	{ RA6M1_SPCMD(1, 2) },
	{ RA6M1_SPCMD(1, 3) },
	{ RA6M1_SPCMD(1, 4) },
	{ RA6M1_SPCMD(1, 5) },
	{ RA6M1_SPCMD(1, 6) },
	{ RA6M1_SPCMD(1, 7) }
    }
};
// RSPCK Phase Setting
#define CPHA  (1U)
inline void ra6m1_spi_cmd_set_cpha__(unsigned int n, unsigned int c, bool cpha)
{
    ra6m1_spcmd[n][c].set_or_clear(CPHA, cpha);
}
inline bool ra6m1_spi_cmd_get_cpha__(unsigned int n, unsigned int c)
{
    return ra6m1_spcmd[n][c].bits_set(CPHA);
}
// RSPCK Polarity Setting
#define CPOL  (1U << 1)
inline void ra6m1_spi_cmd_set_cpol__(unsigned int n, unsigned int c, bool cpol)
{
    ra6m1_spcmd[n][c].set_or_clear(CPOL, cpol);
}
inline bool ra6m1_spi_cmd_get_cpol__(unsigned int n, unsigned int c)
{
    return ra6m1_spcmd[n][c].bits_set(CPOL);
}
// Bit Rate Division Setting
inline void ra6m1_spi_cmd_set_brdv_bits__(unsigned int n, unsigned int c, unsigned char brdv)
{
    unsigned short temp = *(ra6m1_spcmd[n][c]);
    temp &= ~(0b11 << 2);
    ra6m1_spcmd[n][c] = temp | ((brdv & 0b11) << 2);
}
inline unsigned char ra6m1_spi_cmd_get_brdv_bits__(unsigned int n, unsigned int c)
{
    return (*(ra6m1_spcmd[n][c]) >> 2) & 0b11;
}
// SSL Signal Assertion Setting
inline void ra6m1_spi_cmd_set_ssl_select__(unsigned int n, unsigned int c, unsigned char ssl)
{
    unsigned short temp = *(ra6m1_spcmd[n][c]);
    temp &= ~(0b111 << 4);
    ra6m1_spcmd[n][c] = temp | ((ssl & 0b011) << 4);
}
inline unsigned char ra6m1_spi_cmd_get_ssl_select__(unsigned int n, unsigned int c)
{
    return (*(ra6m1_spcmd[n][c]) >> 4) & 0b111;
}
// SSL Signal Level Keeping
#define SSLKP  (1U << 7)
inline void ra6m1_spi_cmd_set_ssl_level_keeping__(unsigned int n, unsigned int c, bool negate_at_completion)
{
    ra6m1_spcmd[n][c].set_or_clear(SSLKP, !negate_at_completion);
}
inline bool ra6m1_spi_cmd_get_ssl_level_keeping__(unsigned int n, unsigned int c)
{
    return !ra6m1_spcmd[n][c].bits_set(SSLKP);
}
// SPI Data Length
inline void ra6m1_spi_cmd_set_data_length__(unsigned int n, unsigned int c, unsigned char length)
{
    /**
     * 0b0100 to 0b0111: 8 bits
     * 0b1000          : 9 bits
     * 0b1001          : 10 bits
     * 0b1010          : 11 bits
     * 0b1011          : 12 bits
     * 0b1100          : 13 bits
     * 0b1101          : 14 bits
     * 0b1110          : 15 bits
     * 0b1111          : 16 bits
     * 0b0000          : 20 bits
     * 0b0001          : 24 bits
     * 0b0010,
     * 0b0011          : 32 bits
     */
    unsigned char bits = 0;
    if (length <= 16)
    {
	bits = length - 1;
    }
    else if (length == 20)
    {
	bits = 0;
    }
    else if (length == 24)
    {
	bits = 1;
    }
    else
    {
	// Assume 32 bits is the only other value that will be passed in.
	bits = 2;
    }
    unsigned short temp = *(ra6m1_spcmd[n][c]);
    temp &= ~(0b1111 << 8);
}
// SPI LSB First
#define LSBF  (1U << 12)
inline void ra6m1_spi_cmd_set_lsb_first__(unsigned int n, unsigned int c, bool lsb_first)
{
    ra6m1_spcmd[n][c].set_or_clear(LSBF, lsb_first);
}
inline bool ra6m1_spi_cmd_get_lsb_first__(unsigned int n, unsigned int c)
{
    return ra6m1_spcmd[n][c].bits_set(LSBF);
}
// SPI Next-Access Delay Enable
#define SPNDEN  (1U << 13)
inline void ra6m1_spi_cmd_set_next_access_delay_enable__(unsigned int n, unsigned int c, bool enabled)
{
    ra6m1_spcmd[n][c].set_or_clear(SPNDEN, enabled);
}
inline bool ra6m1_spi_cmd_get_next_access_delay_enable__(unsigned int n, unsigned int c)
{
    return ra6m1_spcmd[n][c].bits_set(SPNDEN);
}
// SSL Negation Delay Setting Enable
#define SLNDEN  (1U << 14)
inline void ra6m1_spi_cmd_set_ssl_negation_delay_enable__(unsigned int n, unsigned int c, bool enabled)
{
    ra6m1_spcmd[n][c].set_or_clear(SLNDEN, enabled);
}
inline bool ra6m1_spi_cmd_get_ssl_negation_delay_enable__(unsigned int n, unsigned int c)
{
    return ra6m1_spcmd[n][c].bits_set(SLNDEN);
}
// RSPCK Delay Setting Enable
#define SCKDEN  (1U << 15)
inline void ra6m1_spi_cmd_set_rspck_delay_enable__(unsigned int n, unsigned int c, bool enabled)
{
    ra6m1_spcmd[n][c].set_or_clear(SCKDEN, enabled);
}
inline bool ra6m1_spi_cmd_get_rspck_delay_enable__(unsigned int n, unsigned int c)
{
    return ra6m1_spcmd[n][c].bits_set(SCKDEN);
}

// SPI Data Control Register 2
#define RA6M1_SPDCR2(n)  (0x40072020UL + (((unsigned long)(n)) * 0x100UL))
static Reg8 ra6m1_spdcr2[2] =
{
    { RA6M1_SPDCR2(0) },
    { RA6M1_SPDCR2(1) }
};
// Byte Swap Operating Mode Select
#define BYSW  (1U)
inline void ra6m1_spi_set_byte_swap_enable__(unsigned int n, bool enabled)
{
    ra6m1_spdcr2[n].set_or_clear(BYSW, enabled);
}
inline bool ra6m1_spi_get_byte_swap_enable__(unsigned int n)
{
    return ra6m1_spdcr2[n].bits_set(BYSW);
}

#endif // RA6M1_SPI_HH_

