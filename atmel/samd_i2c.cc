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

#define I2C SERCOM0

#include "samd_i2c.hh"
#include "i2c.hh"
#include "main.hh"

void samd_reset_i2c()
{
	Reg32 ctrla{ I2C_CTRLA };
	ctrla = 1;
	Reg32 syncbusy{ I2C_SYNCBUSY };
	while (syncbusy & 1);
}

void __samd_set_i2c_hs_baud(unsigned char hs_baud_high, unsigned char hs_baud_low)
{
	Reg32 baud{ I2C_BAUD };
	baud = ((hs_baud_low & 0xFF) << 24) | ((hs_baud_high & 0xFF) << 16);
}
void __samd_set_i2c_baud(unsigned char baud_high, unsigned char baud_low)
{
	Reg32 baud{ I2C_BAUD };
	baud = ((baud_low & 0xFF) << 8) | (baud_high & 0xFF);
}
void samd_set_i2c_baud_rate(unsigned int f_scl_hz, unsigned int f_gclk_hz)
{
	Reg32 ctrla{ I2C_CTRLA };
	if (f_scl_hz <= 1000000) {
		if (f_scl_hz > 400000) {
			// CTRLA.SPEED = 1 => Fast-mode Plus (Fm+) up to 1MHz
			ctrla |= 0x1 << 24;
		}
		// f_SCL = f_GCLK / (10 + 2BAUD + f_GCLK*T_rise)
		// (10 + 2BAUD + f_GCLK*T_rise)f_SCL = f_GCLK
		// 10 + 2BAUD + f_GCLK*T_rise = f_GCLK / f_SCL
		// 2BAUD = (f_GCLK / f_SCL) - 10 - f_GCLK*T_rise
		// BAUD = ((f_GCLK / f_SCL) - 10 - f_GCLK*T_rise) / 2
		// Assume that T_rise is negligible
		unsigned int baud_rate = ((unsigned int)(f_gclk_hz / f_scl_hz) - 10) / 2;
		__samd_set_i2c_baud(baud_rate & 0xFF, 0);
	}
	else if (f_scl_hz > 1000000) {
		//CTRLA.SPEED = 2 => High-speed mode (Hs) up to 3.4MHz
		ctrla |= 0x2 << 24;
		// I2C Hs mode
		// f_SCL = f_GCLK / (2 + 2HS_BAUD)
		// (2 + 2HS_BAUD)f_SCL = f_GCLK
		// (2 + 2HS_BAUD) = f_GCLK / f_SCL
		// HS_BAUD = ((f_GCLK / f_SCL) - 2) / 2
		unsigned int hs_baud = ((unsigned int)(f_gclk_hz / f_scl_hz) - 2) / 2;
		__samd_set_i2c_hs_baud(hs_baud & 0xFF, 0);
	}
}

void samd_init_i2c()
{
	// Setup GCLK_SERCOM0_CORE
	setup_gclk(GCLKGEN0, GCLK_SERCOM0_CORE);
	samd_enable_i2c_master(1000000, 48000000);
}

void samd_enable_i2c_master(unsigned int f_scl_hz, unsigned int f_gclk_hz)
{
	// Make sure i2c is currently disabled.  Most settings are enable-protected.
	samd_reset_i2c();
	Reg32 ctrla{ I2C_CTRLA };
	// CTRLA.MODE = 0x5 => i2c master
	// CTRLA.SCLSM = 1 => stretch SCL only after ACK bit.
	ctrla = (1 << 27) | (0x5 << 2);
	samd_set_i2c_baud_rate(f_scl_hz, f_gclk_hz);
	// Enable i2c.
	// CTRLA.ENABLE = 1
	ctrla |= (1 << 1);
	// Wait for synchronization to complete.
	Reg32 syncbusy{ I2C_SYNCBUSY };
	while ((syncbusy & (1 << 1)) != 0);
	// Take control of the bus
	Reg16 status{ I2C_STATUS };
	status = (status & ~0x0030) | 0x1;
}

#define CTRLB_WR_MASK ((1 << 18) | (1 << 9) | (1 << 8))
void __samd_i2c_clear_fifos(unsigned char x)
{
	Reg32 ctrlb{ I2C_CTRLB };
	ctrlb = (ctrlb & CTRLB_WR_MASK) | (x << 22);
}
void samd_i2c_clear_tx_fifo()
{
	__samd_i2c_clear_fifos(0x1);
}
void samd_i2c_clear_rx_fifo()
{
	__samd_i2c_clear_fifos(0x2);
}
void samd_i2c_clear_both_fifos()
{
	__samd_i2c_clear_fifos(0x3);
}
void samd_i2c_send_ack()
{
	constexpr unsigned int ACKACT = 1 << 18;
	Reg32 ctrlb{ I2C_CTRLB };
	ctrlb = (ctrlb & CTRLB_WR_MASK) | ACKACT;
}
unsigned short samd_i2c_get_address()
{
	Reg32 addr{ I2C_ADDR };
	return (addr & 0x7FE) >> 1;
}
// Writing to ADDR.ADDR while the bus state is OWNER will result
// in the I2C master issuing a repeated start sequence.  This can
// only be done while INTFLAG.MB or INTFLAG.SB is set.
void samd_i2c_set_address(unsigned short address)
{
	Reg32 addr{ I2C_ADDR };
	unsigned int addr_value{ addr };
	// Bit 0 of ADDR.ADDR is used as the R/W direction flag.
	bool rw{ addr_value & 0x1 };
	addr = (addr_value & ~0x7FF) | (address & 0x3FF) | rw;
}

void __samd_i2c_send_repeat_start()
{
	Reg32 ctrlb{ I2C_CTRLB };
	ctrlb = (ctrlb & CTRLB_WR_MASK) | (0x1 << 16);
}
void samd_i2c_send_repeat_start(unsigned short address)
{
	if (address != samd_i2c_get_address()) {
		samd_i2c_set_address(address);
	}
	else {
		__samd_i2c_send_repeat_start();
	}
}
void samd_i2c_send_stop()
{
	Reg32 ctrlb{ I2C_CTRLB };
	ctrlb = (ctrlb & CTRLB_WR_MASK) | (0x3 << 16);
}

#define I2C_UNKNOWN (0x00)
#define I2C_IDLE    (0x01)
#define I2C_OWNER   (0x02)
#define I2C_BUSY    (0x03)

unsigned char samd_i2c_get_bus_state()
{
	Reg16 status{ I2C_STATUS };
	return (status & 0x0030) >> 4;
}

bool samd_i2c_rx_acked()
{
	Reg16 status{ I2C_STATUS };
	return !(status & (1 << 2));
}

bool samd_i2c_arb_lost()
{
	Reg16 status{ I2C_STATUS };
	return status & (1 << 1);
}

bool samd_i2c_bus_err()
{
	Reg16 status{ I2C_STATUS };
	return status & 1;
}

bool samd_i2c_err()
{
	return samd_i2c_arb_lost() || samd_i2c_bus_err();
}

#undef CTRLB_WR_MASK

inline bool samd_i2c_get_mb_intflag()
{
	Reg8 intflag{ I2C_INTFLAG };
	return intflag & 1;
}
inline bool samd_i2c_get_sb_intflag()
{
	Reg8 intflag{ I2C_INTFLAG };
	return intflag & (1 << 1);
}

inline void samd_i2c_clear_mb_intflag()
{
	Reg8 intflag{ I2C_INTFLAG };
	intflag &= ~1;
}
inline void samd_i2c_clear_sb_intflag()
{
	Reg8 intflag{ I2C_INTFLAG };
	intflag &= ~(1 << 1);
}

void i2c_master_isr()
{
	Reg8 intflag{ I2C_INTFLAG };
	Reg16 status{ I2C_STATUS };
	Reg16 data{ I2C_DATA };
	volatile struct i2c_status_info* i2c_info{ getI2CStatusInfo() };
	
	if (samd_i2c_get_mb_intflag()) {
		// Master on bus
		if (!samd_i2c_rx_acked()) {
			samd_i2c_send_stop();
			i2c_info->error = true;
		}
		else { // RX ACKED
			if (i2c_info->tx_bytes > 0) {
				data = *(i2c_info->tx_fifo++);
				i2c_info->tx_bytes--;
			}
			else {
				samd_i2c_send_stop();
			}
			i2c_info->error = false;
		}
		samd_i2c_clear_mb_intflag();
	}
	else if (samd_i2c_get_sb_intflag()) {
		*(i2c_info->rx_fifo++) = static_cast<unsigned char>(data & 0xFF);
		i2c_info->rx_bytes--;
		if (i2c_info->rx_bytes > 0) {
			__samd_i2c_send_repeat_start();
		}
		else {
			samd_i2c_send_stop();
		}
		i2c_info->error = false;
		samd_i2c_clear_sb_intflag();
	}
}

void i2c_slave_isr()
{

}
