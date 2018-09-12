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

#ifndef _SAMD_I2C_HH
#define _SAMD_I2C_HH

#include "atsamd21/atsamd21.hh"

// I2C master configuration registers

#define I2C_CTRLA        (I2C + 0x00UL)
#define I2C_CTRLB        (I2C + 0x04UL)
#define I2C_BAUD         (I2C + 0x0CUL)
#define I2C_INTENCLR     (I2C + 0x14UL)
#define I2C_INTENSET     (I2C + 0x16UL)
#define I2C_INTFLAG      (I2C + 0x18UL)
#define I2C_STATUS       (I2C + 0x1AUL)
#define I2C_SYNCBUSY     (I2C + 0x1CUL)
#define I2C_ADDR         (I2C + 0x24UL)
#define I2C_DATA         (I2C + 0x28UL)
#define I2C_DBGCTRL      (I2C + 0x30UL)
#define I2C_FIFOSPACE    (I2C + 0x34UL)
#define I2C_FIFOPTR      (I2C + 0x36UL)

void samd_enable_i2c_master(unsigned int, unsigned int);
void samd_init_i2c();


#endif  // _SAMD_I2C_HH
