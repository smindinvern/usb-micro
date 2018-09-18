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

#ifndef MICROCHIP_HH_
#define MICROCHIP_HH_

#define glue(a, b)  a ## b
#define xglue(a, b) glue(a, b)

#define CODE_BASE         (xglue(CHIP_FAMILY, _CODE_BASE))
#define SRAM_BASE         (xglue(CHIP_FAMILY, _SRAM_BASE))
#define PERIPHERALS_BASE  (xglue(CHIP_FAMILY, _PERHIPHERALS_BASE))
#define IOBUS_BASE        (xglue(CHIP_FAMILY, _IOBUS_BASE))
#define SYSTEM_BASE       (xglue(CHIP_FAMILY, _SYSTEM_BASE))

#define PAC0            (xglue(CHIP_FAMILY, _PAC0))
#define PM              (xglue(CHIP_FAMILY, _PM))
#define SYSCTRL		   (xglue(CHIP_FAMILY, _SYSCTRL))
#define GCLK   		   (xglue(CHIP_FAMILY, _GCLK))
#define WDT    		   (xglue(CHIP_FAMILY, _WDT))
#define RTC    		   (xglue(CHIP_FAMILY, _RTC))
#define EIC    		   (xglue(CHIP_FAMILY, _EIC))
#define PAC1           (xglue(CHIP_FAMILY, _PAC1))
#define DSU    		   (xglue(CHIP_FAMILY, _DSU))
#define NVMCTRL		   (xglue(CHIP_FAMILY, _NVMCTRL))
#define PORT   		   (xglue(CHIP_FAMILY, _PORT))
#define DMAC   		   (xglue(CHIP_FAMILY, _DMAC))
#define USB    		   (xglue(CHIP_FAMILY, _USB))
#define MTB    		   (xglue(CHIP_FAMILY, _MTB))
#define PAC2           (xglue(CHIP_FAMILY, _PAC2))
#define EVSYS  		   (xglue(CHIP_FAMILY, _EVSYS))
#define SERCOM0		   (xglue(CHIP_FAMILY, _SERCOM0))
#define SERCOM1		   (xglue(CHIP_FAMILY, _SERCOM1))
#define SERCOM2		   (xglue(CHIP_FAMILY, _SERCOM2))
#define SERCOM3		   (xglue(CHIP_FAMILY, _SERCOM3))
#define SERCOM4		   (xglue(CHIP_FAMILY, _SERCOM4))
#define SERCOM5		   (xglue(CHIP_FAMILY, _SERCOM5))
#define TCC0   		   (xglue(CHIP_FAMILY, _TCC0))
#define TCC1   		   (xglue(CHIP_FAMILY, _TCC1))
#define TCC2   		   (xglue(CHIP_FAMILY, _TCC2))
#define TC3    		   (xglue(CHIP_FAMILY, _TC3))
#define TC4    		   (xglue(CHIP_FAMILY, _TC4))
#define TC5    		   (xglue(CHIP_FAMILY, _TC5))
#define TC6    		   (xglue(CHIP_FAMILY, _TC6))
#define TC7    		   (xglue(CHIP_FAMILY, _TC7))
#define ADC    		   (xglue(CHIP_FAMILY, _ADC))
#define AC     		   (xglue(CHIP_FAMILY, _AC))
#define DAC    		   (xglue(CHIP_FAMILY, _DAC))
#define PTC    		   (xglue(CHIP_FAMILY, _PTC))
#define I2S    		   (xglue(CHIP_FAMILY, _I2S))
#define AC1    		   (xglue(CHIP_FAMILY, _AC1))

#endif  // MICROCHIP_HH_
