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

#define CODE_BASE         (xglue(CHIP_NAME, _CODE_BASE))
#define SRAM_BASE         (xglue(CHIP_NAME, _SRAM_BASE))
#define PERIPHERALS_BASE  (xglue(CHIP_NAME, _PERHIPHERALS_BASE))
#define IOBUS_BASE        (xglue(CHIP_NAME, _IOBUS_BASE))
#define SYSTEM_BASE       (xglue(CHIP_NAME, _SYSTEM_BASE))

#define PAC0            (xglue(CHIP_NAME, _PAC0))
#define PM              (xglue(CHIP_NAME, _PM))
#define SYSCTRL		   (xglue(CHIP_NAME, _SYSCTRL))
#define GCLK   		   (xglue(CHIP_NAME, _GCLK))
#define WDT    		   (xglue(CHIP_NAME, _WDT))
#define RTC    		   (xglue(CHIP_NAME, _RTC))
#define EIC    		   (xglue(CHIP_NAME, _EIC))
#define PAC1           (xglue(CHIP_NAME, _PAC1))
#define DSU    		   (xglue(CHIP_NAME, _DSU))
#define NVMCTRL		   (xglue(CHIP_NAME, _NVMCTRL))
#define PORT   		   (xglue(CHIP_NAME, _PORT))
#define DMAC   		   (xglue(CHIP_NAME, _DMAC))
#define USB    		   (xglue(CHIP_NAME, _USB))
#define MTB    		   (xglue(CHIP_NAME, _MTB))
#define PAC2           (xglue(CHIP_NAME, _PAC2))
#define EVSYS  		   (xglue(CHIP_NAME, _EVSYS))
#define SERCOM0		   (xglue(CHIP_NAME, _SERCOM0))
#define SERCOM1		   (xglue(CHIP_NAME, _SERCOM1))
#define SERCOM2		   (xglue(CHIP_NAME, _SERCOM2))
#define SERCOM3		   (xglue(CHIP_NAME, _SERCOM3))
#define SERCOM4		   (xglue(CHIP_NAME, _SERCOM4))
#define SERCOM5		   (xglue(CHIP_NAME, _SERCOM5))
#define TCC0   		   (xglue(CHIP_NAME, _TCC0))
#define TCC1   		   (xglue(CHIP_NAME, _TCC1))
#define TCC2   		   (xglue(CHIP_NAME, _TCC2))
#define TC3    		   (xglue(CHIP_NAME, _TC3))
#define TC4    		   (xglue(CHIP_NAME, _TC4))
#define TC5    		   (xglue(CHIP_NAME, _TC5))
#define TC6    		   (xglue(CHIP_NAME, _TC6))
#define TC7    		   (xglue(CHIP_NAME, _TC7))
#define ADC    		   (xglue(CHIP_NAME, _ADC))
#define AC     		   (xglue(CHIP_NAME, _AC))
#define DAC    		   (xglue(CHIP_NAME, _DAC))
#define PTC    		   (xglue(CHIP_NAME, _PTC))
#define I2S    		   (xglue(CHIP_NAME, _I2S))
#define AC1    		   (xglue(CHIP_NAME, _AC1))

#endif  // MICROCHIP_HH_
