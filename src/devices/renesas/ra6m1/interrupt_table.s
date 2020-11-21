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

	.global nmi
	.global	hard_fault
	.global mem_manage_fault
	.global bus_fault
	.global usage_fault
	.global svcall
	.global pendsv
	.global systick
	.global usb_interrupt
	.global unused_interrupt
	.global start

	.section .vector_table
interrupt_table:
	.word	0x20001000
	.word 	start + 1
	.word	nmi + 1
	.word	hard_fault + 1
	.word	mem_manage_fault + 1
	.word	bus_fault + 1
	.word	usage_fault + 1
	.word	0
	.word	0
	.word	0
	.word	0
	.word	svcall + 1
	.word	0
	.word	0
	.word	pendsv + 1
	.word	systick + 1
	.word	usb_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1
	.word	unused_interrupt + 1

	.section .option_rom
	
	.word 0xFFFFFFFF	// OFS0 - disable WDT and IWDT
	.word 0xFFFFF9FF	// OFS1 - disable HOCO and LVD
	.word 0x00000000	// SECMPUPCS0
	.word 0x00000000	// SECMPUPCS1
	.word 0xFFFFFFFF	// SECMPUPCE0
	.word 0xFFFFFFFF	// SECMPUPCE1
	.word 0x00000000	// SECMPUS0
	.word 0x00FFFFFF	// SECMPUE0
	.word 0x20000000	// SECMPUS1
	.word 0x200FFFFF	// SECMPUE1
	.word 0x400C0000	// SECMPUS2
	.word 0x400DFFFF	// SECMPUE2
	.word 0x40100000	// SECMPUS3
	.word 0x407FFFFF	// SECMPUE3
	.word 0xFFFFFFFF	// SECMPUAC - disable all regions
	
	
