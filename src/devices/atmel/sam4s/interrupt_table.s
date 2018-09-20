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
.global hard_fault
.global mem_manage_fault
.global bus_fault
.global usage_fault
.global svcall
.global pendsv
.global systick
.global unused_interrupt
.global watchdog_interrupt
.global uart0_interrupt
.global timer_interrupt
.global udp_interrupt
.global start

interrupt_table:
.word   0x20001000                      /* SP */
.word   start + 1                       /* reset */
.word   nmi + 1                         /* nmi */
.word   hard_fault + 1          /* hard fault */
.word   mem_manage_fault + 1 /* memmanage fault */
.word   bus_fault + 1           /* bus fault */
.word   usage_fault + 1         /* usage fault */
.word   0                                       /* reserved */
.word   0                                       /* reserved */
.word   0                                       /* reserved */
.word   0                                       /* reserved */
.word   svcall + 1                      /* svcall */
.word   0                                       /* reserved */
.word   0                                       /* reserved */
.word   pendsv + 1                      /* pendsv */
.word   systick + 1                     /* systick */
.word   unused_interrupt + 1 /* 0 */
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   watchdog_interrupt + 1
.word   unused_interrupt + 1 /* 5 */
.word   unused_interrupt + 1
.word   0
.word   uart0_interrupt + 1
.word   unused_interrupt + 1
.word   0                               /* 10 */
.word   0
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1    /* 15 */
.word   0
.word   0
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1    /* 20 */
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   timer_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1    /* 25 */
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1    /* 30 */
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   unused_interrupt + 1
.word   udp_interrupt + 1

