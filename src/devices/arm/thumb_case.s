// Copyright 2010 The Chromium OS Authors. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//    * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/* Copyright 2014 The Chromium OS Authors. All rights reserved.
 *
 * Thumb mode toolchain helpers for compact switch/case statement.
 */

.text
.syntax unified
.code 16
/*
 * Helpers for compact switch
 *
 * r0: the table index
 * lr: the table base address (need to clear bit 0)
 *
 * r0 and lr must be PRESERVED.
 * r12 can be clobbered.
 */
.section .text.__gnu_thumb1_case_uqi
.global __gnu_thumb1_case_uqi
.thumb_func
__gnu_thumb1_case_uqi:
	mov     r12, r1
	mov     r1, lr
	lsrs    r1, r1, #1
	lsls    r1, r1, #1
	ldrb    r1, [r1, r0]
	lsls    r1, r1, #1
	add     lr, lr, r1
	mov     r1, r12
	bx      lr
.section .text.__gnu_thumb1_case_sqi
.global __gnu_thumb1_case_sqi
.thumb_func
__gnu_thumb1_case_sqi:
	mov     r12, r1
	mov     r1, lr
	lsrs    r1, r1, #1
	lsls    r1, r1, #1
	ldrsb   r1, [r1, r0]
	lsls    r1, r1, #1
	add     lr, lr, r1
	mov     r1, r12
	bx      lr
.section .text.__gnu_thumb1_case_uhi
.global __gnu_thumb1_case_uhi
.thumb_func
__gnu_thumb1_case_uhi:
	push    {r0, r1}
	mov     r1, lr
	lsrs    r1, r1, #1
	lsls    r0, r0, #1
	lsls    r1, r1, #1
	ldrh    r1, [r1, r0]
	lsls    r1, r1, #1
	add     lr, lr, r1
	pop     {r0, r1}
	bx      lr
.section .text.__gnu_thumb1_case_shi
.global __gnu_thumb1_case_shi
.thumb_func
__gnu_thumb1_case_shi:
	push    {r0, r1}
	mov     r1, lr
	lsrs    r1, r1, #1
	lsls    r0, r0, #1
	lsls    r1, r1, #1
	ldrsh   r1, [r1, r0]
	lsls    r1, r1, #1
	add     lr, lr, r1
	pop     {r0, r1}
	bx      lr
.section .text.__gnu_thumb1_case_si
.global __gnu_thumb1_case_si
.thumb_func
__gnu_thumb1_case_si:
	push	{r0, r1}
	mov	r1, lr
	adds.n	r1, r1, #2
	lsrs	r1, r1, #2
	lsls	r0, r0, #2
	lsls	r1, r1, #2
	ldr	r0, [r1, r0]
	adds	r0, r0, r1
	mov	lr, r0
	pop	{r0, r1}
	mov	pc, lr
