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

#include "sam4s.hh"
#include "main.hh"

extern "C" {
    void sam_usb_reset_endpoint(const unsigned int ep) {
		Reg32 udp_rst_ep{ UDP_RST_EP };
	udp_rst_ep = (1 << ep);
	do_dmb();
	udp_rst_ep = 0;
    }

    void sam_usb_set_ep_in_dir(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	udp_csr |= (1 << 7);
    }

    void sam_usb_set_ep_out_dir(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	udp_csr &= ~(1 << 7);
    }

    void sam_usb_wait_for_tx_ready(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
		while (udp_csr & (1 << 4)) {
			wait(10);
		}
    }

    bool sam_usb_tx_completed(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
		return !!(udp_csr & 1);
    }

    void sam_usb_ack_tx_completed(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
		udp_csr &= ~1;
    }

    void sam_usb_set_tx_ready(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
		udp_csr |= (1 << 4);
    }

    void sam_usb_load_from_fifo(const unsigned int ep, char * const data,
				const unsigned int length) {
		Reg32 udp_fdr{ UDP_FDR(ep) };
	for (unsigned int i = 0; i < length; i++) {
	    data[i] = char(udp_fdr & 0xff);
	}
    }
    
    void sam_usb_load_to_fifo(const unsigned int ep, const char * const data,
			      const unsigned int length) {
		Reg32 udp_fdr{ UDP_FDR(ep) };
	for (unsigned int i = 0; i < length; i++) {
	    udp_fdr = data[i];
	}
    }

    bool sam_usb_out_rxd(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	return !!((udp_csr & (1 << 6)) | (udp_csr & (1 << 1)));
    }
    
    void sam_usb_ack_rx_completed(const unsigned int ep, const bool bank) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	const char bit = bank ? 6 : 1;
	udp_csr &= ~(1 << bit);
    }

    bool sam_usb_setup_rxd(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	return udp_csr & (1 << 2);
    }

    void sam_usb_ack_setup_completed(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	udp_csr &= ~(1 << 2);
    }
    
    void sam_usb_stall_ep(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	udp_csr |= (1 << 5);
	wait(15);
    }

    bool sam_usb_ep_stalled(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	return !!(udp_csr & (1 << 3));
    }
    
    void sam_usb_clear_ep_stall(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	udp_csr &= ~(1 << 3);
    }
    
    unsigned int sam_usb_out_token_length(const unsigned int ep) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	return (udp_csr & 0x7ff0000) >> 16;
    }

    unsigned int sam_usb_setup_token_length(const unsigned int ep) {
	return sam_usb_out_token_length(ep);
    }

    void sam_usb_set_address(const unsigned int address) {
	Reg32 udp_glb_stat{ UDP_GLB_STAT };
	Reg32 udp_faddr{ UDP_FADDR };

	do_dmb();
	udp_faddr = address;
	
	if (address != 0) {
		do_dmb();
		udp_faddr |= (1 << 8);
		do_dmb();
		udp_glb_stat = 1;
		do_dmb();
	}
    }

    void sam_usb_configure_endpoint(const unsigned int ep, const bool dir,
				    const unsigned char type, const bool enabled) {
		Reg32 udp_csr{ UDP_CSR(ep) };
	do_dmb();
	udp_csr = (enabled << 15) | (((dir << 2) | type) << 8);
    }

    void sam_usb_set_configured(const bool configured) {
	Reg32 udp_glb_stat{ UDP_GLB_STAT };
	udp_glb_stat |= (1 << configured);
    }

    void sam_usb_init(const unsigned int enabled_eps) {
	Reg32 ccfg_sysio{ CCFG_SYSIO };
	Reg32 pmc_pcer1{ PMC_PCER1 };
	Reg32 udp_txvc{ UDP_TXVC };
	Reg32 udp_ier{ UDP_IER };
	Reg32 udp_rst_ep{ UDP_RST_EP };
	Reg32 udp_faddr{ UDP_FADDR };
	Reg32 piob_pdr{ PIO_PDR(1) };
	
	pmc_pcer1 = (1 << 2); /* enable USB peripheral clock */
	ccfg_sysio &= ~((1 << 11) | (1 << 10)); /* enable USB lines for UDP */
	do_dmb();
	udp_txvc = 0;
	do_dmb();
	udp_txvc = (1 << 9); /* enable the pull-up resistor on DDP */
	udp_ier = (1 << 12) | enabled_eps | 1; /* enable EP and ENDBUSRES interrupts */
	udp_faddr = 0;
	piob_pdr = (1 << 11) | (1 << 10);  // set USB lines as peripheral controlled
    }

    void sam_usb_enter_default_state(const unsigned int endpoints) {
		Reg32 udp_csr0{ UDP_CSR(0) };
		Reg32 udp_txvc{ UDP_TXVC };
		Reg32 udp_ier{ UDP_IER };
		Reg32 udp_rst_ep{ UDP_RST_EP };
		Reg32 udp_faddr{ UDP_FADDR };

	udp_rst_ep = endpoints | 1; /* reset endpoints */
	udp_rst_ep = 0; /* clear reset bits */

	udp_ier = (1 << 12) | endpoints | 1; /* enable EP and ENDBUSRES interrupts */
	udp_faddr = (1 << 8); /* enable the function */
	udp_faddr &= (1 << 8); /* clear the function's address */

	udp_csr0 = (1 << 15); /* enable default control endpoint */
	wait(15);
	udp_txvc = (1 << 9); /* make sure the transceiver is enabled */
    }

    void sam_usb_enable() {
		Reg32 udp_csr0{ UDP_CSR(0) };
		Reg32 udp_txvc{ UDP_TXVC };
		Reg32 udp_ier{ UDP_IER };
		Reg32 udp_faddr{ UDP_FADDR };

	udp_ier |= (1 << 12) | 1; /* enable EP and ENDBUSRES interrupts */
	udp_faddr = (1 << 8); /* enable the function */
	udp_faddr &= (1 << 8); /* clear the function's address */

	udp_csr0 = (1 << 15); /* enable default control endpoint */
	wait(15);
	udp_txvc = (1 << 9); /* make sure the transceiver is enabled */
    }
}
