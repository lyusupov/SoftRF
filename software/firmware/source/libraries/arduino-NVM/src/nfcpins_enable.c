/*
 * Copyright (c) 2016-2023 Makerdiary
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(NRF52840) || defined(NRF52840_XXAA)

#include <nrf.h>
#include <stddef.h>

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

int nfcpins_enable(void)
{
    /* if NFCPINS in UICR is disabled, NFC functionality will be disabled.
     * To enable NFC, NFCPINS in UICR should be erased.
     * This function will erase NFCPINS and keep other registers in UICR unchanged.
     */

    NRF_UICR_Type nrf_uicr;
    volatile uint32_t *addr = NULL;

    if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) != (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)){

        addr = NRF_UICR->NRFFW;
        for (int i = 0; i < ARRAY_SIZE(nrf_uicr.NRFFW); i++) {
            nrf_uicr.NRFFW[i] = *addr;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                ;
            }
            addr++;
        }

        addr = NRF_UICR->NRFHW;
        for (int i = 0; i < ARRAY_SIZE(nrf_uicr.NRFHW); i++) {
            nrf_uicr.NRFHW[i] = *addr;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                ;
            }
            addr++;
        }

        addr = NRF_UICR->CUSTOMER;
        for (int i = 0; i < ARRAY_SIZE(nrf_uicr.CUSTOMER); i++) {
            nrf_uicr.CUSTOMER[i] = *addr;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                ;
            }
            addr++;
        }

        nrf_uicr.PSELRESET[0] = NRF_UICR->PSELRESET[0];
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        nrf_uicr.PSELRESET[1] = NRF_UICR->PSELRESET[1];
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        nrf_uicr.APPROTECT = NRF_UICR->APPROTECT;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        nrf_uicr.NFCPINS = NRF_UICR->NFCPINS;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        nrf_uicr.DEBUGCTRL = NRF_UICR->DEBUGCTRL;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        nrf_uicr.REGOUT0 = NRF_UICR->REGOUT0;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        // Enable Erase mode
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos; // 0x02; 
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        // Erase the UICR registers
        NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase << NVMC_ERASEUICR_ERASEUICR_Pos; //0x00000001;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        // Enable WRITE mode
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
            ;
        }

        addr = NRF_UICR->NRFFW;
        for (int i = 0; i < ARRAY_SIZE(nrf_uicr.NRFFW); i++) {
            if (nrf_uicr.NRFFW[i] != 0xFFFFFFFF) {
                *addr = nrf_uicr.NRFFW[i];
                while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                    ;
                }
            }
            addr++;
        }

        addr = NRF_UICR->NRFHW;
        for (int i = 0; i < ARRAY_SIZE(nrf_uicr.NRFHW); i++) {
            if (nrf_uicr.NRFHW[i] != 0xFFFFFFFF) {
                *addr = nrf_uicr.NRFHW[i];
                while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                    ;
                }
            }
            addr++;
        }

        addr = NRF_UICR->CUSTOMER;
        for (int i = 0; i < ARRAY_SIZE(nrf_uicr.CUSTOMER); i++) {
            if (nrf_uicr.CUSTOMER[i] != 0xFFFFFFFF) {
                *addr = nrf_uicr.CUSTOMER[i];
                while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                    ;
                }
            }
            addr++;
        }

        if (nrf_uicr.PSELRESET[0] != 0xFFFFFFFF) {
            NRF_UICR->PSELRESET[0] = nrf_uicr.PSELRESET[0];
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                ;
            }
        }

        if (nrf_uicr.PSELRESET[1] != 0xFFFFFFFF) {
            NRF_UICR->PSELRESET[1] = nrf_uicr.PSELRESET[1];
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                ;
            }
        }

        if (nrf_uicr.APPROTECT != 0xFFFFFFFF) {
            NRF_UICR->APPROTECT = nrf_uicr.APPROTECT;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                ;
            }
        }

        if (nrf_uicr.DEBUGCTRL != 0xFFFFFFFF) {
            NRF_UICR->DEBUGCTRL = nrf_uicr.DEBUGCTRL;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                ;
            }
        }

        if (nrf_uicr.REGOUT0 != 0xFFFFFFFF) {
            NRF_UICR->REGOUT0 = nrf_uicr.REGOUT0;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
                ;
            }
        }
        
        /* a reset is required for changes to take effect */
        NVIC_SystemReset();
    }

    return 0;
}

#endif /* NRF52840 */

