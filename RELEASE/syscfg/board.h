/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOARD_H
#define BOARD_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// ANALOG -> myANALOGPinMux0 Pinmux
//

//
// EPWM4 -> BOOST1 Pinmux
//
//
// EPWM4_A - GPIO Settings
//
#define GPIO_PIN_EPWM4_A 6
#define BOOST1_EPWMA_GPIO 6
#define BOOST1_EPWMA_PIN_CONFIG GPIO_6_EPWM4_A
//
// EPWM4_B - GPIO Settings
//
#define GPIO_PIN_EPWM4_B 7
#define BOOST1_EPWMB_GPIO 7
#define BOOST1_EPWMB_PIN_CONFIG GPIO_7_EPWM4_B

//
// EPWM2 -> BOOST2 Pinmux
//
//
// EPWM2_A - GPIO Settings
//
#define GPIO_PIN_EPWM2_A 2
#define BOOST2_EPWMA_GPIO 2
#define BOOST2_EPWMA_PIN_CONFIG GPIO_2_EPWM2_A
//
// EPWM2_B - GPIO Settings
//
#define GPIO_PIN_EPWM2_B 3
#define BOOST2_EPWMB_GPIO 3
#define BOOST2_EPWMB_PIN_CONFIG GPIO_3_EPWM2_B

//
// EPWM1 -> BOOST3 Pinmux
//
//
// EPWM1_A - GPIO Settings
//
#define GPIO_PIN_EPWM1_A 0
#define BOOST3_EPWMA_GPIO 0
#define BOOST3_EPWMA_PIN_CONFIG GPIO_0_EPWM1_A
//
// EPWM1_B - GPIO Settings
//
#define GPIO_PIN_EPWM1_B 1
#define BOOST3_EPWMB_GPIO 1
#define BOOST3_EPWMB_PIN_CONFIG GPIO_1_EPWM1_B

//
// EPWM3 -> BOOST4 Pinmux
//
//
// EPWM3_A - GPIO Settings
//
#define GPIO_PIN_EPWM3_A 4
#define BOOST4_EPWMA_GPIO 4
#define BOOST4_EPWMA_PIN_CONFIG GPIO_4_EPWM3_A
//
// EPWM3_B - GPIO Settings
//
#define GPIO_PIN_EPWM3_B 5
#define BOOST4_EPWMB_GPIO 5
#define BOOST4_EPWMB_PIN_CONFIG GPIO_5_EPWM3_B

//
// EPWM5 -> LLC1 Pinmux
//
//
// EPWM5_A - GPIO Settings
//
#define GPIO_PIN_EPWM5_A 8
#define LLC1_EPWMA_GPIO 8
#define LLC1_EPWMA_PIN_CONFIG GPIO_8_EPWM5_A
//
// EPWM5_B - GPIO Settings
//
#define GPIO_PIN_EPWM5_B 9
#define LLC1_EPWMB_GPIO 9
#define LLC1_EPWMB_PIN_CONFIG GPIO_9_EPWM5_B

//
// EPWM6 -> LLC2 Pinmux
//
//
// EPWM6_A - GPIO Settings
//
#define GPIO_PIN_EPWM6_A 10
#define LLC2_EPWMA_GPIO 10
#define LLC2_EPWMA_PIN_CONFIG GPIO_10_EPWM6_A
//
// EPWM6_B - GPIO Settings
//
#define GPIO_PIN_EPWM6_B 11
#define LLC2_EPWMB_GPIO 11
#define LLC2_EPWMB_PIN_CONFIG GPIO_11_EPWM6_B

//
// EPWM7 -> LLCSR Pinmux
//
//
// EPWM7_A - GPIO Settings
//
#define GPIO_PIN_EPWM7_A 12
#define LLCSR_EPWMA_GPIO 12
#define LLCSR_EPWMA_PIN_CONFIG GPIO_12_EPWM7_A
//
// EPWM7_B - GPIO Settings
//
#define GPIO_PIN_EPWM7_B 13
#define LLCSR_EPWMB_GPIO 13
#define LLCSR_EPWMB_PIN_CONFIG GPIO_13_EPWM7_B

//
// EPWM8 -> DCAC Pinmux
//
//
// EPWM8_A - GPIO Settings
//
#define GPIO_PIN_EPWM8_A 14
#define DCAC_EPWMA_GPIO 14
#define DCAC_EPWMA_PIN_CONFIG GPIO_14_EPWM8_A
//
// EPWM8_B - GPIO Settings
//
#define GPIO_PIN_EPWM8_B 15
#define DCAC_EPWMB_GPIO 15
#define DCAC_EPWMB_PIN_CONFIG GPIO_15_EPWM8_B
//
// GPIO59 - GPIO Settings
//
#define ISO_PWR_EN_GPIO_PIN_CONFIG GPIO_59_GPIO59
//
// GPIO19 - GPIO Settings
//
#define LED1_GPIO_PIN_CONFIG GPIO_19_GPIO19
//
// GPIO44 - GPIO Settings
//
#define uINV_DCAC_Relay_GPIO_PIN_CONFIG GPIO_44_GPIO44
//
// GPIO56 - GPIO Settings
//
#define uINV_DCAC_Top_LF_GPIO_PIN_CONFIG GPIO_56_GPIO56
//
// GPIO57 - GPIO Settings
//
#define uINV_DCAC_Bottom_LF_GPIO_PIN_CONFIG GPIO_57_GPIO57
//
// GPIO34 - GPIO Settings
//
#define ENABLE_GPIO_PIN_CONFIG GPIO_34_GPIO34
//
// GPIO17 - GPIO Settings
//
#define uINV_DCAC_TestPoint_1_GPIO_PIN_CONFIG GPIO_17_GPIO17
//
// GPIO16 - GPIO Settings
//
#define uINV_DCAC_TestPoint_2_GPIO_PIN_CONFIG GPIO_16_GPIO16
//
// GPIO31 - GPIO Settings
//
#define Shutdown_Demo_GPIO_PIN_CONFIG GPIO_31_GPIO31
//
// GPIO30 - GPIO Settings
//
#define RapidShutdown_GPIO_PIN_CONFIG GPIO_30_GPIO30
//
// GPIO58 - GPIO Settings
//
#define STATE_TP_GPIO_PIN_CONFIG GPIO_58_GPIO58

//
// SCIA -> SCI_COMMS Pinmux
//
//
// SCIA_RX - GPIO Settings
//
#define GPIO_PIN_SCIA_RX 28
#define SCI_COMMS_SCIRX_GPIO 28
#define SCI_COMMS_SCIRX_PIN_CONFIG GPIO_28_SCIA_RX
//
// SCIA_TX - GPIO Settings
//
#define GPIO_PIN_SCIA_TX 29
#define SCI_COMMS_SCITX_GPIO 29
#define SCI_COMMS_SCITX_PIN_CONFIG GPIO_29_SCIA_TX

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
#define ADC_A_BASE ADCA_BASE
#define ADC_A_RESULT_BASE ADCARESULT_BASE
#define SOC_BOOST1_CS1 ADC_SOC_NUMBER0
#define SOC_BOOST1_CS1_FORCE ADC_FORCE_SOC0
#define SOC_BOOST1_CS1_ADC_BASE ADCA_BASE
#define SOC_BOOST1_CS1_RESULT_BASE ADCARESULT_BASE
#define SOC_BOOST1_CS1_SAMPLE_WINDOW 100
#define SOC_BOOST1_CS1_TRIGGER_SOURCE ADC_TRIGGER_EPWM4_SOCA
#define SOC_BOOST1_CS1_CHANNEL ADC_CH_ADCIN6
#define SOC_BOOST1_CS2 ADC_SOC_NUMBER1
#define SOC_BOOST1_CS2_FORCE ADC_FORCE_SOC1
#define SOC_BOOST1_CS2_ADC_BASE ADCA_BASE
#define SOC_BOOST1_CS2_RESULT_BASE ADCARESULT_BASE
#define SOC_BOOST1_CS2_SAMPLE_WINDOW 100
#define SOC_BOOST1_CS2_TRIGGER_SOURCE ADC_TRIGGER_EPWM4_SOCB
#define SOC_BOOST1_CS2_CHANNEL ADC_CH_ADCIN6
#define SOC_BOOST1_VS ADC_SOC_NUMBER2
#define SOC_BOOST1_VS_FORCE ADC_FORCE_SOC2
#define SOC_BOOST1_VS_ADC_BASE ADCA_BASE
#define SOC_BOOST1_VS_RESULT_BASE ADCARESULT_BASE
#define SOC_BOOST1_VS_SAMPLE_WINDOW 100
#define SOC_BOOST1_VS_TRIGGER_SOURCE ADC_TRIGGER_EPWM4_SOCB
#define SOC_BOOST1_VS_CHANNEL ADC_CH_ADCIN5
#define SOC_BOOST2_CS1 ADC_SOC_NUMBER3
#define SOC_BOOST2_CS1_FORCE ADC_FORCE_SOC3
#define SOC_BOOST2_CS1_ADC_BASE ADCA_BASE
#define SOC_BOOST2_CS1_RESULT_BASE ADCARESULT_BASE
#define SOC_BOOST2_CS1_SAMPLE_WINDOW 100
#define SOC_BOOST2_CS1_TRIGGER_SOURCE ADC_TRIGGER_EPWM2_SOCA
#define SOC_BOOST2_CS1_CHANNEL ADC_CH_ADCIN4
#define SOC_BOOST2_CS2 ADC_SOC_NUMBER4
#define SOC_BOOST2_CS2_FORCE ADC_FORCE_SOC4
#define SOC_BOOST2_CS2_ADC_BASE ADCA_BASE
#define SOC_BOOST2_CS2_RESULT_BASE ADCARESULT_BASE
#define SOC_BOOST2_CS2_SAMPLE_WINDOW 100
#define SOC_BOOST2_CS2_TRIGGER_SOURCE ADC_TRIGGER_EPWM2_SOCB
#define SOC_BOOST2_CS2_CHANNEL ADC_CH_ADCIN4
#define SOC_BOOST2_VS ADC_SOC_NUMBER5
#define SOC_BOOST2_VS_FORCE ADC_FORCE_SOC5
#define SOC_BOOST2_VS_ADC_BASE ADCA_BASE
#define SOC_BOOST2_VS_RESULT_BASE ADCARESULT_BASE
#define SOC_BOOST2_VS_SAMPLE_WINDOW 100
#define SOC_BOOST2_VS_TRIGGER_SOURCE ADC_TRIGGER_EPWM2_SOCB
#define SOC_BOOST2_VS_CHANNEL ADC_CH_ADCIN2
void ADC_A_init();

#define ADC_B_BASE ADCB_BASE
#define ADC_B_RESULT_BASE ADCBRESULT_BASE
#define SOC_BOOST3_CS1 ADC_SOC_NUMBER0
#define SOC_BOOST3_CS1_FORCE ADC_FORCE_SOC0
#define SOC_BOOST3_CS1_ADC_BASE ADCB_BASE
#define SOC_BOOST3_CS1_RESULT_BASE ADCBRESULT_BASE
#define SOC_BOOST3_CS1_SAMPLE_WINDOW 100
#define SOC_BOOST3_CS1_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCA
#define SOC_BOOST3_CS1_CHANNEL ADC_CH_ADCIN3
#define SOC_BOOST3_CS2 ADC_SOC_NUMBER1
#define SOC_BOOST3_CS2_FORCE ADC_FORCE_SOC1
#define SOC_BOOST3_CS2_ADC_BASE ADCB_BASE
#define SOC_BOOST3_CS2_RESULT_BASE ADCBRESULT_BASE
#define SOC_BOOST3_CS2_SAMPLE_WINDOW 100
#define SOC_BOOST3_CS2_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCB
#define SOC_BOOST3_CS2_CHANNEL ADC_CH_ADCIN3
#define SOC_BOOST3_VS ADC_SOC_NUMBER2
#define SOC_BOOST3_VS_FORCE ADC_FORCE_SOC2
#define SOC_BOOST3_VS_ADC_BASE ADCB_BASE
#define SOC_BOOST3_VS_RESULT_BASE ADCBRESULT_BASE
#define SOC_BOOST3_VS_SAMPLE_WINDOW 100
#define SOC_BOOST3_VS_TRIGGER_SOURCE ADC_TRIGGER_EPWM1_SOCB
#define SOC_BOOST3_VS_CHANNEL ADC_CH_ADCIN2
#define SOC_BOOST4_CS1 ADC_SOC_NUMBER3
#define SOC_BOOST4_CS1_FORCE ADC_FORCE_SOC3
#define SOC_BOOST4_CS1_ADC_BASE ADCB_BASE
#define SOC_BOOST4_CS1_RESULT_BASE ADCBRESULT_BASE
#define SOC_BOOST4_CS1_SAMPLE_WINDOW 100
#define SOC_BOOST4_CS1_TRIGGER_SOURCE ADC_TRIGGER_EPWM3_SOCA
#define SOC_BOOST4_CS1_CHANNEL ADC_CH_ADCIN12
#define SOC_BOOST4_CS2 ADC_SOC_NUMBER4
#define SOC_BOOST4_CS2_FORCE ADC_FORCE_SOC4
#define SOC_BOOST4_CS2_ADC_BASE ADCB_BASE
#define SOC_BOOST4_CS2_RESULT_BASE ADCBRESULT_BASE
#define SOC_BOOST4_CS2_SAMPLE_WINDOW 100
#define SOC_BOOST4_CS2_TRIGGER_SOURCE ADC_TRIGGER_EPWM3_SOCB
#define SOC_BOOST4_CS2_CHANNEL ADC_CH_ADCIN12
#define SOC_BOOST4_VS ADC_SOC_NUMBER5
#define SOC_BOOST4_VS_FORCE ADC_FORCE_SOC5
#define SOC_BOOST4_VS_ADC_BASE ADCB_BASE
#define SOC_BOOST4_VS_RESULT_BASE ADCBRESULT_BASE
#define SOC_BOOST4_VS_SAMPLE_WINDOW 100
#define SOC_BOOST4_VS_TRIGGER_SOURCE ADC_TRIGGER_EPWM3_SOCB
#define SOC_BOOST4_VS_CHANNEL ADC_CH_ADCIN4
#define SOC_LLC_VS ADC_SOC_NUMBER6
#define SOC_LLC_VS_FORCE ADC_FORCE_SOC6
#define SOC_LLC_VS_ADC_BASE ADCB_BASE
#define SOC_LLC_VS_RESULT_BASE ADCBRESULT_BASE
#define SOC_LLC_VS_SAMPLE_WINDOW 100
#define SOC_LLC_VS_TRIGGER_SOURCE ADC_TRIGGER_EPWM3_SOCB
#define SOC_LLC_VS_CHANNEL ADC_CH_ADCIN5
#define SOC_IREF ADC_SOC_NUMBER7
#define SOC_IREF_FORCE ADC_FORCE_SOC7
#define SOC_IREF_ADC_BASE ADCB_BASE
#define SOC_IREF_RESULT_BASE ADCBRESULT_BASE
#define SOC_IREF_SAMPLE_WINDOW 100
#define SOC_IREF_TRIGGER_SOURCE ADC_TRIGGER_EPWM3_SOCA
#define SOC_IREF_CHANNEL ADC_CH_ADCIN11
void ADC_B_init();

#define ADC_C_BASE ADCC_BASE
#define ADC_C_RESULT_BASE ADCCRESULT_BASE
#define SOC_DC_BUS_CS1 ADC_SOC_NUMBER0
#define SOC_DC_BUS_CS1_FORCE ADC_FORCE_SOC0
#define SOC_DC_BUS_CS1_ADC_BASE ADCC_BASE
#define SOC_DC_BUS_CS1_RESULT_BASE ADCCRESULT_BASE
#define SOC_DC_BUS_CS1_SAMPLE_WINDOW 200
#define SOC_DC_BUS_CS1_TRIGGER_SOURCE ADC_TRIGGER_EPWM8_SOCA
#define SOC_DC_BUS_CS1_CHANNEL ADC_CH_ADCIN5
#define SOC_AC_CS1 ADC_SOC_NUMBER1
#define SOC_AC_CS1_FORCE ADC_FORCE_SOC1
#define SOC_AC_CS1_ADC_BASE ADCC_BASE
#define SOC_AC_CS1_RESULT_BASE ADCCRESULT_BASE
#define SOC_AC_CS1_SAMPLE_WINDOW 100
#define SOC_AC_CS1_TRIGGER_SOURCE ADC_TRIGGER_EPWM8_SOCA
#define SOC_AC_CS1_CHANNEL ADC_CH_ADCIN3
#define SOC_DC_BUS_CS2 ADC_SOC_NUMBER2
#define SOC_DC_BUS_CS2_FORCE ADC_FORCE_SOC2
#define SOC_DC_BUS_CS2_ADC_BASE ADCC_BASE
#define SOC_DC_BUS_CS2_RESULT_BASE ADCCRESULT_BASE
#define SOC_DC_BUS_CS2_SAMPLE_WINDOW 100
#define SOC_DC_BUS_CS2_TRIGGER_SOURCE ADC_TRIGGER_EPWM8_SOCB
#define SOC_DC_BUS_CS2_CHANNEL ADC_CH_ADCIN5
#define SOC_AC_CS2 ADC_SOC_NUMBER3
#define SOC_AC_CS2_FORCE ADC_FORCE_SOC3
#define SOC_AC_CS2_ADC_BASE ADCC_BASE
#define SOC_AC_CS2_RESULT_BASE ADCCRESULT_BASE
#define SOC_AC_CS2_SAMPLE_WINDOW 100
#define SOC_AC_CS2_TRIGGER_SOURCE ADC_TRIGGER_EPWM8_SOCB
#define SOC_AC_CS2_CHANNEL ADC_CH_ADCIN3
#define SOC_AC_VS ADC_SOC_NUMBER4
#define SOC_AC_VS_FORCE ADC_FORCE_SOC4
#define SOC_AC_VS_ADC_BASE ADCC_BASE
#define SOC_AC_VS_RESULT_BASE ADCCRESULT_BASE
#define SOC_AC_VS_SAMPLE_WINDOW 200
#define SOC_AC_VS_TRIGGER_SOURCE ADC_TRIGGER_EPWM8_SOCA
#define SOC_AC_VS_CHANNEL ADC_CH_ADCIN7
#define SOC_DC_BUS_VS ADC_SOC_NUMBER5
#define SOC_DC_BUS_VS_FORCE ADC_FORCE_SOC5
#define SOC_DC_BUS_VS_ADC_BASE ADCC_BASE
#define SOC_DC_BUS_VS_RESULT_BASE ADCCRESULT_BASE
#define SOC_DC_BUS_VS_SAMPLE_WINDOW 416.6666666666667
#define SOC_DC_BUS_VS_TRIGGER_SOURCE ADC_TRIGGER_EPWM8_SOCB
#define SOC_DC_BUS_VS_CHANNEL ADC_CH_ADCIN11
#define SOC_TEMP ADC_SOC_NUMBER6
#define SOC_TEMP_FORCE ADC_FORCE_SOC6
#define SOC_TEMP_ADC_BASE ADCC_BASE
#define SOC_TEMP_RESULT_BASE ADCCRESULT_BASE
#define SOC_TEMP_SAMPLE_WINDOW 100
#define SOC_TEMP_TRIGGER_SOURCE ADC_TRIGGER_SW_ONLY
#define SOC_TEMP_CHANNEL ADC_CH_ADCIN1
void ADC_C_init();


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// CLB Configurations
//
//*****************************************************************************
#define myCLB0_BASE CLB1_BASE
void myCLB0_init();

//*****************************************************************************
//
// CMPSS Configurations
//
//*****************************************************************************
#define myCMPSS_BASE CMPSS2_BASE
#define myCMPSS_HIGH_COMP_BASE CMPSS2_BASE    
#define myCMPSS_LOW_COMP_BASE CMPSS2_BASE    
void myCMPSS_init();
#define CMPSS_SR_BASE CMPSS3_BASE
#define CMPSS_SR_HIGH_COMP_BASE CMPSS3_BASE    
#define CMPSS_SR_LOW_COMP_BASE CMPSS3_BASE    
void CMPSS_SR_init();

//*****************************************************************************
//
// ECAP Configurations
//
//*****************************************************************************
#define myECAP0_BASE ECAP1_BASE
#define myECAP0_SIGNAL_MUNIT_BASE ECAP1SIGNALMONITORING_BASE
void myECAP0_init();

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
#define BOOST1_BASE EPWM4_BASE
#define BOOST1_TBPRD 250
#define BOOST1_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define BOOST1_TBPHS 2
#define BOOST1_CMPA 0
#define BOOST1_CMPB 0
#define BOOST1_CMPC 0
#define BOOST1_CMPD 0
#define BOOST1_DBRED 2
#define BOOST1_DBFED 2
#define BOOST1_TZA_ACTION EPWM_TZ_ACTION_LOW
#define BOOST1_TZB_ACTION EPWM_TZ_ACTION_LOW
#define BOOST1_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define BOOST2_BASE EPWM2_BASE
#define BOOST2_TBPRD 250
#define BOOST2_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define BOOST2_TBPHS 2
#define BOOST2_CMPA 0
#define BOOST2_CMPB 0
#define BOOST2_CMPC 0
#define BOOST2_CMPD 0
#define BOOST2_DBRED 2
#define BOOST2_DBFED 2
#define BOOST2_TZA_ACTION EPWM_TZ_ACTION_LOW
#define BOOST2_TZB_ACTION EPWM_TZ_ACTION_LOW
#define BOOST2_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define BOOST3_BASE EPWM1_BASE
#define BOOST3_TBPRD 250
#define BOOST3_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define BOOST3_TBPHS 2
#define BOOST3_CMPA 0
#define BOOST3_CMPB 0
#define BOOST3_CMPC 0
#define BOOST3_CMPD 0
#define BOOST3_DBRED 2
#define BOOST3_DBFED 2
#define BOOST3_TZA_ACTION EPWM_TZ_ACTION_LOW
#define BOOST3_TZB_ACTION EPWM_TZ_ACTION_LOW
#define BOOST3_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define BOOST4_BASE EPWM3_BASE
#define BOOST4_TBPRD 250
#define BOOST4_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define BOOST4_TBPHS 2
#define BOOST4_CMPA 0
#define BOOST4_CMPB 0
#define BOOST4_CMPC 0
#define BOOST4_CMPD 0
#define BOOST4_DBRED 2
#define BOOST4_DBFED 2
#define BOOST4_TZA_ACTION EPWM_TZ_ACTION_LOW
#define BOOST4_TZB_ACTION EPWM_TZ_ACTION_LOW
#define BOOST4_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define LLC1_BASE EPWM5_BASE
#define LLC1_TBPRD 120
#define LLC1_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define LLC1_TBPHS 0
#define LLC1_CMPA 0
#define LLC1_CMPB 0
#define LLC1_CMPC 0
#define LLC1_CMPD 0
#define LLC1_DBRED 3
#define LLC1_DBFED 3
#define LLC1_TZA_ACTION EPWM_TZ_ACTION_LOW
#define LLC1_TZB_ACTION EPWM_TZ_ACTION_LOW
#define LLC1_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define LLC2_BASE EPWM6_BASE
#define LLC2_TBPRD 120
#define LLC2_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define LLC2_TBPHS 120
#define LLC2_CMPA 0
#define LLC2_CMPB 0
#define LLC2_CMPC 0
#define LLC2_CMPD 0
#define LLC2_DBRED 3
#define LLC2_DBFED 3
#define LLC2_TZA_ACTION EPWM_TZ_ACTION_LOW
#define LLC2_TZB_ACTION EPWM_TZ_ACTION_LOW
#define LLC2_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define LLCSR_BASE EPWM7_BASE
#define LLCSR_TBPRD 120
#define LLCSR_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define LLCSR_TBPHS 60
#define LLCSR_CMPA 0
#define LLCSR_CMPB 120
#define LLCSR_CMPC 0
#define LLCSR_CMPD 0
#define LLCSR_DBRED 12
#define LLCSR_DBFED 12
#define LLCSR_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define LLCSR_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define LLCSR_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define DCAC_BASE EPWM8_BASE
#define DCAC_TBPRD 500
#define DCAC_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define DCAC_TBPHS 0
#define DCAC_CMPA 0
#define DCAC_CMPB 0
#define DCAC_CMPC 0
#define DCAC_CMPD 0
#define DCAC_DBRED 10
#define DCAC_DBFED 10
#define DCAC_TZA_ACTION EPWM_TZ_ACTION_LOW
#define DCAC_TZB_ACTION EPWM_TZ_ACTION_LOW
#define DCAC_INTERRUPT_SOURCE EPWM_INT_TBCTR_PERIOD

//*****************************************************************************
//
// EPWMXBAR Configurations
//
//*****************************************************************************
void myEPWMXBAR0_init();
#define myEPWMXBAR0 XBAR_TRIP5
#define myEPWMXBAR0_ENABLED_MUXES (XBAR_MUX02)
void myEPWMXBAR1_SRTRIP_P_init();
#define myEPWMXBAR1_SRTRIP_P XBAR_TRIP11
#define myEPWMXBAR1_SRTRIP_P_ENABLED_MUXES (XBAR_MUX05)
void myEPWMXBAR1_SRTRIP_N_init();
#define myEPWMXBAR1_SRTRIP_N XBAR_TRIP12
#define myEPWMXBAR1_SRTRIP_N_ENABLED_MUXES (XBAR_MUX04)

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
#define ISO_PWR_EN 59
void ISO_PWR_EN_init();
#define LED1 19
void LED1_init();
#define uINV_DCAC_Relay 44
void uINV_DCAC_Relay_init();
#define uINV_DCAC_Top_LF 56
void uINV_DCAC_Top_LF_init();
#define uINV_DCAC_Bottom_LF 57
void uINV_DCAC_Bottom_LF_init();
#define ENABLE 34
void ENABLE_init();
#define uINV_DCAC_TestPoint_1 17
void uINV_DCAC_TestPoint_1_init();
#define uINV_DCAC_TestPoint_2 16
void uINV_DCAC_TestPoint_2_init();
#define Shutdown_Demo 31
void Shutdown_Demo_init();
#define RapidShutdown 30
void RapidShutdown_init();
#define STATE_TP 58
void STATE_TP_init();

//*****************************************************************************
//
// INPUTXBAR Configurations
//
//*****************************************************************************
#define myINPUTXBARINPUT0_SOURCE 60
#define myINPUTXBARINPUT0_INPUT XBAR_INPUT1
void myINPUTXBARINPUT0_init();
#define myINPUTXBARINPUT1_SOURCE 58
#define myINPUTXBARINPUT1_INPUT XBAR_INPUT2
void myINPUTXBARINPUT1_init();
#define myINPUTXBARINPUT2_SOURCE 48
#define myINPUTXBARINPUT2_INPUT XBAR_INPUT3
void myINPUTXBARINPUT2_init();
#define myINPUTXBARINPUT3_SOURCE 49
#define myINPUTXBARINPUT3_INPUT XBAR_INPUT4
void myINPUTXBARINPUT3_init();

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_ADC_B_1
#define INT_ADC_B_1 INT_ADCB1
#define INT_ADC_B_1_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1
extern __interrupt void ISR2(void);

// Interrupt Settings for INT_ADC_C_1
#define INT_ADC_C_1 INT_ADCC1
#define INT_ADC_C_1_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1
extern __interrupt void ISR1(void);

// Interrupt Settings for INT_DCAC
#define INT_DCAC INT_EPWM8
#define INT_DCAC_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP3
extern __interrupt void ISR3(void);

//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
#define SCI_COMMS_BASE SCIA_BASE
#define SCI_COMMS_BAUDRATE 115200
#define SCI_COMMS_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define SCI_COMMS_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define SCI_COMMS_CONFIG_PAR SCI_CONFIG_PAR_NONE
void SCI_COMMS_init();

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// SYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	Board_init();
void	ADC_init();
void	ASYSCTL_init();
void	CLB_init();
void	CMPSS_init();
void	ECAP_init();
void	EPWM_init();
void	EPWMXBAR_init();
void	GPIO_init();
void	INPUTXBAR_init();
void	INTERRUPT_init();
void	SCI_init();
void	SYNC_init();
void	SYSCTL_init();
void	PinMux_init();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // end of BOARD_H definition
