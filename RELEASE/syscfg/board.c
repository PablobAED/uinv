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

#include "board.h"

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules. 
// Call this function in your application if you wish to do all module 
// initialization.
// If you wish to not use some of the initializations, instead of the 
// Board_init use the individual Module_inits
//
//*****************************************************************************
void Board_init()
{
	EALLOW;

	PinMux_init();
	SYSCTL_init();
	INPUTXBAR_init();
	SYNC_init();
	ASYSCTL_init();
	ADC_init();
	CLB_init();
	CMPSS_init();
	ECAP_init();
	EPWM_init();
	EPWMXBAR_init();
	GPIO_init();
	SCI_init();
	INTERRUPT_init();

	EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
	//
	// PinMux for modules assigned to CPU1
	//
	
	//
	// ANALOG -> myANALOGPinMux0 Pinmux
	//
	// Analog PinMux for A0/B15/C15/DACA_OUT
	GPIO_setPinConfig(GPIO_231_GPIO231);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(231, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A1/B7/DACB_OUT
	GPIO_setPinConfig(GPIO_232_GPIO232);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(232, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A10/B1/C10
	GPIO_setPinConfig(GPIO_230_GPIO230);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(230, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A12, C5
	GPIO_setPinConfig(GPIO_238_GPIO238);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(238, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A14/B14/C4
	GPIO_setPinConfig(GPIO_239_GPIO239);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(239, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A2/B6/C9
	GPIO_setPinConfig(GPIO_224_GPIO224);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(224, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A3, C7/B9
	GPIO_setPinConfig(GPIO_229_GPIO229);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(229, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A4/B8
	GPIO_setPinConfig(GPIO_225_GPIO225);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(225, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A5
	GPIO_setPinConfig(GPIO_249_GPIO249);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(249, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A6
	GPIO_setPinConfig(GPIO_228_GPIO228);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(228, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A9
	GPIO_setPinConfig(GPIO_227_GPIO227);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(227, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B0/C11
	GPIO_setPinConfig(GPIO_253_GPIO253);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(253, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B11
	GPIO_setPinConfig(GPIO_251_GPIO251);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(251, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B2/C6
	GPIO_setPinConfig(GPIO_226_GPIO226);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(226, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B3/VDAC
	GPIO_setPinConfig(GPIO_242_GPIO242);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(242, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B4/C8
	GPIO_setPinConfig(GPIO_236_GPIO236);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(236, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B5
	GPIO_setPinConfig(GPIO_252_GPIO252);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(252, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C1
	GPIO_setPinConfig(GPIO_248_GPIO248);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(248, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C2/B12
	GPIO_setPinConfig(GPIO_244_GPIO244);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(244, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C3/A7
	GPIO_setPinConfig(GPIO_245_GPIO245);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(245, GPIO_ANALOG_ENABLED);
	//
	// EPWM4 -> BOOST1 Pinmux
	//
	GPIO_setPinConfig(BOOST1_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(BOOST1_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOST1_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(BOOST1_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(BOOST1_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOST1_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM2 -> BOOST2 Pinmux
	//
	GPIO_setPinConfig(BOOST2_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(BOOST2_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOST2_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(BOOST2_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(BOOST2_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOST2_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM1 -> BOOST3 Pinmux
	//
	GPIO_setPinConfig(BOOST3_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(BOOST3_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOST3_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(BOOST3_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(BOOST3_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOST3_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM3 -> BOOST4 Pinmux
	//
	GPIO_setPinConfig(BOOST4_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(BOOST4_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOST4_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(BOOST4_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(BOOST4_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(BOOST4_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM5 -> LLC1 Pinmux
	//
	GPIO_setPinConfig(LLC1_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(LLC1_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(LLC1_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(LLC1_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(LLC1_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(LLC1_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM6 -> LLC2 Pinmux
	//
	GPIO_setPinConfig(LLC2_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(LLC2_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(LLC2_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(LLC2_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(LLC2_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(LLC2_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM7 -> LLCSR Pinmux
	//
	GPIO_setPinConfig(LLCSR_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(LLCSR_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(LLCSR_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(LLCSR_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(LLCSR_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(LLCSR_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM8 -> DCAC Pinmux
	//
	GPIO_setPinConfig(DCAC_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(DCAC_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(DCAC_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(DCAC_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(DCAC_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(DCAC_EPWMB_GPIO, GPIO_QUAL_SYNC);

	// GPIO59 -> ISO_PWR_EN Pinmux
	GPIO_setPinConfig(GPIO_59_GPIO59);
	// GPIO19, X1 -> LED1 Pinmux
	GPIO_setPinConfig(GPIO_19_GPIO19);
	// GPIO44 -> uINV_DCAC_Relay Pinmux
	GPIO_setPinConfig(GPIO_44_GPIO44);
	// GPIO56 -> uINV_DCAC_Top_LF Pinmux
	GPIO_setPinConfig(GPIO_56_GPIO56);
	// GPIO57 -> uINV_DCAC_Bottom_LF Pinmux
	GPIO_setPinConfig(GPIO_57_GPIO57);
	// GPIO34 -> ENABLE Pinmux
	GPIO_setPinConfig(GPIO_34_GPIO34);
	// GPIO17 -> uINV_DCAC_TestPoint_1 Pinmux
	GPIO_setPinConfig(GPIO_17_GPIO17);
	// GPIO16 -> uINV_DCAC_TestPoint_2 Pinmux
	GPIO_setPinConfig(GPIO_16_GPIO16);
	// GPIO31 -> Shutdown_Demo Pinmux
	GPIO_setPinConfig(GPIO_31_GPIO31);
	// GPIO30 -> RapidShutdown Pinmux
	GPIO_setPinConfig(GPIO_30_GPIO30);
	// GPIO58 -> STATE_TP Pinmux
	GPIO_setPinConfig(GPIO_58_GPIO58);
	//
	// SCIA -> SCI_COMMS Pinmux
	//
	GPIO_setPinConfig(SCI_COMMS_SCIRX_PIN_CONFIG);
	GPIO_setPadConfig(SCI_COMMS_SCIRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(SCI_COMMS_SCIRX_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(SCI_COMMS_SCITX_PIN_CONFIG);
	GPIO_setPadConfig(SCI_COMMS_SCITX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(SCI_COMMS_SCITX_GPIO, GPIO_QUAL_ASYNC);


}

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
void ADC_init(){
	ADC_A_init();
	ADC_B_init();
	ADC_C_init();
}

void ADC_A_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(ADC_A_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(ADC_A_BASE, ADC_CLK_DIV_2_0);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(ADC_A_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(ADC_A_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(5000);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(ADC_A_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(ADC_A_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM4_SOCA
	//	  	Channel			: ADC_CH_ADCIN6
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_A_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM4_SOCA, ADC_CH_ADCIN6, 12U);
	ADC_setInterruptSOCTrigger(ADC_A_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM4_SOCB
	//	  	Channel			: ADC_CH_ADCIN6
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_A_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM4_SOCB, ADC_CH_ADCIN6, 12U);
	ADC_setInterruptSOCTrigger(ADC_A_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM4_SOCB
	//	  	Channel			: ADC_CH_ADCIN5
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_A_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM4_SOCB, ADC_CH_ADCIN5, 12U);
	ADC_setInterruptSOCTrigger(ADC_A_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 3 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM2_SOCA
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_A_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM2_SOCA, ADC_CH_ADCIN4, 12U);
	ADC_setInterruptSOCTrigger(ADC_A_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 4 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 4
	//	  	Trigger			: ADC_TRIGGER_EPWM2_SOCB
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_A_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM2_SOCB, ADC_CH_ADCIN4, 12U);
	ADC_setInterruptSOCTrigger(ADC_A_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 5 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 5
	//	  	Trigger			: ADC_TRIGGER_EPWM2_SOCB
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_A_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM2_SOCB, ADC_CH_ADCIN2, 12U);
	ADC_setInterruptSOCTrigger(ADC_A_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);
}
void ADC_B_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(ADC_B_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(ADC_B_BASE, ADC_CLK_DIV_2_0);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(ADC_B_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(ADC_B_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(5000);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(ADC_B_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(ADC_B_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_B_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 12U);
	ADC_setInterruptSOCTrigger(ADC_B_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCB
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_B_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCB, ADC_CH_ADCIN3, 12U);
	ADC_setInterruptSOCTrigger(ADC_B_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCB
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_B_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCB, ADC_CH_ADCIN2, 12U);
	ADC_setInterruptSOCTrigger(ADC_B_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 3 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM3_SOCA
	//	  	Channel			: ADC_CH_ADCIN12
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_B_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM3_SOCA, ADC_CH_ADCIN12, 12U);
	ADC_setInterruptSOCTrigger(ADC_B_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 4 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 4
	//	  	Trigger			: ADC_TRIGGER_EPWM3_SOCB
	//	  	Channel			: ADC_CH_ADCIN12
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_B_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM3_SOCB, ADC_CH_ADCIN12, 12U);
	ADC_setInterruptSOCTrigger(ADC_B_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 5 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 5
	//	  	Trigger			: ADC_TRIGGER_EPWM3_SOCB
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_B_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM3_SOCB, ADC_CH_ADCIN4, 12U);
	ADC_setInterruptSOCTrigger(ADC_B_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 6 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 6
	//	  	Trigger			: ADC_TRIGGER_EPWM3_SOCB
	//	  	Channel			: ADC_CH_ADCIN5
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_B_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_EPWM3_SOCB, ADC_CH_ADCIN5, 12U);
	ADC_setInterruptSOCTrigger(ADC_B_BASE, ADC_SOC_NUMBER6, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 7 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 7
	//	  	Trigger			: ADC_TRIGGER_EPWM3_SOCA
	//	  	Channel			: ADC_CH_ADCIN11
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_B_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_EPWM3_SOCA, ADC_CH_ADCIN11, 12U);
	ADC_setInterruptSOCTrigger(ADC_B_BASE, ADC_SOC_NUMBER7, ADC_INT_SOC_TRIGGER_NONE);
	//
	// ADC Interrupt 1 Configuration
	// 		Source	: ADC_SOC_NUMBER6
	// 		Interrupt Source: enabled
	// 		Continuous Mode	: disabled
	//
	//
	ADC_setInterruptSource(ADC_B_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER6);
	ADC_clearInterruptStatus(ADC_B_BASE, ADC_INT_NUMBER1);
	ADC_disableContinuousMode(ADC_B_BASE, ADC_INT_NUMBER1);
	ADC_enableInterrupt(ADC_B_BASE, ADC_INT_NUMBER1);
}
void ADC_C_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(ADC_C_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(ADC_C_BASE, ADC_CLK_DIV_2_0);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(ADC_C_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(ADC_C_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(5000);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(ADC_C_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(ADC_C_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM8_SOCA
	//	  	Channel			: ADC_CH_ADCIN5
	//	 	Sample Window	: 24 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_C_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM8_SOCA, ADC_CH_ADCIN5, 24U);
	ADC_setInterruptSOCTrigger(ADC_C_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM8_SOCA
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_C_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM8_SOCA, ADC_CH_ADCIN3, 12U);
	ADC_setInterruptSOCTrigger(ADC_C_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM8_SOCB
	//	  	Channel			: ADC_CH_ADCIN5
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_C_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM8_SOCB, ADC_CH_ADCIN5, 12U);
	ADC_setInterruptSOCTrigger(ADC_C_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 3 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM8_SOCB
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_C_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM8_SOCB, ADC_CH_ADCIN3, 12U);
	ADC_setInterruptSOCTrigger(ADC_C_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 4 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 4
	//	  	Trigger			: ADC_TRIGGER_EPWM8_SOCA
	//	  	Channel			: ADC_CH_ADCIN7
	//	 	Sample Window	: 24 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_C_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM8_SOCA, ADC_CH_ADCIN7, 24U);
	ADC_setInterruptSOCTrigger(ADC_C_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 5 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 5
	//	  	Trigger			: ADC_TRIGGER_EPWM8_SOCB
	//	  	Channel			: ADC_CH_ADCIN11
	//	 	Sample Window	: 50 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_C_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM8_SOCB, ADC_CH_ADCIN11, 50U);
	ADC_setInterruptSOCTrigger(ADC_C_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 6 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 6
	//	  	Trigger			: ADC_TRIGGER_SW_ONLY
	//	  	Channel			: ADC_CH_ADCIN1
	//	 	Sample Window	: 12 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADC_C_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN1, 12U);
	ADC_setInterruptSOCTrigger(ADC_C_BASE, ADC_SOC_NUMBER6, ADC_INT_SOC_TRIGGER_NONE);
	//
	// ADC Interrupt 1 Configuration
	// 		Source	: ADC_SOC_NUMBER5
	// 		Interrupt Source: enabled
	// 		Continuous Mode	: disabled
	//
	//
	ADC_setInterruptSource(ADC_C_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER5);
	ADC_clearInterruptStatus(ADC_C_BASE, ADC_INT_NUMBER1);
	ADC_disableContinuousMode(ADC_C_BASE, ADC_INT_NUMBER1);
	ADC_enableInterrupt(ADC_C_BASE, ADC_INT_NUMBER1);
}

//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************
void ASYSCTL_init(){
	//
	// asysctl initialization
	//
	// Enables the temperature sensor output to the ADC.
	//
	ASysCtl_enableTemperatureSensor();
	DEVICE_DELAY_US(500);
	//
	// Set the analog voltage reference selection to internal.
	//
	ASysCtl_setAnalogReferenceInternal( ASYSCTL_VREFHI );
	//
	// Set the internal analog voltage reference selection to 1.65V.
	//
	ASysCtl_setAnalogReference1P65( ASYSCTL_VREFHI );
}

//*****************************************************************************
//
// CLB Configurations
//
//*****************************************************************************
void CLB_init(){
	myCLB0_init();
}

void myCLB0_init(){
	CLB_setOutputMask(myCLB0_BASE,
				(0UL << 0UL), true);
	CLB_enableOutputMaskUpdates(myCLB0_BASE);
	//
	// myCLB0 SPI Buffer Configuration
	//
	CLB_disableSPIBufferAccess(myCLB0_BASE);
	CLB_configSPIBufferLoadSignal(myCLB0_BASE, 0);
	CLB_configSPIBufferShift(myCLB0_BASE, 0);
	CLB_setGPREG(myCLB0_BASE,0);

	CLB_enableCLB(myCLB0_BASE);
}

//*****************************************************************************
//
// CMPSS Configurations
//
//*****************************************************************************
void CMPSS_init(){
	myCMPSS_init();
	CMPSS_SR_init();
}

void myCMPSS_init(){
    //
    // Select the value for CMP2HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_2,1U);
    //
    // Select the value for CMP2LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_2,1U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(myCMPSS_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(myCMPSS_BASE,(CMPSS_INSRC_DAC | CMPSS_INV_INVERTED));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDAC(myCMPSS_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(myCMPSS_BASE,3500U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(myCMPSS_BASE,500U);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(myCMPSS_BASE, 0U, 1U, 1U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(myCMPSS_BASE, 0U, 1U, 1U);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(myCMPSS_BASE,(CMPSS_TRIPOUT_ASYNC_COMP | CMPSS_TRIP_ASYNC_COMP));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(myCMPSS_BASE,(CMPSS_TRIPOUT_ASYNC_COMP | CMPSS_TRIP_ASYNC_COMP));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(myCMPSS_BASE,0U);
    //
    // Configures the comparator subsystem's ramp generator.
    //
    CMPSS_configRamp(myCMPSS_BASE,0U,0U,0U,1U,true);
    //
    // Disables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCHigh(myCMPSS_BASE);
    //
    // Disables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCLow(myCMPSS_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(myCMPSS_BASE,1U);
    //
    // Disables an ePWM blanking signal from holding trip in reset.
    //
    CMPSS_disableBlanking(myCMPSS_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(myCMPSS_BASE,false,false);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(myCMPSS_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
}
void CMPSS_SR_init(){
    //
    // Select the value for CMP3HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_3,4U);
    //
    // Select the value for CMP3LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_3,4U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(CMPSS_SR_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(CMPSS_SR_BASE,(CMPSS_INSRC_DAC | CMPSS_INV_INVERTED));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDAC(CMPSS_SR_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(CMPSS_SR_BASE,2000U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(CMPSS_SR_BASE,2096U);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(CMPSS_SR_BASE, 0U, 7U, 5U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(CMPSS_SR_BASE, 0U, 7U, 5U);
    //
    // Initializes the digital filter of the high comparator.
    //
    CMPSS_initFilterHigh(CMPSS_SR_BASE);
    //
    // Initializes the digital filter of the low comparator.
    //
    CMPSS_initFilterLow(CMPSS_SR_BASE);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(CMPSS_SR_BASE,(CMPSS_TRIPOUT_ASYNC_COMP | CMPSS_TRIP_FILTER));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(CMPSS_SR_BASE,(CMPSS_TRIPOUT_ASYNC_COMP | CMPSS_TRIP_FILTER));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(CMPSS_SR_BASE,0U);
    //
    // Configures the comparator subsystem's ramp generator.
    //
    CMPSS_configRamp(CMPSS_SR_BASE,0U,0U,0U,1U,true);
    //
    // Enables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_enableLatchResetOnPWMSYNCHigh(CMPSS_SR_BASE);
    //
    // Enables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_enableLatchResetOnPWMSYNCLow(CMPSS_SR_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(CMPSS_SR_BASE,7U);
    //
    // Enables an ePWM blanking signal to hold trip in reset.
    //
    CMPSS_enableBlanking(CMPSS_SR_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(CMPSS_SR_BASE,true,true);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(CMPSS_SR_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
}

//*****************************************************************************
//
// ECAP Configurations
//
//*****************************************************************************
void ECAP_init(){
	myECAP0_init();
}

void myECAP0_init(){
	//
	// Disables time stamp capture.
	//
	ECAP_disableTimeStampCapture(myECAP0_BASE);
	//
	// Stops Time stamp counter.
	//
	ECAP_stopCounter(myECAP0_BASE);
	//
	// Sets eCAP in Capture mode.
	//
	ECAP_enableCaptureMode(myECAP0_BASE);
	//
	// Sets the capture mode.
	//
	ECAP_setCaptureMode(myECAP0_BASE,ECAP_ONE_SHOT_CAPTURE_MODE,ECAP_EVENT_4);
	//
	// Sets the Capture event prescaler.
	//
	ECAP_setEventPrescaler(myECAP0_BASE, 0U);
	//
	// Sets the Capture event polarity.
	//
	ECAP_setEventPolarity(myECAP0_BASE,ECAP_EVENT_1,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(myECAP0_BASE,ECAP_EVENT_2,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(myECAP0_BASE,ECAP_EVENT_3,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(myECAP0_BASE,ECAP_EVENT_4,ECAP_EVNT_RISING_EDGE);
	//
	// Configure counter reset on events
	//
	ECAP_disableCounterResetOnEvent(myECAP0_BASE,ECAP_EVENT_1);
	ECAP_disableCounterResetOnEvent(myECAP0_BASE,ECAP_EVENT_2);
	ECAP_disableCounterResetOnEvent(myECAP0_BASE,ECAP_EVENT_3);
	ECAP_enableCounterResetOnEvent(myECAP0_BASE,ECAP_EVENT_4);	
	//
	// Select eCAP input.
	//
	ECAP_selectECAPInput(myECAP0_BASE,ECAP_INPUT_INPUTXBAR1);
	//
	// Sets a phase shift value count.
	//
	ECAP_setPhaseShiftCount(myECAP0_BASE,0U);
	//
	// Enable counter loading with phase shift value.
	//
	ECAP_enableLoadCounter(myECAP0_BASE);
	//
	// Configures Sync out signal mode.
	//
	ECAP_setSyncOutMode(myECAP0_BASE,ECAP_SYNC_OUT_SYNCI);
	//
	// Resets eCAP counters and flags.
	//
	ECAP_resetCounters(myECAP0_BASE);
	//
	// Configures emulation mode.
	//
	ECAP_setEmulationMode(myECAP0_BASE,ECAP_EMULATION_STOP);
	//
	// Set up the source for sync-in pulse..
	//
	ECAP_setSyncInPulseSource(myECAP0_BASE,ECAP_SYNC_IN_PULSE_SRC_DISABLE);
	//
	// Starts Time stamp counter for myECAP0.
	//
	ECAP_startCounter(myECAP0_BASE);
	//
	// Enables time stamp capture for myECAP0.
	//
	ECAP_enableTimeStampCapture(myECAP0_BASE);
	//
	// Re-arms the eCAP module for myECAP0.
	//
	ECAP_reArm(myECAP0_BASE);

    //-----------------Signal Monitoring--------------------//
}

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
void EPWM_init(){
    EPWM_setEmulationMode(BOOST1_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(BOOST1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(BOOST1_BASE, 250);	
    EPWM_setTimeBaseCounter(BOOST1_BASE, 0);	
    EPWM_setTimeBaseCounterMode(BOOST1_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_setCountModeAfterSync(BOOST1_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);	
    EPWM_enablePhaseShiftLoad(BOOST1_BASE);	
    EPWM_setPhaseShift(BOOST1_BASE, 2);	
    EPWM_setSyncInPulseSource(BOOST1_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM8);	
    EPWM_enableSyncOutPulseSource(BOOST1_BASE, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);	
    EPWM_setCounterCompareValue(BOOST1_BASE, EPWM_COUNTER_COMPARE_A, 0);	
    EPWM_setCounterCompareShadowLoadMode(BOOST1_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(BOOST1_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(BOOST1_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(BOOST1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setFallingEdgeDeadBandDelayInput(BOOST1_BASE, EPWM_DB_INPUT_EPWMB);	
    EPWM_setDeadBandDelayPolarity(BOOST1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(BOOST1_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(BOOST1_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(BOOST1_BASE);	
    EPWM_setRisingEdgeDelayCount(BOOST1_BASE, 2);	
    EPWM_setDeadBandDelayMode(BOOST1_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(BOOST1_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(BOOST1_BASE);	
    EPWM_setFallingEdgeDelayCount(BOOST1_BASE, 2);	
    EPWM_setTripZoneAction(BOOST1_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST1_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST1_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST1_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST1_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST1_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_enableADCTrigger(BOOST1_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerSource(BOOST1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);	
    EPWM_setADCTriggerEventPrescale(BOOST1_BASE, EPWM_SOC_A, 6);	
    EPWM_enableADCTrigger(BOOST1_BASE, EPWM_SOC_B);	
    EPWM_setADCTriggerSource(BOOST1_BASE, EPWM_SOC_B, EPWM_SOC_TBCTR_PERIOD);	
    EPWM_setADCTriggerEventPrescale(BOOST1_BASE, EPWM_SOC_B, 6);	
    EPWM_setEmulationMode(BOOST2_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(BOOST2_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(BOOST2_BASE, 250);	
    EPWM_setTimeBaseCounter(BOOST2_BASE, 0);	
    EPWM_setTimeBaseCounterMode(BOOST2_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_setCountModeAfterSync(BOOST2_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);	
    EPWM_enablePhaseShiftLoad(BOOST2_BASE);	
    EPWM_setPhaseShift(BOOST2_BASE, 2);	
    EPWM_setSyncInPulseSource(BOOST2_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM8);	
    EPWM_setCounterCompareValue(BOOST2_BASE, EPWM_COUNTER_COMPARE_A, 0);	
    EPWM_setCounterCompareShadowLoadMode(BOOST2_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(BOOST2_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(BOOST2_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(BOOST2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setFallingEdgeDeadBandDelayInput(BOOST2_BASE, EPWM_DB_INPUT_EPWMB);	
    EPWM_setDeadBandDelayPolarity(BOOST2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(BOOST2_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(BOOST2_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(BOOST2_BASE);	
    EPWM_setRisingEdgeDelayCount(BOOST2_BASE, 2);	
    EPWM_setDeadBandDelayMode(BOOST2_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(BOOST2_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(BOOST2_BASE);	
    EPWM_setFallingEdgeDelayCount(BOOST2_BASE, 2);	
    EPWM_setTripZoneAction(BOOST2_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST2_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST2_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST2_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST2_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST2_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_enableADCTrigger(BOOST2_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerSource(BOOST2_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);	
    EPWM_setADCTriggerEventPrescale(BOOST2_BASE, EPWM_SOC_A, 6);	
    EPWM_enableADCTrigger(BOOST2_BASE, EPWM_SOC_B);	
    EPWM_setADCTriggerSource(BOOST2_BASE, EPWM_SOC_B, EPWM_SOC_TBCTR_PERIOD);	
    EPWM_setADCTriggerEventPrescale(BOOST2_BASE, EPWM_SOC_B, 6);	
    EPWM_setEmulationMode(BOOST3_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(BOOST3_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(BOOST3_BASE, 250);	
    EPWM_setTimeBaseCounter(BOOST3_BASE, 0);	
    EPWM_setTimeBaseCounterMode(BOOST3_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_enablePhaseShiftLoad(BOOST3_BASE);	
    EPWM_setPhaseShift(BOOST3_BASE, 2);	
    EPWM_setSyncInPulseSource(BOOST3_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM8);	
    EPWM_setCounterCompareValue(BOOST3_BASE, EPWM_COUNTER_COMPARE_A, 0);	
    EPWM_setCounterCompareShadowLoadMode(BOOST3_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(BOOST3_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(BOOST3_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(BOOST3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setFallingEdgeDeadBandDelayInput(BOOST3_BASE, EPWM_DB_INPUT_EPWMB);	
    EPWM_setDeadBandDelayPolarity(BOOST3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(BOOST3_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(BOOST3_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(BOOST3_BASE);	
    EPWM_setRisingEdgeDelayCount(BOOST3_BASE, 2);	
    EPWM_setDeadBandDelayMode(BOOST3_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(BOOST3_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(BOOST3_BASE);	
    EPWM_setFallingEdgeDelayCount(BOOST3_BASE, 2);	
    EPWM_setTripZoneAction(BOOST3_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST3_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST3_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST3_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST3_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST3_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_enableADCTrigger(BOOST3_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerSource(BOOST3_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);	
    EPWM_setADCTriggerEventPrescale(BOOST3_BASE, EPWM_SOC_A, 6);	
    EPWM_enableADCTrigger(BOOST3_BASE, EPWM_SOC_B);	
    EPWM_setADCTriggerSource(BOOST3_BASE, EPWM_SOC_B, EPWM_SOC_TBCTR_PERIOD);	
    EPWM_setADCTriggerEventPrescale(BOOST3_BASE, EPWM_SOC_B, 6);	
    EPWM_setEmulationMode(BOOST4_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(BOOST4_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(BOOST4_BASE, 250);	
    EPWM_setTimeBaseCounter(BOOST4_BASE, 0);	
    EPWM_setTimeBaseCounterMode(BOOST4_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_enablePhaseShiftLoad(BOOST4_BASE);	
    EPWM_setPhaseShift(BOOST4_BASE, 2);	
    EPWM_setSyncInPulseSource(BOOST4_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM8);	
    EPWM_setCounterCompareValue(BOOST4_BASE, EPWM_COUNTER_COMPARE_A, 0);	
    EPWM_setCounterCompareShadowLoadMode(BOOST4_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(BOOST4_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(BOOST4_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(BOOST4_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setFallingEdgeDeadBandDelayInput(BOOST4_BASE, EPWM_DB_INPUT_EPWMB);	
    EPWM_setDeadBandDelayPolarity(BOOST4_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(BOOST4_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(BOOST4_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(BOOST4_BASE);	
    EPWM_setRisingEdgeDelayCount(BOOST4_BASE, 2);	
    EPWM_setDeadBandDelayMode(BOOST4_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(BOOST4_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(BOOST4_BASE);	
    EPWM_setFallingEdgeDelayCount(BOOST4_BASE, 2);	
    EPWM_setTripZoneAction(BOOST4_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST4_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST4_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST4_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST4_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(BOOST4_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_enableADCTrigger(BOOST4_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerSource(BOOST4_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);	
    EPWM_setADCTriggerEventPrescale(BOOST4_BASE, EPWM_SOC_A, 6);	
    EPWM_enableADCTrigger(BOOST4_BASE, EPWM_SOC_B);	
    EPWM_setADCTriggerSource(BOOST4_BASE, EPWM_SOC_B, EPWM_SOC_TBCTR_PERIOD);	
    EPWM_setADCTriggerEventPrescale(BOOST4_BASE, EPWM_SOC_B, 6);	
    EPWM_setEmulationMode(LLC1_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(LLC1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(LLC1_BASE, 120);	
    EPWM_setTimeBaseCounter(LLC1_BASE, 0);	
    EPWM_setTimeBaseCounterMode(LLC1_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_disablePhaseShiftLoad(LLC1_BASE);	
    EPWM_setPhaseShift(LLC1_BASE, 0);	
    EPWM_setSyncInPulseSource(LLC1_BASE, EPWM_SYNC_IN_PULSE_SRC_DISABLE);	
    EPWM_enableSyncOutPulseSource(LLC1_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(LLC1_BASE, EPWM_COUNTER_COMPARE_A, 0);	
    EPWM_setCounterCompareShadowLoadMode(LLC1_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(LLC1_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(LLC1_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(LLC1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(LLC1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(LLC1_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(LLC1_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(LLC1_BASE);	
    EPWM_setRisingEdgeDelayCount(LLC1_BASE, 3);	
    EPWM_setDeadBandDelayMode(LLC1_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(LLC1_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(LLC1_BASE);	
    EPWM_setFallingEdgeDelayCount(LLC1_BASE, 3);	
    EPWM_setTripZoneAction(LLC1_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC1_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC1_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC1_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC1_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC1_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setEmulationMode(LLC2_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(LLC2_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(LLC2_BASE, 120);	
    EPWM_setTimeBaseCounter(LLC2_BASE, 0);	
    EPWM_setTimeBaseCounterMode(LLC2_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_enablePhaseShiftLoad(LLC2_BASE);	
    EPWM_setPhaseShift(LLC2_BASE, 120);	
    EPWM_setSyncInPulseSource(LLC2_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5);	
    EPWM_enableSyncOutPulseSource(LLC2_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(LLC2_BASE, EPWM_COUNTER_COMPARE_A, 0);	
    EPWM_setCounterCompareShadowLoadMode(LLC2_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(LLC2_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(LLC2_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(LLC2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(LLC2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(LLC2_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(LLC2_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(LLC2_BASE);	
    EPWM_setRisingEdgeDelayCount(LLC2_BASE, 3);	
    EPWM_setDeadBandDelayMode(LLC2_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(LLC2_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(LLC2_BASE);	
    EPWM_setFallingEdgeDelayCount(LLC2_BASE, 3);	
    EPWM_setTripZoneAction(LLC2_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC2_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC2_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC2_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC2_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(LLC2_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setEmulationMode(LLCSR_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(LLCSR_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(LLCSR_BASE, 120);	
    EPWM_setTimeBaseCounter(LLCSR_BASE, 0);	
    EPWM_setTimeBaseCounterMode(LLCSR_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_enablePhaseShiftLoad(LLCSR_BASE);	
    EPWM_setPhaseShift(LLCSR_BASE, 60);	
    EPWM_setSyncInPulseSource(LLCSR_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM6);	
    EPWM_setCounterCompareValue(LLCSR_BASE, EPWM_COUNTER_COMPARE_A, 0);	
    EPWM_setCounterCompareShadowLoadMode(LLCSR_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(LLCSR_BASE, EPWM_COUNTER_COMPARE_B, 120);	
    EPWM_setCounterCompareShadowLoadMode(LLCSR_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(LLCSR_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setFallingEdgeDeadBandDelayInput(LLCSR_BASE, EPWM_DB_INPUT_EPWMB);	
    EPWM_setDeadBandDelayPolarity(LLCSR_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(LLCSR_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(LLCSR_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(LLCSR_BASE);	
    EPWM_setRisingEdgeDelayCount(LLCSR_BASE, 12);	
    EPWM_setDeadBandDelayMode(LLCSR_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(LLCSR_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(LLCSR_BASE);	
    EPWM_setFallingEdgeDelayCount(LLCSR_BASE, 12);	
    EPWM_setDeadBandOutputSwapMode(LLCSR_BASE, EPWM_DB_OUTPUT_A, true);	
    EPWM_setDeadBandOutputSwapMode(LLCSR_BASE, EPWM_DB_OUTPUT_B, true);	
    EPWM_selectCycleByCycleTripZoneClearEvent(LLCSR_BASE, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD);	
    EPWM_selectDigitalCompareTripInput(LLCSR_BASE, EPWM_DC_TRIP_TRIPIN11, EPWM_DC_TYPE_DCAH);	
    EPWM_selectDigitalCompareTripInput(LLCSR_BASE, EPWM_DC_TRIP_TRIPIN11, EPWM_DC_TYPE_DCAL);	
    EPWM_setTripZoneDigitalCompareEventCondition(LLCSR_BASE, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_setDigitalCompareCBCLatchMode(LLCSR_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_ENABLED);	
    EPWM_selectDigitalCompareCBCLatchClearEvent(LLCSR_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_ON_CNTR_ZERO_PERIOD);	
    EPWM_selectDigitalCompareTripInput(LLCSR_BASE, EPWM_DC_TRIP_TRIPIN12, EPWM_DC_TYPE_DCBH);	
    EPWM_selectDigitalCompareTripInput(LLCSR_BASE, EPWM_DC_TRIP_TRIPIN12, EPWM_DC_TYPE_DCBL);	
    EPWM_setTripZoneDigitalCompareEventCondition(LLCSR_BASE, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DCXL_HIGH);	
    EPWM_setDigitalCompareCBCLatchMode(LLCSR_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_ENABLED);	
    EPWM_selectDigitalCompareCBCLatchClearEvent(LLCSR_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_ON_CNTR_ZERO_PERIOD);	
    EPWM_enableDigitalCompareBlankingWindow(LLCSR_BASE);	
    EPWM_setDigitalCompareBlankingEvent(LLCSR_BASE, EPWM_DC_WINDOW_START_TBCTR_ZERO_PERIOD);	
    EPWM_setDigitalCompareWindowOffset(LLCSR_BASE, 116);	
    EPWM_setDigitalCompareWindowLength(LLCSR_BASE, 40);	
    EPWM_setEmulationMode(DCAC_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(DCAC_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(DCAC_BASE, 500);	
    EPWM_setTimeBaseCounter(DCAC_BASE, 0);	
    EPWM_setTimeBaseCounterMode(DCAC_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_disablePhaseShiftLoad(DCAC_BASE);	
    EPWM_setPhaseShift(DCAC_BASE, 0);	
    EPWM_setSyncInPulseSource(DCAC_BASE, EPWM_SYNC_IN_PULSE_SRC_DISABLE);	
    EPWM_enableSyncOutPulseSource(DCAC_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, 0);	
    EPWM_setCounterCompareShadowLoadMode(DCAC_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(DCAC_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(DCAC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(DCAC_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(DCAC_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(DCAC_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(DCAC_BASE);	
    EPWM_setRisingEdgeDelayCount(DCAC_BASE, 10);	
    EPWM_setDeadBandDelayMode(DCAC_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(DCAC_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(DCAC_BASE);	
    EPWM_setFallingEdgeDelayCount(DCAC_BASE, 10);	
    EPWM_setTripZoneAction(DCAC_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(DCAC_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(DCAC_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(DCAC_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(DCAC_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(DCAC_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_selectDigitalCompareTripInput(DCAC_BASE, EPWM_DC_TRIP_TRIPIN5, EPWM_DC_TYPE_DCAH);	
    EPWM_selectDigitalCompareTripInput(DCAC_BASE, EPWM_DC_TRIP_TRIPIN5, EPWM_DC_TYPE_DCAL);	
    EPWM_setTripZoneDigitalCompareEventCondition(DCAC_BASE, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_selectDigitalCompareTripInput(DCAC_BASE, EPWM_DC_TRIP_TRIPIN5, EPWM_DC_TYPE_DCBH);	
    EPWM_selectDigitalCompareTripInput(DCAC_BASE, EPWM_DC_TRIP_TRIPIN5, EPWM_DC_TYPE_DCBL);	
    EPWM_setTripZoneDigitalCompareEventCondition(DCAC_BASE, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_enableInterrupt(DCAC_BASE);	
    EPWM_setInterruptSource(DCAC_BASE, EPWM_INT_TBCTR_PERIOD);	
    EPWM_setInterruptEventCount(DCAC_BASE, 15);	
    EPWM_enableADCTrigger(DCAC_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerSource(DCAC_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);	
    EPWM_setADCTriggerEventPrescale(DCAC_BASE, EPWM_SOC_A, 4);	
    EPWM_enableADCTriggerEventCountInit(DCAC_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerEventCountInitValue(DCAC_BASE, EPWM_SOC_A, 2);	
    EPWM_forceADCTriggerEventCountInit(DCAC_BASE, EPWM_SOC_A);	
    EPWM_enableADCTrigger(DCAC_BASE, EPWM_SOC_B);	
    EPWM_setADCTriggerSource(DCAC_BASE, EPWM_SOC_B, EPWM_SOC_TBCTR_PERIOD);	
    EPWM_setADCTriggerEventPrescale(DCAC_BASE, EPWM_SOC_B, 4);	
    EPWM_enableADCTriggerEventCountInit(DCAC_BASE, EPWM_SOC_B);	
    EPWM_setADCTriggerEventCountInitValue(DCAC_BASE, EPWM_SOC_B, 3);	
    EPWM_forceADCTriggerEventCountInit(DCAC_BASE, EPWM_SOC_B);	
}

//*****************************************************************************
//
// EPWMXBAR Configurations
//
//*****************************************************************************
void EPWMXBAR_init(){
	myEPWMXBAR0_init();
	myEPWMXBAR1_SRTRIP_P_init();
	myEPWMXBAR1_SRTRIP_N_init();
}

void myEPWMXBAR0_init(){
		
	XBAR_setEPWMMuxConfig(myEPWMXBAR0, XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L);
	XBAR_enableEPWMMux(myEPWMXBAR0, XBAR_MUX02);
}
void myEPWMXBAR1_SRTRIP_P_init(){
		
	XBAR_setEPWMMuxConfig(myEPWMXBAR1_SRTRIP_P, XBAR_EPWM_MUX05_CMPSS3_CTRIPL);
	XBAR_enableEPWMMux(myEPWMXBAR1_SRTRIP_P, XBAR_MUX05);
}
void myEPWMXBAR1_SRTRIP_N_init(){
		
	XBAR_setEPWMMuxConfig(myEPWMXBAR1_SRTRIP_N, XBAR_EPWM_MUX04_CMPSS3_CTRIPH);
	XBAR_enableEPWMMux(myEPWMXBAR1_SRTRIP_N, XBAR_MUX04);
}

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
void GPIO_init(){
	ISO_PWR_EN_init();
	LED1_init();
	uINV_DCAC_Relay_init();
	uINV_DCAC_Top_LF_init();
	uINV_DCAC_Bottom_LF_init();
	ENABLE_init();
	uINV_DCAC_TestPoint_1_init();
	uINV_DCAC_TestPoint_2_init();
	Shutdown_Demo_init();
	RapidShutdown_init();
	STATE_TP_init();
}

void ISO_PWR_EN_init(){
	GPIO_writePin(ISO_PWR_EN, 0);
	GPIO_setPadConfig(ISO_PWR_EN, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(ISO_PWR_EN, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(ISO_PWR_EN, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(ISO_PWR_EN, GPIO_CORE_CPU1);
}
void LED1_init(){
	GPIO_writePin(LED1, 0);
	GPIO_setPadConfig(LED1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(LED1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(LED1, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(LED1, GPIO_CORE_CPU1);
}
void uINV_DCAC_Relay_init(){
	GPIO_writePin(uINV_DCAC_Relay, 0);
	GPIO_setPadConfig(uINV_DCAC_Relay, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(uINV_DCAC_Relay, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(uINV_DCAC_Relay, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(uINV_DCAC_Relay, GPIO_CORE_CPU1);
}
void uINV_DCAC_Top_LF_init(){
	GPIO_setPadConfig(uINV_DCAC_Top_LF, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(uINV_DCAC_Top_LF, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(uINV_DCAC_Top_LF, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(uINV_DCAC_Top_LF, GPIO_CORE_CPU1);
}
void uINV_DCAC_Bottom_LF_init(){
	GPIO_setPadConfig(uINV_DCAC_Bottom_LF, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(uINV_DCAC_Bottom_LF, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(uINV_DCAC_Bottom_LF, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(uINV_DCAC_Bottom_LF, GPIO_CORE_CPU1);
}
void ENABLE_init(){
	GPIO_writePin(ENABLE, 1);
	GPIO_setPadConfig(ENABLE, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(ENABLE, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(ENABLE, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(ENABLE, GPIO_CORE_CPU1);
}
void uINV_DCAC_TestPoint_1_init(){
	GPIO_writePin(uINV_DCAC_TestPoint_1, 0);
	GPIO_setPadConfig(uINV_DCAC_TestPoint_1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(uINV_DCAC_TestPoint_1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(uINV_DCAC_TestPoint_1, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(uINV_DCAC_TestPoint_1, GPIO_CORE_CPU1);
}
void uINV_DCAC_TestPoint_2_init(){
	GPIO_writePin(uINV_DCAC_TestPoint_2, 0);
	GPIO_setPadConfig(uINV_DCAC_TestPoint_2, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(uINV_DCAC_TestPoint_2, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(uINV_DCAC_TestPoint_2, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(uINV_DCAC_TestPoint_2, GPIO_CORE_CPU1);
}
void Shutdown_Demo_init(){
	GPIO_writePin(Shutdown_Demo, 0);
	GPIO_setPadConfig(Shutdown_Demo, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(Shutdown_Demo, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(Shutdown_Demo, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(Shutdown_Demo, GPIO_CORE_CPU1);
}
void RapidShutdown_init(){
	GPIO_writePin(RapidShutdown, 0);
	GPIO_setPadConfig(RapidShutdown, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(RapidShutdown, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(RapidShutdown, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(RapidShutdown, GPIO_CORE_CPU1);
}
void STATE_TP_init(){
	GPIO_writePin(STATE_TP, 0);
	GPIO_setPadConfig(STATE_TP, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(STATE_TP, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(STATE_TP, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(STATE_TP, GPIO_CORE_CPU1);
}

//*****************************************************************************
//
// INPUTXBAR Configurations
//
//*****************************************************************************
void INPUTXBAR_init(){
	myINPUTXBARINPUT0_init();
	myINPUTXBARINPUT1_init();
	myINPUTXBARINPUT2_init();
	myINPUTXBARINPUT3_init();
}

void myINPUTXBARINPUT0_init(){
	XBAR_setInputPin(INPUTXBAR_BASE, myINPUTXBARINPUT0_INPUT, myINPUTXBARINPUT0_SOURCE);
}
void myINPUTXBARINPUT1_init(){
	XBAR_setInputPin(INPUTXBAR_BASE, myINPUTXBARINPUT1_INPUT, myINPUTXBARINPUT1_SOURCE);
}
void myINPUTXBARINPUT2_init(){
	XBAR_setInputPin(INPUTXBAR_BASE, myINPUTXBARINPUT2_INPUT, myINPUTXBARINPUT2_SOURCE);
}
void myINPUTXBARINPUT3_init(){
	XBAR_setInputPin(INPUTXBAR_BASE, myINPUTXBARINPUT3_INPUT, myINPUTXBARINPUT3_SOURCE);
}

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************
void INTERRUPT_init(){
	
	// Interrupt Settings for INT_ADC_B_1
	Interrupt_register(INT_ADC_B_1, &ISR2);
	Interrupt_enable(INT_ADC_B_1);
	
	// Interrupt Settings for INT_ADC_C_1
	Interrupt_register(INT_ADC_C_1, &ISR1);
	Interrupt_enable(INT_ADC_C_1);
	
	// Interrupt Settings for INT_DCAC
	Interrupt_register(INT_DCAC, &ISR3);
	Interrupt_enable(INT_DCAC);
}
//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
void SCI_init(){
	SCI_COMMS_init();
}

void SCI_COMMS_init(){
	SCI_clearInterruptStatus(SCI_COMMS_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
	SCI_clearOverflowStatus(SCI_COMMS_BASE);
	SCI_resetTxFIFO(SCI_COMMS_BASE);
	SCI_resetRxFIFO(SCI_COMMS_BASE);
	SCI_resetChannels(SCI_COMMS_BASE);
	SCI_setConfig(SCI_COMMS_BASE, DEVICE_LSPCLK_FREQ, SCI_COMMS_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
	SCI_disableLoopback(SCI_COMMS_BASE);
	SCI_performSoftwareReset(SCI_COMMS_BASE);
	SCI_enableFIFO(SCI_COMMS_BASE);
	SCI_enableModule(SCI_COMMS_BASE);
}

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************
void SYNC_init(){
	SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);
	//
	// SOCA
	//
	SysCtl_enableExtADCSOCSource(SYSCTL_ADCSOC_SRC_PWM1SOCA|SYSCTL_ADCSOC_SRC_PWM2SOCA|SYSCTL_ADCSOC_SRC_PWM3SOCA|SYSCTL_ADCSOC_SRC_PWM4SOCA|SYSCTL_ADCSOC_SRC_PWM8SOCA);
	//
	// SOCB
	//
	SysCtl_enableExtADCSOCSource(SYSCTL_ADCSOC_SRC_PWM1SOCB|SYSCTL_ADCSOC_SRC_PWM2SOCB|SYSCTL_ADCSOC_SRC_PWM3SOCB|SYSCTL_ADCSOC_SRC_PWM4SOCB|SYSCTL_ADCSOC_SRC_PWM8SOCB);
}
//*****************************************************************************
//
// SYSCTL Configurations
//
//*****************************************************************************
void SYSCTL_init(){
	//
    // sysctl initialization
	//
    SysCtl_setStandbyQualificationPeriod(2);
    SysCtl_configureType(SYSCTL_ECAPTYPE, 0, 0);
    SysCtl_configureType(SYSCTL_SDFMTYPE, 0, 0);
    SysCtl_selectErrPinPolarity(0);

    SysCtl_disableMCD();


    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCB, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCC, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCC, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACB, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACB, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWMA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWMA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWMA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWMA, 
        SYSCTL_ACCESS_HIC, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HICA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HICA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HICA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_CPU1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DMA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CPUBGCRC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1BGCRC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRCAL);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ERAD);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM8);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSITXA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_LINA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_LINB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PMBUSA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HICA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_AESA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPG1);



}

