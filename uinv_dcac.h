//##############################################################################
//
// FILE:  uinv_dcac.h
//
// TITLE: Solution header file for the DC/AC converter stage
//
//#############################################################################
// $TI Release: TIDA_010933 v2.00.00.00 $
// $Release Date: Wed Aug  7 17:16:55 CDT 2024 $
// $Copyright:
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef UINV_DCAC_H_
#define UINV_DCAC_H_

#include "board.h"
#include "cmpss.h"
#include "epwm.h"
#include "uinv_dcac.h"
#include "uinv_dcac_settings.h"
#include "stdint.h"
#include "rampgen.h"
#include "dlog_4ch.h"
#include "spll_1ph_sogi.h"
#include "DCLF32.h"
#include "uinv_dcdc.h"
#include "power_meas_sine_analyzer.h"


// Libreria power_measure
 extern float32_t guiIrms;
 extern POWER_MEAS_SINE_ANALYZER sine_mains;
 extern float32_t guiVrms;
 extern float32_t guiPrms;

//Definition of the function in C
void uINV_DCAC_Init();
void uINV_DCAC_test();
void uINV_DCAC_GlobalVariablesInit();
void uINV_DCAC_GaN_temp();

//Ramp Generator parameters
 extern uint32_t uINV_ACDC_ISR1_FREQUENCY_HZ;
 extern uint32_t uINV_ACDC_AC_FREQ_HZ;
 extern RAMPGEN uINV_DCAC_rgen;
 extern float32_t uINV_DCAC_Index_Modulation;
 extern float32_t uINV_DCAC_Index_Modulation_Amp;
 extern float32_t uINV_DCAC_Index_Modulation_K;
 extern float32_t uINV_ACDC_Duty_Cycle;
 extern float32_t uINV_DCAC_Amplitude;

 extern int32_t uINV_DCAC_POS;

 extern int32_t uINV_ACDC_vAC_sensed_FILTER_POS;
 extern float32_t uINV_ACDC_vAC_sensed_FILTER_K_1;

// PWM Setting
 extern float32_t uINV_DCAC_Counter_PWM_Unit;
 //

/////////////////////////ACDC Settings//////////////////////////

// Virtual Oscilloscope
 extern DLOG_4CH uINV_DCAC_dLog1;
 extern DLOG_4CH uINV_DCAC_dLog2;
 extern float32_t uINV_DCAC_dBuff1[uINV_DCAC_DLOG_SIZE];
 extern float32_t uINV_DCAC_dBuff2[uINV_DCAC_DLOG_SIZE];
 extern float32_t uINV_DCAC_dBuff3[uINV_DCAC_DLOG_SIZE];
 extern float32_t uINV_DCAC_dBuff4[uINV_DCAC_DLOG_SIZE];
 extern float32_t uINV_DCAC_dBuff5[uINV_DCAC_DLOG_SIZE_OSC2];
 extern float32_t uINV_DCAC_dBuff6[uINV_DCAC_DLOG_SIZE_OSC2];
 extern float32_t uINV_DCAC_dBuff7[uINV_DCAC_DLOG_SIZE_OSC2];
 extern float32_t uINV_DCAC_dBuff8[uINV_DCAC_DLOG_SIZE_OSC2];
 extern float32_t uINV_DCAC_dVal1;
 extern float32_t uINV_DCAC_dVal2;
 extern float32_t uINV_DCAC_dVal3;
 extern float32_t uINV_DCAC_dVal4;
 extern float32_t uINV_DCAC_dVal5;
 extern float32_t uINV_DCAC_dVal6;
 extern float32_t uINV_DCAC_dVal7;
 extern float32_t uINV_DCAC_dVal8;
//

//Temperature Measurements
 extern float32_t uINV_ACDC_HS_GaN_Temp_Duty;
 extern float32_t uINV_ACDC_LS_GaN_Temp_Duty;
 extern float32_t uINV_ACDC_HS_GaN_Temp;
 extern float32_t uINV_ACDC_LS_GaN_Temp;
 extern uint32_t uINV_ACDC_GaN_Temp_Counter;

// 90 degree phase shifted
 extern float32_t uINV_ACDC_filter_1;
 extern float32_t uINV_ACDC_filter_2;
 extern float32_t uINV_ACDC_90_Shifted;

 extern SPLL_1PH_SOGI uINV_spll_1ph;



 extern int32_t uINV_DCAC_Relay_Enable;
//


//Analog Measurements
 extern float32_t uINV_ACDC_iAC_sensed_pu;
 extern float32_t uINV_DCAC_vAC_sensed_pu;
 extern float32_t uINV_ACDC_vDC_sensed_pu;
 extern float32_t uINV_ACDC_iDC_sensed_pu;


 extern float32_t uINV_ACDC_iAC_sensed;
 extern float32_t uINV_DCAC_vAC_sensed;
 extern float32_t uINV_ACDC_vDC_sensed;
 extern float32_t uINV_ACDC_iDC_sensed;
//

 // Internal conditioning of the measurements
 extern float32_t uINV_ACDC_vDC_sensed_FILTER;
 extern float32_t uINV_ACDC_vDC_sensed_NOTCH;

 extern float32_t uINV_ACDC_vAC_sensed_FILTER;

 extern float32_t uINV_ACDC_vAC_sensed_FILTER_1H;


//Control Variables
 extern volatile float32_t uINV_ACDC_iAC_real_Ref;
 extern volatile float32_t uINV_ACDC_iAC_imag_Ref;
 extern volatile float32_t uINV_DCAC_iAC_Ref;
 extern volatile float32_t uINV_ACDC_iAC_real_Ref_Prev;
 extern volatile float32_t uINV_ACDC_iAC_amp_Ref;
 extern volatile float32_t uINV_ACDC_iAC_real_Ref_OUT_PI;

 extern volatile float32_t uINV_ACDC_vDC_Ref;
 extern volatile float32_t uINV_ACDC_vDC_Ref_Slewed;



 //Control Variables Error
 extern float32_t uINV_ACDC_iAC_loop_err; // you control the amplitude of the real part
 extern float32_t  uINV_ACDC_vDC_loop_err; // you control the amplitude of the real part

 extern float32_t uINV_ACDC_iAC_loop_out_PI;
 extern float32_t uINV_ACDC_iAC_loop_out_PR1;
 extern float32_t uINV_ACDC_iAC_loop_out_PR3;
 extern float32_t uINV_ACDC_iAC_loop_out_PR5;
 extern float32_t uINV_ACDC_iAC_loop_out_PR7;
 extern float32_t uINV_ACDC_iAC_loop_out_PR9;
 extern float32_t uINV_ACDC_iAC_loop_out_PR11;
 extern float32_t uINV_ACDC_iAC_loop_out_PR13;
 extern float32_t uINV_ACDC_iAC_loop_out_PR15;
 extern float32_t uINV_ACDC_iAC_loop_out_SUM;

 extern DCL_DF22 gi_r1;
 extern DCL_DF22 gi_r3;
 extern DCL_DF22 gi_r5;
 extern DCL_DF22 gi_r7;
 extern DCL_DF22 gi_r9;
 extern DCL_DF22 gi_r11;
 extern DCL_DF22 gi_r13;
 extern DCL_DF22 gi_r15;

 extern DCL_DF22 gi_Vac_Filter; //Filtering out only the 50 Hz component as a FFW
 extern float32_t kiI_1H_Vac,woI_1H_Vac,wrcI_1H_Vac;

 extern  float32_t uINV_ACDC_FFW_K;
 extern  float32_t uINV_ACDC_FFW_K_Slewed;

 extern DCL_DF22 VDC_NOTCH_FILTER_2_Fe;


 extern float32_t kpI_1H;
 extern float32_t kiI_1H, kiI_3H, kiI_5H, kiI_7H, kiI_9H, kiI_11H, kiI_13H, kiI_15H;
 extern float32_t woI_1H, woI_3H, woI_5H, woI_7H, woI_9H, woI_11H, woI_13H, woI_15H;
 extern float32_t wrcI_1H, wrcI_3H, wrcI_5H, wrcI_7H, wrcI_9H, wrcI_11H, wrcI_13H, wrcI_15H;


 extern DCL_PI gi_ACDC_pi;
 extern DCL_PI gv_ACDC_pi;


//


//State Machines
 extern int32_t uINV_ACDC_fault;
 extern int32_t uINV_ACDC_flag;
 extern int32_t uINV_ACDC_stop;
 extern volatile uint16_t uINV_DCAC_clearPWMTrip;
 extern volatile uint16_t uINV_DCAC_update_Duty_Flag;


 extern volatile uint16_t uINV_EN_PR_K_calculation;

 extern volatile uint16_t uINV_DCAC_Counter_1;
 extern volatile uint16_t uINV_DCAC_Counter_Disable_FFW;

 extern volatile float32_t uINV_REF_CHANGE1; // you control the amplitude of the real part
 extern volatile float32_t uINV_REF_CHANGE2; // you control the amplitude of the real part
 extern volatile float32_t uINV_REF_CHANGE3; // you control the amplitude of the real part





 //Definition of the function in C
 void computeDF22_NotchFltrCoeff(DCL_DF22 *v, float32_t sampling_freq,
                                 float32_t notch_freq, float32_t c1, float32_t c2);
 void computeDF22_PRcontrollerCoeff(DCL_DF22 *v, float32_t kp, float32_t ki, float32_t wo,
                                    float32_t fs, float32_t wrc);
 //Definition of the function in C





//[MOD]
extern uint16_t uINV_ACDC_iAC_sensed_u16;
extern uint16_t uINV_ACDC_iAC_limit;
extern uint16_t uINV_ACDC_iAC_bool;
extern uint16_t uINV_ACDC_iAC_overcurrent;
extern uint16_t uINV_ACDC_iAC_overcurrent2;
extern float uINV_ACDC_iAC_current_ref;
extern float uINV_ACDC_iAC_current_ref_p;
extern float feedforward_k;
extern float feedforward_target_k;
extern float feedforward_inc_k;
extern float cross_threshold_var_into_pos;
extern float cross_threshold_var_into_neg;


extern float a1, b0, b1, b2;
extern float pr_w0, pr_wc, pr_ki, pr_kp;
extern uint32_t recompute_control_constants;

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_readCurrentAndVoltageSignals)
static inline void uINV_DCAC_readCurrentAndVoltageSignals()
{
    
    uINV_ACDC_iAC_sensed_u16 = ADC_readResult(SOC_DC_BUS_CS1_RESULT_BASE, SOC_DC_BUS_CS1);

    //combine later on the measurements
     uINV_ACDC_vDC_sensed_pu =  ADC_readResult(SOC_DC_BUS_VS_RESULT_BASE, SOC_DC_BUS_VS)*uINV_ACDC_ADC_PU_SCALE_FACTOR;  //
     uINV_ACDC_iAC_sensed_pu =  uINV_ACDC_iAC_sensed_u16*uINV_ACDC_ADC_PU_SCALE_FACTOR*2-1.0f; //bipolar
     uINV_DCAC_vAC_sensed_pu =  ADC_readResult(SOC_AC_VS_RESULT_BASE, SOC_AC_VS)*uINV_ACDC_ADC_PU_SCALE_FACTOR*2-1.0f; //bipolar
     uINV_ACDC_iDC_sensed_pu =  ADC_readResult(SOC_AC_CS1_RESULT_BASE, SOC_AC_CS1)*uINV_ACDC_ADC_PU_SCALE_FACTOR*2-1.0f; //bipolar

     uINV_ACDC_vDC_sensed = uINV_ACDC_vDC_sensed_pu*uINV_ACDC_vDC_MAX + uINV_ACDC_vDC_Offset;
     uINV_ACDC_iDC_sensed = uINV_ACDC_iDC_sensed_pu*uINV_ACDC_iDC_MAX + uINV_ACDC_iDC_Offset;
     uINV_DCAC_vAC_sensed = uINV_DCAC_vAC_sensed_pu*uINV_ACDC_vAC_MAX + uINV_ACDC_vAC_Offset;
     uINV_ACDC_iAC_sensed = uINV_ACDC_iAC_sensed_pu*uINV_ACDC_iAC_MAX + uINV_ACDC_iAC_Offset;


     //Low Pass Filtering of the voltages
     uINV_ACDC_vDC_sensed_FILTER += uINV_DC_FILTERING_CONSTANT*(uINV_ACDC_vDC_sensed-uINV_ACDC_vDC_sensed_FILTER);
     uINV_ACDC_vAC_sensed_FILTER += uINV_AC_FILTERING_CONSTANT*(uINV_DCAC_vAC_sensed-uINV_ACDC_vAC_sensed_FILTER);

     //PR for filtering out the Voltage
     uINV_ACDC_vAC_sensed_FILTER_1H= DCL_runDF22_C4(&gi_Vac_Filter, uINV_ACDC_vAC_sensed_FILTER);

     //Notch Filtering
     uINV_ACDC_vDC_sensed_NOTCH = DCL_runDF22_C4(&VDC_NOTCH_FILTER_2_Fe, uINV_ACDC_vDC_sensed_FILTER);

     //Overvoltageprotection

    if(uINV_ACDC_vDC_sensed_FILTER > uINV_ACDC_vDC_TRIP_LIMIT_V)
    {
     uINV_ACDC_fault=1;
    }
    
    //PMSA
    sine_mains.i = uINV_ACDC_iAC_sensed;
    sine_mains.v = uINV_DCAC_vAC_sensed*1.125;
    POWER_MEAS_SINE_ANALYZER_run(&sine_mains);
    guiIrms=sine_mains.iRms;
    guiVrms=sine_mains.vRms;
    guiPrms=sine_mains.pRms;

 }

 static inline void uINV_ACDC_clearPWMTripFlags(uint32_t base)
 {
     //
     // Clear all trip flags
     //
     EPWM_clearTripZoneFlag(base, (EPWM_TZ_INTERRUPT_OST |
                                   EPWM_TZ_INTERRUPT_CBC |
                                   EPWM_TZ_INTERRUPT_DCAEVT1));

     //
     // Clear all source flags for the One-shot trip zone
     //
     EPWM_clearOneShotTripZoneFlag(base, (EPWM_TZ_OST_FLAG_OST1 |
                                          EPWM_TZ_OST_FLAG_OST2 |
                                          EPWM_TZ_OST_FLAG_OST3 |
                                          EPWM_TZ_OST_FLAG_DCAEVT1));
}


static float phase = 0;

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_update_Duty)
static inline void uINV_DCAC_update_Duty()
{

    uINV_ACDC_iAC_overcurrent = EPWM_getTripZoneFlagStatus(DCAC_BASE) & EPWM_TZ_FLAG_DCBEVT1;

     //Saturation control. Avoid to reach extreme duty cycle conditions
     uINV_DCAC_Index_Modulation = (uINV_DCAC_Index_Modulation > (float32_t)(uINV_DCAC_Index_Modulation_HIGH_PU))?
             (float32_t)(uINV_DCAC_Index_Modulation_HIGH_PU):uINV_DCAC_Index_Modulation;
     uINV_DCAC_Index_Modulation = (uINV_DCAC_Index_Modulation < (float32_t)(uINV_DCAC_Index_Modulation_LOW_PU))?
             (float32_t)(uINV_DCAC_Index_Modulation_LOW_PU):uINV_DCAC_Index_Modulation;

    phase = phase + 0.0151f;
    float uINV_DCAC_Index_Modulation_sin = uINV_DCAC_Index_Modulation*__sinpuf32(phase);

    if(uINV_ACDC_iAC_overcurrent)
    {
        GPIO_writePin(uINV_DCAC_Relay,0);
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0);
        return;
        //uINV_DCDC_fail();
    }
    if(uINV_ACDC_iAC_bool){

        EPWM_setDeadBandOutputSwapMode(DCAC_BASE, EPWM_DB_OUTPUT_A, false);
        EPWM_setDeadBandDelayMode(DCAC_BASE,EPWM_DB_FED,true);

        if(uINV_DCAC_Index_Modulation_sin >= 0.0f){
            GPIO_writePin(uINV_DCAC_Top_LF, 0);
            GPIO_writePin(uINV_DCAC_Bottom_LF, 1);
            uINV_ACDC_Duty_Cycle=(uINV_DCAC_Index_Modulation_sin);
        }
        else{
            GPIO_writePin(uINV_DCAC_Bottom_LF, 0);
            GPIO_writePin(uINV_DCAC_Top_LF, 1);
            uINV_ACDC_Duty_Cycle=( 1 + uINV_DCAC_Index_Modulation_sin);
        }
        EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, uINV_ACDC_Duty_Cycle*uINV_DCAC_Counter_PWM_Unit);
        
    }
    else{
        GPIO_writePin(uINV_DCAC_Top_LF, 0);
        GPIO_writePin(uINV_DCAC_Bottom_LF, 0);
        EPWM_setDeadBandDelayMode(DCAC_BASE,EPWM_DB_FED,false);
        EPWM_setDeadBandOutputSwapMode(DCAC_BASE, EPWM_DB_OUTPUT_A, true);
    }
}

#pragma FUNC_ALWAYS_INLINE(uINV_RAMP_VDC)
static inline void uINV_RAMP_VDC()
{


        if((uINV_ACDC_vDC_Ref - uINV_ACDC_vDC_Ref_Slewed) > (2.0f * uINV_VOLTS_PER_SECOND_SLEW) *
                                        (ISR_CONTROL_FREQUENCY_AC_INVERSE))
     {
            uINV_ACDC_vDC_Ref_Slewed =  uINV_ACDC_vDC_Ref_Slewed + (uINV_VOLTS_PER_SECOND_SLEW) *
                               (ISR_CONTROL_FREQUENCY_AC_INVERSE);
     }
     else if((uINV_ACDC_vDC_Ref - uINV_ACDC_vDC_Ref_Slewed) <
                        - (2.0f * uINV_VOLTS_PER_SECOND_SLEW) *
                               (ISR_CONTROL_FREQUENCY_AC_INVERSE))
     {
         uINV_ACDC_vDC_Ref_Slewed =  uINV_ACDC_vDC_Ref_Slewed - (uINV_VOLTS_PER_SECOND_SLEW) *
                               (ISR_CONTROL_FREQUENCY_AC_INVERSE);
     }
     else
     {
         uINV_ACDC_vDC_Ref_Slewed =  uINV_ACDC_vDC_Ref;
     }


}

typedef enum CycleState{
    CYCLE_STATE_POS = 0,
    CYCLE_STATE_ZERO_CROSS_POS,
    CYCLE_STATE_STARTUP_INTO_NEG,
    CYCLE_STATE_NEG,
    CYCLE_STATE_ZERO_CROSS_NEG,
    CYCLE_STATE_STARTUP_INTO_POS,
    CYCLE_STATE_IDLE
} CycleState;


static inline bool uINV_DCAC_is_event_cross_down(float vk,float vk1,float threshold){
    return (vk < threshold) && (vk1 >= threshold);
}

static inline bool uINV_DCAC_is_event_cross_up(float vk,float vk1,float threshold){
    return (vk >= threshold) && (vk1 < threshold);
}

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_disable_HF)
static inline void uINV_DCAC_disable_HF(){
    EPWM_setDeadBandDelayMode(DCAC_BASE,EPWM_DB_FED,false);
    EPWM_setDeadBandOutputSwapMode(DCAC_BASE, EPWM_DB_OUTPUT_A, true);
}

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_enable_HF)
static inline void uINV_DCAC_enable_HF(){
    EPWM_setDeadBandOutputSwapMode(DCAC_BASE, EPWM_DB_OUTPUT_A, false);
    EPWM_setDeadBandDelayMode(DCAC_BASE,EPWM_DB_FED,true);
}



// #define Kt ((float)4.1667e+04)
#define Kt ((float)62500.0f)
#define Kt2 (Kt*Kt)
static void uINV_DCAC_compute_control_constants(){
    float pr_w0_sq = pr_w0*pr_w0;
    a1 = 2*pr_ki*Kt*pr_wc;
    b0 = Kt2 + 2*Kt*pr_wc + pr_w0_sq;
    b1 = 2*Kt2 - 2*pr_w0_sq;
    b2 = b0;
}


static float error_current_k1 = 0;
static float error_current_k2 = 0;
static float vctrl_k1 = 0;
static float vctrl_k2 = 0;

static void uINV_DCAC_reset_PR(){
    gi_r1.x1 = 0.0f;
    gi_r1.x2 = 0.0f;

    gi_r3.x1 = 0.0f;
    gi_r3.x2 = 0.0f;

    gi_r5.x1 = 0.0f;
    gi_r5.x2 = 0.0f;

    gi_r7.x1 = 0.0f;
    gi_r7.x2 = 0.0f;

    gi_r9.x1 = 0.0f;
    gi_r9.x2 = 0.0f;

    gi_r11.x1 = 0.0f;
    gi_r11.x2 = 0.0f;

    // gi_r13.x1 = 0.0f;
    // gi_r13.x2 = 0.0f;

    // gi_r15.x1 = 0.0f;
    // gi_r15.x2 = 0.0f;

    error_current_k2 = 0.0f;
    error_current_k1 = 0.0f;
    vctrl_k2 = 0.0f;
    vctrl_k1 = 0.0f;	

}





#define PLL_mu1 (222.0f)
#define PLL_mu2 ((float)1.9894e+03)
#define PLL_mu3 (35.3324f)
#define PLL_K (0.025f)

#define PLL_f0 (50.0f)
#define PLL_U0 (325.0f)

#define PLL_TS ((float)32e-6)

typedef struct PLLState{
    float dU_k1;
    float dv_U_k1;
    float dw_k1;
    float dv_w_k1;
    float theta_k1;
    float w_k1;

    float vout_k;
    float sin_k;
    float cos_k;
    float U_k;
} PLLState;

static PLLState PLL_state;

static void uINV_DCAC_pll_run(float vin){
    float error_k = vin - PLL_state.vout_k;

    float dv_U_k = error_k*PLL_state.sin_k*PLL_mu1;
    float dU_k = PLL_state.dU_k1 + PLL_TS*PLL_state.dv_U_k1;
    float U_k = dU_k + PLL_U0;

    float error_phase_k =  error_k*PLL_K*PLL_state.cos_k;
    float dv_w_k = error_phase_k*PLL_mu2;
    float dw_k = PLL_state.dw_k1 + PLL_TS*PLL_state.dv_w_k1;
    float w_k = dw_k + PLL_f0 + PLL_mu3*error_phase_k;

    float theta_k = PLL_state.theta_k1 + PLL_TS*PLL_state.w_k1;
    if(theta_k >= 1.0f){
        theta_k = theta_k - 1.0f;
    }
    else if(theta_k <= -1.0f){
        theta_k = theta_k + 1.0f;
    }

    PLL_state.sin_k = __sinpuf32(theta_k);
    PLL_state.cos_k = __cospuf32(theta_k);
    PLL_state.vout_k = U_k*PLL_state.sin_k;
    PLL_state.U_k = U_k;

    PLL_state.dU_k1 = dU_k;
    PLL_state.dv_U_k1 = dv_U_k;
    PLL_state.dw_k1 = dw_k;
    PLL_state.dv_w_k1 = dv_w_k;
    PLL_state.theta_k1 = theta_k;
    PLL_state.w_k1 = w_k;
}

#define VCTRL_CNT_MAX (400)
static float vctrl_array[VCTRL_CNT_MAX];
static uint32_t vctrl_array_cnt = 0; 

extern uint32_t action_pr_flag;
extern uint32_t action_pr_frozen;


#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_compute_control)
static inline float uINV_DCAC_compute_control(float ref_k){
    float error_current_k = ref_k - uINV_ACDC_iAC_sensed;

    float vctrl_pr = a1*(error_current_k1 - error_current_k2) + b1*vctrl_k1 - b2*vctrl_k2;
    vctrl_pr = vctrl_pr/b0;
    float vctrl = vctrl_pr + pr_kp*error_current_k;

    error_current_k2 = error_current_k1;
    error_current_k1 = error_current_k;
    vctrl_k2 = vctrl_k1;
    vctrl_k1 = vctrl_pr;
    // if(action_pr_frozen != 0){
    //     vctrl_k2 = vctrl_k1;
    //     vctrl_k1 = vctrl_pr;	
    // }



    //uINV_ACDC_iAC_loop_out_PI = DCL_runPI_C2(&gi_ACDC_pi,err_k,0);
    //uINV_ACDC_iAC_loop_out_PR1 = DCL_runDF22_C1(&gi_r1, error_current_k);  //uINV_ACDC_vAC_sensed_FILTER
    if(action_pr_flag != 0){
        uINV_ACDC_iAC_loop_out_PR3 = DCL_runDF22_C1(&gi_r3, error_current_k);
        uINV_ACDC_iAC_loop_out_PR5 = DCL_runDF22_C1(&gi_r5, error_current_k);
        uINV_ACDC_iAC_loop_out_PR7 = DCL_runDF22_C1(&gi_r7, error_current_k);
        uINV_ACDC_iAC_loop_out_PR9 = DCL_runDF22_C1(&gi_r9, error_current_k);
        uINV_ACDC_iAC_loop_out_PR11 = DCL_runDF22_C1(&gi_r11, error_current_k);
        uINV_ACDC_iAC_loop_out_PR13 = DCL_runDF22_C1(&gi_r13, error_current_k);
        vctrl = vctrl + uINV_ACDC_iAC_loop_out_PR3 + uINV_ACDC_iAC_loop_out_PR5 + uINV_ACDC_iAC_loop_out_PR7 + uINV_ACDC_iAC_loop_out_PR9 + uINV_ACDC_iAC_loop_out_PR11 + uINV_ACDC_iAC_loop_out_PR13;
    }

    if((feedforward_target_k > feedforward_k) && (feedforward_k < 1.0f)){
        feedforward_k += feedforward_inc_k;
    }
    else if((feedforward_target_k < feedforward_k) && (feedforward_k > 0.0f)){
        feedforward_k -= feedforward_inc_k;
    }
    //float vctrl = uINV_ACDC_iAC_loop_out_PR1;

    vctrl_array[vctrl_array_cnt] = vctrl;
    vctrl_array_cnt = vctrl_array_cnt + 1;
    if(vctrl_array_cnt == VCTRL_CNT_MAX){
        vctrl_array_cnt = 0;
    }
    return (feedforward_k*PLL_state.vout_k + vctrl);
    //return (vctrl);
}

static inline void uINV_DCAC_force_reset(float value){
    error_current_k2 = uINV_DCAC_iAC_Ref - uINV_ACDC_iAC_sensed;
    error_current_k1 = error_current_k2;
    vctrl_k2 = value;
    vctrl_k1 = value;
}

static inline void uINV_DCAC_reverse_pr(){
    error_current_k2 = -error_current_k2;
    error_current_k1 = -error_current_k1;
    vctrl_k2 = -vctrl_k2;
    vctrl_k1 = -vctrl_k1;
}

static inline float uINV_DCAC_compute_control_frozen(float ref_k){
    float error_current_k = ref_k - uINV_ACDC_iAC_sensed;

    float vctrl_pr = a1*(error_current_k1 - error_current_k2) + b1*vctrl_k1 - b2*vctrl_k2;
    vctrl_pr = vctrl_pr/b0;
    float vctrl = vctrl_pr + pr_kp*error_current_k;

    error_current_k2 = error_current_k1;
    error_current_k1 = error_current_k;
    // vctrl_k2 = vctrl_k1;
    // vctrl_k1 = vctrl_pr;	

    if((feedforward_target_k > feedforward_k) && (feedforward_k < 1.0f)){
        feedforward_k += feedforward_inc_k;
    }
    else if((feedforward_target_k < feedforward_k) && (feedforward_k > 0.0f)){
        feedforward_k -= feedforward_inc_k;
    }

    vctrl_array[vctrl_array_cnt] = vctrl;
    vctrl_array_cnt = vctrl_array_cnt + 1;
    if(vctrl_array_cnt == VCTRL_CNT_MAX){
        vctrl_array_cnt = 0;
    }
    //return (feedforward_k*uINV_DCAC_vAC_sensed + vctrl);
    return (vctrl);
}

static CycleState cycle_state = CYCLE_STATE_IDLE;
static uint32_t cycle_state_counter = 0;
static float sin_k1 = 0.0f;
#define CYCLE_STATE_COUNTER_MAX (7)
extern float cross_threshold_var;
#define CROSS_THRESHOLD (0.015f)

static const float a_into_neg_startup_table[CYCLE_STATE_COUNTER_MAX] = {
 0.750000,
 0.800000,
 0.850000,
 0.900000,
 1.000000,
 1.000000,
 1.000000,
};

static const float a_into_pos_startup_table[CYCLE_STATE_COUNTER_MAX] = {
 0.250000,
 0.200000,
 0.150000,
 0.100000,
 0.000000,
 0.000000,
 0.000000,
};

static const float b_startup_table[CYCLE_STATE_COUNTER_MAX] = {
 0.500000,
 0.600000,
 0.700000,
 0.800000,
 1.000000,
 1.000000,
 1.000000,
};

    // SOFT_START_CODE
    // if(soft_start_index < CYCLE_STATE_COUNTER_MAX){
    //     if(uINV_DCAC_Index_Modulation <= 0.0f){
    //         uINV_ACDC_Duty_Cycle = 0.0f;
    //     }
    //     else{
    //         uINV_ACDC_Duty_Cycle = uINV_DCAC_Index_Modulation;
    //     }
    //     float a = a_into_pos_startup_table[soft_start_index];
    //     float b = b_startup_table[soft_start_index];
    //     uINV_ACDC_Duty_Cycle = a + uINV_ACDC_Duty_Cycle*b;
    //     if(uINV_ACDC_Duty_Cycle >= 0.90f){
    //         uINV_ACDC_Duty_Cycle = 0.90f;
    //     }
    //     soft_start_index++;
    //     if(soft_start_index == CYCLE_STATE_COUNTER_MAX - 2){
    //         GPIO_writePin(uINV_DCAC_Bottom_LF,1);
    //     };
    // }

    // SOFT_START_CODE
    // if(soft_start_index < CYCLE_STATE_COUNTER_MAX){
    //     if(uINV_DCAC_Index_Modulation >= 0.0f){
    //         uINV_ACDC_Duty_Cycle = 0.0f;
    //     }
    //     else{
    //         uINV_ACDC_Duty_Cycle = uINV_DCAC_Index_Modulation;
    //     }
    //     float a = a_into_neg_startup_table[soft_start_index];
    //     float b = b_startup_table[soft_start_index];
    //     uINV_ACDC_Duty_Cycle = a + uINV_ACDC_Duty_Cycle*b;
    //     if(uINV_ACDC_Duty_Cycle >= 0.95f){
    //         uINV_ACDC_Duty_Cycle = 0.95f;
    //     }
    //     soft_start_index++;
    //     if(soft_start_index == CYCLE_STATE_COUNTER_MAX - 2){
    //         GPIO_writePin(uINV_DCAC_Top_LF,1);
    //     };
    // }   


#define DEBUG_ARRAY_MAX 20

static float debug_array_iac[DEBUG_ARRAY_MAX];
static float debug_array_vac[DEBUG_ARRAY_MAX];
static float debug_array_vbus[DEBUG_ARRAY_MAX];
static float debug_array_duty_01[DEBUG_ARRAY_MAX];
static float debug_array_duty_11[DEBUG_ARRAY_MAX];
static CycleState debug_array_state[DEBUG_ARRAY_MAX];
static uint32_t debug_array_counter = 0;


typedef enum ModulationIndexState{
    MODULATION_INDEX_POSITIVE = 0,
    MODULATION_INDEX_NEGATIVE
} ModulationIndexState;

static uint32_t modulation_index_state_cnt = 0;
static uint32_t soft_start_index = 0;
static ModulationIndexState modulation_index_state;

#define BLANK_TIME_1 (4)
#define BLANK_TIME_2 (104)

extern float min_action_after_cross_volts;

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_runISR1)  
static inline void uINV_DCAC_runISR1(){
    CMPSS_setDACValueHigh(myCMPSS_BASE, 2040 + uINV_ACDC_iAC_limit);
    CMPSS_setDACValueLow(myCMPSS_BASE, 2040 - uINV_ACDC_iAC_limit);

    
    float sin_k = PLL_state.sin_k;
    float cos_k = PLL_state.cos_k;

    // phase = phase + 0.0016f;
    // if(phase >= 1.0f){
    //     phase = 1.0f - phase;
    // }

    // float sin_k = __sinpuf32(phase);
    // float cos_k = __cospuf32(phase);


    // if(sin_k >= 0){
    //     GPIO_writePin(uINV_DCAC_TestPoint_2,1);
    // }
    // else{
    //     GPIO_writePin(uINV_DCAC_TestPoint_2,0);
    // }


    if(recompute_control_constants){
        
        if(recompute_control_constants == 1){
            uINV_DCAC_compute_control_constants();
        }
        else if(recompute_control_constants == 3){
            computeDF22_PRcontrollerCoeff(&gi_r3, 0,kiI_3H,woI_3H,
                                        ISR_CONTROL_FREQUENCY_AC,wrcI_3H);
        }
        else if(recompute_control_constants == 5){
            computeDF22_PRcontrollerCoeff(&gi_r5, 0,kiI_5H,woI_5H,
                                        ISR_CONTROL_FREQUENCY_AC,wrcI_5H);
        }
        else if(recompute_control_constants == 7){
            computeDF22_PRcontrollerCoeff(&gi_r7, 0,kiI_7H,woI_7H,
                                        ISR_CONTROL_FREQUENCY_AC,wrcI_7H);
        }
        else if(recompute_control_constants == 9){
            computeDF22_PRcontrollerCoeff(&gi_r9, 0,kiI_9H,woI_9H,
                                        ISR_CONTROL_FREQUENCY_AC,wrcI_9H);
        }
        else if(recompute_control_constants == 11){
            computeDF22_PRcontrollerCoeff(&gi_r11, 0,kiI_11H,woI_11H,
                                        ISR_CONTROL_FREQUENCY_AC,wrcI_11H);
        }
        else if(recompute_control_constants == 13){
            computeDF22_PRcontrollerCoeff(&gi_r13, 0,kiI_13H,woI_13H,
                                        ISR_CONTROL_FREQUENCY_AC,wrcI_13H);
        }
        recompute_control_constants = 0;
    }

    if(uINV_DCAC_clearPWMTrip == 1U)
    {
        uINV_DCAC_update_Duty_Flag=0; 

        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0); 
        uINV_DCAC_reset_PR(); 
        EPWM_clearTripZoneFlag(DCAC_BASE, EPWM_TZ_INTERRUPT |EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCBEVT1 | EPWM_TZ_FLAG_DCAEVT1);

        // Wait for zero-crossing then clear trips and start power stage
        if(uINV_DCAC_is_event_cross_up(sin_k,sin_k1, -0.015f))
            { 
                uINV_DCAC_clearPWMTrip = 0;
                uINV_DCAC_update_Duty_Flag=1;
                uINV_DCAC_Counter_1=0;

                modulation_index_state = MODULATION_INDEX_POSITIVE;
                modulation_index_state_cnt = 0;
                soft_start_index = 0;
                uINV_DCAC_enable_HF();

                feedforward_k = 1.0f;
                debug_array_counter = 0;
                GPIO_writePin(uINV_DCAC_TestPoint_2,0);
            }
    }

    uINV_ACDC_iAC_overcurrent = EPWM_getTripZoneFlagStatus(DCAC_BASE) & EPWM_TZ_FLAG_DCBEVT1;
    if(uINV_ACDC_iAC_overcurrent || uINV_ACDC_fault){
        //GPIO_writePin(uINV_DCAC_Relay,0);
        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0);
        GPIO_writePin(uINV_DCAC_TestPoint_2,1);
    }
    else if(uINV_ACDC_iAC_bool){
        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0);

        uINV_DCAC_update_Duty_Flag = 0;
        uINV_ACDC_iAC_bool = 0;
    }
    else if(uINV_DCAC_update_Duty_Flag == 1U)
    {

        uINV_ACDC_vDC_loop_err = uINV_ACDC_vDC_Ref_Slewed-uINV_ACDC_vDC_sensed_NOTCH;
        uINV_ACDC_iAC_real_Ref_OUT_PI= -DCL_runPI_C2(&gv_ACDC_pi,uINV_ACDC_vDC_loop_err,0);
        uINV_DCAC_iAC_Ref =uINV_ACDC_iAC_real_Ref_OUT_PI*(sin_k)+uINV_ACDC_iAC_imag_Ref*(cos_k);

        float min_duty_after_cross_into_pos = (min_action_after_cross_volts/uINV_ACDC_vDC_sensed_FILTER);
        float max_duty_after_cross_into_neg = 1.0f - min_duty_after_cross_into_pos;
        
        if(modulation_index_state_cnt > BLANK_TIME_1){
            uINV_DCAC_Index_Modulation = uINV_DCAC_compute_control(uINV_DCAC_iAC_Ref)/(uINV_ACDC_vDC_sensed_FILTER);
            if(modulation_index_state == MODULATION_INDEX_POSITIVE){  
                uINV_ACDC_Duty_Cycle = uINV_DCAC_Index_Modulation;
                if(modulation_index_state_cnt >= BLANK_TIME_2){    
                    if(uINV_DCAC_Index_Modulation <= cross_threshold_var_into_neg){
                        modulation_index_state = MODULATION_INDEX_NEGATIVE;
                        modulation_index_state_cnt = 0;
                        soft_start_index = 0;
                    }
                }
                else if(uINV_ACDC_Duty_Cycle <= min_duty_after_cross_into_pos){
                    uINV_ACDC_Duty_Cycle = min_duty_after_cross_into_pos;
                }
            }
            else if(modulation_index_state == MODULATION_INDEX_NEGATIVE){
                uINV_ACDC_Duty_Cycle = 1.0f + uINV_DCAC_Index_Modulation;
                if(modulation_index_state_cnt >= BLANK_TIME_2){
                    if(uINV_DCAC_Index_Modulation >= cross_threshold_var_into_pos){
                        modulation_index_state = MODULATION_INDEX_POSITIVE;
                        modulation_index_state_cnt = 0;
                        soft_start_index = 0;
                    }
                }
                else if(uINV_ACDC_Duty_Cycle >= max_duty_after_cross_into_neg){
                    uINV_ACDC_Duty_Cycle = max_duty_after_cross_into_neg;
                }
            }
        }
        else if(modulation_index_state_cnt == (BLANK_TIME_1 - 1)){
            
            if(modulation_index_state == MODULATION_INDEX_NEGATIVE){
                GPIO_writePin(uINV_DCAC_Top_LF,1);
                uINV_ACDC_Duty_Cycle = max_duty_after_cross_into_neg;
            }
            else if(modulation_index_state == MODULATION_INDEX_POSITIVE){
                GPIO_writePin(uINV_DCAC_Bottom_LF,1);
                uINV_ACDC_Duty_Cycle = min_duty_after_cross_into_pos;
            }
            
        }
        else if(modulation_index_state_cnt == BLANK_TIME_1){
            uINV_DCAC_enable_HF();
        }
        else{
            uINV_DCAC_disable_HF();
            GPIO_writePin(uINV_DCAC_Top_LF,0);
            GPIO_writePin(uINV_DCAC_Bottom_LF,0); 
        }

        if(uINV_ACDC_Duty_Cycle <= 0.0f){
            uINV_ACDC_Duty_Cycle = 0.0f;
        }else if(uINV_ACDC_Duty_Cycle >= 1.0f){
            uINV_ACDC_Duty_Cycle = 1.0f;
        }
        EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, uINV_ACDC_Duty_Cycle*uINV_DCAC_Counter_PWM_Unit);

        modulation_index_state_cnt++;
    }

    sin_k1 = sin_k;
    uINV_DCAC_pll_run(uINV_DCAC_vAC_sensed);
    
}


#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_runISR1_lab1)  //Run in open loop
static inline void uINV_DCAC_runISR1_lab1()
 {
     
    //GPIO_writePin(uINV_DCAC_TestPoint_2,1);
    CMPSS_setDACValueHigh(myCMPSS_BASE, 2040 + uINV_ACDC_iAC_limit);
    CMPSS_setDACValueLow(myCMPSS_BASE, 2040 - uINV_ACDC_iAC_limit);

    uINV_DCAC_pll_run(uINV_DCAC_vAC_sensed);
    //SPLL_1PH_SOGI_run(&uINV_spll_1ph,uINV_DCAC_vAC_sensed_pu);
    
    // phase = phase + 0.0025f;
    // if(phase >= 1.0f){
    //     phase = 1.0f - phase;
    // }

    // float sin_k = __sinpuf32(phase);
    // float cos_k = __cospuf32(phase);
    float sin_k = PLL_state.sin_k;
    float cos_k = PLL_state.cos_k;
    // float sin_k = uINV_spll_1ph.sine;
    // float cos_k = uINV_spll_1ph.cosine;

    if(sin_k >= 0){
        GPIO_writePin(uINV_DCAC_TestPoint_2,1);
    }
    else{
        GPIO_writePin(uINV_DCAC_TestPoint_2,0);
    }


    if(recompute_control_constants){
        recompute_control_constants = 0;
        uINV_DCAC_compute_control_constants();
        // computeDF22_PRcontrollerCoeff(&gi_r1, kpI_1H,kiI_1H,woI_1H,
        //                             ISR_CONTROL_FREQUENCY_AC,wrcI_1H);
    }
    
    

    if(uINV_DCAC_clearPWMTrip == 1U)
    {
        uINV_DCAC_update_Duty_Flag=0; 

        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0); 
        uINV_DCAC_reset_PR(); 

        EPWM_clearTripZoneFlag(DCAC_BASE, EPWM_TZ_INTERRUPT |EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCBEVT1 | EPWM_TZ_FLAG_DCAEVT1);

        // Wait for zero-crossing then clear trips and start power stage
        if(uINV_DCAC_is_event_cross_up(sin_k,sin_k1, cross_threshold_var_into_pos))
            { 
                uINV_DCAC_clearPWMTrip = 0;
                uINV_DCAC_update_Duty_Flag=1;
                uINV_DCAC_Counter_1=0;

                cycle_state = CYCLE_STATE_STARTUP_INTO_POS;
                uINV_DCAC_enable_HF();
                cycle_state_counter = 0;

                feedforward_k = 1.0f;
                //feedforward_target_k = 0.0f;

                debug_array_counter = 0;
            }
    }
    
    uINV_ACDC_iAC_overcurrent = EPWM_getTripZoneFlagStatus(DCAC_BASE) & EPWM_TZ_FLAG_DCBEVT1;
    if(uINV_ACDC_iAC_overcurrent || uINV_ACDC_fault){
        //GPIO_writePin(uINV_DCAC_Relay,0);
        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0);
        
    }
    else if(uINV_ACDC_iAC_bool){
        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0);

        uINV_DCAC_update_Duty_Flag = 0;
        uINV_ACDC_iAC_bool = 0;
    }
    else if(uINV_DCAC_update_Duty_Flag == 1U)
    {

        uINV_ACDC_vDC_loop_err = uINV_ACDC_vDC_Ref_Slewed-uINV_ACDC_vDC_sensed_NOTCH;
        uINV_ACDC_iAC_real_Ref_OUT_PI= -DCL_runPI_C2(&gv_ACDC_pi,uINV_ACDC_vDC_loop_err,0);
        uINV_DCAC_iAC_Ref =uINV_ACDC_iAC_real_Ref_OUT_PI*(sin_k)+uINV_ACDC_iAC_imag_Ref*(cos_k);
        

        switch (cycle_state) {
            case CYCLE_STATE_POS: {
                if(uINV_DCAC_is_event_cross_down(sin_k,sin_k1, cross_threshold_var_into_neg)){
                    uINV_DCAC_disable_HF();
                    GPIO_writePin(uINV_DCAC_Top_LF,0);
                    GPIO_writePin(uINV_DCAC_Bottom_LF,0);  
                    cycle_state = CYCLE_STATE_ZERO_CROSS_POS;
                    cycle_state_counter = 0;
                }
                else{
                    
                    uINV_DCAC_Index_Modulation = uINV_DCAC_compute_control(uINV_DCAC_iAC_Ref)/(uINV_ACDC_vDC_sensed_FILTER);
                    if(uINV_DCAC_Index_Modulation < 0.0f){
                        uINV_DCAC_Index_Modulation = 0.0f;
                    }
                    else if(uINV_DCAC_Index_Modulation > 1.0f){
                        uINV_DCAC_Index_Modulation = 1.0f;
                    }

                    uINV_ACDC_Duty_Cycle = uINV_DCAC_Index_Modulation;
                    EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, uINV_ACDC_Duty_Cycle*uINV_DCAC_Counter_PWM_Unit);
                }

            }; break;
            case CYCLE_STATE_ZERO_CROSS_POS: {
                cycle_state_counter++; 
                if(cycle_state_counter == 1)   {
                    cycle_state = CYCLE_STATE_STARTUP_INTO_NEG;
                    cycle_state_counter = 0;
                    uINV_DCAC_enable_HF();
                }
            }; break;
            case CYCLE_STATE_STARTUP_INTO_NEG: {
                uINV_DCAC_Index_Modulation = uINV_DCAC_compute_control(uINV_DCAC_iAC_Ref)/(uINV_ACDC_vDC_sensed_FILTER);
                if(uINV_DCAC_Index_Modulation >= 0.0f){
                    uINV_ACDC_Duty_Cycle = 0.0f;
                }
                else{
                    uINV_ACDC_Duty_Cycle = uINV_DCAC_Index_Modulation;
                }
                float a = a_into_neg_startup_table[cycle_state_counter];
                float b = b_startup_table[cycle_state_counter];
                uINV_ACDC_Duty_Cycle = a + uINV_ACDC_Duty_Cycle*b;
                if(uINV_ACDC_Duty_Cycle >= 0.95f){
                    uINV_ACDC_Duty_Cycle = 0.95f;
                }
                EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, uINV_ACDC_Duty_Cycle*uINV_DCAC_Counter_PWM_Unit);

                cycle_state_counter++; 
                if(cycle_state_counter == CYCLE_STATE_COUNTER_MAX)   {
                    GPIO_writePin(uINV_DCAC_Top_LF,1); 
                    cycle_state = CYCLE_STATE_NEG;
                    cycle_state_counter = 0;
                }
            }; break;
            case CYCLE_STATE_NEG: {
                if(uINV_DCAC_is_event_cross_up(sin_k,sin_k1, cross_threshold_var_into_pos)){
                    uINV_DCAC_disable_HF();
                    GPIO_writePin(uINV_DCAC_Top_LF,0);
                    GPIO_writePin(uINV_DCAC_Bottom_LF,0);  
                    cycle_state = CYCLE_STATE_ZERO_CROSS_NEG;
                    cycle_state_counter = 0;
                }
                else{
                    
                    uINV_DCAC_Index_Modulation = uINV_DCAC_compute_control(uINV_DCAC_iAC_Ref)/(uINV_ACDC_vDC_sensed_FILTER);
                    if(uINV_DCAC_Index_Modulation > 0.0f){
                        uINV_DCAC_Index_Modulation = 0.0f;
                    }
                    else if(uINV_DCAC_Index_Modulation < -1.0f){
                        uINV_DCAC_Index_Modulation = -1.0f;
                    }

                    uINV_ACDC_Duty_Cycle = 1.0f + uINV_DCAC_Index_Modulation;
                    
                    EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, uINV_ACDC_Duty_Cycle*uINV_DCAC_Counter_PWM_Unit);
                }
            }; break;
            case CYCLE_STATE_ZERO_CROSS_NEG: {
                 cycle_state_counter++; 
                if(cycle_state_counter == 1)   {
                    cycle_state = CYCLE_STATE_STARTUP_INTO_POS;
                    cycle_state_counter = 0;
                    uINV_DCAC_enable_HF();
                }
            }; break;
            case CYCLE_STATE_STARTUP_INTO_POS: {
                uINV_DCAC_Index_Modulation = uINV_DCAC_compute_control(uINV_DCAC_iAC_Ref)/(uINV_ACDC_vDC_sensed_FILTER);
                if(uINV_DCAC_Index_Modulation <= 0.0f){
                    uINV_ACDC_Duty_Cycle = 0.0f;
                }
                else{
                    uINV_ACDC_Duty_Cycle = uINV_DCAC_Index_Modulation;
                }
                float a = a_into_pos_startup_table[cycle_state_counter];
                float b = b_startup_table[cycle_state_counter];
                uINV_ACDC_Duty_Cycle = a + uINV_ACDC_Duty_Cycle*b;
                if(uINV_ACDC_Duty_Cycle >= 0.95f){
                    uINV_ACDC_Duty_Cycle = 0.95f;
                }
                EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, uINV_ACDC_Duty_Cycle*uINV_DCAC_Counter_PWM_Unit);
                
                cycle_state_counter++; 
                if(cycle_state_counter == CYCLE_STATE_COUNTER_MAX)   {
                    GPIO_writePin(uINV_DCAC_Bottom_LF,1); 
                    cycle_state = CYCLE_STATE_POS;
                    cycle_state_counter = 0;
                }
            }; break;
        }
        debug_array_iac[debug_array_counter] = uINV_ACDC_iAC_sensed;
        // debug_array_vac[debug_array_counter] = uINV_DCAC_vAC_sensed;
        // debug_array_vbus[debug_array_counter] = uINV_ACDC_vDC_sensed_FILTER;
        // debug_array_duty_01[debug_array_counter] = uINV_ACDC_Duty_Cycle;
        // debug_array_duty_11[debug_array_counter] = uINV_DCAC_Index_Modulation;
        // debug_array_state[debug_array_counter] = cycle_state;
        debug_array_counter++;
        if(debug_array_counter == DEBUG_ARRAY_MAX){
            debug_array_counter = 0;
        }
    }



    sin_k1 = sin_k;
    //uINV_DCAC_pll_run(uINV_DCAC_vAC_sensed);
    //GPIO_writePin(uINV_DCAC_TestPoint_2,0);
    
    uINV_DCAC_dVal1 = uINV_DCAC_Index_Modulation;
    uINV_DCAC_dVal2 = uINV_DCAC_vAC_sensed_pu;
    uINV_DCAC_dVal3 = uINV_DCAC_vAC_sensed ;
    uINV_DCAC_dVal4 = uINV_DCAC_Index_Modulation;
    DLOG_4CH_run(&uINV_DCAC_dLog1);
     //

 }



#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_runISR1_lab1_noss)  //Run in open loop
static inline void uINV_DCAC_runISR1_lab1_noss()
 {
     
    //GPIO_writePin(uINV_DCAC_TestPoint_2,1);
    CMPSS_setDACValueHigh(myCMPSS_BASE, 2040 + uINV_ACDC_iAC_limit);
    CMPSS_setDACValueLow(myCMPSS_BASE, 2040 - uINV_ACDC_iAC_limit);


    SPLL_1PH_SOGI_run(&uINV_spll_1ph,uINV_DCAC_vAC_sensed_pu);
    
    // if(uINV_spll_1ph.sine >= 0){
    //     GPIO_writePin(uINV_DCAC_TestPoint_2,1);
    // }
    // else{
    //     GPIO_writePin(uINV_DCAC_TestPoint_2,0);
    // }

    // float sin_k = PLL_state.sin_k;
    // float cos_k = PLL_state.cos_k;

    float sin_k = uINV_spll_1ph.sine;
    float cos_k = uINV_spll_1ph.cosine;

    if(recompute_control_constants){
        recompute_control_constants = 0;
        uINV_DCAC_compute_control_constants();
        // computeDF22_PRcontrollerCoeff(&gi_r1, kpI_1H,kiI_1H,woI_1H,
        //                             ISR_CONTROL_FREQUENCY_AC,wrcI_1H);
    }
    
    

    if(uINV_DCAC_clearPWMTrip == 1U)
    {
        uINV_DCAC_update_Duty_Flag=0; 

        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0); 
        uINV_DCAC_reset_PR(); 

        EPWM_clearTripZoneFlag(DCAC_BASE, EPWM_TZ_INTERRUPT |EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCBEVT1 | EPWM_TZ_FLAG_DCAEVT1);

        // Wait for zero-crossing then clear trips and start power stage
        if(uINV_DCAC_is_event_cross_up(sin_k,sin_k1, -cross_threshold_var))
            { 
                uINV_DCAC_clearPWMTrip = 0;
                uINV_DCAC_update_Duty_Flag=1;
                uINV_DCAC_Counter_1=0;

                cycle_state = CYCLE_STATE_ZERO_CROSS_NEG;
                cycle_state_counter = 0;

                feedforward_k = 1.0f;
                //feedforward_target_k = 0.0f;

                debug_array_counter = 0;
            }
    }
    
    uINV_ACDC_iAC_overcurrent = EPWM_getTripZoneFlagStatus(DCAC_BASE) & EPWM_TZ_FLAG_DCBEVT1;
    if(uINV_ACDC_iAC_overcurrent || uINV_ACDC_fault){
        //GPIO_writePin(uINV_DCAC_Relay,0);
        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0);
        
    }
    else if(uINV_ACDC_iAC_bool){
        uINV_DCAC_disable_HF();
        GPIO_writePin(uINV_DCAC_Top_LF,0);
        GPIO_writePin(uINV_DCAC_Bottom_LF,0);

        uINV_DCAC_update_Duty_Flag = 0;
        uINV_ACDC_iAC_bool = 0;
    }
    else if(uINV_DCAC_update_Duty_Flag == 1U)
    {

        uINV_ACDC_vDC_loop_err = uINV_ACDC_vDC_Ref_Slewed-uINV_ACDC_vDC_sensed_NOTCH;
        uINV_ACDC_iAC_real_Ref_OUT_PI= -DCL_runPI_C2(&gv_ACDC_pi,uINV_ACDC_vDC_loop_err,0);
        uINV_DCAC_iAC_Ref =uINV_ACDC_iAC_real_Ref_OUT_PI*(sin_k)+uINV_ACDC_iAC_imag_Ref*(cos_k);
        

        switch (cycle_state) {
            case CYCLE_STATE_POS: {
                if(uINV_DCAC_is_event_cross_down(sin_k,sin_k1, cross_threshold_var)){
                    uINV_DCAC_disable_HF();
                    GPIO_writePin(uINV_DCAC_Top_LF,0);
                    GPIO_writePin(uINV_DCAC_Bottom_LF,0);  
                    cycle_state = CYCLE_STATE_ZERO_CROSS_POS;
                    cycle_state_counter = 0;
                }
                else{
                    
                    uINV_DCAC_Index_Modulation = uINV_DCAC_compute_control(uINV_DCAC_iAC_Ref)/(uINV_ACDC_vDC_sensed_FILTER);
                    if(uINV_DCAC_Index_Modulation < 0.0f){
                        uINV_DCAC_Index_Modulation = 0.0f;
                    }
                    else if(uINV_DCAC_Index_Modulation > 1.0f){
                        uINV_DCAC_Index_Modulation = 1.0f;
                    }

                    uINV_ACDC_Duty_Cycle = uINV_DCAC_Index_Modulation;
                    EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, uINV_ACDC_Duty_Cycle*uINV_DCAC_Counter_PWM_Unit);
                }

            }; break;
            case CYCLE_STATE_ZERO_CROSS_POS: {
                cycle_state_counter++; 
                if(cycle_state_counter == 3)   {
                    cycle_state = CYCLE_STATE_NEG;
                    cycle_state_counter = 0;
                    uINV_DCAC_enable_HF();                    
                    GPIO_writePin(uINV_DCAC_Top_LF,1); 
                }
            }; break;
            case CYCLE_STATE_NEG: {
                if(uINV_DCAC_is_event_cross_up(sin_k,sin_k1, -cross_threshold_var)){
                    uINV_DCAC_disable_HF();
                    GPIO_writePin(uINV_DCAC_Top_LF,0);
                    GPIO_writePin(uINV_DCAC_Bottom_LF,0);  
                    cycle_state = CYCLE_STATE_ZERO_CROSS_NEG;
                    cycle_state_counter = 0;
                }
                else{
                    
                    uINV_DCAC_Index_Modulation = uINV_DCAC_compute_control(uINV_DCAC_iAC_Ref)/(uINV_ACDC_vDC_sensed_FILTER);
                    if(uINV_DCAC_Index_Modulation > 0.0f){
                        uINV_DCAC_Index_Modulation = 0.0f;
                    }
                    else if(uINV_DCAC_Index_Modulation < -1.0f){
                        uINV_DCAC_Index_Modulation = -1.0f;
                    }

                    uINV_ACDC_Duty_Cycle = 1.0f + uINV_DCAC_Index_Modulation;
                    
                    EPWM_setCounterCompareValue(DCAC_BASE, EPWM_COUNTER_COMPARE_A, uINV_ACDC_Duty_Cycle*uINV_DCAC_Counter_PWM_Unit);
                }
            }; break;
            case CYCLE_STATE_ZERO_CROSS_NEG: {
                 cycle_state_counter++; 
                if(cycle_state_counter == 3)   {
                    cycle_state = CYCLE_STATE_POS;
                    cycle_state_counter = 0;
                    uINV_DCAC_enable_HF();
                    GPIO_writePin(uINV_DCAC_Bottom_LF,1); 
                }
            }; break;
        }
        debug_array_iac[debug_array_counter] = uINV_ACDC_iAC_sensed;
        // debug_array_vac[debug_array_counter] = uINV_DCAC_vAC_sensed;
        // debug_array_vbus[debug_array_counter] = uINV_ACDC_vDC_sensed_FILTER;
        // debug_array_duty_01[debug_array_counter] = uINV_ACDC_Duty_Cycle;
        // debug_array_duty_11[debug_array_counter] = uINV_DCAC_Index_Modulation;
        // debug_array_state[debug_array_counter] = cycle_state;
        debug_array_counter++;
        if(debug_array_counter == DEBUG_ARRAY_MAX){
            debug_array_counter = 0;
        }
    }



    sin_k1 = sin_k;
    //uINV_DCAC_pll_run(uINV_DCAC_vAC_sensed);
    //GPIO_writePin(uINV_DCAC_TestPoint_2,0);
    
    uINV_DCAC_dVal1 = uINV_DCAC_Index_Modulation;
    uINV_DCAC_dVal2 = uINV_DCAC_vAC_sensed_pu;
    uINV_DCAC_dVal3 = uINV_DCAC_vAC_sensed ;
    uINV_DCAC_dVal4 = uINV_DCAC_Index_Modulation;
    DLOG_4CH_run(&uINV_DCAC_dLog1);
     //

 }


#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_runISR1_lab2) //Run in open loop together with PLL.
 static inline void uINV_DCAC_runISR1_lab2()
 {
     RAMPGEN_run(&uINV_DCAC_rgen);

     uINV_DCAC_Index_Modulation = uINV_DCAC_Amplitude*(__sinpuf32(uINV_DCAC_rgen.out));
     SPLL_1PH_SOGI_run(&uINV_spll_1ph,
                       uINV_DCAC_vAC_sensed);

     if(uINV_DCAC_clearPWMTrip == 1U)
     {
         // Wait for zero-crossing then clear trips and start power stage
         if(__sinpuf32(uINV_DCAC_rgen.out) < 0.05f && __sinpuf32(uINV_DCAC_rgen.out) >= -0.05f)
               {
                     uINV_DCAC_clearPWMTrip = 0;
                     uINV_DCAC_update_Duty_Flag=1;
                     EPWM_clearTripZoneFlag(DCAC_BASE, EPWM_TZ_INTERRUPT |EPWM_TZ_FLAG_OST);
                     uINV_DCAC_Counter_1=0;
               }
     }

     if(uINV_DCAC_update_Duty_Flag == 1U)
     {

             uINV_DCAC_update_Duty();

     }

     uINV_DCAC_dVal1 = uINV_DCAC_Index_Modulation;
     uINV_DCAC_dVal2 = (uINV_DCAC_vAC_sensed/(uINV_ACDC_vDC_sensed+1));
     uINV_DCAC_dVal3 = uINV_ACDC_iAC_sensed ;
     uINV_DCAC_dVal4 = uINV_DCAC_vAC_sensed;
     DLOG_4CH_run(&uINV_DCAC_dLog1);
 }

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_runISR1_lab3) //DC Link voltage control. Resistor connected on the AC terminals.
static inline void uINV_DCAC_runISR1_lab3()
{
     RAMPGEN_run(&uINV_DCAC_rgen);

     SPLL_1PH_SOGI_run(&uINV_spll_1ph,uINV_ACDC_vAC_sensed_FILTER);

     if(uINV_DCAC_clearPWMTrip == 1U)
     {
         uINV_DCAC_clearPWMTrip = 0;
         uINV_DCAC_update_Duty_Flag=1;
         EPWM_clearTripZoneFlag(DCAC_BASE, EPWM_TZ_INTERRUPT |EPWM_TZ_FLAG_OST);
         uINV_DCAC_Counter_1=0;
     }

     if(uINV_DCAC_update_Duty_Flag == 1U)
     {

            uINV_ACDC_vDC_loop_err=uINV_ACDC_vDC_Ref_Slewed-uINV_ACDC_vDC_sensed_NOTCH;
            uINV_DCAC_Index_Modulation_Amp=(DCL_runPI_C2(&gv_ACDC_pi,uINV_ACDC_vDC_loop_err,0));

            uINV_DCAC_Index_Modulation_K=uINV_ACDC_vDC_sensed_NOTCH/(uINV_ACDC_vDC_sensed_FILTER+1);
            uINV_DCAC_Index_Modulation = -1.0f*uINV_DCAC_Index_Modulation_Amp*(__sinpuf32(uINV_DCAC_rgen.out))*uINV_DCAC_Index_Modulation_K;
            uINV_DCAC_update_Duty();
     }

     uINV_DCAC_dVal1 = __sinpuf32(uINV_DCAC_rgen.out);
     uINV_DCAC_dVal2 = uINV_DCAC_vAC_sensed;
     uINV_DCAC_dVal3 = uINV_ACDC_iAC_sensed;
     uINV_DCAC_dVal4 = uINV_ACDC_vDC_sensed;
     DLOG_4CH_run(&uINV_DCAC_dLog1);
}

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_runISR1_lab4) //Run current control loop across external resistor.
static inline void uINV_DCAC_runISR1_lab4()
{

     RAMPGEN_run(&uINV_DCAC_rgen);

     SPLL_1PH_SOGI_run(&uINV_spll_1ph,
                       uINV_DCAC_vAC_sensed_pu);

     //Reference Generation, still to implement the correct saturations.
     uINV_DCAC_iAC_Ref =uINV_ACDC_iAC_real_Ref_Prev*(__sinpuf32(uINV_DCAC_rgen.out))+uINV_ACDC_iAC_imag_Ref*(__cospuf32(uINV_DCAC_rgen.out));

     if(uINV_DCAC_clearPWMTrip == 1U)   // Start the machine!!!!!!!!!!!!!
     {
         // Wait for zero-crossing then clear trips and start power stage
         if(__sinpuf32(uINV_DCAC_rgen.out) < 0.05f && __sinpuf32(uINV_DCAC_rgen.out) >= -0.05f )
               {
                     uINV_DCAC_clearPWMTrip = 0;
                     uINV_DCAC_update_Duty_Flag=1;
                     EPWM_clearTripZoneFlag(DCAC_BASE, EPWM_TZ_INTERRUPT |EPWM_TZ_FLAG_OST);
                     uINV_DCAC_Counter_1=0; // so you can trigger during the start up
               }
     }
     if(uINV_DCAC_update_Duty_Flag == 1U)
     {

            uINV_ACDC_iAC_loop_err=uINV_DCAC_iAC_Ref-uINV_ACDC_iAC_sensed;
            uINV_ACDC_iAC_loop_out_PI= DCL_runPI_C2(&gi_ACDC_pi,uINV_ACDC_iAC_loop_err,0);
            uINV_ACDC_iAC_loop_out_PR1= DCL_runDF22_C1(&gi_r1, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR3= DCL_runDF22_C1(&gi_r3, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR5= DCL_runDF22_C1(&gi_r5, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR7= DCL_runDF22_C1(&gi_r7, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR9= DCL_runDF22_C1(&gi_r9, uINV_ACDC_iAC_loop_err);
            uINV_DCAC_Index_Modulation = (uINV_ACDC_iAC_loop_out_PI+uINV_ACDC_iAC_loop_out_PR1+uINV_ACDC_iAC_loop_out_PR3+uINV_ACDC_iAC_loop_out_PR5+uINV_ACDC_iAC_loop_out_PR7+uINV_ACDC_iAC_loop_out_PR9)/(uINV_ACDC_vDC_sensed_FILTER+5);
            uINV_DCAC_update_Duty();

     }


     //Oscilloscope
     uINV_DCAC_dVal1 = __sinpuf32(uINV_DCAC_rgen.out);
     uINV_DCAC_dVal2 = uINV_DCAC_vAC_sensed ;
     uINV_DCAC_dVal3 = uINV_ACDC_iAC_sensed;
     uINV_DCAC_dVal4 = uINV_DCAC_Index_Modulation;
     DLOG_4CH_run(&uINV_DCAC_dLog1);
     //Oscilloscope
     //
}

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_runISR1_lab5) //Run in close loop together with PLL toward the grid. No DC link voltage control
 static inline void uINV_DCAC_runISR1_lab5()
 {

     SPLL_1PH_SOGI_run(&uINV_spll_1ph,
                       uINV_DCAC_vAC_sensed_pu);

     //Zero crossing of the grid
     if((uINV_ACDC_vAC_sensed_FILTER-uINV_ACDC_vAC_sensed_FILTER_K_1) >= 0.4f)
     {
         uINV_ACDC_vAC_sensed_FILTER_POS=1;
     }
     else
     {
         uINV_ACDC_vAC_sensed_FILTER_POS=0;
     }
     uINV_ACDC_vAC_sensed_FILTER_K_1=uINV_ACDC_vAC_sensed_FILTER;
     //Zero crossing of the grid



     //Reference Generation, still to implement the correct saturations.
     uINV_DCAC_iAC_Ref =uINV_ACDC_iAC_real_Ref_Prev*(uINV_spll_1ph.sine)+uINV_ACDC_iAC_imag_Ref*(uINV_spll_1ph.cosine);

     if(uINV_DCAC_clearPWMTrip == 1U)   // Start the machine!!!!!!!!!!!!!
     {
         // Wait for zero-crossing then clear trips and start power stage
         if(uINV_ACDC_vAC_sensed_FILTER < 2 && uINV_ACDC_vAC_sensed_FILTER >= -2 && uINV_ACDC_vAC_sensed_FILTER_POS==1)
               {
                     uINV_DCAC_POS=1;
                     uINV_DCAC_clearPWMTrip = 0;
                     uINV_DCAC_update_Duty_Flag=1;
                     EPWM_clearTripZoneFlag(DCAC_BASE, EPWM_TZ_INTERRUPT |EPWM_TZ_FLAG_OST);
                     uINV_DCAC_Counter_1=0;
               }
     }
     if(uINV_DCAC_update_Duty_Flag == 1U)
     {

            uINV_ACDC_iAC_loop_err=uINV_DCAC_iAC_Ref-uINV_ACDC_iAC_sensed;
            uINV_ACDC_iAC_loop_out_PI= DCL_runPI_C2(&gi_ACDC_pi,uINV_ACDC_iAC_loop_err,0);
            uINV_ACDC_iAC_loop_out_PR1= DCL_runDF22_C1(&gi_r1, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR3= DCL_runDF22_C1(&gi_r3, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR5= DCL_runDF22_C1(&gi_r5, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR7= DCL_runDF22_C1(&gi_r7, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR9= DCL_runDF22_C1(&gi_r9, uINV_ACDC_iAC_loop_err);
            uINV_DCAC_Index_Modulation = (uINV_ACDC_vAC_sensed_FILTER*uINV_ACDC_FFW_K_Slewed+uINV_ACDC_iAC_loop_out_PI+uINV_ACDC_iAC_loop_out_PR1+uINV_ACDC_iAC_loop_out_PR3+uINV_ACDC_iAC_loop_out_PR5+uINV_ACDC_iAC_loop_out_PR7+uINV_ACDC_iAC_loop_out_PR9)/(uINV_ACDC_vDC_sensed_FILTER+5);
            uINV_DCAC_update_Duty();

            if(uINV_DCAC_Counter_Disable_FFW<20000) // you disable the FFW after 1 s because it is only used during start up
            {
                uINV_DCAC_Counter_Disable_FFW++;
            }
            else
            {
                uINV_ACDC_FFW_K=0;
            }

     }

     //Oscilloscope
     uINV_DCAC_dVal1 = uINV_DCAC_iAC_Ref;
     uINV_DCAC_dVal2 = uINV_DCAC_vAC_sensed ;
     uINV_DCAC_dVal3 = uINV_ACDC_vAC_sensed_FILTER_1H;
     uINV_DCAC_dVal4 = uINV_DCAC_Index_Modulation;
     DLOG_4CH_run(&uINV_DCAC_dLog1);
 }

#pragma FUNC_ALWAYS_INLINE(uINV_DCAC_runISR1_lab6) //Run in close loop together with PLL toward the grid and voltage control loop.
 static inline void uINV_DCAC_runISR1_lab6()
 {

     SPLL_1PH_SOGI_run(&uINV_spll_1ph,
                       uINV_DCAC_vAC_sensed_pu);

     //Zero crossing of the grid
     if((uINV_ACDC_vAC_sensed_FILTER-uINV_ACDC_vAC_sensed_FILTER_K_1) >= 0.4f)
     {
         uINV_ACDC_vAC_sensed_FILTER_POS=1;
     }
     else
     {
         uINV_ACDC_vAC_sensed_FILTER_POS=0;
     }
     uINV_ACDC_vAC_sensed_FILTER_K_1=uINV_ACDC_vAC_sensed_FILTER;

     if(uINV_DCAC_clearPWMTrip == 1U)   // Start the machine!!!!!!!!!!!!!
     {
         // Wait for zero-crossing then clear trips and start power stage
         if(uINV_ACDC_vAC_sensed_FILTER < 2 && uINV_ACDC_vAC_sensed_FILTER >= -2 && uINV_ACDC_vAC_sensed_FILTER_POS==1)
               {
                     uINV_DCAC_POS=1;
                     uINV_DCAC_clearPWMTrip = 0;
                     uINV_DCAC_update_Duty_Flag=1;
                     EPWM_clearTripZoneFlag(DCAC_BASE, EPWM_TZ_INTERRUPT |EPWM_TZ_FLAG_OST);
                     uINV_DCAC_Counter_1=0;
               }
     }
     if(uINV_DCAC_update_Duty_Flag == 1U)
     {

         // Voltage Control Loops
             uINV_ACDC_vDC_loop_err=uINV_ACDC_vDC_Ref_Slewed-uINV_ACDC_vDC_sensed_NOTCH;
             uINV_ACDC_iAC_real_Ref_OUT_PI= -DCL_runPI_C2(&gv_ACDC_pi,uINV_ACDC_vDC_loop_err,0);
             uINV_DCAC_iAC_Ref =uINV_ACDC_iAC_real_Ref_OUT_PI*(uINV_spll_1ph.sine)+uINV_ACDC_iAC_imag_Ref*(uINV_spll_1ph.cosine);
         // Voltage Control Loops
         // Current Control Loop
            uINV_ACDC_iAC_loop_err=uINV_DCAC_iAC_Ref-uINV_ACDC_iAC_sensed;
            uINV_ACDC_iAC_loop_out_PI= DCL_runPI_C2(&gi_ACDC_pi,uINV_ACDC_iAC_loop_err,0);
            uINV_ACDC_iAC_loop_out_PR1= DCL_runDF22_C1(&gi_r1, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR3= DCL_runDF22_C1(&gi_r3, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR5= DCL_runDF22_C1(&gi_r5, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR7= DCL_runDF22_C1(&gi_r7, uINV_ACDC_iAC_loop_err);
            uINV_ACDC_iAC_loop_out_PR9= DCL_runDF22_C1(&gi_r9, uINV_ACDC_iAC_loop_err);
            uINV_DCAC_Index_Modulation = (uINV_ACDC_vAC_sensed_FILTER*uINV_ACDC_FFW_K_Slewed+uINV_ACDC_iAC_loop_out_PI+uINV_ACDC_iAC_loop_out_PR1+uINV_ACDC_iAC_loop_out_PR3+uINV_ACDC_iAC_loop_out_PR5+uINV_ACDC_iAC_loop_out_PR7+uINV_ACDC_iAC_loop_out_PR9)/(uINV_ACDC_vDC_sensed_FILTER+5);
         // Current Control Loop
            uINV_DCAC_update_Duty();

            if(uINV_DCAC_Counter_Disable_FFW<20000)
            {
                uINV_DCAC_Counter_Disable_FFW++;
            }
            else
            {
                uINV_ACDC_FFW_K=0;
                gv_ACDC_pi.Umax = 11;
                gv_ACDC_pi.Umin = -11;
            }
     }
     //Oscilloscope
     uINV_DCAC_dVal1 = uINV_DCAC_iAC_Ref;
     uINV_DCAC_dVal2 = uINV_DCAC_vAC_sensed ;
     uINV_DCAC_dVal3 = uINV_ACDC_vAC_sensed_FILTER_1H;
     uINV_DCAC_dVal4 = uINV_DCAC_Index_Modulation;
     DLOG_4CH_run(&uINV_DCAC_dLog1);
 }

#endif /* UINV_DCAC_H_ */
