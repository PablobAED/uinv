 //##############################################################################
//
// FILE:  uinv_comms.c
//
// TITLE:  Solution file for communications
//
//##############################################################################
// $TI Release: TIDA_010933 v2.00.00.00 $
// $Release Date: Wed Aug  7 17:16:55 CDT 2024 $
// $Copyright:
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
// ALL RIGHTS RESERVED
// $
//##############################################################################

#include "driverlib.h"
#include "device.h"
#include "board.h"

#include "uinv_comms.h"
#include "inc/hw_types.h"
#include "uinv_dcac.h"
#include "uinv_dcdc.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

uinv_solar_uinverter_flag_t uINV_comms_flag = (uinv_solar_uinverter_flag_t)0;
uinv_solar_uinverter_t uINV_comms_data;

//Structure to string
size_t length_string = 0;

// Inicializa la estructura data con los valores recibidos por UART aqu�

char output_string[512];  // Tama�o suficiente para contener el string de salida
char message[512];

//
// Variables maquina estados convertidor
//
uint16_t ConverterState = STATE_UNKNOW;
uint16_t startConverting; // variable que simula el mensaje recibido del PLC para comenzar la conversión
uint16_t stopConverting; // variable que simulan l mensaje rcibido del PLC para parar la conversión
uint16_t resetConverter; // variable que simulan l mensaje rcibido del PLC para parar resetear la conversión

uint16_t p1; // Estado de las panles 1 ON, 2 OFF
uint16_t p2;
uint16_t p3;
uint16_t p4;

uint16_t p1_on = 0; // Variable para detectar si hay que encender un boost
uint16_t p2_on = 0;
uint16_t p3_on = 0;
uint16_t p4_on = 0;

uint16_t p1_off = 0; // Variable para detectar si hay que apagar un boost
uint16_t p2_off = 0;
uint16_t p3_off = 0;
uint16_t p4_off = 0;

uint16_t start_chain; // Variable para conocer si se arranca toda la cadena o solo los boost

float32_t umbral1 = 0.2261; // Umbral para que considere DC/DC encendido
float32_t umbral2 = 0.2261; // Umbral para que considere DC/DC encendido
float32_t umbral3 = 0.2261; // Umbral para que considere DC/DC encendido
float32_t umbral4 = 0.2261; // Umbral para que considere DC/DC encendido

void solar_uinverter_to_string(uinv_solar_uinverter_t *data, char *output) {
    // Multiplica cada float por 100 para conservar 2 decimales y convierte a entero
    sprintf(output,
        //"\r\nPanel ID: %u\r\n"
        "\r\nPV Panel Voltages: [%2d, %2d, %2d, %2d]\r\n"
        "PV Panel Currents: [%2d.%01d, %2d.%01d, %2d.%01d, %2d.%01d]\r\n"
        "PV Panel Power: [%3d, %3d, %3d, %3d]\r\n"
        /*"MPPT LLC Input Voltage: %d.%02d\r\n"
        "MPPT LLC Input Current: %d.%02d\r\n"
        "MPPT LLC Input Power: %d.%02d\r\n"
        "LLC Output Voltage: %d.%02d\r\n"
        "LLC Output Current: %d.%02d\r\n"
        "LLC Output Power: %d.%02d\r\n"
        "Inverter Grid Output Voltage: %d.%02d\r\n"*/
        "Inverter Grid Output Current: %2d.%01d\r\n"
        "Inverter Grid Output Power: %4d\r\n"
        /*"Inverter Grid Frequency: %d.%02d\r\n"
        "Fault Flags: %u\r\n"*/,

        //data->panel_id,

        (int)(data->pv_panel_v[0]+0.5),
        (int)(data->pv_panel_v[1]+0.5),
        (int)(data->pv_panel_v[2]+0.5),
        (int)(data->pv_panel_v[3]+0.5),

        (int)(data->pv_panel_a[0]), (int)(data->pv_panel_a[0] * 10) % 10,
        (int)(data->pv_panel_a[1]), (int)(data->pv_panel_a[1] * 10) % 10,
        (int)(data->pv_panel_a[2]), (int)(data->pv_panel_a[2] * 10) % 10,
        (int)(data->pv_panel_a[3]), (int)(data->pv_panel_a[3] * 10) % 10,

        (int)(data->pv_panel_w[0]+0.5),
        (int)(data->pv_panel_w[1]+0.5),
        (int)(data->pv_panel_w[2]+0.5),
        (int)(data->pv_panel_w[3]+0.5),

        /*(int)(data->mppt_llc_in_v * 100), (int)(data->mppt_llc_in_v * 100) % 100,
        (int)(data->mppt_llc_in_a * 100), (int)(data->mppt_llc_in_a * 100) % 100,
        (int)(data->mppt_llc_in_w * 100), (int)(data->mppt_llc_in_w * 100) % 100,

        (int)(data->llc_out_v * 100), (int)(data->llc_out_v * 100) % 100,
        (int)(data->llc_out_a * 100), (int)(data->llc_out_a * 100) % 100,
        (int)(data->llc_out_w * 100), (int)(data->llc_out_w * 100) % 100,

        (int)(data->inverter_grid_out_v * 100), (int)(data->inverter_grid_out_v * 100) % 100,*/
        (int)(data->inverter_grid_out_a), (int)(data->inverter_grid_out_a * 10) % 10,
        (int)(data->inverter_grid_out_w+0.5)

        /*(int)(data->inverter_grid_freq * 100), (int)(data->inverter_grid_freq * 100) % 100,

        data->fault*/
    );
}

void uINV_comms_init() {
    SCI_resetChannels(SCI_COMMS_BASE);
    SCI_resetRxFIFO(SCI_COMMS_BASE);
    SCI_resetTxFIFO(SCI_COMMS_BASE);
    SCI_clearInterruptStatus(SCI_COMMS_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableFIFO(SCI_COMMS_BASE);
    SCI_enableModule(SCI_COMMS_BASE);

    uINV_comms_data.panel_id = 0xC001501A;
}

void uINV_comms_test() {
    static uint32_t comms_counter = 0;

#if UINV_DCAC_COMMS_ENABLED == 1
    //
    // Check if any error in RX
    //
    uint16_t rxStatus = SCI_getRxStatus(SCI_COMMS_BASE);
    if((rxStatus & SCI_RXSTATUS_ERROR) != 0)
    {
        //
        //If Execution stops here there is some error
        //Analyze SCI_getRxStatus() API return value
        //
        uINV_DCDC_fail();
        EPWM_forceTripZoneEvent(DCAC_BASE, EPWM_TZ_FORCE_EVENT_OST);

        ESTOP0;
    }

    //
    // Check if data received
    //
    uint16_t fifoStatus = SCI_getRxFIFOStatus(SCI_COMMS_BASE);
    if(fifoStatus != SCI_FIFO_RX0) {
        uint16_t data  = (uint16_t)(HWREGH(SCI_COMMS_BASE + SCI_O_RXBUF) & SCI_RXBUF_SAR_M);
        uINV_comms_flag = (uinv_solar_uinverter_flag_t)data;
    }
    if((uINV_comms_flag & SOLAR_UINVERTER_FLAG_RSD_ENABLED) != 0) {
        //
        // Check if shutdown requested
        //
        uINV_DCDC_fail();
        EPWM_forceTripZoneEvent(DCAC_BASE, EPWM_TZ_FORCE_EVENT_OST);
    } else {
        if(comms_counter++ == UINV_DCAC_COMMS_COUNTER) {
            uINV_comms_update_data();
            uINV_comms_send();
            comms_counter = 0;
        }
    }
#endif
}

void uINV_comms_send() {
    //
    // Send data to SCI
    //
    //uINV_comms_data.panel_id = sizeof(uINV_comms_data)-1;
    solar_uinverter_to_string(&uINV_comms_data, output_string);
    // Asignar la longitud al primer byte
    length_string = strlen(output_string);
    message[0] = (uint8_t)length_string;
    // Copiar el texto al nuevo arreglo a partir del segundo byte
    memcpy(&message[1], output_string, length_string);
    SCI_writeCharArray(SCI_COMMS_BASE, (const char *)&message, length_string+1);

    // Asignar la longitud al primer byte
    //sDataA[0] = (uint8_t)length;

    // Copiar el texto al nuevo arreglo a partir del segundo byte
    //memcpy(&sDataA[1], msg, length);

    //SCI_writeCharArray(SCI_COMMS_BASE, sDataA, length + 1);
}

void uINV_comms_update_data() {

    uINV_comms_data.fault = (!uINV_DCDC_started);

    uINV_comms_data.pv_panel_v[0] = uINV_DCDC_b1.voltage_pu*uINV_DCDC_VS_SCALE*ANA_REF;
    uINV_comms_data.pv_panel_a[0] = uINV_DCDC_b1.current_pu*20.0f;
    uINV_comms_data.pv_panel_w[0] = uINV_comms_data.pv_panel_v[0] * uINV_comms_data.pv_panel_a[0];

    /*uINV_comms_data.pv_panel_v[0] = 30.48;
    uINV_comms_data.pv_panel_a[0] = 13.42;
    uINV_comms_data.pv_panel_w[0] = 628;*/

    uINV_comms_data.pv_panel_v[1] = uINV_DCDC_b2.voltage_pu*uINV_DCDC_VS_SCALE*ANA_REF;
    uINV_comms_data.pv_panel_a[1] = uINV_DCDC_b2.current_pu*20.0f;
    uINV_comms_data.pv_panel_w[1] = uINV_comms_data.pv_panel_v[1] * uINV_comms_data.pv_panel_a[1];

    uINV_comms_data.pv_panel_v[2] = uINV_DCDC_b3.voltage_pu*uINV_DCDC_VS_SCALE*ANA_REF;
    uINV_comms_data.pv_panel_a[2] = uINV_DCDC_b3.current_pu*20.0f;
    uINV_comms_data.pv_panel_w[2] = uINV_comms_data.pv_panel_v[2] * uINV_comms_data.pv_panel_a[2];

    uINV_comms_data.pv_panel_v[3] = uINV_DCDC_b4.voltage_pu*uINV_DCDC_VS_SCALE*ANA_REF;
    uINV_comms_data.pv_panel_a[3] = uINV_DCDC_b4.current_pu*20.0f;
    uINV_comms_data.pv_panel_w[3] = uINV_comms_data.pv_panel_v[3] * uINV_comms_data.pv_panel_a[3];

    uINV_comms_data.mppt_llc_in_v = uINV_DCDC_globals.vs_pu*uINV_DCDC_VS_SCALE*ANA_REF;
    uINV_comms_data.mppt_llc_in_w = uINV_comms_data.pv_panel_w[0] + uINV_comms_data.pv_panel_w[1] + uINV_comms_data.pv_panel_w[2] + uINV_comms_data.pv_panel_w[3];

    uINV_comms_data.llc_out_v = uINV_ACDC_vDC_sensed_NOTCH;
    uINV_comms_data.llc_out_a = uINV_ACDC_iDC_sensed;

    uINV_comms_data.inverter_grid_out_v = uINV_ACDC_vAC_sensed_FILTER;

    /*uINV_comms_data.inverter_grid_out_a = 10.56;
    uINV_comms_data.inverter_grid_out_w = 1559;*/

    uINV_comms_data.inverter_grid_out_a = guiIrms;
    uINV_comms_data.inverter_grid_out_w = guiIrms*230;

}

void uINV_controller_PLC(){
    
    static uint16_t prevConverterState = STATE_UNKNOW; // Estado anterior
    static uint16_t nextConverterState = STATE_INIT; //Estado siguiente

    switch(ConverterState){
        case(STATE_INIT):
            nextConverterState = uINV_SystemInit(prevConverterState);
        break;

        case(STATE_WAIT_PANELS):
            nextConverterState = uINV_WaitPanels(prevConverterState);
        break;

        case(STATE_START_CONVERTERS):
            nextConverterState = uINV_StartConverters(prevConverterState);
        break;

        case(STATE_CHECK):
            nextConverterState = uINV_Check(prevConverterState);
        break;

        case(STATE_STOP_BOOST):
            nextConverterState = uINV_StopBoost(prevConverterState);
        break;

        case(STATE_STOP):
            nextConverterState = uINV_StopConversion(prevConverterState);
        break;

        case(STATE_F):
            nextConverterState = uINV_ErrorState(prevConverterState);
        break;

        default:
        nextConverterState = STATE_INIT;

    }

    prevConverterState = ConverterState;
    ConverterState = nextConverterState;


}

uint16_t uINV_SystemInit (uint16_t prevConverterState){

    uint16_t nextConverterState = STATE_WAIT_PANELS; // Muchas de las inicializaciones se realizan desde el main. 

    startConverting = 0;
    stopConverting = 0;
    resetConverter = 0;

    uINV_ACDC_iAC_limit = 1152 ; // Proteccion de sobre-corriente (128*9A)
    uINV_ACDC_vDC_Ref = 340 ; // Referencia inicial de VBUS (350V)
    //pr_kp = 4; // Constante proporcional inicial del controlador PR
    gv_ACDC_pi.Umax = 2; // Output máxima del lazo de tensión
    gv_ACDC_pi.Umin = -8; // Output mínima del lazo de tensión

    /*uINV_DCDC_b1.current_ref_pu = 0.025; // Referencia de corriente en los DC/DC
    uINV_DCDC_b2.current_ref_pu = 0.025;
    uINV_DCDC_b3.current_ref_pu = 0.025;
    uINV_DCDC_b4.current_ref_pu = 0.025;*/

    uINV_DCDC_b1.en = 0; // aseguro que esté todo apagado
    uINV_DCDC_b2.en = 0;
    uINV_DCDC_b3.en = 0;
    uINV_DCDC_b4.en = 0;
    //uINV_ACDC_iAC_bool = 1;

    start_chain = 1; // Arranque completo

    p1 = 0; // Inicialización del estado de los paneles;
    p2 = 0;
    p3 = 0;
    p4 = 0;

    p1_on = 0;
    p2_on = 0;
    p3_on = 0;
    p4_on = 0;

    p1_off = 0;
    p1_off = 0;
    p1_off = 0;
    p1_off = 0;
    
    GPIO_writePin(uINV_DCAC_Relay, 0); // Aseguro que el rele está a 0 en el inicio
 
    return(nextConverterState);
}

uint16_t uINV_WaitPanels (uint16_t prevConverterState){

    uint16_t nextConverterState = STATE_WAIT_PANELS; // De momento no es necesario que este estado haga nada ya que se inicializa todo desde el main
    static uint16_t voltage_on = 0;    

    checkPanels_ON_OFF();
    
    // Condicion de arranque
    voltage_on =   (p1_on || p2_on || p3_on || p4_on)  && guiVrms > 200; // ¿Hay tensión en algún panel y en la red?

    if(voltage_on){
        startConverting = 1; // Si hay tensión, arranca el convertidor
    }

    if (startConverting == 1){
        nextConverterState = STATE_START_CONVERTERS;
        voltage_on = 0; 
        start_chain = 1; // Indico que arranque todos los convertidores
    }
            
    return(nextConverterState);
}

uint16_t uINV_StartConverters (uint16_t prevConverterState){
    uint16_t nextConverterState = STATE_START_CONVERTERS;
    static uint16_t counter = 0;
    static uint16_t state_counter = 0;  
    
    // Contador de estado
    if (counter == 8000){ // Cada segundo, una acción       
        if( state_counter == 0 ){
            if(start_chain){
                uINV_DCAC_Relay_Enable = 1; // Abrir relé
                state_counter = 1; // Siguiente estado 
            }else{
                state_counter = 5; // Siguiente estado 
            }
        }else if( state_counter == 1 ){
            if (uINV_ACDC_vDC_Ref_Slewed > 339.0f && uINV_ACDC_vDC_Ref_Slewed < 345.0f){ 
                uINV_DCAC_clearPWMTrip = 1; // Si la referencia esta en 350, habilitar el convertidor
                state_counter ++; // Siguiente estado
            }
        }else if( state_counter == 2 ){
            if(uINV_ACDC_vDC_sensed > 335.0f){
                uINV_ACDC_vDC_Ref = 380.0f; // Si la tensión de BUS llega a 350V, subirla a 380V
                state_counter ++; // Siguiente estado
            }
        }else if ( state_counter == 3 ){
            if(uINV_ACDC_vDC_sensed > 375.0f){
                //pr_kp = 3; // Si la tensión llega a 380V, bajar kp a 3
                state_counter ++;
            }
        }else if ( state_counter == 4 ){
                uINV_DCDC_clearPWMTrip = 1; // Limpiar el trip del DC/DC (ACTIVA LLC?)
                start_chain = 0;
                state_counter ++;
        }else if(state_counter == 5){
            if(uINV_DCDC_globals.vs_real > 68.0f){
                state_counter ++;
            }
        }else if( state_counter == 6){ // Activa DC/DCs donde detecta tensión
            if(p1_on == 1){
                uINV_DCDC_b1.current_ref_pu = 0.025f;
                uINV_DCDC_b1.en = 1; //activa el DC/DC 1
                p1 = 1;
                p1_on = 0;
            } 
            state_counter ++;
        }else if( state_counter == 7){ 
            if(p2_on == 1){
                uINV_DCDC_b2.current_ref_pu = 0.025f;
                uINV_DCDC_b2.en = 1; //activa el DC/DC 2
                p2 = 1;
                p2_on = 0;
            }
            state_counter ++;
 
        }else if( state_counter == 8){ 
            if(p3_on == 1){
                uINV_DCDC_b3.current_ref_pu = 0.025f;
                uINV_DCDC_b3.en = 1; //activa el DC/DC 3
                p3 = 1;
                p3_on = 0;
            }  
            state_counter ++;
 
        }else if( state_counter == 9){ 
            if(p4_on == 1){
                uINV_DCDC_b4.current_ref_pu = 0.025f;
                uINV_DCDC_b4.en = 1; //activa el DC/DC 4
                p4 = 1;
                p4_on = 0;
            } 
        }
        counter = 0;           
    }else{
        counter ++;
    }    

    if(!(p1_on || p2_on || p3_on || p4_on)){ // Si ya estan todos encendidos
        nextConverterState = STATE_CHECK;
        state_counter = 0;
    }
    
    return(nextConverterState);
}

uint16_t uINV_Check(uint16_t prevConverterState){
    uint16_t nextConverterState = STATE_CHECK;
    static uint16_t counter = 0;
    static uint16_t stop_BOOST = 0;
    static uint16_t start_BOOST = 0;

    
    if (counter == 24000){
        if (uINV_DCDC_b1.current_ref_pu == 0.025f && p1 == 1){ // Cambio la referencia a 5A
            uINV_DCDC_b1.current_ref_pu = 0.25f;
        }else if (uINV_DCDC_b2.current_ref_pu == 0.025f && p2 == 1){
            uINV_DCDC_b2.current_ref_pu = 0.25f;
        }else if (uINV_DCDC_b3.current_ref_pu == 0.025f && p3 == 1){
            uINV_DCDC_b3.current_ref_pu = 0.25f;
        }else if (uINV_DCDC_b4.current_ref_pu == 0.025f && p4 == 1){
            uINV_DCDC_b4.current_ref_pu = 0.25f;
        }
        counter = 0;     
    }else{
        counter++;
    }
    
    checkPanels_ON_OFF();
    if(p1_off || p2_off || p3_off || p4_off){
        stop_BOOST = 1;
        }

     if(p1_on || p2_on || p3_on || p4_on){
        start_BOOST = 1;
        }

    if(start_BOOST == 1){
        nextConverterState = STATE_START_CONVERTERS;
        counter = 0;
        stop_BOOST = 0;
        start_BOOST = 0;
    }else if(stop_BOOST){
        nextConverterState = STATE_STOP_BOOST;
        counter = 0;
        stop_BOOST = 0;
        start_BOOST = 0;
    }

    if (uINV_ACDC_iAC_overcurrent || uINV_ACDC_fault ){
        nextConverterState = STATE_F;
    }

    return(nextConverterState);
}

uint16_t uINV_StopBoost(uint16_t prevConverterState){
    uint16_t nextConverterState = STATE_STOP_BOOST;
    static uint16_t counter = 0;
    
    if (counter == 8000){
        if (p1_off){
            uINV_DCDC_b1.en = 0; // Para el Boost 1
            uINV_DCDC_b1_pi_current.i10 = 0;
            uINV_DCDC_b1.current_ref_pu = 0.025f;
            p1_off = 0;
            p1 = 0;
        }else if (p2_off){   
            uINV_DCDC_b2.en = 0;// Para el Boost 2
            uINV_DCDC_b2_pi_current.i10 = 0; 
            uINV_DCDC_b2.current_ref_pu = 0.025f;
            p2_off = 0;
            p2 = 0;
        }else if (p3_off){ 
            uINV_DCDC_b3.en = 0;// Para el Boost 3
            uINV_DCDC_b3_pi_current.i10 = 0;
            uINV_DCDC_b3.current_ref_pu = 0.025f;
            p3_off = 0;
            p3 = 0;
        }else if (p4_off){
            uINV_DCDC_b4.en = 0;// Para el Boost 4
            uINV_DCDC_b4_pi_current.i10 = 0;
            uINV_DCDC_b4.current_ref_pu = 0.025f;
            p4_off = 0;
            p4 = 0;
        }       
        counter = 0;
    }else{
        counter ++;
    }

    if(p1_off||p2_off||p3_off||p4_off){ // Queda todavia alguno por apagar
        nextConverterState = STATE_STOP_BOOST; // Quedate en este estado
    }else if(!(p1||p2||p3||p4)){// Se han apagado todos los DC/DC
        nextConverterState =  STATE_STOP; // Apaga todo
        counter = 0;
    }else{ // Todavia queda alguno encendido
        nextConverterState =  STATE_CHECK; // Pasa al estado check para comprobar si hay que encender o apagar alguno
        counter = 0;
    }

    return(nextConverterState);
}

uint16_t uINV_StopConversion(uint16_t prevConverterState){
    uint16_t nextConverterState = STATE_STOP;
    static uint16_t counter = 0;
    static uint16_t state_counter_off = 0;
    
    if (counter == 8000 ){
        if ( state_counter_off == 0){
            uINV_DCDC_started = 0; // Apaga el LLC
            state_counter_off = 1;    
        }else if (state_counter_off == 1){ 
            uINV_ACDC_iAC_bool = 1; // Apaga el AC/DC
            state_counter_off = 2;  
        }else if (state_counter_off == 2){
            GPIO_writePin(uINV_DCAC_Relay, 0);
            if(resetConverter == 1){
                nextConverterState = STATE_INIT;
                state_counter_off = 0;
                counter = 0;
            }
        }       
        counter = 0;
    }else{
        counter ++;
    }
    return(nextConverterState);
}

uint16_t uINV_ErrorState(uint16_t prevConverterState){
    uint16_t nextConverterState = STATE_F;

    if(resetConverter == 1){
        nextConverterState = STATE_INIT;
    }

    return(nextConverterState);
}

void checkPanels_ON_OFF(){
    if (p1){
        if(uINV_DCDC_b1.voltage_pu < umbral1){
            p1_off = 1;
        }else{
            p1_off = 0;
        }
    }else{
        if(uINV_DCDC_b1.voltage_pu > umbral1){
            p1_on = 1;
        }else{
            p1_on = 0;
        }
    
    }

    if (p2){
        if(uINV_DCDC_b2.voltage_pu < umbral2){
            p2_off = 1;
        }else{
            p2_off = 0;
        }
    }else{
        if(uINV_DCDC_b2.voltage_pu > umbral2){
            p2_on = 1;
        }else{
            p2_on = 0;
        }
    
    }

    if (p3){
        if(uINV_DCDC_b3.voltage_pu < umbral3){
            p3_off = 1;
        }else{
            p3_off = 0;
        }
    }else{
        if(uINV_DCDC_b3.voltage_pu > umbral3){
            p3_on = 1;
        }else{
            p3_on = 0;
        }
    
    }

    if (p4){
        if(uINV_DCDC_b4.voltage_pu < umbral4){
            p4_off = 1;
        }else{
            p4_off = 0;
        }
    }else{
        if(uINV_DCDC_b4.voltage_pu > umbral4){
            p4_on = 1;
        }else{
            p4_on = 0;
        }
    
    }

}
