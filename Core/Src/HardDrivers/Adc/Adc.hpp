/**********************************************************************************/
/**
*  @file
*  Adc.hpp - declaration of class TAdc.
*
*/

/**********************************************************************************/
#ifndef __Adc_H
#define __Adc_H

/**********************************************************************************/
//#include "main.h"
//#include "stm32l0xx_hal_def.h"
//#include "stm32l0xx_hal_dma.h"
//#include "stm32l0xx_hal_adc.h"
#include "..\..\System\Sys.h"

/**********************************************************************************/
#define ADC_NUMBER_OF_CHANNEL  7

#define ADC_VIN_CHANNEL   0
#define ADC_T3_CHANNEL    1
#define ADC_MOT_CHANNEL   2
#define ADC_T1_CHANNEL    3
#define ADC_T2_CHANNEL    4
#define ADC_VREF_CHANNEL  5
#define ADC_TP_CHANNEL    6

/*
Address of the calibrate temperature sensors:
0x1FF8 007A - 0x1FF8 007B - TS_CAL1 - 130 C
0x1FF8 007E - 0x1FF8 007F - TS_CAL2 - 30 C
 */
#define ADC_ADDR_T30_CAL   0x1FF8007A
#define ADC_ADDR_T130_CAL  0x1FF8007E

/*
Address of the calibrate Vref:
0x1FF8 0078 - 0x1FF8 0079
*/
#define ADC_ADDR_VREF_CAL     0x1FF80078
#define ADC_VDDA_3V_CAL       3.0
#define ADC_FULL_SCALE_12BIT  4095
#define ADC_FULL_SCALE_8BIT   255


/**********************************************************************************/
//==================================================================================
class TAdc
{
public:
    ////// variables //////
    

    ////// constants //////


    ////// functions //////
    void Init(void);
    void Start(u16*);
    void Stop(void);
    void StopDma(void);
    void Calibrate(void);
    float ReadTP(u16*);
    float ReadVref(u16*);
    float ReadVdda(u16*);
    float ReadVin(u16*);
    float ReadT1(u16*);
    float ReadT3(u16*);
    float ReadT2(u16*);
    
  
protected:
	////// variables //////
	

	////// constants //////


	////// functions //////

  
private:
	////// variables //////


	////// constants //////


	////// functions //////
    float ReadChannel(u16*, u16);
    void TSensorOn(void);
    void TSensorOff(void);



};
//=== end class TAdc ===============================================================

/**********************************************************************************/

#endif
