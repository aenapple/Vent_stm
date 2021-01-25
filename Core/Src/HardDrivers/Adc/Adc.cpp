/**********************************************************************************/
/**
*  @file
*  Todo: Adc.cpp.
*
*/

/**********************************************************************************/
#include "Adc.hpp"


/**********************************************************************************/
extern ADC_HandleTypeDef hadc;


/**********************************************************************************/
//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
void TAdc::Init(void)
{
    HAL_GPIO_WritePin(VIN_CONTROL_GPIO_Port, VIN_CONTROL_Pin, GPIO_PIN_RESET);
    this->TSensorOn();
    this->Calibrate();
}
//=== end Init =====================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
void TAdc::Start(u16* bufferData)
{
    //for(u16 i = 0; i < ADC_NUMBER_OF_CHANNEL; i++) bufferData[i] = 0;
    HAL_ADC_Start_DMA(&hadc, (u32*)bufferData, ADC_NUMBER_OF_CHANNEL);
}
//=== end Start ====================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
void TAdc::Stop(void)
{
    HAL_ADC_Stop(&hadc);
}
//=== end Stop =====================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return ... .
*/
void TAdc::StopDma(void)
{
    HAL_ADC_Stop_DMA(&hadc);
}
//=== end StopDma ==================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
void TAdc::Calibrate(void)
{
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
}
//=== end Calibrate ================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
float TAdc::ReadTP(u16* valueAdc)
{
    float result;
    u16 tsCal1;
    u16 tsCal2;
    float tsData;


    tsCal1 = *((u16*)ADC_ADDR_T30_CAL);
    tsCal2 = *((u16*)ADC_ADDR_T130_CAL);
    tsData = this->ReadVdda(valueAdc) / ADC_VDDA_3V_CAL * valueAdc[ADC_TP_CHANNEL];
    result = 100.0 / (tsCal2 - tsCal1) * (tsData - tsCal1) + 30;


    return (result);
}
//=== end ReadTP ===================================================================
//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
float TAdc::ReadVref(u16* valueAdc)
{
	return (this->ReadChannel(valueAdc, ADC_VREF_CHANNEL));
}
//=== end ReadVref =================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
float TAdc::ReadVdda(u16* valueAdc)
{
	return (ADC_VDDA_3V_CAL * (*((u16*)ADC_ADDR_VREF_CAL)) / valueAdc[ADC_VREF_CHANNEL]);
}
//=== end ReadVdda =================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
float TAdc::ReadVin(u16* valueAdc)
{
    return (this->ReadChannel(valueAdc, ADC_VIN_CHANNEL) * 2.06);
}
//=== end ReadVin ==================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
float TAdc::ReadT1(u16* valueAdc)
{
    return (this->ReadChannel(valueAdc, ADC_T1_CHANNEL) /* 23*/);
}
//=== end ReadT1 ===================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
float TAdc::ReadT2(u16* valueAdc)
{
    return (this->ReadChannel(valueAdc, ADC_T2_CHANNEL) /* 23*/);  // 22.25476
}
//=== end ReadT2 ===================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
float TAdc::ReadT3(u16* valueAdc)
{
    return (this->ReadChannel(valueAdc, ADC_T3_CHANNEL) /* 23*/);
}
//=== end ReadT3 ===================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
float TAdc::ReadChannel(u16* valueAdc, u16 channelAdc)
{
    //float result;


    //result = ADC_VDDA_3V_CAL * (u16)(*((u16*)ADC_ADDR_VREF_CAL)) * valueAdc[channelAdc];
    //result = result / ADC_FULL_SCALE_12BIT / valueAdc[ADC_VREF_CHANNEL];
    //result = this->ReadVdda(valueAdc) * valueAdc[channelAdc] / ADC_FULL_SCALE_12BIT;





    return (this->ReadVdda(valueAdc) * valueAdc[channelAdc] / ADC_FULL_SCALE_12BIT);
}
//=== end ReadChannel ==============================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
void TAdc::TSensorOn(void)
{
    HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_SET);
}
//=== end TSensorOn ================================================================

//==================================================================================
/**
*  Todo: function description.
*
*  @return void .
*/
void TAdc::TSensorOff(void)
{
    HAL_GPIO_WritePin(CTRL_GPIO_Port, CTRL_Pin, GPIO_PIN_RESET);
}
//=== end TSensorOff ===============================================================

/**********************************************************************************/
