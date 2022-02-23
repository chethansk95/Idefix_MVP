/****************************************************************************/
/** 
* SenseGuard gmbh
* @file pressure.c
* @{
* @details
* 
* This file contains the \e driver function for the <b> NAU7802 </b>().
* 
* <b> Major Tasks </b> of this file includes
*
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who    Date        Changes
* ----- ------ --------    ---------------------------------------------------------------------
* 1.0   sk    10-02-2022   First version
*
* </pre>
*****************************************************************************/
/** @} */


#include "main.h"
#include "pressure.h"

static const char *TAG1 = "initialization";


/*****************************************************************************/
/**
* \brief NAU7802 Initialization (reffered nau7802 datasheet for sequence and regester configuration)
*
* \description
*  The initialization of the ADC follows the sequence given below
*   1.  aplly reset to the ADC (all the regester reset to there default values)
*   2.  Issue power up command
*   3.  Issue switch ON the internal LDO option (so that the sensor connected can get the power supply from the ADC)
*   4.  set gain 
*   5.  set sample rate
*   6.  performs calibration of internal voltage offset
*
* @param      None
*  
* @return     TOTALIZER_ERROR_STATUS
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
TOTALIZER_ERROR_STATUS  nau7802_Initialzation(void)
{
    uint8_t clk_chp_off = 0x30,power_ctrl_read=0,power_ctrl_write = 0, i2c_control_write = 0, i2c_control_read =0;
    TOTALIZER_ERROR_STATUS result = PASS;
    reset();
    result &= powerup();
    result &= setLDO(NAU7802_LDO_3V3);
    result &= setGain(NAU7802_GAIN_128);
    setSampleRate(NAU7802_SPS_80);
    nau7802_register_write(NAU7802_ADC, &clk_chp_off, 1);
    nau7802_register_read(NAU7802_PGA_PWR,&power_ctrl_read,1);
    power_ctrl_write = power_ctrl_read | (SET_BIT_MASK<< NAU7802_PGA_PWR_PGA_CAP_EN);
    nau7802_register_write(NAU7802_PGA_PWR,&power_ctrl_write,1);
    nau7802_register_read(NAU7802_PGA_PWR,&power_ctrl_read,1);

   result &= calirate();
   
    if(power_ctrl_write == power_ctrl_read)
    {
        result &=  PASS;
    }   
    else
    {
        result &=  ERROR;    
    }
   
    return result;
}

/*****************************************************************************/
/**
* \brief Reset NAU7802
*
* \description
*   reset register is used to perform the reset of NAU7802, all the internal register 
    are reset to there default values.
*
* @param      None 
*  
* @return     None 
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/

void reset (void)
{
    uint8_t reset_data = SET_BIT_MASK << NAU7802_PU_CTRL_RR,clear_data = 0;
    nau7802_register_write(NAU7802_PU_CTRL, &reset_data ,1);
    for(int i=0; i<50; i++);
    nau7802_register_read(NAU7802_PU_CTRL,&clear_data,1);
    clear_data = clear_data & 0xFE ; 
    nau7802_register_write(NAU7802_PU_CTRL, &clear_data ,1);
}

/*****************************************************************************/
/**
* \brief ADC start conversion
*
* \description
*   This function sets the ADC start conversion bit in the NAU7802_PU_CTRL register 
*
* @param      None
*  
* @return     None
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/

void adc_start_conversion(void)
{
    uint8_t puctrl_write =0, puctrl_read = 0;
    nau7802_register_read(NAU7802_PU_CTRL,&puctrl_read,1);
    puctrl_write = puctrl_read | 0b00010000;
    nau7802_register_write(NAU7802_PU_CTRL, &puctrl_write ,1);
}

/*****************************************************************************/
/**
* \brief power up option for the NAU7802
*
* \description
*   This function power up the digital and analog circuit of NAU7802
*   The function checks for the power up ready bit (read only status)
*   Return PASS if the bit is set.
*
* @param      None
*  
* @return     TOTALIZER_ERROR_STATUS
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/

TOTALIZER_ERROR_STATUS powerup(void)
{
    uint8_t power_reg_read = 0,power_reg_write= 0,counter = 0;
    nau7802_register_read(NAU7802_PU_CTRL,&power_reg_read,1);
    power_reg_write = power_reg_read | ((SET_BIT_MASK << NAU7802_PU_CTRL_PUD)|(SET_BIT_MASK << NAU7802_PU_CTRL_PUA)|(SET_BIT_MASK<<NAU7802_PU_CTRL_AVDDS));
    nau7802_register_write(NAU7802_PU_CTRL, &power_reg_write ,1);
    while (1)
    {
    nau7802_register_read(NAU7802_PU_CTRL,&power_reg_read,1);
    power_reg_read = (power_reg_read & 0x08) >> NAU7802_PU_CTRL_PUR ;
    if ( power_reg_read== 0x01)
      break; //Good to go
    if (counter++ > 100)
      return (ERROR); //Error
    }
     return (PASS);
}

/*****************************************************************************/
/**
* \brief calibration
*
* \description
*   This function sets the start calibration bit in the control register 2 and 
    defualt offset calibration internal is set and once the calibration is completed 
    the calibration bit is automatically set to 0 by the ADC.
*
* @param      symbol and state
*  
* @return     TOTALIZER_ERROR_STATUS
*
* @author     SK 
*
* @date       10/02/2021
*
* @note
********************************************************************************/

TOTALIZER_ERROR_STATUS calirate(void)
{
    uint8_t ctrl2_reg_read = 0, ctrl2_reg_write = 0;
    nau7802_register_read(NAU7802_CTRL2,&ctrl2_reg_read,1);
    ctrl2_reg_write = ctrl2_reg_read | 0b00000100;
    nau7802_register_write(NAU7802_CTRL2, &ctrl2_reg_write ,1);
    //nau7802_register_read(NAU7802_CTRL2,&ctrl2_reg_read,1);
    while(1)
    {
        nau7802_register_read(NAU7802_CTRL2,&ctrl2_reg_read,1);
        if(ctrl2_reg_read & 0b00000100)
        {
            ESP_LOGI(TAG1, "Offset calibration in progress"); 
        }
        else
        {
            ESP_LOGI(TAG1, "Offset calibration is completed"); 
            break; 
        }

    }
 return PASS;

}


/*****************************************************************************/
/**
* \brief set LDO 
*
* \description
*   This function configures the AVDD pin to the internal LDO and sets the LDO value as per 
    the parameter received by the function. 
*   
*
* @param      LDO value
*  
* @return     TOTALIZER_ERROR_STATUS
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
TOTALIZER_ERROR_STATUS setLDO(uint8_t ldovalue)
{
    uint8_t ctrl1_reg_read = 0,ctrl1_reg_write= 0,counter = 0;
    nau7802_register_read(NAU7802_CTRL1,&ctrl1_reg_read,1);
    ctrl1_reg_write = ctrl1_reg_read & 0b11000111; // clear LDO bits
    ctrl1_reg_write |=  ldovalue << 3; 
    nau7802_register_write(NAU7802_CTRL1, &ctrl1_reg_write ,1);
    while (1)
    {
    nau7802_register_read(NAU7802_CTRL1,&ctrl1_reg_read,1);
    ctrl1_reg_read = (ctrl1_reg_read & 0b00111000) >> 3 ;
    if ( ctrl1_reg_read== ldovalue)
      break; //Good to go
    if (counter++ > 100)
      return (ERROR); //Error
    }
     return (PASS);
}


/*****************************************************************************/
/**
* \brief set gain 
*
* \description
*   This function sets the gain of the ADC. The gain value is set as per the value
    received in function parameter.
*
* @param     Gain value
*  
* @return    TOTALIZER_ERROR_STATUS
*
* @author     SK 
*
* @date       02/11/2021
*
* @note
********************************************************************************/
TOTALIZER_ERROR_STATUS setGain(uint8_t gainValue)
{
    uint8_t ctrl1_reg_read = 0,ctrl1_reg_write= 0,counter = 0;
    if (gainValue > 0b111)
    gainValue = 0b111; //Error check
    nau7802_register_read(NAU7802_CTRL1,&ctrl1_reg_read,1);
    ctrl1_reg_write = ctrl1_reg_read & 0b11111000; // clear gain bits
    ctrl1_reg_write |=  gainValue; 
    nau7802_register_write(NAU7802_CTRL1, &ctrl1_reg_write ,1);
    while (1)
    {
    nau7802_register_read(NAU7802_CTRL1,&ctrl1_reg_read,1);
    ctrl1_reg_read = (ctrl1_reg_read & 0b00000111);
    if ( ctrl1_reg_read== gainValue)
      break; //Good to go
    if (counter++ > 100)
      return (ERROR); //Error
    }
     return (PASS);
}

/*****************************************************************************/
/**
* \brief set sample rate
*
* \description
*   This function sets the sampling rate of the ADC. The rate value is set as per the value
    received in function parameter.
*
* @param      rate value
*  
* @return     totalizer error state
*
* @author     SK 
*
* @date       02/11/2021
*
* @note
********************************************************************************/
TOTALIZER_ERROR_STATUS setSampleRate(uint8_t rate)
{
    uint8_t ctrl2_reg_read = 0,ctrl2_reg_write= 0,counter = 0;
    if (rate > 0b111)
    rate = 0b111; //Error check
    nau7802_register_read(NAU7802_CTRL2,&ctrl2_reg_read,1);
    ctrl2_reg_write = ctrl2_reg_read & 0b10001111; // clear CRS bits
    ctrl2_reg_write |=  rate << 4; 
    nau7802_register_write(NAU7802_CTRL2, &ctrl2_reg_write ,1);
    while (1)
    {
    nau7802_register_read(NAU7802_CTRL2,&ctrl2_reg_read,1);
    ctrl2_reg_read = (ctrl2_reg_read & 0b01110000) >> 4;
    if ( ctrl2_reg_read== rate)
      break; //Good to go
    if (counter++ > 100)
      return (ERROR); //Error
    }
     return (PASS);
}

/*****************************************************************************/
/**
* \brief i2c based, NAU7802 registers read
*
* \description
*   This function reads the value of the required internal register of the NAU7802, 
    the read sequence is followed based on the NAU7802 dataheet and the API are refered 
    from the ESP official website   
*
* @param      register address, data buffer, no. bytes to read
*  
* @return     totalizer error state
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
void nau7802_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    //i2c_master_write_read_device(I2C_MASTER_NUM, NAU7802_ADC_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    //return ;
    i2c_cmd_handle_t read_cmd;
    read_cmd =  i2c_cmd_link_create();
      
     i2c_master_start(read_cmd);
     i2c_master_write_byte(read_cmd,0x54,1);
     i2c_master_write_byte(read_cmd,reg_addr,1);
     i2c_master_start(read_cmd);
     i2c_master_write_byte(read_cmd,0x55,1);
     
     i2c_master_read(read_cmd,data,1,1);
     i2c_master_stop(read_cmd);
     i2c_master_cmd_begin(0,read_cmd,0xFFFF);
     i2c_cmd_link_delete(read_cmd);

}

/*****************************************************************************/
/**
* \brief i2c based, NAU7802 registers write
*
* \description
*   TThis function writes the value of the required internal register of the NAU7802, 
    the write sequence is followed based on the NAU7802 dataheet and the API are refered 
    from the ESP official website   
*
* @param      register address, data buffer, no. of bytes to read
*  
* @return     None
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
void nau7802_register_write(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t write_cmd;
    //uint8_t data =0x06;
   
     write_cmd =  i2c_cmd_link_create();
     i2c_master_start(write_cmd);
     i2c_master_write_byte(write_cmd,0x54,1);
     i2c_master_write_byte(write_cmd,reg_addr,1);
     i2c_master_write(write_cmd,data,1,1);
     i2c_master_stop(write_cmd);
     i2c_master_cmd_begin(0,write_cmd,0xFFFF);
     i2c_cmd_link_delete(write_cmd);

    //i2c_master_write_to_device(I2C_MASTER_NUM, NAU7802_ADC_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    //return ;
}

/*****************************************************************************/
/**
* \brief Read ADC data register
*
* \description
*   This function reads the 3 8 bit data registers to compute 24 bit converted 
    ADC value.
    0x12, 0x13, 0x14 are the address of the three data register of the ADC.
*
* @param      None
*  
* @return     returns 24 bit ADC value
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
uint32_t nau7802_loadsensor_value (void)
{
     i2c_cmd_handle_t write_cmd;
     uint32_t sensordata1=0,sensordata2=0,sensordata3=0;
     uint32_t unsignedresult= 0, tempres= 0;
     int32_t magresult=0;


    
     nau7802_register_read(0x14,&sensordata1,1);
    //ESP_LOGI(TAG1, "pressure value1 %d bytes", sensordata1);
     nau7802_register_read(0x13,&sensordata2,1);
    // ESP_LOGI(TAG1, "pressure value2 %d bytes", sensordata2);
     nau7802_register_read(0x12,&sensordata3,1);
    // ESP_LOGI(TAG1, "pressure value3 %d bytes", sensordata3);
    
    tempres = (tempres | sensordata3<<16) | (sensordata2 << 8) | (sensordata1) ; 


    // recover sign from 24 bit signed original value
   magresult = (int32_t)(tempres<<8);

    return (magresult >> 8);
  // return tempres;

}


/*****************************************************************************/
/**
* \brief calculate pressure value
*
* \description
*   This function calculates the applied pressure value using the ADC raw value
*
* @param      ADC raw value
*  
* @return     Pressure value
*
* @author     SK 
*
* @date       10/02/2022
*
* @note
********************************************************************************/
int32_t pressure_adc_milli_bar_from_raw(uint32_t raw)
{
    int32_t result = 0;

    // Calculate milliVolt from ADC raw value
    // Output of ADC is 23 bits + 1 signed bit. With reference voltage = 3V,
    // the current volt value can be calculated as raw * (3000/2^23)
    int64_t micro_volt = ((int64_t)raw * L2_PRESSURE_ADC_REF_VOLTAGE);
    micro_volt = micro_volt >> L2_PRESSURE_ADC_BITS;

    // 13 micro Volt per millibar -> pressure is equal to (1000 * milliVolt)/13 mBar
    result = (int32_t)(micro_volt / L2_PRESSURE_MICROVOLT_PER_MBAR);
    return (result);

}



