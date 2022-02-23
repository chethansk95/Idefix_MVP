/****************************************************************************/
/**
*
* @file Totalizer.c
* @{
* @details
* 
* This file contains the \e functions related to totalizer board of the <b> IDEFIX </b>().
*
* <b> Major Tasks </b> of this file includes

* -# set and clear the special symbols on the totalizer display
* -# wtire the pressure and temperature value to the totalizer board
* -# read volume, subvolume and flow values from the totalizer board
*
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who    Date       Changes
* ----- ------ --------   ---------------------------------------------------------------------
* 1.0   sk    02-11-2021
*
* </pre>
*****************************************************************************/
/** @} */

/******************************************Includes*******************************************/
#include<stdint.h>
#include "Totalizer.h"
#include "main.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include<string.h>
#include "esp_log.h"

/*********************************************Global variables********************************/
uart_comm UART_DataIntance;
uint8_t crcdatabuf[2];
static const char *TAG = "Totalizer";

/*****************************************************************************/
/**
* \brief This function calculates the checksum for the data packet
*
* \description
*   This function is used to calculate the checksum of the send and receive packet over any communication 
*   protocol. The polynkmial used for the calculation is 0xA0001. Reference document is INOSON Totalizer board
*   Appendix A1 : Claculation of the CRC 
*
* @param    data and size   
*  
* @return   crcdatabuf
*
* @author   SK 
*
* @date     02/11/2021
*
* @note
********************************************************************************/
uint8_t * CRC16(uint8_t *data, uint8_t size)
{
  const uint16_t polynomial= 0xA001;
  uint16_t crc = 0xFFFF;
  uint8_t byte=0,i;

  while( byte < size)                       
{
   crc = crc ^ data[byte];                   // xor the crc value with the next data byte

  for(i=0; i<8; i++)                                           // repeate the process till the 8-bit byte is processed
  {
        if ((crc & 0x0001) == 1) /* test for LSB =1 bit 0 */    // checks whether lsb is 1
        {
             crc = (uint16_t)((crc >> 1) ^ polynomial);               
        }
        else
        {
            crc >>= 1;                        // shift right the crc value 
        }
  }
  byte++; 
}

crcdatabuf[0]= crc & 0x00FF;
crcdatabuf[1]= (crc & 0xFF00)>>8;

return crcdatabuf;
   
}

/*****************************************************************************/
/**
* \brief 
*
* \description
*   This function sets or clear the symbols on the display and check for the 
    response from the totalizer board. Persistancy check for response is 
    implemented for 100 times just for testing, it can be reduced if hardware
    works fine with the less repeat check value. 
*
* @param      symbol and state
*  
* @return     totalizer error state
*
* @author     SK 
*
* @date       02/11/2021
*
* @note
********************************************************************************/

totalizer_error_t L2_totalizer_set_display_symbol (display_symbol_t symbol, uint8_t state)
{ // check wich symbol is meant
   uint8_t crcdata_buff[7], persistance_counter=0;

  if(state==1)                             // check for set state 
  {
    switch(symbol)                         // jump to the required symbol to set on the totalizer board display
    {
      case E_NB : 
      displaydata |= NARROW_BAND;
      break;

      case E_LTE:
      displaydata |= LTE;
      break;

      case E_BLUETOOTH_CONNECTED:
      displaydata |= BLUETOOTH_CONNECTED;
      break;

      case E_BLUETOOTH : 
      displaydata |= BLUETOOTH;
      break;

      case E_WIRELESS_MBUS:
      displaydata |= MB;
      break;

      case E_SUBGHZ:
      displaydata |= SUB_GHZ;
      break;

      case E_WLAN_ACCESS_POINT:
      displaydata |= AP;
      break;

      case E_WIFI:
      displaydata |= WIFI;
      break;

      case E_C100:
      displaydata |= CHARGED100;
      break;

      case E_C75:
      displaydata |= CHARGED75;
      break;

      case E_C50:
      displaydata |= CHARGED50;
      break;

      case E_C25:
      displaydata |= CHARGED25;
      break;

      case E_M100:
      displaydata |= MOBILE100;
      break;

      case E_M75:
      displaydata |= MOBILE75;
      break;

      case E_M50:
      displaydata |= MOBILE50;
      break;

      case E_M25:
      displaydata |= MOBILE25;
      break;

      case E_PRESSURE_UINIT:
      displaydata |= PRESSURE_UNIT;
      break;

      case E_TEMP_UNIT_C:
      displaydata |= TEMPERATURE_UNIT_C;
      break;

      case E_TEMP_UNIT_F:
      displaydata |= TEMPERATURE_UNIT_F;
      break;

      case E_HALF:
      displaydata |= HALF;
      break;

      case E_ONEFOURTH:
      displaydata |= ONE_FOURTH;
      break;

      case E_ONEEIGHT:
      displaydata |= ONE_EIGTH;
      break;

      case E_VALVE_CLOSED:
      displaydata |= VALVE_CLOSED;
      break;

      case E_CHARGING:
      displaydata |= CHARGING;
      break;

      case E_ERROR:
      displaydata |= ERROR;
      break;

      case E_DIAG:
      displaydata |= DIAG;
      break;

      case E_EMPTY:
      displaydata |= EMPTY;
      break;

      case E_LEAK:
      displaydata |= LEAK;
      break;

      case E_BURST:
      displaydata |= BURST;
      break;

      case E_TAMPER:
      displaydata |= TAMPER;
      break;

      case E_CALIB:
      displaydata |= CALIB;
      break;

      case E_LOWBAT:
      displaydata |= LOW_BAT;
      break;

      default:
      break;

    }

  }
  else if(state==0)                                 // check for clear state
  { 
    switch(symbol)                                  // jump to the required symbol to set on the totalizer board display
    {
      case E_NB : 
      displaydata &= ~(NARROW_BAND);
      break;

      case E_LTE:
      displaydata &= ~(LTE);
      break;

      case E_BLUETOOTH_CONNECTED:
      displaydata &= ~(BLUETOOTH_CONNECTED);
      break;

      case E_BLUETOOTH : 
      displaydata &= ~(BLUETOOTH);
      break;

      case E_WIRELESS_MBUS:
      displaydata &= ~(MB);
      break;

      case E_SUBGHZ:
      displaydata &= ~(SUB_GHZ);
      break;

      case E_WLAN_ACCESS_POINT:
      displaydata &= ~(AP);
      break;

      case E_WIFI:
      displaydata &= ~(WIFI);
      break;

      case E_C100:
      displaydata &= ~(CHARGED100);
      break;

      case E_C75:
      displaydata &= ~(CHARGED75);
      break;

      case E_C50:
      displaydata &= ~(CHARGED50);
      break;

      case E_C25:
      displaydata &= ~(CHARGED25);
      break;

      case E_M100:
      displaydata &= ~(MOBILE100);
      break;

      case E_M75:
      displaydata &= ~(MOBILE75);
      break;

      case E_M50:
      displaydata &= ~(MOBILE50) ;
      break;

      case E_M25:
      displaydata &= ~(MOBILE25);
      break;

      case E_PRESSURE_UINIT:
      displaydata &= ~(PRESSURE_UNIT);
      break;

      case E_TEMP_UNIT_C:
      displaydata &= ~(TEMPERATURE_UNIT_C);
      break;

      case E_TEMP_UNIT_F:
      displaydata &= ~(TEMPERATURE_UNIT_F);
      break;

      case E_HALF:
      displaydata &= ~(HALF);
      break;

      case E_ONEFOURTH:
      displaydata &= ~(ONE_FOURTH);
      break;

      case E_ONEEIGHT:
      displaydata &= ~(ONE_EIGTH);
      break;

      case E_VALVE_CLOSED:
      displaydata &= ~(VALVE_CLOSED);
      break;

      case E_CHARGING:
      displaydata &= ~(CHARGING);
      break;

      case E_ERROR:
      displaydata &= ~(ERROR);
      break;

      case E_DIAG:
      displaydata &= ~(DIAG);
      break;

      case E_EMPTY:
      displaydata &= ~(EMPTY);
      break;

      case E_LEAK:
      displaydata &= ~(LEAK);
      break;

      case E_BURST:
      displaydata &= ~(BURST);
      break;

      case E_TAMPER:
      displaydata &= ~(TAMPER);
      break;

      case E_CALIB:
      displaydata &= ~(CALIB);
      break;

      case E_LOWBAT:
      displaydata &= ~(LOW_BAT);
      break;

      default:
      break;

    }
  }
 
  UART_DataIntance.send_data[0]=SET;                // manage send packet
  UART_DataIntance.send_data[1]=COMMAND;
  UART_DataIntance.send_data[2]= SET_SYMBOL;
  UART_DataIntance.send_data[3]= (displaydata & 0xFF000000)>>24;
  UART_DataIntance.send_data[4]= (displaydata & 0x00FF0000)>>16;
  UART_DataIntance.send_data[5]= (displaydata & 0x0000FF00)>>8;
  UART_DataIntance.send_data[6]= (displaydata & 0x000000FF);

  memcpy(crcdata_buff, UART_DataIntance.send_data, 7*sizeof(uint8_t));
  crc_16 = CRC16(crcdata_buff, 7);                               // calculate checksum 

  UART_DataIntance.send_data[7]= crc_16[0];                     // load checksum value to the send packet
  UART_DataIntance.send_data[8]= crc_16[1]; 

  uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)UART_DataIntance.send_data, 9);  // send packet

  while(1)                                                // check for response
  {

  uart_read_bytes(ECHO_UART_PORT_NUM, UART_DataIntance.receive_data, BUF_SIZE, 20 / portTICK_RATE_MS);   // read data over uart1 in polling mode
  memcpy(crcdata_buff, UART_DataIntance.receive_data, 3*sizeof(uint8_t));
  crc_16 = CRC16(crcdata_buff, 3);

  if(crc_16[0]== UART_DataIntance.receive_data[3] && crc_16[1]== UART_DataIntance.receive_data[4] && UART_DataIntance.receive_data[2]== SET_SYMBOL)  // match the address and compare calculated and received CRC value
      {
        
        UART_DataIntance.receive_data[2]= 0x00;
        return TOTALIZER_OK;
      }
  else if (persistance_counter > 100)             // if there is a failure repeat check for 100 times and return failure
      {
        return TOTALIZER_ERROR;

      }
    persistance_counter++;
  }
    
}
  


totalizer_error_t L2_totalizer_set_display_value (float value)
{   
  

  // manage packet to sent to totalizer
   /* set te pressure of MID part*/
    


  

  // ceck  if response form totalizer ok

  // if not resend
    // if more than 3 attemts throw error
    return TOTALIZER_ERROR;
}


/*****************************************************************************/
/**
* \brief 
*
* \description
*   This function sets the temperature value on the totalizer board display
    and checks for the response from the totalizer board
*
* @param    Temperature
*  
* @return   Totalizer error status
*
* @author   SK 
*
* @date     02/11/2021
*
* @note
********************************************************************************/
totalizer_error_t L2_totalizer_set_temperature (uint32_t temperature)
{ 
  uint8_t crcdata_buff[7], persistance_counter=0;

  UART_DataIntance.send_data[0]=SET;                               // manage packet to sent to totalizer
  UART_DataIntance.send_data[1]=COMMAND;
  UART_DataIntance.send_data[2]= TEMPERATURE_ADD;
  UART_DataIntance.send_data[3]= (temperature & 0xFF000000)>>24;
  UART_DataIntance.send_data[4]= (temperature & 0x00FF0000)>>16;
  UART_DataIntance.send_data[5]= (temperature & 0x0000FF00)>>8;
  UART_DataIntance.send_data[6]= (temperature & 0x000000FF);

  memcpy(crcdata_buff, UART_DataIntance.send_data, 7*sizeof(uint8_t));
  crc_16 = CRC16(crcdata_buff, 7);                                       // calculate CRC

  UART_DataIntance.send_data[7]= crc_16[0];                             // load CRC value to the send packet
  UART_DataIntance.send_data[8]= crc_16[1];
   
  uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)UART_DataIntance.send_data, 9);  // send packet

  while(1)                                                                            // check for response
  {

  uart_read_bytes(ECHO_UART_PORT_NUM, UART_DataIntance.receive_data, BUF_SIZE, 20 / portTICK_RATE_MS);   // read data over uart1 in polling mode
  memcpy(crcdata_buff, UART_DataIntance.receive_data, 3*sizeof(uint8_t));
  crc_16 = CRC16(crcdata_buff, 3);

  if(crc_16[0]== UART_DataIntance.receive_data[3] && crc_16[1]== UART_DataIntance.receive_data[4] && UART_DataIntance.receive_data[2]== TEMPERATURE_ADD) // match the address and compare calculated and received CRC value
      {
        
        UART_DataIntance.receive_data[2]= 0x00;
        return TOTALIZER_OK;
      }
  else if (persistance_counter > 100)                 // if there is a failure repeat check for 100 times and return failure
      {
        return TOTALIZER_ERROR;

      }
    persistance_counter++;
  }

}


/*****************************************************************************/
/**
* \brief  This function sets pressure value at the totalizer board
*
* \description
*   This function sends the pressure set commad along with the data to the
    totalizer borad and checks for the response from the totalzer board.  
*
* @param    Pressure value   
*  
* @return   Totalizer error status
*
* @author   SK 
*
* @date     02/11/2021
*
* @note
********************************************************************************/
totalizer_error_t L2_totalizer_set_pressure (uint32_t pressure)
{   
  uint8_t crcdata_buff[7], persistance_counter=0;

  UART_DataIntance.send_data[0]=SET;                              // manage packet to sent to totalizer
  UART_DataIntance.send_data[1]=COMMAND;
  UART_DataIntance.send_data[2]= PRESSURE_ADD;
  UART_DataIntance.send_data[3]= 0x00;//(pressure & 0xFF000000)>>24;
  UART_DataIntance.send_data[4]= 0x00;//(pressure & 0x00FF0000)>>16;
  UART_DataIntance.send_data[5]= 0x11;//(pressure & 0x0000FF00)>>8;
  UART_DataIntance.send_data[6]= 0x4A;//(pressure & 0x000000FF);
  
  memcpy(crcdata_buff, UART_DataIntance.send_data, 7*sizeof(uint8_t));
  crc_16 = CRC16(crcdata_buff, 7);                                              // calculate CRC
  
  UART_DataIntance.send_data[7]= crc_16[0];                                     //load CRC value to send packet
  UART_DataIntance.send_data[8]= crc_16[1];

 
  uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)UART_DataIntance.send_data, 9);       // send packet
  
  while(1)                                                                              //check for response
  {
  
  uart_read_bytes(ECHO_UART_PORT_NUM, UART_DataIntance.receive_data, BUF_SIZE, 20 / portTICK_RATE_MS);        //read data over uart 1 polling mode
  memcpy(crcdata_buff, UART_DataIntance.receive_data, 3*sizeof(uint8_t));
  crc_16 = CRC16(crcdata_buff, 3);
  
  if(crc_16[0]== UART_DataIntance.receive_data[3] && crc_16[1]== UART_DataIntance.receive_data[4] && UART_DataIntance.receive_data[2]== PRESSURE_ADD)   // match the address and compare calculated and received CRC value
      {
        
        UART_DataIntance.receive_data[2]= 0x00;
        return TOTALIZER_OK;
      }
  else if (persistance_counter > 100)         // if there is a failure repeat check for 100 times and return failure
      {
        return TOTALIZER_ERROR;

      }
    persistance_counter++;
  }
  
}

/*****************************************************************************/
/**
* \brief  This function reads the set data parameter value from totalizer board
*
* \description
*   This function sends the GET command to the totalizer board and receives the current 
    set data in response form the totalizer board.
*   Data parameters can be read : 
                                 1. Volume 
                                 2. Subvolume 
                                 3. Flow
* @param    pointer to store the received value and address(which data parameter has to be read) 
*  
* @return   Totalizer error state
*
* @author   SK 
*
* @date     02/11/2021
*
* @note
********************************************************************************/

totalizer_error_t L2_totalizer_get_data (uint32_t *value, uint8_t address)
{
  uint8_t crcdata_buff[7], persistance_counter=0; 
  uint32_t mask= 0x000000FF;
  uint32_t data=0;
  
  UART_DataIntance.send_data[0]=GET;                  // manage packet to sent to totalizer
  UART_DataIntance.send_data[1]=COMMAND;
  UART_DataIntance.send_data[2]=address;
  
  memcpy(crcdata_buff, UART_DataIntance.send_data, 3*sizeof(uint8_t));
  crc_16 = CRC16(crcdata_buff, 3);                                      // calculate CRC
  
  UART_DataIntance.send_data[3]= crc_16[0];                             //load CRC value to send packet
  UART_DataIntance.send_data[4]= crc_16[1];
  
  uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)UART_DataIntance.send_data, 5);      // send packet
  
  while(1)                                                                            //check for response
  {
    
    uart_read_bytes(ECHO_UART_PORT_NUM, UART_DataIntance.receive_data, BUF_SIZE, 20 / portTICK_RATE_MS);   //read uart1 data in polling mode
    //uart_read_bytes(ECHO_UART_PORT_NUM, UART_DataIntance.receive_data, BUF_SIZE, 20 / portTICK_RATE_MS);  
    //memcpy(crcdata_buff, UART_DataIntance.receive_data, 6*sizeof(uint8_t));
    //crc_16 = CRC16(crcdata_buff, 6);
    
    if(69== UART_DataIntance.receive_data[7] && 143== UART_DataIntance.receive_data[8] && UART_DataIntance.receive_data[2]== address) //address
   // if(UART_DataIntance.receive_data[6]== 190 && UART_DataIntance.receive_data[7]== 239) //testing purpose
    {
     //*value = ((((((*value | UART_DataIntance.receive_data[3])<<8)|(UART_DataIntance.receive_data[4] & mask))<<8)|(UART_DataIntance.receive_data[5]&mask))<<8)|(UART_DataIntance.receive_data[6]&mask);
      *value = (((((((*value | (UART_DataIntance.receive_data[2] & mask))<<24)|(UART_DataIntance.receive_data[3]  & mask))<<16)|(UART_DataIntance.receive_data[4] &mask))<<8)|(UART_DataIntance.receive_data[5] &mask));
      UART_DataIntance.receive_data[2]= 0x00;
      return TOTALIZER_OK;
    }
    else if (persistance_counter > 100)           // if there is a failure repeat check for 100 times and return failure
    {
      *value=0;
      return TOTALIZER_ERROR;

    }
    persistance_counter++;
  }
}


/*****************************************************************************/
/**
* \brief  This function reads the set data parameter value from totalizer board
*
* \description
*   This function sends the GET command to the totalizer board and receives the current 
    set data in response form the totalizer board.
*   Data parameters can be read : 
                                 1. Volume 
                                 2. Subvolume 
                                 3. Flow
* @param    pointer to store the received value and address(which data parameter has to be read) 
*  
* @return   Totalizer error state
*
* @author   SK 
*
* @date     02/11/2021
*
* @note
********************************************************************************/

uint32_t * Decimal_ascii_conversion( uint32_t value, uint32_t *index )
{
  uint8_t result,i=0;
  uint32_t temp_value = value;
 // printf("%d\n\r",value);
  while(temp_value != 0)
  {
    temp_value = temp_value /10;
    (*index)++;
  }
   //printf("%d\n\r",*index);
  uint32_t buff[*index+2];
  i= *index -1;
  while(value != 0)
  { 
    result = value % 10;
    buff[i--] = result;
    //printf("%d",result);
    value = value /10;
  }

  buff[*index+1]=(uint32_t) "\n";
  buff[*index+2]=(uint32_t) "\r";

  return buff;  
}