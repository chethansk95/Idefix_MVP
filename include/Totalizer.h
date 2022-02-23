#ifndef __TOTALIZER_H__
#define __TOTALIZER_H__ 
#endif

#ifdef __TOTALIZER_H__
#include<stdint.h>
/*********************************** macro definations **************************/
// macro for commad and address of the parameters of totalizer
#define SET_DELETE_SEGMENTS   0x50
#define TEMPERATURE           0xA0
#define PRESSURE              0xA1
#define SET                   0x73 
#define GET                   0x67
#define COMMAND               0x63
#define SET_SYMBOL            0x50
#define SYSMBOL_CRC_L         0xD9
#define SYSMBOL_CRC_H         0x17
#define TEMPERATURE_ADD       0xA0
#define PRESSURE_ADD          0xA1

// macros for display symbols 
#define NARROW_BAND           0x00000080
#define LTE                   0x00000040
#define BLUETOOTH_CONNECTED   0x00000020
#define BLUETOOTH             0x00000010
#define MB                    0x00000008
#define SUB_GHZ               0x00000004
#define AP                    0x00000002
#define WIFI                  0x00000001
#define CHARGED100            0x00008000
#define CHARGED75             0x00004000
#define CHARGED50             0x00002000
#define CHARGED25             0x00001000
#define MOBILE100             0x00000800
#define MOBILE75              0x00000400
#define MOBILE50              0x00000200
#define MOBILE25              0x00000100
#define PRESSURE_UNIT         0x00800000
#define TEMPERATURE_UNIT_C    0x00400000
#define TEMPERATURE_UNIT_F    0x00200000
#define HALF                  0x00100000
#define ONE_FOURTH            0x00080000
#define ONE_EIGTH             0x00040000
#define VALVE_CLOSED          0x00020000
#define CHARGING              0x00010000
#define ERROR                 0x80000000
#define DIAG                  0x40000000
#define EMPTY                 0x20000000
#define LEAK                  0x10000000
#define BURST                 0x08000000
#define TAMPER                0x04000000
#define CALIB                 0x02000000
#define LOW_BAT               0x01000000

/******************************************Global variables***********************/
uint32_t displaydata;

/****************************************structure*******************************/
typedef struct 
{
   uint8_t send_data[20];
   uint8_t receive_data[20];

}uart_comm;
uart_comm UART_DataIntance;
/***************************************enum**************************************/
typedef enum 
{
   TOTALIZER_OK,
   TOTALIZER_ERROR 
}totalizer_error_t;

typedef enum
{
   E_NB = 0x31,
   E_LTE,
   E_BLUETOOTH_CONNECTED,
   E_BLUETOOTH,
   E_WIRELESS_MBUS,
   E_SUBGHZ,
   E_WLAN_ACCESS_POINT,
   E_WIFI,
   E_C100,
   E_C75 = 0x61,
   E_C50,
   E_C25,
   E_M100,
   E_M75,
   E_M50,
   E_M25,
   E_PRESSURE_UINIT,
   E_TEMP_UNIT_C,
   E_TEMP_UNIT_F,
   E_HALF,
   E_ONEFOURTH,
   E_ONEEIGHT,
   E_VALVE_CLOSED,
   E_CHARGING,
   E_ERROR,
   E_DIAG,
   E_EMPTY,
   E_LEAK,
   E_BURST,
   E_TAMPER,
   E_CALIB,
   E_LOWBAT
}display_symbol_t;

/****************************************************function prototypes***********************/
uint8_t * CRC16(uint8_t *data, uint8_t size);
totalizer_error_t L2_totalizer_set_pressure (uint32_t pressure);
totalizer_error_t L2_totalizer_set_temperature (uint32_t temperature);
totalizer_error_t L2_totalizer_set_display_symbol (display_symbol_t symbol, uint8_t state);
totalizer_error_t L2_totalizer_get_data (uint32_t *value, uint8_t address);
uint32_t * Decimal_ascii_conversion( uint32_t value, uint32_t *index);


#endif