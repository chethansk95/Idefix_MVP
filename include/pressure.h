#include "driver/i2c.h"
#include<stdint.h>

#define L2_PRESSURE_ADC_REF_VOLTAGE 3000 * 1000
#define L2_PRESSURE_ADC_BITS 24
#define L2_PRESSURE_MICROVOLT_PER_MBAR 13
#define SET_BIT_MASK   0x01 

//Bits within the PGA PWR register
typedef enum
{
  NAU7802_PGA_PWR_PGA_CURR = 0,
  NAU7802_PGA_PWR_ADC_CURR = 2,
  NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
  NAU7802_PGA_PWR_PGA_CAP_EN = 7,
} PGA_PWR_Bits;

//Allowed samples per second
typedef enum
{
  NAU7802_SPS_320 = 0b111,
  NAU7802_SPS_80 = 0b011,
  NAU7802_SPS_40 = 0b010,
  NAU7802_SPS_20 = 0b001,
  NAU7802_SPS_10 = 0b000,
} NAU7802_SPS_Values;

typedef enum
{
  NAU7802_LDO_2V4 = 0b111,
  NAU7802_LDO_2V7 = 0b110,
  NAU7802_LDO_3V0 = 0b101,
  NAU7802_LDO_3V3 = 0b100,
  NAU7802_LDO_3V6 = 0b011,
  NAU7802_LDO_3V9 = 0b010,
  NAU7802_LDO_4V2 = 0b001,
  NAU7802_LDO_4V5 = 0b000,
} NAU7802_LDO_Values;

//Allowed gains
typedef enum
{
  NAU7802_GAIN_128 = 0b111,
  NAU7802_GAIN_64 = 0b110,
  NAU7802_GAIN_32 = 0b101,
  NAU7802_GAIN_16 = 0b100,
  NAU7802_GAIN_8 = 0b011,
  NAU7802_GAIN_4 = 0b010,
  NAU7802_GAIN_2 = 0b001,
  NAU7802_GAIN_1 = 0b000,
} NAU7802_Gain_Values;


//Register Map
typedef enum
{
  NAU7802_PU_CTRL = 0x00,
  NAU7802_CTRL1,
  NAU7802_CTRL2,
  NAU7802_OCAL1_B2,
  NAU7802_OCAL1_B1,
  NAU7802_OCAL1_B0,
  NAU7802_GCAL1_B3,
  NAU7802_GCAL1_B2,
  NAU7802_GCAL1_B1,
  NAU7802_GCAL1_B0,
  NAU7802_OCAL2_B2,
  NAU7802_OCAL2_B1,
  NAU7802_OCAL2_B0,
  NAU7802_GCAL2_B3,
  NAU7802_GCAL2_B2,
  NAU7802_GCAL2_B1,
  NAU7802_GCAL2_B0,
  NAU7802_I2C_CONTROL,
  NAU7802_ADCO_B2,
  NAU7802_ADCO_B1,
  NAU7802_ADCO_B0,
  NAU7802_ADC = 0x15, //Shared ADC and OTP 32:24
  NAU7802_OTP_B1,     //OTP 23:16 or 7:0?
  NAU7802_OTP_B0,     //OTP 15:8
  NAU7802_PGA = 0x1B,
  NAU7802_PGA_PWR = 0x1C,
  NAU7802_DEVICE_REV = 0x1F,
  NAU7802_ENABLE_STREAMING = 0x1D,
} Scale_Registers;

//Bits within the PU_CTRL register
typedef enum
{
  NAU7802_PU_CTRL_RR = 0,
  NAU7802_PU_CTRL_PUD,
  NAU7802_PU_CTRL_PUA,
  NAU7802_PU_CTRL_PUR,
  NAU7802_PU_CTRL_CS,
  NAU7802_PU_CTRL_CR,
  NAU7802_PU_CTRL_OSCS,
  NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;


typedef enum 
{
  ERROR =0,
  PASS,

}TOTALIZER_ERROR_STATUS;


typedef struct 
{
   int32_t rawpressuresensor;
   int32_t pressure_millibar;
   uint32_t  T_data_buff;
   uint32_t volume;
   uint32_t subvolume;
   uint32_t flow;
}sensor;

typedef struct 
{
   uint32_t uncom_pressure;
   
}uncom_sensor;

sensor sensor_data;
uncom_sensor sensor_uncom_value;

void nau7802_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
void nau7802_register_write(uint8_t reg_addr, uint8_t *data, size_t len);
uint32_t nau7802_loadsensor_value (void);
int32_t pressure_adc_milli_bar_from_raw(uint32_t raw);
float convert_digital_analog(uint32_t raw);
float weight_adc_kg_from_raw (int32_t raw);
TOTALIZER_ERROR_STATUS  nau7802_Initialzation(void);
void reset (void);
TOTALIZER_ERROR_STATUS powerup(void);
TOTALIZER_ERROR_STATUS setLDO(uint8_t ldovalue);
TOTALIZER_ERROR_STATUS setGain(uint8_t gainValue);
TOTALIZER_ERROR_STATUS setSampleRate(uint8_t rate);
TOTALIZER_ERROR_STATUS calirate(void);
void adc_start_conversion(void);
void nau7802_data_register_read(uint8_t reg_addr, int32_t *data);