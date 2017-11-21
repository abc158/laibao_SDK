//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*****************************************************************************
* edit by yongyong.li
*****************************************************************************/
#include "sensor.h"
#include "hal_amicro_gpio.h"
#include "adc_chan.h"
#include "syscall_api.h"
#include "ui-config.h"
#include "sensor/sensor.h"


extern const adc_chan_t adc_chan_table[ADC_CHAN_MAX];
extern const IO_PIN_CFG io_table[HAL_MAX];

/*缓存每次采样的adc值*/
static U16 adcCache[ADC_CHAN_MAX];

volatile u16 time_4khz_counter = 0;

/*cliff计数采样过程统计的次数*/
volatile u8 cliff_index_on = 0;
volatile u8 cliff_index_off= 0;	

/*lt计数采样过程统计的次数*/
volatile u8 light_index_on[2]={0};
volatile u8 light_index_off[2]={0};

static u8 useok=0;
static u8 robot_ir_init=0;

static s16 cliff_filter[4];

#define USE_LT_AUTO_ADJUST //如果左右的lt 不一致的時候需要進行自動校正
#ifdef USE_LT_AUTO_ADJUST
#define LT_AUTO_ADJUST_THROD 400
s16 signal_offset[8];
#endif

typedef struct
{
  u8 logic_chan;/*逻辑通道*/
  u8 phy_chan;  /*物理通道*/
}ir_sensor_map_t;

/*cliff , lt 的阈值配置， 变量会在sensor_threshold_update里初始化，on: led灯打开时， off：led关闭时*/
static u16 signal_threshold_on[IR_SENSOR_NUM];
static u16 signal_threshold_off[IR_SENSOR_NUM];

/*逻辑通道和物理通道影响关系结构体表*/
const ir_sensor_map_t remap[IR_SENSOR_NUM]={
  {CLIFF_RIGHT,ADC_CHAN_CLIFF_RIGHT}, 
  {CLIFF_FRONTRIGHT,ADC_CHAN_CLIFF_FRONTRIGHT},
  {CLIFF_FRONTLEFT,ADC_CHAN_CLIFF_FRONTLEFT},
  {CLIFF_LEFT,ADC_CHAN_CLIFF_LEFT},
  {CLIFF_REAR_RIGHT,0xFF},
  {CLIFF_REAR_LEFT,0xFF},
  {CLIFF_REV1,0xff},
  {CLIFF_REV2,0xff},  
  {LT_CENTERRIGHT,ADC_CHAN_LT_CENTERRIGHT},
  {LT_FRONTLEFT,ADC_CHAN_LT_FRONTLEFT},
  {LT_RIGHT,ADC_CHAN_LT_RIGHT},
  {LT_LEFT,ADC_CHAN_LT_LEFT},
  {LT_FRONTRIGHT,ADC_CHAN_LT_FRONTRIGHT},
  {LT_CENTERLEFT,ADC_CHAN_LT_CENTERLEFT},
  {LT_CENTERLEFT_L,0xff},
  {LT_CENTERRIGHT_L,0xff},
};

/*cliff，lt的结果*/
static u8 signal_result[IR_SENSOR_NUM] = {0};
/*保存最近4次led on 时的adc值*/
static s16 signal_queue_on[IR_SENSOR_NUM][4];
/*保存最近4次led off 时的adc值*/
static s16 signal_queue_off[IR_SENSOR_NUM][4];
/*保存最近4次led on 时的adc值总和*/
static s16 signal_average_on[IR_SENSOR_NUM] ={0};
/*保存最近4次led off 时的adc值总和*/
static s16 signal_average_off[IR_SENSOR_NUM] = {0};
/*保存一个周期 led on 和 off 的采样差值*/
volatile s16 signal_delta[IR_SENSOR_NUM]  = {0};

#define CLIFF_LED_ADC(M)          \
  M( ADC_CHAN_CLIFF_RIGHT)        \
  M( ADC_CHAN_CLIFF_FRONTRIGHT ) \
  M( ADC_CHAN_CLIFF_FRONTLEFT) \
  M( ADC_CHAN_CLIFF_LEFT)

#define LT_LED_ADC(M)          \
  M( ADC_CHAN_LT_CENTERRIGHT)        \
  M( ADC_CHAN_LT_FRONTLEFT ) \
  M( ADC_CHAN_LT_RIGHT)       \
  M( ADC_CHAN_LT_LEFT) \
  M( ADC_CHAN_LT_FRONTRIGHT) \
  M( ADC_CHAN_LT_CENTERLEFT)

#define SAMPLE_ADC(i)           adcCache[i] = (U16)(adcResult_p[adc_chan_table[i].phy_chan]); 
#define SAMPLE_LT_LED()         LT_LED_ADC(SAMPLE_ADC)
#define SAMPLE_CLIFF_LED()      CLIFF_LED_ADC(SAMPLE_ADC)
        
#define SENSOR_LED_ON   1
#define SENSOR_LED_OFF  0

/*lt 控制开关*/
static u8 lt_onoff_swith = 0;

/*cliff 控制开关*/
static u8 cliff_onoff_swith = 0;

/****************************************************************
*Function   :  sensor_threshold_update
*Author     :  lyy
*Date       :  2017.4.20
*Description:  初始化lt，cliff阈值
*CallBy     :  
*Input      :  参数
*              ui_config:  参考 ui_config_t 结构
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void sensor_threshold_update(const ui_local_config_t* ui_config)
{
  //CLIFF ON

  signal_threshold_on[CLIFF_RIGHT]      = ui_config->cliff_threshold.right_on;
  signal_threshold_on[CLIFF_FRONTRIGHT] = ui_config->cliff_threshold.front_right_on;
  signal_threshold_on[CLIFF_FRONTLEFT]  = ui_config->cliff_threshold.front_left_on;  
  signal_threshold_on[CLIFF_LEFT]       = ui_config->cliff_threshold.left_on;
  signal_threshold_on[CLIFF_REAR_RIGHT] = ui_config->cliff_threshold.rear_right_on;
  signal_threshold_on[CLIFF_REAR_LEFT]  = ui_config->cliff_threshold.rear_left_on;
  //CLIFF OFF
  signal_threshold_off[CLIFF_RIGHT]     = ui_config->cliff_threshold.right_off; 
  signal_threshold_off[CLIFF_FRONTRIGHT]= ui_config->cliff_threshold.front_right_off;
  signal_threshold_off[CLIFF_FRONTLEFT] = ui_config->cliff_threshold.front_left_off;
  signal_threshold_off[CLIFF_LEFT]      = ui_config->cliff_threshold.left_off;
  signal_threshold_off[CLIFF_REAR_RIGHT]= ui_config->cliff_threshold.rear_right_off;
  signal_threshold_off[CLIFF_REAR_LEFT] = ui_config->cliff_threshold.rear_left_off;  
  
  //LT ON  
  signal_threshold_on[LT_CENTERRIGHT]  = ui_config->lighttouch_threshold.center_right_on;
  signal_threshold_on[LT_FRONTLEFT]    = ui_config->lighttouch_threshold.front_left_on;
  signal_threshold_on[LT_RIGHT]        = ui_config->lighttouch_threshold.right_on;
  signal_threshold_on[LT_LEFT]         = ui_config->lighttouch_threshold.left_on;
  signal_threshold_on[LT_FRONTRIGHT]   = ui_config->lighttouch_threshold.front_right_on;
  signal_threshold_on[LT_CENTERLEFT]   = ui_config->lighttouch_threshold.center_left_on;
  //LT OFF
  signal_threshold_off[LT_CENTERRIGHT] = ui_config->lighttouch_threshold.center_right_off; 
  signal_threshold_off[LT_FRONTLEFT]   = ui_config->lighttouch_threshold.front_left_off;
  signal_threshold_off[LT_RIGHT]       = ui_config->lighttouch_threshold.right_off;
  signal_threshold_off[LT_LEFT]        = ui_config->lighttouch_threshold.left_off;
  signal_threshold_off[LT_FRONTRIGHT]  = ui_config->lighttouch_threshold.front_right_off;
  signal_threshold_off[LT_CENTERLEFT]  = ui_config->lighttouch_threshold.center_left_off;
}

/****************************************************************
*Function   :  set_cliff_threshold
*Author     :  lyy
*Date       :  2017.4.20
*Description:  设置cliff的阈值
*CallBy     :  
*Input      :  参数
*              chan:  SENSOR_E cliff逻辑通道
*              val :  阈值(adc)
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void set_cliff_threshold(SENSOR_E chan, int val)
{
  signal_threshold_on[chan] = val;
  signal_threshold_off[chan] = val;
}

/****************************************************************
*Function   :  reset_cliff_threshold
*Author     :  lyy
*Date       :  2017.4.20
*Description:  复位cliff阈值
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void reset_cliff_threshold(void)
{
  sensor_threshold_update(get_local_ui_config());
}

/****************************************************************
*Function   :  sensor_gather
*Author     :  lyy
*Date       :  2017.4.20
*Description:  cliff，lt 的采样流程
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void sensor_gather(void)
{
  if(robot_ir_init==0)
    return;
  
  U32 *adcResult_p = (U32 *)ADC_BASE_ADDR;
  int  adcStep = (time_4khz_counter & 0x7);
  switch (adcStep)
  {
  case 0:
    if(!lt_onoff_swith)
    {
      gpio_set_value(AM_IO_LIGHT_TOUCH_LED,SENSOR_LED_ON); 
    }
    if(!cliff_onoff_swith)
    {
      gpio_set_value(AM_IO_CLIFF_LED,SENSOR_LED_ON); //
    }
    break;
  case 1: 
    break;
  case 2:
    SAMPLE_LT_LED();//lt on
    SAMPLE_CLIFF_LED();  
    gpio_set_value(AM_IO_LIGHT_TOUCH_LED,SENSOR_LED_OFF); 
    gpio_set_value(AM_IO_CLIFF_LED,SENSOR_LED_OFF); //    
    break;	    
  case 3:  
    break;
  case 4:
    SAMPLE_LT_LED();  //lt off
    SAMPLE_CLIFF_LED();//CLIFF OFF
    break;
  case 5:
    break;
  case 6:   
    break;
  case 7:
    break;
  }
}

/****************************************************************
*Function   :  sensor_handle
*Author     :  lyy
*Date       :  2017.4.20
*Description:  cliff，lt 的采样后，结果处理流程
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void sensor_handle(void)
{
  s16 temp = 0;
  if(robot_ir_init==0)
    return;
  
  int  adcStep = time_4khz_counter & 0x7; 
  
  if( cliff_index_off > 3 )
  {
      useok = 1;
  }
  
  switch (adcStep)
  {
  case 0:  
    break;
  case 1: //calc result 
    break;
  case 2:
    {//lt
          int i = 0;
          if(light_index_on[0] >=4)
          {
            light_index_on[0] = 0;             
          }          
           for(i=8;i<=13;i++) 
           {
               signal_average_on[i] = 0;
               signal_queue_on[i][light_index_on[0]] = (adcCache[remap[i].phy_chan] & 0x00000fff);
               for(int j = 0; j<4; j++)
               {               
                 signal_average_on[i] += signal_queue_on[i][j]; 
               }               
           }
           light_index_on[0]++; 
    }
    {//cliff
        u8 i = 0;
        if(cliff_index_on >=4)
        {
            cliff_index_on = 0;             
        }
        
        {
           for(i=0;i<=3;i++) 
           {
               signal_average_on[i] = 0;
               signal_queue_on[i][cliff_index_on] = (adcCache[remap[i].phy_chan] & 0x00000fff);
               for(int j=0;j<4;j++)
               {
                 signal_average_on[i] += signal_queue_on[i][j]; 
               }               
           }
        }
        cliff_index_on++;   
    }    
    break;
  case 3:  
    break;
  case 4://off
    { 
        u8 i = 0;
        if(cliff_index_off >=4)
        {
            cliff_index_off = 0;             
        }
        
        {
           for(i=0;i<=3;i++)
           {
               signal_average_off[i] = 0;
               signal_queue_off[i][cliff_index_off] = (adcCache[remap[i].phy_chan] & 0x00000fff);
               for(int j = 0; j<4; j++)
               {
                 signal_average_off[i] += signal_queue_off[i][j]; 
               }
           }
        }
        cliff_index_off++; 
    }
    
    {//lt
          u8 i = 0;
          if(light_index_off[0] >=4)
          {
            light_index_off[0] = 0;             
          }          
           for(i=8;i<=13;i++) 
           {
               signal_average_off[i] = 0;
               signal_queue_off[i][light_index_off[0]] = (adcCache[remap[i].phy_chan] & 0x00000fff);
               for(int j = 0; j<4; j++)
               {
                 signal_average_off[i] += signal_queue_off[i][j]; 
               }
           }
           light_index_off[0]++; 
    }
  
    break;
  case 5://calc result
    if(cliff_index_off >= 4 && cliff_index_on >= 4)
    {
    	u8 cliff = 0;
        u8 i = 0;
        s16 temp_s16;
	if(useok==0)
	{
	    break;
	}
        
    	for(i = 0;i<=3;i++)
        {
            temp_s16 = (abs(signal_average_off[i] - signal_average_on[i]));
            if(temp_s16 > cliff_filter[i])
            {
              signal_delta[i]=temp_s16;
            }
            else
            {
              signal_delta[i]=cliff_filter[i];
            }
               
            
            if(((signal_average_off[i] + signal_average_on[i])<1024) && (signal_delta[i]  < signal_threshold_off[i]))
            {
              signal_delta[i]=signal_delta[i]<<4;
            }
            

            if(signal_delta[i]  >= signal_threshold_on[i])
            {
                cliff = 0;
            }
            else if(signal_delta[i]  < signal_threshold_off[i])
            {
                cliff = 1;
            }
            else
            {
                cliff = signal_result[i] & 0x1;
            }
            
            cliff_filter[i]=temp_s16;
            
            signal_result[i] = signal_result[i] << 1 ;
            signal_result[i] = signal_result[i] | cliff ;
            signal_result[i] = signal_result[i] & 0xff;
        }
        cliff_index_off = 0;
        cliff_index_on  = 0;
    }
    if(light_index_off[0] >=4 && light_index_on[0]>=4)
    {
    	u8 lt = 0;
        u8 i = 0;
	if(useok==0)
	{
	    break;
	}
    	for(i = 8;i<=13;i++)
        {
#ifdef USE_LT_AUTO_ADJUST
            temp = abs(signal_average_off[i] - signal_average_on[i]);
            if(temp < signal_offset[i-8] && temp>30)
            {
              signal_offset[i-8]=temp;
            }
            signal_delta[i] = abs(temp - signal_offset[i-8]);
#else
            signal_delta[i] = abs(signal_average_off[i] - signal_average_on[i]);
#endif          
            if(signal_delta[i]  >= signal_threshold_on[i])
            {
                lt = 1;//lyy 1--0
            }
            else if(signal_delta[i]  < signal_threshold_off[i])
            {
                lt = 0;
            }
            else
            {
                lt = signal_result[i] & 0x1;
            }
            signal_result[i] = signal_result[i] << 1 ;
            signal_result[i] = signal_result[i] | lt ;
            signal_result[i] = signal_result[i] & 0xff;
        }   
        light_index_off[0] =0;
        light_index_on[0]  =0;
    }
    break;
  case 6:    
    break;
  case 7:
    break;
  }
  /*把lt的结果传送到sdk*/
  robot_lt_update(signal_result);
}

/****************************************************************
*Function   :  robot_close_sensor_led
*Author     :  lyy
*Date       :  2017.4.20
*Description:  闭关led灯，主要是节省功耗
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_close_sensor_led(void)
{
    gpio_set_value(AM_IO_LIGHT_TOUCH_LED,SENSOR_LED_OFF); 
    gpio_set_value(AM_IO_CLIFF_LED,SENSOR_LED_OFF);     
}


uint16_t hal_isr(void)
{
  if(robot_ir_init==0)
    return 1;
  time_4khz_counter = (time_4khz_counter + 1) & 0xff;
  return (1);
}

s16 robot_rear_lt_distance(u8 ingdex)
{
  return 0;
}

s16 robot_signal_distance(u8 index)
{
  return signal_delta[index];
}

u8 robot_is_cliff(u8 index)
{
  if(signal_result[index])
  	return 1;
  else
  	return 0;
}

u8 robot_is_lighttouch(u8 index)
{
  if(signal_result[index])
  	return 1;
  else
  	return 0;
}

u8 robot_is_rear_cliff(void)
{
  return 0;
}

u8 robot_is_front_cliff(void)
{
  return 0;
}

void set_lighttouch_enable(u8 en)
{
  lt_onoff_swith = en;
}

void set_cliff_enable(u8 en)
{
  cliff_onoff_swith = en;
}

void reset_lt_auto_offset()
{
#ifdef USE_LT_AUTO_ADJUST
  int i;
  for(i=0;i<6;i++)
  {
    signal_offset[i] = LT_AUTO_ADJUST_THROD;
  }
#endif
}

void robot_sensor_init(void)
{
  int j,i;
  useok = 0;	  
  time_4khz_counter   = 0;  
  cliff_index_on     = 0;
  cliff_index_off    = 0;
  light_index_on[0]  = 0;
  light_index_on[1]  = 0;
  light_index_off[0] = 0;
  light_index_off[1] = 0;
  
  for(i=0;i<IR_SENSOR_NUM;i++)
  {
    signal_result[i]=0;
    signal_average_on[i] =0;
    signal_average_off[i]=0;
    signal_delta[i]=0;
  }
  
  for(i=0;i<IR_SENSOR_NUM;i++)
  {
    for(j=0;j<4;j++)
    {
      signal_queue_on[i][j] = 0;
      signal_queue_off[i][j]= 0; 
    }
  }
  reset_lt_auto_offset();
  //reset_cliff_threshold();
  sensor_threshold_update(get_local_ui_config());
  robot_ir_init = 1;
}


void print_touch(void)
{
	robot_sensor_gather_start(1);
  printf("touch: l=%d cl=%d fl=%d fr=%d cr=%d r=%d\r\n", \
         signal_delta[LT_LEFT], signal_delta[LT_CENTERLEFT], signal_delta[LT_FRONTLEFT], \
         signal_delta[LT_FRONTRIGHT], signal_delta[LT_CENTERRIGHT], signal_delta[LT_RIGHT]);
}

void print_cliff(void)
{
	robot_sensor_gather_start(1);
  printf("cliff: cl=%d fl=%d fr=%d cr=%d \r\n",\
         signal_delta[CLIFF_LEFT], signal_delta[CLIFF_FRONTLEFT], signal_delta[CLIFF_FRONTRIGHT], \
         signal_delta[CLIFF_RIGHT]);        
}
