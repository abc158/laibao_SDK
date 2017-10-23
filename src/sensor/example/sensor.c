/*****************************************************************************
* edit by yongyong.li
*****************************************************************************/
#include "hal_amicro_gpio.h"
#include "adc_chan.h"
#include "syscall_api.h"
#include "ui-config.h"
#include "sensor/sensor.h"

extern const IO_PIN_CFG io_table[HAL_MAX];

U16 adcCache[ADC_CHAN_MAX];

U16 adccache_lt_f[8];
U16 adccache_lt_r[8];

volatile u16 timel_4khz_counter = 0;
volatile u8 hal_lt_counter = 0;
volatile u8 cliff_index_on = 0;
volatile u8 cliff_index_off= 0;	

volatile u8 light_index_on[2]={0};
volatile u8 light_index_off[2]={0};

static u8 useok=0;
static u8 robot_ir_init=0;


typedef struct
{
  u8 logic_chan ;
  u8 phy_chan ;
}ir_sensor_map_t;

static u16 signal_threshold_on[IR_SENSOR_NUM];
static u16 signal_threshold_off[IR_SENSOR_NUM];


const ir_sensor_map_t remap[IR_SENSOR_NUM]={
  {CLIFF_RIGHT,ADC_CHAN_CLIFF_LEFT},
  {CLIFF_FRONTRIGHT,ADC_CHAN_CLIFF_FRONTLEFT},
  {CLIFF_FRONTLEFT,ADC_CHAN_CLIFF_FRONTRIGHT},
  {CLIFF_LEFT,ADC_CHAN_CLIFF_RIGHT},
  {CLIFF_REAR_RIGHT,ADC_CHAN_REAR_CLIFF_LEFT},
  {CLIFF_REAR_LEFT,ADC_CHAN_REAR_CLIFF_RIGHT} 
};

const u8 lt_phy_logic[8]={
  LT_FRONTLEFT,
  LT_CENTERLEFT,
  LT_CENTERLEFT_L,
  LT_LEFT,  
  LT_FRONTRIGHT,
  LT_CENTERRIGHT,
  LT_CENTERRIGHT_L,
  LT_RIGHT,
};

static u8 signal_varq[IR_SENSOR_NUM] = {0};

static s16 signal_queue_on[IR_SENSOR_NUM][4];
static s16 signal_queue_off[IR_SENSOR_NUM][4];

s16 signal_average_on[IR_SENSOR_NUM] ={0};
s16 signal_average_off[IR_SENSOR_NUM] = {0};

static s16 signal_queue_lt_on_f[8][4];
static s16 signal_queue_lt_off_f[8][4];

s16 signal_average_lt_on_f[8] ={0};
s16 signal_average_lt_off_f[8] = {0};

static s16 signal_queue_lt_on_r[8][4];
static s16 signal_queue_lt_off_r[8][4];

s16 signal_average_lt_on_r[8] ={0};
s16 signal_average_lt_off_r[8] = {0};

static u16 signal_threshold_on[IR_SENSOR_NUM]; 
static u16 signal_threshold_off[IR_SENSOR_NUM];

volatile s16 signal_delta[IR_SENSOR_NUM]  = {0};
volatile s16 signal_delta_lt_r[8]  = {0};

#define CLIFF_LED_ATD(m)          \
  m( ADC_CHAN_CLIFF_RIGHT)        \
  m( ADC_CHAN_CLIFF_FRONTRIGHT ) \
  m( ADC_CHAN_CLIFF_FRONTLEFT) \
  m( ADC_CHAN_CLIFF_LEFT)\
  m( ADC_CHAN_REAR_CLIFF_RIGHT)\
  m( ADC_CHAN_REAR_CLIFF_LEFT)

#define LT_LED_ATD(m)          \
  m( ADC_CHAN_LT_BUMPER_RIGHT)       \
  m( ADC_CHAN_LT_BUMPER_LEFT)


#define ADC_CAPTER_LT_F(i)           adccache_lt_f[i] = (U16)(adcResult_p[ADC_CHAN_LT_BUMPER_RIGHT]); \
                                       adccache_lt_f[i+4] = (U16)(adcResult_p[ADC_CHAN_LT_BUMPER_LEFT]);

#define ADC_CAPTER_LT_R(i)           adccache_lt_r[i] = (U16)(adcResult_p[ADC_CHAN_LT_BUMPER_RIGHT]); \
                                       adccache_lt_r[i+4] = (U16)(adcResult_p[ADC_CHAN_LT_BUMPER_LEFT]);
  
#define ADC_CAPTER(i)           adcCache[i] = (U16)(adcResult_p[i]); 

#define SAMPLE_CLIFF_LED()      CLIFF_LED_ATD(ADC_CAPTER)

#define WIFF_LED_ON   1
#define WIFF_LED_OFF  0
                                       
static u8  lt_onoff_swith = 0;
static u8  cliff_onoff_swith = 0;
static s16 cliff_filter[4];

void sensor_threshold_update(const ui_config_t* ui_config)
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

void set_cliff_threshold(SENSOR_E chan, int val)
{
  signal_threshold_on[chan] = val;
  signal_threshold_off[chan] = val;
}

void reset_cliff_threshold(void)
{
  sensor_threshold_update(get_ui_config());
}

#define AM_IO_A GPIOE(4)
#define AM_IO_B GPIOE(6)
#define AM_IO_C GPIOE(7)

#define AM_IO_RIGHT_HC2  GPIOD(3)
#define AM_IO_LEFT_HC2   GPIOD(9)
#define AM_IO_RIGHT_HC1  GPIOD(4)
#define AM_IO_LEFT_HC1   GPIOD(10)

//c h, b m, a l
static void chose_chan(u8 chan)
{
  switch(chan)
  {
  case 0:
    syscall_irq->gpio_set_value(AM_IO_A,0);
    syscall_irq->gpio_set_value(AM_IO_B,0);
    syscall_irq->gpio_set_value(AM_IO_C,0);
    break;
  case 1:
    syscall_irq->gpio_set_value(AM_IO_A,1);
    syscall_irq->gpio_set_value(AM_IO_B,0);
    syscall_irq->gpio_set_value(AM_IO_C,0);  
    break;
  case 2:
    syscall_irq->gpio_set_value(AM_IO_A,0);
    syscall_irq->gpio_set_value(AM_IO_B,1);
    syscall_irq->gpio_set_value(AM_IO_C,0);    
    break;
  case 3:
    syscall_irq->gpio_set_value(AM_IO_A,1);
    syscall_irq->gpio_set_value(AM_IO_B,1);
    syscall_irq->gpio_set_value(AM_IO_C,0);    
    break;
  case 4:
    syscall_irq->gpio_set_value(AM_IO_A,0);
    syscall_irq->gpio_set_value(AM_IO_B,0);
    syscall_irq->gpio_set_value(AM_IO_C,1);      
    break;
  case 5:
    syscall_irq->gpio_set_value(AM_IO_A,1);
    syscall_irq->gpio_set_value(AM_IO_B,0);
    syscall_irq->gpio_set_value(AM_IO_C,1);      
    break;
  case 6:
    syscall_irq->gpio_set_value(AM_IO_A,0);
    syscall_irq->gpio_set_value(AM_IO_B,1);
    syscall_irq->gpio_set_value(AM_IO_C,1);      
    break;
  case 7:
    syscall_irq->gpio_set_value(AM_IO_A,1);
    syscall_irq->gpio_set_value(AM_IO_B,1);
    syscall_irq->gpio_set_value(AM_IO_C,1);      
    break;    
  }
}

static void hal_lighttouch_adc()
{
  U32 *adcResult_p = (U32 *)ADC_BASE_ADDR;
  switch(hal_lt_counter)
  {
  case 0:
    if(!lt_onoff_swith)
    {
      syscall_irq->gpio_set_value(AM_IO_RIGHT_HC1,WIFF_LED_ON); //F
      syscall_irq->gpio_set_value(AM_IO_LEFT_HC1,WIFF_LED_ON);  //F
      chose_chan(0);
    }
    break;
  case 1:
    break;
  case 2: 
    ADC_CAPTER_LT_F(0);//F on 0
    chose_chan(2);
    break;
  case 3:
    ADC_CAPTER_LT_F(1);//F on 1
    chose_chan(4); 
    break;	    
  case 4: 
    ADC_CAPTER_LT_F(2);//F on 2
    chose_chan(6);
    break;
  case 5://on end
    ADC_CAPTER_LT_F(3);//F on 3   
    syscall_irq->gpio_set_value(AM_IO_RIGHT_HC1,WIFF_LED_OFF); //F
    syscall_irq->gpio_set_value(AM_IO_LEFT_HC1,WIFF_LED_OFF);  //F   
    chose_chan(0);
    break;
  case 6://off start
    break;
  case 7:
    ADC_CAPTER_LT_F(0);//F off 0
    chose_chan(2);
    break;
  case 8: 
    ADC_CAPTER_LT_F(1);//F off 2
    chose_chan(4);    
    break;
  case 9:
    ADC_CAPTER_LT_F(2);//F off 4
    chose_chan(6); 
    break;
  case 10:
    ADC_CAPTER_LT_F(3);//r off 8 
    if(!lt_onoff_swith)
    {
      syscall_irq->gpio_set_value(AM_IO_RIGHT_HC2,WIFF_LED_ON); //r
      syscall_irq->gpio_set_value(AM_IO_LEFT_HC2,WIFF_LED_ON);  //r
      chose_chan(1);
    } 
    break;
  case 11:
    break;
  case 12:
    ADC_CAPTER_LT_R(0);//r on 1
    chose_chan(3);    
    break;
  case 13:
    ADC_CAPTER_LT_R(1);//r on 3
    chose_chan(5);       
    break;
  case 14:
    ADC_CAPTER_LT_R(2);//r on 5
    chose_chan(7);    
    break;
  case 15:
    ADC_CAPTER_LT_R(3);//r on 7
    syscall_irq->gpio_set_value(AM_IO_RIGHT_HC2,WIFF_LED_OFF); //r
    syscall_irq->gpio_set_value(AM_IO_LEFT_HC2,WIFF_LED_OFF);  //r     
    chose_chan(1);  
    break;  
  case 16:
    break;
  case 17:
    ADC_CAPTER_LT_R(0);//r off 1
    chose_chan(3);      
    break;
  case 18:
    ADC_CAPTER_LT_R(1);//r off 3
    chose_chan(5);     
    break;    
  case 19:
    ADC_CAPTER_LT_R(2);//r off 5
    chose_chan(7);      
    break;  
  case 20:
    ADC_CAPTER_LT_R(3);//r off 7    
    break;     
  }
}

void sensor_gather(void)
{
  if(robot_ir_init==0)
    return;
  
  U32 *adcResult_p = (U32 *)ADC_BASE_ADDR;
  int  adcStep = timel_4khz_counter & 0x7; 
  switch (adcStep)
  {
  case 0:
    if(!cliff_onoff_swith)
    {
      syscall_irq->gpio_set_value(io_table[AM_IO_CLIFF_LED].start,WIFF_LED_ON); //
    }
    break;
  case 1: 
    break;
  case 2:
    SAMPLE_CLIFF_LED();  
   syscall_irq->gpio_set_value(io_table[AM_IO_CLIFF_LED].start,WIFF_LED_OFF); //    
    break;	    
  case 3: 
    break;
  case 4:
    SAMPLE_CLIFF_LED();//CLIFF OFF
    break;
  case 5:
    break;
  case 6: 
    break;
  case 7:
    break;    
  }

  hal_lighttouch_adc();
}


static void hal_lt_sensor(void)
{
  switch(hal_lt_counter)
  {
  case 0:
    break;
  case 1:
    break;
  case 2:
    break;
  case 3:
    break;
  case 4:
    break;
  case 5:
    {//lt on far
          int i = 0;
          if(light_index_on[0] >=4)
          {
              light_index_on[0] = 0;             
          }          
           for(i=0;i<8;i++) 
           {
              signal_queue_lt_on_f[i][light_index_on[0]] = adccache_lt_f[i];
              signal_average_lt_on_f[i] = signal_queue_lt_on_f[i][0]+signal_queue_lt_on_f[i][1]+signal_queue_lt_on_f[i][2]+signal_queue_lt_on_f[i][3]; 
           }
           light_index_on[0]++; 
    }    
    break;
  case 6:
    break;    
  case 7:
    break;   
  case 8:
    break;
  case 9:
    break;
  case 10:
    {
          u8 i = 0;
          if(light_index_off[0] >=4)
          {
              light_index_off[0] = 0;             
          }          
           for(i=0;i<8;i++) 
           {
               signal_queue_lt_off_f[i][light_index_off[0]] = adccache_lt_f[i];
               signal_average_lt_off_f[i] = signal_queue_lt_off_f[i][0]+signal_queue_lt_off_f[i][1]+signal_queue_lt_off_f[i][2]+signal_queue_lt_off_f[i][3]; 
           }
           light_index_off[0]++; 
    }     

    if(light_index_off[0] >=4 && light_index_on[0]>=4)
    {
    	u8 lt = 0;
        u8 i = 0;
        u8 j = 0;
	if(useok==0)
	{
	    break;
	}
    	for(i = 0;i < 8;i++)
        {
            //j = i+8;
            j = lt_phy_logic[i];
            signal_delta[j] = abs(signal_average_lt_off_f[i] - signal_average_lt_on_f[i]);

            if(signal_delta[j]  >= signal_threshold_on[6])
            {
                lt = 1;//lyy 1--0
            }
            else if(signal_delta[j]  < signal_threshold_off[6])
            {
                lt = 0;
            }
            else
            {
                lt = signal_delta[j] & 0x1;
            }
            signal_varq[j] = signal_varq[j] << 1 ;
            signal_varq[j] = signal_varq[j] | lt ;
            signal_varq[j] = signal_varq[j] & 0xff;          
        }   
        
        signal_varq[LT_CENTERRIGHT] |= signal_varq[LT_CENTERRIGHT_L];
        signal_varq[LT_CENTERLEFT]  |= signal_varq[LT_CENTERLEFT_L]; 
  
        light_index_off[0] =0;
        light_index_on[0]  =0;
    }
    break; 
  case 11:
    break;  
  case 12:
    break;    
  case 13:
    break;   
  case 14:
    break;
  case 15:
    {//lt on rear
          int i = 0;
          if(light_index_on[1] >=4)
          {
            light_index_on[1] = 0;             
          }          
           for(i=0;i<8;i++) 
           {
               signal_queue_lt_on_r[i][light_index_on[1]] = adccache_lt_r[i];
               signal_average_lt_on_r[i] = signal_queue_lt_on_r[i][0]+signal_queue_lt_on_r[i][1]+signal_queue_lt_on_r[i][2]+signal_queue_lt_on_r[i][3]; 
           }
           light_index_on[1]++; 
    }     
    break;    
  case 16:
    break;   
  case 17:
    break; 
  case 18:
    break;
  case 19:
    break;
  case 20:
    {
          u8 i = 0;
          if(light_index_off[1] >=4)
          {
            light_index_off[1] = 0;             
          }          
           for(i=0;i<8;i++)
           {
               signal_queue_lt_off_r[i][light_index_off[1]] = adccache_lt_r[i];
               signal_average_lt_off_r[i] = signal_queue_lt_off_r[i][0]+signal_queue_lt_off_r[i][1]+signal_queue_lt_off_r[i][2]+signal_queue_lt_off_r[i][3]; 
           }
           light_index_off[1]++; 
    }  

    if(light_index_off[1] >=4 && light_index_on[1]>=4)
    {

        u8 i = 0;
	if(useok==0)
	{
	    break;
	}
    	for(i = 0;i < 8;i++)
        {
            int j = lt_phy_logic[i];
            signal_delta_lt_r[j] = abs(signal_average_lt_off_r[i] - signal_average_lt_on_r[i]);       
        }   
        light_index_off[1] =0;
        light_index_on[1]  =0;
    }
    
    break;
  }
}

void sensor_handle(void)
{
  if(robot_ir_init==0)
    return;
  
  int  adcStep = timel_4khz_counter & 0x7; 

  
  if( cliff_index_off > 3 )
  {
      useok = 1;
  }
  
  switch (adcStep)
  {
  case 0: 
    //calc result of the far
    break;
  case 1: //calc result 
    break;
  case 2:
    {//cliff
        u8 i = 0;
        if(cliff_index_on >=4)
        {
            cliff_index_on = 0;             
        }
        
        {
           for(i=0;i<=5;i++) 
           {
               signal_queue_on[i][cliff_index_on] = adcCache[remap[i].phy_chan];
               signal_average_on[i] = signal_queue_on[i][0]+signal_queue_on[i][1]+signal_queue_on[i][2]+signal_queue_on[i][3]; 
           }
        }
        cliff_index_on++;   
    }    
    break;
  case 3:  
    break;
  case 4://oN
    {
        u8 i = 0;
        if(cliff_index_off >=4)
        {
            cliff_index_off = 0;             
        }
        
        {
           for(i=0;i<=5;i++)
           { 
               signal_queue_off[i][cliff_index_off] = adcCache[remap[i].phy_chan];
               signal_average_off[i] = signal_queue_off[i][0]+signal_queue_off[i][1]+signal_queue_off[i][2]+signal_queue_off[i][3]; 
           }
        }
        cliff_index_off++; 
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
            
    	for(i = 0;i<=5;i++)
        {
            //cliff filter, by lqw 20170109
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
                cliff = signal_varq[i] & 0x1;
            }
            
            cliff_filter[i]=temp_s16;
            
            signal_varq[i] = signal_varq[i] << 1 ;
            signal_varq[i] = signal_varq[i] | cliff ;
            signal_varq[i] = signal_varq[i] & 0xff;
        }
        cliff_index_off = 0;
        cliff_index_on  = 0;
    }
    break;
  case 6:    
    break;
  case 7:
    break;
  default:
    break;
  }  
  hal_lt_sensor();
  
  robot_lt_update(signal_varq);
}

void robot_close_led(void)
{
    syscall_irq->gpio_set_value(AM_IO_RIGHT_HC2,WIFF_LED_OFF); 
    syscall_irq->gpio_set_value(AM_IO_LEFT_HC2,WIFF_LED_OFF);   
    syscall_irq->gpio_set_value(AM_IO_RIGHT_HC1,WIFF_LED_OFF); 
    syscall_irq->gpio_set_value(AM_IO_LEFT_HC1,WIFF_LED_OFF); 
    syscall_irq->gpio_set_value(io_table[AM_IO_CLIFF_LED].start,WIFF_LED_OFF);     
}

uint16_t hal_isr(void)
{
  if(robot_ir_init==0)
    return 1;
  
  timel_4khz_counter = (timel_4khz_counter + 1) & 0xff;
  
  hal_lt_counter++;
  if(hal_lt_counter > 30)
    hal_lt_counter = 0;
  
  return (1);
}

s16 robot_rear_lt_distance(u8 index)
{
    s8 index_in;
    if(index < LT_FRONTLEFT)
    {
      return 0;
    }
    else
    { 
      index_in  = index - LT_FRONTLEFT;
    }
    return signal_delta_lt_r[index_in];
}

s16 robot_signal_distance(u8 index)
{
    return signal_delta[index];
}


u8 robot_is_cliff(u8 index)
{
  if(signal_varq[index])
  	return 1;
  else
  	return 0;
}

u8 robot_is_lighttouch(u8 index)
{
  if(signal_varq[index])
  	return 1;
  else
  	return 0;
}

u8 hal_is_rear_cliff(void)
{
  return 0;
}

u8 hal_is_front_cliff(void)
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

}

void robot_sensor_init(void)
{
  int j,i;
  useok = 0;	  
  timel_4khz_counter   = 0; 
  hal_lt_counter     = 0;
  cliff_index_on     = 0;
  cliff_index_off    = 0;
  light_index_on[0]  = 0;
  light_index_on[1]  = 0;
  light_index_off[0] = 0;
  light_index_off[1] = 0;
  
  for(i=0;i<IR_SENSOR_NUM;i++)
  {
    signal_varq[i]=0;
    signal_average_on[i] =0;
    signal_average_off[i]=0;
    signal_delta[i]=0;
  }
  
  for(i=0;i<8;i++)
  {
    adccache_lt_f[i] = 0;
    adccache_lt_r[i] = 0;
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
  sensor_threshold_update(get_ui_config());  
  reset_cliff_threshold();
  robot_ir_init = 1;
}
