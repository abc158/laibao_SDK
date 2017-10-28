//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/
#include "charge.h"
#include "syscall_api.h"
#include "util/current.h"
#include "am_date_base.h"
#include "ui-song-player.h"
#include "am_config_macro.h"

CHARGING_STATE_ERROR_E charging_error;
CHARGING_STATE_ERROR_E charging_record_error;

u32 charge_stop_cnt;
u16 charge_neg_cnt;
u32 charging_start_recover_minutes;
u32 charging_start_full_minutes;
u32 charging_trickle_timeout;
u32 charging_trickle_ticks;
u32 charging_start_complete_minutes;
u16 charging_vol_count;
u32 charging_trickle_on_counts;
u32 charging_on_counts;
u16 charging_trickle_off_counts;
u16 g_comlete_charging;
u16 charging_state;
u16 trickle_charging_time = 0;
u16 charging_current_setpoint;
u16 before_charging_volt;
u8  charge_overflow;
u8  detect_debund;
u8  comlete_charged;
u8  charging_last;
u8  charge_detect_cnt;

const u16 charging_voltage_max_allowed = CHARGING_MAXVOL;

#define PURLING_TIME  100*5*60 /*��ʼ���ʱ��������ʱ��*/
#define MAX_VOLT      0xffff

s16  g_static_voltage = MAX_VOLT;

extern const IO_PIN_CFG io_table[];

/****************************************************************
*Function   :  convert_to_voltage
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ѵ�ѹadcֵת�ɵ�ѹ����λmv��
*CallBy     :  
*Input      :  ����
*              v_adc �� ��ѹ��adc
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
s16 convert_to_voltage( s16 v_adc) 
{
  s32 mv= 18032*v_adc;//0.18 ---> 18032; 0.174 --> 18975
  mv = (mv>>12);
  return (s16)mv;
}

/****************************************************************
*Function   :  convert_to_current
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ѵ���adcֵת�ɵ�ѹ����λmA��
*CallBy     :  
*Input      :  ����
*              v_adc �� ������adc
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
s16 convert_to_current( s16 c_adc) 
{
  s16 ret =  (s16)(((0.95 - c_adc*3.3/4096.0f)/0.41)*1000);
  return ret;
}

/****************************************************************
*Function   :  charging_print
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ӡ��������Ϣ����ѹ�����������״̬��
*CallBy     :  һ���ڳ������
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  �����ڱ���ʵ��ͬ�����ܺ���
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void charging_print(void)
{
  static u32 one_second=0;
  
  if(get_total_seconds()>(one_second+10))
  {
    one_second = get_total_seconds();
    int16_t adc=get_adc_chan_val( ADC_CHAN_BATTERY_CURRENT );                   
    float current = (950 - ((adc*3300)/4096))/0.41f;
    printf("charging state:%d",get_ui_manager()->charging_state());//charging_state);
    printf(" v:%d c:%d mA t:%d\n",battery_voltage_average(),(int)current,battery_temperature_average());
    
  }
}

/****************************************************************
*Function   :  charging_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ʼ��������
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void charging_init(void)
{
  charging_start_recover_minutes = 0;
  charging_trickle_timeout = 0;
  charging_start_full_minutes = 0;
  charging_error = CHARGING_NONE;
  charging_state = CHARGING_OFF;
  charging_vol_count = 0;
  g_comlete_charging = 0;
  trickle_charging_time = 0;
  set_charge_voltage(CHARGING_MAXVOL);
  set_adaptor_output_voltage(ADAPTER_OUTPUT_VOL);
}

/****************************************************************
*Function   :  charging_state_get
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ȡ���״̬
*CallBy     :  �κεط���Ŀǰ����ӿ��Ǳ���ʵ�֣���Ϊע���SDKʹ��
*Input      :  ��
*Output     :  ��
*Return     :  ���� charging_state�� ���״̬
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u16 charging_state_get(void)
{
  return charging_state;
}


/****************************************************************
*Function   :  charge_current_get
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ��ǰʵ�ʵĳ����� (��λ��mA)
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*Output     :  ��
*Return     :  ���� ʵ�ʵĳ�����ֵ (mA)
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
S16 charge_current_get(void)
{
    return convert_to_current(get_adc_chan_val( ADC_CHAN_BATTERY_CURRENT ));   
}

/****************************************************************
*Function   :  get_charge_stop_current
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ȡ�Ƿ��Ѿ�������
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ���� 1�� ������   0��δ����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
static S8 get_charge_stop_current(void)
{
  if(charge_current_get() <= 200)
  {
    charge_stop_cnt++;
  }
  else
  {
    if(charge_stop_cnt>0)
      charge_stop_cnt--;
  }
  if(charge_stop_cnt > 120000)//if low current charge for 20min,  complete it
  {
    return 1;
  }
  ///////////////////////////////
  if(charge_current_get() < 0)
  {
    charge_neg_cnt++;
  }
  else
  {
    if(charge_neg_cnt>0)
      charge_neg_cnt--;
  }
  
  if(charge_neg_cnt > 6000)//60s is neg continue
  {
    return -1;
  }
  
  return 0;
}


/****************************************************************
*Function   :  reset_charge_stop_current
*Author     :  lyy
*Date       :  2017.4.20
*Description:  �ָ��жϳ���Ƿ�����״̬
*CallBy     :  �κεط�
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
static void reset_charge_stop_current(void)
{
  charge_stop_cnt = 0;
  charge_neg_cnt  = 0;
}

/****************************************************************
*Function   :  go_into_trickle_charging
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����������ģʽ
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
static void go_into_trickle_charging(void)
{
  v_ref_init();
  set_charging_enable(1);//close charging
  charging_start_full_minutes = get_total_seconds()/60;
  charging_start_recover_minutes = get_total_seconds()/60;        
  charging_state = CHARGING_PURLING;//����������³��  
}

/****************************************************************
*Function   :  process_charging
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����������
*CallBy     :  Ŀǰ�Ǹ�SDK����
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u8 begin_charge_report = 0;
static int play_song_cnt = 0;
static u8 play_song_flag = 0;
void process_charging(void)
{
  //�����̿���
  
  if(!begin_charge_report)
  {
    play_song_cnt++;
    if( (play_song_cnt > 200) && (!play_song_flag) )//2s������
    {
      if( sys_runing_mode_get()==ROBOT_STATE_CHARGING) 
      {
        songplayer_play_id(SONG_ID_CHARGING_START, 0);
		printf("begin_charging\r\n");
        begin_charge_report = 1;
        play_song_flag = 1;
      }
    }       
  }
  
  if(before_charging_volt == 0)
  {
      before_charging_volt = battery_voltage_average();
      printf("before volt = %d \r\n",before_charging_volt);
  }  
  
  {
    if(check_battery_voltage(charging_voltage_max_allowed+100)) 
    {
      charging_state = CHARGING_OFF;
      g_comlete_charging = 1;
      set_charging_enable(0);//close charging
      charging_print();
      return;
    }
    
    //���������ֻ�Ǹ�ui��״̬
    if(g_comlete_charging)
    {
      if(check_battery_voltage((charging_voltage_max_allowed-15)))
      {
        set_charging_enable(0);
        charging_state = CHARGING_COMPLETE;//����
        comlete_charged = 1;
        charging_print();
        return;
      }
      else if(check_battery_voltage((charging_voltage_max_allowed-30)))
      {
        g_comlete_charging = 0;
        go_into_trickle_charging();
      }
      else
      {
        charging_state = CHARGING_OFF;
        g_comlete_charging = 0;
      }
    }
    else 
    {
      if( (before_charging_volt > (g_static_voltage-15)) && (before_charging_volt >(charging_voltage_max_allowed-25)) )//���֮ǰ�ǳ����������Ǳ����������Ż��������ʱ��Ӧ�����ѳ�����״̬��
      {
        g_static_voltage = MAX_VOLT;
        go_into_trickle_charging();
        comlete_charged = 1;
      }
    }

    if(trickle_charging_time < PURLING_TIME)
    {
      trickle_charging_time++;
    }

    charging_print();
    switch(charging_state)
    {
    case CHARGING_OFF:
      charging_vol_count = 0;
      comlete_charged   = 0;
      charging_start_full_minutes = get_total_seconds()/60;
      charging_start_recover_minutes = get_total_seconds()/60;
      v_ref_init();
//      if(check_battery_available()) //����Ƿ���ã������¶��Ƿ���ߣ��Ƿ��в�����
//      {
//        charging_error = CHARGING_NOBATTERY;
//        charging_record_error = CHARGING_NOBATTERY;
//      }
//      else 
      if (!(check_battery_voltage(CHARGING_RECOVERYVOL)))
      {
        charging_state = CHARGING_RECOVERY;
        set_charging_enable(1);//close charging
        charging_trickle_on_counts = get_total_seconds(); 
      }
      else
      {
        charging_state = CHARGING_ON;
        set_charging_enable(1);//close charging
        charging_on_counts = get_sum_ticks();
        charging_trickle_on_counts = get_total_seconds();			  
      }
      break;
    case CHARGING_RECOVERY:
      //300ma charging
      charging_current_setpoint = 300;
      set_charge_current(charging_current_setpoint);
      if(check_battery_voltage(CHARGING_RECOVERYVOL))
      {
        charging_state = CHARGING_ON;
      }
      if(((get_total_seconds()/60)-charging_start_recover_minutes)>CHARGING_TIME_RECOVERY_MINUTES)
      {
        charging_start_complete_minutes = get_total_seconds()/60;
        charging_state = CHARGING_COMPLETE;
        comlete_charged = 1;
      }
	  			
      break;
    case CHARGING_ON:
      {
        //1000ma
        if(trickle_charging_time < PURLING_TIME)
        {
          charging_current_setpoint = 220;//0.1C ���
        }
        else
        {
          charging_current_setpoint = 800;
        }
        set_charge_current(charging_current_setpoint);
        if((charge_maybe_full()  && (get_charge_stop_current() > 0)) || (get_charge_stop_current()<0))//current < 250ma && voltage >= 16.8v
        {            
            charge_overflow = 1;
        }
          
        if(charge_overflow)//�����ѹ�����������ֵ
        {
          charge_overflow = 0;
          reset_charge_stop_current();
          charging_trickle_timeout = get_total_seconds()/60; //��¼��ʼֵ
          charging_trickle_ticks = get_sum_ticks();
          charging_state = CHARGING_COMPLETE;//CHARGING_PURLING;
          comlete_charged = 1;
        }
        else
        {
          ;//set_charging_enable(1);
        }
        //vol max
        if(((get_total_seconds()/60)-charging_start_full_minutes)>CHARGING_TIME_ALLOW_MINUTES)
        {
          charging_start_complete_minutes = get_total_seconds()/60;
          charging_state = CHARGING_COMPLETE;
          comlete_charged = 1;
        }
      }
      break;
    case CHARGING_PURLING:
      //50ma
      if(trickle_charging_time < PURLING_TIME)
      {
        charging_current_setpoint = 220;//0.1C ���
      }
      else   
      {
        charging_current_setpoint = 800;
      }
      
      if((charge_maybe_full()  && (get_charge_stop_current()>0)) || (get_charge_stop_current()<0))//current < 300ma && voltage >= 16.8v
      {     
        charge_overflow = 1;
      }
      
      if(charge_overflow)//�����ѹ�����������ֵ
      {
        charge_overflow = 0;
        comlete_charged = 1;
        reset_charge_stop_current();           
        charging_state = CHARGING_COMPLETE;
      }
      else
      {
        ;
      }
      
      if(((get_total_seconds()/60)-charging_start_full_minutes)>CHARGING_TIME_PURLING_MINUTES)
      {
          charging_start_complete_minutes = get_total_seconds()/60;
          charging_state = CHARGING_COMPLETE;
      }      

      break;
    case CHARGING_WAITING:
      break;
    case CHARGING_COMPLETE:
      g_comlete_charging = 1;
      break;
    }
  }
   
}


/****************************************************************
*Function   :  process_charging
*Author     :  lyy
*Date       :  2017.4.20
*Description:  �˳�����������
*CallBy     :  Ŀǰ�Ǹ�SDK����
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void quit_charging(void)
{
  begin_charge_report  = 0;
  before_charging_volt = 0;
  play_song_cnt = 0;
  play_song_flag = 0;
  if(comlete_charged)
  {
    g_static_voltage = battery_voltage_average();
    printf("after volt:%d\r\n",g_static_voltage);
    comlete_charged  = 0;
  }
  else
  {
    g_static_voltage = MAX_VOLT;
  }
  
}

/****************************************************************
*Function   :  charging_jack
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����Ƿ����������
*CallBy     :  �κεط�
*Input      :  ��
*Output     :  ��
*Return     :  1����������� 0����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
U16 charging_jack(void)
{
  return (gpio_get_value(IO_CHARGER_HIGH)==0); 
  return 0;
}

/****************************************************************
*Function   :  charging_dock
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����Ƿ��ڳ������
*CallBy     :  �κεط�
*Input      :  ��
*Output     :  ��
*Return     :  1����������� 0����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
U16 charging_dock(void)
{
  return (gpio_get_value(IO_CHARGER_LOW)==0);
}

