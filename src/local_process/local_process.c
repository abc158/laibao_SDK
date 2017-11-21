//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include <ui-config.h>
#include "ui-commands.h"
#include "am_date_base.h"
#include "time.h"
#include  <am_config_macro.h>
#include "am_key.h"
#include "act.h"
#include "local_key_check.h"
#include "remote.h"
#include "display.h"
#include "ui-song-player.h"
#include "syscall_api.h"
#include "wireless/ARF2496K.H"
#include "sensor/sensor.h"
#include "motor/motor.h"
#include "monitor/robot_batter.h"
#include "ui-manager/exception.h"
#include "am_device.h"
#if defined(USE_WIFI_DEMO_1)
#include "wifi/wifi_demo1/simwifi.h"
#include "wifi/wifi_demo1/simsweep.h"
#elif defined(USE_WIFI_DEMO_2)
#include "wifi/wifi_demo2/simwifi_demo2.h"
#include "wifi/wifi_demo2/SimSweep_demo2.h"


#include "am_tm_api.h"
#endif
#define USE_WATCHDOG_MONITOR  //需要打开看门狗监控，则需要定义
#define FEED_DOG_MAX_TIME     10000 //ms

/* 唤醒源 */
volatile U8 wakeupSource;
/*系统空闲计时*/
U16 idle_cnt;
int wdg_fd =-1;
u8 robot_start_work =0;

extern void ir_rx_capture_4khz(void);
extern void ir_tx_send_4khz(void);
extern void ui_rx_server( void);
extern void ui_uart_cmd_rounte(void);
extern void robot_current_init(void);
extern void set_adjust_adc_value_to_system(void);
extern void wifi_uart_cmd_rounte(void);
extern u8 battery_switch_check(void);
/****************************************************************
*Function   :  start_watchdog
*Description:  启动看门狗 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void start_watchdog()
{
#ifdef USE_WATCHDOG_MONITOR
  u16 max_check_time = FEED_DOG_MAX_TIME;//ms, 10s
  wdg_fd = open(DEV_WDG,0);
  ioctl(wdg_fd,WDT_START,&max_check_time);  
#endif
}

/****************************************************************
*Function   :  stop_watchdog
*Description:  停止看门狗 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void stop_watchdog(void)
{
#ifdef USE_WATCHDOG_MONITOR  
  ioctl(wdg_fd,WDT_STOP,0); 
#endif
}

/****************************************************************
*Function   :  feed_watchdog
*Description:  喂狗 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void feed_watchdog(void)
{
#ifdef USE_WATCHDOG_MONITOR  
  ioctl(wdg_fd,WDT_FEED,0); 
#endif
}

/****************************************************************
*Function   :  local_app_init
*Description:  app初始化  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
extern void InitBK2425(void);
extern void set_38k_on(void);
extern void set_38k_off(void);
void local_app_init(void)
{
  /*init all app*/
  _key_init();
  _remote_init();
  _act_init();
  _display_init();
  set_display_power_up_flag(1);
  songplayer_init();
  robot_ir_detection_init();
  robot_motor_init();
  robot_battery_init();
  robot_current_init();
  robot_universl_wheel_init();//初始化，exit需要在睡眠时处理
  set_adjust_adc_value_to_system();
  set_38k_on();
  InitBK2425();
  //InitARF2496k();
  sys_timer_register(HZ_4K,(long)ir_tx_send_4khz,1);//红外优先级最高
  #if defined(USE_WIFI_DEMO_1)
  UART_Config_PinResetWifiCard();
  #elif defined(USE_WIFI_DEMO_2)
  wifi_init();
  #endif
  if(battery_switch_check() == 1)
  {
		robot_start_work =1;
      songplayer_play_id(SONG_ID_POWER_UP, 1);
  }
  start_watchdog();
  
}



/****************************************************************
*Function   :  main_app_task
*Description:  UI的处理函数，每10ms执行一次  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
extern U8 SPI_Read_Reg(U8 reg);
extern U8 SPI_RW(U8 value)   ;
extern U8 Get_Chip_ID(void);
extern void Send_Packet(U8 type,U8* pbuf,U8 len);
static bool wifi_open=true;
extern void print_cliff(void);
void main_app_task(void *arg)
{
  UI_STATE_E s;
  uint8_t WIFI_STATUS;
  uint32_t wifi_start_time;
  wifi_start_time= timer_ms();
  while(1)
  {  
    //延时太短wifi容易进入失败
    if((wifi_open==true)&&(timer_elapsed(wifi_start_time)>=200))
   {
        wifi_open=false;
        WIFI_STATUS=InsertExtCmd(RestoreFactorySet); //进入wifi配网模式
        set_reset_wifi_flag(1);
   }
    
    {
      key_routine();
      remote_routine();
      #if defined(USE_UART_WIFI)
      #if defined(USE_WIFI_DEMO_1)
      WifiData_Period_Process();
      #elif defined(USE_WIFI_DEMO_2)
      uart_server_routine();
      #endif
      wifi_uart_cmd_rounte();
      #endif
      
    } 

    {      
      s = get_ui_state();
      //if(s != UI_ENTER_SLEEPING)
      {                    
        act_routine(); 
        display_routine(); 
        songplayer_routine();
      } //else {
        //songplayer_quit_playing();
      //}
      robot_battery_monitor_routine();
      sys_exception_monitor();
      //if(s == UI_TEST)   //PC 测试需要去掉这个判断条件
      {
        self_test_routine();
      }
      ui_handle_idle(); 
      #if defined(USE_WIFI_DEMO_2)
      map_data_process();
      #endif
    }
    
    feed_watchdog();
    sleep(1);
  }
  
}

