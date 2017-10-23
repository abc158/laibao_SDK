//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "ui-manager.h"
#include "ui-msg-handler.h"
#include "ui-song-player.h"
#include "syscall_api.h"
#include <display.h>

extern void cleaning_complete_handle(void);
/****************************************************************
*Function   :  ui_msg_handle
*Description:  ϵͳ��app������Ϣ�ĺ��� 
*Input      :  UiMsgType msg:��Ϣ����     
*              int arg ����
*              void* data ���ݴ�ŵ�ָ��   
*Output     :  ��
*Return     :  ����ͷ���յ�������
*Others     :  
******************************************************************/
int ui_msg_handle(UiMsgType msg, int arg, void* data)
{
  switch(msg)
  {        
//  case UI_MSG_BUTTON_REJECT:
//    songplayer_play_id(SONG_ID_BUTTON_REJECT, 0);
//    break;
    
    
//  case UI_MSG_MAP_SERCHING_START:
//    display_map_serching_start();
//    break;
//    
//  case UI_MSG_MAP_SERCHING_STOP:
//    display_map_serching_stop();
//    break;    
    
//  case UI_MSG_SLEEPING_ENTER:
//    display_power_down();
//    break;
//  case UI_MSG_SLEEPING_EXIT:
//    display_enter_idle();
//    break;
    
//  case UI_MSG_POWER_OFF_ENTER:
//    display_power_off();
//    songplayer_play_id(SONG_ID_POWER_OFF, 0);
//    break;
//    
//  case UI_MSG_POWER_UP_OK:
//    display_power_up();
//    songplayer_play_id(SONG_ID_POWER_UP, 0);
//    break;
    
//  case UI_MSG_CRITICAL_LOW_POWER://���ص͵�
//    display_critical_low_power();
//    songplayer_play_id(SONG_ID_CRITICAL_LOW_POWER, 0);
//    break;
//  case UI_MSG_RESERVE_POWER://Ԥ���͵�
//    display_battery_reserve();
//    songplayer_play_id(SONG_ID_RESERVE_POWER, 0);
//    break;
//  case UI_MSG_BATT_LOW://�ϵ�
//    display_low_power();
//    songplayer_play_id(SONG_ID_LOW_POWER, 0);
//    break;
//  case UI_MSG_BATT_ALMOST_FULL://��������
//    display_battery_almost_full();
//    break;
//  case UI_MSG_BATT_FULL://����
//    display_battery_full();
//    break;              
    
//  case UI_MSG_CHARGING_START:
//    break;
//  case UI_MSG_CHARGING_STARTED:
//    display_charging_enter();
//    songplayer_play_id(SONG_ID_CHARGING_START, 0);
//    break;
//  case UI_MSG_CHARGING_STOPPED:
//    display_charging_stop();
//    //songplayer_play_id(SONG_ID_CHARGING_STOP, 0);
//    break;
//  case UI_MSG_CHARGING_COMPLETE:
//    display_charging_complete();
//    //songplayer_play_id(SONG_ID_CHARGING_COMPLETE, 0);
//    break;
    
//  case UI_MSG_DOCKING_START:
//    break;
//  case UI_MSG_DOCKING_STARTED:
//    display_docking_enter();
//    songplayer_play_id(SONG_ID_DOCKING_START, 0);
//    break;
//  case UI_MSG_DOCKING_STOPPED:
//    display_docking_stop();
//    songplayer_play_id(SONG_ID_DOCKING_STOP, 0);
//    printf("UI_MSG_DOCKING_STOPPED\r\n"); 
//    break;
//  case UI_MSG_DOCKING_COMPLETE:
//    display_docking_complete();
//    //songplayer_play_id(SONG_ID_DOCKING_COMPLETE, 0);    
//    break;
    
//  case UI_MSG_HOMING_START:
//    break;
  case UI_MSG_HOMING_STARTED:  /*�ص���ʼλ��*/
    //display_homing_enter();
    songplayer_play_id(SONG_ID_HOMING_START, 0);
    break;
    
  case UI_MSG_HOMING_STOPPED:  /*ֹͣ�ص���ʼλ��*/
    //display_homing_stop();
    songplayer_play_id(SONG_ID_HOMING_STOP, 0);
    break;
    
  case UI_MSG_HOMING_COMPLETE: /*�Ѿ��ص���ʼλ��*/
    //display_homing_complete();
    songplayer_play_id(SONG_ID_HOMING_COMPLETE, 0); 
    break;
    
//  case UI_MSG_NORMAL_CLEANING_START:
//    break;
//  case UI_MSG_NORMAL_CLEANING_STARTED:
//    display_normal_cleaning_enter();
//    songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
//    break;
//  case UI_MSG_NORMAL_CLEANING_STOPPED:
//    display_normal_cleaning_stop();
//    songplayer_play_id(SONG_ID_NORMAL_CLEANING_STOP, 0);
//    break;
  
  case UI_MSG_CLEANING_COMPLETE:     /*��ɨ���*/
//    display_normal_cleaning_complete();
//    printf("UI_MSG_CLEANING_COMPLETE!!!\r\n");
//    songplayer_play_id(SONG_ID_NORMAL_CLEANING_COMPLETE, 1);
	cleaning_complete_handle();  
    break;
    
  case UI_MSG_CLEANING_PICKUP_STOP:  /*��ɨʱ��̧��*/
    songplayer_play_id(SONG_ID_BE_PICKED_UP, 0);
    break;
    
//  case UI_MSG_SPOT_CLEANING_START:
//    break;
//  case UI_MSG_SPOT_CLEANING_STARTED:
//    display_spot_cleaning_enter();
//    songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
//    break;
//  case UI_MSG_SPOT_CLEANING_STOPPED:
//    display_spot_cleaning_stop();
//    songplayer_play_id(SONG_ID_SPOT_CLEANING_STOP, 0);
//    break;
//  case UI_MSG_SPOT_CLEANING_COMPLETE:
//    display_spot_cleaning_complete();
//    songplayer_play_id(SONG_ID_SPOT_CLEANING_COMPLETE, 0);
//    break;
//    
//  case UI_MSG_WALLFOLLOW_CLEANING_START:
//    break;
//  case UI_MSG_WALLFOLLOW_CLEANING_STARTED:
//    display_wallfollow_cleaning_enter();
//    songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);
//    break;
//  case UI_MSG_WALLFOLLOW_CLEANING_STOPPED:
//    display_wallfollow_cleaning_stop();
//    songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_STOP, 0);
//    break;
//  case UI_MSG_WALLFOLLOW_CLEANING_COMPLETE:
//    display_wallfollow_cleaning_complete();
//    songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_COMPLETE, 0);
//    break;
    
  default:
    break;
    
  }
  return 0;
}












