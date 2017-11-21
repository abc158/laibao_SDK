#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "am_type.h"
#include "am_robot_type.h"
#include "syscall_api.h"
#include "ui-test_cmd_handle.h"
#include "am_key.h"
#include "ui-commands.h"
#include "sensor/sensor.h"
#include "ui-ir.h"
#include "docking-new.h"
#include "charge/charge.h"
#include "syscall_api.h"
#include "cfg/adc_chan.h"
#include "sensor/sensor.h"
#include "am_config_macro.h"
#include "motor/robot_brush.h"
#include "motor/robot_suction.h"
#include "util/current.h"
#include "ui-song-player.h"
#include "act.h"

#define  ADJ_BAT_VOL_OFFSET 0
//-30 -25 -20 -15 -10 -5 0 5 10 15 20 25 30 35 40 45 50
const char temp_list[]={148,117,92,71,54,42,32,25,20,15,12,10,8,6,5,4,3};

/*测试项,如果测试顺序调整,改变这个数组的的测试命令顺序即可*/
TEST_CMD_E test_item_table[UI_TEST_ITEM_MAX]={
	CMD_TEST_ALL_TOUCH_FAR,		
	CMD_TEST_ALL_TOUCH_NEAR,
	CMD_TEST_ALL_CLIFF, 
	CMD_TEST_BUMP_DROP,		
	CMD_TEST_ALL_TOUCH_NEAR,
	CMD_TEST_ALL_CLIFF, 	
  CMD_TEST_ALL_TOUCH_FAR,		
	CMD_TEST_ALL_TOUCH_NEAR,
	CMD_TEST_ALL_CLIFF, 	
  CMD_TEST_ALL_TOUCH_FAR,		
	CMD_TEST_ALL_TOUCH_NEAR,
	CMD_TEST_ALL_CLIFF, 	
  CMD_TEST_ALL_TOUCH_FAR,		
	CMD_TEST_ALL_TOUCH_NEAR,
	CMD_TEST_ALL_CLIFF};

static volatile int self_test_result;   /*测试结果*/
static BumpState bump_state;            /*碰撞状态*/
static U8  wheel_test_event = 0;        /*轮子测试*/
static U8  other_motor_test = 0;        /*主刷,边刷,真空测试*/
static U8  cmd_stasis_test = 0;         /*是否万向轮测试*/
static U16 ui_test_num_last= 0;         /*上一次的测试命令*/
static U32 ui_test_last_ts = 0;         /*检测到红外的开始时间*/
static U32 test_result_dly = 0;         /*检测到红外的延时*/
static U32 stasis_state = 0;            /*万向轮状态*/
static U8  write_nvram_flag = 0;
static U32 ui_test_last_ts2[IR_DECODER_MAX] = {0};
static U8  pc_test_enableq  = 0;
extern TEST_CMD_E ui_test_cmd;
extern void ir_test_all_result(void);
extern U8 get_ir_test(IR_REMOT_POSITION_E index);
extern u8 vaccum_is_maybe_stall(void);
extern void print_touch(void);
extern void print_cliff(void);
extern u8 robot_is_cliff_test(u8 index);
extern u8 robot_is_lighttouch_test(u8 index);
u8 touch_near_flag =0;
extern u8 cliff_led_flag;
static uint16_t vac_current_over_count = 0;                 /*真空异常计数*/
u8 pc_test_flag =0; 
extern void ir_send_on_off(U8 state);
extern u8 lt_flag;
static u8 robot_is_near_hazard(u8 index)
{
  return robot_is_cliff(index);
}

static u8 robot_is_near_wall(u8 index)
{
  return robot_is_lighttouch(index);
}


/****************************************************************
*Function   :  robot_wheel_drop_mask
*Description:  判断是否轮子抬起 
*Input      :  无            
*Output     :  无
*Return     :  轮子抬起状态
*Others     :  
******************************************************************/
WheelDropState robot_wheel_drop_mask(void)
{
    WheelDropState mask = WHEEL_DROP_STATE_NONE;
    if (gpio_get_value(AM_IO_WHEEL_DROP_RIGHT)==0) {
        mask |= WHEEL_DROP_STATE_RIGHT;
    }
    if  (gpio_get_value(AM_IO_WHEEL_DROP_LEFT)==0) {
        mask |= WHEEL_DROP_STATE_LEFT;
    }
    return mask;
}

/****************************************************************
*Function   :  robot_bump_mask
*Description:  判断是否有碰撞 
*Input      :  无            
*Output     :  无
*Return     :  碰撞状态
*Others     :  
******************************************************************/
BumpState robot_bump_mask(void)
{
    BumpState mask = BUMP_MASK_NONE;
    if (gpio_get_value(AM_IO_BUMP_LEFT)==0) {
        mask |= BUMP_FRONT_LEFT;
    }
    if  (gpio_get_value(AM_IO_BUMP_RIGHT)==0) {
        mask |= BUMP_FRONT_RIGHT;
    }
    return mask;
}

/****************************************************************
*Function   :  get_bat_temp
*Description:  获取电池温度 
*Input      :  无            
*Output     :  无
*Return     :  电池温度
*Others     :  
******************************************************************/
U8 get_bat_temp(void)
{
    U8 i;
    U16 temp_r;
    temp_r = (U16)battery_temperature_get();
    for(i=0;i<17;i++)
    {
        if(temp_list[i]<temp_r)
            break;
    }
    if(i == 0)
        i = 1;
    else
        i = i - 1;
    if(i<6)
        return ((-5*(6-i))+30);
    else
        return ((5*(i-6))+30);
}

/****************************************************************
*Function   :  handle_test_cmd
*Description:  测试模式,处理每个测试项
*Input      :  U32 cmd:测试命令            
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_test_cmd(U32 cmd)
{
    WheelDropState drop_mask ;
    U8 tmp_state;
    //S16 tmp_speed;
    U16 tmp_test_result;
    uint32_t tmp_value[2];
    if(cmd != ui_test_num_last)
    {
      if(wheel_test_event)
      {
          wheel_test_event = 0;
          set_motor_vels(0, 0, ACCELERATION_NON_EMERGENCY);
          //stop_motor_test(WHEEL_LEFT);
          //stop_motor_test(WHEEL_RIGHT);
      }
      if(other_motor_test)
      {
          other_motor_test = 0;
          //stop_motor_test(MAIN_BRUSH);
          //stop_motor_test(SIDE_BRUSH);
          //stop_motor_test(SUCTION);
          robot_midbrush_vols_set(0);
          robot_sidebrush_vols_set(0);
          robot_suction_vols_set(0);
      }
      cmd_stasis_test = 0;
    }
    switch (cmd)
    {
    case CMD_TEST_WAIT:      // 0xf0000000,
    {
        self_test_result = 0;
        break;
    }
    case CMD_BUMP_L:         // 0xf0000001, /*TURE OR FALSE*/
    {
        bump_state = get_bump_state();
        self_test_result = (bump_state == BUMP_FRONT_LEFT)?1:0;
        break;
    }
    case CMD_BUMP_R:         // 0xf0000002, /*TURE OR FALSE*/
    {
        bump_state = get_bump_state();
        self_test_result = (bump_state == BUMP_FRONT_RIGHT)?1:0;
        break;
    }
    case CMD_BUMP_B:         // 0xf0000003, /*TURE OR FALSE*/
    {
        bump_state = get_bump_state();
        self_test_result = (bump_state == BUMP_FRONT_CENTER )?1:0;
        break;
    }
    case CMD_CLIFF_L:        // 0xf0000004, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_hazard(CLIFF_LEFT);
        break;
    }
    case CMD_CLIFF_R:        // 0xf0000005, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_hazard(CLIFF_RIGHT);
        break;
    }
    case CMD_CLIFF_FL:       // 0xf0000006, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_hazard(CLIFF_FRONTLEFT);
        break;
    }
    case CMD_CLIFF_FR:       // 0xf0000007, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_hazard(CLIFF_FRONTRIGHT);
        break;
    }
    case CMD_CLIFF_RL:       // 0xf0000008, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_hazard(CLIFF_REAR_RIGHT);
        break;
    }
    case CMD_CLIFF_RR:       // 0xf0000009, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_hazard(CLIFF_REAR_LEFT);
        break;
    }
    case CMD_LT_L:           // 0xf000000a, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_wall(LT_LEFT);
        break;
    }
    case CMD_LT_R:           // 0xf000000b, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_wall(LT_RIGHT);;
        break;
    }
    case CMD_LT_CL:          // 0xf000000c, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_wall(LT_CENTERLEFT);
        break;
    }
    case CMD_LT_CR:          // 0xf000000d, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_wall(LT_CENTERRIGHT);
        break;
    }
    case CMD_LT_FL:          // 0xf000000e, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_wall(LT_FRONTLEFT);
        break;
    }
    case CMD_LT_FR:          // 0xf000000f, /*TURE OR FALSE*/
    {
        self_test_result = robot_is_near_wall(LT_FRONTRIGHT);
        break;
    }
    case CMD_WHEEL_DROP_L:   // 0xf0000010, /*TURE OR FALSE*/
    {
        drop_mask = robot_wheel_drop_mask();
        if(drop_mask & WHEEL_DROP_STATE_LEFT)
        {
            self_test_result = 1;
        }
        else
        {
            self_test_result = 0;
        }
        break;
    }
    case CMD_WHEEL_DROP_R:   // 0xf0000011, /*TURE OR FALSE*/
    {
        drop_mask = robot_wheel_drop_mask();
        if(drop_mask & WHEEL_DROP_STATE_RIGHT)
        {
            self_test_result = 1;
        }
        else
        {
            self_test_result = 0;
        }
        break;
    }
    case CMD_REMOTE:         // 0xf0000012, /*TURE OR FALSE*/
    {
      break;
    }
    case CMD_BUOY_L:         // 0xf0000013, /*TURE OR FALSE*/
    {
        break;
    }
    case CMD_BUOY_R:         // 0xf0000014, /*TURE OR FALSE*/
    {
        break;
    }
    case CMD_BUOY_MIDDLE:
        break;
    case CMD_BAT:            // 0xf0000015, /*BAT ADC VOL*/
    {
        self_test_result = battery_voltage_average();
        printf("CMD_BAT=%d\r\n",self_test_result);
        tmp_value[0] = battery_voltage_average();
        if((tmp_value[0] >= 3649/*3775*/) && (tmp_value[0] < 3954/*4090*/))
        {
            vram_read(ADJ_BAT_VOL_OFFSET, tmp_value, 2);
           if((write_nvram_flag == 0)||(tmp_value[0] != 0x55aa))
           {
              write_nvram_flag = 1;
              tmp_value[0] = 0x55aa;
              //vram_write(0, &tmp_value, 1);
              tmp_value[1] = battery_voltage_average();
              vram_write(ADJ_BAT_VOL_OFFSET, tmp_value, 2);
              songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);

           }
           else
           {
              songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
           }                           
        }
        break;
    }
    case CMD_TEMP:           // 0xf0000016, /*TEMP ADC */
    {
        self_test_result = get_bat_temp();
        //self_test_result = battery_temperature_average();
        break;
    }
    case CMD_JACK_CHARGING:  // 0xf0000017, /*TURE OR FALSE*/
    {
        self_test_result = charging_jack(); 
        break;
    }
    case CMD_JACK_CURRENT:   // 0xf0000018, /*CURRENT*/
    {
        self_test_result = charge_current_get(); 
        break;
    }
    case CMD_DOCK_CHARGING:  // 0xf0000019, /*TURE OR FALSE*/
    {
        self_test_result = charging_dock(); 
        break;
    }
    case CMD_DOCK_CURRENT:   // 0xf000001a, /*CURRENT*/
    {
        self_test_result = charge_current_get(); 
        printf("CMD_DOCK_CURRENT=%d\r\n",self_test_result);
        break;
    }
    case CMD_WHEEL_R_FORWARD:// 0xf000001B, /*CURRENT*/
    {
        //if(!wheel_test_event)
        {
          wheel_test_event = 1;
          set_motor_vels(0, 200, ACCELERATION_NON_EMERGENCY);
        }
        self_test_result = get_adc_chan_val(ADC_CHAN_RIGHT_MOTOR_CURRENT);
        printf("CMD_WHEEL_R_FORWARD=%d\r\n",self_test_result);		
        break;
    }
    case CMD_WHEEL_L_FORWARD:// 0xf000001C, /*CURRENT*/
    {
        //if(!wheel_test_event)
        {
          wheel_test_event = 1;
          set_motor_vels(200, 0, ACCELERATION_NON_EMERGENCY);
        }
        self_test_result = get_adc_chan_val(ADC_CHAN_LEFT_MOTOR_CURRENT);
        printf("CMD_WHEEL_L_FORWARD=%d\r\n",self_test_result);		
        break;
    }
    case CMD_WHEEL_R_REV_SPD:// 0xf000001D, /*speed*/
    {
        //if(!wheel_test_event)
        {
          wheel_test_event = 1;
          set_motor_vels(0, -200, ACCELERATION_NON_EMERGENCY);
        }
        self_test_result = 0; 
        break;
    }
    case CMD_WHEEL_L_REV_SPD:// 0xf000001E, /*speed*/
    {
        //if(!wheel_test_event)
        {
          wheel_test_event = 1;
          set_motor_vels(-200, 0, ACCELERATION_NON_EMERGENCY);
        }
        self_test_result = 0; 
        break;
    }
    case CMD_MAIN_FORWARD:   // 0xf000001f, /*CURRENT*/
    {
        if (other_motor_test == 0)
        {
		  robot_midbrush_vols_set(1);
          robot_mid_brush_adjust_set(MAIN_BRUSH_MAX_VOLTAGE);
          other_motor_test = 1;
        }
        self_test_result = get_adc_chan_val(ADC_CHAN_MAIN_BRUSH_CURRENT);
        printf("CMD_MAIN_FORWARD=%d\r\n",self_test_result);
        break;
    }
    case CMD_MAIN_REV:       // 0xf0000020, /*CURRENT*/
    {
    	  if (other_motor_test == 0)
    	  {
			robot_midbrush_vols_set(1);
          robot_mid_brush_adjust_set(MAIN_BRUSH_MAX_VOLTAGE);
          other_motor_test = 1; 
        }
        self_test_result = get_adc_chan_val(ADC_CHAN_MAIN_BRUSH_CURRENT);
        printf("CMD_MAIN_REV=%d\r\n",self_test_result);
        break;
    }
    case CMD_SIDE_FORWARD:   // 0xf0000021, /*CURRENT*/
    {
        if (other_motor_test == 0)
        {
		  robot_sidebrush_vols_set(1);
          robot_side_brush_adjust_set(SIDE_BRUSH_MAX_VOLTAGE);
          other_motor_test = 1;
        }
        self_test_result = get_adc_chan_val(ADC_CHAN_SIDE_BRUSH_CURRENT);
        printf("CMD_SIDE_FORWARD=%d\r\n",self_test_result);
        break;
    }
    case CMD_SIDE_REV:       // 0xf0000022, /*CURRENT*/
    {
    	  if (other_motor_test == 0)
    	  {
			robot_sidebrush_vols_set(1);
          robot_side_brush_adjust_set(-SIDE_BRUSH_MAX_VOLTAGE);
          other_motor_test = 1;
        }
        self_test_result = 0; 
        self_test_result = get_adc_chan_val(ADC_CHAN_SIDE_BRUSH_CURRENT);
        printf("CMD_SIDE_REV=%d\r\n",self_test_result);
        break;
    }
    case CMD_VACCUM:         // 0xf0000023, /*CURRENT*/
    {
    	  if (other_motor_test == 0)
    	  {
			robot_suction_vols_set(1);
          robot_suction_adjust_set(VACUUM_NORMAL_VOLTAGE);
          other_motor_test = 1;
          }	
        self_test_result = get_adc_chan_val(ADC_CHAN_VAC_CURENT);
        printf("CMD_VACCUM=%d\r\n",self_test_result);
        break;
    }
    case CMD_BINFULL:        // 0xf0000024, /*TURE OR FALSE*/
    {
        self_test_result = 0; 
        break;
    }
    case CMD_DEBRIS:         // 0xf0000025, /*TURE OR FALSE*/
    {
        self_test_result = 0; 
        break;
    }
    case CMD_OPEN_ALL_LEDS:       // 0xf0000026,
    {
        self_test_result = 0; 
        break;
    }
    case CMD_UI_VERSION:      // 0xf0000027,
    {

#ifdef GIT_VER_DEF
        extern const char * GIT_SUB_VER;
        printf( "%s \n",GIT_SUB_VER);
#endif
        //extern U8 g_software_version;
        //self_test_result = g_software_version ;
        break;
    }
    case CMD_WHEEL_SPEED:    // 0xf0000028,
    {
        self_test_result = 0; 
        break;
    }
    case CMD_GYRO:           // 0xf0000029,
    {
      self_test_result = gyro_is_ok();
      break;
    }
    case CMD_FLASH_SONG:     // 0xf000002a,
    {
#if 0
      if (! ui_song_playing )
      {
        ui_song_playing = 1;
        songplayer_play_index(UI_MUSIC_ERROR_NOTIFY, FALSE);
        while ( songplayer_is_playing() );
        ui_song_playing = 0;
      }
#endif
      self_test_result = 1; 
      break;
    }
    case CMD_UART:           // 0xf000002b,
    {
        self_test_result = 0; 
        break;
    }
    case CMD_LONGTEST:
    {
        self_test_result = 2;
        break;
    }
    case CMD_STASIS_TEST:
    {
        if(!cmd_stasis_test)//第一次进来需要清除
        {
          cmd_stasis_test = 1;
          robot_stasis_test_clear();
        }

        self_test_result = (robot_stasis_test_get());
        if(self_test_result > 0)
        {
          test_result_dly = timer_ms();
        }

        if(timer_ms() - test_result_dly > 1000)
        {
          robot_stasis_test_clear();
        }
        break;
    }
    case CMD_TEST_ALL:
    {
        /*
      if(!wheel_test_event)
      {
          wheel_test_event = 1;
          //start_motor_test(ALL_TEST,3000);
      }
       other_motor_test = 1;
       */
       set_motor_vels(200, 200, ACCELERATION_NON_EMERGENCY);
	   robot_suction_vols_set(1);
       robot_suction_adjust_set(VACUUM_NORMAL_VOLTAGE); 
	   robot_sidebrush_vols_set(1);
       robot_side_brush_adjust_set(SIDE_BRUSH_MAX_VOLTAGE);
	   robot_midbrush_vols_set(1);
       robot_mid_brush_adjust_set(MAIN_BRUSH_MAX_VOLTAGE);
       break;
    }

    case CMD_NULL:           // 0xf000002c
    {
        self_test_result = 0; 
        break;
    }
    case CMD_MASTER_VERSION:
    {
        self_test_result = get_main_software_version() ;
    }
    break;
    case CMD_TEST_ALL_IR:
    {
         ir_test_all_result();
    }
    break;
    case CMD_TEST_BUMP_DROP:
    {
        self_test_result = 0;
        bump_state = get_bump_state();
        self_test_result = self_test_result|(((bump_state & BUMP_FRONT_LEFT)?1:0)<<4);
        self_test_result = self_test_result|(((bump_state & BUMP_FRONT_RIGHT)?1:0)<<3);
        self_test_result = self_test_result|(((bump_state & BUMP_FRONT_CENTER)?1:0)<<2);

        drop_mask = robot_wheel_drop_mask();
        self_test_result = self_test_result|(((drop_mask & WHEEL_DROP_STATE_LEFT)?1:0)<<1);
        self_test_result = self_test_result|(((drop_mask & WHEEL_DROP_STATE_RIGHT)?1:0)<<0);
        printf("TEST_BUMP_DROP=%x\r\n",self_test_result);

    }
    break;
    case CMD_TEST_CLIFF_LIGHTTOUCH:
    {
        self_test_result = 0;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_REAR_LEFT)<<11;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_LEFT)<<10;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_FRONTLEFT)<<9;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_FRONTRIGHT)<<8;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_RIGHT)<<7;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_REAR_RIGHT)<<6;
        self_test_result = self_test_result|robot_is_near_wall(LT_LEFT)<<5;
        self_test_result = self_test_result|robot_is_near_wall(LT_CENTERLEFT)<<4;
        self_test_result = self_test_result|robot_is_near_wall(LT_FRONTLEFT)<<3;
        self_test_result = self_test_result|robot_is_near_wall(LT_FRONTRIGHT)<<2;
        self_test_result = self_test_result|robot_is_near_wall(LT_CENTERRIGHT)<<1;
        self_test_result = self_test_result|robot_is_near_wall(LT_RIGHT)<<0;
        printf("TEST_CLIFF_LIGHTTOUCH=%x\r\n",self_test_result);	
    }
    break;
    case CMD_TEST_CLIFF_DROP_STASIS:
    {
        if(!cmd_stasis_test)//第一次进来需要清除
        {
          cmd_stasis_test = 1;
          robot_stasis_test_clear();
        }

        self_test_result = 0;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_LEFT)<<0;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_FRONTLEFT)<<1;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_FRONTRIGHT)<<2;
        self_test_result = self_test_result|robot_is_near_hazard(CLIFF_RIGHT)<<3;

        drop_mask = robot_wheel_drop_mask();
        self_test_result = self_test_result|(((drop_mask & WHEEL_DROP_STATE_LEFT)?1:0)<<4);
        self_test_result = self_test_result|(((drop_mask & WHEEL_DROP_STATE_RIGHT)?1:0)<<5);

        stasis_state = ((robot_stasis_test_get())<<6);
        self_test_result = self_test_result|stasis_state;

        if(stasis_state > 0)
        {
            test_result_dly = timer_ms();
        }

        if(timer_ms() - test_result_dly > 1000)
        {
          robot_stasis_test_clear();
        }
    }
    break;
    case CMD_TEST_LIGHTTOUCH_BUMP:
    {
        self_test_result = 0;
        bump_state = get_bump_state();

        self_test_result = self_test_result|robot_is_near_wall(LT_LEFT)<<0;
        self_test_result = self_test_result|robot_is_near_wall(LT_CENTERLEFT)<<1;
        self_test_result = self_test_result|robot_is_near_wall(LT_FRONTLEFT)<<2;
        self_test_result = self_test_result|robot_is_near_wall(LT_CENTERRIGHT)<<3;
        self_test_result = self_test_result|robot_is_near_wall(LT_RIGHT)<<4;
        self_test_result = self_test_result|(((bump_state & BUMP_FRONT_LEFT)?1:0)<<5);
        self_test_result = self_test_result|(((bump_state & BUMP_FRONT_RIGHT)?1:0)<<6);
    }
    break;
    case CMD_TEST_WIRELESS:
    {
         tmp_test_result = 0;
         tmp_state = (U8)get_ir_test((IR_REMOT_POSITION_E)0);

//         if(tmp_state == VW_BEACON_BYTE)
//         {
//            tmp_test_result |= 0x10;
//         }

         if(tmp_state == RIGHT_BEACON_BYTE)
         {
            tmp_test_result |= 0x04;
         }
         if(tmp_state == BOTH_BEACONS_BYTE)
         {
            tmp_test_result |= 0x02;
         }
         if(tmp_state == LEFT_BEACON_BYTE)
         {
            tmp_test_result |= 0x01;
         }
         if(tmp_test_result > 0)
         {
             self_test_result = tmp_test_result;
             ui_test_last_ts = timer_ms();
            // printf("self_test_result=%x\r\n",self_test_result);
         }
         else
         {
             if(timer_elapsed(ui_test_last_ts) > 600)
             {
                self_test_result = 0;
             }
         }


    }
    break;
    case CMD_TEST_MOVE_REVERSE:
    {
      /*  if(!wheel_test_event)
      {
          wheel_test_event = 1;

          //start_motor_test(ALL_TEST,-3000);
      }
      other_motor_test = 1;*/
             set_motor_vels(-200, -200, ACCELERATION_NON_EMERGENCY);
	   robot_suction_vols_set(1);
       robot_suction_adjust_set(VACUUM_NORMAL_VOLTAGE);
	   robot_sidebrush_vols_set(1);
       robot_side_brush_adjust_set(SIDE_BRUSH_MAX_VOLTAGE);
	   robot_midbrush_vols_set(1);
       robot_mid_brush_adjust_set(MAIN_BRUSH_MAX_VOLTAGE);
    }
    break;
    case CMD_TEST_ALL_CHARGE_MODE:
    {
        self_test_result = 0;
        if(charging_jack())
        {
            self_test_result |= 0x40;
        }
        if(charging_dock())
        {
            self_test_result |= 0x01;
        }

    }
    break;
	
	case CMD_TEST_ALL_TOUCH_FAR://墙检强灯
           print_touch();
//          ir_send_on_off(3);
//	touch_near_flag =0;
//	set_cliff_enable(1);//关
//	set_lighttouch_enable(0);//开
//	print_touch();
////	if(robot_is_lighttouch(LT_LEFT)||robot_is_lighttouch(LT_CENTERLEFT)||robot_is_lighttouch(LT_FRONTLEFT)||\
////	  robot_is_lighttouch(LT_FRONTRIGHT)||robot_is_lighttouch(LT_CENTERRIGHT)||robot_is_lighttouch(LT_RIGHT))
//	if(lt_flag)
//	{
////	  printf("touch: l=%d cl=%d fl=%d fr=%d cr=%d r=%d\r\n", \
////			robot_is_lighttouch(LT_LEFT),robot_is_lighttouch(LT_CENTERLEFT),robot_is_lighttouch(LT_FRONTLEFT),\
////	  		robot_is_lighttouch(LT_FRONTRIGHT),robot_is_lighttouch(LT_CENTERRIGHT),robot_is_lighttouch(LT_RIGHT));
//      gpio_set_value(AM_I0_DOCK_LED,1);//蓝灯
//      gpio_set_value(AM_I0_CLEAN_LED,0);
//      gpio_set_value(AM_I0_SPOT_LED,0);		
//	}	
//	else
//	{
//	  gpio_set_value(AM_I0_CLEAN_LED,1);//绿灯	  
//      gpio_set_value(AM_I0_DOCK_LED,0);
//      gpio_set_value(AM_I0_SPOT_LED,0);		
//	}
//	  
	  
	 break; 
	case CMD_TEST_ALL_TOUCH_NEAR://墙检弱灯
//          ir_send_on_off(3);
//	 touch_near_flag =1;
//	set_cliff_enable(1);//关
//	set_lighttouch_enable(0);//开
//	print_touch();
////	if(robot_is_lighttouch(LT_LEFT)||robot_is_lighttouch(LT_CENTERLEFT)||robot_is_lighttouch(LT_FRONTLEFT)||\
////	  robot_is_lighttouch(LT_FRONTRIGHT)||robot_is_lighttouch(LT_CENTERRIGHT)||robot_is_lighttouch(LT_RIGHT)||\
////		robot_is_lighttouch(LT_CENTERLEFT_L))
//	if(lt_flag)
//	{
////	  printf("touch: l=%d cl=%d fl=%d fr=%d cr=%d r=%d\r\n", \
////			robot_is_lighttouch(LT_LEFT),robot_is_lighttouch(LT_CENTERLEFT),robot_is_lighttouch(LT_FRONTLEFT),\
////	  		robot_is_lighttouch(LT_FRONTRIGHT),robot_is_lighttouch(LT_CENTERRIGHT),robot_is_lighttouch(LT_RIGHT));	  
//      gpio_set_value(AM_I0_DOCK_LED,1);//蓝灯
//      gpio_set_value(AM_I0_CLEAN_LED,0);
//      gpio_set_value(AM_I0_SPOT_LED,0);		
//	}	
//	else
//	{
//	  gpio_set_value(AM_I0_CLEAN_LED,1);//绿灯	  
//      gpio_set_value(AM_I0_DOCK_LED,0);
//      gpio_set_value(AM_I0_SPOT_LED,0);		
//	}  
	 break; 
    case CMD_TEST_ALL_CLIFF://地检
      print_cliff();
//      ir_send_on_off(3);
//	  touch_near_flag =1;
//	set_cliff_enable(0);//开
//	set_lighttouch_enable(1);//关
//	print_cliff();
//	
//// 	if(!robot_is_cliff_test(CLIFF_LEFT)&&!robot_is_cliff_test(CLIFF_FRONTLEFT)&&!robot_is_cliff_test(CLIFF_FRONTRIGHT)&&!robot_is_cliff_test(CLIFF_RIGHT))	
////	{
////      gpio_set_value(AM_I0_DOCK_LED,1);
////      gpio_set_value(AM_I0_CLEAN_LED,0);
////      gpio_set_value(AM_I0_SPOT_LED,1);		  
////	  break;
////	}
//    if(!robot_is_cliff_test(CLIFF_LEFT)||!robot_is_cliff_test(CLIFF_FRONTLEFT)||!robot_is_cliff_test(CLIFF_FRONTRIGHT)||!robot_is_cliff_test(CLIFF_RIGHT))	
//	{
////	  printf("cliff: cl=%d fl=%d fr=%d cr=%d \r\n",\
////		robot_is_cliff_test(CLIFF_LEFT),robot_is_cliff_test(CLIFF_FRONTLEFT),robot_is_cliff_test(CLIFF_FRONTRIGHT),robot_is_cliff_test(CLIFF_RIGHT)); 	
//      gpio_set_value(AM_I0_CLEAN_LED,0);
//	  if(cliff_led_flag ==1)//cliff都大于2500
//	  {
//	  	gpio_set_value(AM_I0_SPOT_LED,1);//红灯
//		gpio_set_value(AM_I0_DOCK_LED,1);//蓝灯
//	  }	  
//	  else if(cliff_led_flag ==2)//一个cliff>2500
//	  {
//	  	gpio_set_value(AM_I0_SPOT_LED,1);//红灯
//		gpio_set_value(AM_I0_DOCK_LED,0);//蓝灯
//	  }
//	  else if(cliff_led_flag ==0)
//	  {
//		gpio_set_value(AM_I0_SPOT_LED,0);//红灯
//      	gpio_set_value(AM_I0_DOCK_LED,1);//蓝灯	  	  
//	  }
//      		
//	}	
//	else
//	{
//	  gpio_set_value(AM_I0_CLEAN_LED,1);//绿灯	  
//      gpio_set_value(AM_I0_DOCK_LED,0);
//      gpio_set_value(AM_I0_SPOT_LED,0);		
//	}
	 break;	
	 
    default:
      self_test_result = 0; break;
    }
    ui_test_num_last = cmd;
}

/****************************************************************
*Function   :  ir_test_result
*Description:  某个接收头是否接收到信号
*Input      :  U8 nIrIndex:接收头ID           
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void ir_test_result(U8 nIrIndex)
{
      U8 ir0 = (U8)get_ir_test((IR_REMOT_POSITION_E)nIrIndex) ;
      printf("ir0:%d,result:%d \r\n",ir0,self_test_result );
      if ( IS_BEACONS_BYTE(ir0) )
      {
        self_test_result = 1;
        ui_test_last_ts = timer_ms();
      }
      else
      {
          self_test_result = (timer_elapsed(ui_test_last_ts) <= 1000) ;
      }
}

/****************************************************************
*Function   :  ir_test_all_result
*Description:  接收头是否接收到信号
*Input      :  无           
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void ir_test_all_result(void)
{
    U8 i;
    U8 tmp_ir_result=0;
    U8 ir0;
    self_test_result = 0;
    for(i=0;i<IR_DECODER_MAX;i++)
    {
        ir0 = (U8)get_ir_test((IR_REMOT_POSITION_E)i) ;
        printf("ir0:%d,result:%d \r\n",ir0,self_test_result );
        if ( IS_BEACONS_BYTE(ir0) )
        {
            tmp_ir_result = 1;
            ui_test_last_ts2[i] = timer_ms();
        }
        else
        {
            tmp_ir_result = (timer_elapsed(ui_test_last_ts2[i]) <= 500);
            //self_test_result = (timer_elapsed(ui_test_last_ts) <= 1000) ;
        }
        self_test_result = ((self_test_result<<1)|tmp_ir_result);

    }
   // printf("ir_test_all_result=%x\r\n",self_test_result);

}

/****************************************************************
*Function   :  get_test_result
*Description:  获取测试结果
*Input      :  无           
*Output     :  无
*Return     :  1:测试通过 0:测试不通过
*Others     :  
******************************************************************/
int get_test_result(void)
{
  S16 left_m=0,left_s=0,right_m = 0,right_s=0;
  switch(ui_test_num_last)
  {
  	case CMD_BAT:
  		if (self_test_result >= CHARGING_CUTOFF_VOLTAGE && self_test_result <= CHARGING_MAXVOL)
  		{
  		  self_test_result = 1;
  		}
  		else
  		{
  		  self_test_result = 0;
  		}  
  		break;
  		
  	case CMD_DOCK_CURRENT:
  		if (self_test_result > 0 && self_test_result <= ADAPTER_OUTPUT_CURRENT)
  		{
  		  self_test_result = 1;
  		}
  		else
  		{
  		  self_test_result = 0;
  		} 
  	  break;
  	
  	case CMD_WHEEL_L_FORWARD:
  	case CMD_WHEEL_R_FORWARD:
  		if (self_test_result > 0 && self_test_result <= WHEEL_TWINE_CURRENT)
  		{
  		  self_test_result = 1;
  		}
  		else
  		{
  		  self_test_result = 0;
  		} 
  	  break;
  	  
  	case CMD_MAIN_FORWARD:  
  	case CMD_MAIN_REV:
  		if (self_test_result > 0 && self_test_result <= MAIN_BRUSH_TWINE_CURRENT)
  		{
  		  self_test_result = 1;
  		}
  		else
  		{
  		  self_test_result = 0;
  		} 
  	  break;
  	  
  	case CMD_SIDE_FORWARD:  
  	case CMD_SIDE_REV:
  		if (self_test_result > 0 && self_test_result <= SIDE_BRUSH_TWINE_CURRENT)
  		{
  		  self_test_result = 1;
  		}
  		else
  		{
  		  self_test_result = 0;
  		} 
  	  break;
  	case CMD_VACCUM:
	  	 if(self_test_result > VACUUM_STALL_CURRENT)
		  {
				vac_current_over_count++;
				if (vac_current_over_count > 300) 
				{
					self_test_result =0;
				}
				else
				{
					self_test_result =1;
				}			
		  }
		 else 
		 {
			vac_current_over_count = 0;
		  }	 
  	  break;  	      	
    case CMD_WHEEL_R_REV_SPD:// 0xf000001D, /*speed*/
    {
      get_motor_speeds(&left_m,&right_m);
      get_commanded_speeds(&left_s,&right_s);
      printf("left_m:%d,right_m:%d  ",left_m,right_m);
      printf("left_s:%d,right_s:%d\r\n",left_s,right_s);      
      printf("CMD_WHEEL_R_REV_SPD:%d\r\n",abs(right_m - right_s));
      self_test_result = abs(right_m - right_s)*100/abs(right_s) > 5 ? 0:1;
    }
    break;
    case CMD_WHEEL_L_REV_SPD:// 0xf000001E, /*speed*/
    {
      get_motor_speeds(&left_m,&right_m);
      get_commanded_speeds(&left_s,&right_s);
      printf("left_m:%d,right_m:%d  ",left_m,right_m);
      printf("left_s:%d,right_s:%d\r\n",left_s,right_s);
      printf("CMD_WHEEL_L_REV_SPD:%d\r\n",abs(left_m - left_s));
      self_test_result = abs(left_m - left_s)*100/abs(left_s) > 5 ? 0:1;      
    }
    break;
  case CMD_REMOTE:         // 0xf0000012, /*TURE OR FALSE*/
    {
      ir_test_result(0);
    }
         break;
  case CMD_BUOY_L:
    ir_test_result(1);
    break;
  case CMD_BUOY_R:
    ir_test_result(2);
    break;
  case CMD_BUOY_MIDDLE:
     ir_test_result(3);
    break;
  }
  return self_test_result;
}

//#ifdef PC_AUTO_TEST
#if 1

#define AUTO_TEST_BUFFER_SIZE 80
extern volatile s16 signal_delta[IR_SENSOR_NUM];

extern volatile uint16_t main_brush_current;
extern volatile uint16_t side_brush_current;
extern volatile uint16_t left_motor_current;
extern volatile uint16_t right_motor_current;
extern volatile uint16_t battery_line_current;
extern s32 signal_average_on[];
extern s32 signal_average_off[];

static const uint8_t modeAndVersion[20] = "XR510F VER 0107";


extern void set_debug_uart_tx_fifo_empty_irq(void);
extern u8 any_key(void);
extern void ir_send_task_init(void);
extern U32 get_key_val(void);
extern U16 charging_jack(void);
extern U16 charging_dock(void);
extern u16 get_vaccum_current(void);
extern s16 convert_to_current( s16 c_adc); 
static volatile uint16_t auto_test_time_count = 0;
static volatile uint16_t auto_test_receive_data = 0;
static uint8_t auto_test_buf[AUTO_TEST_BUFFER_SIZE];
static volatile uint8_t auto_test_sending_index = 0;
static uint8_t auto_test_data_size = 0;
static uint32_t last_key_val=0;
static U32 pc_test_dly = 0;
//static U8  write_nvram_flag = 0;
static uint8_t key_press_cnt=0;

int auto_test_tx_handle(void)
{
  int retval = -1;
  if (auto_test_sending_index < auto_test_data_size) {
      retval = auto_test_buf[auto_test_sending_index];
      auto_test_sending_index++;     
  } else {
      auto_test_data_size = 0;
      auto_test_sending_index = 0;
      auto_test_receive_data = 0;
  }
   return retval;
}

int auto_test_rx_handle(int data)
{
   auto_test_receive_data = ((auto_test_receive_data<<8) | (data&0x0FF));
   if (auto_test_receive_data == 0xAAAA) {
       auto_test_time_count = 3000;
   }   
   return 0;
}


  

int send_auto_test_data(void)
{
  uint8_t i;

  WheelDropState drop_mask ;
  uint16_t checksum;
  uint16_t temp_value;
  uint32_t tmp_value;
  uint32_t temp_value2[2];
  uint16_t temp_current;
  uint8_t tmp_state;
  uint32_t key_val = KEY_NULL;
     float a = TICKS_PER_MM;
  int16_t left_speed=0, right_speed = 0;
   get_motor_speeds(&left_speed,&right_speed);
 // robot_get_measured_speeds(&left_speed, &right_speed);
 
  //tmp_state = get_ir_test((IR_REMOT_POSITION_E)0);

  
  memset(auto_test_buf, 0, sizeof(auto_test_buf));

  if((timer_ms() - pc_test_dly > 2500)&&(ui_test_cmd != 0))
  {
     if(ui_test_cmd == CMD_TEST_ALL)
     {
        ui_test_cmd = CMD_TEST_MOVE_REVERSE;
     }
     else if(ui_test_cmd == CMD_TEST_MOVE_REVERSE)
     {
        ui_test_cmd = CMD_TEST_ALL;
     }
     pc_test_dly = timer_ms();
  }
  auto_test_buf[0] = 0xAA;
  //左边地检
  temp_value = signal_delta[CLIFF_LEFT];
  auto_test_buf[1] = (uint8_t)(temp_value>>8);
  auto_test_buf[2] = (uint8_t)(temp_value>>0);
  //左中地检
  temp_value = signal_delta[CLIFF_FRONTLEFT];
  auto_test_buf[3] = (uint8_t)(temp_value>>8);
  auto_test_buf[4] = (uint8_t)(temp_value>>0);
  //右中地检
  temp_value = signal_delta[CLIFF_FRONTRIGHT];
  auto_test_buf[5] = (uint8_t)(temp_value>>8);
  auto_test_buf[6] = (uint8_t)(temp_value>>0);
  //右边地检
  temp_value = signal_delta[CLIFF_RIGHT];
  auto_test_buf[7] = (uint8_t)(temp_value>>8);
  auto_test_buf[8] = (uint8_t)(temp_value>>0);
    
  //左边墙检
  temp_value = signal_delta[LT_LEFT];
  auto_test_buf[9] = (uint8_t)(temp_value>>8);
  auto_test_buf[10] = (uint8_t)(temp_value>>0);
  
  //左中墙检
  temp_value = signal_delta[LT_CENTERLEFT];
  auto_test_buf[11] = (uint8_t)(temp_value>>8);
  auto_test_buf[12] = (uint8_t)(temp_value>>0);
  
  //中间墙检
  temp_value = signal_delta[LT_FRONTLEFT];
  auto_test_buf[13] = (uint8_t)(temp_value>>8);
  auto_test_buf[14] = (uint8_t)(temp_value>>0);  
  //右中墙检
  temp_value = signal_delta[LT_CENTERRIGHT];
  auto_test_buf[15] = (uint8_t)(temp_value>>8);
  auto_test_buf[16] = (uint8_t)(temp_value>>0);  
  //右边墙检
  temp_value = signal_delta[LT_RIGHT];
  auto_test_buf[17] = (uint8_t)(temp_value>>8);
  auto_test_buf[18] = (uint8_t)(temp_value>>0); 
  
  //红外发射 4byte
  auto_test_buf[19] = get_ir_test((IR_REMOT_POSITION_E)0);
  auto_test_buf[20] = get_ir_test((IR_REMOT_POSITION_E)1);
  auto_test_buf[21] = get_ir_test((IR_REMOT_POSITION_E)2);
  auto_test_buf[22] = get_ir_test((IR_REMOT_POSITION_E)3);
  
  //左轮电流
  tmp_value = get_adc_chan_val(ADC_CHAN_LEFT_MOTOR_CURRENT);
  //tmp_value = tmp_value*926/1862/10;
  auto_test_buf[23] = (uint8_t)(tmp_value>>8);
  auto_test_buf[24] = (uint8_t)(tmp_value>>0); 
  
  //左轮速度
  left_speed = abs(left_speed)/(int16_t)a;
  auto_test_buf[25] = (uint8_t)(left_speed>>8);
  auto_test_buf[26] = (uint8_t)(left_speed>>0);
  
  //右轮电流
  tmp_value = get_adc_chan_val(ADC_CHAN_RIGHT_MOTOR_CURRENT);
  //tmp_value = tmp_value*926/1862/10;
  auto_test_buf[27] = (uint8_t)(tmp_value>>8);
  auto_test_buf[28] = (uint8_t)(tmp_value>>0); 
  
  //右轮速度
  right_speed = abs(right_speed)/(int16_t)a;
  auto_test_buf[29] = (uint8_t)(right_speed>>8);
  auto_test_buf[30] = (uint8_t)(right_speed>>0);
  
  //中扫电流
  tmp_value = get_adc_chan_val(ADC_CHAN_MAIN_BRUSH_CURRENT);
  //tmp_value = tmp_value*926/1862/10;
  auto_test_buf[31] = (uint8_t)(tmp_value>>8);
  auto_test_buf[32] = (uint8_t)(tmp_value>>0);
  
  //灰尘盒电流
  tmp_value = get_adc_chan_val(ADC_CHAN_VAC_CURENT);
  //tmp_value = tmp_value*1000/4096*33/10;
  auto_test_buf[33] = (uint8_t)(tmp_value>>8);
  auto_test_buf[34] = (uint8_t)(tmp_value>>0);
  
  //充电电流
  if(charging_state_get() > 0)
  {
      //temp_value =  hal_analog_input_get(HAL_ADC_BATTERY_CURRENT);
      temp_current = charge_current_get();
      //printf("temp_current=%d\r\n",temp_current);
      if(temp_current < 0)
      {
         temp_current = 0;
      }
  }
  else
  {
      temp_current = 0;
  }
  
  auto_test_buf[35] = (uint8_t)(temp_current>>8);
  auto_test_buf[36] = (uint8_t)(temp_current>>0);  
  
  //电池电压
  //temp_value = hal_analog_input_get(HAL_ADC_BATTERY_VOLTAGE);
  //校准电压 
  temp_value2[0] = battery_voltage_average();
  if((temp_value2[0] >= 3649/*3775*/) && (temp_value2[0] < 3954/*4090*/))
  {
     vram_read(ADJ_BAT_VOL_OFFSET, temp_value2, 2);
     if((write_nvram_flag == 0)||(temp_value2[0] != 0x55aa))
     {
        write_nvram_flag = 1;
        temp_value2[0] = 0x55aa;
       // vram_write(0, &tmp_value, 1);
        temp_value2[1] = battery_voltage_average();
        vram_write(ADJ_BAT_VOL_OFFSET, temp_value2, 2);
        songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);

     }
     else
     {
        songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
     }                           
  }
  
  tmp_value = battery_voltage_average();
  tmp_value = ((tmp_value*330000/174)>>12)*10;
  auto_test_buf[37] = (uint8_t)(tmp_value>>8);
  auto_test_buf[38] = (uint8_t)(tmp_value>>0);
  
  //电池温度 1byte
  //temp_value = hal_analog_input_get(HAL_ADC_BATTERY_THERMISTOR);
  //auto_test_buf[39] = (uint8_t)(temp_value>>8);
   auto_test_buf[39] = 55;
  //机器型号 1byte
  auto_test_buf[40] = 0x01;
  
  //机器软件版本 2byte
  //temp_value = g_software_version;
  temp_value = 817;
  auto_test_buf[41] = (uint8_t)(temp_value>>8);
  auto_test_buf[42] = (uint8_t)(temp_value>>0);
  
  //错误代码 1byte
  //auto_test_buf[43] = ui_error_number;
  auto_test_buf[43] = 0;
  /*
  //扩展内容 5byte
  auto_test_buf[44] = 0xAA;
  auto_test_buf[45] = 0xAA;
  auto_test_buf[46] = 0xAA;
  auto_test_buf[47] = 0xAA;
  auto_test_buf[48] = 0xAA;
  */
  auto_test_buf[44] = 0x00;
  //左轮方向
  {
    if( ui_test_cmd == CMD_TEST_MOVE_REVERSE)
    {
        auto_test_buf[44] |= 0x80;
    }
  }
  //右轮方向
  {
    if( ui_test_cmd == CMD_TEST_MOVE_REVERSE)
    {
        auto_test_buf[44] |= 0x40;
    }
  }  
  //中扫方向
  {
    auto_test_buf[44] |= 0x00;
  }  
  //左边地检
  {
    if(!robot_is_near_hazard(CLIFF_LEFT))
    {
       auto_test_buf[44] |= 0x10;
    }

  }
  //左中地检
  {
    if(!robot_is_near_hazard(CLIFF_FRONTLEFT))
    {
        auto_test_buf[44] |= 0x08;
    }

  }  
  //右中地检
  {
    if(!robot_is_near_hazard(CLIFF_FRONTRIGHT))
    {
        auto_test_buf[44] |= 0x04;
    }

  }  
  //右边地检
  {
    if(!robot_is_near_hazard(CLIFF_RIGHT))
    {
        auto_test_buf[44] |= 0x02;
    }
  }
  //左边按键
  {
    
   // key_val = get_key_val();
    if(any_key())
    {
        if(pc_test_dly == 0)
        {
            
            pc_test_dly = timer_ms();
        }
        if((timer_ms() - pc_test_dly)> 1000)
        {
            if(++key_press_cnt >= 4)
            {
                if(ui_test_cmd == 0)
                {
                    ui_test_cmd = CMD_TEST_ALL;
                    pc_test_dly = timer_ms();
                
                }
            }
        }
        else
        {
            //if(key_val > 0)
              //  key_press_cnt = 0;
        }
    }

    //last_key_val = key_val;
   // if(key_val & KEY_CLEAN) 
   if(any_key())
    {
        auto_test_buf[44] |= 0x01;
    }
    
    
  }  
  auto_test_buf[45] = 0x00;
  //中间按键
  {
    if(key_val & KEY_DOCK)
    {
        auto_test_buf[45] |= 0x80;
    }
  }  
  //右边按键
  {
    if(key_val & KEY_SPOT)
	{
        auto_test_buf[45] |= 0x40;
    }
  }
  //左边墙检
  {
    if(robot_is_near_wall(LT_LEFT))
    {
        auto_test_buf[45] |= 0x20;
    }
  }
  //左中墙检
  {
    if(robot_is_near_wall(LT_CENTERLEFT))
    {
        auto_test_buf[45] |= 0x10;
    }
  }
  //中间墙检
  {
    if(robot_is_near_wall(LT_FRONTLEFT))
    {
        auto_test_buf[45] |= 0x08;
    }
  }
  //右中墙检
  {
    if(robot_is_near_wall(LT_CENTERRIGHT))
    {
        auto_test_buf[45] |= 0x04;
    }
  }
  //右边墙检
  {
    if(robot_is_near_wall(LT_RIGHT))
    {
        auto_test_buf[45] |= 0x02;
    }
  }
  //万向轮
  {
        if(!cmd_stasis_test)//第一次进来需要清除
        {
          cmd_stasis_test = 1;
          robot_stasis_test_clear();
        }
        
        self_test_result = (robot_stasis_test_get()); 
        if(self_test_result > 0)
        {
            
          test_result_dly = timer_ms();
        }
        
        if(timer_ms() - test_result_dly > 1000)
        {
          robot_stasis_test_clear();
        }
        if(self_test_result > 0)
        {
            auto_test_buf[45] |= 0x01;
        }
  }

  auto_test_buf[46] = 0x00;
  //左边碰撞
  {
    bump_state = get_bump_state();
    if(bump_state == BUMP_FRONT_LEFT)
    {
        auto_test_buf[46] |= 0x80;
    }
  }  
  //右边碰撞
  {
    if(bump_state == BUMP_FRONT_RIGHT)
    {
        auto_test_buf[46] |= 0x40;
    }
  }
  
  //左边红外
  {
   
    {
       auto_test_buf[46] |= 0x00;
    }
  }
  
  //前面红外
  {
    
    if(tmp_state > 0)
    {
        auto_test_buf[46] |= 0x10;
    }
  }
  //顶上红外
  {
    if(tmp_state > 0)
    {
        auto_test_buf[46] |= 0x08;
    }
  }
  //右边红外
  {
    auto_test_buf[46] |= 0x00;
  }  
  //左边离地
  {
    drop_mask = robot_wheel_drop_mask();
    if(drop_mask & WHEEL_DROP_STATE_LEFT)
    {
        auto_test_buf[46] |= 0x02;
    }
  }
  //右边离地
  {
    if(drop_mask & WHEEL_DROP_STATE_RIGHT)
    {
        auto_test_buf[46] |= 0x01;
    }
  }
  
  //边扫电流
  auto_test_buf[47] = 0x00;
  {
    //if(side_brush_current>10)
      //  printf("side_brush_current=%d\r\n",side_brush_current);
    if(side_brush_current >= SIDE_BRUSH_TWINE_CURRENT)
    {
        
        auto_test_buf[47] |= 0x08;
    }
   //if(main_brush_current>10)
     //   printf("main_brush_current=%d\r\n",main_brush_current);
   if(main_brush_current >= MAIN_BRUSH_TWINE_CURRENT)
    {
        auto_test_buf[47] |= 0x04;
    }
 
    
  }
 //  auto_test_buf[47] |= 0x04;
  //座充
  {
    if(charging_dock())
    {
        auto_test_buf[47] |= 0x02;
    }
  }
  //直充
  {
    if(charging_jack())
    {
        auto_test_buf[47] |= 0x01;
    }
  }
  
  checksum = 0;
  for (i=0; i<48; i++) {
    checksum += auto_test_buf[i];
  }
  auto_test_buf[48] = (uint8_t)checksum;
  
  auto_test_buf[49] = 'x';
  auto_test_buf[50] = 'r';
  auto_test_buf[51] = 'o';
  auto_test_buf[52] = 'b';
  auto_test_buf[53] = 'o';
  auto_test_buf[54] = 't';
  
  for (i=0; i<15; i++) {
    auto_test_buf[55+i] = modeAndVersion[i];
    if (modeAndVersion[i] == '\0') {
      break;
    }
  }
  auto_test_buf[55+i] = '\0';

  tmp_value = side_brush_current;
  //tmp_value = tmp_value*926/1862/10;
  auto_test_buf[55+i+1] = (uint8_t)(tmp_value>>8);
  auto_test_buf[55+i+2] = (uint8_t)(tmp_value>>0);
  auto_test_buf[55+i+3] = (uint8_t)get_ir_test((IR_REMOT_POSITION_E)4);
  auto_test_buf[55+i+4] = (uint8_t)get_ir_test((IR_REMOT_POSITION_E)5);
  i+=4;
  auto_test_data_size = 55+i+1;  
  auto_test_sending_index = 0;
  set_debug_uart_tx_fifo_empty_irq();
  return 0;
}
#endif


/****************************************************************
*Function   :  set_adjust_adc_value_to_system
*Description:  回写电压校准的参数到系统
*Input      :  无           
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void set_adjust_adc_value_to_system(void)
{
    uint32_t tmp_value[2];

    vram_read(ADJ_BAT_VOL_OFFSET, tmp_value, 2);
    if(tmp_value[0] == 0x55aa)
    {
        //vram_read(2, &tmp_value, 1);
        printf("set adc value:%d\r\n",tmp_value[1]);
        robot_set_adc_adjust((uint16_t)tmp_value[1]);
    }
    
}

/****************************************************************
*Function   :  self_test_routine
*Description:  处理测试模式
*Input      :  无           
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void self_test_routine(void)
{
  if (ui_test_cmd != CMD_TEST_WAIT)
  {
    handle_test_cmd(ui_test_cmd);
  }
  /*
  if (auto_test_time_count == 0) {
       debug_uart_register_tx_handle(auto_test_tx_handle);
       debug_uart_register_rx_handle(auto_test_rx_handle);
    } else if (auto_test_time_count == 300){
       debug_uart_reset();
    }
    */
   if (auto_test_time_count < 300) {
      auto_test_time_count++;
    }

   if (auto_test_receive_data == 0xAAAA) 
   {
  
        if((pc_test_enableq == 0)&&(pc_test_dly == 0))
        {
            //write_nvram_flag = 0;
            pc_test_dly = timer_ms();            
        }
        if((timer_ms() - pc_test_dly > 100)&&(pc_test_enableq == 0))
        {

            robot_sensor_gather_start(1);
            last_key_val=0;
            pc_test_dly = 0;
            key_press_cnt=0;
            pc_test_enableq = 1;
			pc_test_flag = 1;
            set_ui_state(UI_TEST);

        }
        if ((auto_test_sending_index >= auto_test_data_size)) {
            send_auto_test_data();
        }
    }
    
}

U8 get_pc_test_flag(void)
{
    return pc_test_enableq;
}

