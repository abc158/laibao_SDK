//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "ui-config.h"
#include "am_config_macro.h"
#include "syscall_api.h"

static const ui_local_config_t ui_local_config_default =
{
  .cliff_threshold = {
    .rear_left_on = CLIFF_REAR_LEFT_THRESHOLD_ON,
    .rear_left_off = CLIFF_REAR_LEFT_THRESHOLD_OFF,
    .rear_right_on = CLIFF_REAR_RIGHT_THRESHOLD_ON,
    .rear_right_off = CLIFF_REAR_RIGHT_THRESHOLD_OFF,
    .left_on = CLIFF_LEFT_THRESHOLD_ON,
    .left_off = CLIFF_LEFT_THRESHOLD_OFF,
    .right_on = CLIFF_RIGHT_THRESHOLD_ON,
    .right_off = CLIFF_RIGHT_THRESHOLD_OFF,
    .front_left_on = CLIFF_FRONT_LEFT_THRESHOLD_ON,
    .front_left_off = CLIFF_FRONT_LEFT_THRESHOLD_OFF,
    .front_right_on = CLIFF_FRONT_RIGHT_THRESHOLD_ON,
    .front_right_off = CLIFF_FRONT_RIGHT_THRESHOLD_OFF,
    .changed = 0,
  },
  .lighttouch_threshold = 
  {
    .center_left_on = LT_CENTERLEFT_THRESHOLD_ON,
    .center_left_off = LT_CENTERLEFT_THRESHOLD_OFF,
    .center_right_on = LT_CENTERRIGHT_THRESHOLD_ON,
    .center_right_off = LT_CENTERRIGHT_THRESHOLD_OFF,
    .left_on = LT_LEFT_THRESHOLD_ON,
    .left_off = LT_LEFT_THRESHOLD_OFF,
    .right_on = LT_RIGHT_THRESHOLD_ON,
    .right_off = LT_RIGHT_THRESHOLD_OFF,
    .front_left_on = LT_FRONT_LEFT_THRESHOLD_ON,
    .front_left_off = LT_FRONT_LEFT_THRESHOLD_OFF,
    .front_right_on = LT_FRONT_RIGHT_THRESHOLD_ON,
    .front_right_off = LT_FRONT_RIGHT_THRESHOLD_OFF,
    .changed = 0,
  },
  .remote_ir_codes =
  {
    .dock = IR_DOCK,
    .spot = IR_SPOT,
    .clean = IR_CLEAN,
    .power = IR_POWER,
    .left = IR_LEFT,
    .right = IR_RIGHT,
    .forward = IR_FORWARD,
    .schedule = IR_SCHEDULE,
    .clock = IR_CLOCK,
    .driver_stop = IR_DRIVERSTOP,
    .grid = IR_GRID,
    .backward = IR_BACKWARD,
    .wall_follow = IR_WALLFOLLOW,
    .dirt = IR_DIRT,
    .clean_drive = IR_CLEAN_DRIVE,
    .random = IR_RANDOM,
    .changed = 0,
  },
  .dock_ir_codes = 
  {
    .force_field = DOCK_FORCE_FIELD,
    .buoy_red = DOCK_BUOY_RED,
    .buoy_green = DOCK_BUOY_GREEN,
    .buoy_both = DOCK_BUOY_BOTH, 
    .changed = 0,
  },  
};

static const ui_config_t ui_config_default =  
{
  .power_config =
  {
    .battery_charge_voltage_max_adc = BAT_VOLTAGE_CHARGE_MAX,
    .main_brush_max_voltage_adc = MAIN_BRUSH_MAX_VOLTAGE,
    .side_brush_max_voltage_adc = SIDE_BRUSH_MAX_VOLTAGE,
    .vacuum_max_voltage_adc = VACUUM_NORMAL_VOLTAGE,
    .changed = 0,
  },
  .hard_structure =
  {
    .wheel_dia = WHEEL_DIA,
    .wheel_left_right_distance = WHEEL_LEFT_RIGHT_DISTANCE,
    .wheel_code_ratio = WHEEL_CODE_RATIO,
    .pad_width = PAD_WIDTH,
    .crash_cond = CRASH_COND,
    .p_value = P_VALUE,
    .i_value = I_VALUE,
    .d_value = D_VALUE,
    .hardware_version = HARDWARE_VERSION,
    .changed = 0,
  },
  .speed_config =
  {
    .clean_speed_max = CLEAN_SPEED_MAX,
    .crash_speed = CLEAN_SPEED_MAX,
    .dock_speed_nomal = CLEAN_SPEED_MAX,
    .dock_speed_low = CLEAN_SPEED_MAX,
    .wallfollow_speed = WALLFOLLOW_SPEED,
    .changed = 0,
  },	
  .clean_motor_config = 
  {
    .clean_fun.bits.main_brush_clean = 1,
    .clean_fun.bits.main_brush_dock = 0,
    .clean_fun.bits.side_brush_clean = 1,     	     
    .clean_fun.bits.side_brush_dock = 1,     	
    .clean_fun.bits.vaccum_clean = 1,
    .clean_fun.bits.vaccum_dock = 0,
    .clean_fun.bits.water_box = 0,
    .clean_fun.bits.resume    = 0,
  },
  
  .wf_info=
  {
    .am_ir_wf_min_val      = 60,//�ر���ǽ����Ĳ�������
    .changed = 0,
  },
  
  .charge_info=
  {
    .native_charging_en = 1, //Ĭ�ϴ�
    .bat_type = BAT_TYPE,
    .adapter_output_vol = ADAPTER_OUTPUT_VOL,
    .adapter_output_current = ADAPTER_OUTPUT_CURRENT,
    .changed = 0,
  },  

  .play_info = 
  {
    .pa_gain  = PA_GAIN,
  },
  
  .funtion_info =
  {
    .clean_fun.bits.motor_driver = 2,//8801
    .clean_fun.bits.wheel_left_dir = 1,
    .clean_fun.bits.wheel_right_dir= 1,
    .clean_fun.bits.wheepdrop    = 1,//has wheep drop test or not?
    .clean_fun.bits.stasis       = 0,//has stasis wheel
    .clean_fun.bits.lt_type      = 0,//�����Զ������lt�Ļ���д 1
    .clean_fun.bits.rear_cliff   = 0,//���ģ�������cliff���� 1 
    .clean_fun.bits.ltBumpNoBack = 1,//lt bump�󣬻����Ƿ��Ⱥ�����ת�䡣 һ��������Ӳ���ģ�ߵ��м䣬�����˴򿪡�
    .clean_fun.bits.gyro_dir     = 0,//�����Ƿ���
    .clean_fun.bits.wall_follow_dir = 0, //0: left and right, 1:left, 2:right
  }
  
};

void ui_config_init(void)
{
  ;
}

ui_config_t* get_ui_config(void)
{
  return (ui_config_t*)&ui_config_default;
}

ui_local_config_t* get_local_ui_config(void)
{
  return (ui_local_config_t*)&ui_local_config_default;
}
