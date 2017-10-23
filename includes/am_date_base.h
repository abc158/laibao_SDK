#ifndef __DATA_BASE_H__
#define __DATA_BASE_H__

#include "am_type.h"

#pragma pack(1) // 1 bytes����

typedef enum
{
 ROBOT_STATE_OFF  = 0,
 ROBOT_STATE_WAITING =1,
 ROBOT_STATE_WALLFOLLOW=2, 
 ROBOT_STATE_CLEANING_ROOM =3,
 ROBOT_STATE_DOCK    =4,
 ROBOT_STATE_SPOTTING =5,
 ROBOT_STATE_REMOTE_DRIVE= 6,
 ROBOT_STATE_DEAD =7,
 ROBOT_STATE_TESTING= 8,
 ROBOT_STATE_CHARGING= 10,
 ROBOT_STATE_CHARGING_ERROR= 11,
 ROBOT_STATE_SLEEP=12,
 ROBOT_STATE_PAUSE =14,
}ROBOT_STATE_E;

/*charge_state*/
//#define CHARGING_STATE_OFF             0
//#define CHARGING_STATE_ON              5
//#define CHARGING_STATE_PURLING         7
//#define CHARGING_STATE_PURLING_STAGE2  8

#define IS_RUNNING_MODE(x) ( (x < ROBOT_STATE_DEAD ) & (x>ROBOT_STATE_WAITING) ) 
#define IS_CLEANNING_MODE(x) ( (x < ROBOT_STATE_REMOTE_DRIVE ) & (x>ROBOT_STATE_WAITING) ) 

/////////////////////////��Դ��Ϣ/////////////////////////////
/*  
 * power info  for bat
 */
typedef struct
{
    U16 bat_capacity_max;        /*�������*/	
    U16 bat_voltage_max;         /*������������ѹ*/
    U16 bat_voltage_min;         /*����������С��ѹ*/  
    U16 bat_voltage_dead;        /*��ֹ������ѹ*/       
    U16 bat_voltage_charge_max;  /*����ֹ����ѹֵ*/
    U16 bat_temperature_max;     /*���й���������¶�ֵ*/
    U16 wheel_crash_current;     /*�ж���ײ�ϰ����������ӵ���С����*/  
    U16 wheel_twine_current;     /*�ж������ֲ�����С����*/  
    U16 main_brush_twine_current;/*�ж���ˢ������С����*/  
    U16 side_brush_twine_current;/*�жϱ�ˢ�ֲ�����С����*/ 
    U16 main_brush_voltage;  /*��ˢ���������ѹ*/
    U16 side_brush_voltage;  /*��ˢ���������ѹ*/
    U16 vacuum_voltage;      /*��յ��������ѹ*/
    U16 main_brush_reverse_voltage;/*��ˢ��ת��ѹ*/
    U16 side_brush_reverse_voltage;/*��ˢ��ת��ѹ*/
    U16 bat_voltage_adjust;       /*adc У׼ֵ*/
    U16 rev1;
    U16 rev2;
    U16 rev3;
    U16 rev4;
    U16 rev5;
    U16 rev6;    
}power_info;
/////////////////////sensor��Ϣ///////////////////
/*  
 * all sensor info 
 */
typedef struct
{
    U8  cliff_en; /*�Ƿ�ʹ��cliff���*/
    U8  lt_en;    /*�Ƿ�ʹ��lt���*/
    U8  drop_en;  /*�Ƿ�ʹ�����ӵ�����*/
    U8  bump_en;
    U8  rev1;
    U8  rev2;
}sensor_info;
//////////////////////Ӳ���ṹ��Ϣ//////////////////////
/*  
 * hard structure info
 */
typedef struct
{
  U32 wheel_code_ratio;         /*���̱���*/	  
  U16 wheel_dia;                /*�������ӵ�ֱ��*/
  U16 wheel_left_right_distance;/*����֮��ľ���*/
  U16 pad_width;                /*��ˢ��ɨ����Ч����*/
  U16 rev1;
  U16 rev2;
  U16 rev3;
  U16 rev4;
  U16 rev5;
  U16 rev6;  
}hard_structure_info;
//////////////////////��Ϊ��Ϣ/////////////////////////
/*������Ϊ���ٶ�*/
typedef struct
{
  U16 clean_speed_max;  /*��ɨ������ٶ�*/
  U16 crash_speed;      /*�����ϰ��ｵ���ٶ�*/
  U16 dock_speed_nomal; /*�����ٶ�*/
  U16 dock_speed_low;   /*���������ٶ�*/
  U16 rev1;
  U16 rev2;
  U16 rev3;
  U16 rev4; 
}act_speed_info;

/*������Ϊ�Ĺ���*/
typedef struct
{
  union{
    struct{	
      U16 main_brush_clean :1; /*����ɨ��ʱ�Ƿ�ر���ˢ, ON:open, OFF:close*/
      U16 main_brush_dock  :1; /*��������ʱ�Ƿ�ر���ˢ, ON:open, OFF:close*/	  
      U16 side_brush_clean :1; /*����ɨ��ʱ�Ƿ�رձ�ˢ, ON:open, OFF:close*/	  		  
      U16 side_brush_dock  :1; /*��������ʱ�Ƿ�رձ�ˢ, ON:open, OFF:close*/
      U16 vaccum_clean     :1; /*����ɨ��ʱ�Ƿ�ر����, ON:open, OFF:close*/
      U16 vaccum_dock      :1; /*��������ʱ�Ƿ�ر����, ON:open, OFF:close*/	  
      U16 water_box        :1; /*��������ʱ�Ƿ�ر����, ON:open, OFF:close*/
      U16 resume           :1; /*��������֧�ֶϵ���ɨ  ��ON:open, OFF:close*/
      U16 rev              :8;
    }BITS;
    U16 BYTE;
  }ACT_FUN;
}act_fun_info;

/*������Ϊ�����ݽṹ*/
typedef struct
{
    act_speed_info act_speed_info_t;
    act_fun_info   act_fun_info_t;
}act_info;
////////////////////������Ϣ////////////////////////
/*  
 * remote info, the value use enum IR_E
 */
typedef struct
{
    U8 dock;    /*������ֵ*/
    U8 clean;   /*��ɨ��ֵ*/  
    U8 spot;    /*�ֲ���ֵ*/  
    U8 power;   /*�ػ���ֵ*/  
    U8 left;    /*������ֵ*/     
    U8 right;   /*������ֵ*/  
    U8 forward; /*ǰ����ֵ*/  
    U8 drivestop; /*ֹͣ��ֵ*/  
    U8 backward;  /*�����ֵ*/
    U8 clock;
    U8 schedule;
    U8 wall_follow;
    U8 dirt;
    U8 random;
    U8 rev1;
    U8 rev2;
    U8 rev3;  
    U8 rev4;    
}remote_info;
/*  
 * dock ir info, the value use enum DOCK_E
 */
typedef struct
{
    U8  force_field; /*������ײ��*/
    U8  buoy_red;    /*��Ŀ*/
    U8  buoy_green;  /*��Ŀ*/
    U8  buoy_both;   /*��׼˫Ŀ*/
    U8  rev1;
    U8  rev2;    
}dock_ir_info;
/*  
 * all ir info
 */
typedef struct
{
    remote_info    remote_info_t;
    dock_ir_info   dock_ir_info_t;
}ir_info;
///////////////////////end config info/////////////////////
////////////////////״̬��Ϣ////////////////////
/*  
 * robot sys info, from main board
 */
typedef struct
{
    U8 robot_state;    /*����״̬*/
    U8 charging_state; /*���״̬*/
    U8 dock_state;     /*dock ״̬*/
    U8 sound_state;    /*����״̬*/
    U8 flush_state;    /*��˸��־*/
    U8 switch_state;   /*��Դ����*/    
    U8 pause_id;       /*��ͣԭ��*/
    U8 bin_full;       /*�����м�� 1:������ 0:δ��*/
    U8 rev1;
    U8 rev2;
    U8 rev3;
    U8 rev4;
    U8 software_version; /*����汾*/
}sys_state_info;


typedef struct
{
    U16 left_motor_current;
    U16 right_motor_current;
    U16 main_brush_current;
    U16 side_brush_current;
    U16 vac_current;
    U16 battery_voltage;
    S16 battery_current;
    S16 battery_thermistor; 
    U16 rev1;
    U16 rev2;    
}sys_current_info;

typedef struct 
{
    S16 bat_type;
    S16 adapter_output_vol;
    S16 adapter_output_current;     	
    S16 pa_gain;     	
    S16 robot_using_lt_bump;
    S16 lt_acc_mmps;
    S16 lt_bump_throd;
    S16 lt_bump_left_right_throd;
    S16 lt_max_light_value; 
    S16 rev1;
    S16 rev2;
    S16 rev3;
    S16 rev4;
    S16 rev5;
    S16 rev6;    
}externsion_info;

typedef struct
{  
    U16 adc_cliff_left;
    U16 adc_cliff_right;     
    U16 adc_cliff_front_left;
    U16 adc_cliff_front_right;
    U16 adc_cliff_rear_left;  
    U16 adc_cliff_rear_right;
    U16 adc_lighttouch_left;
    U16 adc_lighttouch_right; 
    U16 adc_lighttouch_front_left;   
    U16 adc_lighttouch_front_right;    
    U16 adc_lighttouch_center_left;
    U16 adc_lighttouch_center_right;
    U16 rev1;
    U16 rev2;
    U16 rev3;
    U16 rev4;
}sys_sensor_adc_info;
////////////////////////ϵͳʱ����Ϣ/////////////////////////
/*  
 * standard time struct, from main board
 */
typedef struct
{
    U8 hour;
    U8 min;
    U8 sec;
    //������������
    U16 w_year;
    U8  w_month;
    U8  w_date;
    U8  week;
}ui_tm;
/////////////////////////////schedule////////////////////////////////
/*  
 * schedule info, from UI
 *����֧��һ�������10����schdule��Ϣ
 *��Ϣ���ϱ�׼��tm�ṹ������֧�ֵ��δ�����ѭ������
 *1 :tm��������ϢΪ0, ֻ��week, hour, min��ϢʱΪѭ������
 *2 :tm��������Ϣ��Ϊ0��Ϊ���δ���
 */
typedef struct
{
    union{
      struct{
       U8 sunBit  :1;
       U8 monBit  :1;
       U8 tueBit  :1;
       U8 wedBit  :1;
       U8 thuBit  :1;
       U8 friBit  :1;
       U8 satBit  :1;
       U8 rev     :1;
      }BITS;
      U8 BYTE;
    }SCH_ENABLE;	  
    ui_tm t[7];  
}schedule_time_info;

typedef struct 
{
    U16 start_x;
    U16 start_y;
    U16 number;
}map_request_info_t;

typedef enum
{
    MAP_POINT_TYPE_UNEXPLORED = 0,
    MAP_POINT_TYPE_FREEDOM = 1,
    MAP_POINT_TYPE_BUMPPED = 2,
    MAP_POINT_TYPE_CURRENT = 3,
}map_type_t;

#define MAP_POINTS_MAX 16
#define BITMAP_BUF_SIZE 26
typedef struct 
{
    U16 x;
    U16 y;
    U8 type;
}map_point_t;

typedef struct
{
    U16 direction;
    U8  point_count;
    map_point_t points[MAP_POINTS_MAX];
}t_map_info;

typedef struct
{
    t_map_info data;
}obj_map_info_t;


typedef struct
{
    U8 frame_h;
    U8 frame_l;
    U8 start_x;
    U8 start_y;
    U8 end_x;
    U8 end_y;
}cleaned_bitmap_param_t;

typedef struct
{
    U8 err_map_data_flag;
    U8 update_his_map_flag ;
    cleaned_bitmap_param_t param;
    U8 bitdata[BITMAP_BUF_SIZE];
}bitmap_info_t;
typedef struct
{
  U32 cmd;
  U8  cmd_type;
  U8  cmd_state;
  U8  len;
}command;

typedef struct
{
  int    angle;
  float  dist;
  int    rev1;
  int    rev2;
}direction_key_info;

typedef struct
{
    U8 fc;    
    U8 id; 
    command info;     
    U8 data;     
}obj_command_info_t;


typedef struct
{
    U8 fc;    
    U8 id; 
    command info;     
    sys_state_info data;     
}obj_sys_state_info_t;
//////////////////////////////////////////////////////////////
#pragma pack ()
#endif