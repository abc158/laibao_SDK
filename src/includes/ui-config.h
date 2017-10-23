#ifndef _UI_CONFIG_H
#define _UI_CONFIG_H
#include "ui-typedef.h"

typedef struct
{
    U16 left_motor_current;
    U16 right_motor_current;
    U16 main_brush_current;
    U16 side_brush_current;
    U16 vac_current;
    S16 battery_current;
}current_info;


typedef struct
{  
    U16 rear_left_on;         // CLIFF_REAR_LEFT_THRESHOLD_ON      /* ���� */
    U16 rear_left_off;        // CLIFF_REAR_LEFT_THRESHOLD_OFF
    U16 rear_right_on;        // CLIFF_REAR_RIGHT_THRESHOLD_ON     /* ���� */
    U16 rear_right_off;       // CLIFF_REAR_RIGHT_THRESHOLD_OFF 
    U16 left_on;              // CLIFF_LEFT_THRESHOLD_ON           /* �� */
    U16 left_off;             // CLIFF_LEFT_THRESHOLD_OFF 
    U16 right_on;             // CLIFF_RIGHT_THRESHOLD_ON          /* �� */
    U16 right_off;            // CLIFF_RIGHT_THRESHOLD_OFF
    U16 front_left_on;        // CLIFF_FRONT_LEFT_THRESHOLD_ON     /* ǰ�� */
    U16 front_left_off;       // CLIFF_FRONT_LEFT_THRESHOLD_OFF
    U16 front_right_on;       // CLIFF_FRONT_RIGHT_THRESHOLD_ON    /* ǰ�� */
    U16 front_right_off;      // CLIFF_FRONT_RIGHT_THRESHOLD_OFF
    U16 changed;
}cliff_threshold_config_t;


typedef struct
{
    U16 center_left_on;       // LT_CENTERLEFT_THRESHOLD_ON /* �м��� */
    U16 center_left_off;      // LT_CENTERLEFT_THRESHOLD_OFF 
    U16 center_right_on;      // LT_CENTERRIGHT_THRESHOLD_ON/* �м��� */
    U16 center_right_off;     // LT_CENTERRIGHT_THRESHOLD_OFF  
    U16 left_on;              // LT_LEFT_THRESHOLD_ON       /* �� */
    U16 left_off;             // LT_LEFT_THRESHOLD_OFF 
    U16 right_on;             // LT_RIGHT_THRESHOLD_ON      /* �� */
    U16 right_off;            // LT_RIGHT_THRESHOLD_OFF  
    U16 front_left_on;        // LT_FRONT_LEFT_THRESHOLD_ON /* ǰ�� */
    U16 front_left_off;       // LT_FRONT_LEFT_THRESHOLD_OFF
    U16 front_right_on;       // LT_FRONT_RIGHT_THRESHOLD_ON/* ǰ�� */
    U16 front_right_off;      // LT_FRONT_RIGHT_THRESHOLD_OFF
    U16 changed;
}lighttouch_threshold_config_t;


/* ң��������ֵ */
typedef struct
{
    U16 dock;                 // IR_DOCK 
    U16 spot;                 // IR_SPOT
    U16 clean;                // IR_CLEAN 
    U16 power;                // IR_POWER 
    U16 left;                 // IR_LEFT 
    U16 right;                // IR_RIGHT 
    U16 forward;              // IR_FORWARD
    U16 schedule;             // IR_SCHEDULE       // ������
    U16 clock;                // IR_CLOCK          // ������
    U16 driver_stop;          // IR_DRIVERSTOP     // ����
    U16 grid;                 // IR_GRID  
    U16 backward;             // IR_BACKWARD ,
    U16 wall_follow;          // IR_WALLFOLLOW 
    U16 dirt;                 // IR_DIRT           // ����
    U16 clean_drive;          // IR_CLEAN_DRIVE
    U16 random;               // IR_RANDOM
    U16 changed;
    
}remote_ir_code_config_t;

/* ��������������ֵ */
typedef struct
{
    U16 force_field;          // DOCK_FORCE_FIELD  // Բ����ֵ
    U16 buoy_red;             // DOCK_BUOY_RED     // ��
    U16 buoy_green;           // DOCK_BUOY_GREEN   // �� 
    U16 buoy_both;            // DOCK_BUOY_BOTH       
    U16 changed;
}dock_ir_code_config_t;

/* ��Դ������Ϣ */
typedef struct
{
    U16 battery_charge_voltage_max_adc;    
    U16 side_brush_max_voltage_adc;
    U16 main_brush_max_voltage_adc;
    U16 vacuum_max_voltage_adc;
    U16 rev1;   
    U16 rev2;   
    U16 changed; 
}power_config_t;



/* ����ṹ�ϵ�һЩ��Ϣ */
typedef struct
{
    float wheel_dia;                    // WHEEL_DIA                     /* �������ӵ�ֱ�� */
    float wheel_left_right_distance;    // WHEEL_LEFT_RIGHT_DISTANCE     /* ������֮����� */
    float wheel_code_ratio;             // WHEEL_CODE_RATIO              /* ��-���̱��� */
    float pad_width;                   // WHEEL_MAIN_BRUSH_WIDTH        /* ��ˢ�Ŀ�� */
    U16 crash_cond;                   // CRASH_COND    = LT_BUMP_ON,   /* ���ϼ������ */
    U16 p_value;                      // P_VALUE         
    U16 i_value;                      // I_VALUE
    U16 d_value;                      // D_VALUE
    U16 hardware_version;             // HARDWARE_VERSION  
    U16 rev1;   
    U16 rev2;   
    U16 rev3;
    U16 rev4;    
    U16 changed;
}hard_structure_config_t;

/* ��Ϊ�ٶ���Ϣ */

typedef struct
{
    U16 clean_speed_max;              // CLEAN_SPEED_MAX       /* ��ɨʱ����ٶ� */
    U16 crash_speed;                  // CRASH_SPEED           /* �����ϰ��ｵ���ٶ� */
    U16 dock_speed_nomal;             // DOCK_SPEED_NOMAL      /* �����������ٶ� */
    U16 dock_speed_low;               // DOCK_SPEED_LOW        /* �ҵ������󽵵͵��ٶ� */
    U16 wallfollow_speed;             // �ر��ٶ�
    U16 rev2;     
    U16 changed;
}speed_config_t;

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
    }bits;
    U16 funs;
  }clean_fun;    
}clean_motor_config_t;

typedef struct
{
    S16 am_ir_wf_min_val;//IR_WF_MIN_VAL;
    S16 rev0;
    S16 rev1;    
    S16 rev2;
    S16 rev3;    
    S8  changed;
}wf_info_t;

typedef struct 
{
    U8 native_charging_en;//ʹ��һ΢���ϵͳ��Ĭ�ϴ�
    U8 bat_type;
    U8 adapter_output_vol;
    S16 adapter_output_current;     	
    S16 rev1;
    S16 rev2;
    U8  changed;
}charge_info_t;

/*������Ϣ*/
typedef struct
{
  union{
    struct{	
	  U32 motor_driver     :2; /*�������оƬ����0:8843,  1:8870,  2:8801 */
      U32 wheel_left_dir   :1; /*  ��������io�ڿ�������ת���ķ���*/	  
      U32 wheel_right_dir  :1; /*  ��������io�ڿ�������ת���ķ���*/   
      U32 wheepdrop        :1;
      U32 stasis           :1; /*, ON:open, OFF:close*/	   		  
      U32 lt_type          :2; /*, ON:open, OFF:close*/
      U32 rear_cliff       :1;
      U32 ltBumpNoBack     :1; /*, ����lt bumpʱ�������Ƿ���Ҫ�Ⱥ�����ת�䣬0������, 1��������*/
      U32 gyro_dir         :2; /* */	  
      U32 wall_follow_dir  :2; /*0: left and right, 1:left, 2:right*/
      U32 rev7             :2; /*��ON:open, OFF:close*/
      U32 rev8             :16;
    }bits;
    U32 funs;
  }clean_fun;  
}funtion_info_t;


typedef struct
{
   S16 pa_gain; 
   S16 rev;
   S16 rev1;
}play_info_t;

/*����������ṹ����Ϣ�Ƕ�sdkϵͳ�������õ���Ϣ���ṹ�岻�ܽ�����չ��ֻ������rev����*/
typedef struct
{
  power_config_t 			power_config;
  hard_structure_config_t 		hard_structure;
  speed_config_t 			speed_config;
  clean_motor_config_t                  clean_motor_config;
  charge_info_t                         charge_info;
  wf_info_t                             wf_info;
  play_info_t                           play_info;
  funtion_info_t                        funtion_info;
}ui_config_t;

/*����ṹ����Ϣ�Ǳ���ʹ�õ�������Ϣ*/
typedef struct
{
  cliff_threshold_config_t 		cliff_threshold;
  lighttouch_threshold_config_t 	lighttouch_threshold;
  remote_ir_code_config_t 		remote_ir_codes;
  dock_ir_code_config_t 		dock_ir_codes;
}ui_local_config_t;

typedef enum
{
  FAKE_SLEEP     = 0xfe,
  SLIGHT_SLEEP   = 0xfa,
  NOMAL_SLEEP    = 0xfd,
  CHARGING_SLEEP = 0xfc,
  DEEP_SLEEP     = 0xfb,
}SLEEP_TYPE_E;

extern remote_ir_code_config_t remote_ir_codes;
extern ui_config_t* get_ui_config(void);
extern ui_local_config_t* get_local_ui_config(void);

typedef void (*ui_config_update_func_t)(ui_config_t* ui_config);
void ui_config_init(void);
#endif /* #ifndef _UI_CONFIG_H */