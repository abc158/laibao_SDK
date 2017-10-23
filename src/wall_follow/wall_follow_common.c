#include "wall_follow_common.h"
#include "syscall_api.h"
#include "sensor/sensor.h"
#include <math.h>


#define USE_WF_MID_CONTROL
#define MID_THROD 100  //center throd to make robot turn away
#define MID_PLUS  20
#define WF_SPEED_VALUE 240           //the speed following wall
#define WF_SPEED_SIGNAl_SMALL 150    //the speed turn to wall to find the signal

#define IR_MIN_VALUE  25   //A3: 35
#define IR_MAX_VALUE  250

#define PID_INT_MAX_VAL 500
#define WF_PID_p 0.002
#define WF_PID_i 0.00005
#define WF_PID_d 0.00002

#define TURN_COUNT_MAX 200  //150    //when no signal, inner speed is 1/5, after this time, speed set to zero, if 255, no such function

#define PHYSIC_BUMP_ADAPTER_TARGET_VALUE     1/3  //when detect physic bump, max ir value plus this value
#define NO_PHYSIC_BUMP_ADAPTER_TARGET_VALUE  5/6  //when detect no physic bump, max ir value plus this value

#define USE_WF_NOT_CYCLE_PROCESS

//#define OPEN_DEBUG_PRINT
//#define OPEN_CYCLE_WF_PRINT

#ifdef OPEN_DEBUG_PRINT
  #define AM_WF_DEBUG_PRINT(...)    printf(__VA_ARGS__);
#else
  #define AM_WF_DEBUG_PRINT(...)
#endif

#ifdef OPEN_CYCLE_WF_PRINT
  #define AM_WF_CYCLE_DEBUG_PRINT(...)    printf(__VA_ARGS__);
#else
  #define AM_WF_CYCLE_DEBUG_PRINT(...)
#endif


WfParames_t* WfParames_p;

#define UTIL_CLIFF_MASK 0x1
#define UTIL_DOCK_MASK  0x2
// returns angle1-angle2, between -M_PI_F and M_PI_F
float gyro_radian_diff_f(float angle1, float angle2)
{
   // printf("gyro_radian_diff_f!!\r\n");
    float a = angle1 - angle2 + M_PI_F;
    if(a > 2*M_PI_F)
    {
        a = a - 2*M_PI_F*floorf(a/(2*M_PI_F));
    }
    else if(a<0)
    {
        a = a + 2*M_PI_F*(1+floorf(-a/(2*M_PI_F)));
    }

    return a - M_PI_F;
}


typedef struct _wf_global_par_t
{
  float turn_target_angle;
  s16 ir_target_value;
  s16 ir_adapter_max;
  s32 pid_integrate;
  s32 pid_deritive;
  s16 ex_ir_error;
  u8 following_count; //increase counting when robot already following the wall
  u8 util;  //bit 0: cliff sig, bit 1: dock avoid sig
  u8 turn_count;
}wf_g_par_t;

#define FOLLOWING_COUNT_MAX 200
#define FOLLOWING_COUNT_MIN 0

typedef struct _sensor_data_t_
{
  s16 ir_left_value;
  s16 ir_right_value;
#ifdef USE_WF_MID_CONTROL
  s16 ir_mid_left_value;
  s16 ir_mid_right_value;
#endif
}ir_sensor_data_t;

//avoid wall-follow turn cycle
#ifdef USE_WF_NOT_CYCLE_PROCESS
#define CYCLE_ARRAY_INDEX_MAX 16
#define CYCLE_COUNT_MAX 2
#define ANGLE_INTERVEL M_PI_F/4
#define ANGLE_INTERVEL_TIME 2000
#define WF_CYCLE_TIME_OUT_VAL 20000
#define CYCLE_GO_MAX_DISTANCE 4.0f
#define CYCLE_GO_MAX_TIMEOUT  20000
typedef struct _wf_turn_array_t_
{
  float x;
  float y;
  float angle;
  u32 stp;
}wf_turn_array_t;

wf_turn_array_t* wf_turn_array_p=NULL;

u8 wf_turn_index_i;
u8 wf_turn_good_count;
BOOLEAN in_cycle_go_forward_sig;
#endif

wf_g_par_t wf_g_par;


#ifdef USE_WF_NOT_CYCLE_PROCESS
int cycle_check_function()
{
  //check cycle or not
  wf_turn_array_t* wf_turn_at_p;
  wf_turn_array_t* wf_search_turn_p;
  u8 wf_search_trun_index;
  float now_angle,now_x,now_y;
  u32 now_stp;
  u8 i;
  float dist_temp;

    wf_turn_at_p = wf_turn_array_p + wf_turn_index_i;
    now_angle=wf_turn_at_p->angle;
    now_stp=wf_turn_at_p->stp;
    now_x=wf_turn_at_p->x;
    now_y=wf_turn_at_p->y;
    //search back
    wf_search_trun_index = (wf_turn_index_i==0)?(CYCLE_ARRAY_INDEX_MAX-1):(wf_turn_index_i-1);
    for(i=0;i<CYCLE_ARRAY_INDEX_MAX-2;i++)
    {
      wf_search_turn_p = wf_turn_array_p + wf_search_trun_index;
      dist_temp = (now_x-wf_search_turn_p->x)*(now_x-wf_search_turn_p->x)+(now_y-wf_search_turn_p->y)*(now_y-wf_search_turn_p->y);
      if(((now_stp-wf_search_turn_p->stp)>WF_CYCLE_TIME_OUT_VAL) && (dist_temp<0.5*0.5))
      {
        return 0;  //timeout return
      }
      if(fabsf(gyro_radian_diff_f(now_angle,wf_search_turn_p->angle))<M_PI_F/4)
      {
          if(fabsf(now_angle-wf_search_turn_p->angle)>(2*M_PI_F*7/8))
          {
            //check if it's the inner cycle or outer cycle, inner cycle mean cycle one small obstacle. we only check iner cycle.
            if((WfParames_p->contact_side==AM_LEFT && (now_angle>wf_search_turn_p->angle)) || (WfParames_p->contact_side==AM_RIGHT && (now_angle<wf_search_turn_p->angle)))
            {
              //good value
              AM_WF_CYCLE_DEBUG_PRINT("wf cy %d,%d\r\n", wf_search_trun_index,wf_turn_good_count);
              wf_turn_good_count++;
              if(wf_turn_good_count>CYCLE_COUNT_MAX)
              {
                return 1;
              }
              return 0;
            }
            else
            {
              AM_WF_CYCLE_DEBUG_PRINT("not inner %d,%d\r\n", wf_search_trun_index,wf_turn_good_count);
            }
          }
      }
      wf_search_trun_index = (wf_search_trun_index==0)?(CYCLE_ARRAY_INDEX_MAX-1):(wf_search_trun_index-1);
    }
    if(wf_turn_good_count>0)
    {
      wf_turn_good_count--;
    }

  return 0;
}
#endif

void reload_sensor_data(ir_sensor_data_t* sensor_data)
{
  sensor_data->ir_left_value=robot_signal_distance(LT_LEFT)>>1;
  sensor_data->ir_right_value=robot_signal_distance(LT_RIGHT)>>1;
#ifdef USE_WF_MID_CONTROL
  sensor_data->ir_mid_left_value=robot_signal_distance(LT_CENTERLEFT)>>1;
  sensor_data->ir_mid_right_value=robot_signal_distance(LT_CENTERRIGHT)>>1;
#endif
}

//turn the robot away from wall
void turn_away_wall(s16* left_speed, s16* right_speed)
{
  float angle_error = gyro_radian_diff_f(wf_g_par.turn_target_angle, get_gyro_radian());
  float angle_abs_error=fabsf(angle_error);
  float speed_val;

  if(angle_abs_error<M_PI_F/13)
  {
	//reach the angle, stop and change status
    *left_speed=0;
    *right_speed=0;
    WfParames_p->wf_run_state=WF_STAT_FOLLOW; //change the state
    //AM_WF_DEBUG_PRINT("wf s c\r\n");
  }
  else
  {
	//turn away from wall
    if(angle_error>0)
    {
      speed_val = MIN(angle_error, 1) * WF_SPEED_VALUE;
    }
    else
    {
      speed_val = MAX(angle_error, -1) * WF_SPEED_VALUE;
    }
    *left_speed=(s16)speed_val*-1;
    *right_speed=(s16)speed_val;
  }
}


void calculate_wheel_speed(s16 ir_error, s16* close_wheel, s16* far_wheel)
{
  float cof;

  //this variable control the start speed
  wf_g_par.following_count = (wf_g_par.following_count<FOLLOWING_COUNT_MAX)?(wf_g_par.following_count+1):FOLLOWING_COUNT_MAX;

  //calculate the PID i
  wf_g_par.pid_integrate += ir_error;
  if(wf_g_par.pid_integrate>PID_INT_MAX_VAL)
  {
    wf_g_par.pid_integrate=PID_INT_MAX_VAL;
  }
  else if(wf_g_par.pid_integrate < (-1*PID_INT_MAX_VAL))
  {
    wf_g_par.pid_integrate=-1*PID_INT_MAX_VAL;
  }

  //calculate the PID d
  wf_g_par.pid_deritive = (ir_error-wf_g_par.ex_ir_error);

  //calculate PID
  cof = (float)ir_error*WF_PID_p + (float)wf_g_par.pid_integrate*WF_PID_i-(float)wf_g_par.pid_deritive*WF_PID_d;
  cof = MAX(MIN(fabsf(cof), 0.6f), 0.03f);

  //calculate speed
  if(ir_error>0)  //close to wall
  {
    *close_wheel=(s16)(WF_SPEED_VALUE);
    *far_wheel=(s16)((float)WF_SPEED_VALUE*(1-cof));
  }
  else if(ir_error<0) //far away from wall
  {
    *far_wheel=(s16)(WF_SPEED_VALUE);
    *close_wheel=(s16)((float)WF_SPEED_VALUE*(1-cof));
  }
  else
  {
    *far_wheel=WF_SPEED_VALUE;
    *close_wheel=WF_SPEED_VALUE;
  }

  //do the start speed control
  if(wf_g_par.following_count<FOLLOWING_COUNT_MAX-1)
  {
	    *far_wheel=*far_wheel>>1;
	    *close_wheel=*close_wheel>>1;
  }
  else
  {
    //AM_WF_DEBUG_PRINT("wf_f\r\n");
  }
}

void follow_wall_run(s16* left_speed, s16* right_speed, ir_sensor_data_t* sensor_data)
{
  s16 ir_error=0;

  if(WfParames_p->contact_side == AM_LEFT)
  {
    if(sensor_data->ir_left_value < IR_MIN_VALUE)
    {
      //when the ir sensor value is small, just turn to wall
      *right_speed=WF_SPEED_SIGNAl_SMALL;
      if(wf_g_par.util & UTIL_CLIFF_MASK)
      {   //when cliff and dock avoid, the robot turn slowly
        *left_speed=WF_SPEED_SIGNAl_SMALL/2;
      }
      else if(wf_g_par.util &  UTIL_DOCK_MASK)
      {
    	  *left_speed=WF_SPEED_SIGNAl_SMALL*8/11;
      }
      else
      {
    	  if(wf_g_par.turn_count<TURN_COUNT_MAX)
    	  {
              *left_speed=WF_SPEED_SIGNAl_SMALL/5;
    	  }
    	  else   //delay a time, make robot one wheel stop, more closer to obstacle
    	  {
    		  *right_speed=WF_SPEED_SIGNAl_SMALL*3/4;
    		  *left_speed=WF_SPEED_SIGNAl_SMALL/8;
    	  }
      }
      AM_WF_DEBUG_PRINT("wf Lf %d\r\n", sensor_data->ir_left_value);
      wf_g_par.pid_integrate=0;
      wf_g_par.pid_deritive=0;
      wf_g_par.ex_ir_error=0;
      wf_g_par.following_count=FOLLOWING_COUNT_MIN;
      wf_g_par.turn_count = (wf_g_par.turn_count<TURN_COUNT_MAX)?(wf_g_par.turn_count+1):TURN_COUNT_MAX;
    }
    else
    {
      //when the signal is larger than threshold, do the pid control
      ir_error = sensor_data->ir_left_value - wf_g_par.ir_target_value;
#ifdef USE_WF_MID_CONTROL
      if(sensor_data->ir_mid_left_value>(MID_THROD+wf_g_par.ir_target_value)/2)
      {
    	  ir_error += sensor_data->ir_mid_left_value*MID_PLUS;
      }
#endif
      calculate_wheel_speed(ir_error, left_speed, right_speed);
      AM_WF_DEBUG_PRINT("wf Ls(%d,%d),%d,%d,(%d,%d)\r\n", *left_speed, *right_speed, ir_error,wf_g_par.following_count,sensor_data->ir_left_value, wf_g_par.ir_target_value);
      WfParames_p->wf_following_sig=TRUE;
      wf_g_par.util = 0;
      wf_g_par.turn_count=0;
    }
  }
  else  //AM_RIGHT
  {
    if(sensor_data->ir_right_value < IR_MIN_VALUE)
    {
      //when the ir sensor value is small, just turn to wall
      *left_speed=WF_SPEED_SIGNAl_SMALL;
      if(wf_g_par.util & UTIL_CLIFF_MASK)
      {
          *right_speed=WF_SPEED_SIGNAl_SMALL/2;
      }
      else if(wf_g_par.util &  UTIL_DOCK_MASK)
      {
    	  *right_speed=WF_SPEED_SIGNAl_SMALL*8/11;
      }
      else
      {
  	    if(wf_g_par.turn_count<TURN_COUNT_MAX)
  	    {
            *right_speed=WF_SPEED_SIGNAl_SMALL/5;
  	    }
  	    else   //delay a time, make robot one wheel stop, more closer to obstacle
  	    {
  	    	*left_speed=WF_SPEED_SIGNAl_SMALL*3/4;
  	  	    *right_speed=WF_SPEED_SIGNAl_SMALL/8;
  	    }
      }
      AM_WF_DEBUG_PRINT("wf Rf %d\r\n", sensor_data->ir_left_value);
      wf_g_par.pid_integrate=0;
      wf_g_par.pid_deritive=0;
      wf_g_par.ex_ir_error=0;
      wf_g_par.following_count=FOLLOWING_COUNT_MIN;
      wf_g_par.turn_count = (wf_g_par.turn_count<TURN_COUNT_MAX)?(wf_g_par.turn_count+1):TURN_COUNT_MAX;
    }
    else
    {
      //when the signal is larger than threshold, do the pid control
      ir_error = sensor_data->ir_right_value - wf_g_par.ir_target_value;
#ifdef USE_WF_MID_CONTROL
      if(sensor_data->ir_mid_right_value>(MID_THROD+wf_g_par.ir_target_value)/2)
      {
    	  ir_error += sensor_data->ir_mid_right_value*MID_PLUS;
      }
#endif
      calculate_wheel_speed(ir_error, right_speed, left_speed);
      AM_WF_DEBUG_PRINT("wf Rs(%d,%d),%d,%d,(%d,%d)\r\n", *left_speed, *right_speed, ir_error,wf_g_par.following_count,sensor_data->ir_right_value, wf_g_par.ir_target_value);
      WfParames_p->wf_following_sig=TRUE;
      wf_g_par.util = 0;
      wf_g_par.turn_count=0;
    }
  }

  wf_g_par.ex_ir_error=ir_error;
}

int wall_follow_start(AM_LeftRight contact_side, WfParames_t* wfp_p)
{
  WfParames_p=wfp_p;
  if(WfParames_p->wf_run_state != WF_STAT_STOP && WfParames_p->contact_side==contact_side)
  {
        AM_WF_DEBUG_PRINT("wf already run:%d\r\n", WfParames_p->contact_side);
        return 1;
  }
  //WfParames_p->wf_run_state = WF_STAT_FOLLOW;
  //wf_g_par.turn_target_angle = get_gyro_radian();
  
  WfParames_p->wf_run_state=WF_STAT_TURN;
  WfParames_p->contact_side=contact_side;
  if(WfParames_p->contact_side==AM_LEFT)
  {
    wf_g_par.turn_target_angle = get_gyro_radian() - M_PI_F/4.3f;
  }
  else  //AM_RIGHT
  {
    wf_g_par.turn_target_angle = get_gyro_radian() + M_PI_F/4.3f;
  }
  
  wf_g_par.ir_target_value = IR_MIN_VALUE*2;  //set a init value, this variable should be changed during running
  wf_g_par.ir_adapter_max = 0;
  
  wfp_p->wf_following_sig=FALSE;
  AM_WF_DEBUG_PRINT("app wf s\r\n");
  //when state is not stop, and contact_side is different, what should do?
#ifdef USE_WF_NOT_CYCLE_PROCESS
  if(wf_turn_array_p==NULL)
  {
    wf_turn_array_p = malloc(CYCLE_ARRAY_INDEX_MAX * sizeof(wf_turn_array_t));
    if(wf_turn_array_p != NULL)
    {
      memset(wf_turn_array_p, 0, CYCLE_ARRAY_INDEX_MAX * sizeof(wf_turn_array_t));
    }
    wf_turn_index_i=0;
    wf_turn_good_count=0;
    in_cycle_go_forward_sig=FALSE;
  }
#endif
  return 0;
}

void wall_follow_stop(WfParames_t* wfp_p)
{
  WfParames_p=wfp_p;
  if(WfParames_p->wf_run_state != WF_STAT_STOP)
  {
    WfParames_p->wf_run_state=WF_STAT_STOP;
  }
  AM_WF_DEBUG_PRINT("app wf e\r\n");
#ifdef USE_WF_NOT_CYCLE_PROCESS
  if(wf_turn_array_p!=NULL)
  {
    free(wf_turn_array_p);
    wf_turn_array_p=NULL;
  }
#endif
}

//wf_stae_value:
//             WF_STAT_STOP=0
//             WF_STAT_TURN=1
//             WF_STAT_FOLLOW=2
//When sytem process (bump, cliff, stuck), this callback will be block
//This function will be called by system every 10mS.
//Make sure there is no dead loop in the function.
//return: 1: bump return,  2: cliff return,
//      
int wall_follow_callback(WfParames_t* wfp_p)
{
  ir_sensor_data_t sensor_data;
  s16 ir_now_data;

  //update sensor data
  reload_sensor_data(&sensor_data);

  WfParames_p=wfp_p;

  //get the adaptive threshold
  if(WfParames_p->contact_side==AM_LEFT)
  {
	  ir_now_data = sensor_data.ir_left_value;
  }
  else
  {
	  ir_now_data = sensor_data.ir_right_value;
  }
  if(wf_g_par.ir_adapter_max<ir_now_data)
  {
	  wf_g_par.ir_adapter_max = ir_now_data;
  }
  //physic bump need use small touch
  if(wfp_p->physic_bump==TRUE)
  {
     wf_g_par.ir_target_value = wf_g_par.ir_adapter_max*PHYSIC_BUMP_ADAPTER_TARGET_VALUE;
  }
  else
  {
    wf_g_par.ir_target_value = wf_g_par.ir_adapter_max*NO_PHYSIC_BUMP_ADAPTER_TARGET_VALUE;
  }

  if(wf_g_par.ir_target_value<IR_MIN_VALUE*1.5)
  {
    wf_g_par.ir_target_value=(s16)(IR_MIN_VALUE*1.5);
  }
  if(wf_g_par.ir_target_value>IR_MAX_VALUE)
  {
	  wf_g_par.ir_target_value=IR_MAX_VALUE;
  }

  //process the bump
  if((WfParames_p->robot_bump != 0) && (WfParames_p->cliff_bump == 0))  //cliff has high priority
  {
    WfParames_p->wf_run_state=WF_STAT_TURN;
    if(WfParames_p->contact_side==AM_LEFT)
    {
        //Note:it's better change the turn angle by the bump location or touch location
        wf_g_par.turn_target_angle = get_gyro_radian() - M_PI_F/4.3f;
    }
    else  //AM_RIGHT
    {
    	//Note:it's better change the turn angle by the bump location or touch location
        wf_g_par.turn_target_angle = get_gyro_radian() + M_PI_F/4.3f;
    }
    AM_WF_DEBUG_PRINT("app wf b(%d,%d)\r\n", (int)(get_gyro_radian()*100), (int)(wf_g_par.turn_target_angle*100));
    //check if robot is in dock region
    //if(in_dock_region())
    //wf_g_par.util |= UTIL_DOCK_MASK;

    wf_g_par.ir_adapter_max = wf_g_par.ir_adapter_max >> 2;
    wf_g_par.ir_target_value = wf_g_par.ir_target_value >> 2;
    AM_WF_DEBUG_PRINT("app new throd(%d,%d)\r\n", wf_g_par.ir_adapter_max, wf_g_par.ir_target_value);
    wfp_p->wf_following_sig=FALSE;
    in_cycle_go_forward_sig=FALSE;
    return 1;  //return 1, let system process the bump

  }

  //process the cliff
  if(WfParames_p->cliff_bump != 0)
  {
    WfParames_p->wf_run_state=WF_STAT_TURN;
    if(WfParames_p->contact_side==AM_LEFT)
    {
      wf_g_par.turn_target_angle = get_gyro_radian() - M_PI_F/4.3f;
    }
    else  //AM_RIGHT
    {
      wf_g_par.turn_target_angle = get_gyro_radian() + M_PI_F/4.3f;
    }
    AM_WF_DEBUG_PRINT("app wf c(%d,%d)\r\n", (int)(get_gyro_radian()*100), (int)(wf_g_par.turn_target_angle*100));
    wf_g_par.util |= UTIL_CLIFF_MASK;

    wf_g_par.ir_target_value = wf_g_par.ir_target_value >> 2;
    wf_g_par.ir_adapter_max = wf_g_par.ir_adapter_max>>2;
    AM_WF_DEBUG_PRINT("app newc throd(%d,%d)\r\n", wf_g_par.ir_adapter_max, wf_g_par.ir_target_value);
    wfp_p->wf_following_sig=FALSE;
    in_cycle_go_forward_sig=FALSE;
    return 2;  //return 2, let system process the cliff
  }

  //process dock signal
  if(WfParames_p->dock_sig != 0)
  {
      WfParames_p->wf_run_state=WF_STAT_TURN;
      if(WfParames_p->contact_side==AM_LEFT)
      {
        wf_g_par.turn_target_angle = get_gyro_radian() - M_PI_F/2.1f;
      }
      else  //AM_RIGHT
      {
        wf_g_par.turn_target_angle = get_gyro_radian() + M_PI_F/2.1f;
      }
      AM_WF_DEBUG_PRINT("wf d(%d,%d)\r\n", (int)(get_gyro_radian()*100), (int)(wf_g_par.turn_target_angle*100));
      wf_g_par.util |= UTIL_DOCK_MASK;

      wf_g_par.ir_target_value = wf_g_par.ir_target_value >> 2;
      wf_g_par.ir_adapter_max = wf_g_par.ir_adapter_max>>2;
      AM_WF_DEBUG_PRINT("newc throd(%d,%d)\r\n", wf_g_par.ir_adapter_max, wf_g_par.ir_target_value);
      wfp_p->wf_following_sig=FALSE;
      in_cycle_go_forward_sig=FALSE;
      return 3;
  }

  switch(WfParames_p->wf_run_state)
  {
  case WF_STAT_TURN:
    turn_away_wall(&WfParames_p->left_speed_mmps, &WfParames_p->right_speed_mmps);
    wf_g_par.turn_count=0;
    break;
  case WF_STAT_FOLLOW:
    follow_wall_run(&WfParames_p->left_speed_mmps, &WfParames_p->right_speed_mmps, &sensor_data);
    break;
  default:
    break;
  }

  //make sure the wall follow don't go in cycle
  //this process is at last, so it has highest priority
#ifdef USE_WF_NOT_CYCLE_PROCESS
  if(wf_turn_array_p!=NULL)
  {
    AM_Pose robot_pose;
    wf_turn_array_t* wf_turn_at_p;
    u8 index_ex;
    float angle_diff;

    //store the angle record
    robot_pos_get(&robot_pose);
    //check if the data need to be stored or not
    index_ex=(wf_turn_index_i==0)?(CYCLE_ARRAY_INDEX_MAX-1):(wf_turn_index_i-1);
    wf_turn_at_p = wf_turn_array_p + index_ex;
    angle_diff=fabsf(gyro_radian_diff_f(wf_turn_at_p->angle, robot_pose.angle));

    if(angle_diff>ANGLE_INTERVEL || timer_elapsed(wf_turn_at_p->stp)>ANGLE_INTERVEL_TIME)
    { //record the data in circle array
      wf_turn_at_p = wf_turn_array_p + wf_turn_index_i;
      wf_turn_at_p->x=robot_pose.xy.x;
      wf_turn_at_p->y=robot_pose.xy.y;
      wf_turn_at_p->angle=robot_pose.angle;
      wf_turn_at_p->stp=timer_ms();
      AM_WF_CYCLE_DEBUG_PRINT("wf rec (%d,%d),%d\r\n", (int)( wf_turn_at_p->angle*1000), wf_turn_at_p->stp,wf_turn_index_i);
      //turn cycle process function
      if(cycle_check_function()>0)
      {
        //process go forward
        memset(wf_turn_array_p, 0, CYCLE_ARRAY_INDEX_MAX * sizeof(wf_turn_array_t));
        wf_turn_index_i=0;
        wf_turn_good_count=0;
        //the first data the record the position and time
        wf_turn_at_p = wf_turn_array_p + wf_turn_index_i;
        wf_turn_at_p->x=robot_pose.xy.x;
        wf_turn_at_p->y=robot_pose.xy.y;
        wf_turn_at_p->angle=robot_pose.angle;
        wf_turn_at_p->stp=timer_ms();
        in_cycle_go_forward_sig=TRUE;
        AM_WF_CYCLE_DEBUG_PRINT("wf in cyc\r\n");
      }
      wf_turn_index_i = (wf_turn_index_i<CYCLE_ARRAY_INDEX_MAX-1)?(wf_turn_index_i+1):0;
    }

      //do go forward
      if(in_cycle_go_forward_sig==TRUE)
      {
        AM_xy start_xy;
        float go_distance;
        start_xy.x=wf_turn_array_p->x;
        start_xy.y=wf_turn_array_p->y;
        go_distance=robot_xy_dist(&robot_pose.xy, &start_xy);
        if((go_distance>CYCLE_GO_MAX_DISTANCE) || (timer_elapsed(wf_turn_array_p->stp)>CYCLE_GO_MAX_TIMEOUT))
        { //add max distance and timeout check
          wfp_p->left_speed_mmps=0;
          wfp_p->right_speed_mmps=0;
          in_cycle_go_forward_sig=FALSE;
          AM_WF_CYCLE_DEBUG_PRINT("wf cycle stop\r\n");
          return 0;
        }
        wfp_p->left_speed_mmps=WF_SPEED_VALUE;
        wfp_p->right_speed_mmps=WF_SPEED_VALUE;
        //AM_WF_CYCLE_DEBUG_PRINT("wf cycle line\r\n");
      }
  }

#endif

  return 0;
}


