#ifndef __LOCAL_KEY_CHECK_H__
#define __LOCAL_KEY_CHECK_H__
#include "am_type.h"
#include "am_key.h"

typedef struct
{
  U8 state;            /*����״̬��SHORT/LONG/HOLD/SHORT_UP/LONG_UP/HOLD_UP*/
  int am_io;           /*������Ӧ��GPIO*/
  u8 input_gpio_state; /*��������ʱ��GPIO��ƽ*/
  key_val_e keyval;    /*��������*/
}key_state_t;

typedef struct
{
  U8 state;            /*����״̬��SHORT/LONG/HOLD/SHORT_UP/LONG_UP/HOLD_UP*/
  U32 ir_code;         /*ң�ذ����ı���*/
  key_val_e keyval;    /*��������*/
}ir_state_t;

typedef struct
{
  U8 state;            /*����״̬��SHORT/LONG/HOLD/SHORT_UP/LONG_UP/HOLD_UP*/
  U16 cmd_code;        /*�����������*/
  key_val_e keyval;    /*��������*/
}cmd_state_t;

extern  void _key_init(void);
extern  void _key_exit(void);
extern  void key_routine(void);
extern  void handle_key( void );
extern  key_state_t *get_key_state(void);
extern  u8 any_key(void);
extern  u8 clean_key(U32 keyval);

#endif