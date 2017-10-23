
#ifndef __DEFTYPE_SIMWIFI_H__
#define __DEFTYPE_SIMWIFI_H__

//#define  gzsdebug                   //��������
//#define  gzsdebugOut
#define  FULL_FAST_RUN               //�����ܿ��ٶ�ȡ������ӦAPP�����ź�
#define  UART_BAUD_RATE    115200  // 9600 //���ڲ����� 
#define  APP_COM_NUM       0X03EC //һ��������Ŀ(for һ�ֵ����豸)===�е���APP�����
//#define  WAITING_TIMEOUT_COUNTER__MAXVAL        3000     //40*20ms=800ms==0.8S �ȴ�NEXT WORKING ������
#define  WAITING_TIMEOUT_COUNTER__MAXVAL          3  
//#define  WAITING_TIMEOUT_COUNTER__MAXVAL          80
#define LEN_MAX 0xff
#define LEN_MIN 0x1
#define LEN_OF_BASE_PACK 3
#define PACK_HEAD        0xAA
#define WIFI_ID          0x0
#define DEVICE_ID        0xA380
#define PROTOCOL_VER     0x1
#define PROTOCOL_TYPE    0x01F5
#define REV              0x0

#define UART_TX_BUF_SIZE 100


typedef struct
{
  U8  head;   //0xAA
  U8  msglen; //
  U8  cmd;
  U8  *data_buf;    //��Ϣ��
  U8  checksum; //byte2 - byteN ���
}wifi_uart_protocol_t;

typedef struct
{
	U8 hour;
	U8 min;
	U8 period;
	U8 week;
}sch_time_t;

typedef enum
{
  CMD_CHEACK_BASE_INFO  = 0x01, //��ѯ������Ϣ
  CMD_CONTROL           = 0x02, //����ָ���·�
  CMD_MAP               = 0x03, //��ͼָ��
  CMD_WIFI_STATE        = 0x05, //WIFI����״̬��ѯ
  CMD_GET_ROBOT_STATE   = 0x06, //��ƽ̨״̬��ѯ,����ѯɨ�ػ���ǰ��״̬
  CMD_CONFIG_NETWORK    = 0x07, //��������
  CMD_ENTER_TEST        = 0x09, //���볧��ģʽ
}WIFI_CMD_E;

typedef enum
{
  DECODE_PACK_WAIT=0x0,
  DECODE_PACK_HEAD=0x1,
  DECODE_PACK_READING = 0x2,
  DECODE_PACK_OK  = 0x3,
  DECODE_PACK_ERR = 0x4,
}DECODE_STATE_E;

typedef enum
{
  WIFI_WAIT_CONNECT    = 0x1,
  WIFI_CONNECTING      = 0x2,
  WIFI_CONNECT_OK      = 0x3,
}WIFI_STATE_E;

#define UART_CMD_MAX 14


							




#define LOW_POWER  2592   //12v
#define FULL_POWER 3456   //16v 4096*(Vbat*0.174/3.3)
#define MID_POWER  3132   //14.5v

//**********************
extern uint8_t ExtCmdInserted; 
extern  U8 local_map_point_available;
//**********************
uint8_t InsertExtCmd(uint8_t ExtCmd);
extern U16 get_uart_cmd(void);
//extern key_state_t *get_uart_cmd_state(void);
extern void uart_cmd_rounte(void);
extern void IRQ_Usart1RxData_Process(void);
extern void WifiData_Period_Process(void);
extern void UART_Config_PinResetWifiCard(void);
//extern U16 get_Value_HeaterPower(void);
extern void set_wifi_enable(uint8_t value);
extern uint8_t get_wifi_enable_state(void);
extern void set_reset_wifi_flag(uint8_t value);
extern uint8_t get_reset_wifi_flag(void);
extern void set_wifi_state(uint8_t value);
extern uint8_t get_wifi_state(void);
extern void exit_wifi(void);
extern U8 send_pack(U8 cmd, U8 *data_buf, U8 datalen);
extern void map_data_process(void);
extern U8 uart_server_routine(void);
extern void wifi_init(void);
extern void wifi_handle_sleep(void);
//********************

//**********************

//***********************
//***********

#endif
