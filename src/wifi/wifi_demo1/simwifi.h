
#ifndef __DEFTYPE_SIMWIFI_H__
#define __DEFTYPE_SIMWIFI_H__

//#define  gzsdebug                   //?��?��?��??
//#define  gzsdebugOut
#define  FULL_FAST_RUN               //������壬�����ܿ��ٶ�ȡ������ӦAPP�����ź�,����20mSһ�ζ�ȡ
#define  UART_BAUD_RATE     38400  //9600 //���ڲ����� 
#define  APP_COM_NUM       0X03EC //һ��������Ŀ(for һ�ֵ����豸)===�е���APP�����,
                                   //����������ƽ̨����
#define  WAITING_TIMEOUT_COUNTER__MAXVAL        40      //40*20ms=800ms==0.8S �ȴ�NEXT WORKING ������
//#define  WAITING_TIMEOUT_COUNTER__MAXVAL          80
//****************************************************************
#define  UART_RX_BUF_SIZE   64     //UART���ջ�������С,ͳһ����
#define  UART_TX_BUF_SIZE   64     //UART���ͻ�������С,ͳһ����
//***********************************

#define NO_SUCCESS   0
#define SUCCESS_OK   1
//*******************************
#define WIFI_DEVICE_OK           00 //�豸�������� 
#define IN_RESTORE_FACTORY       03 //wifiģ��,���ڻָ�����������
//*******************************
#define ZERO_FRAME       0
#define FRAME_OK         1
#define FRAME_ERROR      2
#define UART_CMD_ACK_OK           0     //ack oK
#define UART_CMD_ACK_WAIT         1     //ACK WAIT
#define UART_CMD_ACK_ERR          2     //ACK ERROR
//*******
#define  WaitRX                                 0X00   //����״
#define  PinResetWifiCard                       0X01   //ResetPin,��λWifiCard
//********
#define  TestConnectivity                       0X02
#define  TestConnectivityAck                    0X03
//*******
#define  ParaSaveToWifiE2ROM                    0X04
#define  ParaSaveToWifiE2ROMAck                 0X05
//*******
#define  DefaultIdle                            0X06   //����״
#define  ReadFromAPPKeySig                      0X07
#define  ReadFromAPPKeySigAck                   0X08
#define  WriteToAPPDisplay                      0X09
#define  WriteToAPPDispalyAck                   0X0A
//********
#define  RestoreFactorySet                      0X0B
#define  RestoreFactorySetAck                   0X0C

//*********************************************
#define  NullTestFunCode        0X00
#define  ReadMemFunCode         0X01
#define  WriteMemFunCode        0X02
#define  WriteMemEEROMFunCode   0X03     //дRAM,�Ҵ���EEROM
#define  EnReadMemFunCode       0X05    //���¼�Ӧ��� 
#define  EnWriteMemFunCode      0X06   //���¼�Ӧ��д 
#define  WReadMemFunCode        0X07    //������д�������� 
#define  WriteRMemFunCode       0X08   // д�������� 

#define  NullTestAckCode       (NullTestFunCode|0X80)
#define  ReadMemAckCode        (ReadMemFunCode|0X80)
#define  WriteMemAckCode       (WriteMemFunCode|0X80)
#define  WriteMemEEROMAckCode  (WriteMemEEROMFunCode|0X80)
#define  EnReadMemAckCode      (EnReadMemFunCode|0X80)
#define  EnWriteMemAckCode     (EnWriteMemFunCode|0X80)
#define  WReadMemAckCode       (WReadMemFunCode|0X80)
#define  WriteRMemAckCode      (WriteRMemFunCode|0X80)

#define  UartCmdFinOK          0X00   //=0X00,�������
#define  UartCmdFinErr1        0X01   //=0X01,������Ч��
#define  UartCmdFinErr3        0X03   //=0X03,����δ֪��

#define UART_CMD_MAX 14


							


/*!< Signed integer types  */



#define oData0Mode                0 //ȱʡloopģʽ
#define oData1Mode                1 //����Ƭ1,���
#define oData2Mode                2 //����Ƭ2,���
#define oData3Mode                3 //����Ƭ3,���
extern uint8_t DataModeFlag;

#define MapDatDelay 3
#define ONCE   2 


//**********************
extern uint8_t ExtCmdInserted; 
extern uint8_t  wifi_send_sleep_flag;
//**********************
uint8_t InsertExtCmd(uint8_t ExtCmd);
extern U16 get_uart_cmd(void);
//extern key_state_t *get_uart_cmd_state(void);
extern void uart_cmd_rounte(void);
extern u16 sys_uart_trig_tx_evnet(u16 uart_index);
extern void WifiData_Period_Process(void);
extern void UART_Config_PinResetWifiCard(void);
extern void IRQ_Usart1RxData_Process(uint8_t value);
//extern U16 get_Value_HeaterPower(void);
extern void set_wifi_enable(uint8_t value);
extern uint8_t get_wifi_enable_state(void);
extern void set_reset_wifi_flag(uint8_t value);
extern uint8_t get_reset_wifi_flag(void);
extern void exit_wifi(void);
extern void init_wifi_module(void);
extern void powerup_wifi_module(void);
extern U8 get_bat_level(void);
//********************

//**********************

//***********************
//***********

#endif


































