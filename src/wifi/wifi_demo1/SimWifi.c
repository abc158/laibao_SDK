#include <ui-config.h>
#include "ui-commands.h"
#include "am_date_base.h"
#include "am_config_macro.h"
#include "time.h"
#include "lib.h"
#include "local_key_check.h"
#include "syscall_api.h"
#include "remote.h"
#include "act.h"
#include "display.h"
#include "am_key.h"
#include "sensor/sensor.h"
#include "ui-manager/exception.h"
#include "motor/robot_suction.h"
#include "ui-song-player.h"
#include "display.h"
#include "ui_uart_handle/mqueue.h"
#include "simwifi.h"
#include "simsweep.h"
#include "charge/charge.h"
#if defined(USE_WIFI_DEMO_1)
#define WIFI_UART 1


//extern data U8 Fg_System20msSig; //��ֲʱ��������
//extern bit wifi_enable_flag;
extern u16 sys_uart_trig_tx_evnet(u16 uart_index);
extern schedule_time_info  obj_schedule_info_msg_t;
static U32 wifi_delay_ts = 0;
uint16_t Value_HeaterPower=0;
/*ʹ��wifi���� 0��ʹ�ܣ�1ʹ��*/
//uint8_t  enable_wifi = 1;
/*��λwifiģ���־��0����λ��1��λ��2�м�״̬*/
uint8_t  reset_wifi_flag = 0;
uint8_t  wifi_enable_flag = 0;
uint8_t  wifi_send_sleep_flag =0;
uint8_t UartRxBuf[UART_RX_BUF_SIZE];//UART���ջ�����
uint8_t P_RxBufWrite=0;//UART���ջ�����,дָ��
uint8_t fg_RXFrame=ZERO_FRAME;//UART����֡,״̬
uint8_t UartTxBuf[UART_TX_BUF_SIZE];//UART���ͻ�����
//uint16_t error_count = 0;

static sys_state_info sys_state_info_p; 
static sys_current_info sys_current_info_p;   /*ϵͳ����*/
void Usart1_SendByte(uint8_t Value);
/****************************************************
**����������Delay_nms
**�������������뼶��ʱ
**���������u32 m
**�����������   
*****************************************************/

#if 0
void Delay_N_mS(uint32_t m)
{
  uint16_t i,j;

  for(i=0;i<m;i++)
  {
    for(j=0;j<1200;j++);
  }
}
#endif

void ResetUartRxBuf(void)
{
#ifdef gzsdebug 
   uint8_t i;
    for(i=0;i<UART_RX_BUF_SIZE;i++)
   {
     UartRxBuf[i]=0;
   }
#endif   
   fg_RXFrame=ZERO_FRAME;//reset fg_RXFrame�ź�
   P_RxBufWrite=0;//reset P_RxBufWrite
 }
#ifdef gzsdebug 
void ResetUartTxBuf(void)
{
   uint8_t i;
    for(i=0;i<UART_TX_BUF_SIZE;i++)
   {
     UartTxBuf[i]=0;
   }
 }
#endif 

//********************
#ifdef gzsdebug
uint8_t DataTest=0;
#endif 
//****************

void StructureTestNullPack(void)
{
  uint16_t P_AddSum=0;
  uint8_t i;
  UartTxBuf[0]=0X55;
  UartTxBuf[1]=0XAA;
  UartTxBuf[2]=4;
  UartTxBuf[3]=NullTestFunCode; //������0X00// 
  UartTxBuf[2]+=2;  //����
  //P_AddSum=(uint16_t *)(UartTxBuf+4);
  //*P_AddSum=0;
  for(i=0;i<4;i++)
  {
    P_AddSum+=UartTxBuf[i];
  }  
 // printf("P_AddSum=%x\r\n",P_AddSum);
  UartTxBuf[4]=(P_AddSum>>8);
  UartTxBuf[5]=(P_AddSum&0x00ff);

//********************
#if 0
  //test++;
  uint8_t len=64;
  UartTxBuf[2]=len;
  P_AddSum=(uint16_t *)(UartTxBuf+len-2);
  *P_AddSum=0; 
  
   for(i=0;i<4;i++)
   {
     *P_AddSum+=UartTxBuf[i];
   }
   
  for(i=4;i<len-2;i++)
  {
     UartTxBuf[i]=i+DataTest;
     *P_AddSum+=UartTxBuf[i];
   }   
#endif 
   
//********************************
 
}

void StructureWifiCmdRegPack(uint8_t cmd)
{
  uint16_t P_AddSum=0;
  uint8_t i;
  UartTxBuf[0]=0X55;
  UartTxBuf[1]=0XAA;
  UartTxBuf[2]=7;
  UartTxBuf[3]=WriteMemFunCode; //������0X02
  UartTxBuf[4]=0XF0; //RamAdr 0XF0
  UartTxBuf[5]=8;    //8 Bit len
  UartTxBuf[6]=cmd; //CMD Reg
  UartTxBuf[2]+=2; //����    
  //P_AddSum=(uint16_t *)(UartTxBuf+7);
  //*P_AddSum=0;
  for(i=0;i<7;i++)
  {
    P_AddSum+=UartTxBuf[i];
  }  
  UartTxBuf[7] = (P_AddSum>>8);
  UartTxBuf[8] = (P_AddSum&0x00ff);
}

#define iData0Adr 0X00    //0 ������ APP�����ź�
//#define iData0BitLen 32    
//***********1V3_Begin
#define iData0BitLen    48    
//***********1V3_End

#define iData1Adr       0X08    //1 ������ ԤԼ����
#define iData1BitLen    64    
#define iData2Adr       0X10    //2 ������ ԤԼ����
#define iData2BitLen    64   
#define iData3Adr       0X18    //3 ������ ������Oem����
#define iData3BitLen    24
#define iData4Adr       0X20    //4 ������ ֡��ţ���ʷ��ͼ֡
#define iData4BitLen    48
#define iData5Adr       0X43    //5 ������ [�źű��Bit1,Bit0]=[0X19,0X18];[�źű��Bit2]=[0X1A]
#define iData5BitLen    8    
#define iData6Adr       0XF1    //6 ������ 0xF1 wificard status reg
#define iData6BitLen    8    
#define iData7Adr       0XF8    //7 ������ 0xF8 RTC
#define iData7BitLen    64   

//***************1V3_Begin
#define iData8Adr       0X1A    //8 ������ 0x1A ����
#define iData8BitLen    8   
//***************1V3_End

//**********************����,�ɲ����µ�������
  


#define iData0Index 5                        //��Ӧ���Ķ�Ӧ�ģ������±�λ��
#define iData1Index (iData0Index+iData0BitLen/8)
#define iData2Index (iData1Index+iData1BitLen/8)
#define iData3Index (iData2Index+iData2BitLen/8)
#define iData4Index (iData3Index+iData3BitLen/8)
#define iData5Index (iData4Index+iData4BitLen/8)
#define iData6Index (iData5Index+iData5BitLen/8)
#define iData7Index (iData6Index+iData6BitLen/8)
//***************1V3_Begin
#define iData8Index (iData7Index+iData7BitLen/8)
//***************1V3_End

//**********************����,�ɲ����µ�������  


void StructureReadAPPKeySigPack(void)  //���û�VAR,һ�ζ���
{
  uint16_t P_AddSum=0;
  uint8_t  index=0,i;
  UartTxBuf[index++]=0X55;
  UartTxBuf[index++]=0XAA;
  UartTxBuf[index++]=0;                 //pack len init=0;
  UartTxBuf[index++]=EnReadMemFunCode; //������0X05  
  UartTxBuf[index++]=iData0Adr;   
  UartTxBuf[index++]=iData0BitLen;  
  UartTxBuf[index++]=iData1Adr;  
  UartTxBuf[index++]=iData1BitLen;    
  UartTxBuf[index++]=iData2Adr; 
  UartTxBuf[index++]=iData2BitLen;  
  UartTxBuf[index++]=iData3Adr; 
  UartTxBuf[index++]=iData3BitLen;    
  UartTxBuf[index++]=iData4Adr; 
  UartTxBuf[index++]=iData4BitLen;  
  UartTxBuf[index++]=iData5Adr; 
  UartTxBuf[index++]=iData5BitLen;  
  UartTxBuf[index++]=iData6Adr; 
  UartTxBuf[index++]=iData6BitLen; 
   UartTxBuf[index++]=iData7Adr; 
  UartTxBuf[index++]=iData7BitLen;
//***************1V3_Begin  
  UartTxBuf[index++]=iData8Adr; 
  UartTxBuf[index++]=iData8BitLen; 
//***************1V3_End
  
  //**********************����,�ɲ����µ������� 
 
  UartTxBuf[2]=index+2;//pack len  
 // P_AddSum=(uint16_t *)(UartTxBuf+index);
 // *P_AddSum=0;
  for(i=0;i<index;i++)//�����ۼӺ�
  {
     P_AddSum+=UartTxBuf[i];
  }
  UartTxBuf[index]=(P_AddSum>>8);
  UartTxBuf[index+1]=(P_AddSum&0xff);
  
}

#define oData0Adr 0X50    //0 ״̬������ ɨ�ػ�״̬�ź�
#define oData0BitLen 32   
#define oData1Adr 0X58    //1 ״̬������ ɨ�ػ���ǰԤԼ����
#define oData1BitLen 64   
#define oData2Adr 0X60    //2 ״̬������ ɨ�ػ���ǰԤԼ����
#define oData2BitLen 64   
#define oData3Adr 0X68    //3 ״̬������ ɨ�ػ���Ʒ����,��ص�������������
#define oData3BitLen 32
//*************1V2_Begin
#define oData4Adr 0X54    //4 ״̬������ UV���ڷ񣬵��ִ��ڷ����ݿ����
#define oData4BitLen 8
#define oData5Adr 0X55    //5 ״̬������ ɨ�ػ���ͼ���
#define oData5BitLen 8
//**************1V2_End
#define oData10Adr 0X70    //6 ״̬������ ����λ��1
#define oData10BitLen 64   
#define oData11Adr 0X78    //7 ״̬������ ����λ��2
#define oData11BitLen 64   
#define oData12Adr 0X80    //8 ״̬������ ����λ��3
#define oData12BitLen 64   
#define oData13Adr 0X88    //9 ״̬������ ����λ��4
#define oData13BitLen 64

#define oData20Adr 0X90    //10 ״̬������ ��ȡλ��1
#define oData20BitLen 64   
#define oData21Adr 0X98    //11 ״̬������ ��ȡλ��2
#define oData21BitLen 64   
#define oData22Adr 0XA0    //12 ״̬������ ��ȡλ��3
#define oData22BitLen 64   
#define oData23Adr 0XA8    //13 ״̬������ ��ȡλ��4
#define oData23BitLen 64

#define oData30Adr 0XB0    //14״̬������ ��ʷ��ͼ1
#define oData30BitLen 64   
#define oData31Adr 0XB8    //15 ״̬������ ��ʷ��ͼ2
#define oData31BitLen 64   
#define oData32Adr 0XC0    //16 ״̬������ ��ʷ��ͼ3
#define oData32BitLen 64   
#define oData33Adr 0XC8    //17 ״̬������ ��ʷ��ͼ4
#define oData33BitLen 64


uint8_t DataModeFlag=1;
void StructureAPPDisplaySigPack(void)
{
  uint16_t P_AddSum=0;
  uint8_t i,index=0;
  UartTxBuf[index++]=0X55;
  UartTxBuf[index++]=0XAA;
  UartTxBuf[index++]=0;//init 0;  
  
  /*#define oData0Mode             0 //ȱʡloopģʽ
#define oData1Mode                1 //����Ƭ1,���
#define oData2Mode                2 //����Ƭ2,���
#define oData3Mode                3 //����Ƭ3,���*/ 
  
  switch(DataModeFlag)
  {
    case oData0Mode://�豸״̬����Ƭ
        UartTxBuf[index++]=EnWriteMemFunCode;         //������0X06,�¼�Ӧ��ģʽ
        UartTxBuf[index++]=oData0Adr;                 //����out Data0
        UartTxBuf[index++]=oData0BitLen;    
        Get_oData0_0X50_0X53(UartTxBuf+index,oData0BitLen/8);//get oData0
        index+=(oData0BitLen/8);
        
        UartTxBuf[index++]=oData1Adr;                 //����out Data1
        UartTxBuf[index++]=oData1BitLen;    
        Get_oData1_0X58_0X5F(UartTxBuf+index,oData1BitLen/8);//get oData1
        index+=(oData1BitLen/8);  
       
        UartTxBuf[index++]=oData2Adr;                 //����out Data2
        UartTxBuf[index++]=oData2BitLen;    
        Get_oData2_0X60_0X67(UartTxBuf+index,oData2BitLen/8);//get oData2
        index+=(oData2BitLen/8); 
        
        UartTxBuf[index++]=oData3Adr;                 //����out Data3
        UartTxBuf[index++]=oData3BitLen;    
        Get_oData3_0X68_0X6B(UartTxBuf+index,oData3BitLen/8);//get oData3
        index+=(oData3BitLen/8);  
       
        //******1V2_Begin
        UartTxBuf[index++]=oData4Adr;                 //����out Data4
        UartTxBuf[index++]=oData4BitLen;    
        Get_oData4_0X54_0X54(UartTxBuf+index,oData4BitLen/8);//get oData4
        index+=(oData4BitLen/8); 
       //*****1V2_End
        
        UartTxBuf[index++]=oData5Adr;                 //����out Data5
        UartTxBuf[index++]=oData5BitLen;    
        Get_oData5_0X55_0X55(UartTxBuf+index,oData5BitLen/8);//get oData5
        index+=(oData5BitLen/8);        
        break;
//***********************************  
    case oData1Mode:   //����λ������Ƭ
        UartTxBuf[index++]=WriteMemFunCode;         //������0X02
        UartTxBuf[index++]=oData10Adr;                 //����out Data6
        UartTxBuf[index++]=oData10BitLen;    
        Get_oData6_0X70_0X77(UartTxBuf+index,oData10BitLen/8);//get oData6
        index+=(oData10BitLen/8);
        
        UartTxBuf[index++]=oData11Adr;                 //����out Data7
        UartTxBuf[index++]=oData11BitLen;    
        Get_oData7_0X78_0X7F(UartTxBuf+index,oData11BitLen/8);//get oData7
        index+=(oData11BitLen/8);  
       
        UartTxBuf[index++]=oData12Adr;                 //����out Data8
        UartTxBuf[index++]=oData12BitLen;    
        Get_oData8_0X80_0X87(UartTxBuf+index,oData12BitLen/8);//get oData8
        index+=(oData12BitLen/8); 
        
        UartTxBuf[index++]=oData13Adr;                 //����out Data9
        UartTxBuf[index++]=oData13BitLen;    
        Get_oData9_0X88_0X8F(UartTxBuf+index,oData13BitLen/8);//get oData9
        index+=(oData13BitLen/8);  
        break;
//***********************************  
    case oData2Mode:  //��ȡλ������Ƭ   
        UartTxBuf[index++]=WriteMemFunCode;         //������0X02
        UartTxBuf[index++]=oData20Adr;                 //����out Data10
        UartTxBuf[index++]=oData20BitLen;    
        Get_oData10_0X90_0X97(UartTxBuf+index,oData20BitLen/8);//get oData10
        index+=(oData20BitLen/8);
        
        UartTxBuf[index++]=oData21Adr;                 //����out Data11
        UartTxBuf[index++]=oData21BitLen;    
        Get_oData11_0X98_0X9F(UartTxBuf+index,oData21BitLen/8);//get oData11
        index+=(oData21BitLen/8);  
       
        UartTxBuf[index++]=oData22Adr;                 //����out Data12
        UartTxBuf[index++]=oData22BitLen;    
        Get_oData12_0XA0_0XA7(UartTxBuf+index,oData22BitLen/8);//get oData12
        index+=(oData22BitLen/8); 
        
        UartTxBuf[index++]=oData23Adr;                 //����out Data13
        UartTxBuf[index++]=oData23BitLen;    
        Get_oData13_0XA8_0XAF(UartTxBuf+index,oData23BitLen/8);//get oData13
        index+=(oData23BitLen/8);  
        break;
//***********************************  
    case oData3Mode:  //��ʷ��ͼ����Ƭ    
        UartTxBuf[index++]=WriteMemFunCode;         //������0X02 
        UartTxBuf[index++]=oData30Adr;                 //����out Data14
        UartTxBuf[index++]=oData30BitLen;    
        Get_oData14_0XB0_0XB7(UartTxBuf+index,oData30BitLen/8);//get oData14
        index+=(oData30BitLen/8);
        
        UartTxBuf[index++]=oData31Adr;                 //����out Data15
        UartTxBuf[index++]=oData31BitLen;    
        Get_oData15_0XB8_0XBF(UartTxBuf+index,oData31BitLen/8);//get oData15
        index+=(oData31BitLen/8);  
       
        UartTxBuf[index++]=oData32Adr;                 //����out Data16
        UartTxBuf[index++]=oData32BitLen;    
        Get_oData16_0XC0_0XC7(UartTxBuf+index,oData32BitLen/8);//get oData16
        index+=(oData32BitLen/8); 
        
        UartTxBuf[index++]=oData33Adr;                 //����out Data17
        UartTxBuf[index++]=oData33BitLen;    
        Get_oData17_0XC8_0XCF(UartTxBuf+index,oData33BitLen/8);//get oData17
        index+=(oData33BitLen/8); 
       // test_print = 1;
        
        break;  
//***********************************  
    default:         
        break;        
  }      
  UartTxBuf[2]=index+2;   //pack len 
 // P_AddSum=(uint16_t *)(UartTxBuf+index);
 // *P_AddSum=0;
  for(i=0;i<index;i++)
   {
     P_AddSum+=UartTxBuf[i];
   } 
  UartTxBuf[index]=(P_AddSum>>8);
  UartTxBuf[index+1]=(P_AddSum&0xff);
  /*
  if(test_print == 1)
  {
    printf("send his data:\r\n");
     for(i=0;i<(index+2);i++)
   {
      printf("%x ",UartTxBuf[i]);
   } 
     test_print = 0;
  }*/
  
}


void SetPushFullDataFlag(uint8_t oDataMode )//Push FULL DATA FLAG
{                                              //������ݷ���
                                                //oDataMode==0��������loop ���
                                                //oDataMode==1,����Ƭ1,���
                                                //oDataMode==2,����Ƭ2,���
                                                //oDataMode==3,����Ƭ3,���
  DataModeFlag=oDataMode; //ָ������Ƭ�����ģʽ
}


void StructureE2ROM_ParaPack(void)
{
  uint16_t P_AddSum=0;
  uint8_t i;
  UartTxBuf[0]=0X55;
  UartTxBuf[1]=0XAA;
  UartTxBuf[2]=0;                  //�ݲ�=ֵ
  UartTxBuf[3]=WriteMemEEROMFunCode; //������0X03
  
  UartTxBuf[4]=0XF4; //RamAdr 0XF4
  UartTxBuf[5]=16;    //16 Bit len
 // *(uint16_t *)(UartTxBuf+6)=APP_COM_NUM; //�����
  UartTxBuf[6]=(APP_COM_NUM>>8);    //16 Bit len
  UartTxBuf[7]=(APP_COM_NUM&0x00ff);    //16 Bit len
  UartTxBuf[8]=0XF6; //RamAdr 0XF6
  UartTxBuf[9]=16;    //16 Bit len
  P_AddSum = Get_OEM_CODE();
  UartTxBuf[10]=(P_AddSum>>8);
  UartTxBuf[11]=(P_AddSum&0xff);
  //*(uint16_t *)(UartTxBuf+10)=Get_OEM_CODE();//OemCode=�������  
  UartTxBuf[2]=12;  
  UartTxBuf[2]+=2;
  //P_AddSum=(uint16_t *)(UartTxBuf+12);
  P_AddSum=0;
  for(i=0;i<12;i++)
   {
     P_AddSum+=UartTxBuf[i];
   }
  UartTxBuf[12]=(P_AddSum>>8);
  UartTxBuf[13]=(P_AddSum&0xff);
 
}


//extern void init_wifi_uart(void);
void UART_Config_PinResetWifiCard(void)
{
  volatile uint32_t time = 2000000;
  init_wifi_module(); 
  while(time--);
  powerup_wifi_module(); 
  InsertExtCmd(ParaSaveToWifiE2ROM);//��ʼ��������ţ��׸�����
  wifi_delay_ts = timer_ms();
}

/*****************************************************
**����������Usart2_SendByte
**��������������2����1�ֽ�
**���������Value 
**����������� 
*****************************************************/
void Usart1_SendByte(uint8_t Value)
{
  // senduart2(Value);
}

/*****************************************************
**����������Usart1_SendHexString
**��������������2���Ͷ�����������
**���������u8 *pData,u32 Long(�����С)
**����������� 
*****************************************************/
extern void set_wifi_uart_tx_fifo_empty_irq(void);
void Usart1_SendHexString(uint8_t *pHexBuf,uint32_t Long)
{
  uint32_t i;
 // printf("send: ");
  for(i=0;i<Long;i++)
  {
   // printf("%x ",pHexBuf[i]);
    uart_txrx_q_put(WIFI_TX_INDEX,pHexBuf[i]);
  }
 // printf("\r\n");
  set_wifi_uart_tx_fifo_empty_irq();
  //sys_uart_trig_tx_evnet(WIFI_UART);
}

void  SendTestNullPack(void)
{
  StructureTestNullPack();          //������Կհ�
  Usart1_SendHexString(UartTxBuf,UartTxBuf[2]);
}
void SendCmdRegPack(uint8_t cmd)
{
   StructureWifiCmdRegPack(cmd);
   Usart1_SendHexString(UartTxBuf,UartTxBuf[2]);
}

void SendAPPDisplaySigPack(void )
{
   StructureAPPDisplaySigPack();
   Usart1_SendHexString(UartTxBuf,UartTxBuf[2]);
} 
void SendReadAPPKeySigPack(void )
{
   StructureReadAPPKeySigPack();//���������
   Usart1_SendHexString(UartTxBuf,UartTxBuf[2]);
}
void SendParaSaveToWifiE2ROMPack(void )
{
    StructureE2ROM_ParaPack();
    Usart1_SendHexString(UartTxBuf,UartTxBuf[2]);      
} 

/*****************************************************
**����������IRQ_Usart2RxData_Process
**�����������жϴ���2�������ݴ���
**�����������
**����������� 
*****************************************************/
uint8_t FrameLen=0; 
uint16_t FrameAddSum=0;
void IRQ_Usart1RxData_Process(uint8_t value)
{
  uint8_t Dat;
  uint16_t check_sum=0;
  //UART1_GetITStatus( UART1_IT_OR);
  Dat=value; //readme Uart  
    //printf("DAT=%x\r\n",Dat);
  if((fg_RXFrame==FRAME_ERROR)||(fg_RXFrame==FRAME_OK)) //stop RecFrame ֱ��main���������֡�ź� 
         return;   
  switch(P_RxBufWrite)
  {
  case 0:
      if(Dat==0XAA) //frame head syn
      {
        UartRxBuf[P_RxBufWrite]=Dat;
        FrameAddSum=0XAA;
        P_RxBufWrite++;  
      }
      break;
  case 1:  
      if(Dat==0X55)//frame head syn
      {
        UartRxBuf[P_RxBufWrite]=Dat;
        FrameAddSum+=Dat;       
        P_RxBufWrite++;
       // check_count = 0;
      }
      else
      {
        P_RxBufWrite=0;//reset P_RxBufWrite
      }
      break;
  case 2:                   //frame len
      if(Dat<4)//FrameLen<4
      {
           P_RxBufWrite=0;           //reset P_RxBufWrite       
      }
      else
      {  
          FrameLen=Dat;
          UartRxBuf[P_RxBufWrite]=Dat;
          FrameAddSum+=Dat;       
          P_RxBufWrite++;
      }
      break;    
  default:    
      UartRxBuf[P_RxBufWrite]=Dat;   //uart ������ 
      if(P_RxBufWrite<FrameLen-2)  //�ۼӺͣ�����
         FrameAddSum+=Dat;
      P_RxBufWrite++;
      if(FrameLen==P_RxBufWrite)//
      {   
          check_sum = (UartRxBuf[FrameLen-2]<<8)|(UartRxBuf[FrameLen-1]);
          if(check_sum ==FrameAddSum ) 
            fg_RXFrame=FRAME_OK;//UART����֡,����Frame_OK�ź�
          else
            fg_RXFrame=FRAME_ERROR;//UART����֡,�ۼӺʹ�       
      }
      else if (P_RxBufWrite>FrameLen)
      {  
          fg_RXFrame=FRAME_ERROR;//UART����֡,����FrameError�ź�  
          P_RxBufWrite=0;           //reset P_RxBufWrite
      }        
      break; 
  }      
  if(P_RxBufWrite>(UART_RX_BUF_SIZE-1))//�������ջ�������С,�����UARTRXBUFSIZE��С
  {
    P_RxBufWrite=0;                     //reset P_RxBufWrite
  }  
}


/*****************************************************
**����������ReadData_From_UartRxBuf
**�����������ӽ��ջ�������ȡ����,�Ҵ���
**���������CmdAckFun_8,ack������
**�������������ACK״̬
*****************************************************/
uint8_t ReadData_From_UartRxBuf(uint8_t CmdAckFun_8)
{
 
  if(fg_RXFrame==ZERO_FRAME) //wait 
        return UART_CMD_ACK_WAIT;
  
  if((CmdAckFun_8!=UartRxBuf[3])|(fg_RXFrame==FRAME_ERROR)|(UartRxBuf[4]!=UartCmdFinOK))//�ж��Ƿ��ǵ�ǰҪACK,ACK�Ƿ������
  {
        fg_RXFrame=ZERO_FRAME;//reset fg_RXFrame�ź�
        P_RxBufWrite=0;//reset P_RxBufWrite
        return UART_CMD_ACK_ERR;
  } 
    
   return UART_CMD_ACK_OK;
}




uint8_t AppInstalledFlag=1;//=0,�豸���ڳ���״̬��1=��ʾ�Ѱ�װAPP���ɹ���,Ĭ��
void ProcessAPPKeySig (void)
{
//  uint8_t tmp;
  uint8_t APPinstalledEvent=0;//APP ��װ�ɹ��¼�ָʾ  
#ifdef gzsdebugOut  
  uint8_t ii=0,AdrHex[2];
  //*************1V3_Begin
  for(ii=0;ii<iData0BitLen/8;ii++)     
   //************1V3_End
  {
    AdrHex[0]=ii;  
    if(UartRxBuf[iData0Index+ii]) 
    {
      AdrHex[1]=UartRxBuf[iData0Index+ii];
      Value_HeaterPower=*(uint16_t*)AdrHex;
      break;
    } 
  }
#endif

    
 //5��λ�ź�,����  iData0Index 
  if(UartRxBuf[iData0Index+0])          Action_0X00(UartRxBuf[iData0Index+0]);      //1Byte
  if(UartRxBuf[iData0Index+1])          Action_0X01(UartRxBuf[iData0Index+1]);      //1Byte
  //if(UartRxBuf[iData0Index+4]&0X20)     Action_0X1B_0X1E(UartRxBuf+iData3Index+4);  //4Byte
  if(UartRxBuf[iData0Index+2])          Action_0X02(UartRxBuf[iData0Index+2]);      //1Byte  
  if(UartRxBuf[iData0Index+3]&0XFB)     Action_0X03(UartRxBuf[iData0Index+3]&0XFB); //1Byte   
 // ԤԼ�ر���  iData1Index,iData2Index
  if(UartRxBuf[iData0Index+3]&0X04)     Action_0X08_0X17(UartRxBuf+iData1Index,16); //16Byte 
  
  if(UartRxBuf[iData0Index+4])          Action_0X04(UartRxBuf[iData0Index+4]);      //1Byte 
  if(UartRxBuf[iData0Index+5])          Action_0X05(UartRxBuf[iData0Index+5]);      //1Byte
  
  if(UartRxBuf[iData0Index+4]&0X20)     Action_0X20_0X21(UartRxBuf+iData4Index,2);  //2Byte
  if(UartRxBuf[iData0Index+5]&0X20)     Action_0X20_0X25(UartRxBuf+iData4Index,6);  //6Byte
  
  
 //�����̴���  iData3Index ,iData5Index,2�ֽ�
  if(UartRxBuf[iData5Index+0]&0X03)     //0X43=iData5
  {
    Action_0X18_0X19(*(uint16_t*)(UartRxBuf+iData3Index)); //2Byte 
    InsertExtCmd(ParaSaveToWifiE2ROM);//�����ⲿ����ParaSaveToWifiE2ROM
  }   
 //************1V3_Begin 
  //�����趨
  if(UartRxBuf[iData5Index+0]&0X04)    //0X43 =iData5
  {
    Action_0X1A(UartRxBuf[iData8Index]); //1Byte 
   }   
 //************1V3_End    
 
  //wificard status  iData6Index 
  if((UartRxBuf[iData6Index+0]&0X03)==0X00) 
    {
      if(AppInstalledFlag==0)         
        APPinstalledEvent=1;
      AppInstalledFlag=1;      
    } 
  else 
     AppInstalledFlag=0;
     Action_0XF1(UartRxBuf[iData6Index]);//1Byte
 //RTC  iData7Index  
  Action_0XF8_0XFF(UartRxBuf+iData7Index,8,APPinstalledEvent);  //8Byte  
}


U16 voltage_adc[] = 
{
    /*0% 5%   10%  15%  20%*/
    2739,2822,2859,2887,2909,
    /*25% 30%  35%  40%  45%*/
    2928,2944,2959,2975,2993,
    /*50% 55%  60%  65%  70%*/
    3013,3038,3069,3104,3145,
    /*75% 80%  85%  90%   95%*/
    3189,3233,3260,3320,3347,
    /*100%*/
    3390
};

U16 charge_voltage_adc[] = 
{
    /*0% 5%   10%  15%  20%*/
    2739,2822,3009,3050,3100,
    /*25% 30%  35%  40%  45%*/
    3200,3300,3350,3395,3440,
    /*50% 55%  60%  65%  70%*/
    3470,3485,3500,3510,3528,
    /*75% 80%  85%  90%   95%*/
    3540,3558,3575,3590,3600,
    /*100%*/
    3628
};
static U8  now_electricity = 0;
static U8  last_electricity = 0;

void calc_electricity(U16  voltage)
{
    U8 i;
    last_electricity = now_electricity;
    //sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
    for(i=0;i<21;i++)
    {
        if(sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)
        {
            if(voltage_adc[i] > voltage)
            {
               break;
            }
        }
        else
        {
            if(charge_voltage_adc[i] > voltage)
            {
                break;
            }
        }
    }
    if(i == 0)
    {
        now_electricity = 0;
    }
    else if(i == 21)
    {
        now_electricity = 100;
    }
    else
    {
        now_electricity = 5*(i-1);
    }

    if(((now_electricity < last_electricity) && (sys_state_info_p.robot_state == ROBOT_STATE_CHARGING))
        || ((now_electricity > last_electricity) && (sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)))
    {
        if(last_electricity > 0)
        {
            now_electricity = last_electricity;
        }
    }

    if(sys_state_info_p.charging_state == CHARGING_COMPLETE)
    {
        now_electricity = 100;
    }

    
}
U8 get_bat_level(void)
{
    return now_electricity;
}

/*****************************************************
**����������WifiData_Period_Process
**����������Wifiģ�����ݴ���
**�����������
**����������� 
*****************************************************/

uint8_t ExtCmdInserted=DefaultIdle; //����WifiCard ״̬������,����                                    

uint8_t WorkingStatus=TestConnectivity;  //״̬��
uint8_t NextWorkingStatus=DefaultIdle;   //Ŀ��̬

uint8_t WaitingRxPackCounter=0;     //1000mS=20mS*50
uint8_t RxPackTimeOutErrorCounter=0;    //״̬����ʱ�����
uint8_t OldSystem20msSig=0;          //20ms ϵͳʱ����ֵ
uint8_t flag=1;//debug
uint8_t MapDatFlag=OFF;
uint8_t MapFunFlag=OFF;
U8      MapDatCount=0;
extern uint8_t  Sim_oData4[1];

U16 convert_to_adc( s16 voltage) 
{
    U32 mv = voltage<<12;
    mv = mv/18975;//0.18 ---> 18032; 0.174 --> 18975       
    return (U16)mv;
}

void WifiData_Period_Process(void)
{
  
  uint8_t UartCmdAck;
  static U16  voltage_average = 0;
  U16   tmp_voltage = 0;
  static U16   tmp_cnt = 0;
  static U8   first_check_voltage_flag = 0;
  sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
  sys_info_get(SYS_CURRENT_INFO,(long)&sys_current_info_p);

  tmp_voltage = convert_to_adc(sys_current_info_p.battery_voltage);

  voltage_average += tmp_voltage; 

 
    if(voltage_average >= 5000)
    {
	    voltage_average = (voltage_average>>1);
    }

    tmp_cnt += 1;
    if((tmp_cnt >= 2000)||((first_check_voltage_flag == 0)&&(tmp_cnt == 20)))
    {
        calc_electricity(voltage_average);
        tmp_cnt = 0;
        first_check_voltage_flag = 1;
    }

    if(wifi_send_sleep_flag == 1)
    {
       // return;
    }
 // if(OldSystem20msSig != Fg_System20msSig) //����--
 // {
      //OldSystem20msSig=Fg_System20msSig;
      if(WaitingRxPackCounter) WaitingRxPackCounter--;
  //}
/*
#ifndef FULL_FAST_RUN
  else            //20ms ִ��һ�Σ��źŶ�д
        return ;        
#endif
*/
  if(timer_elapsed(wifi_delay_ts) > 130)
  {
    if(MapDatCount > 0)
    {
      //  printf("MapDatCount--\r\n");
      MapDatCount--;
    }
    
    wifi_delay_ts = timer_ms();
  }
  
  switch(WorkingStatus)
   {
     
//*****************WaitRX �м�״̬    
       case WaitRX:
          switch(NextWorkingStatus)
          {
              case TestConnectivityAck:
                    UartCmdAck=ReadData_From_UartRxBuf(NullTestAckCode);  break;                   
              case ReadFromAPPKeySigAck:      
                    UartCmdAck=ReadData_From_UartRxBuf(EnReadMemAckCode);  break;                 
              case WriteToAPPDispalyAck: 
                    if(DataModeFlag==oData0Mode)
                    {UartCmdAck=ReadData_From_UartRxBuf(EnWriteMemAckCode);}
                    else
                    {UartCmdAck=ReadData_From_UartRxBuf(WriteMemAckCode);}
                    break;                 
              case RestoreFactorySetAck: 
                    UartCmdAck=ReadData_From_UartRxBuf(WriteMemAckCode);  break;                 
              case ParaSaveToWifiE2ROMAck:                          
                    UartCmdAck=ReadData_From_UartRxBuf(WriteMemEEROMAckCode);  break; 
              default: 
                    WorkingStatus=DefaultIdle;
                    NextWorkingStatus=DefaultIdle;   
                    break;   
          }      
          
          if (UartCmdAck==UART_CMD_ACK_OK)  //fin ok 
          { 
            RxPackTimeOutErrorCounter=0;
            WaitingRxPackCounter=0;
            WorkingStatus=NextWorkingStatus;//jump�¸�״̬
            NextWorkingStatus=DefaultIdle;//�ָ��ȴ�״
            flag=1;//debug
          }
          else if((UartCmdAck==UART_CMD_ACK_WAIT)&&(WaitingRxPackCounter!=0))
          {  
               if((WaitingRxPackCounter<WAITING_TIMEOUT_COUNTER__MAXVAL-10)&&(flag==1))//debug
               {
                      flag=0;
                      WaitingRxPackCounter=WaitingRxPackCounter;}
          }
          else         
         {  
                  RxPackTimeOutErrorCounter++;    //�Ƴ������                                              
                  WorkingStatus=TestConnectivity;//��ͨ�Բ���
                  WaitingRxPackCounter=0;   
                  ResetUartRxBuf();//reset rxbuf
                  if(RxPackTimeOutErrorCounter==200)//Wificard �����Զ���λ 10*0.8S=8S
                  {
                     RxPackTimeOutErrorCounter=0;
                     WorkingStatus=PinResetWifiCard;
                     //UART_Config_PinResetWifiCard();//NeedTest 
                  }   
          }  
          break; 
//************ PinReset WifiCard         
          
      case PinResetWifiCard: 
          //UART_Config_PinResetWifiCard();  //cold ������[��������������],NeedTest
          WorkingStatus=TestConnectivity; //������ͨ��
          ResetUartRxBuf();//reset rxbuf    
          break;        

//*******Test Connectivity         
     case TestConnectivity:
          SendTestNullPack();
          WorkingStatus=WaitRX;//wait ���,�м�״
          NextWorkingStatus=TestConnectivityAck;//��ȷ��ɣ�Ǩ��״̬������Ӧ����
          WaitingRxPackCounter=WAITING_TIMEOUT_COUNTER__MAXVAL;//��ʱ��ʱ��λ
          break;
     case TestConnectivityAck:  //��ͨ�Բ��ԣ�ACK
          WorkingStatus=DefaultIdle;
          ResetUartRxBuf();//reset rxbuf         
          break; 
          
//************Para Save To WifiE2ROM
     case ParaSaveToWifiE2ROM:
          SendParaSaveToWifiE2ROMPack();
          WorkingStatus          =WaitRX;//wait ���,�м�״ 
          NextWorkingStatus      =ParaSaveToWifiE2ROMAck;//Ŀ��״
          WaitingRxPackCounter=WAITING_TIMEOUT_COUNTER__MAXVAL;//��ʱ��ʱ��λ
          break;    
     case ParaSaveToWifiE2ROMAck: //�����д,��OEM����     
          WorkingStatus=DefaultIdle;
          ResetUartRxBuf();//reset rxbuf         
          break;               
//*************Default Idle          
     case DefaultIdle:
          if(ExtCmdInserted==DefaultIdle)//���ⲿ����
          {

             WorkingStatus=ReadFromAPPKeySig;
          }
          else  //����һ���ⲿcmd ask and ack
          {  
             WorkingStatus=ExtCmdInserted;
             ExtCmdInserted=DefaultIdle;//reset �ⲿ����
          }            
          break;        

 //****** Read  APP KeySig      
     case ReadFromAPPKeySig:       
#ifdef  gzsdebug      
          ResetUartTxBuf();
#endif        
          SendReadAPPKeySigPack();
					
          WorkingStatus=WaitRX;//wait ���,�м�״          
          NextWorkingStatus=ReadFromAPPKeySigAck;//Ŀ��״
          WaitingRxPackCounter=WAITING_TIMEOUT_COUNTER__MAXVAL;//��ʱ��ʱ��λ
          break;
     case ReadFromAPPKeySigAck:
 //******************************* 1V2_Begin    
          WorkingStatus=WriteToAPPDisplay;   //ȱʡ����WriteToAPPDisplay
          
          ProcessAPPKeySig();                //�ڲ����ܸ���ΪWorkingStatus
          
          
          
          if(MapFunFlag!=ONCE)//�жϵ�ͼ����Ƭ�ǻ�ȡ��MapFunFlag=ONCE)��������(MapFunFlag=ON)
          {
            if(Sim_oData4[0]&0x14)MapFunFlag=ON;
            else MapFunFlag=OFF;
          }
          
          if(MapFunFlag==ON) //�����ͼ����Ƭ����Ϊ���ͣ����ͼ����Ƭÿ300ms����һ�Σ�
                             //����һ�κ�תΪ���ͻ���״̬
          {
            if(MapDatCount==0){MapDatFlag=ON;MapDatCount=MapDatDelay;}
            else MapDatFlag=OFF;
          }
          else if(MapFunFlag==ONCE)//�����ͼ����Ƭ����Ϊ��ȡ�����ͼ����Ƭֻ���һ�Ρ�
          {
            MapFunFlag=OFF;
            MapDatFlag=ON;
          }
          else MapDatFlag=OFF;            
                    
          if(MapDatFlag==ON)SetPushFullDataFlag(oData1Mode);
          else SetPushFullDataFlag(oData0Mode);   
          
          
          
          
          ResetUartRxBuf();//reset rxbuf  
          break;
 //*******Write APP Display Sig         
     case WriteToAPPDisplay: 
       
          SendAPPDisplaySigPack();
          WorkingStatus=WaitRX;//wait ���,�м�״   
          NextWorkingStatus=WriteToAPPDispalyAck;//Ŀ��״
          WaitingRxPackCounter=WAITING_TIMEOUT_COUNTER__MAXVAL;//��ʱ��ʱ��λ
          break;  
     case WriteToAPPDispalyAck:    //var д���         
           //д��APP����������������ͼ����ʱ��ͼ�������Ƭ������״̬ѭ��������޵�ͼ����ʱֻ�������״̬����Ƭ��
          //��ͼ����Ƭ�����Ƭѭ������������oData1Mode��֮��oData2Mode��֮��oData3Mode��֮��oData0Mode��oData0Mode�������һ��������� 
          //��ͼ����Ƭ���һ���ֻغ���һ����ΪDefaultIdle;
          
          
          if(MapDatFlag==ON)
          {
            if(DataModeFlag==oData1Mode)                       
            {  
              SetPushFullDataFlag(oData2Mode);
              WorkingStatus=WriteToAPPDisplay;           //ÿ������Ƭѭ�������ݿ�δ�����꣬����һ����Ϊд��APP 
              ResetUartRxBuf();//reset rxbuf
            }
            else if(DataModeFlag==oData2Mode)                                           
            {              
              SetPushFullDataFlag(oData3Mode);
              WorkingStatus=WriteToAPPDisplay;           //ÿ������Ƭѭ�������ݿ�δ�����꣬����һ����Ϊд��APP 
              ResetUartRxBuf();//reset rxbuf
            }  
            else if(DataModeFlag==oData3Mode)                                           
            {
              SetPushFullDataFlag(oData0Mode);
              WorkingStatus=WriteToAPPDisplay;           //ÿ������Ƭѭ�������ݿ�δ�����꣬����һ����Ϊд��APP 
              ResetUartRxBuf();//reset rxbuf            
            }  
            else if(DataModeFlag==oData0Mode)                                           
            {  
              SetPushFullDataFlag(oData1Mode);
              WorkingStatus=DefaultIdle;              
              ResetUartRxBuf();//reset rxbuf             
            }  
            else
            {
              SetPushFullDataFlag(oData1Mode);
              WorkingStatus=DefaultIdle;              
              ResetUartRxBuf();//reset rxbuf            
            }  
          }
          else
          {
              SetPushFullDataFlag(oData0Mode);
              WorkingStatus=DefaultIdle;               //ÿ������Ƭѭ�������ݿ�δ�����꣬����һ����Ϊд��APP 
              ResetUartRxBuf();//reset rxbuf 
          }
          
          
          break;    
//*********Restore FactorySet WifiCard
          
     case RestoreFactorySet:
          SendCmdRegPack( 0X80);//reset wificard gzsdebug          
          /*<BIT7>��                                     0X80
=1���ָ�wifi_card��������
=0���ָ�wifi_card�������óɹ�,WIFI��CARD�Զ���λ
<BIT0>��reboot_wifi_sys                                  0X01
=1��hot����wifi����
=0��hot reboot ���,WIFI��CARD�Զ���λ
          */
          WorkingStatus        =WaitRX;//wait ���,�м�״ 
          NextWorkingStatus    =RestoreFactorySetAck;//Ŀ��״
          WaitingRxPackCounter=WAITING_TIMEOUT_COUNTER__MAXVAL;  //��ʱ��ʱ��λ              
          break; 
     case RestoreFactorySetAck://����Ĵ���д 
          WorkingStatus=DefaultIdle;
          ResetUartRxBuf();//reset rxbuf         
          break;  
//*********************        
     default:
          WorkingStatus=DefaultIdle; 
          break;        
   } 

  
}

uint8_t InsertExtCmd(uint8_t ExtCmd)
{
   uint8_t status=SUCCESS_OK;
	
  if(ExtCmdInserted==DefaultIdle) //�ɽ����ⲿ����
  {
     ExtCmdInserted=ExtCmd;       //reset �ⲿ����
     printf("reset wifi cmd!!\r\n");
  }
  else  
     status=NO_SUCCESS;
  return  status;
}




 
void set_wifi_enable(uint8_t value)
{
	
	if(value)
	{
		wifi_enable_flag=1;
        gpio_set_value(AM_IO_WIFI_POWER,1);
	}
	else
	{
		wifi_enable_flag=0;
        gpio_set_value(AM_IO_WIFI_POWER,0);
	}
	
	
}
/*
void init_wifi_uart(void)
{
    set_wifi_enable(1);
    wifi_delay_ts = allwork_timer_ms();
    while(allwork_timer_elapsed(wifi_delay_ts) < 3);
    UART_Config_PinResetWifiCard();
    wifi_delay_ts = allwork_timer_ms();
}*/
extern int gpio_direction_output(unsigned gpio, int value);
void init_wifi_module(void)
{
    gpio_request_one(AM_IO_WIFI_POWER,GPIO_F_DIR_OUT|GPIO_F_INIT_LOW);
    gpio_request_one(AM_IO_RESET_WIFI,GPIO_F_DIR_OUT|GPIO_F_INIT_LOW);
    gpio_set_value(AM_IO_WIFI_POWER,1);
    gpio_set_value(AM_IO_RESET_WIFI,0);
}

void powerup_wifi_module(void)
{
    gpio_set_value(AM_IO_WIFI_POWER,1);
    gpio_set_value(AM_IO_RESET_WIFI,1);
  //  gpio_set_value(GPIOD(12), 1);
 //   gpio_set_value(GPIOD(13), 1);
   // gpio_set_value(GPIOD(9), 1);
    
}

uint8_t get_wifi_enable_state(void)
{
    return wifi_enable_flag;
}

void set_reset_wifi_flag(uint8_t value)
{
    reset_wifi_flag = value;
}

uint8_t get_reset_wifi_flag(void)
{
    return reset_wifi_flag;
}

void exit_wifi(void)
{
   // uartwifi_init();
   
    set_wifi_enable(0);
    
}

#endif

