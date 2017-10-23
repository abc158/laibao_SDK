//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/////////������оƬ��IO�ڶ���/////////////////////////////////

//#define MCU_ID 0X1FFFF7E8  // STM32 ID��ʼ��ַ����96bit
#define MCU_ID 0X1FFFF7E2  // STM32 ID��ʼ��ַ����96bit
#define PARTNERSHIP  0Xff

////////////////////////ͷ�ļ�////////////////////////////////////
#include "syscall_api.h"
#include "am_exti_cfg.h"
#include "am_device.h"
#include "ui-ir.h"
#include "ARF2496K.H"
#include "ui-config.H"
#include "act.h"
#include "syscall_api.h"
#include "docking-new.h"

#ifdef IR_WIRELESS

////////////////////////ȫ�ֱ���//////////////////////////////////
RF_DATA rf_data;	// RF����
u8 rssi_val[10];	// �ź�ǿ��
u8 rssi_cnt;
u8 pipe_numb;  // ���Ͷ˵�ַ
u8 rf_recvbuff[16];
u8 rf_sendbuff[16]={0x98,0x76,0x54,0x32,0x76,0x54,0x32,0x76,0x54,0x32,0x76,0x54,0x54,0x32,0x76,0x54};
u8 rf_askbuff[5] = {0,0,0,0,0};

extern u8 get_sleep_level(void);
extern int hal_id_to_gpio_num(int id);

static volatile bool RFInceptSign;//�����źţ�Ϊtrue��ʾ���յ�һ���źţ���ִ�ж�������ʱ���������

static const u8 PartnershipAddress[6] ={0x98,0x76,0x54,0x32,HOST_ADDR,SLAVE_ADDR};
static u8 RFGroupAddress[4] = {0x98,0x76,0x54,0x32};//������ַ
static u8 RFDeviceAddress = HOST_ADDR;//������ַ
static u8 RFSlaveAddr = SLAVE_ADDR;//�ֻ���ַ
static unsigned char rf_channel = 0x44; // ͨѶƵ��

static volatile bool EnableRFState;//�Ƿ�ʹ�����߽��ա���оƬ����һЩ�������Զ��������ģʽ���߽������ģʽ��
u8 check_partnership_ok_flag = 0;
u8 signal_flag=0;
U32 ARF2496k_id = 0;

int arf2498_spi_id = 0;


extern int gpio_direction_input(unsigned gpio);
extern int gpio_direction_output(unsigned gpio, int value);
extern u16 system_spi_tx_fifo_full(void);
extern u16 system_spi_rx_fifo_empty(void);
extern uint8_t remote_ir_get(IR_REMOT_POSITION_E index);
s16 afio_mutex_mod_get(u32 id);
void ir_rx_code_get(u8 chan, u8 val);

const static u8  RF_InitTable[] =
{
  0x00,ARF2498_CHANNEL,
  0x03,0x6f,		//  ���ַ4���ֽ� ,0DB, 2byte-CRC
  0x04,0xff,		//  ��������ͨ��
  0x05,0x00,		//  ��ֹ����ͨ���Զ�Ӧ��
  0x06,0x00,		//  ��ֹ����ͨ����̬����
  0x07,0x11,		//  �����ظ����͵ȴ�ʱ�� 500us+250us*Resend_cnt
  //	0x08,0x30,		//  �Զ��ظ�����3��
  0x08,0x30,		//  �Զ��ظ�����3��
  0x16,0x10,		//   ���ش�С��16���ֽ�
  0x17,0x10,		//
  0x18,0x10,		//
  0x19,0x10,		//
  0x1a,0x10,		//   ���ش�С��16���ֽ�
  0x1b,0x10,		//
  0x1c,0x10,		//
  0x1d,0x10,		//

  //0x20,0x20,		//  ʹ���Զ�ˢ���ź�ǿ��
  0x21,0x04,		//  LNA������
  0xaa,0xaa,
};
/////////////////////////////////////////////////////////////
//��ʱ����
static void delay(u32 dl)
{
	 /*
  dl = dl*3;
  for(; dl != 0; dl--);
  */
  u16 i;
  for(i=0;i<0xD56;i++)//1us
  {
    ;
  }
}
static void delay_us(u16 us)
{
   u16 i;
   for(i=0;i<us;i++)
   delay(1);
}
static void delay_ms(u8 ms)
{
  delay_us(ms*1000);
}

//��1
void SetARF2498_MISO_Port(void)
{
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 1);
  //gpio_set_value(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 1);
   gpio_set_value(AM_I0_FLASH_DATA_O, 1);
}
//��0
void ResetARF2498_MISO_Port(void)
{
 // gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 0);
 // gpio_set_value(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 0);
 gpio_set_value(AM_I0_FLASH_DATA_O, 0);
}

//��1
void SetARF2498_MOSI_Port(void)
{
 //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_O), 1);
 // gpio_set_value(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_O), 1);
   gpio_set_value(AM_I0_FLASH_DATA_I, 1);
 // gpio_direction_output(GPIOF(14), 1);
  //i0_hal_digital_io_set(GPIOF(14) ,1);
}
//��0
void ResetARF2498_MOSI_Port(void)
{
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_O), 0);
  //gpio_set_value(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_O), 0);
  gpio_set_value(AM_I0_FLASH_DATA_I, 0);
  // gpio_direction_output(GPIOF(14), 0);
//i0_hal_digital_io_set(GPIOF(14) ,0);

}

//��1
void SetARF2498_SCK_Port(void)
{
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_CLOCK), 1);
  gpio_set_value(AM_I0_FLASH_CLOCK, 1);
}
//��0
void ResetARF2498_SCK_Port(void)
{
// gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_CLOCK), 0);
  gpio_set_value(AM_I0_FLASH_CLOCK, 0);
}

//��1
void SetARF2498_CS_Port(void)
{
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_WIRELESS_SPIEN), 1);
  //gpio_set_value(hal_id_to_gpio_num(HAL_I0_WIRELESS_SPIEN), 1);
  gpio_set_value(AM_IO_ARF2498_SPI_EN, 1);
}
//��0
void ResetARF2498_CS_Port(void)
{
 //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_WIRELESS_SPIEN), 0);
 // gpio_set_value(hal_id_to_gpio_num(HAL_I0_WIRELESS_SPIEN), 0);
 gpio_set_value(AM_IO_ARF2498_SPI_EN, 0);

}



//��1
void SetARF2498_CE_Port(void)
{
//  gpio_direction_output(hal_id_to_gpio_num(HAL_I0_LED_DIG1), 1);
   gpio_set_value(AM_I0_WIRELESS_SPIEN, 1);
}
//��0
void ResetARF2498_CE_Port(void)
{
 // gpio_direction_output(hal_id_to_gpio_num(HAL_I0_LED_DIG1), 0);
    gpio_set_value(AM_I0_WIRELESS_SPIEN, 0);
}

u8 ReadARF2498_MISO_Port(void)
{
  return gpio_get_value(AM_I0_FLASH_DATA_O);
}
/***********************************************************
��ʼ��RFӲ����IO��
***********************************************************/
void InitRfHardWare(void)
{
#ifdef WIRELESS_SIMULATE_SPI

  arf2498_spi_id = spi0_id_get();

  irq_io_set_vaule(AM_I0_FLASH_DATA_I, 0);
  irq_io_set_vaule(AM_I0_FLASH_CLOCK, 0);
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 0);
  irq_io_set_vaule(AM_I0_FLASH_DATA_O)  ;

  irq_io_set_vaule(AM_I0_WIRELESS_SPIEN, 0);
  irq_io_set_vaule(AM_I0_LED_DIG1, 0);

  irq_io_set_vaule(AM_IO_ARF2498_IRQ);//ARF2496K_TRINT


  ResetARF2498_SCK_Port();
  ResetARF2498_CE_Port();
  SetARF2498_MOSI_Port();
  SetARF2498_CS_Port();
#else

  gpio_set_value(AM_IO_ARF2498_SPI_EN, 0);
  gpio_set_value(AM_I0_WIRELESS_SPIEN, 0);

  //gpio_direction_input(HAL_IO_ARF2498_IRQ);//ARF2496K_TRINT

  ResetARF2498_CE_Port();
  SetARF2498_CS_Port();
#endif

}

#ifndef WIRELESS_SIMULATE_SPI

static u16 spi_ARF2498_transmit_byte(u8 data)
{
    volatile u8 arg = 0;
    ioctl(arf2498_spi_id,SPI_FIFO_TX_STATE,(void*)&arg);
    //while(system_spi_tx_fifo_full());
    while(arg)
    {
      ioctl(arf2498_spi_id,SPI_FIFO_TX_STATE,(void*)&arg);
    }
    //system_spi_write_byte(data);
    write(arf2498_spi_id,(u8 *)&data,1);

    //while(system_spi_rx_fifo_empty());

    ioctl(arf2498_spi_id,SPI_FIFO_RX_STATE,(void*)&arg);
    while(arg)
    {
      ioctl(arf2498_spi_id,SPI_FIFO_RX_STATE,(void*)&arg);
    }
    read(arf2498_spi_id,(u8*)&arg,1);
    //return system_spi_read_byte();
    return arg;
}
#endif

/************************************************************
��һ���ֽ�,�Ӹ�λ��ʼ����λ��
************************************************************/
u8 SPI_RW(u8 dat)
{
  u8 i;
  //GPIO_ResetBits(ARF2498_SCK_Port,ARF2498_SCK_Pin);
  ResetARF2498_SCK_Port();
  delay_us(1);
  for(i=0; i<8; i++)
  {
    if(dat&0x80){
      SetARF2498_MOSI_Port();
      //GPIO_SetBits(ARF2498_MOSI_Port, ARF2498_MOSI_Pin);		//����������
    }
    else{
      ResetARF2498_MOSI_Port();
      //GPIO_ResetBits(ARF2498_MOSI_Port, ARF2498_MOSI_Pin);    //����������
    }
    dat <<= 1;
    SetARF2498_SCK_Port();
    //GPIO_SetBits(ARF2498_SCK_Port, ARF2498_SCK_Pin);
    delay_us(1);
    //if(GPIO_ReadInputDataBit(ARF2498_MISO_Port,ARF2498_MISO_Pin)){
    if(ReadARF2498_MISO_Port()){
      dat |= 0x01;
    }
    else{
      dat &= 0xfe;
    }
    ResetARF2498_SCK_Port();
    //GPIO_ResetBits(ARF2498_SCK_Port,ARF2498_SCK_Pin);
    delay_us(1);
  }
  return dat;
}
/************************************************************
ʹ��spi�ӿڶ�оƬ��һ���Ĵ������ж�����	 �Ĵ��������ܴ���63
************************************************************/
u8 ReadARF2498Reg(u8 addr)
{
  u8 RegValue;  //�Ĵ���ֵ
  u8 commd;
  commd = addr & 0x3f;    //����λ����,�ڶ�SPIʱ����7,6λΪ00��
#ifdef WIRELESS_SIMULATE_SPI
  ResetARF2498_CS_Port();
  delay_us(1);
  //����8λ��ַ�Լ���дλ
  SPI_RW(commd);
  RegValue = SPI_RW(0);
  SetARF2498_CS_Port();
#else
  //sys_spi_cs_low();

  //hal_ints_off();
  ResetARF2498_CS_Port();
  //delay_us(1);
  spi_ARF2498_transmit_byte(commd);
  //RegValue = system_spi_read_byte();
  RegValue = spi_ARF2498_transmit_byte(0);
  //sys_spi_cs_high();
  SetARF2498_CS_Port();
  //hal_ints_on();
#endif


  return RegValue;
}
/************************************************************
ʹ��spi�ӿڶ�оƬ��һ���Ĵ�������д����	 �Ĵ��������ܴ���63
************************************************************/
void WriteARF2498Reg(u8 addr, u8 RegValue)		//д��Ĵ���ֵ
{
  u8 commd;
  commd = (addr&0x3f) | 0x40;    //����λ��1,��дSPIʱ����7,6λΪ01��

#ifdef WIRELESS_SIMULATE_SPI
  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  delay_us(1);
  //����8λ��ַ�Լ���дλ
  SPI_RW(commd);
  SPI_RW(RegValue);
  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();
#else
  //sys_spi_cs_low();
  //hal_ints_off();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  //system_spi_write_byte(RegValue);
  spi_ARF2498_transmit_byte(commd);
  spi_ARF2498_transmit_byte(RegValue);
  //sys_spi_cs_high();
  SetARF2498_CS_Port();

  //hal_ints_on();
#endif
}
/************************************************************
ʹ��spi�ӿڶ�оƬ��һ���Ĵ�������д����	 �Ĵ��������ܴ���63
************************************************************/
void Flush_Tx_Fifo(void)		//д��Ĵ���ֵ
{
  u8 commd;
  commd = 0xc5;    //����λ��1,��дSPIʱ����7,6λΪ01��
#ifdef WIRELESS_SIMULATE_SPI
  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  delay_us(1);
  //����8λ��ַ�Լ���дλ
  SPI_RW(commd);
  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();
#else
  //sys_spi_cs_low();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  spi_ARF2498_transmit_byte(commd);
  //sys_spi_cs_high();
  SetARF2498_CS_Port();
#endif
}
/************************************************************
ʹ��spi�ӿڶ�оƬ��һ���Ĵ�������д����	 �Ĵ��������ܴ���63
************************************************************/
void Flush_Rx_Fifo(void)		//д��Ĵ���ֵ
{
  u8 commd;
  commd = 0xc4;    //����λ��1,��дSPIʱ����7,6λΪ01��
#ifdef WIRELESS_SIMULATE_SPI
  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  delay_us(1);
  //����8λ��ַ�Լ���дλ
  SPI_RW(commd);
  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();
#else
  //sys_spi_cs_low();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  spi_ARF2498_transmit_byte(commd);
  //sys_spi_cs_high();
  SetARF2498_CS_Port();
#endif
}
/************************************************************
	����оƬ��FIFO����
************************************************************/
void Read_Fifo_Data(u8 *datbuff)		//д��Ĵ���ֵ
{
  u8 readnumber;
  u8 commd;
  u8 temp;
  commd = ARF2498_RXMODE;    //����λ��1,��дSPIʱ����7,6λΪ01��

#ifdef WIRELESS_SIMULATE_SPI

  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  SPI_RW(commd);
  temp = SPI_RW(0);
  rssi_val[rssi_cnt++] = (temp&0xf0)>>4;	// �ź�ǿ��
  if(rssi_cnt>=10){
    rssi_cnt = 0;
  }
  pipe_numb = temp&0x0f;		// ���Ͷ˵�ַ
  readnumber = SPI_RW(0)&0x1f;	// ���ݳ���
  for (temp = 0; temp<16; temp++)
  {
    if(temp < readnumber){
      datbuff[temp]=SPI_RW(0);	// ���ݰ�
    }
    else{
      datbuff[temp]=SPI_RW(0);	// ���ݰ�
    }
  }
  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();

#else
  //sys_spi_cs_low();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  spi_ARF2498_transmit_byte(commd);
  temp = spi_ARF2498_transmit_byte(0);//system_spi_read_byte();
  rssi_val[rssi_cnt++] = (temp&0xf0)>>4;	// �ź�ǿ��
  if(rssi_cnt>=10){
    rssi_cnt = 0;
  }
  pipe_numb = temp&0x0f;		// ���Ͷ˵�ַ

  readnumber = spi_ARF2498_transmit_byte(0)&0x1f;	// ���ݳ���
  for (temp = 0; temp<16; temp++)
  {
    if(temp < readnumber){
      datbuff[temp]=spi_ARF2498_transmit_byte(0);	// ���ݰ�
    }
    else{
      datbuff[temp]=spi_ARF2498_transmit_byte(0);	// ���ݰ�
    }
  }

  //sys_spi_cs_high();
  SetARF2498_CS_Port();
#endif
}
/************************************************************
	����оƬ��FIFO����,��Ӧ���ȴ�Ӧ��
************************************************************/
void Write_Fifo_Data(u8 rfpipe,u8 nack,u8 *datbuff,u8 sendnumber)		//д��Ĵ���ֵ
{
  u8 commd;
  u8 cnt;
  commd = ARF2498_TXMODE|0X10|rfpipe;    // 0DB
#ifdef WIRELESS_SIMULATE_SPI
  delay_ms(1);
  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  delay_us(1);
  SPI_RW(commd);
  if(nack == 0){
    SPI_RW(rf_channel);	// ��Ӧ��
  }
  else{
    SPI_RW(rf_channel|0x80);	 // ��Ӧ��
  }
  for (cnt =0; cnt < sendnumber ; cnt ++)
  {
    SPI_RW(datbuff[cnt]);
  }

  if(sendnumber<16){
    for(cnt = sendnumber;cnt<16;cnt++){
      SPI_RW(rf_sendbuff[cnt]);
    }
  }

  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();
  //���� CE ����50US������
  //GPIO_SetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
  SetARF2498_CE_Port();
  delay_us(50);
  ResetARF2498_CE_Port();
  //GPIO_ResetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
  delay_us(150);  // ��ȴ�130US
#else
  //sys_spi_cs_low();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  spi_ARF2498_transmit_byte(commd);
  if(nack == 0){
    spi_ARF2498_transmit_byte(rf_channel);	// ��Ӧ��
  }
  else{
    spi_ARF2498_transmit_byte(rf_channel|0x80);	 // ��Ӧ��
  }
  for (cnt =0; cnt < sendnumber ; cnt ++)
  {
    spi_ARF2498_transmit_byte(datbuff[cnt]);
  }

  if(sendnumber<16){
    for(cnt = sendnumber;cnt<16;cnt++){
      spi_ARF2498_transmit_byte(datbuff[cnt]);
    }
  }

  //sys_spi_cs_high();
  SetARF2498_CS_Port();

  SetARF2498_CE_Port();
  //delay_us(50);
  ResetARF2498_CE_Port();
  //delay_us(150);  // ��ȴ�130US

#endif
}
/************************************************************
	����оƬӦ���
************************************************************/
void Rf_Send_Ack(u8 rfpipe,u8 *datbuff,u8 readnumber)		//д��Ĵ���ֵ
{
	u8 commd;
	u8 i = 0 ;
	commd = ARF2498_TXACKMODE|0X08|rfpipe;    // 0DB
	//GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
#ifdef WIRELESS_SIMULATE_SPI
	ResetARF2498_CS_Port();
	SPI_RW(commd);
	for (; i < readnumber ; i ++)
	{
		SPI_RW(datbuff[i]);
	}
	//GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
	SetARF2498_CS_Port();
#else
        //sys_spi_cs_low();
        ResetARF2498_CS_Port();
        spi_ARF2498_transmit_byte(commd);
	for (; i < readnumber ; i ++)
	{
          spi_ARF2498_transmit_byte(datbuff[i]);
	}
        //sys_spi_cs_high();
        SetARF2498_CS_Port();
#endif
	delay_us(150);  // ��ȴ�130US
}
/************************************************************
	����оƬ��FIFO����,�ظ�����
*************************************************************/
void Rf_ReSend(u8 rfpipe,u8 *datbuff,u8 readnumber)		//д��Ĵ���ֵ
{
	u8 commd;
	commd = ARF2498_REUSETX;    // 0DB
	//GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
#ifdef WIRELESS_SIMULATE_SPI
	ResetARF2498_CS_Port();
	delay_us(1);
	SPI_RW(commd);
	SPI_RW(rf_channel);
    SetARF2498_CS_Port();
#else
        //sys_spi_cs_low();
        ResetARF2498_CS_Port();
        spi_ARF2498_transmit_byte(commd);
        spi_ARF2498_transmit_byte(rf_channel);
        //sys_spi_cs_high();
        SetARF2498_CS_Port();
#endif
	//GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
	delay_us(150);  // ��ȴ�130US
}
/**********************************************************
�������ģʽ 26uA // ��ʡ��ģʽ�������ӳ�1.5mS
**********************************************************/
void EnterStandMode(void)
{
	//GPIO_ResetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
	ResetARF2498_CE_Port();
	rf_data.mode = RF_STANDBY;
}
/**********************************************************
�������ģʽ 900nA
**********************************************************/
void RFPowerSaveMode(void)
{
	WriteARF2498Reg(RFREG_PWR,0X02);
	rf_data.mode = RF_SLEEP;
}
/**********************************************************
�������ģʽ 15mA
**********************************************************/
void EnterRXMode(void)
{
	Flush_Rx_Fifo();
	WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// ��������жϱ�־
	//GPIO_SetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
	SetARF2498_CE_Port();
   // WriteARF2498Reg(RFREG_Rx_Chn,0xc4);
	rf_data.mode = RF_RECV;
}
/**********************************************************
��ȡ��ǰRF״̬��0 Power_Save,1 standby,,2 tx_ack,3 tx_normal
4 rx_ack,5 rx_normal, 6 rx_receiving,7 invalid
**********************************************************/
u8 RFGetMode(void)
{
	return ReadARF2498Reg(0x22)&0xff;
}

/**********************************************************/
u8 RFGetFIFOState(void)
{
	return ReadARF2498Reg(0x1f)&0xff;
}

u8 RFGetIRQState(void)
{
	return ReadARF2498Reg(RFREG_Int)&0xff;
}

/************************************************************
	ʹ������ģ���жϣ� //ARF2498_IRQ_Port	//ARF2498_IRQ_Pin
*************************************************************/
void arf2498_handler(u8 p)
{
  RFIrq();
}


void EnableRfInt(void)
{
  //int mod;
  exti_init_t exti_user_init;

  exti_user_init.IntEn       = ENABLE;
  exti_user_init.Int_type    = EXTI_LOW_LEVEL;
  exti_user_init.WakeupirqEn = ENABLE;
  exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;

  ARF2496k_id = open(DEV_EXTI,ARF2496_IRQ_PIN);
  ioctl(ARF2496k_id,EXTI_INIT,&exti_user_init);
  request_irq(ARF2496k_id,(long)arf2498_handler,0);

  // WriteARF2498Reg(RFREG_Int,0X38);	//�͵�ƽ��Ч,���շ��Ͷ������ж�
  WriteARF2498Reg(RFREG_Int,0X20);	//�͵�ƽ��Ч�����յ�һ�����ݰ�
}
/************************************************************
	ʹ������ģ���жϣ� //ARF2498_IRQ_Port	//ARF2498_IRQ_Pin
0:
1��
*************************************************************/

void arf2498_sleep(u8 handle_time)
{
  //sys_exti_release(ARF2496k_id);
  if(handle_time == 0)
  {
    exti_init_t exti_user_init;
    exti_user_init.IntEn       = DISABLE;
    exti_user_init.Int_type    = EXTI_LOW_LEVEL;//EXTI_POSITIVE_EDGE;
    exti_user_init.WakeupirqEn = ENABLE;
    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    ioctl(ARF2496k_id,EXTI_INIT,&exti_user_init);
    if(get_enter_sleep_state() == SLEEP_MODE_S1)
    {
         EnterRXMode();
    }
  }

  if(handle_time == 1)
  {
    if(get_enter_sleep_state() == SLEEP_MODE_S1)
    {
      SetARF2498_CE_Port();
    }
    else//standby ģʽ
    {
      EnterStandMode();
    }
  }

}

void arf2498_wakeup(u8 handle_time)
{
  (void)handle_time;
  exti_init_t exti_user_init;
  exti_user_init.IntEn       = ENABLE;
  exti_user_init.Int_type    = EXTI_LOW_LEVEL;//EXTI_POSITIVE_EDGE;
  exti_user_init.WakeupirqEn = ENABLE;
  exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
  ioctl(ARF2496k_id,EXTI_INIT,&exti_user_init);
  EnterRXMode();
}

static power_ctrl_t sleep_funcs = {
  .sleep = arf2498_sleep,
  .wake  = arf2498_wakeup,
};
/**********************************************************
ʹ�����߽���
**********************************************************/
void EnableRFIncept(void)
{
  if(FALSE == EnableRFState)
  {
    EnableRFState = TRUE;
	EnterRXMode();
  }
}
/********************************************************
��ֹ���߽���
********************************************************/
void DisableRFIncept(void)
{
  EnableRFState = FALSE;
  EnterStandMode();
}
/**********************************************************
	��ȡRFͨѶĿ�ĵ�ַ �������γɱ�����ַ
**********************************************************/
void  ReadDeviceAddress(void)
{
	u32 useraddress=0;
	//u8 temp;
        useraddress = chipid_read();//0x123456789;//
    /*
	for(temp = 0;temp<3;temp++)	{
		//useraddress += *(u32 *)(MCU_ID+temp*4);
		useraddress += (MCU_ID+temp*4);
	}*/
	RFGroupAddress[0] = (u8)(useraddress&0xff);
	RFGroupAddress[1] = (u8)((useraddress>>8)&0xff);
	RFGroupAddress[2] = (u8)((useraddress>>16)&0xff);
	RFGroupAddress[3] = (u8)((useraddress>>24)&0xff);
	//RFDeviceAddress = 0;
	//for(temp = 0;temp<12;temp++)	{
		//RFDeviceAddress += *(u8 *) (MCU_ID+temp);
	//}
}
/**********************************************************
	����RF��Ⱥ���ַ
**********************************************************/
void SetDestinationAddress(void)
{
	WriteARF2498Reg(0x09,RFGroupAddress[0]);
	WriteARF2498Reg(0x0a,RFGroupAddress[1]);
	WriteARF2498Reg(0x0b,RFGroupAddress[2]);
	WriteARF2498Reg(0x0c,RFGroupAddress[3]);
	WriteARF2498Reg(0x0d,RFDeviceAddress);
	WriteARF2498Reg(0x0e,RFSlaveAddr);
}
/**********************************************************
	����RF�Ķ����ַ
**********************************************************/
void SetPartnershipAddr(void)
{
	WriteARF2498Reg(0x09,PartnershipAddress[0]);
	WriteARF2498Reg(0x0a,PartnershipAddress[1]);
	WriteARF2498Reg(0x0b,PartnershipAddress[2]);
	WriteARF2498Reg(0x0c,PartnershipAddress[3]);
	WriteARF2498Reg(0x0d,PartnershipAddress[4]);	 // ͨ���˹ܵ�����
	WriteARF2498Reg(0x0e,PartnershipAddress[5]);	 // ͨ���˹ܵ�����
}
/**********************************************************
	����RFƵ��
**********************************************************/
void Set_rfchannel(u8 ch)
{
	if(ch>=125){
		ch = ARF2498_CHANNEL;
	}
	rf_channel = ch;
}
int init_aft2498 = 0;
/**********************************************************
��ʼ��RF
**********************************************************/
void InitARF2496k(void)
{
  //return;//lyy
  static u8 temp = 0;
  InitRfHardWare();
  //delay_ms(10);
  if(ReadARF2498Reg(0x3d) == 0xa1){
    delay_ms(1);  // ��ȡID����ȷ
    //AM_DEBUG_INFO("read ID ok!!!\r\n");
  }

  ReadDeviceAddress();
  Set_rfchannel(ARF2498_CHANNEL);

  while(RF_InitTable[temp] != 0xaa)
  {
    WriteARF2498Reg(RF_InitTable[temp],RF_InitTable[temp+1]);
    delay_us(10);
    if(RF_InitTable[temp+1] != ReadARF2498Reg(RF_InitTable[temp]))
    {
      delay_us(10);
      // AM_DEBUG_INFO("error write!!!\r\n");
    }
    else{
      temp += 2;
      // AM_DEBUG_INFO("ok write!!!\r\n");
    }
  }

  WriteARF2498Reg(RFREG_PWR,0X00);
  WriteARF2498Reg(0x20,0x20); // �Զ�ˢ���ź�ǿ��
  SetDestinationAddress();	// �����ַ
  WriteARF2498Reg(0x0f,0x03);
  WriteARF2498Reg(0x10,0x04);
  WriteARF2498Reg(0x11,0x05);
  WriteARF2498Reg(0x12,0x06);
  WriteARF2498Reg(0x13,0x07);
  WriteARF2498Reg(0x14,0x08);
  WriteARF2498Reg(0x15,0x09);

  Flush_Tx_Fifo();
  Flush_Rx_Fifo();
  WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// ��������жϱ�־
  EnableRfInt();
  EnterRXMode();
  //
  set_power_callback(&sleep_funcs);
  init_aft2498 = 1;
}



/***********************************************************
������������
***********************************************************/
void SendRF(u8 *senddata,u8 sendnumber)
{
  u8 cnt;
  //GPIO_ResetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
  ResetARF2498_CE_Port();
  Flush_Tx_Fifo();
  WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// ��������жϱ�־
  rf_data.mode = RF_SEND;
  Write_Fifo_Data(1,1,senddata, sendnumber);
  for(cnt =0;cnt<20;cnt++){
    if(ReadARF2498Reg(RFREG_State)&0x18){
      //AM_DEBUG_INFO("Write_Fifo_Data ok\r\n");
      break;
    }
    else{
      delay_ms(1);
      //DelayMs(1);
    }
  }
  WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// ��������жϱ�־
  delay_us(200);
  SetDestinationAddress();
  EnterRXMode();
}
/*******************************************************
	rf�����е��жϺ���
*******************************************************/
//extern void get_schedule_data(u8 *data,u8 number);
void Set_virsual_ir_rx_code(void)
{
   u8 readdata[16];
   if(QueryARF2496KInceptSign()==TRUE)
   {
    ReadRF(readdata,16);
   }

   if((readdata[0]>= 0x1)&&(readdata[0]<4)&&(readdata[1] == 0xff))
   {
     //AM_DEBUG_INFO("readdata[1]=%x\r\n",readdata[1]);
       check_partnership_ok_flag = 1;
   }
   if(readdata[0]==0x01) //ң�������ź�
   {
   // AM_DEBUG_INFO("readdata[1]=%x\r\n",readdata[1]);
       ir_rx_code_get(1,readdata[1]);
       if(readdata[1] == 0x9)
       {
           //get_schedule_data(&readdata[2],6);
       }
   }
   else if(readdata[0]==0x02) //DOCK���ź�
   {
    ir_rx_code_get(0,readdata[1]);
       dock_wireless_rx_code_get(0,readdata[1]);
       //AM_DEBUG_INFO("dock readdata[1]=%x\r\n",readdata[1]);
   }
   else if(readdata[0]==0x03)
   {
      //AM_DEBUG_INFO("readdata[0]=%x\r\n",readdata[0]);
      //AM_DEBUG_INFO("readdata[1]=%x\r\n",readdata[1]);
       ir_rx_code_get(0,readdata[1]);
   }
   signal_flag=1;
}

void RFIrq(void)
{
  u8 temp = 0;
  ResetARF2498_CE_Port();
  //GPIO_ResetBits(ARF2498_CE_Port, ARF2498_CE_Pin);	// �رս���
  temp = ReadARF2498Reg(RFREG_State);

  //if(temp&0x20){  // bit5 ���յ�һ������
  if(temp&0x2){  // bit5 ���յ�һ������
    //WriteARF2498Reg(RFREG_Rx_Chn,0);
    Read_Fifo_Data(rf_recvbuff);//spi
    //Rf_Send_Ack(pipe_numb-1, rf_askbuff, 5);
    RFInceptSign = TRUE;
    //GPIO_SetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
    Set_virsual_ir_rx_code();
    SetARF2498_CE_Port();

  }
  WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// ��������жϱ�־
  Flush_Tx_Fifo();
  EnterRXMode();
}
/***********************************************************
�ӽ��ջ����н�������,���������ݲ��ܴ���16���ֽ�
***********************************************************/
void ReadRF(u8 *readdata,u8 readnumber)
{
	for(;readnumber >0;readnumber--) //����readnumber�����ݵ�������
	{
		readdata[readnumber-1] =  rf_recvbuff[readnumber-1];
	}
	RFInceptSign = FALSE;			  //����֮�������־
}
/***********************************************************
��ѯ�Ƿ���������ź�
***********************************************************/
bool QueryARF2496KInceptSign(void)
{
	return RFInceptSign;
}
/**********************************************************
	RF����
**********************************************************/
void PartnershipRF(void)
{

	u16 i=0;
    //delay_us(200);
    SetPartnershipAddr();	//���ö����ַ
//	gbv_rf_send_interval = 1000;

	rf_sendbuff[0] = HOST_ID;
	rf_sendbuff[1] = PARTNERSHIP;
	rf_sendbuff[2] = RFGroupAddress[0];
	rf_sendbuff[3] = RFGroupAddress[1];
	rf_sendbuff[4] = RFGroupAddress[2];
	rf_sendbuff[5] = RFGroupAddress[3];
	rf_sendbuff[6] = RFDeviceAddress;
	rf_sendbuff[15] = 0;
	for(;i<15;i++){
		rf_sendbuff[15] += rf_sendbuff[i];
	}
	//memset(rf_sendbuff,0x01,16);
	SendRF(rf_sendbuff,16);
   /*
	WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// ��������жϱ�־
	delay_us(200);
	SetDestinationAddress();
	EnterRXMode();	*/

}

void Send_ACK_Package(void)
{
    u8 temp[16]={1};
    SendRF(temp,16);
}
void Wireless_Dock(void)
{
  u8 temp[2];
  // DisableRFIncept();
  // WriteDeviceAddressOfARF2496K();

  SetDestinationAddress();

  temp[0] = 0;
  temp[1] = 0x40;
  SendRF(temp,2);
 // DelayMs(2000);

 // EnableRFIncept();

}

void Wireless_Rcon(void)
{
  u8 temp[2];
  // DisableRFIncept();
  // WriteDeviceAddressOfARF2496K();

  SetDestinationAddress();

  temp[0] = 0;
  temp[1] = 0x20;
  SendRF(temp,2);
 // DelayMs(2000);

 // EnableRFIncept();

}
/****************************************************************
ȡ��RF�ź�ǿ��
****************************************************************/
u8 read_rf_Rssi(void)
{
	int i = 0;
	u16 sum = 0;
	for(;i<10;i++){
		sum += rssi_val[i];
	}
	return sum/10;
}

u8 get_partnership_ok_flag(void)
{
    return check_partnership_ok_flag;
}

void set_partnership_ok_flag(u8 partnership_ok_flag)
{
    check_partnership_ok_flag = partnership_ok_flag;
}

void opend_dock_signal(void)
{
  static u8 send_dock_command_cnd=4;
  static u32 time_flag = 0;
  clear_dock_wireless_rx_code();
  while(send_dock_command_cnd)
  {
    if(remote_ir_get((IR_REMOT_POSITION_E)0) ==0x40)
      break;
    else
    {
      if(timer_elapsed(time_flag)>10)
      {
        send_dock_command_cnd--;
        time_flag=timer_ms();
        Wireless_Dock() ;
      }
    }
  }
}

#endif



