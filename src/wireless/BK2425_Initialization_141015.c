////////////////////////头文件////////////////////////////////////
#include "syscall_api.h"
#include "am_exti_cfg.h"
#include "am_device.h"
#include "ui-ir.h"
#include "ARF2496K.H"
#include "ui-config.H"
#include "act.h"
#include "syscall_api.h"
#include "docking-new.h"
#include "am_spi_cfg.h"
#include "bk2425.h"
#define         SPI_FLASH_CS_LOW()        gpio_set_value(AM_IO_BK2425_SPI_EN, 0)  
#define         SPI_FLASH_CS_HIGH()       gpio_set_value(AM_IO_BK2425_SPI_EN, 1)  
#define         SPI_WIRELESS_CE_LOW()     gpio_set_value(AM_IO_BK2425_SPI_CE, 0)
#define         SPI_WIRELESS_CE_HIGH()    gpio_set_value(AM_IO_BK2425_SPI_CE, 1)
UINT8 op_status;
extern void Delay_us(UINT32 nTime);
/* RF channel table,updated on May 30,2012 */
UINT16 RF_TABLE[16]={
2402,2405,2413,
2415,2417,2420,
2423,2428,2432,
2435,2438,2440,
2442,2446,2457,2459,//for 16 channels,use above frequency 
};

/* Bank1 register initialization value,updated on May 17,2012 */

//In the array Bank1_Reg0_13[],all the register values are the byte reversed!!!!!!!!!!!!!!!!!!!!!
unsigned long Bank1_Reg0_13[]={
0x1B8296f9,
/*REG4,
----------------------------------------------------------------
|			               1Mbps      | 2Mbps	          |	250Kbps		|
|     Normal Mode		 0x1B8296f9	|	0xdB8296f9      | 0xdB8a96f9
|     CW Normal Mode 0x218296f9	
|     外部PA         0xDB8A96C1 | 0x1B8296C1      | 0xDB8296C1
----------------------------------------------------------------
*/
0xA60F0624,
/*REG5,
----------------------------------------------------------------
|			1Mbps:0xA60F0624      disable rssi
|     2Mbps:0xB60F0624			disable rssi
|     259kbps:0xB60F0624		disable rssi
----------------------------------------------------------------
*/
0x00127300,
/*REG12,120517
0x00127300:PLL locking time 120us 
0x00127305(chip default):PLL locking time 130us compatible with nRF24L01;
*/
0x36B48000,//REG13
};

UINT8 Bank1_Reg14[]=
{
0x41,0x20,0x08,0x04,0x81,0x20,0xcf,0xF7,0xfe,0xff,0xff
};

//Bank0 register initialization value
  UINT8 Bank0_Reg[][2]={
{0,0x0F},
{1,0x3F},
{2,0x3F},
{3,0x03},
{4,0xff},
{5,0x17},
{6,0x0F},	//REG6,120517,0x0F or 0x2F:2Mbps; 0x07:1Mbps ; 0x27:250Kbps
{7,0x07},
{8,0x00},
{9,0x00},
{12,0xc3},
{13,0xc4},
{14,0xc5},
{15,0xc6},
{17,0x20},
{18,0x20},
{19,0x20},
{20,0x20},
{21,0x20},
{22,0x20},
{23,0x00},
{28,0x3F},
{29,0x07}
};


  UINT8 RX0_Address[]={0x15,0x59,0x23,0xC6,0x29};
  UINT8 RX1_Address[]={0x10,0x56,0x24,0xCD,0x78};

///////////////////////////////////////////////////////////////////////////////
//                  SPI access                                               //
///////////////////////////////////////////////////////////////////////////////

/**************************************************         
Function: SPI_RW();                                         
                                                            
Description:                                                
	Writes one UINT8 to BK2425, and return the UINT8 read 
**************************************************/     
extern UINT8 SPI_RW_Byte(UINT8 byte);	
int bk2425_spi_id = 0;
extern void BK2425_Initialize(void);
void InitBK2425(void)
{
  bk2425_spi_id = spi0_id_get();
  SPI_FLASH_CS_HIGH();
  BK2425_Initialize();
}
static u8 spi_BK2425_transmit_byte(u8 data)
{
    volatile u8 arg = 0;
    ioctl(bk2425_spi_id,SPI_FIFO_TX_STATE,(void*)&arg);

    while(!arg)
    {
      ioctl(bk2425_spi_id,SPI_FIFO_TX_STATE,(void*)&arg);
    }

    write(bk2425_spi_id,(u8 *)&data,1);


    ioctl(bk2425_spi_id,SPI_FIFO_RX_STATE,(void*)&arg);
    while(!arg)
    {
      ioctl(bk2425_spi_id,SPI_FIFO_RX_STATE,(void*)&arg);
    }
    read(bk2425_spi_id,(u8*)&arg,1);
    return arg;
}
UINT8 SPI_RW(UINT8 value)                                    
{                                                           
	UINT8 rec_data;
//	for(bit_ctr=0;bit_ctr<8;bit_ctr++)   // output 8-bit
//	{
//		if(value & 0x80)
//		{
//			MOSI=1;
//		}
//		else
//		{
//			MOSI=0;		
//		}

//		value = (value << 1);           // shift next bit into MSB..
//		SCK = 1;                      // Set SCK high..
//		value |= MISO;       		  // capture current MISO bit
//		SCK = 0;            		  // ..then set SCK low again
//	}
        rec_data=spi_BK2425_transmit_byte(value);
	return(rec_data);           		  // return read UINT8
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Write_Reg();                                  
                                                            
Description:                                                
	Writes value 'value' to register 'reg'              
**************************************************/        
void SPI_Write_Reg(UINT8 reg, UINT8 value)                 
{
	SPI_FLASH_CS_LOW();                   // CSN low, init SPI transaction
	op_status = SPI_RW(reg);      // select register
	SPI_RW(value);             // ..and write value to it..
	SPI_FLASH_CS_HIGH();                   // CSN high again
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Read_Reg();                                   
                                                            
Description:                                                
	Read one UINT8 from BK2425 register, 'reg'           
**************************************************/        
UINT8 SPI_Read_Reg(UINT8 reg)                               
{                                                           
	UINT8 value;
	SPI_FLASH_CS_LOW();                // CSN low, initialize SPI communication...
	op_status=SPI_RW(reg);            // Select register to read from..
	value = SPI_RW(0);    // ..then read register value
	SPI_FLASH_CS_HIGH();                // CSN high, terminate SPI communication
	return(value);        // return register value
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Read_Buf();                                   
                                                            
Description:                                                
	Reads 'length' #of length from register 'reg'         
**************************************************/        
void SPI_Read_Buf(UINT8 reg, UINT8 *pBuf, UINT8 length)     
{                                                           
	UINT8 byte_ctr;                              
                                                            
	SPI_FLASH_CS_LOW();                    		// Set CSN l
	SPI_RW(reg);       		// Select register to write, and read status UINT8
                                                            
	for(byte_ctr=0;byte_ctr<length;byte_ctr++)           
		pBuf[byte_ctr] = SPI_RW(0);    // Perform SPI_RW to read UINT8 from BK2425
                                                            
	SPI_FLASH_CS_HIGH();                           // Set CSN high again
               
}                                                           
/**************************************************/        
                                                            
/**************************************************         
Function: SPI_Write_Buf();                                  
                                                            
Description:                                                
	Writes contents of buffer '*pBuf' to BK2425         
**************************************************/        
void SPI_Write_Buf(UINT8 reg, UINT8 *pBuf, UINT8 length)    
{                                                           
	UINT8 byte_ctr;                              
                                                            
	SPI_FLASH_CS_LOW();                   // Set CSN low, init SPI tranaction
	op_status = SPI_RW(reg);    // Select register to write to and read status UINT8
	for(byte_ctr=0; byte_ctr<length; byte_ctr++) // then write all UINT8 in buffer(*pBuf) 
		SPI_RW(*pBuf++);                                    
	SPI_FLASH_CS_HIGH();                 // Set CSN high again      

}                                                           
/**************************************************/        


/**************************************************
Function: SwitchToRxMode();
Description:
	switch to Rx mode
**************************************************/
void SwitchToRxMode()
{
	  UINT8 value;

	SPI_Write_Reg(FLUSH_RX,0);//flush Rx

	value=SPI_Read_Reg(STATUS);	// read register STATUS's value
	SPI_Write_Reg(WRITE_REG|STATUS,value);// clear RX_DR or TX_DS or MAX_RT interrupt flag

	SPI_WIRELESS_CE_LOW();

	value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PRX
	value=value|0x01;//set bit 1
  	SPI_Write_Reg(WRITE_REG | CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..

	SPI_WIRELESS_CE_HIGH();
}

/**************************************************
Function: SwitchToTxMode();
Description:
	switch to Tx mode
**************************************************/
void SwitchToTxMode()
{
	  UINT8 value;
	SPI_Write_Reg(FLUSH_TX,0);//flush Tx

	SPI_WIRELESS_CE_LOW();
	value=SPI_Read_Reg(CONFIG);	// read register CONFIG's value
//PTX
	value=value&0xfe;//set bit 1
  	SPI_Write_Reg(WRITE_REG | CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.

	SPI_WIRELESS_CE_HIGH();
}

/**************************************************
Function: SwitchCFG();
                                                            
Description:
	 access switch between Bank1 and Bank0 

Parameter:
	_cfg      1:register bank1
	          0:register bank0
Return:
     None
**************************************************/
UINT8 reg_temp;
void SwitchCFG(char _cfg)//1:Bank1 0:Bank0
{
	UINT8 Tmp;

	reg_temp=Tmp=SPI_Read_Reg(7);
	Tmp=Tmp&0x80;

	if( ( (Tmp)&&(_cfg==0) )
	||( ((Tmp)==0)&&(_cfg) ) )
	{
		SPI_Write_Reg(ACTIVATE_CMD,0x53);
	}
}

/**************************************************
Function: SetChannelNum();
Description:
	set channel number

**************************************************/
void SetChannelNum(UINT8 ch)
{
	SPI_Write_Reg((UINT8)(WRITE_REG|5),(UINT8)(ch));
}



///////////////////////////////////////////////////////////////////////////////
//                  BK2425 initialization                                    //
///////////////////////////////////////////////////////////////////////////////
/**************************************************         
Function: BK2425_Initialize();                                  

Description:                                                
	register initialization
**************************************************/   
void BK2425_Initialize(void)
{
	INT8 i,j;
 	UINT8 WriteArr[4];

	DelayMs(100);//delay more than 50ms.
	
	SwitchCFG(0);

//********************Write Bank0 register******************
	for(i=20;i>=0;i--)
	{
		SPI_Write_Reg((WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);
		if(i==0)break;
	}

//reg 10 - Rx0 addr
	SPI_Write_Buf((WRITE_REG|10),RX0_Address,5);
	
//REG 11 - Rx1 addr
	SPI_Write_Buf((WRITE_REG|11),RX1_Address,5);

//REG 16 - TX addr
	SPI_Write_Buf((WRITE_REG|16),RX0_Address,5);
//	printf("\nEnd Load Reg");

	i=SPI_Read_Reg(29);

	if(i==0) // i!=0 showed that chip has been actived.so do not active again.
		SPI_Write_Reg(ACTIVATE_CMD,0x73);// Active

	for(i=22;i>=21;i--)
		SPI_Write_Reg((WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);

//********************Write Bank1 register******************
	SwitchCFG(1);

	for(i=0;i<=1;i++)//reverse
	{
		for(j=0;j<4;j++)
			WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(j) ) )&0xff;

		SPI_Write_Buf((WRITE_REG|(i+4)),&(WriteArr[0]),4);
	}

	for(i=2;i<=3;i++)
	{
		for(j=0;j<4;j++)
			WriteArr[j]=(Bank1_Reg0_13[i]>>(8*(3-j) ) )&0xff;

		SPI_Write_Buf((WRITE_REG|(i+10)),&(WriteArr[0]),4);
	}

	SPI_Write_Buf((WRITE_REG|14),&(Bank1_Reg14[0]),11);

//toggle REG4<25,26>
	for(j=0;j<4;j++)
		WriteArr[j]=(Bank1_Reg0_13[0]>>(8*(j) ) )&0xff;

	WriteArr[0]=WriteArr[0]|0x06;
	SPI_Write_Buf((WRITE_REG|4),&(WriteArr[0]),4);

	WriteArr[0]=WriteArr[0]&0xf9;
	SPI_Write_Buf((WRITE_REG|4),&(WriteArr[0]),4);


	DelayMs(10);
	
//********************switch back to Bank0 register access******************
	SwitchCFG(0);

        SwitchToRxMode();//switch to RX mode

}

/**************************************************         
Function: DelayMs();                                  

Description:                                                
	delay ms,please implement this function according to your MCU.
**************************************************/  
//延时程序
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
void DelayMs(UINT16 ms)
{
delay_ms(ms);

}


