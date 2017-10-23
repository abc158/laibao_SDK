#include "bk2425.h"

/**************************************************
Function: Send_Packet
Description:
	fill FIFO to send a packet
Parameter:
	type: WR_TX_PLOAD or  W_TX_PAYLOAD_NOACK_CMD
	pbuf: a buffer pointer
	len: packet length
Return:
	None
**************************************************/
void Send_Packet(UINT8 type,UINT8* pbuf,UINT8 len)
{
	UINT8 fifo_sta;

	fifo_sta=SPI_Read_Reg(FIFO_STATUS);	// read register FIFO_STATUS's value

	if((fifo_sta&FIFO_STATUS_TX_FULL)==0)//if not full, send data  
	  	SPI_Write_Buf(type, pbuf, len); // Writes data to buffer  A0,B0,A8
	  	 	
}

/**************************************************
Function: Receive_Packet
Description:
	read FIFO to read a packet
Parameter:
	None
Return:
	None
**************************************************/
extern void get_rec_ir_data(U8 index,U8 data);
void Receive_Packet()
{
	UINT8 len,sta,fifo_sta;
	UINT8 rx_buf[MAX_PACKET_LEN];
        UINT8 i;
	sta=SPI_Read_Reg(STATUS);	// read register STATUS's value
	if((sta&STATUS_RX_DR)==STATUS_RX_DR)				// if receive data ready (RX_DR) interrupt
	{
		do
		{
			len=SPI_Read_Reg(R_RX_PL_WID_CMD);	// read len

			if(len<=MAX_PACKET_LEN)
			{
				SPI_Read_Buf(RD_RX_PLOAD,rx_buf,len);// read receive payload from RX_FIFO buffer
				//printf("\r\nPacket:");
				for(i=0;i<len;i++)
				{
                                  get_rec_ir_data(i,rx_buf[i]);
					//printf("%x;",rx_buf[i]);
                                  //if(rx_buf[i])
                                 //printf("REC %d  IR %d  STA  %d\r\n",((rx_buf[i]>>5)&0x07),((rx_buf[i]>>2)&0x07),((rx_buf[i])&0x03));
				}
			}
			else
			{
				SPI_Write_Reg(FLUSH_RX,0);//flush Rx
			}

			fifo_sta=SPI_Read_Reg(FIFO_STATUS);	// read register FIFO_STATUS's value
						
		}while((fifo_sta&FIFO_STATUS_RX_EMPTY)==0); //while not empty
	}

	SPI_Write_Reg(WRITE_REG|STATUS,sta);// clear RX_DR or TX_DS or MAX_RT interrupt flag
	
}


