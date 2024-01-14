#include "max6675.h"
#include "stdio.h"
#define 	MAX6675_CS	 		GPIO_Pin_4
#define 	MAX6675_CSL()		GPIOA->BRR = MAX6675_CS;
#define 	MAX6675_CSH()		GPIOA->BSRR = MAX6675_CS;

/*
 * 函数名：SPI1_Init
 * 描述  MAX6675 接口初始化
 * 输入  ：无 
 * 输出  ：无
 * 返回  ：无
 */																						  
void SPI_MAX6675_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;	
	
	/* 使能 SPI1 时钟 */                         
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

	/* ---------通信I/O初始化----------------
	 * PA5-SPI1-SCK :MAX6675_SCK
	 * PA6-SPI1-MISO:MAX6675_SO
	 * PA7-SPI1-MOSI:MAX6675_SI	 
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// 复用输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ---------控制I/O初始化----------------*/
	/* PA4-SPI1-NSS:MAX6675_CS */							// 片选
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		// 推免输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);						  
	GPIO_SetBits(GPIOA, GPIO_Pin_4);						// 先把片选拉高，真正用的时候再拉低
 

	/* SPI1 配置 */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	
	/* 使能 SPI1  */
	SPI_Cmd(SPI1, ENABLE); 
}

/*
 *
 *
 *
 */
unsigned char MAX6675_ReadByte(void)
{
	
	/* Loop while DR register in not emplty */
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI1, 0xff);
	
	/* Wait to receive a byte */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);
}
u8 Get_temprature(float *temprature)
{
		unsigned int t,i;
		unsigned char c;
		unsigned char flag;
		MAX6675_CSL();
		c = MAX6675_ReadByte();
		i = c;
		i = i<<8;
		c = MAX6675_ReadByte();
		MAX6675_CSH();
		
		i = i|((unsigned int)c);			//i是读出来的原始数据
		flag = i&0x04;						//flag保存了热电偶的连接状态
		t = i<<1;
		t = t>>4;
	 if( i != 0 && flag == 0)
	 {
			*temprature = t*0.25;			
		}
	 if(i != 0 && flag == 1)
		 printf("K line off\r\n");
		return flag;
	
}
