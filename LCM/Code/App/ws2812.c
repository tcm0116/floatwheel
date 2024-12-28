#include "ws2812.h"

uint16_t WS2812_Counter = 0;
uint8_t WS2812_Buff[WS2812_N][24];

/**************************************************
 * @brie   :WS2812_Init()
 * @note   :WS2812初始化 
 * @param  :无
 * @retval :无
 **************************************************/
void WS2812_Init(void)
{
	uint16_t i = 0;
	uint8_t*	ws2812_buff_add = (uint8_t*)&WS2812_Buff;
	
	//SPI_Config();
	IO_WS2812_Init();
	
	while(i < (WS2812_N*24))
	{
		*ws2812_buff_add = (uint8_t)WS2812_0;
		ws2812_buff_add++;
		i++;
	}
}

// Set a range of LEDs to the same color
void WS2812_Set_AllColours(uint8_t start, uint8_t end, uint8_t red, uint8_t green, uint8_t blue) {
	uint8_t startIndex = start - 1;
	uint8_t endIndex = end - 1;

	for (uint8_t i=0; i<10; i++) {
		if (i >= startIndex && i <= endIndex ) {
			WS2812_Set_Colour(i,red,green,blue);
		} else {
			WS2812_Set_Colour(i,0,0,0);
		}
	}
}

/**************************************************
 * @brie   :WS2812_Set_Colour()
 * @note   :WS2812设置颜色 
 * @param  :num 	第几个灯
 *          red 	0-255
 *          green	0-255
 *          blue    0-255
 * @retval :无
 **************************************************/
void WS2812_Set_Colour(uint8_t num,uint8_t red,uint8_t green,uint8_t blue)
{
	uint8_t i = 0;
	
	for(i=0; i<24; i++)
	{
		if(i<8)   //红灯
		{
			if(green&0x80)
			{
				WS2812_Buff[num][i] = WS2812_1;
			}
			else
			{
				WS2812_Buff[num][i] = WS2812_0;
			}
			green <<= 1;
		}
		else if(i<16)  //绿灯
		{
			if(red&0x80)
			{
				WS2812_Buff[num][i] = WS2812_1;
			}
			else
			{
				WS2812_Buff[num][i] = WS2812_0;
			}
			red <<= 1;		
		}
		else if(i<24)  //蓝灯
		{
			if(blue&0x80)
			{
				WS2812_Buff[num][i] = WS2812_1;
			}
			else
			{
				WS2812_Buff[num][i] = WS2812_0;
			}
			blue <<= 1;			
		}
		
	}
	
}

///**************************************************
// * @brie   :WS2812_Refresh()
// * @note   :WS2812刷新一次
// * @param  :无
// * @retval :无
// **************************************************/
//void WS2812_Refresh(void)
//{
//	uint16_t 	i = 0;
//	uint8_t*	ws2812_buff_add = (uint8_t*)&WS2812_Buff;
//	
//	for(i=0; i<250; i++)   //复位信号
//	{
//		SPI1_Send_Byte(WS2812_R);
//	}
//	
//	for(i=0; i<(WS2812_N*24); i++)
//	{
//		SPI1_Send_Byte((uint8_t)(*(ws2812_buff_add)));
//		ws2812_buff_add++;
//	}
//	
//}

void delay(uint16_t i)
{
	while(i--);
}

void WS2812_Refresh(void)
{
	uint16_t i = 0;
	uint8_t* ws2812_buff_add = (uint8_t*)&WS2812_Buff;
	
	//WS2812在发数据时不允许中断打断否则不正常
	
	__set_PRIMASK(1);//关总中断
	
	GPIOD->BRR = GPIO_Pin_4;
	delay(250);
	
	for(i=0; i<(WS2812_N*24); i++)
	{
		if(*(ws2812_buff_add) == WS2812_0)
		{
			WS2812_0_Code();
		}
		else
		{
			WS2812_1_Code();
		}
		ws2812_buff_add++;
	}
	
	__set_PRIMASK(0);//�����ж�
	//GPIOD->BSRR = GPIO_Pin_4;
}
