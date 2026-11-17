/*@brief :此库经过修理，从C移植到C++，抽象为了对象，可以同时设置多个OLED
 *关于如何使用：
 *通过 Oled oled();
 *  或 Oled oled(Pin SCL,Pin SDA,Pin RST,Pin DC); 初始化对象
 *								\
 *								 \------>先使用Pin声明对应引脚的 GPIOx 和 GPIO_Pin_x,再通过构造函数创建OLED对象
 *			
 *其次，使用
 *			void init(u8 ID=0);																					必须先init()才可使用OLED
 *			void wrData(u8 data);           														可忽略
 *			void wrCmd(u8 cmd);  																				可忽略
 *			void setPos(u8 x, u8 y);																		设置输入光标位置( x:[0:127] ,y:[0:7] (注意，y代表页数，一页有8行) )  
 *			void fill(u8 data);																					全屏使用data填充
 *			void cls(void);																							全屏清空
 *			void clsY(u8 y);																						清空第 y 页
 *			void clsLineArea(u8 x,u8 from,u8 width);										清空第 y 页从 from 开始 width 长度的像素
 *			void lineY(u8 y);																						第 y 页画线
 *			void putPixel(u8 x,u8 y);																		放置像素在( x[0:127] y[0:63] 注意，可能同一页上的已存内容被覆盖消失)
 *			void putColumn(u8 x,u8 data);																第 x 列使用 data 填充
 *			void rectangle(u8 x1,u8 y1,u8 x2,u8 y2);										绘制矩形(未测试)
 *			void P6x8Str(u8,u8,u8 *ch);																	输出文字
 *			void P8x16Str(u8,u8 ,u8 *ch);
 *			void P16x16Str(u8 ,u8 ,u8 *ch);
 *			//extern void LCD_Print(u8 x, u8 y, u8 *ch);
 *			void Print(u8 , u8 , u8 *ch,u8 char_size, u8 ascii_size);
 *			void drawBMP(u8 ,u8 ,const u8 *bmp); 
 *
 *
 *
 *首次封装:魏俊臣 2024.12.5
 *更新 2025.2.16
 */




#include "oled.h"

Oled oled;

void Oled::wrData(u8 data)
{
	u8 i=8;
	#if USE_CS
		OLED_CS(0);//CS拉低，开始通信
	#endif
	DC_SET();
	SCL_CLR();    
	while(i--)
	{
		if(data&0x80)
			SDA_SET();
		else
			SDA_CLR();
		SCL_SET();            
		SCL_CLR();    
		data<<=1;    
	}
	#if USE_CS
		OLED_CS(1);//CS拉高，结束通信
	#endif
}

void Oled::wrCmd(u8 cmd)
{
	u8 i=8;
	#if USE_CS
		OLED_CS(0);//CS拉低，开始通信
	#endif
	DC_CLR();
	SCL_CLR();
	while(i--)
	{
		if(cmd&0x80)
			SDA_SET();
		else
			SDA_CLR();
		SCL_SET();            
		SCL_CLR();    
		cmd<<=1;;   
	}
	#if USE_CS
		OLED_CS(1);//CS拉高，结束通信
	#endif
}

/*设置光标位置(x=0~128,y=0~7)
*y轴是按8格递进的，y轴像素0~63，但只能按8格的倍数显示，
*因为列行式只能按0~7进行*/

void Oled::setPos(u8 x, u8 y)
{ 
  wrCmd(0xb0+y);
  wrCmd(((x&0xf0)>>4)|0x10);
  wrCmd((x&0x0f)|0x01); 
} 
/*
设置全屏覆盖
*/
void Oled::fill(u8 data)
{
	for(u8 y=0;y<8;y++)
	{
		setPos(0,y);
		for(u8 x=0;x<X_WIDTH;x++)
			wrData(data);
	}
}

void Oled::cls(void)
{
	for(u8 y=0;y<8;y++)
	{
		setPos(0,y);
		for(u8 x=0;x<X_WIDTH;x++)
			wrData(0x00);
	}
}
//清除第y页
void Oled::clsY(u8 y)
{
	setPos(0,y);
	for(u8 x=0;x<X_WIDTH;x++)
		wrData(0x00);	
}
//画一页横线
void Oled::lineY(u8 y)
{
	setPos(0,y);
	for(u8 x=0;x<X_WIDTH;x++)
		wrData(0xff);
}

//清除y页上的一块区域
void Oled::clsLineArea(u8 y,u8 from,u8 width)
{
	setPos(from,y);
	for(u8 x=from;x<width;x++)
		wrData(0x00);
}

void Oled::OLED_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	RCC_AHB1PeriphClockCmd(GPIO_TO_RCC(SCL.GPIOx), ENABLE);
	GPIO_InitStructure.GPIO_Pin = SCL.Pinx;
	GPIO_Init(SCL.GPIOx, &GPIO_InitStructure);
	RCC_AHB1PeriphClockCmd(GPIO_TO_RCC(SDA.GPIOx), ENABLE);
	GPIO_InitStructure.GPIO_Pin = SDA.Pinx;
	GPIO_Init(SDA.GPIOx, &GPIO_InitStructure);
	RCC_AHB1PeriphClockCmd(GPIO_TO_RCC(RST.GPIOx), ENABLE);
	GPIO_InitStructure.GPIO_Pin = RST.Pinx;
	GPIO_Init(RST.GPIOx, &GPIO_InitStructure);
	RCC_AHB1PeriphClockCmd(GPIO_TO_RCC(DC.GPIOx), ENABLE);
	GPIO_InitStructure.GPIO_Pin = DC.Pinx;
	GPIO_Init(DC.GPIOx, &GPIO_InitStructure);
	#if USE_CS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//CS配置
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	#endif
	SCL_SET();
	SDA_SET();
}

void Oled::init(u8 ID)        
{
	if(resetCnt==0)
	{
		OLED_GPIO_Init();
		RST_SET();
		delayMs(10);
		RST_CLR();
		delayMs(10);
		RST_SET();
		resetCnt=1;
	}
	#if USE_CS
		OLED_CS(ID);
	#endif
	//ssd1306显示屏每次重启(上电)后，需要主动发送配置信息(顺序一定)，否则无法显示,一下即为固定配置:
	wrCmd(0xae);//--turn off oled panel
	wrCmd(0x00);//---set low column address
	wrCmd(0x10);//---set high column address
	wrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	wrCmd(0x81);//--set contrast control register
	wrCmd(Brightness); // Set SEG Output Current Brightness
	wrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	wrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	wrCmd(0xa6);//--set normal display
	wrCmd(0xa8);//--set multiplex ratio(1 to 64)
	wrCmd(0x3f);//--1/64 duty
	wrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	wrCmd(0x00);//-not offset
	wrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
	wrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	wrCmd(0xd9);//--set pre-charge period
	wrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	wrCmd(0xda);//--set com pins hardware configuration
	wrCmd(0x12);
	wrCmd(0xdb);//--set vcomh
	wrCmd(0x40);//Set VCOM Deselect Level
	wrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
	wrCmd(0x02);//
	wrCmd(0x8d);//--set Charge Pump enable/disable
	wrCmd(0x14);//--set(0x10) disable
	wrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
	wrCmd(0xa6);// Disable Inverse Display On (0xa6/a7) 
	wrCmd(0xaf);//--turn on oled panel
	fill(0x00);  //初始清屏
	setPos(0,0);  
} 
//==============================================================
//函数名： void PutPixel(u8 x,u8 y)
//功能描述：绘制一个点（x,y）
//参数：真实坐标值(x,y),x的范围0～127，y的范围0～64
//返回：无
//==============================================================
void Oled::putPixel(u8 x,u8 y)
{
	u8 data1 = 0x01<<(y%8); 
    setPos(x,(y>>3)); 	
	wrData(data1);
}
//==============================================================
//函数名： void Put_Column(u8 x,u8 y,u8 data)
//功能描述：操作一列显示，一列全显示0XFF,清除一列0X00;
//参数：真实坐标值(x,y),x的范围0～127，y的范围0～64
//返回：无
//==============================================================
void Oled::putColumn(u8 x,u8 data)
{	
	for(u8 y=0;y<8;y++)
	{
		setPos(x,y);
		wrData(data); 
	}	
}
//==============================================================
//函数名： void Rectangle(u8 x1,u8 y1,
//                   u8 x2,u8 y2,u8 color,u8 gif)
//功能描述：绘制一个实心矩形
//参数：左上角坐标（x1,y1）,右下角坐标（x2，y2）
//      其中x1、x2的范围0～127，y1，y2的范围0～63，即真实坐标值
//返回：无
//==============================================================
//@warning 未完成
void Oled::rectangle(u8 x1,u8 y1,u8 x2,u8 y2)
{
	setPos(x1,y1>>3);
	for(u8 n=x1;n<=x2;n++)
	{
		wrData(0x01<<(y1%8)); 			
	}  
	setPos(x1,y2>>3);
  	for(u8 n=x1;n<=x2;n++)
	{
		wrData(0x01<<(y2%8)); 			
	}
	
}  
//==============================================================
//函数名：P6x8Str(u8 x,u8 y,u8 *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================  
void Oled::P6x8Str(u8 x,u8 y,u8 *ch)
{
	u8 c=0,i=0,j=0;

	while (*(ch+j)!='\0')
	{    
		c =*(ch+j)-32;
		if(x>X_WIDTH-6)//129 - 6 
		{
			x=0;
			y++;
		}
		setPos(x,y);    
		for(i=0;i<6;i++) 
		{
			wrData(*(F6x8+c*6+i));
		}
		x+=6;
		j++;
	}
}

void Oled::P6x8Str(u8 x,u8 y,u8 *ch,u8 reverse)
{
	u8 c=0,i=0,j=0;

	while (*(ch+j)!='\0')
	{    
		c =*(ch+j)-32;
		if(x>X_WIDTH-6)//129 - 6 
		{
			x=0;
			y++;
		}
		setPos(x,y);    
		for(i=0;i<6;i++) 
		{
			if(reverse)
				wrData(0xff-*(F6x8+c*6+i));
			else
				wrData(*(F6x8+c*6+i));
		}
		x+=6;
		j++;
	}
}
//==============================================================
//函数名：P8x16Str(u8 x,u8 y,u8 *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～63，要显示的字符串
//返回：无
//==============================================================  
void Oled::P8x16Str(u8 x,u8 y,u8 *ch)
{
  u8 c=0,i=0,j=0;
        
  while (*(ch+j)!='\0')
  {    
    c =*(ch+j)-32;
    if(x>X_WIDTH -16)//129-16
	{	
		x=0;
		y+=2;
	}
    setPos(x,y); 
  	for(i=0;i<8;i++) 
  	{
  	  	wrData(*(F8X16+c*16+i));
  	}
  	x+=8;
  	j++;
  }
}
//输出汉字字符串
void Oled::P16x16Str(u8 x,u8 y,u8 *ch)
{
	u8 wm=0,ii = 0;
	u16 adder=1; 
	
	while(*(ch+ii) != '\0')
	{
  	wm = 0;
  	adder = 1;
  	while(*(F16x16_Idx+wm) > 127)
  	{
  		if(*(F16x16_Idx+wm) == *(ch+ii))
  		{
  			if(*(F16x16_Idx+wm + 1) == *(ch+ii + 1))
  			{
  				adder = wm * 16;
  				break;
  			}
  		}
  		wm += 2;			
  	}
  	if(x>118){x=0;y++;}
  	setPos(x , y); 
  	if(adder != 1)// 显示汉字					
  	{
  		setPos(x , y);
  		for(wm = 0;wm < 16;wm++)               
  		{
  			wrData(*(F16x16+adder));	
  			adder += 1;
  		}      
  		setPos(x,y + 8); 
  		for(wm = 0;wm < 16;wm++)          
  		{
  			wrData(*(F16x16+adder));
  			adder += 1;
  		}   		
  	}
  	else			  //显示空白字符			
  	{
  		ii += 1;
      	setPos(x,y);
  		for(wm = 0;wm < 16;wm++)
  		{
  			wrData(0);
  		}
  		setPos(x,y + 1);
  		for(wm = 0;wm < 16;wm++)
  		{   		
  			wrData(0);	
  		}
  	}
  	x += 16;
  	ii += 2;
	}
}

/*
输出汉字和字符混合字符串
*Y轴是按8格递进的，y轴0~63，只能按8格的倍数显示，
*因为列行式只能按8个字节进行
*


void Print(u8 x, u8 y, u8 *ch,u8 char_size, u8 ascii_size)
{
	u8 i,n,t=0,m=0,r;
	while(1)
	{
		if(m==0)
		{
			ch2[0] = *(ch+ii);
	 		ch2[1] = *(ch+ii+1);
			ch2[2] = '\0';			//汉字为两个字节
			P16x16Str(x , y, ch2,F16x16_Idx,F16x16);	//显示汉字
			x += 16;
			ii += 2;
		}
		if(t==num)
			{
				for(r=0;r<16*space;r++)      //显示间隔
				 {
					for(i=1;i<144;i++)
						{
							for(n=0;n<8;n++)
							{
								OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
							}
						}
           OLED_Refresh();
				 }
        t=0;
      }
		m++;
		if(m==16){m=0;}
		for(i=1;i<144;i++)   //实现左移
		{
			for(n=0;n<8;n++)
			{
				P8x16Str(x , y ,ch2,F8X16);	//显示字母
				x += 8;
			}
			else if(TYPE6X8==ascii_size)
			{
				P6x8Str(x , y ,ch2,F6x8);	//显示字母
				x += 6;
			}
			
			ii+= 1;
		}
		OLED_Refresh();
	}
}

//==============================================================
//函数名： void Draw_BMP(u8 x,u8 y)
//功能描述：显示BMP图片128×64
//参数：起始点坐标(x,y),x的范围0～127，y为页的范围0～63
//返回：无
//==============================================================
void Draw_BMP(u8 x,u8 y,const u8 *bmp)
{ 	
	u8 wm=0;
	u8 adder=0;
  
  	setPos(x , y);
	for(wm = 0;wm < 16;wm++)               
	{
		WrDat(*(bmp+adder));	
		adder += 1;
	}      
	setPos(x,y + 8); 
	for(wm = 0;wm < 16;wm++)          
	{
		WrDat(*(bmp+adder));
		adder += 1;
	} 
  u32 ii=0;
  u8 x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<=y1;y++)
	{
		setPos(x0,y);				
    for(x=x0;x<x1;x++)
	    {      
	    	WrDat(bmp[ii++]);	    	
	    }
	}	
}*/