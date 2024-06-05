#include <reg51.h>
typedef unsigned char u8;
typedef unsigned char u16;
sbit LED1 = P1^0;
sbit LED2 = P1^1;
sbit LED3 = P1^2;
sbit LED4 = P1^3;
sbit BIT1 = P2^0;
sbit BIT2 = P2^1;
sbit BIT3 = P2^2;
sbit BIT4 = P2^3;
u8 key,mode,oldmode;
void DelayMS(u16 x)
{
 	u8 i;
	while(x--)
	{
	 	for(i=120;i>0;i--);
	}
}
void led_off()
{
	P1 =0xff;
}
int keyscan()
{
	if(BIT1==0)//左转
	{
		key = 1;
	}
	else if(BIT2==0)//左转
	{
		key = 2;
	}
	else if(BIT3==0)//左转
	{
		key = 3;
	}
	else if(BIT4==0)//左转
	{
		key = 4;
	}
	else
		key=0;
		return key;
}
void led()
{
	if(oldmode!=mode)
	{
		led_off();
		oldmode=mode;
	}
	if(mode==1)
	{
		LED1 = 0;
	}
	else if(mode == 2)
	{
		LED2 = 0;
	}
	else if(mode == 3)
	{
		LED3 = 1;
		DelayMS(500);
		LED3 = 0;
		DelayMS(500);
	}
	else if(mode == 4)
	{
		LED4 = 1;
		DelayMS(500);
		LED4 = 0;
		DelayMS(500);
	}
	if(mode==0)
	{
		led_off();
	}
}
void main()
{
 	while(1)
	{
		key=keyscan();
		 mode=key;
		led();
	}
}