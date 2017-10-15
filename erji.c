#include "reg52.h"
#include "intrins.h"
#include "math.h"
sbit SCL=P2^7;
sbit SDA=P2^6;
#define SlaveAddress 0x3c
#define FOSC 11059200L          //系统频率
#define BAUD 34560            //串口波特率	//最终波特
#define M_PI 3.1415926
#define NONE_PARITY     0       //无校验
#define ODD_PARITY      1       //奇校验
#define EVEN_PARITY     2       //偶校验
#define MARK_PARITY     3       //标记校验
#define SPACE_PARITY    4       //空白校验
sfr P0M1 = 0x93;
sfr P0M0 = 0x94;
sfr P1M1 = 0x91;
sfr P1M0 = 0x92;
sfr P2M1 = 0x95;
sfr P2M0 = 0x96;
sfr P3M1 = 0xB1;
sfr P3M0 = 0xB2;
sfr P4M1 = 0xB3;
sfr P4M0 = 0xB4;
sfr P5M1 = 0xC9;
sfr P5M0 = 0xCA;
sfr P6M1 = 0xCB;
sfr P6M0 = 0xCC;
sfr P7M1 = 0xE1;
sfr P7M0 = 0xE2;

sfr AUXR  = 0x8e;               //辅助寄存器

sfr P_SW1   = 0xA2;             //外设功能切换寄存器1
sfr INT_CLKO=0X8F;
#define S1_S0 0x40              //P_SW1.6
#define S1_S1 0x80              //P_SW1.7

sbit P22 = P2^2;
char Rec_Data[6];
char buf[3];
bit busy;
void InitUart();

/*延时*/
void Delay(int t)
{
	while(t--)
	{}
}
/*起始信号*/
void IIC_Start(void)
{
	SDA=1;
	SCL=1;
	Delay(5);
	SDA=0;
	Delay(5);
	SCL=0;
}
/*停止信号*/
void IIC_Stop(void)
{
	SDA=0;
	SCL=1;
	Delay(5);
	SDA=1;
	Delay(5);
}
void IIC_SendAck(bit Ack)
{
	SDA=Ack;
	SCL=1;
	Delay(5);
	SCL=0;
	Delay(5);
}
bit IIC_RecAck(void)
{
	SCL=1;
	Delay(5);
	CY=SDA;
	SCL=0;
	Delay(5);
	return CY;
}
void HMC5883_Send_Byte(char Dat)
{
	char i;
	for (i=0;i<8;i++)
	{
		Dat<<=1;
		SDA=CY;
		SCL=1;
		Delay(5);
		SCL=0;
		Delay(5);
	}
	IIC_RecAck();
}
char HMC5883_Rec_Byte(void)
{
	char i,Dat=0;
	SDA=1;
	for (i=0;i<8;i++)
	{
		Dat<<=1;
		SCL=1;
		Delay(5);
		Dat|=SDA;
		SCL=0;
		Delay(5);
	}
	return Dat;
}
void   Single_Write_HMC5883(char Address,char Dat)
{
	IIC_Start();
	HMC5883_Send_Byte(SlaveAddress);
	HMC5883_Send_Byte(Address);
	HMC5883_Send_Byte(Dat);
	IIC_Stop();
}
char Single_Read_HMC5883(char Addr)
{
	char Value;
	IIC_Start();
	HMC5883_Send_Byte(SlaveAddress);
	HMC5883_Send_Byte(Addr);
	IIC_Start();
	HMC5883_Send_Byte(SlaveAddress+1);
	Value=HMC5883_Rec_Byte();
	IIC_SendAck(1);
	IIC_Stop();
	return Value;
}
void Multiple_Read_HMC5883(void)
{
	int i;
	IIC_Start();
	HMC5883_Send_Byte(SlaveAddress);
	HMC5883_Send_Byte(0x03);
	IIC_Start();
	HMC5883_Send_Byte(SlaveAddress+1);
	for (i=0;i<6;i++)
	{
	  	Rec_Data[i]=HMC5883_Rec_Byte();
		if(i==5)
			IIC_SendAck(1);
		else
			IIC_SendAck(0);
	}
	IIC_Stop();
	Delay(100);
}
void HMC5883_Init(void)
{
	Single_Write_HMC5883(0x00,0x70);
	Single_Write_HMC5883(0x01,0x20);
	Single_Write_HMC5883(0x02,0x00);
	
}
void SendData(char dat)
{
    SBUF = dat;                     //发送当前数据
	while (!TI);                    //等待前一个数据发送完成
    TI = 0;                         //清除发送标志
}
int fillBuffer(int x1, int x2) 
{   
  x1 = x1 ;   
  x2 = x2 ;  
  buf[0] = x1;  
  buf[1] = x2;  
}   

void main(void)
{	
	char Acr;
	double X,Y,Z;
	int i;
	float heading,heading2;
	int x1;
	int x2;
	P0M0 = 0x00;
    P0M1 = 0x00;
    P1M0 = 0x00;
    P1M1 = 0x00;
    P2M0 = 0x00;
    P2M1 = 0x00;
    P3M0 = 0x00;
    P3M1 = 0x00;
    P4M0 = 0x00;
    P4M1 = 0x00;
    P5M0 = 0x00;
    P5M1 = 0x00;
    P6M0 = 0x00;
    P6M1 = 0x00;
    P7M0 = 0x00;
    P7M1 = 0x00;
	INT_CLKO|=0X40;
	EA=1;
	InitUart();
	HMC5883_Init();
	buf[2]=0;
	while(1)
	{
		Multiple_Read_HMC5883();
		X=Rec_Data[0]<<8|Rec_Data[1];
		Z=Rec_Data[2]<<8|Rec_Data[3];
		Y=Rec_Data[4]<<8|Rec_Data[5];
		heading = atan2(X, Y);  
		if(heading < 0)    
			heading += 2 * M_PI;  
		x1=heading*180/M_PI/360*256;  		  
		heading2 = atan2(Y, Z);  
		if(heading2 < 0)  
			heading2 += 2 * M_PI;  
		x2=heading2*180/M_PI/360*256;  
		fillBuffer(x1,x2);  
		for ( i=0;i<3;i++)
		 {
		 	SendData(buf[i]);
			Delay(50);   
		 }
		Delay(50);
	}
}
void InitUart()
{
    ACC = P_SW1;
    ACC &= ~(S1_S0 | S1_S1);    //S1_S0=0 S1_S1=0
    P_SW1 = ACC;                //(P3.0/RxD, P3.1/TxD)

#if (PARITYBIT == NONE_PARITY)
    SCON = 0x50;                //8位可变波特率
#elif (PARITYBIT == ODD_PARITY) || (PARITYBIT == EVEN_PARITY) || (PARITYBIT == MARK_PARITY)
    SCON = 0xda;                //9位可变波特率,校验位初始为1
#elif (PARITYBIT == SPACE_PARITY)
    SCON = 0xd2;                //9位可变波特率,校验位初始为0
#endif

    AUXR = 0x40;                //定时器1为1T模式
    TMOD = 0x00;                //定时器1为模式0(16位自动重载)
    TL1 = (65536 - (FOSC/4/BAUD));   //设置波特率重装值
    TH1 = (65536 - (FOSC/4/BAUD))>>8;
    TR1 = 1;                    //定时器1开始启动
	PT0=1;
	ES = 1;                     //使能串口中断
    EA = 1;
	ET0=1;
	TR0=0;
}
