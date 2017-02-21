#include  <msp430g2553.h>
#include <msp430.h>
#define   uchar unsigned char
#define   uint  unsigned int
typedef unsigned char BYTE;
typedef unsigned short WORD;

//#define debug
#define normal

#define V_T1s	1   //串口的发送速率，数越大串口发送的延时越大
#define V_T5s	1  //当在剧烈变动时，监测周期为5s
#define V_T20s	4*20  //当在稳定状态时，监测周期为20s

#define H0      50    //一开始设置xyz0方差的计算
#define H1      10//5     //和x0、y0、z0的差小于哪个范围当0
#define  H2     600   //D-G的阈值
#define  HH     50//100     //那四组数据方差小于多少才算稳定

#define N0       10  //N0要是10才行。。。。不知道为什么调成9就不行了

unsigned int clock=0;
unsigned char clock_flag=0;
unsigned char buf_full = 0;
unsigned char digi[5]={0,0,0,0,0};
//使用的端口，请按照以下接线
#define SCL_H P1OUT |= BIT6
#define SCL_L P1OUT &= ~BIT6
#define SDA_H P1OUT |= BIT7
#define SDA_L P1OUT &= ~BIT7
#define SDA_in  P1DIR &= ~BIT7   //SDA改成输入模式
#define SDA_out P1DIR |= BIT7    //SDA变回输出模式
#define SDA_val P1IN&BIT7        //SDA的位值

#define EXIST_H P2OUT |= BIT0
#define NON_EXIST_L P2OUT &= ~BIT0
#define MEIWENDING_H P2OUT |= BIT1
#define WENDING_L P2OUT &= ~BIT1

#define TRUE    1
#define FALSE   0

#define INITIAL 0
#define STEADY 1
#define CHANGE 2

#define	SlaveAddress   0x3C	  //定义器件5883在IIC总线中的从地址

BYTE BUF[8];                         //接收数据缓存区
uchar ge,shi,bai,qian,wan;           //显示变量
int  dis_data;                       //变量
uint xyz00[3];
uint x10buf[N0],y10buf[N0],z10buf[N0];
uint S2xyz[3];   //10个数据的方差
uint avg10x,avg10y,avg10z;
uchar set_flag=0;

uint S2=0;
uint G0=0,S0=0;
//************
uint V_T=V_T1s;

uchar state=0;
uchar steady_flag=0;

void port_init(void)
{
	    P1DIR |= BIT6;      // P1.6 output  // 设置SCL_OUT
	    /*
	    State： P0.0		——P2.0   有车还是没车  output
		Busy:   P0.1	——P2.1 单片机数据稳定了可以发送了  output
		SEL1:   P1.2	——P2.2 input
		SEL2:   P1.3	——P2.3 input
		RECORD : P1.1	——连到了单片机的reset
		是否zigbee在发送 : P1.0	——P2.4   input    高的话就不要测量，低就启动测量
	     */
	    P2DIR |= BIT0+BIT1;
	    P2DIR &= ~BIT2;
	    P2DIR &= ~BIT3;
	    P2DIR &= ~BIT4;
}

//TIMER0 initialize -
// desired value: 5ms
#ifdef normal
void timer0_init(void)
{
	// Configure Timer0
		TA0CTL = TASSEL_1 + MC_1 ;          //  SMCLK=1MHz, UP mode  选择ACLK
		CCTL0 = CCIE;                  	//CCR0 interrupt enabled
		//TA0CCR0 = 1200;
		TA0CCR0 = 11450/4;

		//TA0CCR0 = 11450;
}

// Timer0_A0 interrupt service routine

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
		if (++clock>=V_T)
		{
			clock_flag = 1;
			clock = 0;
			LPM3_EXIT;
	     }
}
#endif normal

void uart_init(void)
{
	P1SEL |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	P1SEL2 |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	UCA0CTL1 |= UCSSEL_2;                     // uart鏃堕挓: SMCLK=1MHz
	UCA0BR0 = 104;                            // 璁剧疆娉㈢壒鐜?9600
	UCA0BR1 = 0;                              // 璁剧疆娉㈢壒鐜?9600
	UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
}

void init_devices(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

		if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)
		{
			while(1);            	// If calibration constants erased, trap CPU!!
		}

		//鎸崱婧愪负鐗囧唴RC鎸崱鍣紝DCO=8MHz,渚汣PU;      SMCLK=1MHz,渚涘畾鏃跺櫒A0銆佷覆琛屽彛UART
		BCSCTL1 = CALBC1_1MHZ; 		    // Set range
		DCOCTL = CALDCO_1MHZ;  		    // Set DCO step + modulation锛孌CO=8MHz
		BCSCTL3 |= LFXT1S1;                      // LFXT1 = VLO
		IFG1 &= ~OFIFG;                           // Clear OSCFault flag


 port_init();
 timer0_init();
 uart_init();
 _BIS_SR(GIE);   //将GIE位置1，开启总中断
 //all peripherals are now initialized
}



void conversion(uint temp_data)
{
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;   //取余运算
	qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;    //取余运算
    bai=temp_data/100+0x30   ;
    temp_data=temp_data%100;     //取余运算
    shi=temp_data/10+0x30    ;
    temp_data=temp_data%10;      //取余运算
    ge=temp_data+0x30;
}
/*******************************/
void delay(unsigned int k)
{
unsigned int i,j;
for(i=0;i<k;i++)
{
for(j=0;j<121;j++)
{;}}
}


/**************************************
延时5微秒(STC90C52RC@12M)
不同的工作环境,需要调整此函数，注意时钟过快时需要修改
当改用1T的MCU时,请调整此延时函数
**************************************/
void Delay5us()
{
    _delay_cycles(5);
}

/**************************************
延时5毫秒(STC90C52RC@12M)
不同的工作环境,需要调整此函数
当改用1T的MCU时,请调整此延时函数
**************************************/
void Delay5ms()
{
    unsigned short n = 560;

    while (n--);
}

/**************************************
起始信号
**************************************/
void HMC5883_Start()
{
    SDA_out;
	SDA_H;                    //拉高数据线
    SCL_H;                    //拉高时钟线
    Delay5us();                 //延时
    SDA_L;                    //产生下降沿
    Delay5us();                 //延时
    SCL_L;                    //拉低时钟线
}

/**************************************
停止信号
**************************************/
void HMC5883_Stop()
{
	SDA_out;
    SDA_L;                    //拉低数据线
    SCL_H;                    //拉高时钟线
    Delay5us();                 //延时
    SDA_H;                    //产生上升沿
    Delay5us();                 //延时
}

/**************************************
发送应答信号
入口参数:ack (0:ACK 1:NAK)
**************************************/
void HMC5883_SendACK1()
{
	SDA_out;
	SDA_H;                  //写应答信号
    SCL_H;                    //拉高时钟线
    Delay5us();                 //延时
    SCL_L;                    //拉低时钟线
    Delay5us();                 //延时
}
void HMC5883_SendACK0()
{
	SDA_out;
	SDA_L;                  //写应答信号
    SCL_H;                    //拉高时钟线
    Delay5us();                 //延时
    SCL_L;                    //拉低时钟线
    Delay5us();                 //延时
}

//**************************************//
//接收应答信号/
//**************************************//
uchar HMC5883_RecvACK()
{
    uchar a;
    SDA_in;

    SCL_H;                    //拉高时钟线
    Delay5us();                 //延时

    a = SDA_val;                 //读应答信号
    SCL_L;                    //拉低时钟线
    Delay5us();                 //延时
    SDA_out;

    return a;
}
/**************************************
向IIC总线发送一个字节数据
**************************************/
void HMC5883_SendByte(uchar dat)
{
    uchar i;
    SDA_out;
    for(i = 0;i < 8;i++)//8位计数器
    {

       if(dat & 0x80)//1
            SDA_H;
        else
            SDA_L;
        SCL_H;
        Delay5us();
        SCL_L;
        Delay5us();
        dat <<= 1;
    }
    HMC5883_RecvACK();
}

/**************************************
从IIC总线接收一个字节数据
**************************************/
uchar HMC5883_RecvByte()
{
    uchar  rdata = 0x00,i;
    uchar flag;

    SDA_H;
    for(i = 0;i < 8;i++)
    {
            SCL_H;
    SDA_in;
            Delay5us();
            flag = SDA_val;
            rdata <<= 1;
            if(flag)
                rdata |= 0x01;
    SDA_out;
            SCL_L;
            Delay5us();
    }
    return rdata;
}

//***************************************************

void Single_Write_HMC5883(uchar REG_Address,uchar REG_data)
{
    HMC5883_Start();                  //起始信号
    HMC5883_SendByte(SlaveAddress);   //发送设备地址+写信号
    HMC5883_SendByte(REG_Address);    //内部寄存器地址，请参考中文pdf
    HMC5883_SendByte(REG_data);       //内部寄存器数据，请参考中文pdf
    HMC5883_Stop();                   //发送停止信号
}
//********单字节读取内部寄存器*************************//

uchar Single_Read_HMC5883(uchar REG_Address)
{
    uchar REG_data;
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress);           //发送设备地址+写信号
    HMC5883_SendByte(REG_Address);                   //发送存储单元地址，从0开始
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress+1);         //发送设备地址+读信号
    REG_data = HMC5883_RecvByte();              //读出寄存器数据
    HMC5883_SendACK1();
    HMC5883_Stop();                           //停止信号
    return REG_data;
}
//******************************************************//
//
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//
//******************************************************//
void Multiple_Read_HMC5883(void)
{
	uchar i;
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress);           //发送设备地址+写信号                                 //0x3C是写地址
    HMC5883_SendByte(0x03);                   //发送存储单元地址，从0x3开始是Data Output X MSB Register，3、4是X  5、6是Y  7、8是Z
    HMC5883_Start();                          //起始信号
    HMC5883_SendByte(SlaveAddress+1);         //发送设备地址+读信号                                       //0x3D是读地址
    for (i=0; i<6; i++)                       //连续读取6个地址数据，存储中BUF    3、4是X  5、6是Y  7、8是Z
    {
        BUF[i] = HMC5883_RecvByte();          //BUF[0]存储数据
        if (i == 5)
        {
           HMC5883_SendACK1();                //最后一个数据需要回NOACK
        }
        else
        {
          HMC5883_SendACK0();                //回应ACK
       }
   }
    HMC5883_Stop();                          //停止信号
    _delay_cycles(5000);
}

//初始化HMC5883，根据需要请参考pdf进行修改****
void Init_HMC5883()
{									   //void Single_Write_HMC5883(uchar REG_Address,uchar REG_data)
  // Single_Write_HMC5883(0x02,0x00);  //02是Mode Register  最低的两位设置成00的话就是连续测量模式，设置成01是单次测量模式        10、11均是 idle model      ！！！注意：MR5有一个设置的Lowest power mode.
     Single_Write_HMC5883(0x02,0x01);
     //Single_Write_HMC5883(0x00,0x70);
     Single_Write_HMC5883(0x00,0x7C);
     //Single_Write_HMC5883(0x01,0xE0);  //01：Configuration Register B       The configuration register B for setting the device gain.    CRB7-CRB5 ： Gain
     Single_Write_HMC5883(0x01,0xA0);
}

uint d(uint aa,uint bb)
{
	uint tt;
	tt=(aa-bb)*(aa-bb);
	return tt;
}

void countS2(void)
{
	uchar k;
    long int tmp;
    ///////////////////////////////
    for(k=0,tmp=0;k<N0;k++)
    	tmp+=x10buf[k];
    avg10x=tmp/N0;
    for(k=0,tmp=0;k<N0;k++)
    	tmp+=(x10buf[k]-avg10x)*(x10buf[k]-avg10x);
    S2xyz[0]=tmp/N0;
    ///////////////////////////////
    for(k=0,tmp=0;k<N0;k++)
    	tmp+=y10buf[k];
    avg10y=tmp/N0;
    for(k=0,tmp=0;k<N0;k++)
    	tmp+=(y10buf[k]-avg10y)*(y10buf[k]-avg10y);
    S2xyz[1]=tmp/N0;
    ///////////////////////////////
    for(k=0,tmp=0;k<N0;k++)
    	tmp+=z10buf[k];
    avg10z=tmp/N0;
    for(k=0,tmp=0;k<10;k++)
    	tmp+=(z10buf[k]-avg10z)*(z10buf[k]-avg10z);
    S2xyz[2]=tmp/N0;
}

void setxyz0(void)
{
	if(S2xyz[0]<H0 && S2xyz[1]<H0 && S2xyz[2]<H0)
	{
		xyz00[0]=avg10x;
		xyz00[1]=avg10y;
		xyz00[2]=avg10z;
		set_flag=1;
		steady_flag=1;
	}
}


uint distance(uint xx1,uint xx2,uint yy1,uint yy2,uint zz1,uint zz2)
{
	return (d(xx1,xx2)+d(yy1,yy2)+d(zz1,zz2)) ;
}

uint absu(uint a1,uint a2)
{
	if(a1>=a2)  return (a1-a2);
	else return (a2-a1);
}

void Set_G_S(void)
{
	//情况1：那个距离小，所以没关系

	if(absu(S2,G0)<H2)
	{
		if(S2>=S0)
			G0=S2-S0;
		else
		{
			S0=0;
			G0=S2;
		}
	}
	else
	{
		S0=S2-G0;
	}
}


void Init_state(void)
{
//NON_EXIST_L;
EXIST_H;
MEIWENDING_H;
set_flag=0;

S2=0;
G0=0;
S0=0;
V_T=V_T1s;

state=0;
steady_flag=0;




}

void main(void)
{

    unsigned int i=0;
	unsigned char count=0,bufcount=0;
	long int xbuf[4],ybuf[4],zbuf[4];
	uint xyz[3];
    uint avg[3];
    uint digxyz[3];
    uint digxyz0[3];

    uint SS2[3];
    long int ttemp[3];
    uint aavg[3];

    uchar fuhaoxyz[3];
    uchar fuhaoxyz0[3];
    uchar exist=0;

    uchar buf_used[4];

    uchar abolish=0;

	INIT_num:
		Init_state();
		i=0;
count=0;
bufcount=0;
	
 exist=0;


 abolish=0;
	init_devices( );
	_delay_cycles(500000);

	Init_HMC5883();
	buf_full = 0;
    for(i=0;i<4;i++)
    {
    	xbuf[i]=0;
    	ybuf[i]=0;
    	zbuf[i]=0;
    	buf_used[i]=0;
    }
#ifdef debug
    _delay_cycles(5000000);
    LPM3;
    LPM3;
#endif debug
	 while(1)
	 	{
	 	while(!(P2IN & BIT4)) 
	  {

		if (clock_flag==1)
		{
			clock_flag=0;
			abolish=0;
wrong_num:

	       //Multiple_Read_HMC5883();      //连续读出数据，存储在BUF中

			Single_Write_HMC5883(0x02,0x01);
			_delay_cycles(10000);
			Multiple_Read_HMC5883();      //连续读出数据，存储在BUF中
			//Multiple_Read_HMC5883();      //连续读出数据，存储在BUF中

		    
		    Multiple_Read_HMC5883();
			xyz[0] = BUF[0] << 8 | BUF[1];   //X
			xyz[1] = BUF[4] << 8 | BUF[5];   //Y
			xyz[2] = BUF[2] << 8 | BUF[3];   //Z
		    xbuf[count]=xyz[0];
		    ybuf[count]=xyz[1];
		    zbuf[count]=xyz[2];
		    buf_used[count]=1;

				avg[0]=xbuf[count];
				avg[1]=ybuf[count];
				avg[2]=zbuf[count];
				if((avg[0]==0xFFFF && avg[1]==0xFFFF && avg[2]==0xFFFF))
					goto wrong_num;
				count++;

				// 假如还没有设置环境值
				if(set_flag==0)
				{
					x10buf[bufcount]=avg[0];
					y10buf[bufcount]=avg[1];
					z10buf[bufcount]=avg[2];

					if(++bufcount>=N0 )
					{
						//bufcount=0;
						//countS2();
						//setxyz0();
						buf_full = 1;
					}
					if(buf_full ==1 )
					{
						//bufcount=0;
						bufcount = bufcount % N0;
						countS2();
						setxyz0();
						//buf_full = 1;
					}
				}

				// 设置环境值了
				else if(set_flag && !steady_flag)
				{
					if(buf_used[0] && !buf_used[1] && !buf_used[2] && !buf_used[3])
						steady_flag=0;
					else if(buf_used[0] && buf_used[1] && !buf_used[2] && !buf_used[3])
					{
						ttemp[0]=(xbuf[0]+xbuf[1]);
						ttemp[1]=(ybuf[0]+ybuf[1]);
						ttemp[2]=(zbuf[0]+zbuf[1]);
						for(i=0;i<3;i++)
							aavg[i]=ttemp[i]/2;
						ttemp[0]=d(xbuf[0],aavg[0])+d(xbuf[1],aavg[0]);
						SS2[0]=ttemp[0]/2;
						if(SS2[0]>HH)
							abolish=1;

						ttemp[1]=d(ybuf[0],aavg[1])+d(ybuf[1],aavg[1]);
						SS2[1]=ttemp[1]/2;
						if(SS2[1]>HH)
							abolish=1;

						ttemp[2]=d(zbuf[0],aavg[2])+d(zbuf[1],aavg[2]);
						SS2[2]=ttemp[2]/2;
						if(SS2[2]>HH)
							abolish=1;
						if(abolish)
						{
							count=1;
							xbuf[0]=xbuf[1];
							ybuf[0]=ybuf[1];
							zbuf[0]=zbuf[1];
							buf_used[1]=0; buf_used[2]=0; buf_used[3]=0;
						}
						steady_flag=0;
					}
					else if(buf_used[0] && buf_used[1] && buf_used[2] && !buf_used[3])
					{
						ttemp[0]=(xbuf[0]+xbuf[1]+xbuf[2]);
						ttemp[1]=(ybuf[0]+ybuf[1]+ybuf[2]);
						ttemp[2]=(zbuf[0]+zbuf[1]+zbuf[2]);
						for(i=0;i<3;i++)
							aavg[i]=ttemp[i]/3;
						ttemp[0]=d(xbuf[0],aavg[0])+d(xbuf[1],aavg[0])+d(xbuf[2],aavg[0]);
						SS2[0]=ttemp[0]/3;
						if(SS2[0]>HH)
							abolish=1;

						ttemp[1]=d(ybuf[0],aavg[1])+d(ybuf[1],aavg[1])+d(ybuf[2],aavg[1]);
						SS2[1]=ttemp[1]/3;
						if(SS2[1]>HH)
							abolish=1;

						ttemp[2]=d(zbuf[0],aavg[2])+d(zbuf[1],aavg[2])+d(zbuf[2],aavg[2]);
						SS2[2]=ttemp[2]/3;
						if(SS2[2]>HH)
							abolish=1;
						if(abolish)
						{
							steady_flag=0;
							count=1;
							xbuf[0]=xbuf[2];
							ybuf[0]=ybuf[2];
							zbuf[0]=zbuf[2];
							buf_used[1]=0; buf_used[2]=0; buf_used[3]=0;
						}
						steady_flag=0;
					}
					else if(buf_used[0] && buf_used[1] && buf_used[2] && buf_used[3])
					{
						ttemp[0]=(xbuf[0]+xbuf[1]+xbuf[2]+xbuf[3]);
						ttemp[1]=(ybuf[0]+ybuf[1]+ybuf[2]+ybuf[3]);
						ttemp[2]=(zbuf[0]+zbuf[1]+zbuf[2]+zbuf[3]);
						for(i=0;i<3;i++)
							aavg[i]=ttemp[i]/4;
						ttemp[0]=d(xbuf[0],aavg[0])+d(xbuf[1],aavg[0])+d(xbuf[2],aavg[0])+d(xbuf[3],aavg[0]);
						SS2[0]=ttemp[0]/4;
						if(SS2[0]>HH)
							abolish=1;

						ttemp[1]=d(ybuf[0],aavg[1])+d(ybuf[1],aavg[1])+d(ybuf[2],aavg[1])+d(ybuf[3],aavg[1]);
						SS2[1]=ttemp[1]/4;
						if(SS2[1]>HH)
							abolish=1;

						ttemp[2]=d(zbuf[0],aavg[2])+d(zbuf[1],aavg[2])+d(zbuf[2],aavg[2])+d(zbuf[3],aavg[2]);
						SS2[2]=ttemp[2]/4;
						if(SS2[2]>HH)
							abolish=1;
						if(abolish)
						{
							steady_flag=0;
							count=1;
							xbuf[0]=xbuf[3];
							ybuf[0]=ybuf[3];
							zbuf[0]=zbuf[3];
							buf_used[1]=0; buf_used[2]=0; buf_used[3]=0;
						}
						else
						{
							steady_flag=1;
							buf_used[3]=0;
							count=3;
							for(i=0;i<3;i++)
							{
								xbuf[i]=xbuf[i+1];
								ybuf[i]=ybuf[i+1];
								zbuf[i]=zbuf[i+1];
							}
						}
					}
					if(steady_flag)
					{
					S2=d(xyz00[0],avg[0])+d(xyz00[1],avg[1])+d(xyz00[2],avg[2]);
					if(absu(avg[0],xyz00[0])<H1 && absu(avg[1],xyz00[1])<H1 && absu(avg[2],xyz00[2])<H1)
					{
						G0=0;
						S0=0;
					}
					else
					Set_G_S();
					if(S0)
						exist=1;
					else
						exist=0;
					}
				}
				else if(set_flag && steady_flag)
				{
					ttemp[0]=(xbuf[0]+xbuf[1]+xbuf[2]+xbuf[3]);
					ttemp[1]=(ybuf[0]+ybuf[1]+ybuf[2]+ybuf[3]);
					ttemp[2]=(zbuf[0]+zbuf[1]+zbuf[2]+zbuf[3]);
					for(i=0;i<3;i++)
						aavg[i]=ttemp[i]/4;
					ttemp[0]=d(xbuf[0],aavg[0])+d(xbuf[1],aavg[0])+d(xbuf[2],aavg[0])+d(xbuf[3],aavg[0]);
					SS2[0]=ttemp[0]/4;
					if(SS2[0]>HH)
						abolish=1;

					ttemp[1]=d(ybuf[0],aavg[1])+d(ybuf[1],aavg[1])+d(ybuf[2],aavg[1])+d(ybuf[3],aavg[1]);
					SS2[1]=ttemp[1]/4;
					if(SS2[1]>HH)
						abolish=1;

					ttemp[2]=d(zbuf[0],aavg[2])+d(zbuf[1],aavg[2])+d(zbuf[2],aavg[2])+d(zbuf[3],aavg[2]);
					SS2[2]=ttemp[2]/4;
					if(SS2[2]>HH)
						abolish=1;
					if(abolish)
					{
						count=1;
						xbuf[0]=xbuf[3];
						ybuf[0]=ybuf[3];
						zbuf[0]=zbuf[3];
						buf_used[1]=0; buf_used[2]=0; buf_used[3]=0;
						steady_flag=0;
					}
					else
					{
						steady_flag=1;
						buf_used[3]=0;
						count=3;
						for(i=0;i<3;i++)
						{
							xbuf[i]=xbuf[i+1];
							ybuf[i]=ybuf[i+1];
							zbuf[i]=zbuf[i+1];
						}
					}
					if(steady_flag)
					{
					S2=d(xyz00[0],avg[0])+d(xyz00[1],avg[1])+d(xyz00[2],avg[2]);
					if(absu(avg[0],xyz00[0])<H1 && absu(avg[1],xyz00[1])<H1 && absu(avg[2],xyz00[2])<H1)
					{
						G0=0;
						S0=0;
					}
					else
					Set_G_S();
					if(S0)
						exist=1;
					else
						exist=0;
					}
				}



			if(set_flag && !steady_flag)
			{
				state=CHANGE;
				V_T=V_T5s;
				MEIWENDING_H;
			}

			else if(set_flag && steady_flag)
			{
				state=STEADY;
				V_T=V_T20s;
				WENDING_L;
				if(exist)   EXIST_H;
			else NON_EXIST_L;
			}

			else
			{
				state=INITIAL;
				V_T=V_T1s;
				MEIWENDING_H;
			}

			//if(exist)   EXIST_H;
			//else NON_EXIST_L;

			
while((P2IN & BIT2))  goto INIT_num;



////////////////////////////////////////////////////////////////////////////////////////////
#ifdef normal
			fuhaoxyz[0]=0;
			fuhaoxyz[1]=0;
			fuhaoxyz[2]=0;
			if((xbuf[count-1]&0x00008000)>>15 ==1)
			{
			fuhaoxyz[0]=1;
			digxyz[0]=-xbuf[count-1];
			}
			else digxyz[0]=xbuf[count-1];
			if((ybuf[count-1]&0x00008000)>>15 ==1)
			{
			fuhaoxyz[1]=1;
			digxyz[1]=-ybuf[count-1];
			}
			else digxyz[1]=ybuf[count-1];
			if((zbuf[count-1]&0x00008000)>>15 ==1)
			{
			fuhaoxyz[2]=1;
			digxyz[2]=-zbuf[count-1];
			}
			else digxyz[2]=zbuf[count-1];

			if(count>3)
			{
				count=0;
				for(i=0;i<3;i++)
					buf_used[i]=0;
			}

            for(i=0;i<3;i++)
            {
			digi[0] = digxyz[i]/10000;
			digi[1] = (digxyz[i]-digi[0]*10000)/1000;
			digi[2] = (digxyz[i]-digi[0]*10000-digi[1]*1000)/100;
			digi[3] = (digxyz[i]-digi[0]*10000-digi[1]*1000-digi[2]*100)/10;
			digi[4] = (digxyz[i]-digi[0]*10000-digi[1]*1000-digi[2]*100-digi[3]*10);
			//閫氳繃uart涓插彛TX鍚慞C鍙戦€佸瓧绗?
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			if(fuhaoxyz[i]==1)
			{
				while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
					 UCA0TXBUF = '-';
			}
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[0]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[1]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[2]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[3]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[4]+0x30;
            }
            if(set_flag)
            {
            for(i=0;i<3;i++){
			if((xyz00[i]&0x8000)>>15 ==1)
			{
			fuhaoxyz0[i]=1;
			digxyz0[i]=-xyz00[i];
			}
			else digxyz0[i]=xyz00[i];  }

            for(i=0;i<3;i++)
            {
			digi[0] = digxyz0[i]/10000;
			digi[1] = (digxyz0[i]-digi[0]*10000)/1000;
			digi[2] = (digxyz0[i]-digi[0]*10000-digi[1]*1000)/100;
			digi[3] = (digxyz0[i]-digi[0]*10000-digi[1]*1000-digi[2]*100)/10;
			digi[4] = (digxyz0[i]-digi[0]*10000-digi[1]*1000-digi[2]*100-digi[3]*10);
			//閫氳繃uart涓插彛TX鍚慞C鍙戦€佸瓧绗?
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			if(fuhaoxyz0[i]==1)
			{
				while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
					 UCA0TXBUF = '-';
			}
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[0]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[1]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[2]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[3]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[4]+0x30;
            }
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = 'D';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ':';

			digi[0] = S2/10000;
			digi[1] = (S2-digi[0]*10000)/1000;
			digi[2] = (S2-digi[0]*10000-digi[1]*1000)/100;
			digi[3] = (S2-digi[0]*10000-digi[1]*1000-digi[2]*100)/10;
			digi[4] = (S2-digi[0]*10000-digi[1]*1000-digi[2]*100-digi[3]*10);
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[0]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[1]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[2]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[3]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[4]+0x30;

			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = 'G';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ':';

			digi[0] = G0/10000;
			digi[1] = (G0-digi[0]*10000)/1000;
			digi[2] = (G0-digi[0]*10000-digi[1]*1000)/100;
			digi[3] = (G0-digi[0]*10000-digi[1]*1000-digi[2]*100)/10;
			digi[4] = (G0-digi[0]*10000-digi[1]*1000-digi[2]*100-digi[3]*10);
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[0]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[1]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[2]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[3]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[4]+0x30;
				 
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = 'S';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ':';

			digi[0] = S0/10000;
			digi[1] = (S0-digi[0]*10000)/1000;
			digi[2] = (S0-digi[0]*10000-digi[1]*1000)/100;
			digi[3] = (S0-digi[0]*10000-digi[1]*1000-digi[2]*100)/10;
			digi[4] = (S0-digi[0]*10000-digi[1]*1000-digi[2]*100-digi[3]*10);
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[0]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[1]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[2]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[3]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[4]+0x30;

			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = exist+0x30;
            }
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = ' ';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = 0x30+state;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = 0x0D;
#endif normal
			LPM3;
			//while((P2IN & BIT4))   LPM3;
			//while(!(P2IN & BIT4))   LPM3;
		  }

		}}
}

