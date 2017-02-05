#include  <msp430g2553.h>

#define V_T1s	200

unsigned char clock1s=0;
unsigned char clock1s_flag=0;
unsigned char led1_red,led2_green;


void port_init(void)
{
	    P1DIR |= BIT0+BIT6;      
}

//TIMER0 initialize -
// desired value: 5ms
void timer0_init(void)
{
	// Configure Timer0
		TA0CTL = TASSEL_2 + MC_1 ;          // SMCLK=1MHz, UP mode
		TA0CCR0 = 5000;                 
		CCTL0 = CCIE;                  	//CCR0 interrupt enabled
}

// Timer0_A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void)
{
	
		if (++clock1s>=V_T1s)
		{
			clock1s_flag = 1; 
			clock1s = 0;
	     }

		if (led1_red==1)
		    P1OUT &= ~ BIT0; 
		else
			P1OUT |=  BIT0;  

		if (led2_green==1)
		    P1OUT &= ~ BIT6;  
		else
			P1OUT |=  BIT6;   
}


void uart_init(void)
{
	P1SEL |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	P1SEL2 |= BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	UCA0CTL1 |= UCSSEL_2;                     // uart SMCLK=1MHz
	UCA0BR0 = 104;                            
	UCA0BR1 = 0;                              
	UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST;             // **Initialize USCI state machine**
	IE2 |= UCA0RXIE;                      // Enable USCI_A0 RX interrupt
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
     switch (UCA0RXBUF)
  			{
  				case '1': led1_red=0;
  						break;

  				case '2': led1_red=1;
  						break;

  				case '3': led2_green=0;
  						break;

  				case '4': led2_green=1;
  						break;
  				default:
  				         break;
  			}
}


void init_devices(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

		if (CALBC1_8MHZ ==0xFF || CALDCO_8MHZ == 0xFF)
		{
			while(1);            	// If calibration constants erased, trap CPU!!
		}

		BCSCTL1 = CALBC1_8MHZ; 		    // Set range
		DCOCTL = CALDCO_8MHZ;  		   
		BCSCTL3 |= LFXT1S_2;                      // LFXT1 = VLO
		IFG1 &= ~OFIFG;                           // Clear OSCFault flag
		BCSCTL2 |= DIVS_3;      //  SMCLK = DCO/8 = 1MHz

 port_init();
 timer0_init();
 uart_init();
 _BIS_SR(GIE); 
 //all peripherals are now initialized
}


void main(void)
{

	init_devices( );

	 while(1)
	  {
		if (clock1s_flag==1)   		  {
			clock1s_flag=0;
	       	if (++test_counter>=10000) test_counter=0;
			digi[0] = test_counter/1000;  	
			digi[1] = (test_counter-digi[0]*1000)/100; 
			digi[2] = (test_counter-digi[0]*1000-digi[1]*100)/10; 	
			digi[3] = (test_counter-digi[0]*1000-digi[1]*100-digi[2]*10); 

			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
				 UCA0TXBUF = 0x0D;                    
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[0]+0x30;           
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[1]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[2]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = digi[3]+0x30;
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = ' ';
			while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
			     UCA0TXBUF = 'S';

		  }
	  }

}
