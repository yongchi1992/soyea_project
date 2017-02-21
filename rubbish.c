/*
 * rubbish.c
 *
 *  Created on: 2015-7-21
 *      Author: soyea-whw
 */





//  uart涓插彛RX鎺ユ敹鍒颁竴涓瓧绗?浜х敓涓柇澶勭悊
//#pragma vector=USCIAB0RX_VECTOR
//__interrupt void USCI0RX_ISR(void)
//{
//     switch (UCA0RXBUF)
//  			{
//  				case '1': led1_red=0;
//  						break;
//
//  				case '2': led1_red=1;
//  						break;
//
//  				case '3': led2_green=0;
//  						break;
//
//  				case '4': led2_green=1;
//  						break;
//  				default:
//  				         break;
//  			}
//}


//					while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//					     UCA0TXBUF = BUF[0];            // 鏁板瓧鎵€瀵瑰簲鐨凙SCII鐮?
//					while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//					     UCA0TXBUF = BUF[1];
//					while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//					     UCA0TXBUF = BUF[2];
//					while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//					     UCA0TXBUF = BUF[3];
//					while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//					     UCA0TXBUF = BUF[4];
//					while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
//					     UCA0TXBUF = BUF[5];




     //---------显示X轴
//void Angle_Deal()//return to Angle[]
//{
//    int x,y,z;
//    double angle;
//
//    x = BUF[0] << 8 | BUF[1]; //Combine MSB and LSB of X Data output register
//    z = BUF[2] << 8 | BUF[3]; //Combine MSB and LSB of Z Data output register
//    y = BUF[4] << 8 | BUF[5]; //Combine MSB and LSB of Y Data output register
//
//    angle *= 10;
//    conversion(angle);       //计算数据和显示
//}



  // conversion(angle);       //计算数据和显示


//BCSCTL2 |= DIVS_3;      //  SMCLK = DCO/8 = 1MHz
