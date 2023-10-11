# UART LAB
## Objectives
Program your microontroller to transmit:<br>
"F0"-if SW1 is pressed<br>
"AA"-if SW2 is pressed <br>
Over UART with baud rate 9600 and odd parity. <br>
Your program should also listen for incoming data on the UART with the same baud and parity config; if "AA" is received LED should be GREEN; if "F0" is recieved, the LED should be BLUE and if any error is detected LED should be RED. Test this by communicating with your neighboring group.<br>
   
## Theory
### What is UART ?
UART, which stands for Universal Asynchronous Receiver/Transmitter, is a communication protocol widely used for serial communication between devices. It enables the transfer of data between devices using two wires: one for transmitting (TX) and the other for receiving (RX). UART is asynchronous, meaning that data is transmitted without a shared clock signal, and devices synchronize based on agreed-upon parameters such as baud rate, data bits, parity, and stop bits.

Key points about UART:

#### Asynchronous Communication: 
UART does not rely on a shared clock signal between devices. Instead, both the transmitting and receiving devices agree on communication parameters.

#### Basic Components:<br>
UART communication involves two main components—a transmitter and a receiver. The transmitter converts parallel data into serial form for transmission, and the receiver converts the received serial data back into parallel form.

#### Baud Rate: 
Baud rate defines the speed at which data is transmitted and received. Both communicating devices must operate at the same baud rate for successful communication.

#### Data Framing: 
Each piece of data is framed by start and stop bits, indicating the beginning and end of a data byte. This framing helps the receiver identify and extract the transmitted data.

#### Configurable Parameters: 
UART communication parameters include data bits (usually 8 bits per byte), parity (optional error-checking mechanism), and stop bits (typically 1 or 2 bits).

#### Half-Duplex or Full-Duplex: 
UART supports both half-duplex and full-duplex communication. In half-duplex mode, devices take turns transmitting and receiving, while in full-duplex mode, they can simultaneously transmit and receive.

#### Widespread Usage: 
UART is a fundamental communication protocol and is commonly used in various applications, including serial communication between microcontrollers, sensors, and peripheral devices.

### Calculation
 We calculate Baudrate Register parameters using 9600.
 We get integer and fractional part for baud rate speciification to register by  given expression<br>
   > 16000000/(16*9600)=104.1666 <br>
 
 Integer part taken as 104 and we calculated fractional part using below expression
  > 0.167*64+0.5=11
####  Register used:
  UART DR,UARTFR,UART_IBRD,UART_FBRD,UARTLCRH,UARTCTL,UARTIM,UARTCC.(mainly)
  
## Code
```C
/*
 * UART with baud rate 9600 and odd parity.
 * F0- if SW1 is pressed
 * AA if SW2 is pressed
 * if "AA" is received LED should be GREEN;
 * if "F0" is recieved, the LED should be BLUE
 * if any error is detected LED should be RED.
 *
 * Using PC4, PC5; UART Module 4
 * PC4 = Rx; PC5 = Tx
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define Sw_Bits 0x11
#define S1_data 0xF0
#define S2_data 0xAA

#define Red 0X02
#define Blue 0X04
#define Green 0X08

void PortF_Config(void);
void UART_Config(void);
void PortF_Handler(void);
void UART_Handler(void);
void Systick_Handler(void);

#define STCTRL *((volatile long *) 0xE000E010)      //Control and Status Register
#define STRELOAD *((volatile long *) 0xE000E014)   //SysTick Reload Value Register
#define STCURRENT *((volatile long *) 0xE000E018)  //SysTick Current Value Register

// configurations for  Systick CSR(Control and Status Register)
#define ENABLE (1<<0)       //bit 0 of CSR enables systick counter
#define INT_EN (1<<1)       //bit 1 of CSR to generate interrupt to the NVIC when SysTick counts to 0
#define Clk_SRC (1<<2)      //bit 2 of CSR to select system clock
#define COUNT_FLAG (1<<16)  //bit 16 of CSR

void main(void)
{
    SYSCTL_RCGCGPIO_R |= (1<<5);        //Enable and provide a clock to GPIO PortF
    SYSCTL_RCGCUART_R |= (1<<4);      //Enable and provide a clock to UART module4  in Run mode
    SYSCTL_RCGCGPIO_R |= (1<<2);      //Enable and provide a clock to GPIO Port C

    UART_Config();
    PortF_Config();

    while(1){}
}

void PortF_Config(void)
{
    GPIO_PORTF_LOCK_R = 0x4C4F434B;     //Unlock PortF register
    GPIO_PORTF_CR_R = 0x1F;             //Enable  function

    GPIO_PORTF_PUR_R = 0x11;            //Pull-up for user switches
    GPIO_PORTF_DEN_R = 0x1F;            //Enable all pins on port F
    GPIO_PORTF_DIR_R = 0x0E;            //Defining PortF LEDs as output and switches as input

    //PortF Interrupt Configurations: User Sw should trigger hardware interrupt
    GPIO_PORTF_IS_R &= ~Sw_Bits;        //Edge trigger detected
    GPIO_PORTF_IBE_R &= ~Sw_Bits;       //Trigger interrupt according to GPIOIEV
    GPIO_PORTF_IEV_R &= ~Sw_Bits;       //Trigger interrupt on falling edge
    GPIO_PORTF_IM_R &= ~Sw_Bits;        //Mask interrupt bits
    GPIO_PORTF_ICR_R |= Sw_Bits;        //clear any prior interrupts
    GPIO_PORTF_IM_R |= Sw_Bits;         //enable interrupts for bits corresponding to Mask_Bits

    //NVIC Configuration
    //PortF interrupts correspond to interrupt 30 (EN0 and PRI7 registers)
    NVIC_EN0_R |= (1<<30);              //Interrupts enabled for port F
    NVIC_PRI7_R &= 0xFF3FFFFF;          //Interrupt Priority 1 to Port F
}

void UART_Config(void)
{
    /*
    *BRDI = integer part of the BRD; BRDF = fractional part
    *BRD = BRDI + BRDF = UARTSysClk / (ClkDiv * Baud Rate)*UARTSysClk = 16MHz, ClkDiv = 16, Baud Rate = 9600 *BRD = 104.167; BRDI = 104; BRDF = 167; *UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5) = 11
    */
    /*UARTCTL should  not be changed while UART is enabled so we have to disable UART*/
    UART4_CTL_R &= (0<<0);                       //Disable UART module 4
    UART4_IBRD_R = 104;
    UART4_FBRD_R = 11;
    UART4_CC_R = 0x00;                          //System Clock
    UART4_LCRH_R = 0x62;                        //8 bit word length, FIFO enable, Parity Enable
    UART4_CTL_R |= ((1<<0)|(1<<8)|(1<<9));      //Enable UART module 4

    //UART interrupt configuration
    UART4_IM_R &= ((0<<4)|(0<<5)|(0<<8));       //Mask Tx, Rx and Parity interrupts
    UART4_ICR_R &= ((0<<4)|(0<<5)|(0<<8));      //Clear Tx, Rx and Parity interrupts
    UART4_IM_R |= (1<<4);                       //Enable Rx interrupt
    NVIC_EN1_R |= (1<<28);                      //Interrupts enabled for UART4
    NVIC_PRI15_R &= 0xFFFF5FFF;                 //Interrupt Priority 2 to UART4

    GPIO_PORTC_LOCK_R = 0x4C4F434B;     //Unlock PortE register
    GPIO_PORTC_CR_R = 0xFF;             //Enable Commit function
    GPIO_PORTC_DEN_R = 0xFF;            //Enable all pins on port C
    GPIO_PORTC_DIR_R |= (1<<4);         //Define PC5 as output(Tx)
    GPIO_PORTC_AFSEL_R |= 0x30;         //Enable Alternate function for PC4 and PC5
    //GPIO_PORTC_AMSEL_R = 0;
    GPIO_PORTC_PCTL_R |= 0x00110000;    //Selecting UART function for PD6 and P7
}

void PortF_Handler()
{
    GPIO_PORTF_IM_R &= ~Sw_Bits;

    if(GPIO_PORTF_RIS_R & 0x10)         //Usr Sw 1
    {
        UART4_DR_R = 0xF0;
    }
    else if (GPIO_PORTF_RIS_R & 0x01)   //Usr Sw 2
    {
        UART4_DR_R = 0xAA;
    }
}

void UART_Handler(void)
{
    UART4_IM_R &= (0<<4);       //Mask UART Rx interrupt

    if(UART4_FR_R & (1<<6))    //Rx flag register set (data recieved)
    {
        if(UART4_DR_R == 0xAA)
        {
            GPIO_PORTF_DATA_R = Green;
        }
        else if(UART4_DR_R == 0xF0)
        {
            GPIO_PORTF_DATA_R = Blue;
        }
    }

    if(UART4_RSR_R & 0x0000000F)    //Any error detected
    {
        GPIO_PORTF_DATA_R = Red;
    }

    UART4_ECR_R &= 0xFFFFFFF0;        //Clear UART errors

    STCURRENT=0x00;                         //Reinitialise Systick Counter to Zero
    STRELOAD = 16*1000000/2;                //Run Systick for 0.5 second
    STCTRL |= (ENABLE | INT_EN | Clk_SRC);  //Enable Systick, Enable Interrupt Generation, Enable system clock (80MHz) as source

    GPIO_PORTF_ICR_R = Sw_Bits;

}

void Systick_Handler(void)
{
    GPIO_PORTF_DATA_R &= 0x00;               //Turn off LED
          //mask, clear and unmask gpio interrupt
    GPIO_PORTF_ICR_R = Sw_Bits;
    GPIO_PORTF_IM_R |= Sw_Bits;
    UART4_IM_R |= (1<<4);                   //UnMask UART Rx interrupt
}

```

## Observation
*UARTCTL  register  should  not  change  while  UART  is  enabled .
so  there  is  a  sequence  to  change  UARTCTL.



## References
tm4c123gh6pm data sheet.
