// CranePID_v2.c main code for Gantry Crane PID Control
// ENGPHYS 3BB3 Final Design Project
// Jarod Coppens, Josh Eizenga - April 2019

//-----------------------------------------------------------------
//  Read Me
//-----------------------------------------------------------------
//  Use under the following conditions:
//  MSP430 is configured for 16MHz clock.
//  UART is set for 115200 baud, using the x16 clock mode
//  UART interrupt for receiving data from MATLAB (updates the setpoint and PID constants)
//  PWM output is on P2.1
//  H-Bridge IN1 and IN2 connections are on P2.1 (PWM) and P1.0
//  H-Bridge ENA held at +5V
//  Inputs from motor's optical encoders are on P1.6 (A) and P1.7 (B)
//  Port1 interrupt triggered by optical encoder on P1.6
//  Angle optical sensor input at P1.4 (ADC input)
//-----------------------------------------------------------------

#include "io430.h"    
#include <stdlib.h>
#include <math.h>

#define IN2             P1OUT_bit.P0
#define motorOutputA    P1IN_bit.P6 //formerly P1.3
#define motorOutputB    P1IN_bit.P7 //formerly P1.5
#define angleOutput     P1IN_bit.P4
#define maxPWM          300         //limit to max motor speed
#define posLimit        1700
#define negLimit        -200
#define zeroAngle       53

//-----------------------------------------------
// GLOBAL VARIABLES
//-----------------------------------------------

int position = 0, i = 0;
int KPX = 8,  KDX = 0, KIX = 0;
int KPA = 0, KDA = 0; 
int targetX = 100;   //target position (updated by MATLAB input)

//-----------------------------------------------
// Initialization + Misc Functions
//-----------------------------------------------
void Init(void){
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD; 

  // set system clock to calibrated values
  DCOCTL  = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;
  
  P1DIR |= BIT0; //set P1.0 to output (for H-bridge IN2)
  P2DIR |= BIT1; //set P2.1 to output direction (for PWM/motor IN1)
  
  P1IE = BIT6; //enable interrupts on P1.3 input
  __enable_interrupt();
}

void delay(unsigned long int n){
  while(--n);
}

//-----------------------------------------------
// UART Module + Functions
//-----------------------------------------------

void Init_UART(void){  // initialize the USCI
  // RXD is on P1.1
  // TXD is on P1.2
  // configure P1.2 and P1.2 for secondary peripheral function
  P1SEL = P1|P2;
  P1SEL2 = P1|P2;

  // divide by 9 for 115200b with 16MHz clock
  UCA0BR1 = 0;
  UCA0BR0 = 9;  
  
  // use x16 clock
  UCA0MCTL_bit.UCOS16 = 1;

  // select UART clock source
  UCA0CTL1_bit.UCSSEL1 = 1;
  UCA0CTL1_bit.UCSSEL0 = 0;

  UCA0CTL1_bit.UCSWRST = 0; // release UART RESET
    
  UC0IE |= UCA0RXIE; // Enable USCI_A0 RX interrupt  
}

//sends a character
void putc(unsigned char c){
  while (!IFG2_bit.UCA0TXIFG);
  UCA0TXBUF = c;
}

//receives a character
char getc(void){
  while (!IFG2_bit.UCA0RXIFG);
  return UCA0RXBUF;
}

//-----------------------------------------------
// PWM SETUP
//-----------------------------------------------

void Init_PWM(void){
  //configure P2.1 to output TA1.1 (secondary peripheral function)
  P2SEL_bit.P1 = 1;
  P2SEL2_bit.P1 = 0;
  
  TA1CTL = TASSEL_2 + MC_1; //select SMCLK, up mode
  
  TA1CCTL1 = OUTMOD_7; //Output mode: Reset/Set
  
  //pulse width 1000-500us = 500us (with 1MHz clock)
  //64us with 16MHz clock
  TA1CCR0 = 999; //total period = TA1CCR0 + 1
  TA1CCR1 = 0;   //period in microseconds that power is ON
  IN2 = 0;
}

//-----------------------------------------------
// ADC Module + Functions
//-----------------------------------------------

void Init_ADC(void){
  //initialize 10-bit ADC using input channel 4 on P1.4 
  // use Mode 2 - Repeat single channel 
  ADC10CTL1 = INCH_4 + CONSEQ_2; // use P1.4 (channel 4) 
  ADC10AE0 |= BIT4;   // enable analog input channel 4 
  
  //select sample-hold time, multisample conversion and turn on the ADC
  ADC10CTL0 |= ADC10SHT_0 + MSC + ADC10ON; 
  
  // start ADC 
  ADC10CTL0 |= ADC10SC + ENC; 
}

//-----------------------------------------------
// Motor Control
//-----------------------------------------------

// stop motor motion
void motorStop(void){
  TA1CCR1 = 0;
  IN2 = 0;
}

// move the motor at certain speed
// direction determined by sign of speed
void moveTo(int speed){
  if(speed>0){
    TA1CCR1 = speed;
    IN2 = 0;
  }
  else if(speed<0){
    TA1CCR1 = 1000 + speed;
    IN2 = 1;
  }
}
//-----------------------------------------------
// INTERRUPTS
//-----------------------------------------------

// P1.3 interrupt to count motor turns
// interrupt triggers on high->low from outputA on P1.6 (so outputA always low)
// if outputB also low, motion in FWD direction
// if outputB high, motion in REV direction
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void){
  
  if(motorOutputB == 0) ++position;
  else                  --position;
  
  //boundary conditions
  if(position > posLimit || position < negLimit) {    
    motorStop();
    exit(1); //end entire program
  }  
  P1IFG_bit.P6 = 0; // clear the interrupt request flag
}

//Interrupt to receive data from MATLAB 
//updates the PID constants and target position
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void){
  if(getc() == 'A'){ 
    KPX = getc();
    KIX = getc();
    KDX = getc();
    KPA = getc();
    KDA = getc();
    targetX = getc()*100;
  }  
}

//-----------------------------------------------
// MAIN CODE
//-----------------------------------------------
void main(){  
  Init();
  Init_UART();
  Init_ADC();
  Init_PWM();  
  
  int X, prevX = 0;     //current & previous position                    
  int positionError = 0;//targetPosition - currentPosition
  int derivativeX;      
  int integralX = 0;
  int angle;        
  int prevAngle = zeroAngle;
  int angleError = 0;
  float derivativeA;  
  int output;

  while(1){    
    X = position;
    angle = ADC10MEM >> 2; 
    
    positionError = (targetX - X);
    derivativeX = X - prevX;
    integralX = integralX + positionError;     
    angleError = (zeroAngle - angle);
    derivativeA = 0.3*derivativeA + 0.7*(angle - prevAngle); //running average for angle derivative      
    output = positionError*KPX + derivativeX*KDX + integralX*KIX - angleError*KPA - (int)derivativeA*KDA;
    
    //statements below provide a clamping limit to the PID output so it doesn't output past the value of maxPWM
    if(output > maxPWM){
      output = maxPWM;
      integralX = 0;
    }
    else if(output < -maxPWM){
      output = -maxPWM;
      integralX = 0;
    }    
    moveTo(output);
    
    prevAngle = angle;
    prevX = X;
  
    putc('A'); 
    putc(X >> 8);
    putc(X);    
    putc(angle);           
   }
 }
