#ifndef JMT18F003_REG_H
#define JMT18F003_REG_H

sfr P0          = 0x80;
sbit PA0        = P0^0;
sbit PA1        = P0^1;
sbit PA2        = P0^2;
sbit PA3        = P0^3;
sbit PA4        = P0^4;
sbit PA5        = P0^5;
sbit PA6        = P0^6;
sbit PA7        = P0^7;

sfr SP          = 0x81;
sfr DPL         = 0x82;
sfr DPH         = 0x83;
sfr SFRPAGE     = 0x84;
sfr SFRPGEN     = 0x85;
sfr MOVXCON     = 0x86;
//  8Fh
//  97h
sfr P1          = 0x90;
sbit PB0        = P1^0;
sbit PB1        = P1^1;
sbit PB2        = P1^2;
sbit PB3        = P1^3;
sbit PB4        = P1^4;
sbit PB5        = P1^5;
sbit PB6        = P1^6;
sbit PB7        = P1^7;
//  9Fh
//  A7h
sfr P2          = 0xA0;
sbit PC0        = P2^0;
//  AFh
sfr IEN0        = 0xA8;
//  B7h
sfr P3          = 0xB0;
//  BFh
sfr IEN1        = 0xB8; 
//  C7h
sfr IEN2        = 0xC0;  
//  CFh
sfr P4          = 0xC8;
//  D7h
sfr PSW         = 0xD0;
//  DFh
sfr IP0         = 0xD8;
//  E7h
sfr ACC         = 0xE0;
//  EFh
sfr IP1         = 0xE8; 
//  F7h
sfr B           = 0xF0;
// F8h - FFh
sfr IP2         = 0xF8;

///////////////////////////PAGE0//////////////////////////////////////
// 80h - 87h
// 88h - 8Fh
sfr TCON        = 0x88;
sfr TMOD        = 0x89;
sfr TL0         = 0x8A;
sfr TL1         = 0x8B;
sfr TH0         = 0x8C;
sfr TH1         = 0x8D;
sfr TPSC        = 0x8E;
//  97h
sfr ADCCON0    = 0x91; 
sfr ADCCON1    = 0x92; 
sfr ADCPGAC    = 0x93; 
sfr ADCETC     = 0x94; 
sfr ADCDLY     = 0x95; 
sfr ADCRL      = 0x96; 
sfr ADCRH      = 0x97; 
//  9Fh
sfr SCON       = 0x98;
sfr SBUF       = 0x99;
sfr SRELL      = 0x9A;
sfr SRELH      = 0x9B;
sfr SIRCON     = 0x9C;
sfr LINCON      = 0x9D;
//  A7h
//  AFh
//  B7h
//  BFh
//  C7h
//  CFh
sfr SPICON      = 0xC9;
sfr SPIIE       = 0xCA; 
sfr SPIIF       = 0xCB;
sfr SPIBR       = 0xCC; 
sfr SPIBUF      = 0xCD;
sfr SPISPC      = 0xCE;
sfr SPISTS      = 0xCF;
//  D7h
sfr T2CON       = 0xD1;
sfr RL2         = 0xD2;
sfr RH2         = 0xD3;
sfr TL2         = 0xD4;
sfr TH2         = 0xD5;
sfr T2PSC       = 0xD6;
//  DFh
//  E7h
sfr I2CCON      = 0xE1;
sfr I2CSADDRL   = 0xE2;
sfr I2CSADDRH   = 0xE3;
sfr I2COADDRL   = 0xE4;
sfr I2COADDRH   = 0xE5;
sfr I2CDIV      = 0xE6;
sfr I2CDUTYL    = 0xE7;  
//  EFh
sfr I2CDUTYH    = 0xE9;
sfr I2CHOLD     = 0xEA;
sfr I2CWBUF     = 0xEB;
sfr I2CRBUF     = 0xEC;
sfr I2CSTS      = 0xED;
sfr I2CISC      = 0xEE;
sfr I2CIEN      = 0xEF;  
//  F7h
//sfr S1CON       = 0xF1;
//sfr S1BUF       = 0xF2;
//sfr S1RELL      = 0xF3;
//sfr S1RELH      = 0xF4;
//sfr S1IRCON     = 0xF5;
//  FFh
sfr FLSC        = 0xF9; 
sfr FLSK        = 0xFA; 

///////////////////////////PAGE1//////////////////////////////////////


///////////////////////////PAGE2//////////////////////////////////////
sfr PA0CTRL     = 0x89; 
sfr PA1CTRL     = 0x8A; 
sfr PA2CTRL     = 0x8B; 
sfr PA3CTRL     = 0x8C; 
sfr PA4CTRL     = 0x8D; 
sfr PA5CTRL     = 0x8E; 
sfr PA6CTRL     = 0x8F; 
sfr PB0CTRL     = 0x91;
sfr PB1CTRL     = 0x92;
sfr PB2CTRL     = 0x93;
sfr PB3CTRL     = 0x94;
sfr PB4CTRL     = 0x95;
sfr PB5CTRL     = 0xBA;
sfr PB6CTRL     = 0xBB;
sfr PA7CTRL     = 0x99;
sfr PB7CTRL     = 0x9A;
sfr PADSRDR     = 0x9B;
sfr AIOEN0      = 0x9C;
sfr AIOEN1      = 0x9D;
sfr FPSC        = 0x9E;
sfr ADCOEN      = 0x9F;
sfr EXINTSEL0   = 0xA1; 
sfr EXINTSEL1   = 0xA2; 
sfr EXINTPE     = 0xA3; 
sfr EXINTNE     = 0xA4; 
sfr EXINTEN     = 0xA5; 
sfr EXINTCON    = 0xA6; 
sfr PC0CTRL     = 0xB9;


//////////////////////////////////////////////////////////////////////////////////////////////
// TCON 0x88
sbit TF1     = 0x8F;                   // Timer1 overflow flag
sbit TR1     = 0x8E;                   // Timer1 on/off control
sbit TF0     = 0x8D;                   // Timer0 overflow flag
sbit TR0     = 0x8C;                   // Timer0 on/off control

// S0CON 0xF8
sbit SM0    = 0x9F;                   // Serial mode control bit 0
sbit SM1    = 0x9E;                   // Serial mode control bit 1  
sbit SM2    = 0x9D;                   // Multiprocessor communication enable
sbit REN    = 0x9C;                   // Receive enable
sbit TB8    = 0x9B;                   // Transmit bit 8
sbit RB8    = 0x9A;                   // Receive bit 8
sbit TI     = 0x99;                   // Transmit interrupt flag
sbit RI     = 0x98;                   // Receive interrupt flag

//// S1CON 0xF8
//sbit SM01    = 0xFF;                   // Serial mode control bit 0
//sbit SM11    = 0xFE;                   // Serial mode control bit 1  
//sbit SM21    = 0xFD;                   // Multiprocessor communication enable
//sbit REN1    = 0xFC;                   // Receive enable
//sbit TB81    = 0xFB;                   // Transmit bit 8
//sbit RB81    = 0xFA;                   // Receive bit 8
//sbit TI1     = 0xF9;                   // Transmit interrupt flag
//sbit RI1     = 0xF8;                   // Receive interrupt flag

// IEN0 0xA8
sbit EA     = 0xAF;                   // Global interrupt enable
sbit INT8   = 0xAD;                   // External interrupt 8 enable
sbit ES     = 0xAC;                   // UART0 interrupt enable
sbit ET1    = 0xAB;                   // Timer1 interrupt enable
sbit INT1   = 0xAA;                   // External interrupt 1 enable
sbit ET0    = 0xA9;                   // Timer0 interrupt enable
sbit EXINT  = 0xA8;                   // External interrupt 0 enable

// IEN1 0xB8
sbit INT7   = 0xBD;                   // External interrupt 7 enable
sbit INT6   = 0xBC;                   // External interrupt 6 enable
sbit INT5   = 0xBB;                   // External interrupt 5 enable
sbit INT4   = 0xBA;                   // External interrupt 4 enable
sbit INT3   = 0xB9;                   // External interrupt 3 enable
sbit INT2   = 0xB8;                   // External interrupt 2 enable

// PSW 0xE8
sbit CY      = 0xEF;                   // Carry flag
sbit AC      = 0xEE;                   // Auxiliary carry flag
sbit F0      = 0xED;                   // User flag 0
sbit RS1     = 0xEC;                   // Register bank select 1
sbit RS0     = 0xEB;                   // Register bank select 0
sbit OV      = 0xEA;                   // Overflow flag
sbit F1      = 0xE9;                   // User flag 1
sbit P       = 0xE8;                   // Accumulator parity flag


//crpm
sfr CLK_EN0         = 0xE9;
sfr CLK_EN1         = 0xEA;
sfr PWM_CLK_CTRL    = 0xED;
sfr CMP_CLK_DIV     = 0xF1;
sfr GPIO_CLK_DIV    = 0xF2;
sfr SWRST0          = 0xCB;
sfr SWRST1          = 0xCC;
sfr GLOBALRST       = 0xCD;
sfr RSTFLAG         = 0xCE;
sfr TRMVAL72M       = 0xCF;
sfr TRMVAL32K       = 0xD1;
sfr CRMCTRL         = 0xD2;
sfr ADCTRIM         = 0xD3;
sfr TRIM_CNT_73ML   = 0xD4;
sfr TRIM_CNT_73MH   = 0xD5;
sfr OSCPD           = 0xD6;
sfr SYS_CLK_DIV     = 0xD7;
sfr BANDGAP_CTRL    = 0xD9;
sfr SYS_CLK_SEL     = 0xDA;
sfr ADC_CLK_DIV     = 0xDB;
sfr JTAGEN          = 0xDC;
sfr TRIMKEY         = 0xDD;
sfr BUZZER_CTRL0    = 0xDE;
sfr BUZZER_CTRL1    = 0xDF;

sfr  CLK_READY       = 0XE1;
sfr  OSC24_CTRL      = 0XE2;
sfr  OSC_STABL       = 0XE3;
sfr  OSC_STABH       = 0XE4;
sfr  FLTCKSEL        = 0XE5;

//WDT - PAGE0
sfr  WDTCKDIV        = 0xD9; 
sfr  WDTINT          = 0xDA;
sfr  WDTINTF         = 0xDB;
sfr  WDTLD           = 0xDC;
sfr  WDTST           = 0xDD;

//PMU - PAGE0
sfr  PMUCTR          = 0xA1; 
sfr  PMUPD           = 0xA2;
sfr  PMUDBG          = 0xA3;
sfr  DLYH73M         = 0xA4;
sfr  DLYL73M         = 0xA5;

//PWM2 - PAGE1
//sfr  PWMSUBM2     = 0xD1;
//sfr  PWMOFSL2     = 0xD2;
//sfr  PWMDBDY2     = 0xD3;
//sfr  PWMSEL2      = 0xD4; 
//sfr  PWMINTCTL2   = 0xD5;
//sfr  PWMINTSTA2   = 0xD6;
//sfr  PWMPERDL2    = 0xD7;
//sfr  PWMPERDH2    = 0xD9;
//sfr  PWMCCNTL2    = 0xDA;
//sfr  PWMCCNTH2    = 0xDB;
//sfr  PWMRCR2      = 0xDC;
//
////PWM1 - PAGE1
//sfr  PWMSUBM1     = 0xC1;
//sfr  PWMOFSL1     = 0xC2;
//sfr  PWMDBDY1     = 0xC3;
//sfr  PWMSEL1      = 0xC4; 
//sfr  PWMINTCTL1   = 0xC5;
//sfr  PWMINTSTA1   = 0xC6;
//sfr  PWMPERDL1    = 0xC7;
//sfr  PWMPERDH1    = 0xC9;
//sfr  PWMCCNTL1    = 0xCA;
//sfr  PWMCCNTH1    = 0xCB;
//sfr  PWMRCR1      = 0xCC;
//   
////PWM0 - PAGE1
//sfr  PWMSUBM0     = 0xB1;
//sfr  PWMOFSL0     = 0xB2;
//sfr  PWMDBDY0     = 0xB3;
//sfr  PWMSEL0      = 0xB4; 
//sfr  PWMINTCTL0   = 0xB5;
//sfr  PWMINTSTA0   = 0xB6;
//sfr  PWMPERDL0    = 0xB7;
//sfr  PWMPERDH0    = 0xB9;
//sfr  PWMCCNTL0    = 0xBA;
//sfr  PWMCCNTH0    = 0xBB;
//sfr  PWMRCR0      = 0xBC;
//sfr  PWMOVLD0     = 0xBD;
//sfr  PWMOEXG0     = 0xBE;
//sfr  PWMOUP0      = 0xBF;


sfr TIM0_CONR0   = 0xD1;
sfr TIM0_CONR1   = 0xD2;
sfr TIM0_CONR2   = 0xD3;
sfr TIM0_CONR3   = 0xD4;
sfr TIM0_TGICR0  = 0xD5;
sfr TIM0_TGICR1  = 0xD6;
sfr TIM0_IER0    = 0xD7;
sfr TIM0_IER1    = 0xFB;

sfr TIM0_CC0MR   = 0xD9;
sfr TIM0_CC1MR   = 0xDA;
sfr TIM0_CC2MR   = 0xDB;
sfr TIM0_CC3MR   = 0xDC;
sfr TIM0_CCENR   = 0xDD;
sfr TIM0_CCPS    = 0xDE;
sfr TIM0_DTC     = 0xDF;

sfr TIM0_ARRL    = 0xE1;
sfr TIM0_ARRH    = 0xE2;
sfr TIM0_PSCL    = 0xE3;
sfr TIM0_PSCH    = 0xE4;
sfr TIM0_RCR     = 0xE5;
sfr TIM0_CC0RL  = 0xE6;
sfr TIM0_CC0RH  = 0xE7;
sfr TIM0_CC1RL  = 0xE9;
sfr TIM0_CC1RH  = 0xEA;
sfr TIM0_CC2RL  = 0xEB;
sfr TIM0_CC2RH  = 0xEC;
sfr TIM0_CC3RL  = 0xED;
sfr TIM0_CC3RH  = 0xEE;
sfr TIM0_BRKC    = 0xEF;

sfr TIM0_EGR     = 0xF1;
sfr TIM0_PHACON  = 0xF2;
sfr TIM0_ISR0    = 0xF3;
sfr TIM0_ISR1    = 0xF4;
sfr TIM0_CNTL    = 0xF5;
sfr TIM0_CNTH    = 0xF6;
sfr TIM0_PHASEL  = 0xF7;

sfr TIM0_PHASEH  = 0xF9;
sfr TIM0_CNTEN   = 0xFA;


sfr TIM1_CONR0   = 0xA1;
sfr TIM1_CONR1   = 0xA3;
sfr TIM1_CONR2   = 0xA4;
sfr TIM1_TGICR0  = 0xA5;
sfr TIM1_IER0    = 0xA7;
sfr TIM1_IER1    = 0xCB;

sfr TIM1_CC0MR   = 0xA9;
sfr TIM1_CC1MR   = 0xAA;
sfr TIM1_CC2MR   = 0xAB;

sfr TIM1_CCENR   = 0xAD;
sfr TIM1_CCPS    = 0xAE;
sfr TIM1_DTC     = 0xAF;

sfr TIM1_ARRL    = 0xB1;
sfr TIM1_ARRH    = 0xB2;
sfr TIM1_PSCL    = 0xB3;
sfr TIM1_PSCH    = 0xB4;
sfr TIM1_RCR     = 0xB5;
sfr TIM1_CC0RL  = 0xB6;
sfr TIM1_CC0RH  = 0xB7;
sfr TIM1_CC1RL  = 0xB9;
sfr TIM1_CC1RH  = 0xBA;
sfr TIM1_CC2RL  = 0xBB;
sfr TIM1_CC2RH  = 0xBC;
sfr TIM1_BRKC    = 0xBF;
sfr TIM1_EGR     = 0xC1;
sfr TIM1_ISR0    = 0XC3;
sfr TIM1_ISR1    = 0XC4;
sfr TIM1_CNTEN   = 0xCA;

//PAGE CONTROL - PAGE2
sfr  PGSTACK0     = 0xF9;
sfr  PGSTACK1     = 0xFA;
sfr  PGSTACK2     = 0xFB;
sfr  PGSTACK3     = 0xFC;

//CMP - PAGE0
sfr  CMPEN        = 0xB9;
sfr  CMPFT        = 0xBA;
sfr  CMPVDD       = 0xBB;
sfr  CMPREF       = 0xBC;
sfr  CMPINT       = 0xBD;
sfr  CMPFLAG      = 0xBE;
sfr  OCPRSEL      = 0xC1;
sfr  ODPRSEL      = 0xC2;
sfr  BKPRSEL      = 0xC3;


#endif
