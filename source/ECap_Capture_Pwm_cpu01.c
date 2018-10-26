//###########################################################################
// FILE:    ECap_Capture_Pwm_cpu01.c
// TITLE:   Capture EPWM3.
//
//! \addtogroup cpu01_example_list
//! <h1>ECAP Capture PWM Example</h1>
//!
//! This example configures ePWM3A for:
//! - Up count
//! - Period starts at 2 and goes up to 1000
//! - Toggle output on PRD
//!
//! eCAP1 is configured to capture the time between rising
//! and falling edge of the ePWM3A output.
//!
//! \b External \b Connections \n
//! - eCAP1 is on GPIO19
//! - ePWM3A is on GPIO4
//! - Connect GPIO4 to GPIO19.
//!
//! \b Watch \b Variables \n
//! - \b ECap1PassCount - Successful captures
//! - \b ECap1IntCount - Interrupt counts
//
//###########################################################################
// $TI Release: F2837xS Support Library v180 $
// $Release Date: Fri Nov  6 16:27:58 CST 2015 $
// $Copyright: Copyright (C) 2014-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F28x_Project.h"     // Device Headerfile and Examples Include File


// Prototype statements for functions found within this file.
__interrupt void ecap1_isr(void);
//__interrupt void ecap2_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void xint1_isr(void);
__interrupt void xint2_isr(void);

void InitECapture(void);
void Fail(void);
void EPwmSetup(void);


// Global variables used in this example
Uint32  ECap1IntCount;
Uint32  ECap1PassCount;
Uint32  EPwm3TimerDirection;
Uint32  T1,T2,T3,T4;
Uint32  j;
Uint32  count_tulun;
Uint32  c0, c1;
bool    Cycle_flag;
float32  T1_2,T2_2,T3_2,T4_2;
float32  Speed,phase;
volatile Uint32 Xint1Count;
Uint32   time,temp;
Uint32   Quzhou_chishu,Tulun_chishu;
bool    flag_t0,flag_t1;


bool  flag_phase; // 第一个方波上升沿到达标志
//float32 phase_lib[12]= {5/72, 10/72,15/72,21/72,26/72,31/72,41/72,46/72,51/72,57/72,62/72,67/72};
float32 tu_phase[12]= {0.069444, 0.138888,0.208333,0.291666,0.361111,0.430555,0.569444,0.638888,0.708333,0.791666,0.861111,0.930555};
float32 *phase_lib;
// To keep track of which way the timer value is moving
#define EPWM_TIMER_UP   1
#define EPWM_TIMER_DOWN 0


void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xS_SysCtrl.c file.


	InitSysCtrl();

// Step 2. Initialize GPIO: 
// This example function is found in the F2837xS_Gpio.c file and
// illustrates how to set the GPIO to its default state.
// InitGpio();  // Skipped for this example  

   InitEPwm3Gpio();
   InitEPwm2Gpio();
   InitECap1Gpio(19);

   GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_ASYNC);
   //GPIO_SetupPinOptions(13, GPIO_INPUT, GPIO_ASYNC);

   GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 0);  //hep
   GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_PUSHPULL);//hep

   GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 0);  //hep
   GPIO_SetupPinOptions(11, GPIO_OUTPUT, GPIO_PUSHPULL);//hep

   GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 0);  //hep
   GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_PUSHPULL);//hep

   GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0);  //hep
   GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);//hep

   //GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);  //hep
   //GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_OPENDRAIN);//hep

   //GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 0);  //hep
   //GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_OPENDRAIN);//hep

   // GPIO13/15 are input
      EALLOW;
      GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;         // GPIO
      GpioCtrlRegs.GPADIR.bit.GPIO15 = 0;          // input
      GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 0;        // XINT1 Synch to SYSCLKOUT only
      EDIS;

      /*
      EALLOW;
      GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;         // GPIO
      GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;          // input
      GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 0;        // XINT2 Synch to SYSCLKOUT only
      EDIS;
      */

      // GPIO15 is XINT1, GPIO13 is XINT2
         GPIO_SetupXINT1Gpio(15);//K1  GPIO15
         GPIO_SetupXINT2Gpio(13);//K2  GPIO13

// Step 3. Clear all __interrupts and initialize PIE vector table:
// Disable CPU __interrupts 
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.  
// This function is found in the F2837xS_PieCtrl.c file.
   InitPieCtrl();
   
// Disable CPU __interrupts and clear all CPU __interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xS_DefaultIsr.c.
// This function is found in F2837xS_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.ECAP1_INT = &ecap1_isr;
   PieVectTable.XINT1_INT = &xint1_isr;
   PieVectTable.XINT2_INT = &xint2_isr;
   PieVectTable.TIMER0_INT = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripherals:

   InitECapture();
   InitCpuTimers();   // For this example, only initialize the Cpu Timers

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
   ConfigCpuTimer(&CpuTimer0, 200, 500000);

// Step 5. User specific code, enable __interrupts:

// Initialize counters:   
   ECap1IntCount = 0;
   ECap1PassCount = 0;
   

   j=0;
   c0=0;
   c1=0;
   T1_2=0;
   T2_2=0;
   Cycle_flag=false;
   Speed=0.0;
   time=0;
   temp=1;
   phase=0.0;
   count_tulun=0;
   Quzhou_chishu = 33;
   Tulun_chishu = 6;
   flag_t0 =0;
   flag_t1=0;

   flag_phase = 0;
   phase_lib = tu_phase;


   // Configure XINT1/2
      XintRegs.XINT1CR.bit.POLARITY = 3;      // Rising edge interrupt  凸轮轴任意沿
      XintRegs.XINT2CR.bit.POLARITY = 1;      // both edges interrupt   方波上升沿

   // Enable XINT1 and XINT2
      XintRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1
      XintRegs.XINT2CR.bit.ENABLE = 1;        // Enable XINT2

// Enable CPU INT4 which is connected to ECAP1-4 INT:
   // Enable CPU INT1 which is connected to xINT1/2, Timer:  hep
   IER |= M_INT4;
   IER |= M_INT1;

// Enable eCAP INTn in the PIE: Group 3 __interrupt 1-6
   PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4  XINT1
   PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5  XINT2
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;          //timer  hep

   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global __interrupt INTM
   ERTM;   // Enable Global realtime __interrupt DBGM

   DELAY_US(1000);
   EPwmSetup();
   //GPIO_WritePin(10, 1);

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)
   {
   //   asm("          NOP");
      /*hep*/
   //GPIO_WritePin(10, 1);DELAY_US(20);
   //GPIO_WritePin(10, 0);DELAY_US(20);

	//GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1;
  	for(j=0;j<17;j++)
  	{
  		GPIO_WritePin(10, 1);DELAY_US(198);
  		GPIO_WritePin(10, 0);DELAY_US(792);
  	}
  	DELAY_US(990);
  	/*
  	for(j=0;j<3;j++)
  	{
  		GPIO_WritePin(12, 1);DELAY_US(10);
  		GPIO_WritePin(12, 0);DELAY_US(10);
  	}
    */
  	for(j=0;j<16;j++)
  	{
  		GPIO_WritePin(10, 1);DELAY_US(198);
  		GPIO_WritePin(10, 0);DELAY_US(792);
  	}

  	DELAY_US(1980);

		/*hep*/

   }
} 


void InitECapture()
{
   ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture __interrupts
   ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP __interrupt flags
   ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;      // One-shot
   ECap1Regs.ECCTL2.bit.STOP_WRAP = 0;        // Stop at 1 events
   ECap1Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP2POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP4POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation         
   ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;         // Enable sync in
   ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;        // Pass through
   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units

   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;        // Start Counter
   ECap1Regs.ECCTL2.bit.REARM = 1;            // arm one-shot
   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable CAP1-CAP4 register loads
   ECap1Regs.ECEINT.bit.CEVT1 = 1;            // 4 events = __interrupt

}

__interrupt void ecap1_isr(void)
{
   //ECap runs twice as fast on F2837xS so multiplier
   //was increased from x2 to x4 and the fudge factor was
   //increased from 1 to 4. 

   // Cap input is syc'ed to SYSCLKOUT so there may be
   // a +/- 1 cycle variation
	c0=c1;

	T1=ECap1Regs.CAP1;
	c1=T1;

	 if(((2.5*c0)<c1)&&(c1<(3.5*c0)))
	    {
	    	flag_t0 = 1;
	    }

	if(((0.8*c0)<c1)&&(c1<(1.2*c0)))
		Speed = 200000000/(c1*36)*60;
	else if(((1.8*c0)<c1)&&(c1<(2.2*c0)))
		Speed = 200000000/(c1/2*36)*60;
	else if((((2.3*c0)<c1)&&(c1<(3.7*c0)))||ECap1IntCount==33)
 	   {
		GpioDataRegs.GPATOGGLE.bit.GPIO11 = 1;
		ECap1IntCount=0;
 	   }

    if(flag_t0==1)
    {
    	 ECap1IntCount++;
    }

    if((0<ECap1IntCount)&&(ECap1IntCount<31)&&ECap1IntCount%5==0)
    {
    	GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1;
    }
   //ECap1PassCount++;

   ECap1Regs.ECCLR.bit.CEVT1 = 1;
   ECap1Regs.ECCLR.bit.INT = 1;
   ECap1Regs.ECCTL2.bit.REARM = 1;

   // Acknowledge this __interrupt to receive more __interrupts from group 4
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;


}

/*
__interrupt void ecap2_isr(void)
{
   //ECap runs twice as fast on F2837xS so multiplier
   //was increased from x2 to x4 and the fudge factor was
   //increased from 1 to 4.

   // Cap input is syc'ed to SYSCLKOUT so there may be
   // a +/- 1 cycle variation

    //CpuTimer0Regs.TCR.all = 0x0010; // Use write-only instruction to set TSS bit = 0
	T1_2=ECap2Regs.CAP1;
    //T2_2=ECap2Regs.CAP2;
	//T3_2=ECap2Regs.CAP3;
	//T3_2=ECap2Regs.CAP4;

	//CpuTimer0Regs.PRD.all= 100000000;
	time = CpuTimer0Regs.TIM.all;
    CpuTimer0Regs.TCR.all = 0x0010; // Use write-only instruction to set TSS bit = 0
	phase=(100000000-time)/T1_2*100;




	//phase = (100000000-time)/T1_2;
//   if(ECap1Regs.CAP2 > EPwm3Regs.TBPRD*4+4 || ECap1Regs.CAP2 < EPwm3Regs.TBPRD*4-4)
//   {
//       Fail();
//   }

	Speed = 200000000/(T1_2+1)*60*2;

	ECap2Regs.ECCLR.bit.CEVT1 = 1;
	//ECap2Regs.ECCLR.bit.CEVT2 = 1;
   ECap2Regs.ECCLR.bit.INT = 1;
   ECap2Regs.ECCTL2.bit.REARM = 1;

   // Acknowledge this __interrupt to receive more __interrupts from group 4
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;


}
*/


void Fail()
{
   asm("   ESTOP0");
}

__interrupt void cpu_timer0_isr(void)
{
   //CpuTimer0.InterruptCount++;

   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

__interrupt void xint1_isr(void)
{
	//CpuTimer0Regs.TCR.all = 0x0010;
	if(flag_phase==1)
	{
		if(count_tulun<11)
			{

				time = CpuTimer0Regs.TIM.all;
				//phase_lib = &tu_phase[count_tulun];
				phase = (100000000-time)/(400000000.0*60/Speed)*100-tu_phase[count_tulun]*100;
				count_tulun++;
			}
		else
			{

				time = CpuTimer0Regs.TIM.all;
				//phase_lib = &tu_phase[11];
				phase = (100000000-time)/(400000000.0*60/Speed)*100-tu_phase[count_tulun]*100;
			    CpuTimer0Regs.TCR.all = 0x0010; // Use write-only instruction to set TSS bit = 0
			    count_tulun=0;
			}


	}

	// Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void xint2_isr(void)
{
	//CpuTimer0Regs.TCR.all = 0x0010;
	CpuTimer0Regs.TCR.all = 0x4020; // Use write-only instruction to set TSS bit = 0

	flag_phase = 1;

	// Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void EPwmSetup(void)
{
   // InitEPwm3Gpio();
	EPwm3Regs.TBSTS.all=0;
	EPwm3Regs.TBPHS.bit.TBPHS=0;
	EPwm3Regs.TBCTR = 0;

	//EPwm2Regs.CMPCTL.all=0x50;        // Immediate mode for CMPA and CMPB
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = 0x0;
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0x0;
	EPwm3Regs.CMPCTL.bit.LOADBMODE = 0x0;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = 0x0;
	EPwm3Regs.CMPA.bit.CMPA = 1000;
	EPwm3Regs.CMPB.bit.CMPB = 2000;

	//EPwm3Regs.AQCTLA.all=0x60;        // EPWMxA = 1 when CTR=CMPA and counter inc
	                                                         // EPWMxA = 0 when CTR=CMPA and counter dec
	EPwm3Regs.AQCTLA.bit.CBD = 0x0;
	EPwm3Regs.AQCTLA.bit.CBU = 0x0;
	EPwm3Regs.AQCTLA.bit.CAD = 0x0;
	EPwm3Regs.AQCTLA.bit.CAU = 0x1;
	EPwm3Regs.AQCTLA.bit.PRD = 0x0;
	EPwm3Regs.AQCTLA.bit.ZRO = 0x2;
	//EPwm3Regs.AQCTLA.all = 0x0012;

	EPwm3Regs.AQCTLB.bit.CBD = 0x0;
	EPwm3Regs.AQCTLB.bit.CBU = 0x1;
	EPwm3Regs.AQCTLB.bit.CAD = 0x0;
	EPwm3Regs.AQCTLB.bit.CAU = 0x0;
	EPwm3Regs.AQCTLB.bit.PRD = 0x0;
	EPwm3Regs.AQCTLB.bit.ZRO = 0x2;
	//EPwm3Regs.AQCTLB.all = 0x0102;

	EPwm3Regs.AQSFRC.bit.RLDCSF = 0x0;
	EPwm3Regs.AQCSFRC.all=0x0;

	EPwm3Regs.DBCTL.all =0x0;
	EPwm3Regs.DBRED.all =0;
	EPwm3Regs.DBFED.all =0;

	EPwm3Regs.TZSEL.all=0;
	EPwm3Regs.TZCTL.all=0;
	EPwm3Regs.TZEINT.all=0;
	EPwm3Regs.TZFLG.all=0;
	EPwm3Regs.TZCLR.all=0;
	EPwm3Regs.TZFRC.all=0;

	EPwm3Regs.ETSEL.all=0;            // Interrupt when TBCTR = 0x0000
	EPwm3Regs.ETFLG.all=0;
	EPwm3Regs.ETCLR.all=0;
	EPwm3Regs.ETFRC.all=0;
	EPwm3Regs.PCCTL.all=0;

	EPwm3Regs.TBPRD = 49999;
	EPwm3Regs.TBCTL.bit.FREE_SOFT = 0x2;
	EPwm3Regs.TBCTL.bit.CLKDIV = 0x2;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0x5;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = 0x3;
	EPwm3Regs.TBCTL.bit.PRDLD = 0x0;
	EPwm3Regs.TBCTL.bit.PHSEN = 0x0;
	EPwm3Regs.TBCTL.bit.CTRMODE = 0x0;

}
//===========================================================================
// No more.
//===========================================================================
