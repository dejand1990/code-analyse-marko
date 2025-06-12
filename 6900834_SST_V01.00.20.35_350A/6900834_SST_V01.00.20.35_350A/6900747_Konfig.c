// fuer Version 840b
#include "p33EP32GS502.h"
#include "6900747_HeaderFile.h"
//volatile unsigned int pwm_p;
void Konfig(void)
{			//      5432109876543210
	TRISA  		= 0B0000000000000111;
	TRISB		= 0B0001111000100110;       //502

	ODCB		= 0B0100000000000000;			// Port B  kein o.D
	ODCA		= 0B0000000000011000;			// SW als OpenDrain A3
                           
// Configure the I/O pins to be used as analog inputs.
ANSELA=0; ANSELB=0;
ANSELAbits.ANSA0 = 1;     // AN0/RA0 connected the dedicated core 0
ANSELAbits.ANSA1 = 1;     // AN1/RA1 connected the dedicated core 1 
ANSELBbits.ANSB9 = 1;     // AN4/RB9 connected the Shared core
ANSELBbits.ANSB10 = 1;    // AN5/RB10 
ANSELBbits.ANSB2 = 1;     // AN7/RB2
  
//TIMER KONFIG
T1CON 		= 0b1000000000010000;				// timer comunication frame max.
/*														//bit15 =0		Timer1 ein
//														//bit13 =0		kein Idle
//														//bit06 =0		kein gate trigger
//														//bit05_04 =01	step 125ns,
//														//bit02 = 0		kein synk
//														//bit01 = 0		clock von FCY
 */
PR1=320;                                                //40us

T3CON 		= 0b1000000000110000;				// State machine timing
														/*bit15 =1		Timer3 eingeschaltet
														//bit13 =0		Idle ignorirt
														//bit06 =0		Gate trigger aus
														//bit05_04 =11	step 4us, overflow 260ms
														//bit03 = 0  		Timer 32 nicht konfiguriert
														//bit01 = 0  		clock von Fp
                                                         */
PR3=_1ms;											//1ms Zaehler


				//    5432109876543210
//	T2CON 		= 0b1000000000100000;				//  netz Frequenz 
														/*bit15 =1		Timer2 eingeschaltet
														//bit13 =0		Idle ignorirt
														//bit06 =0		Gate trigger aus
														//bit05_4 =10	step 0,4us , 1:64
														//bit03 = 0  		Timer 32 nicht konfiguriert
														//bit01 = 0  		clock von FCY
                                                         */
//	PR2=(_1ms * 100);									//default fuer einschalten
                                                         
// fuer OC
            //    5432109876543210
    T2CON	=	0b1000000000000000;
    T2CONbits.TON = 0;       // Turn off Timer2
    T2CONbits.TCS = 0;       // Select internal clock (F_P)
    T2CONbits.TGATE = 0;     // Disable gated time accumulation
    T2CONbits.TCKPS = 0b00;  // Select 1:1 prescaler

    // 2. Set Timer2 Period for 50 µs
    PR2 = 3199;             //
    IFS0bits.T2IF       = 0;                                                    // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE       = 1;   
//Grosse Nummer grosse Prioritaet
	IPC14bits.PSEMIP	=	5;                      // Special Interrupt hoechste Prioritaet
    IPC18bits.PSESIP    =   6;
    IPC27bits.ADCAN1IP  =   4;
    IPC4bits.CNIP	 	=   7;	
	IPC0bits.T1IP 		=   2;
	//IPC28bits.ADCAN3IP 	= 3;                    	// Set ADC Pair 3 Interrupt Priority (Level 2)
     
// Konfig UART, LSB
//	IEC0bits.U1TXIE=0;
	U1MODEbits.STSEL = 0;							// 1-stop bit
	U1MODEbits.PDSEL = 2;							// odd Parity, 8-data bits
	U1MODEbits.ABAUD = 0;							// Auto-Baud Disabled
	U1MODEbits.BRGH = 1;							// 1 high speed, bei 0 low speed
	U1BRG = 3;//(CLOCK_FREQ_D4 / BAUD) - 1;			// 4 Mbaud, bei BRGH=1
	U1STAbits.UTXISEL0	 = 0;						// Interrupt after one TX character is transmitted to Shift register
	U1STAbits.UTXISEL1	 = 1;	
	U1STAbits.URXISEL	 = 3;						//interrupt bei 4 bytes, buffer voll
	U1MODEbits.UARTEN	 = 1;						// Enable UART TX/*
	U1STAbits.UTXEN		 = 1;						// Initiate transmission

	IFS0bits.U1RXIF=0;								// Clear Interrupt Flag
	IEC0bits.U1RXIE=1;								// Interrupt erlauben
    TRISBbits.TRISB12=1;
  
//U2MODEbits.STSEL = 0; // 1-Stop bit
//U2MODEbits.PDSEL = 2; // No Parity, 8-Data bits
//U2MODEbits.ABAUD = 0; // Auto-Baud disabled
//U2MODEbits.BRGH = 1; // Standard-Speed mode
//U2BRG = 1; // Baud Rate setting for 9600
//U2STAbits.UTXISEL0 = 0; // Interrupt after one TX character is transmitted
//U2STAbits.UTXISEL1 = 1;
//
//
//
//IEC1bits.U2TXIE = 1; // Enable UART TX interrupt
//U2MODEbits.UARTEN = 1; // Enable UART
//U2STAbits.UTXEN = 1; // Enable UART TX  
    
    
    
}
//

void Init_UART(void)
{
    U1MODEbits.STSEL = 0;							// 1-stop bit
	U1MODEbits.PDSEL = 2;							// odd Parity, 8-data bits
	U1MODEbits.ABAUD = 0;							// Auto-Baud Disabled
	U1MODEbits.BRGH = 1;							// 1 high speed, bei 0 low speed
    if(baud_rq == 1)
    {
        U1BRG = 3;//(CLOCK_FREQ_D4 / BAUD) - 1;			// 4 Mbaud, bei BRGH=1
    }
    else if(baud_rq == 2)
    {
        U1BRG = 1;//(CLOCK_FREQ_D4 / BAUD) - 1;			// 8 Mbaud, bei BRGH=1
    } 
	U1STAbits.UTXISEL0	 = 0;						// Interrupt after one TX character is transmitted to Shift register
	U1STAbits.UTXISEL1	 = 1;	
	U1STAbits.URXISEL	 = 3;						//interrupt bei 4 bytes, buffer voll
	U1MODEbits.UARTEN	 = 1;						// Enable UART TX/*
	U1STAbits.UTXEN		 = 1;						// Initiate transmission

	IFS0bits.U1RXIF=0;								// Clear Interrupt Flag
	IEC0bits.U1RXIE=1;								// Interrupt erlauben
    TRISBbits.TRISB12=1;
}

void PWM_Konfig(void)
{
  
PTCONbits.PTEN=0;                                   // zu erst abschalten
// PTCON2: PWM Clock Divider Select Register
	PTCON2 = 0;                                    // max. Clock
    STCON2 = 0;
    
// PTPER: PWM Master Time Base Register
	PTPER		= PWM_P & 0xfff8;					// Period PWM1 13,79us (72,52 kHz) ==> 13007 * 1,06ns | diethering Bag
    STPER		= PWM_P & 0xfff8;
// PTCON: PWM Time Base Control Register
	PTCONbits.SEIEN=1;                              // Special Event Interrupt Enable bit, base
    STCONbits.SEIEN=1;                              // secondary

// SEVTCMP: PWM Special Event Compare Register	
    SEVTCMP	= _InterruptZeit-100;                   // Interrupt von PWM nach max. Zeit ,auf PWM - Anfang setzen!
    SSEVTCMP =(STPER>>1)+_InterruptZeit;            // hier auch
 
    IEC3bits.PSEMIE=1;                              // Interrupt enable
    IEC4bits.PSESIE=1;

    //--------------------------------------------------------------------------
// ~~~~~~~~~~~~~~~~~~~~~~ PWM1 Configuration Strom ~~~~~~~~~~~~~~~~~~~~~~~~~ //
//--------------------------------------------------------------------------
   				  //5432109876543210
PWMCON1 = 0B0000000000000000;           			// TRGIEN = 0, CLIEN = 0 FLTIEN = 0
// Pinkonfiguration
//   PWMCON1bits.TRGIEN=1;
/*
//	IOCON1bits.PENH	=1;								// 1 = control bei PWM
//	IOCON1bits.PENL	=0;								// 1 = control bei PWM
//	IOCON1bits.POLH	=0;								// 0 = PWMxH pin is active-high
//	IOCON1bits.POLL	=0;								// 0 = PWMxH pin is active-high
// 	IOCON1bits.PMOD	=0b11;							// 11 = PWM I/O pin pair is in the True Independent Output mode
//	IOCON1bits.OVRENH =0;	
//	IOCON1bits.OVRENL =0;	
//	IOCON1bits.OVRDAT =0b00;
//	IOCON1bits.FLTDAT =0b00; 						// 0 = Fault mode aktiv 0
//	IOCON1bits.CLDAT =0b00;							// bit 3-2 ,Current mode limit aktiv 0
//	IOCON1bits.SWAP	=0;
//	IOCON1bits.OSYNC =0;							// bit 0 
 */
   				  //5432109876543210
	IOCON1		= 0b1000110000000000;
    
 // FCLCON1 	= 0b0000000100001001;				//independent, current limit.  FLT1 und FLT2 fuer Strombegr. fuer PWM1
 //	FCLCON1bits.IFLTMOD = 0;						// 1- Independent Fault mode
	FCLCON1bits.CLSRC = 15;                      	// Fault14 als Current - Quelle 
//	FCLCON1bits.CLPOL = 0;							// aktive High
	FCLCON1bits.CLMOD = 1;							// forces PWMxH, PWMxL pins to CLDAT values (cycle)
	FCLCON1bits.FLTSRC =14;              			// Fault13 als Ueberstrom
//	FCLCON1bits.FLTPOL = 0;							// aktive High
	FCLCON1bits.FLTMOD = 0b01;						// forces PWMxH, PWMxL pins to FLTDAT values (cycle)
  
// Leading-Edge Blanking
   			      //5432109876543210
	LEBCON1 	= 0B1000010000000000;          		// Silizium Bug + 50ns Ausblendung
	LEBDLY1bits.LEB = 15;							// 30 *8ns, um PEM - Anfang zu sperren (Stoerung)

//	AUXCON1		= 0b0000000100000000;
	TRIG1 		= 0;								// nicht mehr genutzt, wird bei Interrupt gemessen
	STRIG1		= 0;
	PHASE1	 	= 0;								// Phase von PWM1H, verschiebt in Minusrichtung
	SPHASE1 	= 0;                                // 
	DTR1   		= 0;                            	// Dead-Time Value for PWM1, verschiebt in Plusrichtung. Original um 1/2 Periode
	PDC1		= 0;							// PWM1 Duty Cycle
	ALTDTR1		= 0;
	SDC1		= 0;								// 

//--------------------------------------------------------------------------
// ~~~~~~~~~~~~~~~~~~~~~~ PWM2 Configuration Ueberstrom ~~~~~~~~~~~~~~~~~~~~ //
//---------------------------------------0-----------------------------------
   			      //5432109876543210
	PWMCON2 	= 0B0000000000000000;				// 
   // PWMCON2bits.IUE = 1;

/*    
//	IOCON2bits.PENH	=0;								// 1 = control bei PWM
//	IOCON2bits.PENL	=1;								// 1 = control bei PWM
//	IOCON2bits.POLH	=0;								// 0 = PWMxH pin is active-high
//	IOCON2bits.POLL	=0;								// 0 = PWMxH pin is active-high
// 	IOCON2bits.PMOD	=0b11;							// 11 = PWM I/O pin pair is in the True Independent Output mode
//	IOCON2bits.OVRENH =0;	
//	IOCON2bits.OVRENL =0;	
//	IOCON2bits.OVRDAT =0b00;
//	IOCON2bits.FLTDAT =0b00; 						// 0 = Fault mode aktiv 0
//	IOCON2bits.CLDAT =0b00;							// Current mode limit aktiv 0
//	IOCON2bits.SWAP	=0;
//	IOCON2bits.OSYNC =0;
*/
   			      //5432109876543210
	IOCON2		= 0b0100110000000000;

//	FCLCON2bits.IFLTMOD = 0;						// 1- Independent Fault mode
	FCLCON2bits.CLSRC = 16;          				// auch Strom
//	FCLCON2bits.CLPOL = 0;							// aktive High
	FCLCON2bits.CLMOD = 1;							// forces PWMxH, PWMxL pins to CLDAT values (cycle)
	FCLCON2bits.FLTSRC =14;                  		// Fault2 , Ueberstrom
//	FCLCON2bits.FLTPOL = 0;							// aktive High
	FCLCON2bits.FLTMOD = 0b01;						// forces PWMxH, PWMxL pins to FLTDAT values (cycle)

   			      //5432109876543210
//	FCLCON2 	= 0b0000010100100011;				// independent, current limit. Fault2 // Fault zu Fault8(kein)  
    
	LEBCON2 	= 0b0010010110000000;           	// Silizium Bug + 100ns aus blendung
    LEBDLY2bits.LEB = 15;                           // um 240ns

	PDC2		= 0;								// PWM2 Duty Cycle init, nicht gebraucht
    PHASE2      = (PTPER>>1);
	DTR2		= 0;								// Dead-Time Value for PWM2
	TRIG2		= 2000;                         	// Triggerung ADC an Strom Im Impuls	
	STRIG2		= 1000;//(PTPER>>1)+1000;                  // Triggerung ADC an Strom Im Impuls, der zweite
	SDC2		= 0;                            //  max., wird durch Komparator gekuertzt
	SPHASE2		=(PTPER>>1);						// um 180° verschoben wegen , so kommt 2 * fs im Ausgang zusammen

////////////////////////////////////////////   PWM3
PWMCON3 = 0;	
IOCON3 = 0;                                //sonst sind die Ausgaenge aktiv! 
 PTCONbits.PTEN = 1;						// Enable PWM Modul nach Konfigurieren
}
//
void InitIO()
{
// Virtual Pinbelegung:
__builtin_write_OSCCONL(OSCCON & ~(1<<6));	//  ioUnlock();

// Virtual verbindung zwischen GND und FLT3
	RPINR13bits.FLT3R = 0;                  // Fault 3 auf Vss 

// Port Configure SPI
	RPOR3bits.RP38R	= 0B000110;			// RP38 --> SPI clock out
	RPOR3bits.RP39R	= 0B000101;			// RP39 --> SPI data out
	RPINR20bits.SDI1R = 43;				// RP43 --> SPI data in
    RPINR20bits.SCK1INR = 38;           // SPI clock auch zu Out

// Port Configure UART
	RPOR6bits.RP45R		= 0b000001;		// RP45 --> UART data out
	RPINR18bits.U1RXR	= 44;			// RP44 --> UART data in
    
    
//     RPOR4bits.RP40R		= 0b000011;		// RP40 --> UART2 data out
   /*
#ifdef _Enable_OC
    RPOR4bits.RP40R = 0b010000;         //RP47 (info pin) set as OC pin
#endif  */
    
#ifdef _KompamPIN
		RPOR7bits.RP47R	= 0b110010;					// ACMP4 output am RB15 (pin 15)
//		RPOR7bits.RP47R	= 0b011001;					// ACMP2 output am RB15 (pin 15)

#endif

//Pin virtuali sigillati
__builtin_write_OSCCONL(OSCCON | (1<<6));			 //ioLock();
}
//
void COMP_DAC(void)
{
// konfigurieren der Pins fuer analog Komparator
// bit 7-6 INSEL
//11 = Selects CMPxD input pin
//10 = Selects CMPxC input pin
//01 = Selects CMPxB input pin
//00 = Selects CMPxA input pin

//		b15. CMPON=1 		Komparator 1 eingeschaltet
//		b13. CMPSIDL=0 ; 	keine IDLE
//		b7_6. INSEL=01 ; 	Komparator CMP2B zu AN3
//		b5. =0 ; 		
//      b4. HYSPOL=0;       Hystereso pol. low-high
//		b3. CMPSTAT=0 ; ;	Komparator State (mit CMPPOL)
//		b1. CMPPOL=0 ; 		Ausgang nicht invertiert
//		b0. 1 ;             kein RANGE bei EP_502

		 //5432109876543210
CMP2CON =0B1000000000000001;			// komparator 2A an AN2 Imax
CMP1CON =0B1000000000000001;			// komparator 1A zu AN0 fuer Spannung Uist(Fokus..)
CMP3CON =0B1000000010000001;            // komparator 3C zu AN6 fuer PWM1
CMP4CON =0B1000000000000001;            // komparator 4A zu AN6 fuer PWM2

//CMP1CONbits.DACOE = 1;                  // Wert zu RB3

CMP1DAC= 4000;                          // hier noch aus um 100V
CMP3DAC= 0;                             // Strom aus
CMP4DAC= 0;
//IEC6bits.AC2IE = 1;                     // Interrupt enable, Komp 2, Ueberstarom
}//
void InitOsc(void)
{
	// Configure Oscillator to operate the device at 64MHz
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	// Fosc= 7.37 *(74)/(2*2)=136.4Mhz for Fosc, Fcy = 68.Mhz 
	// To have INTOSC= 7.31MHz OSCTUN=47 

// Configure PLL prescaler, PLL postscaler, PLL divisor 
    // fuer 64MHz
	OSCTUN=47;										// 7,314 MHz
    PLLFBD=68;                              		// (von 2 - 512) M = PLLFBD + 2 =70 
    
	CLKDIVbits.PLLPOST=0;                      		// N2 = 2 *(POST+1) =2
	CLKDIVbits.PLLPRE=0;	    					// N1 = PRE+ 2 =2

	OSCCONbits.NOSC=1;              				// Enable FRC with PLL
	CLKDIVbits.FRCDIV=0;	    					// FRC divider=1 
	__builtin_write_OSCCONH(0x01);      		  	// New Oscillator FRC w/ PLL 
	__builtin_write_OSCCONL(0x01);      	   		// Enable Switch 
	while(OSCCONbits.COSC != 0b001);     	 		// Wait for new Oscillator to become FRC w/ PLL   
	while(OSCCONbits.LOCK != 1);            		// Wait for Pll to Lock   

// clock fuer PWM
	ACLKCONbits.FRCSEL = 1;             	    	// FRC provides input for Auxiliary PLL (x16) 
	ACLKCONbits.ASRCSEL = 0;            			// Clk source for FRCSEL, 1=Primary Oscillator, 0=No Clk
	ACLKCONbits.SELACLK = 1;          		     	// Auxiliary Oscillator provides clock source for PWM & ADC 
	ACLKCONbits.APSTSCLR = 7;                   	// Divide Auxiliary clock by 1 
	ACLKCONbits.ENAPLL = 1;            	        	// Enable Auxiliary PLL  ACLK=FRC*16 
}
//
void Init_ADC(void)
 {
  	ADCON1L = 0x0000;								// Erst Setting, nach einschalten
    
    // Configure the common ADC clock.
    ADCON3Hbits.CLKSEL = 0b01;                      // 01 = FOSC (System Clock x 2)
    ADCON3Hbits.CLKDIV = 0b000001;                  // 000001 = 2 Source Clock Periods
    
    // ADC Core Input Clock Divider TADCORE=TCORESRC/2 
    // FADCORE = FOSC / 2 / 2 = 91,30MHZ / 2 / 2 = 22,83MHz 
    // FCORESRC = FOSC / 2 = 91,30MHZ /2 = 45,65MHz
    // Conversion Time = 8 * TCORESRC + (Resolution + 2,5) * TADCORE = 0,7us(10Bit)
    
    ADCORE0Hbits.ADCS =     0b0000011;                // ADC Dedicated Core 0 Input Clock Divider: FADC /6 0000001 = 2 Source Clock Periods
    ADCORE1Hbits.ADCS =     0b0000011;                // ADC Dedicated Core 1 Input Clock Divider: FADC /6
    ADCON2Lbits.SHRADCS =   0b0000011;                // ADC Shared Core Input Clock Divider: FADC /6
    
    // ADC Aufloesung
    ADCORE0Hbits.RES = 0b11;                        // Dedicated ADC Core 0 Resolution Selection bits 10=10Bit
    ADCORE1Hbits.RES = 0b11;                        // Dedicated ADC Core 1 Resolution Selection bits 10=10Bit
    ADCON1Hbits.SHRRES = 0b11;                      // Shared ADC Core Resolution Selection bits. 0b10=10bit, 0b11=12bit 
 
    ADCON1Hbits.FORM = 0;                           // Ausgabe als Integer. Für alle ADC Cores.
    
    ADCON4Lbits.SYNCTRG0 = 0;                       // Dedicated SAR ADC Core Synchronous Sampling. SAR No. 0 V502
    ADCON4Lbits.SYNCTRG1 = 0;                       // Dedicated SAR ADC Core Synchronous Sampling. SAR No. 1
   
// Set initialization time to maximum
    ADCON5Hbits.WARMTIME = 15;
	ADCON1Lbits.ADON = 1;					// ADC Ein 
    Delay_ms(1);

    // ADC Core enable 
    // Turn on analog power for dedicated core 0
    ADCON5Lbits.C0PWR = 1;
    // Wait when the core 0 is ready for operation
    while(ADCON5Lbits.C0RDY == 0);
    // Turn on digital power to enable triggers to the core 0
    ADCON3Hbits.C0EN = 1;
   
    // Turn on analog power for dedicated core 1
    ADCON5Lbits.C1PWR = 1;
    // Wait when the core 1 is ready for operation
    while(ADCON5Lbits.C1RDY == 0);
    // Turn on digital power to enable triggers to the core 1
    ADCON3Hbits.C1EN = 1; 
 
    // Turn on analog power for shared core
    ADCON5Lbits.SHRPWR = 1;
    // Wait when the shared core is ready for operation
    while(ADCON5Lbits.SHRRDY == 0);
    // Turn on digital power to enable triggers to the shared core
    ADCON3Hbits.SHREN = 1; 
 
     // Enable calibration for the dedicated core 0
    ADCAL0Lbits.CAL0EN = 1;
    // Single-ended input calibration
    ADCAL0Lbits.CAL0DIFF = 0;
    // Start calibration
    ADCAL0Lbits.CAL0RUN = 1;
    // Poll for the calibration end
    while(ADCAL0Lbits.CAL0RDY == 0);
    // End the core 0 calibration
    ADCAL0Lbits.CAL0EN = 0;  
   
    // Enable calibration for the dedicated core 1
    ADCAL0Lbits.CAL1EN = 1;
    // Single-ended input calibration
    ADCAL0Lbits.CAL1DIFF = 0;
    // Start calibration
    ADCAL0Lbits.CAL1RUN = 1;
    // Poll for the calibration end
    while(ADCAL0Lbits.CAL1RDY == 0);
    // End the core 1 calibration
    ADCAL0Lbits.CAL1EN = 0;  

    // Enable calibration for the shared core
    ADCAL1Hbits.CSHREN = 1;
    // Single-ended input calibration
    ADCAL1Hbits.CSHRDIFF = 0;
    // Start calibration
    ADCAL1Hbits.CSHRRUN = 1;
    // Poll for the calibration end
    while(ADCAL1Hbits.CSHRRDY == 0);
    // End the core 3 calibration
    ADCAL1Hbits.CSHREN = 0;
    
    // Selects trigger source for conversion of analog channels 
    // AN0
	ADTRIG0Lbits.TRGSRC0=0b00110;                 // 00110 = PWM2 Primary Trigger selected
    // AN1
	ADTRIG0Lbits.TRGSRC1=0b00110;                 // 00110 = PWM2 Primary Trigger selected
    // AN2
	ADTRIG0Hbits.TRGSRC2=0b00000;                 // 00000 = no conversion selected AN2 e AN3
    // AN3
	ADTRIG0Hbits.TRGSRC3=0b00000;                 // 00000 = no conversion selected AN2 e AN3
    // AN4
	ADTRIG1Lbits.TRGSRC4=0b00100;                 // 00100 = PWM1 Primary Trigger selected
    // AN5
	ADTRIG1Lbits.TRGSRC5=0b00100;                 // 00100 = PWM1 Primary Trigger selected
    // AN6
	ADTRIG1Hbits.TRGSRC6=0b00000;                 // 00100 = PWM1 Primary Trigger selected
    // AN7
	ADTRIG1Hbits.TRGSRC7=0b00100;                 // 00100 = PWM1 Primary Trigger selected
    
    // Configure and enable ADC interrupts.
//ADIELbits.IE1 = 1; // enable interrupt for AN1
//IFS5bits.PWM1IF = 0; // clear interrupt flag for AN1
//_ADCAN1IE = 1; // enable interrupt for AN1

 }
//
/*
void InitOC(void)
{
    OC1CON1 = 0b0001110000001110;
    OC1CON2 = 0b0000000000011111;
    OC1R = 32000; //default = middle
    OC1RS = 64000;   //means with Fcy = 64 MHz ---> period sync of 15 us
}*/