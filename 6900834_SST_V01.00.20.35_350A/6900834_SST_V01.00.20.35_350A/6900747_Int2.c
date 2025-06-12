
#include "p33EP32GS502.h"
#include "6900747_HeaderFile.h"

volatile unsigned int Sollwert_Hardware;		// Potiwert gemittelt
volatile unsigned int Uzk_mid1=10;				// obere Spannung, Uzk(SPI)-Uzk21. bei 748 Uzk-Uzk21
volatile unsigned int Uzk_mid2=10;				// untere Spannung, Uzk22_p.  bei 748 zusammen, gleicher Wert wie Uzk_mid2
volatile unsigned int Uzk=10;					// gesamt, oben, nach Relais
volatile unsigned int Uzk_p=10;					// primaer, vor Relais, bei 747 ueber PFC SPI
volatile unsigned int UzkAvr=10;				// Mittelwert der beiden UZK(mid1, mid2), fuer Reglerkorrektur
volatile signed int Sym_relative=0;			// Abweichung in ns zu PWM - Copy
//volatile unsigned int C5=350;					// Hilfsvar. fuer Lm-Berechnung 350V / Lm startwert ( in V/mH)

volatile unsigned int Timer_cut_off;			// Sendung der Daten Ueberwachung, Alive
volatile unsigned int Netz_zeit=0;				// die Zeit der vollen Netzwelle ( Frequenz))
volatile unsigned int NetzAusfall=0;			// 0-> ok, 1 -> keine Halbewelle in ueber 100ms

volatile unsigned int Duty;
volatile unsigned int ProtokollTimer;			// Sendungen ueber UART trigern, erlauben 
volatile unsigned int LaufTimer;				// Messzeit fuer test der Kurzschluesse ,1ms
volatile unsigned int TimerLk;					// alle 10ms Drossel ermitteln
volatile unsigned int Rampe=1;					// sonfte Aenderung vom Strom
volatile signed int Istep=10;					// max. Stromsprung bei Rampe
volatile signed int Pol=1;                      // Richtung vom Sollert zu vorher
volatile signed int Trepp=8;                    // bis 8 Treppen vom Sollwert
volatile unsigned int aux_i2=0;
volatile unsigned int Alive_timeout_cnt =0;
volatile unsigned int dummy=0;
volatile unsigned int Dum_fix=0;
volatile signed int Ignition_time_cnt1 = 0;
volatile signed int Ignition_time_cnt2 = 0;
volatile signed int Ignition_time_cnt3 = 0;
volatile signed int Ignition_time_cnt4 = 0;
volatile signed int Ignition_time_cnt5 = 0;
volatile signed int Ignition_time_cnt6 = 0;
volatile signed int Ignition_time_cnt7 = 0;
volatile unsigned int block_Isoll = 0;
volatile unsigned int Iblock_cnt = 0;
volatile unsigned int TEST_CNTR = 1;
volatile unsigned int KOMMANDO_RCV;                                             // Command stored for veryfying the strom mode
volatile unsigned int HYPER_PULSE_MODUS;                                        // HyperPulse mode = 1 or Normal mode = 0
volatile unsigned int PULSE_STROM_FAKTOR_BIT;                                   // find the 15th bit (0 or 1) in order to calculate the I1 and I2
volatile unsigned int PULSE_STROM_FAKTOR_BIT_LAST; 
volatile unsigned int PULSE_STROM_FAKTOR_VALUE;                                 // Pulse strom factor value
volatile unsigned long PULSE_STROM_I1;                                          // Soll Strom I1
volatile unsigned long PULSE_STROM_I2;                                          // Soll Strom I2
volatile unsigned long PULSE_ZEIT_T1_LOW_HOLD;
volatile unsigned long PULSE_ZEIT_T1_HIGH_HOLD;
volatile unsigned long PULSE_ZEIT_T2_LOW_HOLD;
volatile unsigned long PULSE_ZEIT_T2_HIGH_HOLD;
volatile unsigned long PULSE_FREQUENZ;                                           // Pulse frequency
volatile unsigned long PULSE_FREQUENZ_DIV_LAST;
volatile unsigned long PULSE_FREQUENZ_L;
volatile unsigned int PULSE_BALANCE;                                            // Pulse balance z.B. 50%
volatile unsigned int g_u16i_Iecht_temp;
volatile unsigned int puls_strom_change_akt_inakt;
volatile unsigned int g_u16i_Iecht_temp_last;
volatile unsigned int g_u16i_AC_STATUS;
volatile unsigned int g_u16i_AC_PULSE_COUNTER ;
volatile unsigned int m_u16i_AC_DC_STATE;
volatile unsigned long PULSE_FREQUENZ_DIV;   
volatile unsigned long PULSE_FREQUENZ_STORE;
volatile unsigned int FRQ_Changed;
volatile unsigned int ACKNG;
volatile unsigned int VALID_one;
volatile unsigned int VALID_two;
volatile signed long STROM_DIFF;  
// Datensatz zum Senden//
volatile unsigned int start_count=0;											
volatile unsigned int deltaI=0;
volatile unsigned int PFC_UZK_cnt = 0;	
volatile unsigned int five_hun_ms_counter=0;
volatile unsigned int recheck=0;
volatile unsigned int block_counter=0;
volatile unsigned int t1_t2_full_rx;
volatile unsigned int g_u16i_Timer_LT_U0_Reduktion;
volatile unsigned int g_u16i_Wig_zundung_Mode_Lift_arc;
volatile unsigned int g_u16i_AC_Lock;
volatile unsigned int Umpolung_Aktiv;

#ifdef Trafo450
volatile unsigned int Imax_setpoint1 = 2400;
volatile unsigned int Imax_setpoint_nominal = 1860;
volatile unsigned int Imax_setpoint_time1 = 500;
volatile unsigned int Imax_setpoint_nominal_A = 460;
volatile unsigned int Imax_setpoint_max = 750;
#else
// Mr. K ought to limit the maximum set current to 500A but, when Imax_setpoint_max sets to 500, then plus region of AC rises beyond 570A, so 449A is set!
volatile unsigned int Imax_setpoint1 = 1796;                                    
volatile unsigned int Imax_setpoint_nominal = 1560;
volatile unsigned int Imax_setpoint_time1 = 500;
volatile unsigned int Imax_setpoint_nominal_A = 390;
// Mr. K ought to limit the maximum set current to 500A but, when Imax_setpoint_max sets to 500, then plus region of AC rises beyond 570A, so 449A is set!
volatile unsigned int Imax_setpoint_max = 449;                                  
#endif
				
// empfangener Datensatz

//// INTERRUPT LEVEL 4

//
void __attribute__((__interrupt__, no_auto_psv)) _ISR _T2Interrupt(void)	// Timer2 Interrupt  Zeitkonstante (360ms)
{ 
    /*##RAVI: check here now communication is done directly every 50us*/
    TMR2=0;
    Kommunikation(); 
    IFS0bits.T2IF = 0; 
}

void __attribute__((__interrupt__, no_auto_psv)) _ISR _T5Interrupt(void)	// Timer2 Interrupt  Zeitkonstante (360ms)
{
        IFS1bits.T5IF=0;        
}
//// INTERRUPT LEVEL 2
void __attribute__((__interrupt__, no_auto_psv)) _ISR _T3Interrupt(void)		// Timer3 Interrupt  Zeitkonstante	(1ms)
{
	static int tt;
    static int si16TimerSec = 0;
	TMR3=0;																		// Timer_Zuruecksetzen
    g_u16i_1msec_Period_Pdc1 ++;
	Timer++;
    // Haupt Timer+1ms																// UART Timer+1ms
	 si16TimerSec++;
    if(si16TimerSec > 1000)
    {
        si16TimerSec      = 0;
        g_u16i_Timer_LT_U0_Reduktion++;
    }
    
    if(tt)
	{tt=0; TimerUART++;}
	else	tt=1;
    
	Timer_cut_off++;															// UART Fehlertimer +1ms
	LaufTimer++;
	IFS0bits.T3IF=0;
	TimerRelais++;                                                              // Wartezeit auf Relais wieder ein
    start_counter++;
    Alive_timeout_cnt++;
    if(Alive_timeout_cnt >= 500 && INV_Enable == 1)
    {
        Alive_timeout_cnt = 510;
        INV_Enable = 0;
        OnOff = 0;  
        // = 1;
        //LED_B = 1;
    }
    //Imax/Time checkup
    if(enable_Iblock_cnt == 1)
    {
        Iblock_cnt++;
        if(Iblock_cnt >= 2)
        {
            enable_Iblock_cnt = 0;
            Iblock_cnt = 0;
        }
    }
    if(Ist_wert >= Imax_setpoint1)
    {
        Ignition_time_cnt1++;
        if(Ignition_time_cnt1 >= Imax_setpoint_time1)
        {
            block_Isoll = 1;
            Isoll=(Imax_setpoint_nominal_A<<2);
        }        
    }    
    else if(Ist_wert <= Imax_setpoint_nominal)
    {
        if(Ignition_time_cnt1 > 0)
        {
            Ignition_time_cnt1--;
        }
        else
        {
            Ignition_time_cnt1 = 0;
        }        
    }
    
    dummy++;
    
    if(dummy >= 1023)
    {
        dummy = 0;
        Dum_fix=Fault_pwm_code;
        //Dum_fix = 250;
    }
    
    Timer_LEDgelb++;
    
    if(dummy >= Dum_fix)
    {
        //TestPin=0;
    }
    else
    {
        //TestPin=1;
    }
    
    PFC_UZK_cnt++;
    if (block_start==1)
    {
        block_counter++;
    }
    
    if (block_counter>=500)
    {
        block_calculation=0;
        block_start=0;
    }        
}
//
void __attribute__((__interrupt__, no_auto_psv)) _ISR _T1Interrupt(void)		// Timer1 Interrupt timerzaehler sehen PR1 fuer die Zeitkonstant (23us)
{																			// nach der erste angekommende 8 bit, die weiterer 24 bit sollen innerhalb 20 us gekommen sein 
	
    T1CONbits.TON=0;         														// stoppt timer
	TMR1=0;                             										// zuruecksetzung der Zaehler
	//UARTinit();																	// clear der UART Buffer
	IFS0bits.T1IF=0;	

}
//
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)			// UART Buffer voll, die 4 Byte sind gekommen 7,7us 64MHz
	{
   
    static unsigned int Iecht_alt;
	TMR1=0; T1CONbits.TON=0;       IFS0bits.T1IF=0;             				// Timer stoppt, weil Sendung rechtzeitig da

	unsigned int Flag_UARTi=done;												// Init Flag
	if (U1STAbits.PERR==1)														// check nach paritaetfehler
		{Flag_UARTi=none;}
	UART_1=U1RXREG;                                                             // Byte 1 : Befehl-Kommando nummer
	if (U1STAbits.PERR==1)
		{Flag_UARTi=none;}
	UART_2=U1RXREG;                                                             // Byte 2 : Parameter High-Byte
	if (U1STAbits.PERR==1)
		{Flag_UARTi=none;}
	UART_3=U1RXREG;                                                             // Byte 3 : Parameter Low-Byte
	if (U1STAbits.PERR==1)
		{Flag_UARTi=none;}
	UART_4=U1RXREG;                                                             // Byte 4 : Check Summe
	CkS=UART_1+UART_2+UART_3;                                                   // Checksumme
	if (CkS!=UART_4)				                                            // Checksumme check
		{Flag_UARTi=none;}


	if (Flag_UARTi==done)			// gueltige Daten empfangen
		{ 
		if(UART_1< 16)												// wenn allg. Kommand
		RxDaten.Cmd=UART_1	;									
		else
		RxDaten.Cmd=(UART_1 | Page);								//  Kommando speichern mit Page 
		
		RxDaten.Wert=(UART_2<<8) + UART_3;							// Speichern von beiden bytes

		if (RxDaten.Cmd==0 && RxDaten.Wert==0)
			{
			Flag_UART=none;
			TxDaten.Wert=UART_1 & MASK_CMD;							// Fehler noACK senden
			TxDaten.Cmd=ACK;
			TxDaten.Ack=1;											// Fehler CS senden
			UARTinit();
			}														// Checksumme Fehler, noACK senden
		else
			{	
			if(Timer_cut_off>2000 && OnOff_Strom) {OnOff_Strom=0; OnOff=0;}
			Timer_cut_off=none; 									// Kommunikation erfolgt, timeout=0;
            
#ifdef _TestPanel
            if (RxDaten.Cmd==0xA0 || RxDaten.Cmd==0x5A0)
            {
                P5_Daten[ID_P5_LT_ONOFF]=RxDaten.Wert;				// speichern in die Matrix
                OnOff=RxDaten.Wert;									// speichern in die Variable

                TxDaten.Wert=0x0100 | (LT_ONOFF & MASK_CMD);
                TxDaten.Cmd=ACK;
                TxDaten.Ack=1;										//  senden
            }
            if (RxDaten.Cmd==0x81)			// Stromsollwert angekommen
#else
                    /*SEND_ALIVE signal kommt an*/
                    if(RxDaten.Cmd==SEND_ALIVE)
                    {
                        g_c_SEND_ALIVE_SIGNAL = AKTIV;
                        g_c_SEND_ALIVE_STATE  = SEND_ALIVE_SEND_PAUSE_STATE;

                        Alive_timeout_cnt = 0;

                        TxDaten.Cmd       = SEND_ALIVE;                                                 // kommand zum Senden
                        TxDaten.Wert      = (ALIVE | Page);                                            // was zu senden
                        if(RxDaten.Wert==0x05AA)                                                // nur senden wenn von SST passt
                        {
                            TxDaten.Ack   = 1;                                                      // nur Alive baantworten
                        }

                        Senden4Byte();
                    }
            
            
                    /*##RAVI: OK but not quite sure if it is here sent TEST it*/
                    if (RxDaten.Cmd==LT_ONOFF)                               // LT_ONOFF CMD , ST waits for ACK with in 1 msec 
                    {
                        P5_Daten[ID_P5_LT_ONOFF]=RxDaten.Wert;				// speichern in die Matrix
                        OnOff=RxDaten.Wert;									// speichern in die Variable

                        if(OnOff == 0)
                        {
                            block_Isoll = 0;
                            Ignition_time_cnt1 = 0;
                        }
                        /// Due to ACK fehler during pressing of brenner push button, UART sends ack immediately// with normal uart subroutine we have issue of  ACK fehler 
                        U1TXREG=0x05;                                   //CMD NR: CMD ACK
                        U1TXREG=0x01;                                   // UART_HI:ACK TYP 1 data received
                        U1TXREG=0xA0;                                   // UART_LO:CMD NR of the received data
                        U1TXREG=0x05+0x01+0xA0;                         // checksum
                    }               
                
            if (RxDaten.Cmd==SOLL_STROM_I1 && Page==0x500)			// Stromsollwert angekommen
#endif			
				{ 	
				if((RxDaten.Wert & 0x03FF)<850)						// nur wenn Strom kleiner, uebernehmen
				  {
					Iecht=P5_Daten[ID_P5_SOLL_STROM_I1]=(RxDaten.Wert &0x03ff);		// in der Matrix speichern

					if(RxDaten.Wert & 0x4000) Rampe=1;						// 0-> Rampe (standard) 1-> keine
					else Rampe=0;
                    // ab da *4 bei EP
                    if(Iecht > Imax_setpoint_max)
                    {
                        Iecht = Imax_setpoint_max;
                    }
                    
                    if(block_Isoll == 1)
                    {
                        if(Iecht <= Imax_setpoint_nominal_A)
                        {
                            if(enable_Iblock_cnt == 0)
                            {
                                block_Isoll = 0;
                                Ignition_time_cnt1 = 0;
                            }                  
                        }
                        else
                        {
                            Iecht = Isoll>>2;
                        }
                    }                 
                    Iecht=Iecht<<2;             // mal 4 , 12bit
                    g_u16i_Iecht_temp = Iecht;

					
						if(Iecht-8>(Isoll_alt) ) {Istep=(Iecht-Isoll_alt)>>3; }	// 8 Schritte	Polaritaet nach oben
						if(Iecht+8<(Isoll_alt) ) {Istep=(Isoll_alt-Iecht)>>3;	}	// 8 Schritte	Polaritaet nach unten
						
                        if(Istep<1) Istep=1;
                       // Trepp=0;
                                              
//                    else
//                    {
//                        Trepp=8;
//                    }
					if(Iecht<12)                                                //##12 
                    {
						OnOff_Strom=0;
                    }
					else 
                    {	
						OnOff_Strom=1;											// ab 5 A Leistung einschalten
						Isoll=Iecht;                                            // bei 840 1:1
                    }			
					Iecht_alt=Iecht;	
                    VALID_one=1;
				  }
				  else OnOff_Strom=0;                                           // falscher Strom, abschalten!!  	
                  Alive_timeout_cnt = 0;
                  
				}
                else if(RxDaten.Cmd == AC_STATUS && Page==0x500)
                {
                  
                    P5_Daten[ID_P5_AC_STATUS] = (RxDaten.Wert & 0xffff);
                    if (RxDaten.Wert==1)
                    {
                        g_u16i_AC_STATUS          = AKTIV;
                          //  _RB8 = 1 ;
                    }
                    else  
                    {
                        g_u16i_AC_STATUS          = INAKTIV;
                           // _RB8 = 0 ;
                    }
                    Alive_timeout_cnt = 0;
                }
                else if (RxDaten.Cmd==SOLL_STROM_I2 && Page==0x500)                 // Stromsollwert angekommen
                { 	
                    if((RxDaten.Wert & 0x03FF)<100)                                 // nur wenn Strom kleiner, uebernehmen
                    {
                        aux_i2 =(RxDaten.Wert &0x03ff);	
                        I2_UART = (aux_i2 << 2);
                        P5_Daten[ID_P5_SOLL_STROM_I2] = I2_UART;
                    }		
                    Alive_timeout_cnt = 0;
                }
                else if((RxDaten.Cmd == STROM_MODE) && (Page == 0x500))             // Strom mode angekommen
                {               
                    P5_Daten[ID_P5_STROM_MODE] = RxDaten.Wert;                      // Received data stored in the matrix
                    KOMMANDO_RCV               = RxDaten.Wert;                      // 

                    switch(KOMMANDO_RCV)
                    {
                        case (HYPERPULSE_INAKTIV):
                        {
                            HYPER_PULSE_MODUS = 0;

                            break;
                        }
                        case (HYPERPULSE_AKTIV):
                        {
                            HYPER_PULSE_MODUS = 1;

                            break;
                        }
                    }  
                   Alive_timeout_cnt = 0;
                }
                else if((RxDaten.Cmd == PULS_STROM_FAKTOR) && (Page == 0x500))      // Puls Strom Faktor angekommen
                {  
                    PULSE_STROM_FAKTOR_BIT            = (RxDaten.Wert & 0x8000);    // Bit 15 extraction
                    PULSE_STROM_FAKTOR_BIT            = PULSE_STROM_FAKTOR_BIT>>15; // Bit 15 shifted to Bit 0
                    PULSE_STROM_FAKTOR_VALUE          = (RxDaten.Wert & 0x7FFF);  
                    P5_Daten[ID_P5_PULS_STROM_FAKTOR] = (RxDaten.Wert &0x7fff); 
                    
                  Alive_timeout_cnt = 0;
              
                }
                
           //////////////////////////////  Receive sequence of T1 and T2 ///////////////////////////////////////////
            
            /*
             PULSE_ZEIT_T1_LOW_HOLD          // holds lower 16 bits t1 value in usec later converted to counter period of PWM special event(interrupt at 14,2 usec) 
             PULSE_ZEIT_T1_HIGH_HOLD         // holds higher 16 bits t1 value in usec
             PULSE_ZEIT_T2_LOW_HOLD         // holds lower 16 bits t2 value in usec
             PULSE_ZEIT_T2_HIGH_HOLD        // holds higher 16 bits t2 value in usec
              t1_t2_full_rx                 // when 1 , t1 and t2  have already arrived // when 0 receiving  new t1 and t2 time    
             */
            
             else if((RxDaten.Cmd == PULSE_ZEIT_T1_LOW) && (Page == 0x500))         
                {                
                    P5_Daten[ID_P5_PULSE_ZEIT_T1_LOW] = (RxDaten.Wert & 0xffff);
                    PULSE_ZEIT_T1_LOW_HOLD                = (RxDaten.Wert & 0x0000ffff);  
                    t1_t2_full_rx=0;
                    Alive_timeout_cnt = 0;
                }
            
                else if((RxDaten.Cmd == PULSE_ZEIT_T1_HIGH) && (Page == 0x500))          
                {                
                    P5_Daten[ID_P5_PULSE_ZEIT_T1_HIGH] = (RxDaten.Wert & 0xffff);
                    
                    PULSE_ZEIT_T1_HIGH_HOLD    =(RxDaten.Wert & 0x0000ffff);
                   
                    Alive_timeout_cnt = 0;
                }
            
            
             else if((RxDaten.Cmd == PULSE_ZEIT_T2_LOW) && (Page == 0x500))         
                {                
                    P5_Daten[PULSE_ZEIT_T2_LOW] = (RxDaten.Wert & 0xffff);
                    PULSE_ZEIT_T2_LOW_HOLD                = (RxDaten.Wert & 0x0000ffff); 
                    Alive_timeout_cnt = 0;
                }
            
                else if((RxDaten.Cmd == PULSE_ZEIT_T2_HIGH) && (Page == 0x500))         
                {                
                    P5_Daten[ID_P5_PULSE_ZEIT_T2_HIGH] = (RxDaten.Wert & 0xffff);
                    PULSE_ZEIT_T2_HIGH_HOLD    =(RxDaten.Wert & 0x0000ffff);
                    t1_t2_full_rx=1;
                    Alive_timeout_cnt = 0;
                }
            
            
            else if ((RxDaten.Cmd == WIG_ZUND_MODE) && (Page == 0x500))         
                {                
                    P5_Daten[ID_P5_WIG_ZUND_MODE] = RxDaten.Wert ;
                        if (RxDaten.Wert==0)
                        {
                            g_u16i_Wig_zundung_Mode_Lift_arc       =1;
                        }
                    else 
                    {
                        g_u16i_Wig_zundung_Mode_Lift_arc=0;
                    }
                    Alive_timeout_cnt = 0;
                }
            
            
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                  
            
                else
                {	Flag_UART=done; 	}                                           // keine Stromdaten, weiter bearbeiten
                    Alive_timeout_cnt = 0;
                }
		}
	else
	{
		UARTinit();
	}    
IFS0bits.U1RXIF=0;							// Interrupt Rx Flag loeschen	
}	

void __attribute__((__interrupt__, no_auto_psv)) _ISR _CNInterrupt(void)		// On change interrupt Nulldurchgang erkennung	
{
    if (( Aux == EIN)&&(g_u16i_AC_STATUS == AKTIV))
    {
        Umpolung_Aktiv    = AKTIV;
        IOCON1bits.OVRENH = 1;                                                // enable PWM1H over write; PWM1H over write value is 0 configured by IOCON1bits.OVRDAT = 0;
        __builtin_nop();
        IOCON2bits.OVRENL = 1;
        Komparator_3      = AUS;
        Komparator_4      = AUS;
        IEC1bits.CNIE=0;                                                      // ON CHANGE iNTERRUPT DISABLED // ENABLED IN PWM SPECIAL EVENT INTERRUPT
        CNENBbits.CNIEB5=0;     
    }
    IFS1bits.CNIF=0;   
}
        