/**********************************************************************
* FileName:        840Haupt_P.c
* Processor:       dsPIC33EP23GS502
***********************************************************************/
// TCY 64MHz

// 25.06.2018
// Version 501 fuer EPxxGS502, nur fuer 840d/e mit Umbau pin 9 gegen pin 5 !!!!!!!!!!!!!!!!
//basiert auf V501 von EPxxGS202 mit 2 Komparatoren als Symmetrie. Fuer Prozessor 502
// Komparatoren 3 u 4 zum Stromregeln, Komparator 2 fuer Imax. Komparator 1 koennte Uist auswerten, noch nicht aktiv.
// 24V, 15V Sendungen neue Nummer
// Treppen richtig rechnen.
// regler "Integral 2 mal schneller. /4 .weniger Nachschwinger.
// Version 502
// ADC Initialiesierung als nicht Synchron, scannt besser und wartet nicht auf Perioden.
// Version 503 fuer EP32GS502 am 11.07.2018
// Version 05.03.2025 Mr. K asks to limit the current to 500 A. So Imax-setpoint1 and  Imax_setpoint_max are limited to 500A!


#include "p33EP32GS502.h"
#include "6900747_HeaderFile.h"
#include "math.h"

//////////////////////////////Routine e Prototyp////////////////////////////////
void InitOsc(void);
void Konfig (void);
void Start(void);
void COMP_DAC (void);
void InitIO (void);
void Init_ADC(void);
void Ausgleichen(void);							// INV_Enable steuern, QuelleStromwertn -> EIN(opto), Stromwert<3A, Befehl on/off
void PWM_Konfig(void);
void Alles_Aus(void);
void InitDaten(void);							// Daten vom LT initialisieren

void Spannungsbegrenzer(void);
//void Send_Last_Fault(unsigned int reg);

volatile unsigned int INV_Enable;				// Freigabe vom PWM (Inverter) -> Leistung ein
volatile unsigned int Freigabe; 				// nach Check freigeben wenn ok
volatile unsigned int MaxAnzahl; 				// Zaehler fuer Uzkmax
volatile unsigned int UmaxTimer; 				// Uzkmax suchen, um 100ms lang
volatile unsigned int Uzkmax; 					// Netzspannung ueber 100ms gemittelt
volatile unsigned int UzkmaxSum; 				// Netzspannung ueber 100ms gemittelt alle 100 Werte
volatile unsigned int ueberstrom=0;				// Zaehler , bei 1 mal LT aus. Reset mit Signal EIN=1 ( heisst "aus")

volatile unsigned int Ut=5000;					// Uampl*t1
volatile unsigned int IL3=32;					// (IL1+IL2)²/256
volatile unsigned int K1=50;					// Ut/t2
volatile unsigned int K=230;					// 128/IL3*K1
volatile unsigned int Rmess=20;					// SchweisskreisKurzschlussWiderstand in mOhm
volatile unsigned int IL11;						// IL1 verrechnet um Ianf
volatile unsigned int UZK_loaded = 0;
volatile unsigned int start_counter = 0;
volatile unsigned int error_reg1_read=0;
volatile unsigned int error_reg2_read=0;
volatile unsigned int error_reg1_write=0;
volatile unsigned int error_reg2_write=0;
volatile unsigned int actual_P = 0;
volatile unsigned int actual_Pintegral = 0;
volatile unsigned int enable_Iblock_cnt = 0;
//	8 BIT unsigned
volatile unsigned char Inhib=1;					// LT Sperre bei Unsymmetrie, Ueberspannung ....
volatile unsigned int Failure_code=0;
volatile unsigned int full_Uzks = 0;
volatile unsigned int uzk_empty_cnt = 0;
volatile unsigned int timer_ref = 0;
volatile unsigned int Isoll_aux = 0;
volatile unsigned int LAST_PULSE_BALANCE;
volatile unsigned long LAST_PULSE_FREQUENZ; 
volatile unsigned int block_calculation=0;
volatile unsigned int block_start=0;	
volatile unsigned long I1_Time_hold;
volatile unsigned long I2_Time_hold;
volatile unsigned long Time_uart_send;

volatile unsigned long I1_Time_hold1_LAST;
volatile unsigned long I2_Time_hold2_LAST;

volatile unsigned long I1_Time_samp;
volatile unsigned long I2_Time_samp;



volatile unsigned long I1_Time_hold1;
volatile unsigned long I2_Time_hold2;
//const unsigned short boot1 __attribute__((address(0x9fe)))=0x1234; //
//const unsigned short boot2 __attribute__((address(0x2bfe)))=0x5678; //
const unsigned int Error_register1 __attribute__((space(prog),address(0x2B00)))=0x0000;
const unsigned int Error_register2 __attribute__((space(prog),address(0x2B10)))=0x0000;

#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) )
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)


#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration bit (Allow only one reconfiguration)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)
#pragma config PLLKEN = ON              // PLL Lock Enable Bit (Clock switch to PLL source will wait until the PLL lock signal is valid)

#pragma config PWMLOCK = 0              // kein PWM schutz

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config WDTEN = OFF              // Watchdog Timer Enable bits (WDT and SWDTEN disabled)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)

// FICD
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED2)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
//#pragma config BTSWP = OFF              // BOOTSWP Instruction Enable/Disable bit (BOOTSWP instruction is disabled)

//_FPOR(FPWRT_PWR128)							// Power ON 128ms
//_FICD(ICS_PGD2& JTAGEN_OFF)

int main (void)
{
	t22=(__builtin_divud(__builtin_muluu(PWM_P,106),100))>>8;               // Periode/256=54ns?

    InitOsc();								// Oszillatoren Initialisierung
	Konfig();								// Konfig Ports, Timer ...
	COMP_DAC();								// Konfig Komparatoren und DAC
	InitIO();								// Virtual pin verbinden
	Init_ADC();								// ADC Init
	InitDaten();							// initialisieren der Daten
	SPI_Konfig_Master();
	PWM_Konfig();
    //Read_NMV();
    //Send_Last_Fault(error_reg1_read);
/*#ifdef _Enable_OC
    InitOC();
#endif*/
    
	IEC0bits.T1IE=1;						// Enable Timer1 Frame UART
	IEC0bits.T3IE=1;						// Start T3 Interrupt fuer Zeit-Zaehler
	IEC0bits.T2IE=1;						// Enable Timer2 , Synchro?
    IEC1bits.CNIE=1;
    CNENBbits.CNIEB5=1;
	UARTinit();
	Alles_Aus();							// Verwendet als Startfunktion
    
 	RF=0;									// SW Ausgang offen. Strom - Messung
   	Fehler.bits.SPN=0;

	Komparator_2 = Imax3PH;                 // Abschaltgrenze fuer Strom setzen, hier nicht mehr

	//LED_F=1;								// noch Fehler
	Freigabe=1;								// noch zu machen!!!! dass Uart einschaltet die Freigabe	  
	TxDaten.Ack=5;                          // Sendungen taetigen
    Delay_ms(1);
    g_c8_TX_PENDING                       = INAKTIV;
    ODCB		= 0B0000000000000000;			// Port B  kein o.D
	ODCA		= 0B0000000000001000;			// SW als OpenDrain A3
    //_RB4 = 1;
#ifndef _Probe 
	while (Freigabe==0)						// Freigabe -Wartenzeit ueber USART
	    {	Kommunikation();}
#endif  //nProbe
// ******************************************************** Hauptschleife **********************************************************	
  while(1)
 {   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
      
   T2CONbits.TON = 1;   
	if (U1STAbits.RIDLE==0 && T1CONbits.TON==0   )             				// pruefen ob Rx da, dann Zeit starten
		{ 
		T1CONbits.TON=1;                                                    // Messung der Zeit in einer Sendung, wenn alle 4 bytes nicht rechzeitig kommen ->Fehler
		}
    
    
    ////////////////////////////////////// 32 bit I1 and I2 time from low and high 16 bits///////////////////////////////////////////////////
                        /*
                         I1_Time_hold1                  /// 32 BIT I1_Time value in usec  
                         I2_Time_hold2                  /// 32 BIT I2_Time  value in usec 
                         Totaltime                      /// 32 BIT Total_Time  Max 10000000 Min 57(i.e 0,1 Hz to 17,5 KHz)
                         PULSE_ZEIT_T1_LOW_HOLD         /// 16 bit I1 time low bits // set by uart1 rx interrupt while receiving new t1 and t2 
                         PULSE_ZEIT_T1_HIGH_HOLD        /// 16 bit I1 time high bits // set by uart1 rx interrupt while receiving new t1 and t2 
                         PULSE_ZEIT_T2_HIGH_HOLD        /// 16 bit I2 time high bits // set by uart1 rx interrupt while receiving new t1 and t2 
                         PULSE_ZEIT_T2_LOW_HOLD         /// 16 bit I2 time high bits // set by uart1 rx interrupt while receiving new t1 and t2 
                         t1_t2_full_rx                  /// 1 when all above mentioned uart variable are set(i.e PULSE_ZEIT_T1_LOW_HOLD ....to.....PULSE_ZEIT_T2_LOW_HOLD), 0 when new uart variable are receiving(i.e PULSE_ZEIT_T1_LOW_HOLD ....to.....PULSE_ZEIT_T2_LOW_HOLD)
                        */
    
    if (t1_t2_full_rx==1){
     I1_Time_hold1=(PULSE_ZEIT_T1_HIGH_HOLD<<16)|PULSE_ZEIT_T1_LOW_HOLD; // concatinates   PULSE_ZEIT_T1_HIGH_HOLD and PULSE_ZEIT_T1_LOW_HOLD
     I2_Time_hold2=(PULSE_ZEIT_T2_HIGH_HOLD<<16)|PULSE_ZEIT_T2_LOW_HOLD; 
     Totaltime=I1_Time_hold1+I2_Time_hold2;
     t1_t2_full_rx=0;
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    ///////////////////////////////////// Calculate I1_Time & I2_Time ///////////////////////////////////////////////////////////////////////
                         
                           /*
                             I1_Time_samp                        // holds 32 bit I1_Time value =  I1_Time_hold1/14
                             I2_Time_samp                        // holds 32 bit I2_Time value =  I2_Time_hold2/14
                             I1_Time_hold1_LAST             // T1 change detector variable
                             I1_Time_hold1_LAST             // T2 change detector variable
                            
                             
                            if condition only valid when change in frequency (I1_Time_hold1 and I2_Time_hold2) detected
                            */
    
   if ((((I2_Time_hold2!=I2_Time_hold2_LAST))))
     { 
       
        if ((Totaltime>=57)&&(Totaltime<=250))
            {
    
        if (Totaltime==57)
        {
        
        I1_Time_samp=2;

        I2_Time_samp=2;
        
        }      //17,5k
        else if (Totaltime==71)
        {
        
        I1_Time_samp=2;

        I2_Time_samp=3;
        
        }   //14k
        else if (Totaltime==83)
        {
        I1_Time_samp=3;

        I2_Time_samp=3;
        
        } //12k
        else if (Totaltime==100)
        
        {
        I1_Time_samp=3;

        I2_Time_samp=4;
        
        }   //10k
        else if (Totaltime==111)
        {
        
        I1_Time_samp=4;

        I2_Time_samp=4;
        
        }   //9k
        else if (Totaltime==125)
        {
        
        I1_Time_samp=4;

        I2_Time_samp=5;
        }   //8k
        else if (Totaltime==142)
        {
        I1_Time_samp=5;

        I2_Time_samp=5;
        
        }   //7k
        else if (Totaltime==166)
        {
        
        I1_Time_samp=6;

        I2_Time_samp=5;
        }   //6k
        else if (Totaltime==200)
        {
        I1_Time_samp=7;

        I2_Time_samp=7;
        
        }   //5k
        else if (Totaltime==250)
        {
        
        I1_Time_samp=9;

        I2_Time_samp=8;
        }   //4k    
        
    
    }    
       
else if (Totaltime>250)
    
{
I1_Time_samp=(I1_Time_hold1>>7)+(I1_Time_hold1>>4);

I2_Time_samp=(I2_Time_hold2>>7)+(I2_Time_hold2>>4);
}
I1_Time_hold1_LAST=I1_Time_hold1;

I2_Time_hold2_LAST=I2_Time_hold2;


}  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  
  
switch(PULSE_STROM_FAKTOR_BIT)
                    {
                        case(AKTIV):
                                                 // Strom Pulse I1 <= Strom Pulse I2   
                        {      
                            if((m_u16i_Iecht_Check_I2 != g_u16i_Iecht_temp)||(PULSE_STROM_FAKTOR_VALUE!=m_u16i_STROM_FAKTOR_CHECK))
                            {  
                                 
                                PULSE_STROM_I2_hold = g_u16i_Iecht_temp;
                                PULSE_STROM_I1_hold = __builtin_muluu(PULSE_STROM_I2_hold,PULSE_STROM_FAKTOR_VALUE);
                                PULSE_STROM_I1_hold = PULSE_STROM_I1_hold + ROUND_FAKTOR;
                                PULSE_STROM_I1_hold = __builtin_divud(PULSE_STROM_I1_hold,NORM);
                               
                                    PULSE_STROM_I1=PULSE_STROM_I1_hold;
                                    PULSE_STROM_I2= PULSE_STROM_I2_hold;
                                    m_u16i_Iecht_Check_I2 = g_u16i_Iecht_temp;
                                    m_u16i_Iecht_Check=0;
                              
                                    m_u16i_STROM_FAKTOR_CHECK=PULSE_STROM_FAKTOR_VALUE;

                            
                            }
                            break;
                        }
                        case(INAKTIV):
                        {                            
                             if((m_u16i_Iecht_Check != g_u16i_Iecht_temp)||(PULSE_STROM_FAKTOR_VALUE!=m_u16i_STROM_FAKTOR_CHECK))
                            {
                                 
                                PULSE_STROM_I1_hold=g_u16i_Iecht_temp;
                                PULSE_STROM_I2_hold=__builtin_muluu(PULSE_STROM_I1_hold,PULSE_STROM_FAKTOR_VALUE);
                                PULSE_STROM_I2_hold= PULSE_STROM_I2_hold + ROUND_FAKTOR;
                                PULSE_STROM_I2_hold=__builtin_divud(PULSE_STROM_I2_hold,NORM);
                                
                                
                                    PULSE_STROM_I1=PULSE_STROM_I1_hold;
                                    PULSE_STROM_I2= PULSE_STROM_I2_hold;
                                    m_u16i_Iecht_Check = g_u16i_Iecht_temp;  
                                    
                                     m_u16i_Iecht_Check_I2=0; 
                                     m_u16i_STROM_FAKTOR_CHECK=PULSE_STROM_FAKTOR_VALUE;

                            
                            }
                             
                             
                            break;
                        }
                    }    
    
  
    
    
    if(IFS0bits.T1IF) { IFS0bits.T1IF=0; TMR1=0; } 
    
	if(!INV_Enable || P5_Daten[ID_P5_IST_STROM]==0)  LaufTimer=0;

    //#ifdef _Probe												// Tests ohne und mit Netz ( von Hand hochfahren!!!)

	if(Timer_cut_off>2000)									// mehr als 2 Sek. kein UART, nur Handbetrieb vom Poti
		{
        Failure_code=16;
		Timer_cut_off=2001;									// Zeit halten
		P5_Daten[ID_P5_LT_ONOFF]=0;							// abschalten!!
//		OnOff=0; OnOff_Strom=0;


		//##if((Sollwert_Hardware<100) || Aux==0 ) { OnOff=0; OnOff_Strom=0; ueberstrom=0;  }		// bei kleinen Werten abschalten ewtl. Ueberstrom reseten. Fuer ED-Test

		/*if (Aux==1 && (Sollwert_Hardware>140))									// kleine Hysterese +5 bits, Aux ist low - aktiv
			  { OnOff=1; OnOff_Strom=1; Isoll=Sollwert_Hardware; }*/			// vom Poti gesteuert, ueber Aux ein/aus



		}
//#endif	 // Probe

	SPI_Kom_Master(PFC_Kommando);			//  SPI senden

// UZK Mittelwert ueber beide Haelften
	if(Uzk_mid1>10 || Uzk_mid2>10)								//  aber nicht kleiner als 10!! , wegen Rechnung
	UzkAvr=((Uzk_mid1+Uzk_mid2)>>1);
	else 
	UzkAvr=10;

	//Ilm=__builtin_divud(PWMH,1000);								// vereifachte Rechnung vom Ilm (0,5us)

// Berechnung der Variablen fuer L-Messung 
if(lrechnen) Lrechnen();

Uampl=__builtin_divud(__builtin_muluu(UzkAvr,96),Ue1);		// in 0,1V Schritten, Anpassung zu echt deswegen 92. PWMH in Schritten

#ifndef KPWM
 Spannungsbegrenzer();											// Uaus Regler, (um 0,8us)
#endif

 //Kommunikation();	// Uart, (zwischen 0,4 us)
 if(baud_change_needed ==1 && PWM_counter >= 20)
{
    Init_UART();   
    uart_change_start_done = 0;
    baud_change_needed = 0;
    PWM_counter = 0;
}
 
 
 Alles_Aus();
 FSym();														// Symmetrie pruefen (0,4us)
//root start
 if(trigger_root == 1)
{
    //LED_B = 1;
    sum_old_Iist = sum_old_Iist + Ist_wert - sum_new_Iist;
    sum_new_Iist = __builtin_divud(sum_old_Iist,3); 

    sum_old_uist = sum_old_uist + Uist - uist_filtered;
    uist_filtered = __builtin_divud(sum_old_uist,3);    

    if(sum_new_Iist > 0)
    {
        new_R = __builtin_divud(__builtin_mulus(uist_filtered,1000),sum_new_Iist);     
    }   
    trigger_root = 0;
 }
//LED_B = 0;
//root end 

// Test der Symmetrie
	if (AB_Sym > _Sym_Toll_p || AB_Sym < _Sym_Toll_m )
		{ Inhib=done; Fehler.bits.SYM=1;
            if(one_time_failure == 0)
            {
                one_time_failure = 1;
                Failure_code=11;
            }
        }						// sperren, Symmetrie ausser Tol.


 
// Test der Spannungen
#ifndef _Probe
	if (Uzk_mid1>BULK_MAX)	{Inhib=done; Fehler.bits.UZK=1;
        if(one_time_failure == 0)
            {
                one_time_failure = 1;
                Failure_code=12;
            }
       
        }	// sperren, Uzk nicht ok
	if (Uzk_mid2>BULK_MAX)	{Inhib=done; Fehler.bits.UZK=1;
        if(one_time_failure == 0)
            {
                one_time_failure = 1;
                Failure_code=13;
            }
        }	// sperren, Uzk nicht ok
        
	if ((Uzk>Uzk_max_3Ph))				// Falsche Spannung im Netz!
		{Inhib=done; Fehler.bits.UZK=1;
        if(one_time_failure == 0)
            {
                one_time_failure = 1;
                Failure_code=14;
            }
        }
	if (Uzk<(Uzk_min_3Ph))				// Falsche Spannung im Netz!
		{Inhib=done; Fehler.bits.UZK=1;
        if(one_time_failure == 0)
            {
                one_time_failure = 1;
                Failure_code=15;
            }
        }
#ifndef _No_UZKmin_check
 if(timer_ref != Timer)
 {
	if(INV_Enable == 1)
    {
        full_Uzks = Uzk_mid1 + Uzk_mid2;        
        if(full_Uzks < Uzk)
        {
            uzk_empty_cnt++;
            if(uzk_empty_cnt >= 20)
            {
                enable_Iblock_cnt = 1;
                block_Isoll = 1; 
                if(Isoll > (Imax_setpoint_nominal_A<<2))
                {
                    Isoll_aux = Isoll;
                    Isoll=__builtin_divud(__builtin_mulus(Isoll_aux,85),100);
                }
            }            
        }
        else
        {
            if(uzk_empty_cnt > 0)
            {
                uzk_empty_cnt--;
            }            
        }
    }
    else
    {
        uzk_empty_cnt = 0;
    }
    timer_ref = Timer;
 }
#endif
// Uzk max rechnen ueber 64 Messungen
  	if(Timer!=UmaxTimer)
   	{
		UmaxTimer=Timer;
		UzkmaxSum+=P5_Daten[ID_P5_NETZSPANNUNG];
		MaxAnzahl++;
		if(MaxAnzahl==64)
			{
			Uzkmax=UzkmaxSum>>6;
			MaxAnzahl=0;
			UzkmaxSum=0;
			}
   	}
#endif	// n_Probe
	if(INV_Enable) LED_B=1;	
    else LED_B=0;	

  }	// Ende while(1)
}	// Ende Main 
// *************************************************************** main ende ***********************************************
void Alles_Aus(void)							// Starten, Ueberwachen, Relais..
{
	Ausgleichen();								// INV_Enable steuern
	Start();
}
//
void Ausgleichen(void)							// Inverter ein/auschalten
{
    
	if(( OnOff==0 || OnOff_Strom==0 || Inhib  || UZK_loaded == 0))
		{
		if(OnOff==0 || Inhib  )					// PFC nicht bei Strom ==0 abschalten
        {
			PFC_Kommando=_PFC_Aus;          	// sollte PFC da sein
        }

//		PDC1=0;//DTR1;
//		SDC2=0;
		INV_Enable=0; 
		Uzkmax=0;
		//ueberstrom=0;	// loeschen bei extern aus 	
        PFC_UZK_cnt = 0;
		}
   //SPI_mirror[9]=((OnOff)|(OnOff_Strom<<1)|(Inhib<<2)|(ueberstrom<<3)|(INV_Enable<<4)|(UZK_loaded<<5));
	if((INV_Enable==0) &&  OnOff && OnOff_Strom && !Inhib  && (!ueberstrom)) 
		{
#ifdef _istPFC
#ifndef ausPFC  
        if (P5_Daten[ID_P5_PFC]& 2)             // auch PFC ein
#else 
          ;
#endif
#endif
           {
          if(UZK_loaded == 1 && INV_Enable == 0)
            {
                INV_Enable=1; 							// Start Signal, INV_Enable bekommt 1 im interrupt, wo die neue Breite schon bestimmt ist.	
                PFC_UZK_cnt = 0;
            }
         
            PFC_Kommando=_PFC_Ein;        			 // sollte PFC da sein
          }
        else
        {
            PFC_UZK_cnt = 0;
        }
    }
    else
    {
        PFC_UZK_cnt = 0;
    }
	if( OnOff==0 ) 
    {
        ueberstrom=0;				// Merker loeschen wenn von extern aus
        PFC_UZK_cnt = 0;
    }
}
//
void Kommunikation()							// UART  bei PFC
{ 
    if (OnOff==0)                      // URAT DATA Rx during non-welding phase
        {
            UART_Manangement();
        }
	
	if (ProtokollTimer>_SendTimer)				// jede 42us, also 3 * 14us, zum 
		{ 
		ProtokollTimer=0;
		Protokoll();							//  wird eine Sendung gemacht
		}
}
//
void Spannungsbegrenzer(void)
{
	unsigned int DutyC;
    switch (g_u16i_Elektrode_Aktiv)
    {
        case AKTIV:
            /***************************PDC1 will be determined in special event interrupt****************************/
            break;
        case INAKTIV:
            DutyC=((__builtin_divud(__builtin_muluu(P5_Daten[ID_P5_SOLL_SPG_U1],PTPER),Uampl))>>1)+DLsv;	//DTR1+6000;	
            
            if (DutyC>PWM_D)
            {
                DutyC=PWM_D;
            }
           /***********FOR LIFT ARC start phase to avoid the high impulse current ******************/
            if ((g_u16i_Pdc_first_time_CM==0)&&(g_u16i_AC_STATUS==0))
            {
                    pdc1 = 4000;                    //1000 best
                        if (g_u16i_Wig_zundung_Mode_Lift_arc==1)
                        {
                                pdc1 = 1000;                    //1000 best
                        }
            }
            
            if(g_u16i_AC_STATUS==1)
            {
                 pdc1=DutyC;
            }
            break;
    }
    
}
//
void Delay_ms (unsigned int Mx)
{ unsigned long  i=0,j, k=6000;
for(j=0;j<Mx;j++)
  {
    while(i<k){ i++;}
   j++; i=0;
}
}
//
void Lrechnen(void)
	{ 
	Ut=__builtin_muluu(Uampl,t1);								// Hilfswert
	//	Ut=Uampl*t1;
	if(IL1>Ianf) IL11=IL1-Ianf;
		else IL11=1;												// wegen Uebelauf der Folgerechnung

	Lmess=(Ut/IL11)>>3;											// (Ut/IL1)/8

	P5_Daten[ID_P5_INDUKTIVITAET] =Lmess;

	K1=Ut/t2;													// Faktor K1=Ut/t2 fuer die Berechnung
	IL3=((IL1+IL2)*(IL1+IL2))>>5;								// Hilfswert (IL1+IL2)²/32
	//IL3=__builtin_muluu((IL1+IL2),(IL1+IL2))>>8;
	K=(K1<<7)/IL3;
	Rmess=(K*IL1)-(K*IL2/IL1*IL2);									// (K*IL1-K*(IL2²/IL1))/4
	//	Rmess=__builtin_muluu(K,IL1)-__builtin_muluu(__builtin_divud(__builtin_muluu(K,IL2),IL1),IL2);
	//Rmess=__builtin_muluu(K1,IL1)-__builtin_muluu(__builtin_divud(__builtin_muluu(K,IL2),IL1),IL2);
	Rmess=Rmess>>2;
	P5_Daten[ID_P5_WIDERSTAND] =Rmess;
	lrechnen=0; 
	}

void Read_NMV(void)
{
    volatile unsigned int dummy_read = 0;
    
    error_reg1_read=__builtin_tblrdl(0x2B00);										// read 16bit Low of 24
    dummy_read=__builtin_tblrdh(0x2B00);										// read 8bit Hi of 24
    asm("nop");																			// Standard bedarf
    asm("nop");																			// Standard bedarf
    /*error_reg2_read=__builtin_tblrdl(0x2B10);										// read 16bit Low of 24
    dummy_read=__builtin_tblrdh(0x2B10);										// read 8bit Hi of 24
    asm("nop");																			// Standard bedarf
    asm("nop");																			// Standard bedarf    
    error_reg2_write=error_reg2_read;
    error_reg1_write=error_reg1_read;*/
}

