#include "p33EP32GS502.h"
#include "6900747_HeaderFile.h"
#include "math.h"
//#include "stdlib.h"

volatile unsigned int H_W_Er=0xFFFF;
volatile unsigned int Timer;				// Haupttimer in ms

FEHLER Fehler;								// structur fuer Fehlermeldungen 

volatile unsigned int NTC_On_Grenze;
volatile unsigned int NTC_Off_Grenze;
volatile signed int AB_Sym;                 // Absolutwert der Symmetrieabweichung
volatile unsigned int UzkAnLauf;			// Spannung hinter dem Relais im Anlauf
volatile unsigned int FSymFlag;				// Symmetrie auswerten Flag
volatile unsigned int TimeAnlauf;
volatile unsigned int Time_Summe;			// Timer
volatile unsigned int PFC_Kommando;			// an PFC Befehle
volatile unsigned int Periode_tmp;
volatile unsigned int Relais=0;
volatile unsigned int TimerRelais;			// 1 Sek Wartezeit nach ok.

volatile unsigned int init_stored_1=0;
volatile unsigned int init_stored_2=0;
volatile unsigned int init_stored_3=0;
volatile unsigned int init_stored_4=0;
volatile unsigned int init_stored_5=0;

volatile unsigned int Fault_pwm_code=0;
volatile unsigned int Fault_saved=0;
volatile unsigned int Timer_LEDgelb = 0;
volatile unsigned int one_time_failure = 0;

void Send_Last_Fault(unsigned int reg);
void Write_NMV();
volatile long LongSym;
volatile signed int Sym,Sym2,Sym3,Sym4,Sym5;				// Spannungsdifferenz von beiden Zwischenkreisen
void FSym(void);
unsigned int FehlerAnzeigen_LED_Booster(unsigned int Entry);

/******************************************************************************/
  ///////////////////////*Start Prozess, check und Anlauf */////////////////////
////////////// 		 *Return Fehler.Val = 0 wenn alles ok      *//////////////////
/******************************************************************************/

void Start(void)
{ 
//*********************** Uzk Auswertung *************

	if((P1_Daten[ID_PFC_EXIST]==0) || (P1_Daten[ID_PFC_EXIST]==0xFFFF))
		{ Fehler.bits.PFC=1;   
          Failure_code=1;
        }
	else
		{ Fehler.bits.PFC=0; }

// ***************************** Ueberpruefung der Feheler jede 1 ms ************************
 if( Periode_tmp!=Timer )
 { 
	Periode_tmp=Timer; 				// fuer neue Zeit

// Versorgung 15V und 24V ueberwachen	
	if ((P5_Daten[ID_P5_SPG_15V_1]<(V15Soll+V15Toll)) && (P5_Daten[ID_P5_SPG_15V_1]>(V15Soll-V15Toll))) 
		{ Fehler.bits.VERS=0; }
	else 
		{Fehler.bits.VERS=1;  
             Failure_code=2;
        }

	if ((U24<(V24Soll+V24Tollp)) && (U24>(V24Soll-V24Tollm))) 
		{ Fehler.bits.VERS=0; }
	else 
		{Fehler.bits.VERS=1;  
          Failure_code=3;
        }
	#ifdef _TestPanel
        Fehler.bits.VERS=0;
    #endif


	
// Temperaturen ueberwachen.  NTC defekt oder Uebertemperatur
	if ((P5_Daten[ID_P5_TEMP_IMS]<NTC_KS) || (P5_Daten[ID_P5_TEMP_IMS]>NTC_Auf) || (P5_Daten[ID_P5_TEMP_IMS]<NTC_C125) )		
		{ Fehler.bits.NTC1=1; 
            Failure_code=4;
        }
	else if (P5_Daten[ID_P5_TEMP_IMS]>NTC_C110)	
       { Fehler.bits.NTC1=0; 	// mit Histerese wieder erlauben
        if (Failure_code==4)
        {
            Failure_code=0;
        }
    }
// von PFC IMS
	if ((P5_Daten[ID_P5_TEMP_PFC]<NTC_KS) || (P5_Daten[ID_P5_TEMP_PFC]>NTC_Auf) || (P5_Daten[ID_P5_TEMP_PFC]<NTC_C125) )		
		{ Fehler.bits.NTC2=1;  
            Failure_code=5;
        }
	else if (P5_Daten[ID_P5_TEMP_PFC]>NTC_C110)	{
        Fehler.bits.NTC2=0; 	// mit Histerese wieder erlauben
        if(Failure_code==5){
            Failure_code=0;
        }
    }
	
// *************************** Analuf und Spannungskontrolle **************************************
	if ((Uzk>Uzk_min_3Ph) && (Uzk<Uzk_max_3Ph) &&  (Uzk_mid1<BULK_MAX) && (Uzk_mid2<BULK_MAX))  	// Spannung im zul. Bereich
		{ 
		if ((TimeAnlauf>=829) && (Relais==none))					// nur im Anlaufprozess machen, nach 800ms.
			{ 
			TimeAnlauf=0;
				if ((Uzk-5)<UzkAnLauf && ((signed)P5_Daten[ID_P5_NETZSPANNUNG]-(signed)Uzk)<10)				 // < als max. Differenz am Relais ,war 1V!!!!!!!
				{
				Fehler.bits.ANL=0; 					
				}
				else { 
                    Fehler.bits.ANL=1;
                    Failure_code=6;
                }		

			UzkAnLauf=Uzk;
			}
		TimeAnlauf++;
		Fehler.bits.UZK=0;
		}
	else
    { if(Uzk<Uzk_min_3Ph) {
          Fehler.bits.UZK=1;
          Failure_code=7;
		}
     UzkAnLauf=0;  TimeAnlauf=0; Fehler.bits.ANL=1;
     Failure_code=8;
    }
	if (AB_Sym < _Sym_Toll_m || AB_Sym > _Sym_Toll_p )
		{ Fehler.bits.SYM=1;  
        Failure_code=9;
        }
	else
		{ Fehler.bits.SYM=0;	}
	
	Fehler.bits.PHA=0;											// noch nicht drin, zuruecksetzen
	
	if(NetzAusfall)												// Netz wurde abgeschaltet!
		{ Fehler.bits.SPN=1;  
        Failure_code=10;
        }
	else
		Fehler.bits.SPN=0; 
  
 	P5_Daten[ID_P5_UZK_MITTE] =UzkAvr;							// fuer Uart
	P5_Daten[ID_P5_UZK_GESAMT]=Uzk;

  }		

#ifdef _Probe
	Probe();
#else	 // keine Probe , normaler Betrieb

	//************************** kein Fehler und keine Freistellung, also erlauben ********************************
  if (Fehler.Val==0 && Inhib==1 && Relais)						
	{
		Inhib=none;
        LED_F=0; //	Relais=done;
		TimeAnlauf=0;UzkAnLauf=0;
		Freigabe=1;											// noch falsch. Danach muss die Freigabe von Steuerung kommen
/*	Time_Summe=0;
		while (Time_Summe!=300)								// Delay_ms(300);
			{  
			Time_Summe++;
			Periode_tmp=Timer;
			while (Periode_tmp==Timer)						// Zeit feur Kommunikation, jede 1ms
				{	Kommunikation();}
			}
		Time_Summe=Timer+1000;
			
		Time_Summe=0;
		while (Time_Summe!=100)								//		Delay_ms(100);
			{ 
			Time_Summe++;
			Periode_tmp=Timer;
			while (Periode_tmp==Timer)
				{ Kommunikation();	}
			} */
	}
	else if(Fehler.Val & 0xFE)
    {	
        if(Failure_code > 0 && Failure_code < 17)
        {
            LED_F = FehlerAnzeigen_LED_Booster(Failure_code);
        }
        else
        {
            LED_F=1;
        }
        //
    }					 // bei Fehler LED

  /* 
  if(start_counter >= 30000)
    {
        start_counter = 30000;
        Fehler.Val=0b100000;
    }*/
  if ((Fehler.Val & 0b111110)==0 && (P5_Daten[ID_P5_PFC]& 4)==0)	// Relais einschalten wenn keine Phase und Spannungsfehler
	 {
      if( TimerRelais>500) 
       {
         Relais=done;
         Fault_saved=1;
       } 
     }							// und StatusPFC -> Anlauf ok
  else
	 {	
      Relais=none; 
      TimerRelais=0; 
      Inhib=done; 
      if (Fault_saved==1)
        {
          PDC1=0;
          SDC2=0;
          Komparator_3=0;
          Komparator_4=0;
          //Write_NMV();
          Fault_saved=0;
        }
     }				
  
  if ((Fehler.Val & 0b111111110)>0) {Inhib=done; }			// bei Fehler nur Leistung aus

#endif	  // _Probe
}
//
//***************** Auswertung der Spannungsdifferenz zwischen den beiden UZKs. **************************
void FSym(void)
{ 
	// sonstige analoge Werte uebernehmen
		P5_Daten[ID_P5_SPG_15V_1]=__builtin_divud(U15,11);  // SST hat es als 0,1V Aufloesung
        P5_Daten[ID_P5_SPG_24V_1]=111;//__builtin_divud(U24,11); 
        
		P1_Daten[ID_BG_HW_SW] = 100;//Vers;                 // hier fest, weil dort 15v - Messung
		Sollwert_Hardware= An_Er;                           // Potiwert/4 weil 12 bit Wandler, dann in A

		AB_Sym=__builtin_divsd(Sym+Sym2+Sym3+Sym4+Sym5,5);                 // Mittelwert aus zwei, sonst Schwingungen
		Sym5=Sym4;Sym4=Sym3; Sym3=Sym2;Sym2=Sym;


	if (AB_Sym > _Sym_Toll_p)							// Max erreicht, begrenzen
		{ AB_Sym = _Sym_Toll_p; }
	if (AB_Sym < _Sym_Toll_m)							// Min. erreicht, begrenzen
		{ AB_Sym = _Sym_Toll_m; }

		Sym_relative=AB_Sym;//__builtin_mulus(10,AB_Sym);   		// umrechnen in Zeit ,10ns Schritt pro 1V-delta, max also 500ns
}
//
void Probe(void)
{
	if (Timer>(_2s+100) && Relais==none)			// nach 2 Sek. auf jeden Fall einschalten
		{

		Inhib=none;									// Fehler bei _Probe ignorieren;									// erlauben
		//LED_F=0;									// Relais ein
		P5_Daten[ID_P5_LT_ONOFF]=1;

 Relais=done;
		  }
if ((Fehler.Val & 0b100000000)!=0) Inhib=done;						// bei Fehler nur Leistung aus
else if(Relais) Inhib=none;
}
void Write_NMV(void)
{
    volatile unsigned long progADDR = 0x002B00;
    init_stored_1 = IEC0;
    init_stored_2 = IEC1;
    init_stored_3 = IEC3;
    init_stored_4 = IEC4;
    init_stored_5 = IEC5;
    
    IEC0 = 0;
    IEC1 = 0;
    IEC3 = 0;
    IEC4 = 0;
    IEC5 = 0;
    
    NVMCON = 0x4003;
    NVMADRU = progADDR >> 16;
    NVMADR = progADDR & 0xFFFF;
    __builtin_disi(5);															// Disable Interrupt f?r 5 Takte
    __builtin_write_NVM();
    Delay_ms(1);
    
    TBLPAG=0xFA;																	// Set page to 
    NVMCON = 0x4001;												// Word Schreiben in Flash aktiv
    NVMADRU = progADDR >> 16;
    NVMADR = progADDR & 0xFFFF;
    __builtin_tblwtl(0, Fehler.Val);									// schreiben 16bit Low of 24, neuen Neigungswert schreiben. 
    __builtin_tblwth(0, 0x00);									// schreiben 16bit Hi of 24
    __builtin_disi(5);															// Disable Interrupt f?r 5 Takte
    __builtin_write_NVM();														// Unlock code f?r Flash schreiben und Flash schreiben starten.
       	
    IEC0 = init_stored_1;
    IEC1 = init_stored_2;
    IEC3 = init_stored_3;
    IEC4 = init_stored_4;
    IEC5 = init_stored_5;  
}

void Send_Last_Fault(unsigned int reg)
{
/*
  Fault Struct
  Bit0 PFC:1;		// PFC
  Bit1 ANL:1;		// Anlauf
  Bit2 PHA:1;		// Phasen
  Bit3 UZK:1;		// UZK
  Bit4 SYM:1;		//Symmetrie
  Bit5 SPN:1;		// Spannung abgeschaltet
  Bit6 NTC1:1;		// Temp. NTC1, INV IMS
  Bit7 NTC2:1;		// Temp. NTC2, PFC IMS
  Bit8 VERS:1;		// Versorgung +15/-15
 */
  if ((reg & 0b000010)==0b000010) // Anlauf
    {
        Fault_pwm_code=150;
    }
    else if ((reg &0b000100)==0b000100) // Phasen
    {
        Fault_pwm_code=300;
    }
    else if ((reg & 0b001000)==0b001000) // UZK
    {
        Fault_pwm_code=450;
    }
    else if ((reg & 0b010000)==0b010000)//Symmetrie
    {
        Fault_pwm_code=600;
    }
    else if ((reg & 0b100000)==0b100000)// Spannung abgeschaltet
    {
        Fault_pwm_code=750;
    }
    else if (reg==0)// No Fault
    {
        Fault_pwm_code=1023;
    }
       
}

unsigned int FehlerAnzeigen_LED_Booster(unsigned int Entry)//LED gelb
	{
	static unsigned int StepLED=0;
	static unsigned int out;
        static unsigned int onetime = 0; 
        
        if(onetime == 0){
            Timer_LEDgelb = 0;
            onetime = 1;
        }
       
	if (StepLED<Entry)														// Blink "Entry" mal
		{ 
		if (Timer_LEDgelb<500)													// 0,5 Sek ein
			{
			out=1;															// LED vrd einschalten
			}
		else
			{
			out=0;															// LED vrd ausschalten
			if (Timer_LEDgelb>1000)												// 0,5 Sek aus
				{
				Timer_LEDgelb=0;
				StepLED++;
				}
			}
		}
	else
		{
            
            if (Timer_LEDgelb>2000)													// Blink pause 2 Sek
            {
                StepLED=0;
                Timer_LEDgelb=0;
            }  
            if(Entry == 0)
            {                
                out=0;
                StepLED=0;
                Timer_LEDgelb=0;
            }
        }        
	return(out);
	}
