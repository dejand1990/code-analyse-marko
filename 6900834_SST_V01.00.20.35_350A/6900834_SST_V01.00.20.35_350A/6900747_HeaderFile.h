/////MAIN DEFINE
#include "LeistungsteilComKommando.h"

//#define _TestPanel 25
//#define _Enable_OC 23
#define _Enable_Root 40
#ifdef _Enable_OC
    #define OC_Option 1 //0 +1 increment(test)    //1 R filtered  //2 Uist filtered    //3 Iist filtered  //4 R raw 
#else
    #define _Enable_Rtrigger 26 
#endif
#define _Enable_LOWPASS_filter 24       //enable lowpass filter or not
#ifdef _Enable_LOWPASS_filter
    #define _Filterwalue_U  4
    #define _Filterwalue_I  4
    #define _Filterwalue_R  50
    #define _Filterversion  2          //1 = GIL version, 2 = WIEGAND version ---> better Wiegand!
#else
    #define _AverageFilter 25           //enable AVG /4 filter for Uist
#endif
#define _Root_Option 2     //0 = none, 1 = Uregulation, 2 = Iregulation + DeltaU, 3 = PWM constant + DeltaI
//#define _DeltaUenabled 70
#define _DeltaRenabled 71
//#define _DeltaRfilter 72
#define _enable_P_Ureg  0   //0 --> no proportional part of Uregulator active, 1 --> Proportional part /32 active, 2 --> Proportional part /16 active 
#define _enable_I_Ureg  1   //0 --> no integral part of Uregulator active, 1 --> Integral part /32 active, 2 --> Integral part /16 active 
#define _DeltaI_trigger 10    //deltaI trigger in (A)
#define _DeltaU_trigger 20      //deltaU trigger in resolution 0.1V = 4 dec: 20 = 0,5V
//#define _Rtrigger_mOhm 5
//#define _DeltaR_trigger (_Rtrigger_mOhm*10)
#define _DeltaR_trigger 50
#define _Roffset_value 40
#define _ConstantI_time_trigger 27        //constant Isoll time trigger in 15us resolution ---> 10 = 150us, 58 --> 800us
#define _LB_Current 4
#define _Delta_Current_min -40
#define _Us_70 6
//#define _Alternate_PWM 28
//#define _No_UZKmin_check 29


//
#define SW_VERSION_ARTIKEL_NR				6900834
#define SW_VERSION_HAUPT					1       //  max 99
#define SW_VERSION_NEBEN					00       //  max 99
#define SW_VERSION_REVISION					20       //  max 99
#define SW_VERSION_BUILD					35     //  max 99

// Multiplikations Faktoren der einzelnen Versions Nummern (Haupt, Neben, Revision, Build)

#define FAKTOR_SW_VERSION_HAUPT				1000000
#define FAKTOR_SW_VERSION_NEBEN				10000
#define FAKTOR_SW_VERSION_REVISION			100

#define SW_VERSION			((SW_VERSION_HAUPT * FAKTOR_SW_VERSION_HAUPT) + \
							(SW_VERSION_NEBEN * FAKTOR_SW_VERSION_NEBEN) + \
							(SW_VERSION_REVISION * FAKTOR_SW_VERSION_REVISION) + \
							SW_VERSION_BUILD)

/*******************************************************************************/
/*********************je nach Model einstellen**********************************/
#define Korr  0              // Korrekturwert fuer Iist wegen EP32GS502 ADC-Problem
//#define _Probe				// testen ohne Anlauf!! Spannungen hochdrehen!!
//#define ausPFC				// zum PFC abschalten beim Testen

//****** Trafos definieren. Wenn beide auskommentiert-> "normaler Trafo (Tiger 9:2)
//  #define Trafo450            // Trafo mit Flachkupfer 9:2
//#define Trafo500            // Trafo mit 10:2 Uebersetzung, Flachkupfer

// beide ohne Grund icht veraendern!!!! 
//#define immerEinPFC		// PFC immer eingeschaltet  für test 
#define _istPFC				// wenn PFC - Board 6900747 da, fuer Berechnungen der UZKs und Trafo.
//#define LMess                 // bei Kontakt L messen

/////////////////////////////////////// AD  //////////////////////////////////////
// Label der analogen Kanaele

// 12 bit Werte!
#define Uist					(ADCBUF0)
#define SC						(ADCBUF1)		// Schweissstrom vom Wandler sek. 1024A 3,3V. Version vom Prozessor "FJ" 1,65V (R13 500R !!!!!!!
#define U24     				(ADCBUF4)		// 24V Messung
#define U15         			(ADCBUF5)		// +15V , -15V Ueberwachung
#define An_Er					(ADCBUF7)		// Sollwert vom Poti

///////////////////////////////////////Schwelle//////////////////////////////////
#define done							1
#define none							0
#define OK                              1
#define NOK                             0
#define _200V							200				// Voltschwelle fuer Phaseerkennung

// Werte mal 4 weil 12 bit Wandler
#define V15Soll							832             // 16V/-16V  mit Spannungsteiler 100k/200k//10k, ueberwachter Wert
#define V15Toll							160             // 2V mit Spannungsteiler 100k/200k/10k, Toleranzbereich der Spannung
#define V24Soll							2710			// 24V mit 100k/10k
#define V24Tollp						240				// plus um 10%
#define V24Tollm						600				// nach unten mehr	bis um 18V

// die Werte sind als 10 bit (V)
#define _Sym_Toll_m						-50				// max. Symmetrieabweihung minus in V
#define _Sym_Toll_p						50				// max. Symmetrieabweihung plus in V
#define Uzk_max_3Ph						800				// maximal Spannung 3Ph
#define Uzk_min_3Ph  	      			200				// minimal Spannung 3Ph (177Vac) //###200
#define Uzk_be_max_3Ph					800				// maximal Spannung 3Ph im Betrieb (565VAC)
#define BULK_MAX						420

#define _1s								1000
#define _2s								2000
#define _1ms							250             // x 4us
#define _6ms                            428
#define _12ms                           857
#define _18ms                           1286

/////////////////////////////////////// Trafo, Drossel //////////////////////////////////
#define DTv								225				// Treiberverzoergerung vorn 240us, bei Ein 225ns /1.066
#define DTh								300				// Treiberverzoergerung hinten 320us, bei Aus (dovon IGBT um 180!))
#define Dd								141				// recovery der Dioden 150ns
#define DT								666				// Dtv+Dth+Dd
//
#ifdef _istPFC
	#ifdef Trafo450											// super Trafo
		#define LS							2300			// in nH Streuung vom Trafo  
		#define LM							1400			// in uH Trafo-Hauptinduktivitaet
		#define Ue1							45				// Uebesetzung vom Trafo in 0,1 Schritten
		#define LsUe						135         	// LS/Ue1 (nH/(N*4))/*Takt (1,06ns) .4 weil 12bit
        #define IMAX                        5900            // um 850A
        #define IMAXCMP                     3000            // Max comparator value
    #else 
    #ifdef Trafo500                                         // Supertrafo 10:2 
		#define LS							1300			// in nH Streuung vom Trafo 
		#define LM							1500			// in uH Trafo-Hauptinduktivitaet
		#define Ue1							50				// Uebesetzung vom Trafo in 0,1 Schritten
		#define LsUe						115         	// LS/Ue1 (nH/(N*4))*Takt (1,06ns) ?. 4 weil 12bit
        #define IMAX                        5900            // um 850A
        #define IMAXCMP                     3000            // Max comparator value ##Original 3750
    #else                                                   // Trafo 4:1, wie Tieger
		#define LS							3102			// in nH Streuung vom Trafo 4250nH (neue Schienen)
		#define LM							1200			// in uH Trafo-Hauptinduktivitaet
		#define Ue1							45				// Uebesetzung vom Trafo in 0,1 Schritten
		#define LsUe						183         	// LS/Ue1 (nH/(N*4))*Takt (1,06ns) 
        #define IMAX                        5900            // um 850A
        #define IMAXCMP                     3300            // Max comparator value
	#endif
    #endif
	
#else	// ohne PFC, andere Trafo 4:1
	#define LS							1948			// in nH Streuung vom Trafo 3000nH(neue Schienen) / Stromkorrektur (1,54 oPFC)
	#define LM							950				// in uH Trafo-Hauptinduktivitaet
	#define Ue1							40				// Uebesetzung vom Trafo in 0,1 Sritten
	#define LsUe						129				// LS/Ue1 (nH/N*4)*Takt (1,06ns)
	#define IMAX    					2600			// um 640A
#endif

#define Imax3PH                			3750            // 190A -auf Prim. 
//
// ///////////////////  andere Konstanten fuer Stromberechnung EP Version ///////////////
#define IDC								27				// in A*4, feste Vorspannung vom Komparator, um 22mV (3,3V und 100k_pull-up/0,667k)
#define IRAMP							3000			// in A*4/us x100  ,kuenstliche Rampe am Komparator, (9Vss ueber 56k auf 1k||2k1 in 6,38us)
#define C4								55				// C4=PTPER*8/1887. Mal 8 fuer leichtere Rechnung von TastV

///////////// NTC Schwelle gerechnet mit Pull Up Widerstand von 2k2 noch FJ auf Prim.
#define NTC_Auf                       1022 	 			// NTC offen
#define NTC_KS                        40				// NTC Kurzschluss
#define NTC_C38                       730				// 38 Grad C
#define NTC_C40                       710
#define NTC_C45                       660
#define NTC_C50                       608
#define NTC_C60                       502
#define NTC_C85                       276
#define NTC_C90                       242
#define NTC_C100                      183
#define NTC_C110                      190//140      //AG: value taken from PFC SW
#define NTC_C120                      105
#define NTC_C125                      136//92       //AG: value taken from PFC SW
#define NTC_C130                      121//92       //New Limit

/////////////////////////////////////// Ports /////////////////////////////////////
#define LED_F					LATBbits.LATB3			// LED Fehler
#define LED_B					LATBbits.LATB4			// LED Betrieb
#define RF						LATAbits.LATA3			// SW -Ausgang, als Open Col zum fuer Stromwandler2
//#define TestPin						LATBbits.LATB4			// Testpin am Programmsockel
#define Info					LATBbits.LATB15			// Info" an Verfahren ueber opto, Uaus>8V bei 1
#define Aux						PORTBbits.RB5			// Ein Aus von aussen ueber Opto. Bei low ->ein, jetzt 
      
extern volatile unsigned int one_time_failure;
extern volatile unsigned int Timer_LEDgelb;
extern volatile unsigned int H_W_Er;
extern volatile unsigned int start_counter;
extern volatile unsigned int Timer;
extern volatile unsigned int Prog;
extern volatile unsigned int SPI_mirror[16];
extern volatile unsigned int g_u16i_Isoll_AUS;
extern volatile unsigned int g_u16i_Isoll_AUS_CNT;
extern volatile unsigned int g_u16i_Timer_11_ms;
extern volatile unsigned long Soll_wert;
extern volatile signed long  Soll_k;
extern volatile signed int AB_Sym;
extern volatile unsigned int Sollwert_Hardware;
extern volatile unsigned int Uzk_mid1;
extern volatile unsigned int Uzk_mid2;	
extern volatile unsigned int Uzk;
extern volatile unsigned int UzkAvr;
extern volatile unsigned int Uaus;
extern volatile unsigned int NetzAusfall;
extern volatile unsigned int power;
extern volatile unsigned int Uzkmax;
extern UART_DATA TxDaten;
extern UART_DATA RxDaten;	
extern DATEN P5_Sendungen[];


extern volatile unsigned int Hold;
extern volatile unsigned int Timer_cut_off;		// max Zeit zwischen Kommunikation - Packeten
extern volatile unsigned int INV_Enable;
extern volatile unsigned int FSymFlag;
extern volatile unsigned int PFC_Kommando;
extern volatile unsigned int Freigabe;
extern volatile unsigned int Isoll, Iecht;
extern volatile unsigned int UZK_loaded;

extern volatile unsigned int TimerUART;
extern volatile unsigned char CkS;
extern volatile unsigned int UART_1,UART_2,UART_3,UART_4;
extern volatile unsigned int Flag_UART;
extern volatile unsigned int Cmd;
extern volatile unsigned int RampeKorr;
extern volatile unsigned int PWMH;
extern volatile unsigned int ProtokollTimer;
extern volatile signed int Sym_relative;
extern volatile unsigned int State_Sym;
extern volatile unsigned int P5_Daten[];
extern  unsigned int P1_Daten[];
extern volatile unsigned int Netz_zeit;
extern volatile unsigned int trig2;

extern volatile unsigned int pdc1;
extern volatile unsigned long I1_Time_hold1;
extern volatile unsigned long I2_Time_hold2;

extern volatile unsigned long I1_Time_samp;
extern volatile unsigned long I2_Time_samp;

extern volatile unsigned long PULSE_ZEIT_T1_LOW_HOLD;
extern volatile unsigned long PULSE_ZEIT_T1_HIGH_HOLD;
extern volatile unsigned long PULSE_ZEIT_T2_LOW_HOLD;
extern volatile unsigned long PULSE_ZEIT_T2_HIGH_HOLD;
extern volatile unsigned long Totaltime;
extern volatile unsigned long Time_uart_send;
extern volatile unsigned int t1_t2_full_rx;
extern volatile unsigned long I1_Time_hold1_LAST;
extern volatile unsigned long I2_Time_hold2_LAST;
extern volatile unsigned int Uampl;
//extern volatile unsigned int C5;

extern volatile unsigned char Inhib;
extern volatile unsigned char PFC_Status;
extern volatile signed int Sym,Sym2;
extern volatile unsigned int OnOff, OnOff_Strom;
extern volatile unsigned int maxTimerUART;
extern volatile unsigned int  DLsv;
extern volatile unsigned int LaufTimer;

extern volatile unsigned int Wert;
extern volatile unsigned int Lkreis;
extern volatile unsigned int LkreisC;
extern volatile unsigned int Lmess;
extern volatile unsigned int lrechnen;
extern volatile unsigned int t1, t2, IL1, IL2;
extern volatile unsigned int Rmess;
extern volatile unsigned int K1,K, IL3,Ut;
extern volatile unsigned int Ianf;
extern volatile unsigned int t22;

extern volatile unsigned int ueberstrom;
extern volatile unsigned int TimerRelais;
extern volatile unsigned int  Isoll_alt;	
extern volatile unsigned int Rampe;
extern volatile signed int Istep;

extern volatile unsigned int Err;
extern volatile unsigned int Relais;

extern volatile unsigned int Ilm;
extern volatile unsigned int Timer_cut_off;
extern volatile unsigned int SPI_Timer;

extern volatile unsigned int maxTimerUART;

extern volatile unsigned int Hold_M;
extern volatile unsigned int Page;

extern volatile unsigned int I2_UART;
extern volatile unsigned int baud_rq;
extern volatile unsigned int baud_change_needed;
extern volatile unsigned int uart_change_start_done;
extern volatile unsigned int PWM_counter;
extern volatile unsigned int Failure_code;
extern volatile unsigned int PFC_UZK_cnt;

extern volatile unsigned int sum_new_uist;
extern volatile unsigned int sum_old_uist;
extern volatile unsigned int y_new_uist;
extern volatile signed int y_diff_uist;
extern volatile unsigned int sum_new_Iist;
extern volatile unsigned int sum_old_Iist;
extern volatile unsigned int y_new_Iist;
extern volatile signed int y_diff_Iist;
extern volatile unsigned int sum_new_Rist;
extern volatile unsigned int sum_old_Rist;
extern volatile unsigned int y_new_Rist;
extern volatile signed int y_diff_Rist;
extern volatile unsigned int new_R;
extern volatile unsigned int Ist_wert;
extern volatile unsigned int uist_filtered;
extern volatile unsigned int trigger_root;
extern volatile unsigned int alu_mode_activ;
extern volatile unsigned int Alive_timeout_cnt;
extern volatile unsigned int Fault_pwm_code;
extern volatile unsigned int actual_P;
extern volatile unsigned int actual_Pintegral;
extern volatile unsigned int block_Isoll;
extern volatile unsigned int Imax_setpoint_nominal_A;
extern volatile unsigned int enable_Iblock_cnt;
extern volatile signed int Ignition_time_cnt1;
extern volatile unsigned long I1_Counter;
extern volatile unsigned long I2_Counter;
extern volatile unsigned int Isoll_Test;
extern volatile unsigned int HYPER_PULSE_MODUS;                                 // HyperPulse mode or normal mode
extern volatile unsigned int PULSE_STROM_FAKTOR_BIT;
extern volatile unsigned int PULSE_STROM_FAKTOR_VALUE;                          // Pulse strom factor value
extern volatile unsigned long PULSE_STROM_I1;
extern volatile unsigned long PULSE_STROM_I2;
extern volatile unsigned long PULSE_FREQUENZ;
extern volatile unsigned int PULSE_BALANCE;                                     // Pulse balance z.B. 50%
extern volatile unsigned int g_u16i_PULSE_BALANCE;
extern volatile unsigned int g_u16i_AC_STATUS;
extern volatile unsigned int KOMMANDO_RCV;
extern volatile unsigned int g_u16i_Iecht_temp;
extern volatile unsigned int g_u16i_AC_PULSE;
extern volatile unsigned int g_u16i_AC_PULSE_COUNTER ;
extern volatile unsigned long Period_p;
extern volatile unsigned long Total_counts_Us_below;
extern volatile unsigned long Proz_Wert_I1;
extern volatile unsigned long I1_Time;
extern volatile unsigned long I2_Time;
extern volatile unsigned int INV_EN_FRQ_CHANGE;
extern volatile unsigned int Calculation_done;
extern volatile unsigned long I1_Time_hold;
extern volatile unsigned long I2_Time_hold;
extern volatile unsigned int LAST_PULSE_BALANCE;
extern volatile unsigned long LAST_PULSE_FREQUENZ;
extern volatile unsigned long PULSE_FREQUENZ_DIV; 
extern volatile unsigned long Freq_Aktuell;
extern volatile unsigned int m_u32l_Balance_Aktuell;
extern volatile unsigned long Period;
extern volatile unsigned long m_u32_ActualPeriod;
extern volatile unsigned int FRQ_Changed;
extern volatile unsigned int ACKNG;
extern volatile unsigned int m_u16i_Iecht_Check;
extern volatile unsigned int m_u16i_Iecht_Check_I2;
extern volatile unsigned int Check_one;
extern volatile unsigned int Check_two;
extern volatile unsigned int start_count;
extern volatile unsigned int recheck;
extern volatile unsigned int PULSE_STROM_FAKTOR_BIT_LAST; 

extern volatile unsigned int block_calculation;
extern volatile unsigned int block_start;
extern volatile unsigned long PULSE_FREQUENZ_DIV_LAST;
extern volatile unsigned int VALID_one;
extern volatile unsigned int VALID_two;
extern volatile unsigned int m_u16i_STROM_FAKTOR_CHECK;
extern volatile unsigned long PULSE_STROM_I1_hold;                                          // Soll Strom I1
extern volatile unsigned long PULSE_STROM_I2_hold;
extern volatile unsigned long nicht_lesen;

extern volatile unsigned int g_u16i_STATE_TRANSITION;
extern volatile unsigned int g_u16i_Timer_LT_U0_Reduktion;
extern volatile unsigned int g_u16i_Elektrode_Aktiv;
extern volatile unsigned int g_u16i_Elektrode_Lock;
extern volatile unsigned int g_u16i_1msec_Period_Pdc1;
extern volatile unsigned int g_u16i_Wig_zundung_Mode_Lift_arc;
extern volatile unsigned int g_u16i_Pdc_first_time_CM;
extern volatile unsigned int Umpolung_Aktiv;
extern volatile unsigned long INV_FREE;
extern volatile char g_c_SEND_ALIVE_SIGNAL;
extern volatile char g_c_SEND_ALIVE_STATE;
extern volatile char g_c_SEND_ALIVE_DATAPACKET_SEND;
extern volatile unsigned int g_u16i_SEND_ALIVE_TIMER;
extern volatile char g_c8_TX_PENDING;
extern volatile unsigned int g_u16i_SAVE_TX_DATA;
extern volatile unsigned int g_u16i_SAVE_TX_DATA_CMD;

void FSym(void);
void ioLock (void);
void ioUnlock (void);
void SPI_Kom_Master(unsigned int PFC_Kommand);		// Kommandos Senden ueber SPI
void SPI_Konfig_Master (void);
void UART_Manangement(void);
void UARTinit(void);
void Senden4Byte(void);									// Senden der 4 Bytes
void InitSST(void);										// Datem zum Initialisieren der SST vorbereiten
void Operation(void);									// Daten von oder zu SST empfangen oder senden
void EmpfangOperation(void);							// Empfangene Daten von SST im Operation - Modus bearbeiten
void TimeSendung(void);									// welche Daten gerade zu senden sind
void StromRegler (unsigned int ZeitMessWert);
void Kommunikation(void);								// Uart, Protokoll Aufruf
void Probe(void);										// Test ohne feste Netzspannung. Langsam Hochfahren!!!
void Kurzlb(void);										// KurzLB symulieren
void PWM1(void);
void Lrechnen(void);
void InitOC(void);
void Init_UART(void);
void Read_NMV(void);
void Send_Last_Fault(unsigned int reg);

#define _USER_PROG_INIT_33					0x0000		// Anfang User Program
#define _USER_PROG_END_33          	        0x2800		// Ende User Program
#define _PAGE_SIZE_33						0x400		// Page Groesse
#define _ROW_SIZE_33						128    		// Reie Groesse
#define _Schlussel_program					251  	    // Updating Schlussel

#define Komparator_1						CMP1DAC		// Uist dedektor
#define Komparator_2						CMP2DAC		// fuer Strom Ueberwachung, max.
#define Komparator_3						CMP3DAC     // Strom Regelung PWM1
#define Komparator_4						CMP4DAC     // Strom Regelung PWM2

#define ISOLL_MAX_LIMIT                     48000               //bis 355 A needs to increase when 450 A, test with the excel filter simulation
#define ISOLL_MAX_LIMIT_2                   12000  
#define ISOLL_MIN_LIMIT                     0

//UART SET UP DEFINITION
#define CLOCK_FREQ_D16						4250000  	 // frequenza di clock divisa per 16 (68MHz/16)
#define CLOCK_FREQ_D4						17500000    // frequenza di clock divisa per 4 (68MHz/4)

#define BAUD								4000000		// fuer UART ,um 4M Baut

#define _TMR1_ON							0x8000 		// zum Einschalten T1
#define TMR1_OFF							0
#define TMR1_CLEAR							0
#define ELK_LOCK                            1
#define UNLOCK                              0
#define EIN                                 1
#define AUS                                 0
#define Elktode_spannung   0x02BC
//HyperPulse aktiv/inaktiv
#define HYPERPULSE_AKTIV                    1
#define HYPERPULSE_INAKTIV                  0
#define INAKTIV                             0
#define AKTIV                               1
#define CR9_TEST                            0
#define HYPER_PULSE_END                     2
#define ROUND_FAKTOR                        5000
#define NORM                                10000   
#define _17_8KHZ                            17500
#define RESET                               0
#define _Us_140                             800                                 //11 ms
// PWM def.
#define PWM_P				13000							// 
#define PWM_D				6000							// max. duty PWM1 6,36us	46,15% ==>  6000 * 1,06ns
#define _InterruptZeit		6150                        	//PWM Period/2 - 600 - 500 -> . Interrupt ausloesen fuer PWM. 520ns Inerrupt Ausfuehrung!? In der Summe um 150ns vom Ende der Periode um Port lesen zu koennen.
#define Time_Us             1000000                         //1s
#define Time_10_5_Us        100000                          //0.1s

#define Div                 100                             //Umrechnen faktor
#define Div_PERIOD          14                              //Minimum 1 Period
#define _15HZ               15                               // 15Hz frequency
#define _17_5KHZ            17500                            // 17.5Khz frequency
#define _2KHZ               200000
#define _25_BAL             25                               //25% Balance
#define CM_AKTIV            0                                //CM regler aktive

#define SCHWEISS_VERFAHREN_MMA 3                             //MMA schweiss verfahren

#define _SendTimer			3								// alle 3 PWM - Perioden
#define esum_Limit_AC_unten_20AMP_Positive    2200
#define esum_Limit_AC_unten_20AMP_Negative   -2200



void Delay_ms (unsigned int Mx);
void Protokoll(void);				// sendet Daten
									

typedef struct Fb
 {
  unsigned int PFC:1;		// PFC
  unsigned int ANL:1;		// Anlauf
  unsigned int PHA:1;		// Phasen
  unsigned int UZK:1;		// UZK
  unsigned int SYM:1;		//Symmetrie
  unsigned int SPN:1;		// Spannung abgeschaltet
  unsigned int NTC1:1;		// Temp. NTC1, INV IMS
  unsigned int NTC2:1;		// Temp. NTC2, PFC IMS
  unsigned int VERS:1;		// Versorgung +15/-15
 } Fbits;					// Fehlerbits

//typedef struct Dro
// {
//  unsigned int DStrom;		// Delta Strom_max-Strom_Mitte
//  unsigned int DSpannung;	// Spannung an der Drossel, Uampl-Uaus
//  unsigned int Timp;			// Impuls, Drosselanstieg.
// } S_KREIS;

typedef union Fby
 {
  unsigned int Val;			// wenn 0 keine Fehler
  Fbits bits;
 } FEHLER;

extern FEHLER Fehler;

#define _PFC_Ein	1
#define _PFC_Aus	0
#define PFC_FEHLER			0			

//Regler type
typedef enum
{
    REGLER_TYP_CM = 0,
    REGLER_TYP_PWM = 1
}e_I_REGLER_TYP;

typedef enum
{
    TRANSFER_ESUM_VAL_PWM_LOCK = 0,
    TRANSFER_ESUM_VAL_PWM      = 1,
    TRANSFER_ESUM_VAL_CM_LOCK  = 2,
    TRANSFER_ESUM_VAL_CM       = 3
}e_I_TRANSFER_ESUM_VAL;

typedef enum
{
    DC_STATE = 0,
    AC_STATE = 1
}e_I_AC_DC_STATE;

/*State machine for the SEND_ALIVE command*/
typedef enum
{
    SEND_ALIVE_SEND_START_STATE = 0,                                            // Send the response data (0x01 0x05 0x55 0x5B) once when the SEND_ALIVE(0x01) command comes from VK 
    SEND_ALIVE_SEND_PAUSE_STATE,                                                // No data is sent
    SEND_ALIVE_SEND_STOP_STATE,                                                 // stop the response data to send
}eSTATE_SEND_ALIVE;

typedef enum
{
    STATE_LT_U0_REDUKTION_INAKTIV_U0 = 0,                   // U0 Reduktion Inkativ - Leistungsteil Leerlauf Spannung
    STATE_LT_U0_REDUKTION_INAKTIV_STROM,                    // U0 Reduktion Inkativ - Strom Fliesst
    STATE_LT_U0_REDUKTION_AKTIV_SLOPE,                      // Leistungsteil Leerlauf Spannung Reduktion Slope
    STATE_LT_U0_REDUKTION_AKTIV,                            // Leistungsteil Leerlauf Spannung Reduktion
}LT_U0_ReduktionStates;
