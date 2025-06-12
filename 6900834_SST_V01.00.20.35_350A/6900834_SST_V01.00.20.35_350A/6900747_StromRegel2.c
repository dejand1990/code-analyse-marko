// fuer Version 840b

#include "p33EP32GS502.h"
#include "6900747_HeaderFile.h"
#include "math.h"

// Variablen fuer die Stromberechnung
volatile unsigned int Ilm=0;					// in 1 A Anstieg vom Magentisirungsstrom ueber Lm des Trafos

volatile unsigned long Soll_wert;
volatile unsigned int MessungZeit;				// Laenge der echten PWM (Komparator) fuer Spannung und Strom-messungen

volatile unsigned int PWMH;						// PWM in ns verfolgt gemessene Werte vom Komparator mit Zeitfilter
volatile unsigned int pwm_z;
volatile unsigned int pwm;

volatile unsigned int  DLsv=2000;				// Delay vom Ls des Trafos x100
volatile unsigned int  Uaus;					// Ausgangsspannung in 0,1V
volatile unsigned int  Uaus_alt;				// Ausgangsspannung in 0,1V vorher
volatile signed int  Ulb;						// Ausgangsspannung am Brenner ohne Lkreis, in 0,1V

volatile unsigned int RampeKorr=1;				// Strom diff. von der Rampenzeit alle Einfluesse
volatile unsigned int Lkreis=50;//145;			// Induktivitat vom Schweisskreis in 0,1uH
volatile unsigned int LkreisC=100;				// korrigierter Wert fuer Berechnungen

//S_KREIS Drossel;								// fuer die Drosselberechnung

volatile unsigned int Uampl=10;					// Ausgangsspannung - Amplitude, UZK/Ue. Init mit 10!!!! fuer Rechnung

volatile unsigned int Ist_wert_alt;	
volatile unsigned int  Iramp;
volatile unsigned int  Isoll_alt ,Isoll_alt2;	

volatile signed int Idyn;
volatile signed int Istwert_Fehler;
volatile signed int Istwert_Fehler_P;
volatile signed long Integral;
volatile signed int Prop;
volatile signed long esum, esumM;

volatile signed long Regelung;

volatile unsigned int gesamtStr;
volatile unsigned int bufferStr[50];
volatile unsigned int a_s;					// Zaehler
volatile unsigned int StromA;
volatile unsigned int trig2;

volatile unsigned int Isoll, Iecht;
volatile unsigned int RegFreigabe;

volatile unsigned int MinCopy;

volatile unsigned int pdc1=PWM_D;					// max PWM 
volatile unsigned int einbruch=34;					// 200us
volatile unsigned int sperrzeit=70;					// 700us
volatile unsigned int time_max=300;					// max Kurzschluss-Zeit, 4,2ms
volatile unsigned int kpwm;
volatile unsigned int z;							// Kurzschluss States
volatile unsigned int time;							// Kurzschluss Zeit
volatile unsigned int k=0;							// Stromwert-Ernidrigung (bit) bei Focus
volatile unsigned int d_strom;
volatile unsigned int i_max;
volatile unsigned int ZeitMesswertAvr;				//
volatile unsigned int PowerAvr;
volatile unsigned int ZeitMessWert;					// gemessene PWM in Stritten

volatile unsigned int schweisszeit;                 // in PWM-Einheiten, um 14us
volatile unsigned int StromMerker;
//volatile unsigned int MessLkreis;					// 1->Auswerten, 0-> Messwerte finden
volatile unsigned int Ist_wert=5;					// gemessener Stromwert, min 5
volatile unsigned int Ioffset=0;					// Strom - Offset vom Wandler bei Aus
volatile unsigned int mode=0;
volatile signed int Uabr, UausAvr;
volatile unsigned int Istwert_alt;
volatile unsigned int zeit_z;
volatile unsigned int ZAvr[32];
volatile unsigned int Z_sum;
volatile unsigned int Lmess=150,Ll, Lzeit,lrechnen=0;
volatile unsigned int t1=8300/128;
volatile unsigned int t22=200;
volatile unsigned int t2=800;	// t22*4
volatile unsigned int u1, u2, u3;                   // Eckspannungen bei Lmes

volatile unsigned int per_z;						// Perioden Zaehler bei Start, L - Messung
volatile unsigned int IL1=65, IL2=26;				// Strom bei Zeit 0, Strom nach 3 Per absenkung ( fuer R, L Berechnung)
volatile unsigned int pwm_werte[8];
volatile unsigned int Ianf;
volatile unsigned int komp_pwm2, komp_pwm1;
volatile unsigned int pwm_cap, pwm_capAlt1; 
volatile unsigned int merker;
const   int d=(PWM_P>>1)+DTv+DTh;

extern volatile signed int Pol;                      
extern volatile signed int Trepp; 

//new from Root test
volatile unsigned int Uist_adc = 0;
volatile unsigned int Shortcircuit_detected = 0;
volatile unsigned int Constant_PWM_enabled = 0;
volatile unsigned int Shortcircuit_PWM = 0;
volatile unsigned int constant_Isoll_cnt = 0;
volatile unsigned int constant_PWM_cnt = 0;
volatile unsigned int old_Isoll = 0;
volatile unsigned int old_Iist = 0;
volatile signed int idiff = 0;
volatile signed int udiff = 0;
volatile unsigned int Routput = 0;
volatile unsigned int I50_delay_active = 0;
volatile unsigned int I50_delay_cnt = 0;
volatile unsigned int PWM_old = 0;
volatile unsigned int Iref = 0;
volatile unsigned int Uref = 0;
volatile unsigned int uist_buffer[4];
volatile unsigned int uist_idx;
volatile unsigned int uist_sum;
volatile unsigned int uist_filtered = 0;
volatile signed int integral_ureg = 0;
volatile unsigned int PWM_duty = 0; 

//variables Root with U/I/R filtering --> > v19.07
volatile unsigned int sum_new_uist = 0;
volatile unsigned int sum_old_uist = 0;
volatile unsigned int y_new_uist = 0;
volatile signed int y_diff_uist = 0;
volatile unsigned int sum_new_Iist = 0;
volatile unsigned int sum_old_Iist = 0;
volatile unsigned int y_new_Iist = 0;
volatile signed int y_diff_Iist = 0;
volatile unsigned int sum_new_Rist = 0;
volatile unsigned int sum_old_Rist = 0;
volatile unsigned int y_new_Rist = 0;
volatile signed int y_diff_Rist = 0;
volatile unsigned int new_R = 0;
volatile unsigned int Actual_root_op = _Root_Option;
#ifdef _Enable_OC
volatile unsigned int OC_selector = OC_Option;
#endif
volatile unsigned int constant_Iactive = 0;
volatile unsigned int Pselector = _enable_P_Ureg;
volatile unsigned int Iselector = _enable_I_Ureg;
volatile unsigned int DeltaI = (_DeltaI_trigger << 2);
volatile unsigned int DeltaU = _DeltaU_trigger;
volatile unsigned int DeltaT = _ConstantI_time_trigger;
#ifdef _Enable_LOWPASS_filter
    volatile unsigned int filter_version = _Filterversion;
#endif
volatile unsigned int Rtrigger = 0;
volatile signed int Rdiff = 0;
volatile signed int Idifference = 0;
volatile signed int aux_DeltaU = 0;
volatile unsigned int min_trig_Icounter = 0;
volatile unsigned int last_uist_filter = 0;
volatile signed int res_diff = 0;
volatile unsigned int Roffset = 0;
volatile unsigned int toogle = 0;
volatile unsigned int Rlimit = _DeltaR_trigger;
volatile unsigned int Rlimit_dyn = 25;
volatile unsigned int Rlimit_cnt = 0;
volatile unsigned int Ref_Isoll = 0;
volatile unsigned int I2_UART = 140;
volatile signed int PWM_out = 0;
volatile unsigned int PWM_counter = 0;
volatile unsigned int reset_failure_once = 0;
volatile unsigned int trigger_root = 0;
volatile unsigned int alu_mode_activ = 0;
volatile unsigned int Regler_Change = 1;
volatile unsigned int Transfer_Esum_P;
volatile unsigned int PWM_Val;
volatile unsigned int PWM_Val_0;
volatile unsigned int PWM_Val_1;
volatile unsigned int PWM_Val_2;
volatile signed long Ist_filter;
volatile signed long Regel_Wert;
volatile signed int kp;
volatile unsigned int m_u16iIsollTest = 240;
volatile unsigned int m_u16i_Regler_Typ;
volatile signed int m_u16iTemp;
volatile unsigned long Freq_Aktuell;
volatile unsigned int m_u32l_Balance_Aktuell;
volatile unsigned long Period;
volatile unsigned long m_u32_ActualPeriod;
volatile unsigned long Proz_Wert_I1;
volatile unsigned long I1_Time;
volatile unsigned long I2_Time;

volatile unsigned int I2_time_set = 0;

volatile unsigned long Totaltime;
volatile unsigned long Totaltime_check;
volatile unsigned long I1_Counter;
volatile unsigned long I2_Counter;
volatile unsigned int g_u16i_PULSE_BALANCE;
volatile unsigned int m_u32l_TempPeriod;
volatile unsigned int m_u16i_Iecht_temp;
volatile unsigned int m_u16i_Iecht_Check;
volatile unsigned int m_u16i_Iecht_Check_I2;
volatile unsigned int m_u16i_STROM_FAKTOR_CHECK;

volatile unsigned int Isoll_Test;
volatile unsigned int g_u16i_AC_PULSE;
volatile unsigned int PDCCOUNTER;
volatile unsigned int  Isoll_alt_reg;
volatile unsigned  long Total_counts_Us   =  100000000;
volatile unsigned  long Total_counts_Us_below   =  100000000;
volatile unsigned long Period_p;
volatile unsigned int INV_EN_FRQ_CHANGE;
volatile unsigned int Calculation_done;
volatile unsigned int update_required;
volatile unsigned int counter_one;
volatile unsigned int counter_two;
volatile unsigned int Check_one;
volatile unsigned int Check_two;
volatile unsigned long PULSE_STROM_I1_hold;                                          // Soll Strom I1
volatile unsigned long PULSE_STROM_I2_hold;

volatile unsigned int Erste_cnt=0;
volatile unsigned int reg_set;
volatile unsigned long reg_set_cnt;
volatile unsigned int pulseset;
volatile unsigned int comparator_limt;
volatile unsigned long fuss_regler;
volatile unsigned int fuss_regler_cap;
volatile unsigned long I1_Time_store;
volatile unsigned long I2_Time_store;
volatile unsigned long Below_10Hz;
volatile unsigned long freqnz_check;
volatile unsigned long PULSE_STROM_I1_prev;
volatile unsigned long PULSE_STROM_I1_prev_lock;
volatile unsigned long nicht_lesen;
volatile unsigned long nicht_lesen_cnt;
volatile signed int m_s16i_Isoll_diff;
volatile unsigned int m_u16i_Isoll_alt_alt;
volatile unsigned int m_u16i_Isoll_alt_stell;
volatile signed int m_s16i_Isoll_esum;
volatile unsigned int filter_counter;
volatile unsigned int filter_aktive;

volatile signed int m_s16i_Isoll_diff_2;
volatile unsigned int m_u16i_Isoll_alt_alt_2;
volatile unsigned int m_u16i_Isoll_alt_stell_2;
volatile signed int m_s16i_Isoll_esum_2;
volatile unsigned int Hyper_pulse_set;
volatile unsigned int Isoll_alt_save;
volatile unsigned int Hyper_pluse_end_cnt;
volatile char g_c8_TX_PENDING;
volatile unsigned int m_u16i_STATE_LT_U0_ReduktionAktuell;
volatile unsigned int m_u16i_STATE_LT_U0_Reduktion;
volatile unsigned int pdc_elektode = 5500;
volatile unsigned int g_u16i_STATE_TRANSITION;
volatile unsigned int g_u16i_1msec_Period_Pdc1;
volatile unsigned int g_u16i_wait_for_5_AMP;
volatile unsigned int g_u16i_Pdc_first_time_CM;
volatile unsigned int g_u16i_DC_PLUS_DIV;
volatile unsigned int g_u16i_DC_MINUS_DIV;
volatile unsigned int g_u16i_AC_MINUS_DIV;
volatile unsigned int g_u16i_AC_PLUS_DIV;
volatile signed long g_u16i_AC_PLUS_LIMIT;
volatile signed long g_u16i_AC_MINUS_LIMIT;
volatile signed long g_u16i_DC_PLUS_LIMIT;
volatile signed long g_u16i_DC_MINUS_LIMIT;
volatile unsigned int Umpolung_Aktiv_timer;
volatile char g_c_SEND_ALIVE_SIGNAL;
volatile char g_c_SEND_ALIVE_STATE;                                             // STATE for different timing to send SEND_ALIVE response data to VK
volatile char g_c_SEND_ALIVE_DATAPACKET_SEND;
volatile unsigned int g_u16i_SEND_ALIVE_TIMER;

/*
// ADC AN1 ISR
void __attribute__((interrupt, no_auto_psv)) _ADCAN1Interrupt(void)
{ Info=!Info;
CMP1DAC = ADCBUF1; // read conversion result
_ADCAN1IF = 0; // clear interrupt flag
}
*/

void __attribute__((__interrupt__, no_auto_psv)) _PWMSecSpEventMatchInterrupt(void)    // PWM Sp - interrupt, Eintritt 240ns
{ 
    RF = 0;
    RF = 0;  
    RF = 0;
    RF = 0;
    RF = 1; // RF hat hier 220ns
/* erst nach zwei in Folge abschalten, geloescht wird bei LT "AUS"*/
    if(IFS6bits.AC2IF==1)
    { 
      ueberstrom++; 
      IFS6bits.AC2IF=0;
      if(ueberstrom>1) INV_Enable=0;
    }
    
    /*****************************************************TRANSFER shift register + BUFFER is not free, call until the data completely transfers *******************************************************/   
    if(g_c8_TX_PENDING == AKTIV)
    {
        Senden4Byte();
    }
/***************************************************************************************************************************************************************************************************/
   
    /*State machine for SEND_ALIVE signal*/
    switch (g_c_SEND_ALIVE_SIGNAL)
    {
        /*SEND_ALIVE signal is active*/
        case AKTIV:
        {

            /*SEND_ALIVE data is three times sent at the interval of 6ms*/
            switch(g_c_SEND_ALIVE_STATE)
            {
                case(SEND_ALIVE_SEND_START_STATE):
                {  
                   
                    Alive_timeout_cnt = 0;
    
                    TxDaten.Cmd       = SEND_ALIVE;                                                 // kommand zum Senden
                    TxDaten.Wert      = (ALIVE | Page);                                            // was zu senden
                    if(RxDaten.Wert==0x05AA)                                                // nur senden wenn von SST passt
                    {
                        TxDaten.Ack   = 1;                                                      // nur Alive baantworten
                    }
                    
                    Senden4Byte();
                    
                    if(g_c_SEND_ALIVE_DATAPACKET_SEND != OK) g_c_SEND_ALIVE_DATAPACKET_SEND = OK;
                    if(g_c_SEND_ALIVE_DATAPACKET_SEND == OK)g_c_SEND_ALIVE_STATE = SEND_ALIVE_SEND_PAUSE_STATE;
                    
                    break;
                }
                                                               
                /*When the data packet is sent once,then jump here to wait for timer to reach! Basically in between two consequitive states*/
                case(SEND_ALIVE_SEND_PAUSE_STATE):
                {                    
                   
                    g_u16i_SEND_ALIVE_TIMER++;                                  // Start the timer
                    
                    switch (g_u16i_SEND_ALIVE_TIMER)
                    {
                        /*Wait timer till 6ms to send SEND_ALIVE data packet second time, once 6ms is passed then send it again by jumping
                        to the SEND_ALIVE_SEND_START_STATE*/
                        case(_6ms):
                            g_c_SEND_ALIVE_STATE = SEND_ALIVE_SEND_START_STATE;
                            break;
                        /*Wait timer till 12ms to send SEND_ALIVE data packet third time, once 12ms is passed then send it again by jumping
                        to the SEND_ALIVE_SEND_START_STATE*/
                        case(_12ms):
                            g_c_SEND_ALIVE_STATE = SEND_ALIVE_SEND_START_STATE;
                            break;
                            /*Add here one more case if the data is sent again*/
                        /*Wait timer till 18ms to stop send*/
                        case(_18ms):
                            g_c_SEND_ALIVE_STATE = SEND_ALIVE_SEND_STOP_STATE;
                            break;                           
                    }                                      
                    if(g_c_SEND_ALIVE_DATAPACKET_SEND != NOK)g_c_SEND_ALIVE_DATAPACKET_SEND = NOK; 
                    break;
                }
                
                /*When THREE TIMES SEND_ALIVE packet is sent*/
                case(SEND_ALIVE_SEND_STOP_STATE):
                {
                    /*Send signal produced from Rx Interrupt for SEND_ALIVE should be deactivated, wait for next 20ms or 100ms to come
                     Rx Interrupt routine will notify this */                
                    g_c_SEND_ALIVE_SIGNAL   = INAKTIV;
                    g_u16i_SEND_ALIVE_TIMER = RESET;
                    
                    break;
                }              
            }
            break;
        }
        /*SEND_ALIVE signal is not active*/
        case INAKTIV:
        {
            /*Just to make sure timer is reseted when SEND_ALIVE is not active*/
            g_u16i_SEND_ALIVE_TIMER = RESET;
            break;
        }
    }
        /*State machine for SEND_ALIVE signal ends*/
    
    /*****************************************************OVERRUN ERROR at UART RECEPTION check***********************************************/
    if(U1STAbits.OERR == AKTIV)
    {        
        /*Reseting buffer and shift register to empty state if overrun happens*/ 
        U1STAbits.OERR = RESET;
         // Step 2: Read all data in the FIFO
        while (U1STAbits.URXDA) {
            char received = U1RXREG;
        // Process received data here (or buffer it)
        }
    }
/*****************************************************OVERRUN ERROR at UART RECEPTION check ends***********************************************/    

    
    IFS4bits.PSESIF=0;
}
/****************************************************** StromRegler ************************************************************/
/***************************************************   im Interrupt PWM    *******************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _PWMSpEventMatchInterrupt(void)   // PWM Sp Iinterrupt, Eintritt 370ns
{ 	
  // die Zeilen Zeitkritisch weil RF Signal ueber Port. Bei neuem Prozessor PWM - nutzen, wird genauer!!!
    RF = 0;
    RF = 0;
    RF = 0;
    RF = 0;
    RF = 1;
    
   pwm_cap = PWMCAP1;
  
	switch(INV_Enable)
	{ 
        case 1:                                                                     // wenn Betrieb ein
        {
            
/******************************AC Signal when the polarity changes from - to + **************************************************/
/***********************************AUX signal changes from low to high**********************************************************/
            if (Umpolung_Aktiv == AKTIV)
            {              
                Umpolung_Aktiv_timer++;
                esum = 0;                                                       // Integrator is reseted at this position(polarity change position) because the AC current starts from 30A to maxima
            }
            /*Next cycle after Umpolung Aktiv ist*/
            if (Umpolung_Aktiv_timer>2)         
            {            
                IOCON1bits.OVRENH    = 0;                                       // stop over write
                __builtin_nop();
                 IOCON2bits.OVRENL   = 0;
                    /*Here add if conditional statement for UMPOLUNG_AKTIV if UMPOLUNG_AKTIV_TIMER is increased */
                Umpolung_Aktiv       = INAKTIV;
                Umpolung_Aktiv_timer = 0;
                IEC1bits.CNIE        = 1;
                CNENBbits.CNIEB5     = 1;
            }           
/****************************ISOLL_UMPOLUNG manipulieren für 100 us*******************************************/                   
            
/*************************************Schweissverfahren Signal MMA - 3 comes from VK************************************************/
/***************************************Duty cycle is reduced when LB is out for 3 seconds******************************************/
            switch (g_u16i_Elektrode_Aktiv)
            {            
                case AKTIV:
                {
                    Regler_Change     = CM_AKTIV; 
                  
                    switch(m_u16i_STATE_LT_U0_Reduktion)
                    {
                    // U0 Reduktion Inkativ - Leistungsteil Leerlauf Spannung
                        case STATE_LT_U0_REDUKTION_INAKTIV_U0:

                            if(m_u16i_STATE_LT_U0_Reduktion != m_u16i_STATE_LT_U0_ReduktionAktuell)
                            {
                                Regler_Change     = CM_AKTIV;
                                // Volle Uo
                                pdc1 = 6000;
                                // Reset Timer "U0 Reduktion"
                                g_u16i_Timer_LT_U0_Reduktion = 0;

                                m_u16i_STATE_LT_U0_ReduktionAktuell = m_u16i_STATE_LT_U0_Reduktion;
                            }

                            // Strom vorhanden (> 4A) und Spannung kleiner 50V
                            if((Ist_wert > 16) && (Uist < 2068))
                            {
                                // State wechseln
                                m_u16i_STATE_LT_U0_Reduktion = STATE_LT_U0_REDUKTION_INAKTIV_STROM;
                            }

                            // Timer "U0 Reduktion" abgelaufen 3s
                            if(g_u16i_Timer_LT_U0_Reduktion > 120)
                            {
                                // State wechseln
                                g_u16i_Timer_LT_U0_Reduktion = 0;
                                m_u16i_STATE_LT_U0_Reduktion = STATE_LT_U0_REDUKTION_AKTIV_SLOPE;
                            }
                           // U2TXREG = 1;                      
                            break;

                        // U0 Reduktion Inkativ - Strom Fliesst
                        case STATE_LT_U0_REDUKTION_INAKTIV_STROM:

                            if(m_u16i_STATE_LT_U0_Reduktion != m_u16i_STATE_LT_U0_ReduktionAktuell)
                            {
                                // Volle Uo
                                pdc1 = 6000;
                                //LED_B = 1;
                                m_u16i_STATE_LT_U0_ReduktionAktuell = m_u16i_STATE_LT_U0_Reduktion;
                            }
                            // Strom nicht vorhanden (< 4A) und Spannung groesser 55V
                           // if((Ist_wert < 16) && (Uist > 2275))
                            if(Ist_wert < 16)   
                            //if(Uist > 2275)   
                            {
                                // State wechseln
                                m_u16i_STATE_LT_U0_Reduktion = STATE_LT_U0_REDUKTION_INAKTIV_U0;
                                //LED_B = 0;
                            }                     
                            break;

                        // Leistungsteil Leerlauf Spannung Reduktion Slope
                        case STATE_LT_U0_REDUKTION_AKTIV_SLOPE:
                            
                            if(m_u16i_STATE_LT_U0_Reduktion != m_u16i_STATE_LT_U0_ReduktionAktuell)
                            {
                            // Reduzierte Spannung 40V
                                m_u16i_STATE_LT_U0_ReduktionAktuell = m_u16i_STATE_LT_U0_Reduktion;
                            }                       
                            if(g_u16i_1msec_Period_Pdc1 > 4)
                            {
                                   g_u16i_1msec_Period_Pdc1 = 0;
                                   pdc_elektode=pdc_elektode-3;
                                   pdc1=pdc_elektode;
                                   if(pdc1 < 1000) 
                                   {
                                       pdc1 = 1000;
                                       // State wechseln
                                       m_u16i_STATE_LT_U0_Reduktion = STATE_LT_U0_REDUKTION_AKTIV;
                                   }
                            }                                               
                            break;

                        // Leistungsteil Leerlauf Spannung Reduktion
                        case STATE_LT_U0_REDUKTION_AKTIV:
                            
                            if(m_u16i_STATE_LT_U0_Reduktion != m_u16i_STATE_LT_U0_ReduktionAktuell)
                            {
                            // Reduzierte Spannung 40V			
                                m_u16i_STATE_LT_U0_ReduktionAktuell = m_u16i_STATE_LT_U0_Reduktion;
                            }
                            pdc1      = 1000;
                            // Strom vorhanden (> 4A) oder Spannung geaendert 30V
                            //if((Ist_wert > 16) || (Uist < 827))   //20V 
                            if((Ist_wert > 16) || (Uist < 827))
                            {
                                // State wechseln
                                m_u16i_STATE_LT_U0_Reduktion = STATE_LT_U0_REDUKTION_INAKTIV_U0;
                            }
                            break;                          
                    }
                    break;
                }
                case INAKTIV:
                    break;
            }
        
            if((SC+Korr) > Ioffset)	Ist_wert=(SC+Korr)-Ioffset;          	// Wert uebernehmen, 5A*4 bei 12 bit Fehler von AD!!?
    
            if(m_u16i_Regler_Typ == REGLER_TYP_CM)
            {
                Ist_filter   = (pwm_cap + PWM_Val_0 + PWM_Val_1 + PWM_Val_2) >> 4;
                PWM_Val_2     = PWM_Val_1;
                PWM_Val_1     = PWM_Val_0;
                PWM_Val_0     = pwm_cap;
            }
            // Ls-Trafo Verzoergerung in Takt (1,06ns)
            DLsv=__builtin_divud(__builtin_muluu(Ist_wert,LsUe),UzkAvr);	// berechnet vom gemesenen Strom, LS korrigiert um Uebersetung weil Ist_wert auch
            if(DLsv>2700) DLsv=2700;										// max. 2,7us bei 750A? und 5uH?
            
/*********************Isoll korrigieren whärend Hyper pulse inaktiv ist***************************************/
            switch (HYPER_PULSE_MODUS)                                                  
            {
                case INAKTIV:
                {
                    if (Rampe==0)   // Stromrampe interpoliert fuer 100us (genau 8*14us)                                            // Stromrampe interpoliert fuer 100us (genau 8*14us). Kann abgeschaltet werden
                    {
                        if(Isoll>(Isoll_alt+Istep)) Isoll_alt+=Istep;							// steigende
                        else if(Isoll<(Isoll_alt-Istep) && Isoll_alt>(Istep)) Isoll_alt-=Istep;	// falende
                        else Isoll_alt=Isoll;													// Rest
                    }
                    else 
                    {
                        Isoll_alt=Isoll;
                    } 
                     
                    switch (g_u16i_AC_STATUS)
                    {            
                        case AKTIV:
                            break;

                        case INAKTIV:
                            m_s16i_Isoll_diff      = (signed int)(Isoll_alt - m_u16i_Isoll_alt_alt); //##################### PT1 führungsregler
                            m_s16i_Isoll_esum      = m_s16i_Isoll_esum + m_s16i_Isoll_diff;
                            if(m_s16i_Isoll_esum > ISOLL_MAX_LIMIT)
                            {
                                m_s16i_Isoll_esum  = ISOLL_MAX_LIMIT;
                            }
                            else if(m_s16i_Isoll_esum < ISOLL_MIN_LIMIT)
                            {
                                m_s16i_Isoll_esum  = ISOLL_MIN_LIMIT;
                            }
                            m_u16i_Isoll_alt_stell = (unsigned int)(m_s16i_Isoll_esum >> 4);
                            m_u16i_Isoll_alt_alt   = m_u16i_Isoll_alt_stell;
                            Isoll_alt              = m_u16i_Isoll_alt_stell;
                            break;
                    }
                    break;
                }
                case AKTIV:
                {
                    break;
                }
            }
// Kunstrampe RC
 
#ifdef KLB		// simuliert Stromsprung vom KurzLB
Kurzlb();
#endif
#ifdef _Enable_Root

            Iramp=(__builtin_divud(Isoll_alt,24))+10;
/***************************************************CURRENT MODE is selected for less than 11 ms, HP mode and AC mode*************************************************************************/
            if(((Ist_wert >= 0)&&(Regler_Change < _Us_140)) || (HYPER_PULSE_MODUS == 1) || (g_u16i_AC_STATUS == 1))                          // LB erkennung und xx us
            {                                    
                PDC1=SDC2=pdc1;                                                      // wenn Spannungsbegrenzer aktiv Istwert_Fehler >= _Delta_Current

                if(HYPER_PULSE_MODUS != AKTIV)
                {
                    Regler_Change ++;
                    m_u16i_Regler_Typ = REGLER_TYP_CM;
                    /*When AC welding not aktive is*/
                    if((Ist_wert >= 0 && Ist_wert < 3)&&(g_u16i_wait_for_5_AMP==0)&&(g_u16i_AC_STATUS==0))
                    {
                        Regler_Change = 0;
                        esum          = 0;
                    }
                    else
                    {  
                        g_u16i_wait_for_5_AMP=1;
                        if (g_u16i_Pdc_first_time_CM==0)
                        {
                            pdc1 = pdc1 + 100;
                            if(pdc1 >= 6100) pdc1 = 6100;
                        }
                    }
                }
                else //HyperPulse Aktiv
                {                
                      m_u16i_Regler_Typ           = REGLER_TYP_CM;                
                      Hyper_pulse_set             = 1;
                      m_u16i_Isoll_alt_stell_2    = 0;
                      m_u16i_Isoll_alt_alt_2      = 0;
                      m_s16i_Isoll_diff_2         = 0;
                      m_s16i_Isoll_esum_2         = 0;
                    ///////// first caputure the I1 and I2 time let it run till end of I2 time if frenz is greater than 10hz 
                    if (Erste_cnt==0)
                    {
                        I1_Time=I1_Time_samp;     
                        I2_Time=I2_Time_samp;
                        Totaltime_check=I2_Time_samp;
                        Erste_cnt=1; 
                    }

                     if((I2_Time_samp>=50000)&&(Totaltime_check!=I2_Time_samp))           // below 10Hz immediate update
                    {
                     I1_Time=I1_Time_samp;
                     I2_Time=I2_Time_samp;
                     Totaltime_check=I2_Time_samp;
                     I1_Counter=1;
                     I2_Counter=1;
                     Below_10Hz=1;
                    }
                    else if (I2_Time_samp<50000)
                    {
                        if (Below_10Hz==1)
                        {
                            I1_Time=I1_Time_samp;
                            I2_Time=I2_Time_samp;
                            Totaltime_check=0;
                            Below_10Hz=0;
                        }
                    }

                    if(I1_Counter <= I1_Time)
                    {     
                        m_s16i_Isoll_diff      = (signed int)(PULSE_STROM_I1 - m_u16i_Isoll_alt_alt); //#####################
                        m_s16i_Isoll_esum      = m_s16i_Isoll_esum + m_s16i_Isoll_diff;
                        if(m_s16i_Isoll_esum > ISOLL_MAX_LIMIT)
                        {
                            m_s16i_Isoll_esum  = ISOLL_MAX_LIMIT;
                        }
                        else if(m_s16i_Isoll_esum < ISOLL_MIN_LIMIT)
                        {
                            m_s16i_Isoll_esum  = ISOLL_MIN_LIMIT;
                        }
                        m_u16i_Isoll_alt_stell = (unsigned int)(m_s16i_Isoll_esum >> 4);
                        m_u16i_Isoll_alt_alt   = m_u16i_Isoll_alt_stell;

                        Isoll_alt              = m_u16i_Isoll_alt_alt;

                        if(I1_Counter == I1_Time)
                        {
                           I2_Counter = 1;
                           I2_time_set=1;
                        }
                        I1_Counter++;             
                    }
                    else
                    {                      
                        Isoll_alt = PULSE_STROM_I2;
                        if(I2_Counter >= I2_Time)
                        {
                            if (Below_10Hz==0)
                            {

                            I1_Time=I1_Time_samp;
                            I2_Time=I2_Time_samp;

                            }
                            I1_Counter = 1;                      
                        }
                        I2_Counter++;
                    }              
                }                      
            }
/***************************************************PWM MODE is selected*************************************************************************/
            else
            {   
                g_u16i_Pdc_first_time_CM = AKTIV;
                m_u16i_Regler_Typ        = REGLER_TYP_PWM;
            }
/* erst nach zwei in Folge abschalten, geloescht wird bei LT "AUS"*/
            if(IFS6bits.AC2IF==1  ) 		// Ueberstrom Komparator gekommen
            { 
                ueberstrom++; 
                if(ueberstrom>1) 
                {
                    INV_Enable=0; 
                }			
                IFS6bits.AC2IF=0;
            }
            break;
        }
    
        case 0:                                                                     // Inverter aus
        {           
            pdc1                     = 4000;
            if(g_u16i_Wig_zundung_Mode_Lift_arc == AKTIV)
            {
                pdc1 = 1000;                    //1000 best
            }
            g_u16i_1msec_Period_Pdc1  = 0;
            g_u16i_STATE_TRANSITION   = 0;
            m_u16i_STATE_LT_U0_Reduktion = STATE_LT_U0_REDUKTION_INAKTIV_U0;
            m_u16i_STATE_LT_U0_ReduktionAktuell = STATE_LT_U0_REDUKTION_INAKTIV_STROM;
            Hyper_pluse_end_cnt       = 0;
            Hyper_pulse_set           = 0;
            filter_aktive             = 0;
            filter_counter            = 0;
            m_u16i_Isoll_alt_stell    = 0;
            m_u16i_Isoll_alt_alt      = 0;
            m_s16i_Isoll_diff         = 0;
            m_s16i_Isoll_esum         = 0;
            m_u16i_Isoll_alt_stell_2  = 0;
            m_u16i_Isoll_alt_alt_2    = 0;
            m_s16i_Isoll_diff_2       = 0;
            m_s16i_Isoll_esum_2       = 0;
            nicht_lesen_cnt           = 0;
            nicht_lesen               = 0;
            PULSE_STROM_I1_prev       = 0;
            PULSE_STROM_I1_prev_lock  = 0;
            Totaltime_check           = 0;
            fuss_regler_cap           = 0;
            pulseset                  = 0;
            reg_set_cnt               = 0;
            reg_set                   = 0;
            PDC1                      = 0;
            SDC2                      = 0;
            merker                    = 0;
            Komparator_3=komp_pwm2    = 0;                                             // 100A
            Komparator_4              = 0;  
            Transfer_Esum_P           = 0;
            Regler_Change             = 0;
            m_u16i_Iecht_Check        = 0;
            m_u16i_Iecht_Check_I2     = 0;
            m_u16i_STROM_FAKTOR_CHECK = INAKTIV;
            Uaus                      = 800;
            ZeitMessWert              = PWM_D;
            esum                      = 0;
            Iramp                     = 0;
            per_z                     = 0; 
            I1_Counter                = 1;
            I2_Counter                = 1;
            Ist_wert                  = 0;
            Isoll_alt                 = 0;
            m_u16i_Regler_Typ         = REGLER_TYP_CM;
            g_u16i_AC_PULSE           = 0;
            g_u16i_AC_PULSE_COUNTER   = 0;
            HYPER_PULSE_MODUS         = 0;
            PDCCOUNTER                = 0;
            INV_EN_FRQ_CHANGE         = 0;
            counter_two               = 1;
            counter_one               = 1;
            recheck                   = 0;
            start_count               = 0;
            PULSE_FREQUENZ_DIV_LAST   = 0;
            I2_time_set               = 0;
            Erste_cnt=0;
            g_u16i_wait_for_5_AMP     = 0;
            g_u16i_Pdc_first_time_CM  = 0;
            if(SC<40 && SC>0) 
            { 
                Ioffset = (Ioffset+SC)>>1;
            }       // wenn Strom kleiner als 10A, Mittelwert aus 2
            break;
        }
	}
 
    switch(g_u16i_AC_PULSE_COUNTER)
    {
        case 1:
        {
            g_u16i_AC_PULSE ++;                                                     //AC signal speichert  
            break;
        }
        case 2:
        {
            g_u16i_AC_PULSE = 0;
            break;
        }
        default:
        {
            g_u16i_AC_PULSE = 0;
            break;
        }
    }

	ProtokollTimer++;                                                           // UART - Sendungen synchron zu PWM
    SPI_Timer++;                                                                // PWM steuert SPI

/***************************REGELUNG CM starts here**********************************************/
/************************************************************************************************/
   Istwert_Fehler = (signed int)((Isoll_alt + (Isoll_alt >> 5)) - Ist_wert);
        
   if ((g_u16i_AC_STATUS==1)&&(HYPER_PULSE_MODUS==0)) 
   {
        g_u16i_AC_PLUS_LIMIT = 2500;                                               //4000
        g_u16i_AC_PLUS_DIV   = 3;                                                  //3

        g_u16i_AC_MINUS_LIMIT= -11000;  
        g_u16i_AC_MINUS_DIV  = 4; 
   }
   else
   {
        g_u16i_AC_PLUS_LIMIT = 4000;                                                  //#4000 originally
        g_u16i_AC_PLUS_DIV   = 4;                                                       // 4
        
        g_u16i_AC_MINUS_LIMIT= -9600;                                                //#-11000
        g_u16i_AC_MINUS_DIV  = 4;                                                      // 4
   }   
    
    if(((Ist_wert >= 0)&&(Regler_Change < _Us_140)) || (HYPER_PULSE_MODUS ==1)||(g_u16i_AC_STATUS == 1))
    {      
        if(m_u16i_Regler_Typ == REGLER_TYP_PWM)
        {
            m_u16i_Regler_Typ = REGLER_TYP_CM;
            esum = g_u16i_AC_PLUS_LIMIT;
        }
        esum=esum+Istwert_Fehler;
        
        if(esum < g_u16i_AC_MINUS_LIMIT) 
        {
            esum = g_u16i_AC_MINUS_LIMIT;
        }
        else if(esum > g_u16i_AC_PLUS_LIMIT)
        {
            esum = g_u16i_AC_PLUS_LIMIT;
        }
        
        if(esum < 0)
        {
            Integral = 0 - esum;
            Integral = (Integral >> g_u16i_AC_MINUS_DIV);
            Integral = 0 - Integral;
        }
        else
        {
            Integral=esum>>g_u16i_AC_PLUS_DIV;	// Verstaerkung /16, EP, dadurch max. Eingriff von 400/16 ->25 um +/-6A ( bei IGBTs wegen toff!
        }
        
        Regelung=Isoll_alt + Integral + Iramp + IDC;
        if (Regelung>IMAXCMP)	Soll_wert=IMAXCMP;                 // um 850A  3000 Max ##3300
        else if (Regelung<IDC)	Soll_wert=IDC;
        else	Soll_wert=Regelung;
        
        if(INV_Enable==1)       // nur im Betrieb
        {            
            Komparator_3    = Soll_wert;
            Komparator_4    = Soll_wert+(signed long)Sym_relative;

            if(Umpolung_Aktiv  == AKTIV)
            {
               Komparator_3    = AUS;
               Komparator_4    = AUS; 
            }
            Transfer_Esum_P = TRANSFER_ESUM_VAL_PWM;        
        }
    }
/***************************REGELUNG CM ends here*****************************************************************/
/*****************************************************************************************************************/
    else
    {     
/*****************************************REGELUNG PWM starts here************************************************/ 
/*****************************************************************************************************************/     
        Istwert_Fehler   = (signed int)((Isoll_alt -8 + (Isoll_alt >> 5)) - Ist_wert);
        Istwert_Fehler_P = (signed int)((Isoll_alt -8+(Isoll_alt >> 5))- Ist_wert);        
        if((PDC1 == 0) & (SDC2 == 0))
        {
            esum = 0;
        }
        else
        {          
            if(Transfer_Esum_P == TRANSFER_ESUM_VAL_PWM )
            {
                Transfer_Esum_P = TRANSFER_ESUM_VAL_PWM_LOCK;
                
                if (g_u16i_Wig_zundung_Mode_Lift_arc==0)        // HF zunden
                {
                    esum = 0;                 //1000 best
                }
              
            }    
            //I anteil mit filter
            if(Istwert_Fehler < 0)
            {
                if(Istwert_Fehler <= -32)
                {
                    esum=esum - 32;                                             //immer um -8A schritt
                }else
                {
                    if(Istwert_Fehler > -2)
                    {
                        esum = esum - 1;
                    }   
                    else
                    {
                        esum = esum + (signed long)(Istwert_Fehler >> 1) ;
                    }
                }
            }
            else
            {
                if(Istwert_Fehler >= 32)
                {
                    esum=esum + 32;             ///immer um 8A schritt
                }else
                {
                    if(Istwert_Fehler < 2)
                    {
                        esum = esum + 1;
                    }   
                    else
                    {
                        esum = esum + (signed long)(Istwert_Fehler >> 1) ;
                    }
                }               
            }
            //P anteil ohne Filter
            if(Istwert_Fehler_P < 0)
            {
                if(Istwert_Fehler_P <= -32)  //dead time for P controller
                {
                    kp = 8;
                }else
                {
                    kp = 0;
                }
            }
            else
            {
                if(Istwert_Fehler_P >= 32)                                      // if only more than 8A, then kp value is used
                {
                    kp = 8;
                }else
                {
                    kp = 0;                                                     //  else kp is 0
                }               
            }
        }
        
        // Integrator maximum und minimum limit
        if(esum<-49000) 
        {
            esum=-49000;
        }
        else if(esum > 49000)
        {
            esum = 49000;
        }
        
        Regelung = (signed long)__builtin_mulss(Isoll_alt, 2) + (signed long)__builtin_mulss(Istwert_Fehler_P, kp) + (signed long)__builtin_divsd(esum, 16);
        
        if(Istwert_Fehler_P < 0)
        {
            if(Istwert_Fehler_P <= -32)                                     // dead time for P controller
            {
                Regelung = Regelung + 32;                                   // offset of 8A addieren
            }else
            {
                Regelung = Regelung;
            }
        }
        else
        {
            if(Istwert_Fehler_P >= 32)
            {
                Regelung = Regelung - 32;                                   // offset of -8A addieren
            }
            else
            {
                Regelung = Regelung;
            }               
        }
        
        if (Regelung > IMAX)	Regel_Wert = IMAX;                               // max 6 us erlaubt
        else if (Regelung < IDC)	Regel_Wert = IDC;
        else	Regel_Wert = Regelung;
        

        if(INV_Enable==1)                                                       // nur im Betrieb
        {                   
            if(Transfer_Esum_P == TRANSFER_ESUM_VAL_PWM_LOCK)
            {
                
                PDC1            = (unsigned int) Ist_filter;
                SDC2            = (unsigned int) Ist_filter;
                
                Transfer_Esum_P = TRANSFER_ESUM_VAL_CM;
            }
            else
            { 
                Komparator_3      = IMAXCMP;                                                    //Comparator1 value to maximum
                Komparator_4      = IMAXCMP;                                                    //Comparator2 value to maximum
                PDC1              = (unsigned int) Regel_Wert;
                SDC2              = (unsigned int) Regel_Wert;
                Transfer_Esum_P   = TRANSFER_ESUM_VAL_CM;
                
            }
        }   
    }
/*****************************************REGELUNG PWM ends here************************************************/ 
/***************************************************************************************************************/   
#else
    Iramp=(__builtin_divud(Isoll_alt,12))+10;
    PI_regulator();
#endif        
    if(uart_change_start_done == 1)
    {
        PWM_counter++;
    }
    trigger_root = 1;
    IFS3bits.PSEMIF=0;
// bis da 6,15us
}
//




