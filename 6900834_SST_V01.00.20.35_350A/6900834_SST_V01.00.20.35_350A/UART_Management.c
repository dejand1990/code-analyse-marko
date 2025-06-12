#include "p33EP32GS502.h"
#include "6900747_HeaderFile.h"

void ProgProtokoll(void);							// download vom Program

volatile unsigned int TimerUART;					// Zeit zwischen Time-Sendungen ueberwachen

volatile unsigned int TimerRast;					//
volatile unsigned int Prog;
volatile unsigned int Cmd;							// Nummer des Komandos in der definierten Liste bei Senden oder Empfangen
volatile unsigned int UART_1,UART_2,UART_3,UART_4;	// Daten aus der Kommunikation, High+Low Byte = 16bit
volatile unsigned int Flag_UART;					// zeigt,dass empfangene Daten da
volatile unsigned int cut_off_Power=3000;			// Zeit fuer eine neue Sendung
volatile unsigned int TP1,TP2, TP3, TP4;			// Hilfsvariablen zum Senden

volatile unsigned char CkS;
volatile unsigned int Page=0;						// Befehlsseite - Nummer,
volatile unsigned int Index=0;						// Aufzaehlung fuer Senden 254 wenn LT->SST, 255 kein Senden mehr ,Ende

volatile unsigned int OnOff=0, OnOff_Strom=0;		// von SST ein, auch ueber Stromwerte.
volatile unsigned int AliveOne=0;					// 0, nach Init bei Alive einmal Datenpacket senden dann 1 (gesendet)
volatile unsigned int power=0;						// Merker power gesendet
volatile unsigned int einmal=0;                     // Nr der extra Sendung, z.B. PFC-Ein/Aus -> 0x0540

volatile unsigned int baud_rq = 1;
volatile unsigned int baud_change_needed = 0;
volatile unsigned int uart_change_start_done = 0;
volatile unsigned int init_error_idx = 0;
volatile unsigned int one_time_op_page = 0;
volatile unsigned int SPI_mirror[16];
volatile unsigned int g_u16i_Elektrode_Lock;
volatile unsigned int g_u16i_Elektrode_Aktiv;
volatile unsigned int g_u16i_SAVE_TX_DATA;
volatile unsigned int g_u16i_SAVE_TX_DATA_CMD;

extern FEHLER Fehler;
//UART_DATA Sendungen[25];

// Datensatz zum Senden
UART_DATA TxDaten;	

// empfangener Datensatz
UART_DATA RxDaten;	

// regelmaesig senden
DATEN P5_Sendungen[P5_TIME];

unsigned int CheckInit(void);
// Variablen

// Allgemeinen Daten - sind in jeder Page identische; Wertebereich 0x00 bis 0x0F
unsigned int AllgDaten[ALLG_ANZAHL];

// Page 0 Daten: Leistungsteil Init Daten - Init Daten welche von der Steuerung gesendet werden
unsigned int P0_Daten[P0_ANZAHL_GESAMT];

// Page 1 Daten: Leistungsteil Init Daten - Init Daten welche von der Steuerung empfangen werden
unsigned int P1_Daten[P1_ANZAHL_GESAMT];

// Page 5 Daten: Leistungsteil Operation Daten
volatile unsigned int P5_Daten[P5_CMD_GESAMT];

// Initialisierung der Datenfelder
void InitDaten()
{ int i;
	AllgDaten[ID_ALLG_SET_COM_P] = INIT_LT;		// initialisieren mit Init- Page
	AllgDaten[ID_ALLG_ALIVE] = ALIVE;			// Alive Signal senden
	AllgDaten[ID_ALLG_ERROR] = 0;				// 
	AllgDaten[ID_ALLG_CLEAR_ERROR] = 0;			//
	AllgDaten[ID_ALLG_POWER_ON_OFF] = 0;		// init mit off
	AllgDaten[ID_ALLG_ACK] = 0;					// keine

// Initialisierungsdaten fuer LT in Page 0
	P0_Daten[ID_P0_INIT_START] = 0;	
	P0_Daten[ID_P0_INIT_ENDE] = P0_ANZAHL_GESAMT;
	P0_Daten[ID_P0_LEISTUNGSKLASSE] = 350;			// 350A

// Initialisierungdaten fuer SST in Page 1
	P1_Daten[ID_P1_INIT_START] = P1_ANZAHL;									// Initialisierung Start - Wert: Anzahl der Init Daten
	P1_Daten[ID_P1_INIT_ENDE] = P1_ANZAHL;                                  // Initialisierung Ende - Wert: Anzahl der Init Daten
	P1_Daten[ID_SW_LT_VERSION_LOW] = (SW_VERSION & 0xFFFF);	// Software Version Low-Word
	P1_Daten[ID_SW_LT_VERSION_HIGH] = (SW_VERSION >> 16);       // Software Version High Word
    P1_Daten[ID_SW_PFC_VERSION_LOW] = 0;
    P1_Daten[ID_SW_PFC_VERSION_HIGH] = 0;
	P1_Daten[ID_BG_INDEX] = 840;					// Baugruppen Index Nummer -> "840a"
	P1_Daten[ID_BG_VARIANTE] = 1;				// Hardware Variante 840_1
	P1_Daten[ID_BG_HW_SW] = 0;					// Baugruppe Hardware / Software Kompatibilitaet
#ifdef Trafo450
    P1_Daten[ID_LT_TYP] = 2;					// Leistungsteil Typ 0, 1 ,2 (Trafo4:1, Trafo 4,5:1, Trafo 5:1)
#else
    P1_Daten[ID_LT_TYP] = 5;					// Leistungsteil Typ 0, 1 ,2 (Trafo4:1, Trafo 4,5:1, Trafo 5:1)
#endif	
    
#ifdef _istPFC
	P1_Daten[ID_STROM_MAX] = 360;				// Maximaler Effektivstrom [A] je nach Typ
	P1_Daten[ID_PFC_EXIST] = 1;						// PFC Status - Wert: 0 = Kein PFC vorhanden / 1 = PFC vorhanden
#else
	P1_Daten[ID_STROM_MAX] =640;				// Maximaler Effektivstrom [A]
	P1_Daten[ID_PFC_EXIST] = 0;
#endif

// Operationsdaten vom LT in Page 5
	P5_Daten[ID_P5_IST_SPANNUNG] = 0;			// Ist-Spannung  0,1V
	P5_Daten[ID_P5_IST_STROM] = 0;				// Ist-Strom [1A]

	P5_Daten[ID_P5_INDUKTIVITAET] = 10;			// Schweisskreis Induktivitaet [uH]
	P5_Daten[ID_P5_WIDERSTAND] = 10;			// Schweisskreis Widerstand [1 Milli Ohm] = 0;] = 0;

	P5_Daten[ID_P5_PFC] = 0;					// PFC Status-Wert: 0 = PFC nicht da; 1 = PFC vorhanden
	//P5_Daten[ID_P5_PFC2] = 0;					// PFC2 Status-Wert: 0 = PFC nicht da; 1 = PFC vorhanden, nur bei Doppelinverter

	// Kategorie Time
	P5_Daten[ID_P5_NETZSPANNUNG] = 400;     	// Netzspannung [1V] = 0;
	P5_Daten[ID_P5_NETZSTROM] = 0;				// Netzstrom [1A] = 0;
	P5_Daten[ID_P5_NETZFREQUENZ] = 50;			// Netzfrequenz [1Hz] = 0;
	P5_Daten[ID_P5_UZK_GESAMT] = 560;			// Zwischenkreis Spannung Gesamt [1V] = 0;
	P5_Daten[ID_P5_UZK_MITTE] = 280;			// Zwischenkreis Spannung Mitte [1V] = 0;
    P5_Daten[ID_P5_SPG_15V_1] = 75;             // +15/-15V als Differenz Idealfall 7,5V
	P5_Daten[ID_P5_SPG_24V_1] = 240;			// Vorgungsspannung +24V, gemessen 100k/10k 12bit
	P5_Daten[ID_P5_TEMP_PFC] = 780;				// Temperatur Kuehlkoerper PFC 1 roh;
	P5_Daten[ID_P5_TEMP_IMS] = 780;				// Temperatur Kuehlkoerper IMS 1 roh;

	// Kategorie NACK
	P5_Daten[ID_P5_SOLL_SPG_U1] = 1000;			// Soll-Spannung U1
	P5_Daten[ID_P5_SOLL_STROM_I1] = 100;		// Soll-Strom I1
	P5_Daten[ID_P5_SPG_I1_I2] = 10;				// Spannung Umschaltung I1 / I2
	P5_Daten[ID_P5_TODZEIT_I1_I2] = 100;		// Todzeit Umschaltung I1 / I2
	P5_Daten[ID_P5_SOLL_SPG_U2] = 3;			// Soll-Spannung U2
	P5_Daten[ID_P5_SOLL_STROM_I2] = 3;			// Soll-Strom I2
	P5_Daten[ID_P5_SPG_I2_I1] = 70;				// Spannung Umschaltung I2 / I1
	P5_Daten[ID_P5_TODZEIT_I2_I1] = 100;		// Todzeit Umschaltung I2 / I1

	// Kategorie ACK
	P5_Daten[ID_P5_LT_ONOFF] = 0;				// Leistungsteil Ein-/Ausschalten - Wert: 0 = LT Aus; 1 = LT Ein
	P5_Daten[ID_P5_SIMULATION] = 0;				// Simulation - 0 = Inaktiv] 1 = Aktiv

    I2_UART = 140;
#ifdef _Probe
for( i=0;i<P5_TIME;i++)
	 {	P5_Sendungen[i].Zeit=2;					// In probe alle 2 ms, 
		P5_Sendungen[i].Send=0;					//  1 senden
	 }
#else
for( i=0;i<P5_TIME;i++)
	 {	P5_Sendungen[i].Zeit=0;					// Standard 0 ms, nicht senden!
		P5_Sendungen[i].Send=0;					// nicht gesendet
	 }

#endif

//???????, war 2 mal
	TxDaten.Cmd=10;
	TxDaten.Wert=0x0004;
	TxDaten.Ack=0;
}
//
// *************************ankommense Befehle auswerten ******************
void UART_Manangement()
{  
   if(Flag_UART==done)									// nur, wenn Gueltiges empfangen wurde
	{
	Flag_UART=none;										// Merker loeschen 	
	if(RxDaten.Cmd < ALLG_ANZAHL)						// Kommando Allgemein
    	{
		switch(RxDaten.Cmd)								// was will SST?				
			{	
		case (SET_PAGE):								// Seite setzen
			{
                        AllgDaten[ID_ALLG_SET_COM_P]=RxDaten.Wert;
                        switch(Page)
                        {
                            case(INIT_LT):
                                if((AllgDaten[ID_ALLG_SET_COM_P] == 0x01) || (AllgDaten[ID_ALLG_SET_COM_P] == 0x09) || (AllgDaten[ID_ALLG_SET_COM_P] == 0x04))
                                {
                                    Page=RxDaten.Wert<<8; 
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=Page>>8;
                                    TxDaten.Wert+=0x0100;
                                    TxDaten.Ack=1;
                                    Index=255;   
                                    if(Page == UARTBAUD || Page == SW_UPDATE)
                                    {
                                        OnOff = 0; 
                                    }
                                }
                                else
                                {
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=RxDaten.Wert;                                    
                                    TxDaten.Ack=1;
                                }
                                break;
                            case(INIT_SST1):
                                if((AllgDaten[ID_ALLG_SET_COM_P] == 0x05) || (AllgDaten[ID_ALLG_SET_COM_P] == 0x09) || (AllgDaten[ID_ALLG_SET_COM_P] == 0x04))
                                {
                                    Page=RxDaten.Wert<<8; 
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=Page>>8;
                                    TxDaten.Wert+=0x0100;
                                    TxDaten.Ack=1;
                                    Index=255;   
                                    Alive_timeout_cnt = 0;
                                    if(Page == UARTBAUD || Page == SW_UPDATE)
                                    {
                                        OnOff = 0; 
                                    }
                                }
                                else
                                {
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=RxDaten.Wert;                          
                                    TxDaten.Ack=1;
                                }
                                break;   
                            case(INIT_ERROR):
                                if(AllgDaten[ID_ALLG_SET_COM_P] == 0x09)
                                {
                                    Page=RxDaten.Wert<<8; 
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=Page>>8;
                                    TxDaten.Wert+=0x0100;
                                    TxDaten.Ack=1;
                                    Index=255;   
                                    Alive_timeout_cnt = 0;
                                    if(Page == UARTBAUD || Page == SW_UPDATE)
                                    {
                                        OnOff = 0; 
                                    }
                                }
                                else
                                {
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=RxDaten.Wert;                          
                                    TxDaten.Ack=1;
                                }
                                break;       
                            case(OPERATION):
                                if(AllgDaten[ID_ALLG_SET_COM_P] == 0x09)
                                {
                                    Page=RxDaten.Wert<<8; 
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=Page>>8;
                                    TxDaten.Wert+=0x0100;
                                    TxDaten.Ack=1;
                                    Index=255;   
                                    if(Page == UARTBAUD || Page == SW_UPDATE)
                                    {
                                        OnOff = 0; 
                                    }
                                }
                                else
                                {
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=RxDaten.Wert;                                
                                    TxDaten.Ack=1;
                                }
                                break;  
                            case(UARTBAUD):
                                if(AllgDaten[ID_ALLG_SET_COM_P] == 0x14)
                                {
                                    Page=RxDaten.Wert<<8; 
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=Page>>8;
                                    TxDaten.Wert+=0x0100;
                                    TxDaten.Ack=1;
                                    Index=255;   
                                    if(Page == UARTBAUD || Page == SW_UPDATE)
                                    {
                                        OnOff = 0; 
                                    }
                                }
                                else
                                {
                                    TxDaten.Cmd=0x00;
                                    TxDaten.Wert=RxDaten.Wert;                        
                                    TxDaten.Ack=1;
                                }
                                break;  
                            case(SW_UPDATE):                                
                                TxDaten.Cmd=0x00;
                                TxDaten.Wert=RxDaten.Wert;                              
                                TxDaten.Ack=1;                                
                                break;    
                        }                                    
                        break;
                    }
		case (SEND_ALIVE):								// Alive senden nur im Operation Mode
			{
			//##RAVI:SEND_ALIVE data packet is being sent directly from special event routine!!!
			break;	
			}
		case (SET_ERROR):								// Errorstand senden
			{
			AllgDaten[ID_ALLG_ALIVE]=RxDaten.Wert;
			TxDaten.Cmd=SET_ERROR;
			TxDaten.Wert=Fehler.Val;
			break;
			}
		case (CLEAR_ERROR):								// Loesche errors
				{
			AllgDaten[ID_ALLG_ERROR]=RxDaten.Wert;
			break;
				}
		case (POWER_ON_OFF):							// Abschalte mit Daten sichern
				{
			AllgDaten[ID_ALLG_POWER_ON_OFF]=RxDaten.Wert;
			break;
				}
		case (ACK):											// Bestaetigung, Daten waren ok
				{
			AllgDaten[ID_ALLG_ACK]=RxDaten.Wert;
			break;
				}
			}
		} 
	else															// normale Kommandos
		{   
		switch(Page)												// Arbeitsmode ,init, init, operation, pruefen, ....
			{	            
			case (INIT_LT):											// Page0, Init des LT mit Daten von SST
				{ 
                    one_time_op_page = 0;
				switch (RxDaten.Cmd & MASK_CMD)				// Komandos ausfiltern
					{ 
					case (P0_INIT_START):							// Start Init Kommando von SST
						{
					P0_Daten[ID_P0_INIT_START]=RxDaten.Wert;	// speichern in der Matrix
					TxDaten.Cmd=P0_INIT_START;					// Sendekommando, als Bestaetigung
					TxDaten.Wert=P0_ANZAHL;						// was zu senden
					TxDaten.Ack=1;								// Merker fuer den Sender, wenn moeglich senden
					AliveOne=0;									// Datenpaket bei 0 einmal senden im Alive
					break;
						}
					case (P0_INIT_ENDE):							// Ende Init 
						{
					P0_Daten[ID_P0_INIT_ENDE]=RxDaten.Wert;
					TxDaten.Cmd=P0_INIT_ENDE;	
					TxDaten.Wert=P0_ANZAHL;
					TxDaten.Ack=1;
					break;
						}
					case (P0_LEISTUNGSKLASSE):
						{
					P0_Daten[ID_P0_LEISTUNGSKLASSE]=RxDaten.Wert;
					break;
						}
					default:
				   		{
					   	TxDaten.Cmd=0x06;						// ACk senden negativ
						TxDaten.Wert=0x0000;
						TxDaten.Ack=1;
						}
					}
			break;	
				}
			case (INIT_SST1):										// Page 1 Initialisierung von SST, Daten an SST
                one_time_op_page =0;
                switch (RxDaten.Cmd)
					{ 
					case (P1_INIT_START):                                   // Start Init Kommando von LT
						{
// todo?					if(P1_Daten[ID_P1_INIT_START]==RxDaten.Wert)	// speichern in der Matrix
                        Index=ID_P1_INIT_START;                             // ab hier senden
//                    else ;// Feheler !! todo
					break;	
						}
					case (P1_INIT_ENDE):							// Ende Init 
						{
// todo					if(P1_Daten[ID_P1_INIT_ENDE]==RxDaten.Wert)
                      ;// ok todo
//                    else ;// Fehler todo
					break;
						}
					default:
						{
						TxDaten.Cmd=0x06;						// ACk senden negativ
						TxDaten.Wert=0x0000;
						TxDaten.Ack=1;
						}
					}
			break;	
            case (UARTBAUD):										// Page 5 Betrieb
                { 
                    if((RxDaten.Cmd & MASK_CMD) == 0x11)  //baud change request
                    {
                        TxDaten.Cmd=0x11;	
                        baud_rq = RxDaten.Wert;
                        TxDaten.Wert=(0x100 | baud_rq);			
                        TxDaten.Ack=1;							
                    }
                    else if((RxDaten.Cmd & MASK_CMD) == 0x12)  //baud change request start
                    {
                        baud_rq = RxDaten.Wert;
                        //baud_change_needed = 1; 
                        uart_change_start_done = 1;
                        TxDaten.Cmd=0x12;	
                        baud_rq = RxDaten.Wert;
                        TxDaten.Wert=baud_rq;			
                        TxDaten.Ack=1;	
                    }
                    else if((RxDaten.Cmd & MASK_CMD) == 0x13)  //baud change request end
                    {
                        TxDaten.Cmd=0x13;	
                        baud_rq = RxDaten.Wert;
                        TxDaten.Wert=baud_rq;			
                        TxDaten.Ack=1;
                    }
                    break;	
                }    
			case (OPERATION):										// Page 5 Betrieb
				{ 
                    if(one_time_op_page == 0)
                    {
                        Alive_timeout_cnt = 0;
                        one_time_op_page = 1;
                    }
				EmpfangOperation();								// Empfangene Daten speichern, Anfragen bearbeiten, beantworten
		//		TxDaten.Cmd=0x06;								// ACk senden positiv
		//		TxDaten.Wert=0x0001;
		//		TxDaten.Ack=1;
			break;	
				}
			case (PRUEFUNG):										// Page 6 Pruefbetrieb
				{
			//	switch (RxDaten.Cmd & MASK_CMD)					// angekommende komandos bearbeiten
			//		{;}
				break;	
				}
			case (SW_UPDATE):									// Page 7 SW
				{
			//	switch (RxDaten.Cmd & MASK_CMD)					// angekommende komandos bearbeiten
			//		{ ;}
				break;
				}
            case (INIT_ERROR):										// Page 1 Initialisierung von SST, Daten an SST
            {
                init_error_idx = ((RxDaten.Cmd - 0x10) & 0x00FF);
                if(init_error_idx > 0x0B)
                {
                    init_error_idx = 0x0B;
                }
                TxDaten.Cmd=RxDaten.Cmd;	
                TxDaten.Wert=P1_Daten[init_error_idx];			
                TxDaten.Ack=1;
                break;
            }
			default:
			   	{
				 TxDaten.Cmd=0x06;								// ACk senden negativ
				TxDaten.Wert=0x0000;
				TxDaten.Ack=1;
				}
			} // switch
		} // else
	}
}	
//
// *************** sendet Datensaetze, wird alle 40us aufgerufen, 	*********
void Protokoll(void)										
{ unsigned int d_Index;
#ifdef _TestPanel
    
#else
    switch(TxDaten.Ack)
		{ 
		case 0:
			{
			switch (Page)
				{
				case (INIT_SST1):									// Page 1 Initialisierung vom SST, Daten vom LT
					{
					if(Index <= P1_SEND)
                        InitSST();        	// Daten einmal senden
					break;
					}
				case (OPERATION):
					{ 
					Operation();									// Datenvorbereitung aufrufen
					if(OnOff) TxDaten.Ack=5;						// nur wenn Ein dann auch U/I senden
					else { TxDaten.Ack=7;}							// nur zeitgesteuerte ohne "Ein"
					break;	
					}
				}
			break;
			}	
		case 1:												// einzeln senden
			{
			TxDaten.Ack=0;
			if(Page==0x100 && Index==254) Index=2;      	// starten damit
			if(AliveOne==0 && (RxDaten.Wert==0x05AA))
			{ TxDaten.Ack=5; AliveOne=1; Index=0;}          // Datenpaket senden von Anfang an
			/*##RAVI:HERE SA V2 SW take as a reference*/
            if (TxDaten.Cmd != SEND_ALIVE){
                if(RxDaten.Cmd == SEND_ALIVE)
                {
                       //Don't send anything
                }
                else
                {      
                   
                    Senden4Byte();  //Send here and Because the data packet SEND_ALIVE is already sent from the PWM Interrupt routine.
                }
            }
            /*##RAVI:ENDS SA V2 SW as reference*/
            if(((RxDaten.Cmd & MASK_CMD) == 0x12) && (Page == UARTBAUD))
            {
                //uart_change_start_done = 0;
                baud_change_needed = 1;                         
            }
			break;
			}
		case 2:												// power on/off senden
			{
			TxDaten.Cmd=POWER_ON_OFF;
			TxDaten.Wert=power;		
			TxDaten.Ack=0;					
			Senden4Byte();
			break;
			}
		case 4:												// nur alive beantworten
			{
			Index=ID_P5_SIMULATION;                         // Anfang fuer die Zyklen
			TxDaten.Ack=5;									// weiter mit Zyklen
			Senden4Byte();
			break;
			}
		case 5:												// Strom Istwert senden
			{ 
                /*##RAVI:HERE SA V2 SW take as a reference*/    
                TxDaten.Ack=6;
                /*##RAVI:ENDS SA V2 SW as reference*/
                break;
			}
		case 6:												// Spannung Istwert
			{ 
              /*##RAVI:HERE SA V2 SW take as a reference*/
               TxDaten.Ack=7;	//test
               /*##RAVI:ENDS SA V2 SW as reference*/
                break;
			}
		case 7:												// Betriebsdaten senden im Kreis
			{
                /*##RAVI : Don't send SEND_ALIVE from here */
                // einmaliges Senden von Veraenderungen
                /*##RAVI: check here 'einmal' variable from original SW
                 einmal changes when PFC_STATUS changes, einmal = 5/0 and the info comes via SPI*/
                if(einmal)
                {
                    switch(einmal)
                      { 
                      case ID_P5_PFC:
                       TxDaten.Cmd=PFC_STATUS;
                       TxDaten.Wert=P5_Daten[ID_P5_PFC];
                       Senden4Byte(); 
                       /*PFC_STATUS is always sent from here as soon as the status is changed, i.e PFC = AKTIV, INAKTIV
                           */
                      break;
                      }
                   einmal=0;       
                }
           
                Index++;
                if(Index <ID_P5_NETZSPANNUNG)
                { 
                    Index=ID_P5_NETZSPANNUNG; 
                }				// auf Anfang setzen
                if(Index >ID_P5_TEMP_IMS)						// wieder von vorne
                { 
                    Index=ID_P5_NETZSPANNUNG;	
                }

                d_Index=Index-ID_P5_NETZSPANNUNG;
                    /*##RAVI: check here with ORIGINAL FAP SW*/
                    if(P5_Sendungen[d_Index].Zeit)					// only if the array element has non zero value then condition is true,.i.e only valid if send time cycle is received from ST
                    {
                        if(TimerUART==P5_Sendungen[d_Index].Send)	// send when required time has elapsed
                        { /*##RAVI:Test here comes or not*/
                            /*SEND_ALIVE signal isn't sent from HERE*/
                            /*IMS,PFC temperatures,NETZFREQUENZ are sent*/
                            Operation();                            // what is being sent
                            P5_Sendungen[d_Index].Send=P5_Sendungen[d_Index].Zeit+TimerUART;	// when to senf again Next time 
                            
                            Senden4Byte();                          // transmit the data                            
                        }
                    }

                
                if(OnOff) TxDaten.Ack=5;
                break;
			}
		}
#endif
	

}
//
// ***********Die Routine schaltet die UART Rx-Interrupt ein und leert den Rx-Buffer
void UARTinit(void)
	{
	// bei unterbrochenen Kommunikation zuruecksetzen
	unsigned int muster;
	while (U1STAbits.URXDA==1)					// Buffer leeren
		{
		U1STAbits.OERR=0;
		muster=U1RXREG;							// verwerfen die Datei
		}
	IFS0bits.U1RXIF=0;							// Falg loeschen
	IEC0bits.U1RXIE=1;                  		// Rx interrupt eingeschaltet
	}
//
// ******************* Senden von einem Satz ********************************
void Senden4Byte(void)
{     unsigned int dumm;

		IEC0bits.U1RXIE=0;						// Rx interrupt ausgeschaltet, damit die Daten nicht ueberschrieben werden
		/*##RAVI:HERE SA V2 SW take as a reference*/
         if(g_c8_TX_PENDING!=AKTIV)
        {
            dumm = TxDaten.Wert;
            TP2  = (unsigned char)TxDaten.Cmd;
        }
        else
        {
            dumm = g_u16i_SAVE_TX_DATA;
            TP2  = g_u16i_SAVE_TX_DATA_CMD;
        }
        /*##RAVI:ENDS SA V2 SW as reference*/
		IEC0bits.U1RXIE=1;						// Rx interrupt eingeschaltet
		TP1=(unsigned char)dumm;
		TP3=(unsigned char)(dumm>>8);
		TP4=(TP1+TP2+TP3);						// CS berechnen
		
		 if(U1STAbits.TRMT != RESET)
        {      
            U1TXREG=TP2;
            U1TXREG=TP3;
            U1TXREG=TP1;															
            U1TXREG=TP4;	
            g_c8_TX_PENDING = INAKTIV;
        }
        else
        {
            //LED_R = AKTIV;
            g_u16i_SAVE_TX_DATA     = dumm;
            g_u16i_SAVE_TX_DATA_CMD = TP2;
            g_c8_TX_PENDING = AKTIV;
            //LED_R = INAKTIV;    
        }	
}
//
// *************** Initialisierungsdaten fuer SST zum Senden steuern ********
void InitSST(void)
{ 
	switch(Index)
	{ 
		case(ID_P1_INIT_START):
		{
			TxDaten.Cmd=P1_INIT_START;			// kommand
			TxDaten.Wert=P1_ANZAHL;				// was zu senden
			TxDaten.Ack=1;							// senden
			Index=ID_SW_LT_VERSION_LOW;
			break;
		}
		case(ID_P1_INIT_ENDE):
		{
			TxDaten.Cmd=P1_INIT_ENDE;
			TxDaten.Wert=P1_ANZAHL;
			TxDaten.Ack=1;
			Index=255;								// nichts mehr senden
			break;
		}
		case(ID_SW_LT_VERSION_LOW):
		{
			TxDaten.Cmd=SW_LT_VERSION_LOW;
			TxDaten.Wert=P1_Daten[ID_SW_LT_VERSION_LOW];
			TxDaten.Ack=1;
			Index=ID_SW_LT_VERSION_HIGH;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;	// vorzeitig beenden wenn SST 
			break;
		}
		case(ID_SW_LT_VERSION_HIGH):
		{
			TxDaten.Cmd=SW_LT_VERSION_HIGH;
			TxDaten.Wert=P1_Daten[ID_SW_LT_VERSION_HIGH];
			TxDaten.Ack=1;
			Index=ID_SW_PFC_VERSION_LOW;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;
			break;
		}
		case(ID_SW_PFC_VERSION_LOW):
		{
			TxDaten.Cmd=SW_PFC_VERSION_LOW;
			TxDaten.Wert=P1_Daten[ID_SW_PFC_VERSION_LOW];
			TxDaten.Ack=1;
			Index=ID_SW_PFC_VERSION_HIGH;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;
			break;
		}	
		case(ID_SW_PFC_VERSION_HIGH):
		{
			TxDaten.Cmd=SW_PFC_VERSION_HIGH;
			TxDaten.Wert=P1_Daten[ID_SW_PFC_VERSION_HIGH];
			TxDaten.Ack=1;
			Index=ID_BG_INDEX;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;
			break;
		}
		case(ID_BG_INDEX):
		{
			TxDaten.Cmd=BG_INDEX;
			TxDaten.Wert=5;
			TxDaten.Ack=1;
			Index=ID_BG_VARIANTE;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;
			break;
		}
		case(ID_BG_VARIANTE):
			{
			TxDaten.Cmd=BG_VARIANTE;
			TxDaten.Wert=5;
			TxDaten.Ack=1;
			Index=ID_BG_HW_SW;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;
			break;
			}
		case(ID_BG_HW_SW):
			{
			TxDaten.Cmd=BG_HW_SW;
			TxDaten.Wert=0;
			TxDaten.Ack=1;
			Index=ID_LT_TYP;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;
			break;
			}
		case(ID_LT_TYP):
			{
			TxDaten.Cmd=LT_TYP;
			TxDaten.Wert=P1_Daten[ID_LT_TYP];
			TxDaten.Ack=1;
			Index=ID_STROM_MAX;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;
			break;
			}
		case(ID_STROM_MAX):
			{
			TxDaten.Cmd=STROM_MAX;
			TxDaten.Wert=P1_Daten[ID_STROM_MAX];
			TxDaten.Ack=1;
			Index=ID_PFC_EXIST;
			if(Index>P1_SEND) Index=ID_P1_INIT_ENDE;
			break;
			}
		case(ID_PFC_EXIST):
			{ 
			TxDaten.Wert=P1_Daten[ID_PFC_EXIST];
			TxDaten.Cmd=PFC_EXIST;
			TxDaten.Ack=1;
			if(Page == INIT_ERROR)
            {
                Index=255;
            }
            else
            {
                Index=ID_P1_INIT_ENDE;
            }
			break;
			}
		default: 
			;
		}
}
//
// ************** mormal Operation Daten zum Senden steuern *****************
void Operation(void)
{
		switch (Index)							// nach Reihenfolge
			{ 
//			case (ID_P5_SIMULATION):
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_SIMULATION];
//			TxDaten.Cmd=SIMULATION;
//			break;
//				}
//			case (ID_P5_STROMREGLER_TYP):					
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_STROMREGLER_TYP];
//			TxDaten.Cmd=STROMREGLER_TYP;
//			break;
//				}
			case (ID_P5_PFC):
				{
			TxDaten.Wert=P5_Daten[ID_P5_PFC];
			TxDaten.Cmd=PFC_STATUS;
			break;
				}
			case (ID_P5_LT_ONOFF):
			{
                /*##RAVI: Check it from ORIGINAL FAP stand*/
                break;
			}
            case (ID_P5_STROM_MODE):                                            //HyperPulse Mode
            {
                  
                TxDaten.Wert = P5_Daten[ID_P5_STROM_MODE];
                TxDaten.Cmd = STROM_MODE;
                 
            break;
            }
			case (ID_P5_SOLL_SPG_U1):
				{
			TxDaten.Wert=P5_Daten[ID_P5_SOLL_SPG_U1];
			TxDaten.Cmd=SOLL_SPG_U1;
			break;
				}
//			case (ID_P5_SOLL_STROM_I1):
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_SOLL_STROM_I1];
//			TxDaten.Cmd=SOLL_STROM_I1;
//			break;
//				}
//			case (ID_P5_SPG_I1_I2):
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_SPG_I1_I2];
//			TxDaten.Cmd=SPG_I1_I2;
//			break;
//				}
//			case (ID_P5_TODZEIT_I1_I2):
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_TODZEIT_I1_I2];
//			TxDaten.Cmd=TODZEIT_I1_I2;
//			break;
//				}
//			case (ID_P5_SOLL_SPG_U2):
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_SOLL_SPG_U2];
//			TxDaten.Cmd=SOLL_SPG_U2;
//			break;
//				}
//			case (ID_P5_SOLL_STROM_I2):
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_SOLL_STROM_I2];
//			TxDaten.Cmd=SOLL_STROM_I2;
//			break;
//				}
//			case (ID_P5_SPG_I2_I1):
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_SPG_I2_I1];
//			TxDaten.Cmd=SPG_I2_I1;
//			break;
//				}
//			case (ID_P5_TODZEIT_I2_I1):
//				{
//			TxDaten.Wert=P5_Daten[ID_P5_TODZEIT_I2_I1];
//			TxDaten.Cmd=TODZEIT_I2_I1;
//			break;
//				}
			case (ID_P5_INDUKTIVITAET):
				{
			TxDaten.Wert=P5_Daten[ID_P5_INDUKTIVITAET];
			TxDaten.Cmd=INDUKTIVITAET;
			break;
				}
			case (ID_P5_WIDERSTAND):
				{
			TxDaten.Wert=P5_Daten[ID_P5_WIDERSTAND];
			TxDaten.Cmd=WIDERSTAND;
			break;
				}
// schnell, max. alle 50us. hier jede 1ms als Grundzeit
			case (ID_P5_IST_STROM):
				{
			TxDaten.Wert=P5_Daten[ID_P5_IST_STROM];
			TxDaten.Cmd=IST_STROM;
			break;
				}
			case (ID_P5_IST_SPANNUNG):
				{
			TxDaten.Wert=P5_Daten[ID_P5_IST_SPANNUNG];
			TxDaten.Cmd=IST_SPANNUNG;
			break;
				}
// **********   nach Zeit, Werte in der Tabelle sind die Zeiten in ms
			case (ID_P5_NETZSPANNUNG):
				{
			TxDaten.Wert=P5_Daten[ID_P5_NETZSPANNUNG];
			TxDaten.Cmd=NETZSPANNUNG;
			break;
				}
			case (ID_P5_NETZSTROM):
				{
			TxDaten.Wert=P5_Daten[ID_P5_NETZSTROM];
			TxDaten.Cmd=NETZSTROM;
			break;
				}
			case (ID_P5_NETZFREQUENZ):
				{
			TxDaten.Wert=P5_Daten[ID_P5_NETZFREQUENZ];
			TxDaten.Cmd=NETZFREQUENZ;
			break;
				}
			case (ID_P5_UZK_GESAMT):
				{
			TxDaten.Wert=P5_Daten[ID_P5_UZK_GESAMT];
			TxDaten.Cmd=UZK_GESAMT;
			break;
				}
			case (ID_P5_UZK_MITTE):
				{
			TxDaten.Wert=P5_Daten[ID_P5_UZK_MITTE];
			TxDaten.Cmd=UZK_MITTE;
			break;
				}
			case (ID_P5_SPG_15V_1):
				{
			TxDaten.Wert=__builtin_divud(__builtin_muluu(P5_Daten[ID_P5_SPG_15V_1],363),1000);// wegen R-Beschaltung
			TxDaten.Cmd=SPG_15V_1;
			break;
				}
            case (ID_P5_SPG_24V_1):
				{
			TxDaten.Wert=__builtin_divud(__builtin_muluu(P5_Daten[ID_P5_SPG_24V_1],363),1000);// wegen R-Beschaltung
			TxDaten.Cmd=SPG_24V_1;
			break;
				}
			case (ID_P5_TEMP_PFC):
				{
			TxDaten.Wert=P5_Daten[ID_P5_TEMP_PFC];
			TxDaten.Cmd=TEMP_PFC;
			break;
				}
			case (ID_P5_TEMP_IMS):
				{
			TxDaten.Wert=P5_Daten[ID_P5_TEMP_IMS];
			TxDaten.Cmd=TEMP_IMS;
			break;
				}
			default:
			{;}
		}
}
//
// ****** Daten auf Anfrage SST vorbereiten, speichern in die Matrix oder diese senden
void EmpfangOperation(void)	
{
unsigned int delta, Temp;

	if(RxDaten.Wert>1)
		 Temp=RxDaten.Wert>>1;
	else
	Temp=RxDaten.Wert;

	switch (RxDaten.Cmd)										// Kommandos ausfiltern
		{
			case (IST_STROM):									// SST will sofort sehen
				{
			TxDaten.Wert=P5_Daten[ID_P5_IST_STROM];
			TxDaten.Cmd=IST_STROM;
			TxDaten.Ack=1;										// senden
			break;
				}
			case (IST_SPANNUNG):							
				{
			TxDaten.Wert=P5_Daten[ID_P5_IST_SPANNUNG];
			TxDaten.Cmd=IST_SPANNUNG;
			TxDaten.Ack=1;
			break;
				}
			case (INDUKTIVITAET):
				{
			TxDaten.Wert=P5_Daten[ID_P5_INDUKTIVITAET];
			TxDaten.Cmd=INDUKTIVITAET;
			TxDaten.Ack=1;
			break;
				}
			case (WIDERSTAND):
				{
			TxDaten.Wert=P5_Daten[ID_P5_WIDERSTAND];
			TxDaten.Cmd=WIDERSTAND;
			TxDaten.Ack=1;
			break;
				}
// nach Zeit, Werte in der Tabelle sind die Zeiten in ms		hier werden nur Zeiten gespeichert
			case (NETZSPANNUNG):
				{
			delta=ID_P5_NETZSPANNUNG-ID_P5_NETZSPANNUNG;
			P5_Sendungen[delta].Zeit=Temp;						// uebernehme die Zeit
			P5_Sendungen[delta].Send=TimerUART+Temp;
			TxDaten.Wert=P5_Daten[ID_P5_NETZSPANNUNG];
			TxDaten.Cmd=NETZSPANNUNG;						
			TxDaten.Ack=1;										// Antwort 
			break;
				}
			case (NETZSTROM):
				{
			delta=ID_P5_NETZSTROM-ID_P5_NETZSPANNUNG;
			P5_Sendungen[delta].Zeit=Temp;		
			P5_Sendungen[delta].Send=TimerUART+Temp;
			TxDaten.Wert=P5_Daten[ID_P5_NETZSTROM];
			TxDaten.Cmd=NETZSTROM;						
			TxDaten.Ack=1;								
			break;
				}
			case (NETZFREQUENZ):
				{
			delta=ID_P5_NETZFREQUENZ-ID_P5_NETZSPANNUNG;
			P5_Sendungen[delta].Zeit=Temp;		
			P5_Sendungen[delta].Send=TimerUART+Temp;
			TxDaten.Wert=P5_Daten[ID_P5_NETZFREQUENZ];
			TxDaten.Cmd=NETZFREQUENZ;						
			TxDaten.Ack=1;
			break;
				}
			case (UZK_GESAMT):
				{
			delta=ID_P5_UZK_GESAMT-ID_P5_NETZSPANNUNG;
			P5_Sendungen[delta].Zeit=Temp;		
			P5_Sendungen[delta].Send=TimerUART+Temp;
			TxDaten.Wert=P5_Daten[ID_P5_UZK_GESAMT];
			TxDaten.Cmd=UZK_GESAMT;						
			TxDaten.Ack=1;
			break;
				}
			case (UZK_MITTE):
				{
			delta=ID_P5_UZK_MITTE-ID_P5_NETZSPANNUNG;
			P5_Sendungen[delta].Zeit=Temp;		
			P5_Sendungen[delta].Send=TimerUART+Temp;
			TxDaten.Wert=P5_Daten[ID_P5_UZK_MITTE];
			TxDaten.Cmd=UZK_MITTE;						
			TxDaten.Ack=1;
			break;
				}
			case (SPG_15V_1):
				{
			delta=ID_P5_SPG_15V_1-ID_P5_NETZSPANNUNG;
			P5_Sendungen[delta].Zeit=Temp;	
			P5_Sendungen[delta].Send=TimerUART+Temp;
			TxDaten.Wert=P5_Daten[ID_P5_SPG_15V_1];
			TxDaten.Cmd=SPG_15V_1;						
			TxDaten.Ack=1;
			break;
				}
			case (TEMP_PFC):
				{
			delta=ID_P5_TEMP_PFC-ID_P5_NETZSPANNUNG;
			P5_Sendungen[delta].Zeit=Temp;
			P5_Sendungen[delta].Send=TimerUART+Temp;
			TxDaten.Wert=P5_Daten[ID_P5_TEMP_PFC];
			TxDaten.Cmd=TEMP_PFC;						
			TxDaten.Ack=1;
			break;
				}

			case (TEMP_IMS):
				{
			delta=ID_P5_TEMP_IMS-ID_P5_NETZSPANNUNG;
			P5_Sendungen[delta].Zeit=Temp;
			P5_Sendungen[delta].Send=TimerUART+Temp;
			TxDaten.Wert=P5_Daten[ID_P5_TEMP_IMS];
			TxDaten.Cmd=TEMP_IMS;						
			TxDaten.Ack=1;
			break;
				}
// keine ACK
			case (SOLL_SPG_U1):
			{                          
                    if (RxDaten.Wert>1000)
                        P5_Daten[ID_P5_SOLL_SPG_U1]=1000;
                    else if(RxDaten.Wert>50)
                        ;//	P5_Daten[ID_P5_SOLL_SPG_U1]=RxDaten.Wert;			// nur speichern, kein Ack
                    else
                        ;//	P5_Daten[ID_P5_SOLL_SPG_U1]=50;						// min Usoll 5,0V
                    break;
			}
			case (SOLL_STROM_I1):
				{
			P5_Daten[ID_P5_SOLL_STROM_I1]=RxDaten.Wert;
			break;
				}
			case (SPG_I1_I2):
				{
			P5_Daten[ID_P5_SPG_I1_I2]=RxDaten.Wert;
			break;
				}
			case (TODZEIT_I1_I2):
				{
			P5_Daten[ID_P5_TODZEIT_I1_I2]=RxDaten.Wert;
			break;
				}
			case (SOLL_SPG_U2):
				{
			P5_Daten[ID_P5_SOLL_SPG_U2]=RxDaten.Wert;
			break;
				}
			case (SOLL_STROM_I2):
				{
			P5_Daten[ID_P5_SOLL_STROM_I2]=RxDaten.Wert;
			break;
				}
			case (SPG_I2_I1):
				{
			P5_Daten[ID_P5_SPG_I2_I1]=RxDaten.Wert;
			break;
				}
			case (TODZEIT_I2_I1):
				{
			P5_Daten[ID_P5_TODZEIT_I2_I1]=RxDaten.Wert;
			break;
				}
            case(STROM_MODE):                                                   //HyperPulse Mode
            {
                P5_Daten[ID_P5_STROM_MODE] = RxDaten.Wert;                      //speichern in die Matrix
                break;
            }
            case (PULS_STROM_FAKTOR):
                {
                    P5_Daten[ID_P5_PULS_STROM_FAKTOR] = RxDaten.Wert;           //Puls strom faktor in Matrix speichert
                    break;
                }
            case (PULSE_ZEIT_T1_LOW):
                {
                    P5_Daten[ID_P5_PULSE_ZEIT_T1_LOW] = RxDaten.Wert;               //Puls frequenz in Matrix speichert
                    break;
                }
            case (PULSE_ZEIT_T1_HIGH):
                {
                    P5_Daten[ID_P5_PULSE_ZEIT_T1_HIGH] = RxDaten.Wert;               //Puls frequenz in Matrix speichert
                    break;
                }
            case (PULSE_ZEIT_T2_LOW):
                {
                    P5_Daten[ID_P5_PULSE_ZEIT_T2_LOW] = RxDaten.Wert;                //Puls balance in Matrix speichert
                    break;
                }
            case (PULSE_ZEIT_T2_HIGH):
                {
                    P5_Daten[ID_P5_PULSE_ZEIT_T2_HIGH] = RxDaten.Wert;                //Puls balance in Matrix speichert
                    break;
                }
            case (AC_STATUS):
            {
                P5_Daten[ID_P5_AC_STATUS] = RxDaten.Wert;
                break;
            }           
            case (WIG_ZUND_MODE):           
            {
                P5_Daten[ID_P5_WIG_ZUND_MODE]= RxDaten.Wert;
                break;
            }
            case(SCHWEISS_VERFAHREN):                                               // Determine which welding proces(MMA,WIG,MIGMAG etc.) is involved
            {
                P5_Daten[ID_P5_SCHWEISS_VERFAHREN] = RxDaten.Wert;
                if (P5_Daten[ID_P5_SCHWEISS_VERFAHREN] == SCHWEISS_VERFAHREN_MMA)
                {
                    g_u16i_Elektrode_Aktiv = AKTIV;
                }else
                {
                    g_u16i_Elektrode_Aktiv = INAKTIV;
                }
                break;
            }
// mit ACK
			case (LT_ONOFF):
			{
            	/*##RAVI: check with SA V2 SW, do what the best fits with*/							//  senden
			break;
			}
			case (SIMULATION):
				{
			P5_Daten[ID_P5_SIMULATION]=RxDaten.Wert;
			TxDaten.Wert=0x0100 | (SIMULATION & MASK_CMD);
			TxDaten.Cmd=ACK;
			TxDaten.Ack=1;
			break;
				}
            case (STROMREGLER_MODE):
			{                      
                break;
            }
			default:
			{;}
		}
}
//
