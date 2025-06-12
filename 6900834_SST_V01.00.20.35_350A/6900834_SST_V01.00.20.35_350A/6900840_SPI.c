#include "p33EP32GS502.h"
//#include "spi.h"
#include "6900747_HeaderFile.h"

#define _Master
#define Tx1	0
#define Rx1 1
#define Tx2 2		// 1 Takt Pause
#define Rx2 3

volatile unsigned int Dummy;
volatile unsigned int SPI_State;
volatile unsigned char PFC_Status=1;				// bei start falsch
volatile unsigned int SPI_Timer_P;					// Zeit in PWM - Schritten

//state machine fuer SPI		als Master 					

#define SPI_Ein SPI1STAT=0b1000000000000000;

volatile unsigned int SPI_Timer;                    // 1 -> 1PWM also um 14us
volatile unsigned int SPI_Tx_Master;
volatile unsigned int SPI_Rx_Master;
unsigned int Dummy1,Dummy2;

//extern FEHLER Fehler;
extern volatile unsigned int Relais;
extern volatile unsigned int einmal;

void SPI_Konfig_Master (void)
	{ 				     //5432109876543210
	SPI1CON1			=0b0000010000110101;		// Prescaler =16: post 25us fuer 16 bit sendung
	SPI_Ein;
	}
//
void SPI_Kom_Master(unsigned int PFC_Kommand)       	// PFC_Kommand es ist  nicht verwendet	MARCO 
{	 static unsigned int dum=0x1000;
	 static unsigned int uzk11v=0;                  	// Speicher fuer vorlaeufige Werte uzk11
     //static unsigned int lok1,lok2;
     static unsigned int statusPFCalt=0;
     
if(SPI_Timer>25)
    { //  SPI1STAT=0;
        SPI_Timer=0; SPI_State=Tx1;
        Dummy1=SPI1BUF;
 //       SPI_Ein;
    }      // wenn zu lange Reseten

if(SPI_Timer>10)                                        // Abstand zwischen den Sendungen, um 140us
{
	switch(SPI_State)
	{
	case Tx1:                                           // erste Sendung um 6 x 14us verzoergert, Erkennung im Slave
#ifdef _istPFC
	#ifdef ausPFC 
		dum=0x1000 | (Relais<<8);						// nur Signal "Relais" senden
	#else
		#ifdef immerEinPFC 
			if (Relais==done)	dum=0x1100;               	// Relais ein
			else	dum=0x1000;								// keine Relais - Freigabe, PFC wartet
	
			if ( P5_Daten[ID_P5_PFC] & 2) 	dum=0x1101;		// Rueckmeldung ok dann einschalten PFC
		#else
			{ dum=0x1000 | (Relais<<8 | PFC_Kommand);}			// Signal Relais ok vom PFC also Leistung ein
			
		#endif
	#endif
#else // keinPFC
	dum=0x1000 | ( Relais<<8);									// nur Signal "Relais" senden
#endif // _istPFC

// Struktur fuer Flags zum Steuern PFC
// bit 0,  Ein PFC vom LT
// bit 8  Relais Enable vom LT
// bit 16 immer 1 als test
    //SPI_mirror[5]=dum;
	SPI1BUF=dum;									// Byte1 senden
	SPI_State=Rx1;
	break;

	case Rx1:
  	 if (SPI1STATbits.SPIRBF )
		{ Dummy1=SPI1BUF;       					// erstes Byte empfangen
		  SPI_State=Tx2;
			SPI_Timer_P=SPI_Timer;
		}
	break;

	case Tx2:
		if(SPI_Timer>(SPI_Timer_P+1))				// Abstand um 2 also um 28us ( bei >+1) fuer 16 bit
			{ SPI_State=Rx2;
				SPI1BUF=dum;						// Sendungswiederholung als CRC
			}
	break;

	case Rx2:								
	if (SPI1STATbits.SPIRBF )
		{	Dummy2=SPI1BUF; 						// zweites Byte empfangen, als CRC, dann gleich

		if (Dummy1==Dummy2 && Dummy1>0)				// "CRC" ok
			{ 
			SPI_Timer=0;
			Dummy2=Dummy2>>12;					// erste Position -> Verteiler der Daten
			Dummy1=Dummy1 & 0x0FFF;				// Werte ausfiltern, bit 11 Erkennung

		switch(Dummy2)              						// empfangene Daten verteilen
			{
            case 0:											// ---> Status PFC
             // Aenderung vom PFC-Status ueber UART senden *********************
            if(Dummy1 != statusPFCalt)
               { 
               einmal = ID_P5_PFC;
               P5_Daten[ID_P5_PFC]=Dummy1;
               statusPFCalt=Dummy1;                         // uebernehmen
               //SPI_mirror[0]=Dummy1;
                } 
            if(Dummy1 & 0b1010000000) 						// wenn bit 9 da, PFC-IMS vorhanden
                P1_Daten[ID_PFC_EXIST]=PFC_EXIST_PFC_VORHANDEN;
            else
                P1_Daten[ID_PFC_EXIST]=PFC_EXIST_PFC_NICHT_VORHANDEN;
            
            if(Dummy1 & 0b100000000000) 						// wenn bit 12 da, UZK loaded
                UZK_loaded = 1;
            else
                UZK_loaded = 0;
            break;         

            case 1:													// ---> Temperatur PFC
                P5_Daten[ID_P5_TEMP_PFC]=Dummy1;
                
            break;

            case 2:													// ---> Netzstrom
                P5_Daten[ID_P5_NETZSTROM]=Dummy1 & 0x3F;            // nur Strom ausfiltern
                P1_Daten[ID_BG_VARIANTE]=Dummy1>>8;
                //SPI_mirror[1]=P5_Daten[ID_P5_NETZSTROM];
            break;

            case 3:													// ---> Netzspannung gefiltert im PFC
                P5_Daten[ID_P5_NETZSPANNUNG]=Dummy1;
            break;

            case 4:													// Software vom PFC
                P1_Daten[ID_SW_PFC_VERSION_LOW]=Dummy1;
                SPI_mirror[6]=Dummy1;
            break;

            case 5:                                         		// Temperatur IMS Prim 
                P5_Daten[ID_P5_TEMP_IMS]=Dummy1;
               // SPI_mirror[7]=Dummy1;
            break;

            case 6:                                             	//  Uzk
                Uzk=Dummy1;
                //SPI_mirror[2]=Dummy1;
            break;

            case 7:                                         		// Spannung UZk_mid1 (oben)
                uzk11v=Dummy1;
                //SPI_mirror[3]=Dummy1;
            break;

            case 8:                                             	// Spannung Uzk_mid2 (unten)
                Uzk_mid2=Dummy1;
                Uzk_mid1=uzk11v;                                	// gleichzeitig, wegen Symmetrie aendern
                Sym=(signed int)Uzk_mid1-(signed int)Uzk_mid2;      // Symmetrie berechnen
                //SPI_mirror[4]=Dummy1;
            break;

            case 9:													// Software vom PFC
                P1_Daten[ID_SW_PFC_VERSION_HIGH]=Dummy1;
                SPI_mirror[7]=Dummy1; 
            break; 

            case 10:
                ;
            break;                
 			} // wenn CRC ok
      
		SPI_Timer=0; SPI_State=Tx1;                               // wieder von Vorn
		}
	}
	break;
	} // switch SPI_State
  }
}
 // PFC_Daten[0] ->           
  // EIN	:1;		// bit 0,  PFC ein fuer Inverter
  // EN 	:1;		// bit 1,  Relais Enable
  // PHA	:1;		// bit 2,  Phase falsch
  // UZK	:1;		// bit 3,  UZK falsch
  // SYM	:1;		// bit 4,  Symmetrie
  // TRA	:1;		// bit 5,  Versorgung Treiber falsch (nicht 15V+/-1)
  // NTC1	:1;		// bit 6,  Temp. NTC1_PFC falsch
  // PFC_S  :1;		// bit 7,  PFC da, im Moment immer 1, keine Option der HW noch vorgesehen
            
  // NTC2	:1;		// bit 8,  Temp NTC2, Prim falsch
  // PFCDA	:1;		// bit 9,  keine PFC Schaltung , bei 0
  // STAT1  :1;     // bit 10 immer 0, im Moment (25.07.2017)
  // STAT2  :1;     // bit 11 immer 1, Kennung      
  // RES	:4;
  
// PFC_Daten[1] -> TemperaturPFC, roh als Spannung
// PFC_Daten[2] -> Netzstrom-Avr in 1A, ab bit 8-11 HW-Version PFC. 1, 2, 3.. 0 und 16 Fehler
// PFC_Daten[3] -> Netzspannung 1V
// PFC_Daten[4] -> PFC VERSION LOW
// PFC_Daten[5] -> Temp. IMS prim
// PFC_Daten[6] -> UZK
// PFC_Daten[7] -> UZK1, oben
// PFC_Daten[8] -> UZK2, unten
// PFC_Daten[9] -> PFC VERSION HIGH
// PFC_Daten[10] -> PFC error, test
