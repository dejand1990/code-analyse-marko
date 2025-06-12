/**************************************************************************************************
 * Projekt:					Steuerung
 *
 * Filename:				LeistungsteilComKommando.h
 *
 * Firma:					Rehm GmbH & Co. KG
 * Ersteller:				Wiehr Michael
 * Datum:					25.08.2014
 *-------------------------------------------------------------------------------------------------
 * Beschreibung:
 *
 *		Definitionen der Kommandos fuer die Kommunikation mit dem Leistungsteil.
 *
 * Erlaeuterung:
 *
 *	Die Kommandos sind unterteilt in die Pages 
 *		Page 0: Init Daten von SST an LT	- Wertebereich: 0x000 bis 0x0FF
 *		Page 1: Init Daten von LT 1 an SST	- Wertebereich: 0x100 bis 0x1FF
 *		Page 2: Init Daten von LT 2 an SST	- Wertebereich: 0x200 bis 0x2FF
 *		Page 3: 
 *		Page 4: 
 *		Page 5: Operation					- Wertebereich: 0x500 bis 0x5FF
 *		Page : Software Update				- Wertebereich: 0x00 bis 0xFF
 *
 *	Die Hunderter Stelle 0x100 in der Hexadezimalen Darstellung definiert die Page Nummer.
 *	Die Kommando Nummer 0x00 bis 0x0F in jeder Page sind unabhaengig von der Page Nummer, d.h. sind in jeder Page identisch.
 *
 * Aufbau des Kommunikations Protokolles
 *
 *		Byte 1:  Kommando
 *		Byte 2:  Daten High Byte
 *		Byte 3:  Daten Low Byte
 *		Byte 4:  Check Summe
 *
 * 
 *  Definition der Softwareversion des Mikrocontroller Programms.
 *
 *			Aufbau der Software Version
 *			690xxxx.aa.bb.cc.dd  (Haupt.Neben.Revision.Build)
 *			
 *				690xxxx	-> Artikel-Nummer
 *				aa		-> Neben Nummer - Signifikante Änderung am Programm
 *				bb		-> Neben Nummer - Funktionelle Erweiterung und signifikante Aenderungen des Programms
 *				cc		-> Revisions Nummer- Fehlerbehebung im Programm und kleine funktionale Ergaenzungen
 *				dd		-> Build Nummer - Kennzeichnet den Fortschritt der Entwicklungs Staende
 *
 *************************************************************************************************/

#ifndef __LEISTUNGSTEIL_COM_KOMMANDO_H
	#define __LEISTUNGSTEIL_COM_KOMMANDO_H

//*************************************************************************************************
// Konstanten

//-------------------------------------------------------------------------------------------------
// Allgemeine Daten

// Anzahl der Allgemeinen Daten
#define ALLG_ANZAHL							16  // max. 16

//-------------------------------------------------------------------------------------------------
// Page 0: Initialisierung SST -> LT 

// Anzahl der Initialisierungs Daten, welche von der Steuerung an das Leistungsteil gesendet werden 
#define P0_ANZAHL			1

// Gesamte Daten Anzahl in der Kommunikations Page 0: Initialisierung SST Rx / LT Tx
#define P0_ANZAHL_GESAMT	(P0_ANZAHL + 2)	// 2 --> Kommando Start & Ende

// Beginn der Init Daten - Index Nummer
#define P0_BEGIN		2

// Ende der Init Daten - Index Nummer		
#define P0_ENDE		(P0_ANZAHL_GESAMT - 1)

//-------------------------------------------------------------------------------------------------
// Page 1: Initialisierung  LT -> SST

// Anzahl der Initialisierungs Daten welche vom der Steuerung empfangen werden 
#define P1_ANZAHL			10

// Gesamte Daten Anzahl in der Kommunikations Page 1: Initialisierung SST Rx / LT Tx
#define P1_ANZAHL_GESAMT	(P1_ANZAHL + 2)	// 2 --> Kommando Start & Ende

// Beginn der Init Daten - Index Nummer
#define P1_BEGIN	(ALLG_ANZAHL + 2)		// 2 --> Kommando Start & Ende

// Ende der Init Daten - Index Nummer		
#define P1_ENDE	(P1_BEGIN + P1_ANZAHL)

// Gesamte Daten Anzahl der Kommandos
#define P1_CMD_GESAMT	(P1_ANZAHL + ALLG_ANZAHL + 2)	// 2 --> Kommando Start & Ende

// gewollte Daten von SST - Index Nummer		
#define P1_SEND	(P1_ANZAHL + 1)

//-------------------------------------------------------------------------------------------------
// Kommando Page 5: Operation - Wertebereich: 0x500 bis 0x5FF

// Daten Anzahl in der Kommunikations Page 5: Operation 
#define P5_ANZAHL		32	//		0xEF		// 239 = 255 - 16 (Allgemeine Daten Anzahl)

// Gesamte Daten Anzahl der Kommandos
#define P5_CMD_GESAMT	32	//			0xFF

// Mask & Compare der Kommando Kategorien 

// Kommando Kategorie "Allgemein"
#define P5_MASK_ALLG			0x05F0
#define P5_COMPARE_ALLG			0x0500

// Kommando Kategorie "LT Zyklisch"
#define P5_MASK_LT_ZYKL			0x05F0
#define P5_COMPARE_LT_ZYKL		0x0510

// Kommando Kategorie "LT Aktiv"
#define P5_MASK_LT_AKTIV		0x05F0
#define P5_COMPARE_LT_AKTIV		0x0520

// Kommando Kategorie "LT Status"
#define P5_MASK_LT_STATUS		0x05E0
#define P5_COMPARE_LT_STATUS	0x0540

// Kommando Kategorie "LT Time"
#define P5_MASK_LT_TIME			0x05E0
#define P5_COMPARE_LT_TIME		0x0560
#define P5_TIME					ID_P5_TEMP_IMS-ID_P5_NETZSPANNUNG+1

// Kommando Kategorie "SST NACK"
#define P5_MASK_SST_NACK		0x05E0
#define P5_COMPARE_SST_NACK		0x0580

// Kommando Kategorie "SST ACK"
#define P5_MASK_SST_ACK			0x05E0
#define P5_COMPARE_SST_ACK		0x05A0

// Kommando Kategorie "SST NACK" & "SST ACK"
#define P5_MASK_SST_NACK_ACK	0x05C0
#define P5_COMPARE_SST_NACK_ACK	0x0580

//-------------------------------------------------------------------------------------------------
// Sonstiges

// Anzahl der Kommando Pages
#define P_ANZAHL				5

// Maximale Kommando Anzahl pro Kommando Page
#define CMD_MAX			0x00FF

// Maske fuer die Ermittlung der Kommando Nummer aus dem gesamten Kommando
#define MASK_CMD		0x00FF

// Maske fuer die Ermittlung der Kommando Page Nummer aus dem gesamten Kommando
#define MASK_PAGE		0xFF00

// Maske fuer die Ermittlung der allgmeinen Kommandos (unabhaengig von der Page Nummer) aus dem gesamten Kommando
#define MASK_ALLG		0x000F

// Kommando "Alive"
#define ALIVE 			0x0055	// fuer 00 kommt aktuelle pagenummer

#define ALIVE_SST		0x00AA	// fuer 00 kommt pagenummer

//*************************************************************************************************
// Aufzaehlungstypen

// Definitionen der Kommandos
//		Hinweis:
//			- Kommando Nummer 0x_00 bis 0x_0F sind unabhaengig von der Page Nummer, d.h. sind in jeder Page identisch / vorhanden
//
typedef enum
{
	//---------------------------------------------------------------------------------------------
	// Allgemeine Daten - Wertebereich: 0x_00 bis 0x_0F; _ = Page Nummer
	SET_PAGE =			0x00,		// Kommunikations Page setzen
	SEND_ALIVE =		0x01,		// Alive - Wird zyklisch in einem zeitlich definiertem Abstand gesendet
	SET_ERROR =			0x02,		// Fehler setzen
	CLEAR_ERROR =		0x03,		// Fehler loeschen
	POWER_ON_OFF =		0x04,		// Status der Netzspannungs Versorgung des Leistungsteils
	ACK =				0x05,		// Acknowledge; Bestaetigung einer erfolgreichen Uebertragung - Wert: Tx Kommando (ZZ)
									// Wert 0xYYZZ - YY =  Ack Status "ACK = 1, NACK = 0
//	NOT_ACK = 0x06,					// Not Acknowledge; Bestaetigung einer nicht erfolgreichen Uebertragung - Wert: Tx Kommando
//
	//---------------------------------------------------------------------------------------------
	// Kommando Page 0: Initialisierung SST -> LT - Wertebereich: 0x010 bis 0x0FF
	//
	//	Wertebereich: 0x000 bis 0x00F Reserviert fuer die Allgemeinen Daten
	P0_INIT_START =		0x010,		// Initialisierung Start - Wert: Anzahl der Init Daten
	P0_INIT_ENDE = 		0x011,		// Initialisierung Ende - Wert: Anzahl der Init Daten
	P0_LEISTUNGSKLASSE =0x012,		// Leistungsklasse - Wert: Maximaler Dauerstrom [A]
	P0_ED = 			0x013,		// Einschaltdauer (ED) - Wert: Einschaltdauer [%]
//	P0_TEMP_ED_ABSCHALTUNG = 0x023,	// Temperatur ED Abschaltung  - Wert: Temperatur [°C]
//	P0_ = 0x015,				// 
	
	//---------------------------------------------------------------------------------------------
	// Kommando Page 1: Initialisierung  LT -> SST - Wertebereich: 0x110 bis 0x1FF
	//
	//	Wertebereich: 0x100 bis 0x10F Reserviert fuer die Allgemeinen Daten
	P1_INIT_START = 		0x110,			// Initialisierung Start - Wert: Anzahl der Init Daten
	P1_INIT_ENDE =	 		0x111,			// Initialisierung Ende - Wert: Anzahl der Init Daten
	SW_LT_VERSION_LOW = 	0x112,			// Software Version Low-Word
	SW_LT_VERSION_HIGH = 	0x113,			// Software Version High Word
	SW_PFC_VERSION_LOW=		0x114,			// PFC Software Version Low-Word
	SW_PFC_VERSION_HIGH=	0x115,			// PFC Software Version High-Word
	BG_INDEX = 				0x116,			// Baugruppen Index Nummer
	BG_VARIANTE = 			0x117,			// Hardware Variante PFC
	BG_HW_SW =				0x118,			// Baugruppe Hardware / Software Kompatibilitaet
	LT_TYP = 				0x119,			// Leistungsteil Typ
	STROM_MAX = 			0x11A,			// Maximaler Effektivstrom [A]
	PFC_EXIST = 			0x11B,			// PFC Exist - Wert: 0 = Kein PFC vorhanden / 1 = PFC vorhanden
	//---------------------------------------------------------------------------------------------
	// Kommando Page 2: Initialisierung SST Rx / LT 2 Tx - Wertebereich: 0x210 bis 0x2FF
	// nur bei Doppelinverter.
	//---------------------------------------------------------------------------------------------
	// Kommando Page 3: xxx- Wertebereich: 0x300 bis 0x3FF
	//
	//	Wertebereich: 0x300 bis 0x30F Reserviert fuer die Allgemeinen Daten
	P3_RES1 =			 0x310,			// 
	
	//---------------------------------------------------------------------------------------------
	// Kommando Page 4: Software Update - Wertebereich: 0x400 bis 0x4FF
	//
	//	Wertebereich: 0x400 bis 0x40F Reserviert fuer die Allgemeinen Daten
	P4_RES1 = 			0x410,			// 

	//---------------------------------------------------------------------------------------------
	// Kommando Page 5: Operation - Wertebereich: 0x500 bis 0x5FF
	//
	//	Wertebereich: 0x500 bis 0x50F Reserviert fuer die Allgemeinen Daten

	// Kommando Kategorie: LT Zyklisch - 0x510 bis 0x51F
	IST_STROM = 		0x510,		// Ist-Strom [1A]
	IST_SPANNUNG = 		0x511,		// Ist-Spannung [0,1V]

	// Kommando Kategorie: LT Aktiv bei jedem "Ein"  - 0x520 bis 0x52F
	INDUKTIVITAET = 	0x520,		// Schweisskreis Induktivitaet [1 µH]
	WIDERSTAND = 		0x521,		// Schweisskreis Widerstand [1 Milli Ohm]]

	// Kommando Kategorie: Reserve - 0x530 bis 0x53F

	// Kommando Kategorie: LT Status - 0x540 bis 0x55F
	PFC_STATUS =		0x540,		// PFC1 Status - Wert: 0 = PFC aus / 1 = PFC ein, todo

	// Kommando Kategorie: LT Time - 0x560 bis 0x57F
	NETZSPANNUNG = 		0x560,		// Netzspannung [1V]
	NETZSTROM    = 		0x561,		// Netzstrom [1A]
	NETZFREQUENZ = 		0x562,		// Netzfrequenz [1Hz]
	UZK_GESAMT   = 		0x563,		// Zwischenkreis Spannung Gesamt LT [1V]
	UZK_MITTE    = 		0x565,		// Zwischenkreis Spannung Mitte LT [1V]
//	RESERVE1     = 		0x567,		// Reserve1
	TEMP_PFC     = 		0x569,		// Temperatur Kuehlkoerper PFC LT [1°C]
	TEMP_IMS    = 		0x56B,		// Temperatur Kuehlkoerper IMS LT [1°C]
	//TEMP_IMS = 		0x56C,		// Temperatur Kuehlkoerper IMS LT2 [1°C], nue bei Doppelinv.
    SPG_15V_1   = 		0x56F,		// Versorgungsspannung +15V/-15V LT1, als Differenz (7,5V beide ok)
    //SPG_15V_2 = 		0x570,      // LT2 
    SPG_24V_1   = 		0x571,      // 24V vom LT1 
    //SPG_24V_2 = 		0x572,      // 24V vom LT2  


// Kommando Kategorie: SST NACK - 0x580 bis 0x59F
	SOLL_SPG_U1       = 0x580,		// Soll-Spannung U1
	SOLL_STROM_I1     = 0x581,		// Soll-Strom I1
	SPG_I1_I2         =	0x582,		// Spannung Umschaltung I1 / I2
	TODZEIT_I1_I2     =	0x583,		// Todzeit Umschaltung I1 / I2
	SOLL_SPG_U2       =	0x584,		// Soll-Spannung U2
	SOLL_STROM_I2     =	0x585,		// Soll-Strom I2
	SPG_I2_I1         =	0x586,		// Spannung Umschaltung I2 / I1
	TODZEIT_I2_I1     = 0x587,		// Todzeit Umschaltung I2 / I1
    STROM_MODE        = 0x588,      // HyperPulse - 0= Inaktiv, 1= Aktiv
    PULS_STROM_FAKTOR = 0x589,      // Pulse Faktor für I1 / I2 calculations
    PULSE_ZEIT_T1_LOW = 0x58A,      // 
    PULSE_ZEIT_T1_HIGH = 0x58B,      //
    PULSE_ZEIT_T2_LOW = 0x58C,      // 
    PULSE_ZEIT_T2_HIGH = 0x58D,      //
    AC_STATUS          = 0x58E,      // AC status = 1, DC status = 0
    WIG_ZUND_MODE      = 0x58F,
    SCHWEISS_VERFAHREN = 0x590,     //0 = Undefined / Inaktiv, 1= MSG (MIG/MAG),2 = WIG,3 = MMA (Elektrode),4 = Fugenhobeln,5= Plasma Schneiden,6=  Naht Reinigen, 7= MSG Manuell,0xFE = SQ Pruefung, 0xFF = Undefined - Geloeschter Speicher
    STROMREGLER_MODE  = 0x59F,      // 

// Kommando Kategorie: SST ACK  - 0x5A0 bis 0x5BF
	LT_ONOFF =			0x5A0,		// Leistungsteil Ein-/Ausschalten - Wert: 0 = LT Aus; 1 = LT Ein
	SIMULATION =		0x5A1,		// Simulation - 0 = Inaktiv, 1 = Aktiv
            
	//---------------------------------------------------------------------------------------------
// Kommando Page 6: xxx - Wertebereich: 0x600 bis 0x6FF
	//
	//	Wertebereich: 0x600 bis 0x60F Reserviert fuer die Allgemeinen Daten
	P6_RES1 = 			0x610,	
}CMD;

// Definitionen der Index Nummern fuer die Allgemeinen Kommandos - Kommandos die fuer jede Page gleich sind
typedef enum
{
	ID_ALLG_SET_COM_P   = 0,	// Kommunikations Page setzen
	ID_ALLG_ALIVE,				// Alive - Wird zyklisch in einem zeitlich definiertem Abstand gesendet
	ID_ALLG_ERROR,				// Fehler setzen
	ID_ALLG_CLEAR_ERROR,		// Fehler loeschen
	ID_ALLG_POWER_ON_OFF,		// Netzspannungs Versorgung Aus = 0, Netzspannungs Versorgung Ein = 1
	ID_ALLG_ACK,				// Acknowledge; Bestaetigung einer erfolgreichen Uebertragung - Wert: Tx Kommando
								// Wert 0xYYZZ - YY =  Ack Status "ACK = 1, NACK = 0
    							//	 ZZ = CMD
}ID_ALLG;

// Definitionen der Index Nummern fuer die Kommando Page 0: Initialisierung SST Tx / LT Rx 
typedef enum
{
	ID_P0_INIT_START = 0,		// Initialisierung Start - Wert: Anzahl der Init Daten
	ID_P0_INIT_ENDE,			// Initialisierung Ende - Wert: Anzahl der Init Daten
	ID_P0_LEISTUNGSKLASSE,		// Leistungsklasse - Wert: Maximaler Dauerstrom [A]
}ID_P0;	

// Definitionen der Index Nummern fuer die Kommando Page 1: Initialisierung SST Rx / LT Tx
typedef enum
{	
	ID_P1_INIT_START = 0,		// Initialisierung Start - Wert: Anzahl der Init Daten
	ID_P1_INIT_ENDE,			// Initialisierung Ende - Wert: Anzahl der Init Daten
	ID_SW_LT_VERSION_LOW,		// Software Leistungsteil Version Low-Word
	ID_SW_LT_VERSION_HIGH,		// Software Leistungsteil Version High Word
	ID_SW_PFC_VERSION_LOW,		// Software PFC Version Low-Word
	ID_SW_PFC_VERSION_HIGH,		// Software PFC Version High Word
	ID_BG_INDEX,				// Baugruppen Index Nummer
	ID_BG_VARIANTE,				// Hardware Variante
	ID_BG_HW_SW,				// Baugruppe Hardware / Software Kompatibilitaet
	ID_LT_TYP,					// Leistungsteil Typ
	ID_STROM_MAX,				// Maximaler Effektivstrom [A]
	ID_PFC_EXIST,				// PFC Status - Wert: 0 = Kein PFC vorhanden / 1 = PFC vorhanden
}ID_P1;

// Definitionen der Index Nummern fuer die Kommando Page 5: Operation 
typedef enum
{
// schnell, immer im Betrieb 50us
	ID_P5_IST_STROM = 0,		// Ist-Strom [1A]
	ID_P5_IST_SPANNUNG,			// Ist-Spannung [0,1V]

// einmalig, nach jedem "Ein"
	ID_P5_INDUKTIVITAET,		// Schweisskreis Induktivitaet [1 µH]
	ID_P5_WIDERSTAND,			// Schweisskreis Widerstand [1 Milli Ohm]]

// nur bei Aenderung und beim "Ein"
	ID_P5_PFC,					// PFC Status- Wert: 0 = PFC leerlauf; 1 = PFC gestartet

	// Zeitgesteuert
	ID_P5_NETZSPANNUNG,			// Netzspannung [1V]
	ID_P5_NETZSTROM,			// Netzstrom [1A]
	ID_P5_NETZFREQUENZ,			// Netzfrequenz [1Hz]
	ID_P5_UZK_GESAMT,			// Zwischenkreis Spannung Gesamt [1V]
	ID_P5_UZK_MITTE,			// Zwischenkreis Spannung Mitte [1V]
	ID_P5_SPG_15V_1,			// Versorgungsspannung +15V [0,1V]
    ID_P5_SPG_24V_1,			// Versorgungsspannung +24V [0,1V]        
	ID_P5_TEMP_PFC,				// Temperatur Kuehlkoerper PFC  [1°C]
	ID_P5_TEMP_IMS,				// Temperatur Kuehlkoerper IMS  [1°C]

	// no ACK
	ID_P5_SOLL_SPG_U1,			// Soll-Spannung U1
	ID_P5_SOLL_STROM_I1,		// Soll-Strom I1
	ID_P5_SPG_I1_I2,			// Spannung Umschaltung I1 / I2
	ID_P5_TODZEIT_I1_I2,		// Todzeit Umschaltung I1 / I2
	ID_P5_SOLL_SPG_U2,			// Soll-Spannung U2
	ID_P5_SOLL_STROM_I2,		// Soll-Strom I2
	ID_P5_SPG_I2_I1,			// Spannung Umschaltung I2 / I1
	ID_P5_TODZEIT_I2_I1,		// Todzeit Umschaltung I2 / I1
    ID_P5_STROM_MODE,            // Hyperpulse mode - 0 = Inaktiv, 1 = Aktiv 
    ID_P5_PULS_STROM_FAKTOR,    // Strom pulse factor für die I2 berechnung
    ID_P5_PULSE_ZEIT_T1_LOW,        // Pulse frequenz für die HyperPulse
    ID_P5_PULSE_ZEIT_T1_HIGH,        // Pulse frequenz für die HyperPulse
    ID_P5_PULSE_ZEIT_T2_LOW,
    ID_P5_PULSE_ZEIT_T2_HIGH,
    ID_P5_AC_STATUS,            // AC Status = 1, DC Status = 0
    ID_P5_WIG_ZUND_MODE,
    ID_P5_SCHWEISS_VERFAHREN,   // determines the welding process from VK, i.e. MMA or WIG or MIGMAG
// ACK
	ID_P5_LT_ONOFF,				// Leistungsteil Ein-/Ausschalten - Wert: 0 = LT Aus; 1 = LT Ein
	ID_P5_SIMULATION,			// Simulation - 0 = Inaktiv, 1 = Aktiv
}ID_P5;

// Definitionen der Kommunikations Page
typedef enum
{
	INIT_LT =			0x000,		// Page 0: Initialisierung SST Tx / LT Rx
	INIT_SST1 = 		0x100,		// Page 1: Initialisierung SST Rx / LT1 Tx
	INIT_SST2 = 		0x200,		// Page 2: Initialisierung SST Rx / LT2 Tx, nur bei Doppelinverter
	RES1 =				0x300,		// Page 3
	INIT_ERROR =		0x400,		// Page 4
	OPERATION = 		0x500,		// Page 5: Operation
	PRUEFUNG =			0x600,		// Page 6: Pruefung
	UARTBAUD =          0x900,        
	SW_UPDATE = 		0x1400		// Page 7: Software Update 
}PAGE;

// Struktur & Union
// Definition der Daten Struktur von Rx/TX Daten 
typedef struct
{
	unsigned int Cmd;				// Kommando (Siehe Datei "LeistungsteilComKommando.h")
	unsigned int Wert;				// Wert des Kommandos
	unsigned int Ack;				// Sendesteuerung, 0 nein 1 ja, 2 ,3, 4, 5, 6, 7
}UART_DATA;

typedef struct
{
	unsigned int Zeit;				// wie offt will SST
	unsigned int Send;				// wann gesendet

} DATEN;

typedef enum
{
	PFC_EXIST_PFC_NICHT_VORHANDEN = 0,		// PFC ist nicht vorhanden
	PFC_EXIST_PFC_VORHANDEN	=1				// PFC ist vorhanden
}_PFC_EXIST;

#endif	// __LEISTUNGSTEIL_COM_KOMMANDO_H

/**************************************************************************************************
 **************************************************************************************************
 *                                          Dateiende
 **************************************************************************************************
 *************************************************************************************************/
