#include "p33EP32GS502.h"
#include "6900747_HeaderFile.h"

///////////////////////////DEFINIZIONE VARIABILI GLOBALI//////////////////////////
//	8 BIT non segnate
void FehlerBehandlungTraps(void);
void __attribute__((__interrupt__)) _OscillatorFail(void);
void __attribute__((__interrupt__)) _AddressError(void);
void __attribute__((__interrupt__)) _StackError(void);
void __attribute__((__interrupt__)) _MathError(void);
void __attribute__((__interrupt__)) _AltOscillatorFail(void);
void __attribute__((__interrupt__)) _AltAddressError(void);
void __attribute__((__interrupt__)) _AltStackError(void);
void __attribute__((__interrupt__)) _AltMathError(void);

volatile unsigned int Err;
extern volatile unsigned int Error;
//extern FEHLER Fehler;
extern volatile unsigned int Relais;

#define _INV_aus 0

// Ueber die Zeitverhaeltnis kann man erkennen welcher Fehler aufgetreten ist (1,2,4,8,16 ). Alles in mSek.
void FehlerBehandlungTraps(void)
{
	while(1)
		{
		//Info=!Info;
		Delay_ms(Err);
		Relais=none; 
#ifndef __DEBUG

#endif
		}
}
//

void __attribute__((__interrupt__, no_auto_psv)) _OscillatorFail(void)
{
    INTCON1bits.OSCFAIL = 0;    	  		// Clear the trap flag
	PDC1=_INV_aus;
	SDC2=_INV_aus;
	Err=1;									//Fehlerlabel
	//FehlerBehandlungTraps();
}
//

void __attribute__((__interrupt__, no_auto_psv)) _AddressError(void)
{
	INTCON1bits.ADDRERR = 0;        		//Clear the trap flag
	PDC1=_INV_aus;
	SDC2=_INV_aus;
	Err=4;									//Fehlerlabel
	//FehlerBehandlungTraps();
}
//

void __attribute__((__interrupt__, no_auto_psv)) _StackError(void)
{
	INTCON1bits.STKERR = 0;         		//Clear the trap flag
	PDC1=_INV_aus;
	SDC2=_INV_aus;
	Err=8;									//Fehlerlabel
	//FehlerBehandlungTraps();
}
//

void __attribute__((__interrupt__, no_auto_psv)) _MathError(void)
{
	INTCON1bits.MATHERR = 0;        		//Clear the trap flag
	PDC1=_INV_aus;
	SDC2=_INV_aus;
	Err=16;			
	return;									//bei DEBUGen kann eingeschaltet werden
	//FehlerBehandlungTraps();
}
//

void __attribute__((__interrupt__, no_auto_psv)) _AltOscillatorFail(void)
{
	INTCON1bits.OSCFAIL = 0;
	PDC1=_INV_aus;
	SDC2=_INV_aus;
	Err=32;									//Fehlerlabel
	//FehlerBehandlungTraps();
}
//

void __attribute__((__interrupt__, no_auto_psv)) _AltAddressError(void)
{
	INTCON1bits.ADDRERR = 0;
	PDC1=_INV_aus;
	SDC2=_INV_aus;
	Err=64;									//Fehlerlabel
	//FehlerBehandlungTraps();
}
//

void __attribute__((__interrupt__, no_auto_psv)) _AltStackError(void)
{
	INTCON1bits.STKERR = 0;
	PDC1=_INV_aus;
	SDC2=_INV_aus;
	Err=7;									//Fehlerlabel
	//FehlerBehandlungTraps();
}
//

void __attribute__((__interrupt__, no_auto_psv)) _AltMathError(void)
{
	INTCON1bits.MATHERR = 0;
	PDC1=_INV_aus;
	SDC2=_INV_aus;
	Err=8;									//Fehlerlabel
	//FehlerBehandlungTraps();
}
//
