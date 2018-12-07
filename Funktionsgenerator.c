/*
===============================================================================
Name        	: Funktionsgenerator_Stromann_Hiljegerdes.c
Autoren     	: Oliver Stromann und Arne Hiljegerdes
Version     	: Final
Copyright   	: Oliver Stromann und Arne Hiljegerdes
Beschreibung	: Funktionsgenerator für Sinus-, Dreieck- und Rechteckfunktionen
Frequenzbereich	: 1Hz bis 2000Hz
Datum			: 18.12.2014
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include "i2c.h"
#include "lcdlib_armcm3.h"

#define RGB_RGB (0b111 << 7)	//all three RGB channels
#define RGB_R	(0b010 << 7)	//red channel
#define RGB_G	(0b100 << 7)	//green channel
#define RGB_B	(0b001 << 7)	//blue channel

#define OUTPUT (1 << 27)		//activate output 0 -> digital output for triggers
#define MR0I1 (1 << 0)       	//Interrupt on MR0: an interrupt is generated when MR0 matches the value in the TC (timercounter von Timer0).
#define MR0R1 (1 << 1)        	//Reset on MR0: the TC will be reset if MR0 matches it.
#define TIM0_ENABLE (1 << 0)  	//enable Timer0

#define TV 0					//RC button TV on
#define OK_KEY 41				//RC button confirm
#define VOLUME_UP 16			//RC button volume up
#define VOLUME_DOWN 17			//RC button volume down

#define PI 3.14159265

//list different signal options
enum signale
{
	sine = 1, tri, saw
};

//list change states (flags)
enum change_state
{
	changed_signal = 1, changed_frequency = 2, changed_amplitude = 4
};


//----------GLOBAL VARIABLES--
uint16_t sinus_arr[90];
uint8_t signal = 0;
uint16_t amplitude = 0;
uint16_t frequency = 1;
uint16_t timestep = 0; // IRQ support vectors for output function (poiting to array index)
uint16_t output = 0; // output value on DAC

uint8_t rc5_code = 0;
uint8_t rc5_address = 0;
uint8_t rc5_new_key = 0;	//toggled flip-bit -> new key pressed
uint8_t rc5_flag = 0; 		//flag to indicate new rc5 values

//----------PROTOTYPES---------
//Control functions
void
	delay (uint16_t i);

//Initialising functions
void
	init_all (void);
void
	init_sinus_arr (void);
void
	init_RGB (void);
void
	init_level_led (void);
void
	init_lcd (void);
void
	init_digital_out (void);
void
	init_ADC (void);
void
	init_DAC (void);
void
	init_RC5 (void);
void
	init_RIT (void);

//Input functions
void
	input (uint8_t *changes);
void
	readADC (uint8_t *changes);
void
	readRC5 (uint8_t *changes, uint8_t *temp_signal);
uint8_t
	readTaster (uint8_t tasterbit);

//GUI functions
void
	gui (uint8_t *changes);
void
	rgb (void);
void
	write_lcd (uint8_t *changes);
void
	level_led (void);

//Output functions
void
	output_func ();
void
	writeDAC ();
uint16_t
	sinus ();
uint16_t
	triangle ();
uint16_t
	sawtooth ();


int
	main ()
{
	uint8_t changes = 0b111;	//check changes

	init_all ();				//boot up

	do
	{
		input (&changes);		//read input
		gui (&changes);			//set GUI
	}
	while (1);

	return 0;
}

/*delay
* void delay (uint_16 time)
* Summary:	delay time
*
* Parameters:	time: number of cycles
*/
void
	delay (uint16_t time)
{
	volatile uint16_t i = 0;

	for (i = 0; i < time; i++)
		;
}

//---------Initialising Functions-----------------
/*Create table of sine values
* void init_sinus_arr(void)
* Summary:	writes 90 sine values in 1 degree steps into a table
*
* Global variables: 
*	-output: uint16_t sinus_arr[90]: Table of sin values. Range of values: 0 - 512.
*/
void
	init_sinus_arr ()				// 512: DAC has 1024 possible values - sine values alternate around mean; + offset of 512
{					// Maximum genommen für gute Auflösung - daraus kleinere Amplitufen ableiten später im Aufruf
	uint8_t i;
	float rad = PI / 180;	//radiant-value für 1°

	for (i = 0; i < 90; i++)
		sinus_arr[i] = 512 * sin (i * rad);	//sinus-Werte für 0 bis 90° berechnen (maximale Aussteuerung!)

}
/*Initialisierung
* void init_all (void)
* Summary:	Initialisierung aller verwendeten Baugruppen
*
*/
void
	init_all (void)
{
	init_sinus_arr ();			//sinus array initialisieren

	//Display initialiseren
	lcd_init (4);		//LC Display initialisieren
	lcd_clrscr ();		//LC Display löschen
	init_lcd ();			//LC Display initialisieren mit festen Werten

	//rote LEDs initialisieren
	init_level_led ();
	init_RGB ();
	init_digital_out ();
	init_ADC ();
	init_DAC ();
	init_RC5 ();
	init_RIT ();
}
/*Initialisierung des Displays mit festen Werten
* void init_lcd (void)
* Summary:	Signal, Frequency und Amplitude aufs Display schreiben
*			Diese Stellen des Displays werden danach nicht mehr beschrieben
*
*/
void
	init_lcd (void)
{
	//zeilenweise Ausgabe strings
	char zeile1[20] = "Signal: ";
	char zeile2[20] = "";
	char zeile3[20] = "Frequency:       Hz";
	char zeile4[20] = "Amplitude:       mV";

	lcd_write_string_xy (zeile1, 1, 1); //String in Reihe 1 an 1. Stelle schreiben
	lcd_write_string_xy (zeile2, 1, 2);	//Schreiben Reihe 2 Stelle 1
	lcd_write_string_xy (zeile3, 1, 3);	//Schreiben Reihe 3 Stelle 1
	lcd_write_string_xy (zeile4, 1, 4);	//Schreiben Reihe 4 Stelle 1
}
/*Initialisierung der roten LEDs
* void level_led_init(void)
* Summary:	Portexpander an denen die roten LEDs hängen als output definieren
*/
void
	init_level_led (void)
{

	I2C_init ();

	I2CWriteLength = 4;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = 0x20 | 0;			//MAX7311 + write
	I2CMasterBuffer[1] = 0x06;			//Configuration Register
	I2CMasterBuffer[2] = 0x00;			//Port1 als Output definieren
	I2CMasterBuffer[3] = 0x00;			//Port2 als Output definieren
	I2C_engine ();
}
/*Initialisierung Digital-Analog-Wandlers
* void init_DAC(void)
* Summary:	-
*/
void
	init_DAC (void)
{

	LPC_SC->PCLKSEL0 &= ~(3 << 22);    // Takt aktivieren   TAB40

	LPC_PINCON->PINSEL1 &= ~(1 << 20);   // Funktion GPIO Port festlegen (AOUT)	-> Table 80
	LPC_PINCON->PINSEL1 |= (1 << 21);	// dito

	LPC_PINCON->PINMODE1 |= (1 << 21);	// Festlegung KEIN Pullup/ Pulldown Widerstand /DAC-Funktion)
	LPC_PINCON->PINMODE1 &= ~(1 << 20);	// dito -> Table 87
}
/*Initialisierung Analog-Digital-Wandlers
* void init_ADC(void)
* Summary:	-
*/
void
	init_ADC (void)
{
	LPC_SC->PCONP |= 1 << 12;			//Stromversorgung ein
	LPC_SC->PCLKSEL0 |= 1 << 24;		//jeden 8. Tkt abfragen (vorher 3)

	LPC_ADC->ADCR |= 1 << 21;			//bereitstellen
	LPC_ADC->ADCR |= 0b11111111 << 8;			// 1111.. + 1 = 0 -> immer noch jeden 8. Takt abfragen

	LPC_PINCON->PINSEL1 |= 1 << 14;		//Funktion auf AD0.0 festlegen
	LPC_PINCON->PINSEL1 &= ~(1 << 15);	//dito
	LPC_PINCON->PINMODE1 &= ~(1 << 14);	//weder pullup noch pulldown-widerstand
	LPC_PINCON->PINMODE1 |= 1 << 15;	//dito
}
/*Initialisierung Digitalen Ausgangs A0
* void init_digital_out(void)
* Summary:	-
*/
void
	init_digital_out (void)
{
	LPC_GPIO1->FIODIR = OUTPUT; // ...1.... Da Wert auf 1 -> A[0] als Ausgang definiert
	LPC_GPIO1->FIOMASK &= ~OUTPUT; //...0..... Maske-> Alle ungenutzten Ports werden deaktiviert
	LPC_GPIO1->FIOCLR = OUTPUT;      	//A0 ausschalten
}
/*Initialisierung Repetitive Interrupt Timers
* void init_RIT(void)
* Summary:	-
*/
void
	init_RIT (void)
{
	// RIT Konfigurieren
	LPC_SC->PCONP |= (1 << 16);		//RIT mit Spannung versorgen	-> Table 46
	LPC_SC->PCLKSEL1 |= (1 << 26);	//CLK ohne Vorteiler( direkt tak von CPU 100MHz)
	LPC_SC->PCLKSEL1 &= ~(1 << 27);	// "

	LPC_RIT->RICTRL &= ~(1 << 3);	//Disable Timer -> RITEN Table 436 vor init erst einmal ausschalten
	//277778 = 360 Hz -> 1 Hz
	//27778 = 3600 Hz -> 10 Hz
	//2778 = 36000 Hz -> 100 Hz
	//1389 = 72000 Hz -> 200 Hz danach ist Schluss! Zeit für ISR ist länger als die Zeit bis der nächste IR kommt Abarbeitung dauert länger
	//ab dann Stützstellen verringern!
	LPC_RIT->RICOMPVAL = 277778; //Vergleichswert: 100MHz -> 1 Takt 10 ns -> 360 Hz müssen haben -> Periodenlänge 1/360 =2,777777778 ms -> / 10 ns = 277778 -> 360 Hz entspricht durch stützstellen 1 Hz außen
	LPC_RIT->RICOUNTER = 0;	//Counter zurück setzen

	LPC_RIT->RICTRL |= (1 << 1);	//CTRL Register einschalten	RITENCLR Table 436 // damit register aktiv machen
	LPC_RIT->RICTRL |= (1 << 3);	//Enable TimerRITEN Table 436

	NVIC_EnableIRQ (RIT_IRQn);	//Interrupt für RIT einschalten

}
/*Initialisierung des Timer0 und des P0.15 für das RC5-Protokoll
* void init_RC5(void)
* Summary:	-
*/
void
	init_RC5 (void)
{
	//IR-Eingang
	LPC_PINCON->PINMODE0 |= 2 << 30; 	//Pull-Down an Pin 0.15

	//Timer0 der CPU-clock verwenden
	LPC_SC->PCONP |= (1 << 1); 		//Timer0 einschalten -> Power Control Bit für Timer 0 setzen (Tabelle 46)
	LPC_SC->PCLKSEL0 &= ~(3 << 2);  	//Vorteiler: 4 -> 40 ns pro Takt

	LPC_TIM0->MR0 = 7404; //Interrupt Match Register0 100 MHz/4 = 25 MHz -> 1/25 MHZ = 40 ns -> 1,778 ms (rc5) 6 mal abtasten -> 1,778ms/(40ns*6)=7404
	LPC_TIM0->MCR |= (MR0I1) | (MR0R1); //Interrupt auslösen, wenn TC (T0) = MR0 und TC zurücksetzen, vgl. UM10360 -> Table 429
	// Steuersignale an das Register MCR

	LPC_TIM0->TCR |= TIM0_ENABLE;        //einschalten Timer0

	NVIC_EnableIRQ (TIMER0_IRQn);        //Interrupt für Timer0 einschalten
}
/*Initialisierung der RGB-LED
* void init_RGB(void)
* Summary:	-
*/
void
	init_RGB (void)
{

	LPC_GPIO2->FIODIR |= RGB_RGB;		//RGB-Kanäle aktivieren
	LPC_GPIO2->FIOMASK &= ~RGB_RGB;		//ungenutze Ports ausmaskieren
	LPC_GPIO2->FIOSET = RGB_RGB;		//RGB-LED ausschalten
}

//---------Interrupthandler------------------
/*Interruptandler des Repetitive Interrupt Timers
* void RIT_IRQHandler(void)
*
* Global variables:
*	-input: uint16_t frequency	(1...2000)
*			uint16_t timestep	(0...359)
*
*
* Summary:	-Vergleichsregister des RIT-Timers bzw. timestep-intervall an Frequenz anpassen
*			-Timestep inkrementieren
*			-Digitalen Ausgang entsprechend ein-/ausschalten
*			-Berechnung der Ausgabewerte aufrufen
*			-beschreiben des DAC aufrufen
*/
void
	RIT_IRQHandler (void)
{
	static uint8_t addtimestep = 1;

	LPC_RIT->RICTRL |= 1;		//clear interrupt flag eieruhr soll aufhören zu klingeln
	if (frequency < 200)// Stützstellenintervall = 1; alles kann über MAtchregister RICOMPVAR einstellbar keine Schwierigkeiten weil abhandlung IR HAndler vor nächstem IR fertig
	{
		LPC_RIT->RICOMPVAL = 277778 / frequency; // wert 1HZ durch eingestellte frequenz
		addtimestep = 1;			// Stützstellenintervall
	}
	else
	{					// BEreich COMPVAL  wird zu klein neuer IR in Routine

		addtimestep = frequency / 100;	// Stützstellenintervall  verändern (vergrößern)
		LPC_RIT->RICOMPVAL = 277778 * addtimestep / frequency;	// wenn f=202 -> addt=2 --> doppllet so schnell bei 101 Hz =^ 202 Hz

	}

	timestep = timestep + addtimestep;	//nächster timestep-intervall für jedes Grad einmal in RIT reinspringen
	timestep = timestep % 360;	//start again

	if (timestep >= 0 && timestep < 180)
		LPC_GPIO1->FIOPIN |= OUTPUT; //einschalten digitalen Ausgang A0
	else if (timestep >= 180)
		LPC_GPIO1->FIOPIN &= ~OUTPUT; //ausschalten digitalen Ausgang A0

	output_func ();	//Wert der an DAC ausgegeben werden soll berechnen berechnte globale output var ruft auch sine tri und saw auf

	writeDAC ();		//Wert an DAC ausgeben
}
/*Interrupthandler für Timer0 für RC5-Protokoll
* void TIMER0_IRQHandler(void)
*
* Global Variables:
*	-output:	uint8_t rc5_code	(0...63)
*				uint8_t rc5_address	(0...31)
*				uint8_t rc5_flag	(0,1)
*				uint8_t rc5_new_key	(0,1)
*		
*
* Summary:	-Einlesen der RC5-Schnittstelle
*			-Bei erfolgreichem Lesen: beschreiben der globalen Variablen rc5_code und rc5_address
*			-Vorhandensein von neuen Daten über rc5_flag signalisieren
*/
void
	TIMER0_IRQHandler (void)
{ // static - nachdem Timeraufruf alten wert behalten
	static uint8_t bitcounter = 0; // zählen der 14 bits
	static uint8_t taktcounter = 0; // 6 mal abtastung
	static uint8_t letztesbit = 0; // zuletzt gelesene Bit aus Datenstrom
	static uint8_t startflag = 0; // signalisiert das gerade etwas ankommt (erste steigende Flanke)
	static uint8_t rc5_code_tmp = 0;
	static uint8_t rc5_address_tmp = 0;
	static uint8_t rc5_flip_tmp = 0;
	static uint8_t rc5_flip = 0;
	uint8_t aktuellesbit = 0;

	LPC_TIM0->IR |= (1 << 0);     //Interruptflag von Timer0 zurücksetzen

	if (rc5_flag == 0)	//keine Daten mehr da, die abgeholt werden müssen?
	{	//Wenn bereit für neue RC5-Übertragung
		aktuellesbit = ((LPC_GPIO0->FIOPIN & (1 << 15)) >> 15) ^ 1; // Port einlesen abtastung des ports

		//Start
		if ((aktuellesbit == 1) && (startflag == 0)) // Neuer Datenstrom
		{
			startflag = 1;
			taktcounter = 6;		 // Abtastung sicherstellung Startmoment richtig erfassen und über halbbit 0 hinaus
			bitcounter = 0;
			rc5_flip_tmp = 0;
			rc5_code_tmp = 0;
			rc5_address_tmp = 0;
		}

		if ((letztesbit != aktuellesbit) && (taktcounter > 3)) //Flankenwechsel oberhalb der Mitte
		{
			//erkanntesbit = aktuellesbit;
			bitcounter++;
			taktcounter = 0;
			//Code-Bereich
			if (bitcounter >= 9)
			{
				rc5_code_tmp |= (aktuellesbit << (14 - bitcounter));

			}
			else if (bitcounter == 3)
			{
				rc5_flip_tmp = aktuellesbit;
			}
			//Address-Bereich
			else if ((bitcounter >= 4) && (bitcounter <= 8))
			{
				rc5_address_tmp |= (aktuellesbit << (8 - bitcounter));
			}

		}
		else if (taktcounter > 10)
		{
			taktcounter = 0;
			bitcounter = 0;
			startflag = 0;
		}

		if (bitcounter == 14)
		{
			if (rc5_flip_tmp != rc5_flip)
			{
				rc5_new_key = 1;
				rc5_flip = rc5_flip_tmp;
			}
			else
				rc5_new_key = 0;

			rc5_code = rc5_code_tmp;
			rc5_address = rc5_address_tmp;
			rc5_flag = 1; //Fertig!
			bitcounter = 0;
			startflag = 0;
		}

		//Takt weiterzählen
		if (startflag)
		{
			taktcounter++;
		}

		letztesbit = aktuellesbit; // für Flankenerkennung notwendig
	}
}

//---------Input Functions-----------------
/*Eingabefunktionen
* void input (uint8_t *changes)
* 
* globale Variable:
*	-in-/output:	uint8_t signal (sine, tri, saw) 
*					uint8_t rc5_flag (0,1)
*
* Eingangsvariable:	uint8_t *changes (0,changed_signal, changed_amplitude, changed_frequency)
*
* Ausgangsvariable:	uint8_t *changes (0,changed_signal, changed_amplitude, changed_frequency)
*
* Summary:	Aufruf aller Eingabefunktionen für Taster, AD-Wandler und IR-Fernbedienung
*/
void
	input (uint8_t *changes)
{
	uint8_t temp_signal = signal; //Kopie alter Signalwert
	readADC (changes);

	if (rc5_flag)	//daten verfügbar?
	{
		readRC5 (changes, &temp_signal); // gesetzt wenn 14bit da sind
		rc5_flag = 0;	// muss wieder auf 0 setzen
	}

	if (readTaster (1) == 0)
		temp_signal = sine;
	if (readTaster (2) == 0)
		temp_signal = tri;
	if (readTaster (3) == 0)
		temp_signal = saw;
	if (temp_signal != signal)
	{
		*changes |= changed_signal;  // siganl hat geändert
		signal = temp_signal; // in die globale var
	}
}
/*Taster auslesen
* uint8_readTaster (uint8_t tasterbit)
*
* Eingangsvariable:	uint8_t tasterbit (1,2,3)
*
* Ausgangsvariable:	uint8_t value (0,1)
*
* Summary:	Auslesen der Tasterwerte
*/
uint8_t
	readTaster (uint8_t tasterbit)
{
	uint8_t value = 0;
	value = (LPC_GPIO2->FIOPIN & (1 << (tasterbit + 2)));
	delay (1000); // Tasterentprellung
	return value;
}
/*AD-Wandler auslesen
* void readADC (uint8_t *changes)
*
* globale Variable:
*	-in-/output:	uint16_t amplitude (0...4095)
*
* Ausgangsvariable:	uint8_t *changes (0, changed_amplitude)
*
* Summary:	Auslesen der Werte des AD-Wandler
* 			Tiefpassfilterung
* 			Ergebnis in Amplitude speichern
*/
void
	readADC (uint8_t *changes)
{
	uint16_t temp_amplitude;
	float p, q;
	q = 0.7;	//Gewichtung alter Wert  (TIEFPASSFILTER) (je größer q desto träääger)
	p = 1 - q;	//Gewichtung neuer Messwert

	LPC_ADC->ADCR |= 1;					//Kanal0 verwenden
	LPC_ADC->ADCR |= 1 << 21; 	//The A/D converter is operational ->Table 531 betriebsbereit
	LPC_ADC->ADCR |= 0b001 << 24; 	//Startbit setzen -> Table 531 Start now!
	LPC_ADC->ADCR &= ~(0b110 << 24); 	// dito

	while (!(LPC_ADC->ADGDR & 1 << 31))
		;	//Warten bis AD Wandler fertig (Done-Bit = 1)

	LPC_ADC->ADCR &= ~(0b111 << 24);		//Startbit löschen
	LPC_ADC->ADCR &= ~1;			//Kanal0 löschen
	LPC_ADC->ADCR &= ~(1 << 21);	//The A/D converter is in power-down mode

	//Tiefpassfilterung (Vermeidung der Flimmers der LEDs)
	temp_amplitude = amplitude * q + ((LPC_ADC->ADGDR >> 4) & 0xfff) * p;	// neuer Wert = alter Wert * q + neuer Messwert *p
	amplitude = temp_amplitude;
	*changes |= changed_amplitude;
}
/*RC5-Protokoll auswerten
* void readRC5 (uint8_t *changes, uint8_t *temp_signal)
*
* globale Variable:
*	-in-/output:	uint16_t frequency (1...2000)
*	-input:			uint8_t rc5_code	(0...63)
*					uint8_t rc5_address	(0...31)
*					uint8_t rc5_new_key	(0,1)
*
* Ausgangsvariable:	uint8_t *changes (0, changed_signal, changed_frequency)
*						uint8_t *temp_signal (0, sine, tri, saw)
*	
*
* Summary:	Auslesen der Adresse und des Codes der RC5-Übertragung
*			entsprechende Änderung durchführen und unter changes eintragen
*
*/
void
	readRC5 (uint8_t *changes, uint8_t *temp_signal)		// Auswertung des RC5 nach gesetzten rc5_flag
{
	static uint8_t addfrequency = 1;	// Schritte wie schnell Frequenz höher
	static uint16_t freq_arr[4];
	static uint8_t digit = 0;
	static uint8_t eingabemodus = 0;
	uint16_t tmp_frequency;
	uint8_t i, j;

	if ((rc5_address == TV))			// TV-Gerät
	{
		if (!eingabemodus)	//normaler Modus?
			switch (rc5_code)
			// was passiert bei welcher gedrückten Taste
		{
			case VOLUME_UP:
				//Volume up
				if (!rc5_new_key)			// gedrückt halten Vol+ vorher shon mal gedrpckt
				{
					if (addfrequency <= 50)
						addfrequency *= 2; // max um 32 werte springen (vorspulen)
				}
				else
					// noch nciht vorher gedrpckt
					addfrequency = 1; // frequenz nur um 1 erhöhen
				frequency += addfrequency; //  gewünschte freqz einstellen
				if (frequency > 2000)		// BEgrenzung
					frequency = 2000;
				*changes |= changed_frequency;

				break;
			case VOLUME_DOWN:
				//Volume down
				if (!rc5_new_key)
				{
					if (addfrequency <= 50)
						addfrequency *= 2;
				}
				else
					addfrequency = 1;
				frequency -= addfrequency;
				if (frequency < 1 || frequency > 2000) // Überlaufschutz
					frequency = 1;
				*changes |= changed_frequency;
				break;
			case 1:		// Signalauswahl
				// Taste 1
				*temp_signal = sine;
				*changes |= changed_signal;
				break;
			case 2:
				// Taste 2
				*temp_signal = tri;
				*changes |= changed_signal;
				break;
			case 3:
				// Taste 3
				*temp_signal = saw;
				*changes |= changed_signal;
				break;
			case OK_KEY:
				//OK-Taste
				if(rc5_new_key)
				{
					eingabemodus = 1;
					lcd_write_string_xy ("    ", 11, 3);	//Frequenzanzeige löschen
					lcd_gotoxy (14,3);	//cursor auf niedrigste Frequenzstelle setzen
					lcd_cursor (2);	//cursormode blinkend
				}
				break;

			default:
				break;
		}
		else if (rc5_new_key)	//Eingabemodus: Reaktion auf einzelne Tastendrücke
		{

			if (rc5_code == OK_KEY)	//OK-Taste erneut betätigt
			{
				lcd_cursor (0);	//cursormode nicht mehr blinkend
				eingabemodus = 0;
				tmp_frequency = 0;

				//tmp_frequency
				for (i = digit; i >= 1; i--)
				{
					for (j = digit; j > i; j--)
						freq_arr[i - 1] *= 10;	//dezimalstellen berechnen

					tmp_frequency += freq_arr[i - 1];
					freq_arr[i - 1] = 0;
				}

				if (tmp_frequency > 2000)
					frequency = 2000;
				else if (tmp_frequency < 1)
					frequency = 1;
				else
					frequency = tmp_frequency;

				*changes |= changed_frequency;
				digit = 0;

			}
			else if (rc5_code >= 0 && rc5_code <= 9)
			{
				if (digit < 4)
					freq_arr[digit] = rc5_code;
				//printf ("freq_arr[%d]: %d\n", digit, rc5_code);
				for (i = 0; i <= digit; i++)
				{
					lcd_gotoxy(14-digit+i,3);
					lcd_write_uint(freq_arr[i],1);
				}
				digit++;
				lcd_gotoxy(14,3);

			}

		}

	}
}

//---------Grafische Ausgabe-Funktionen---------
/*Graphical User Interface
* void gui(uint8_t *changes)
*
* Eingangsvariable: uint8_t *changes (0, changed_signal, changed_frequency, changed_amplitude)
* Ausgangsvariable: uint8_t *changes (0)
*
* Summary:	Aufruf aller Ausgabefunktionen für LCD, rote LEDs und RGB-LED
*/
void
	gui (uint8_t *changes)
{
	rgb ();			//Signalart auf RGB-LED ausgeben

	level_led ();	//Amplitude auf roten LEDs ausgeben

	if (*changes) 		// Wenn immer neu schreiben viel zu langsam!
	{
		write_lcd (changes);			//alle Infos auf LCD-Display ausgeben
		*changes = 0;	//changes_flag rücksetzen
	}
}
/*Red-Green-Blue-LED
* void rgb(void)
*
* globale Variablen:	
*	- input: uint8_t signal (0, sine, tri, saw)
*		
* Summary:	Statusanzeige auf RGB-LED
*/
void
	rgb (void)
{
	if (signal == sine) 		//sinus lila
	{
		LPC_GPIO2->FIOCLR = RGB_R;	//rot an
		LPC_GPIO2->FIOCLR = RGB_B;	//blau an
		LPC_GPIO2->FIOSET = RGB_G;	//grün aus
	}
	else if (signal == tri)	//Dreieck blau
	{
		LPC_GPIO2->FIOCLR = RGB_B;	//blau an
		LPC_GPIO2->FIOSET = ~RGB_B;	//rest aus
	}
	else if (signal == saw)	//Sägezahn grün
	{
		LPC_GPIO2->FIOCLR = RGB_G;	//grün an
		LPC_GPIO2->FIOSET = ~RGB_G;	//rest aus
	}
	else
		LPC_GPIO2->FIOSET = RGB_RGB;	//alles aus
}
/*Pegel auf roten LEDs ausgeben
* void level_led(void)
*
* globale Variablen:
*	- input: amplitude (0 ...4095)
*
* Summary:	Stellt die Amplitude auf den roten LEDs dar
*/
void
	level_led ()
{
	uint8_t rled_highbyte, rled_lowbyte, i;  // port2, port1, laufvariable
	uint16_t max_amplitude = 0xfff;
	uint16_t var = 0;

	//Aufteilen der Amplitude in 16 Teilbereich
	for (i = 1; i < 17; i++)
	{
		if (amplitude > (max_amplitude / 16) * i)	//Wenn Amplitude als i-ter Teilbereich
			var |= (1 << (16 - i));	//weitere LED anmachen auf 15 schieben erste LED weil bit 15 (anzeige verdreht)
		else
			//Sonst Abbruch
			i = 17;
	}

	rled_lowbyte = var;	//unteren 8 bit
	rled_highbyte = (var >> 8);	//oberen 8 bit

	// I²C KOmmunikation
	I2CWriteLength = 4;	//4 Byte schreiben
	I2CReadLength = 0;	//0 Byte lesen

	//Registerschreibbefehle
	I2CMasterBuffer[0] = 0x20 | 0;			//MAX7311 + write
	I2CMasterBuffer[1] = 0x02;				//Output Register
	I2CMasterBuffer[2] = ~rled_lowbyte;		//Wert an Port[1]
	I2CMasterBuffer[3] = ~rled_highbyte;		//Wert der über Port[2]an LED's anliegt
	I2C_engine ();	//Über I²C-Bus senden

}
/*LC-Display beschreiben
* void write_lcd(uint8_t *changes)
*
* globale Variablen:
*	-input: uint8_t signal (sine, tri, saw)
*			uint16_t frequency (1...2000)
*			uint16_t amplitude (0...4095)
*
* Eingangsvariable: uint8_t *changes (changed_signal, changed_frequency, changed_amplitude) 
*	
* Summary:	Werte bei Änderungen aufs Display schreiben
*
*/
void
	write_lcd (uint8_t *changes)
{
	float var;

	//Änderung des Signals
	if (changed_signal & *changes)
	{
		//Zeile 1 - Signalart
		if (signal == sine)
			lcd_write_string_xy ("sine    ", 9, 1);
		else if (signal == tri)
			lcd_write_string_xy ("triangle  ", 9, 1);
		else if (signal == saw)
			lcd_write_string_xy ("sawtooth ", 9, 1);
	}

	//Änderung der Frequenz
	if (changed_frequency & *changes)
	{
		//Zeile 3 - Frequenz
		char frqstr[10];
		snprintf (frqstr, 6, "%04d", frequency);	//cast integer to string
		lcd_write_string_xy (frqstr, 11, 3);

	}

	//Änderung der Amplitude
	if (changed_amplitude & *changes)
	{
		var = (amplitude / 4.095) * 1.5;	//Konvertierung auf 4095 -> 1500 mV

		//Zeile 4 - Amplitude
		char ampstr[10];
		snprintf (ampstr, 7, "%.1f", var);	//cast float to string
		lcd_write_string_xy (ampstr, 11, 4);
	}
}

//---------Ausgabe-Funktionen-------------------
/*Ausgabewert für DA-Wandler holen
* void output_func(void)
*
* globale Variablen:
*	-output:	uint16_t output (0...1023)
*	-input:		uint8_t signal (0, sine, tri, saw)			
*
* Summary:	Aufruf der entsprechenden Signal-Funktionen
*
*/
void
	output_func ()
{
	switch (signal)
	{
	case sine:
		output = sinus ();
		break;
	case tri:
		output = triangle ();
		break;
	case saw:
		output = sawtooth ();
		break;
	default:
		output = 511;	// alter wert übernehmen falls signal fehlerhaft
		break;
	}
}
/*Sinusfunktion
* uint16_t sinus(void)
*
* globale Variablen:
*	-output:	uint16_t output (0...1023)
*	-input:		uint16_t timestep (0...359)
*				uint16_t amplitude (0...4095)
*
* Summary:	Sinuswerte aus Tabelle holen
*			Je nach Position (in viertelwellen) Addition oder subtraktion vom Mittelwert
*			bzw. positiver oder negativer durchlauf der Sinustabelle
*
*/
uint16_t
	sinus ()
{

	float temp_amp = (float) amplitude / 4095;	//skalieren (max. amplitde ADC)

	//erste Viertelwelle (0-90°)
	if (timestep >= 0 && timestep < 90)
	{
		output = sinus_arr[timestep];	//positiv max sine value (0 to 90)
		output = 511 + output * (temp_amp);	//offset + ^^ * factor
	}

	//zweite Viertelwelle (90°-180°)
	else if (timestep >= 90 && timestep < 180)
	{
		output = sinus_arr[90 - (timestep - 89)];	//positiv max sine value  ( 90 to 0)
		output = 511 + output * (temp_amp);		//offset + ^^ * factor
	}

	//dritte Viertelwelle (180°-270°)
	else if (timestep >= 180 && timestep < 270)
	{
		output = sinus_arr[(timestep - 180)];		//positiv max sine value ( 0 to 90)
		output = 511 - output * (temp_amp);	// offset - ^^ * factor
	}

	//vierte Viertelwelle (270°-360°)
	else if (timestep >= 270 && timestep < 360)
	{
		output = sinus_arr[90 - (timestep - 269)];	//positiv max sine value  ( 90 to 0)
		output = 511 - output * (temp_amp);		// offset - ^^ * factor
	}

	return output;
}
/*Sägezahnfunktion
* uint16_t sawtooth(void)
*
* globale Variablen:
*	-output:	uint16_t output (0...1023)
*	-input:		uint16_t timestep (0...359)
*				uint16_t amplitude (0...4095)
* Summary:	Berechnen der Sägezahnfunktion
*/
uint16_t
	sawtooth ()
{

	float temp_amp = (float) amplitude / 4095;	//skalieren

	//erste Halbwelle (0-180°)
	if (timestep >= 0 && timestep < 180)
	{
		output = 511 - temp_amp * 511 + timestep * temp_amp / 180 * 511;

	}
	//zweite Halbwelle (180°-360°)
	else if (timestep >= 180 && timestep < 360)
	{
		output = 511 + (timestep - 180) * temp_amp / 180 * 511;
	}

	return output;
}
/*Dreieckfunktion
* uint16_t triangle(void)
*
* globale Variablen:
*	-output:	uint16_t output (0...1023)
*	-input:		uint16_t timestep (0...359)
*				uint16_t amplitude (0...4095)
*
* Summary:	Berechnen der Dreieckfunktion
*/
uint16_t
	triangle ()
{
	float temp_amp = (float) amplitude / 4095;	//skalieren

	//erste Viertelwelle (0-90°)
	if (timestep >= 0 && timestep < 90)
	{
		output = (511 + timestep * temp_amp / 90 * 511);
	}

	//zweite Viertelwelle (90°-180°)
	else if (timestep >= 90 && timestep < 180)
	{
		output = (511 + temp_amp * 511) - ((timestep - 90) * temp_amp / 90 * 511);
	}

	//dritte Viertelwelle (180°-270°)
	else if (timestep >= 180 && timestep < 270)
	{
		output = (511 - (timestep - 180) * temp_amp / 90 * 511);

	}

	//vierte Viertelwelle (270°-360°)
	else if (timestep >= 270 && timestep < 360)
	{
		output = (511 + temp_amp * 511) - ((360 - (timestep - 90)) * temp_amp / 90 * 511);
	}

	return output;
}
/*DA-Wandler beschreiben
* void writeDAC(void)
*
* globale Variablen:
*	-input: uint16_t output (0...1023)
*
* Summary:	Registereintrag vorbereiten für Ausgabe am DA-Wandler
*/
void
	writeDAC ()
{
	//output_register vorbereiten	
	uint_32 output_reg = LPC_DAC->DACR ; //alten Werte aus Register holen

	output &= ~(0xfc00);	//maskieren auf 10 Bit (damit andere Werte niht überschrrieben werden)
	output_reg |= (output << 6);	//Value-Bereich des Registers: einsen reinschieben
	output |= (0xfc00);	//bit 10 - 16 mit einsen beschreiben
	output_reg &= (output << 6);	//Value-Bereich des Registers: nullen reinschieben

	//schreiben
	LPC_DAC->DACCTRL |= (0b11 << 1); //DBLBUF_ENA und CNT_ENA Table 540
	LPC_DAC->DACR = output_reg;	// werte auf DAC ausgeben
}
