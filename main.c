
#include "MKL25Z4.h"
#include "smt160_kl25.h"
#include "drv_lcd.h"
#include <stdio.h>
#include "drv_gpio.h"
#include "drv_systick.h"
#include <string.h>

#define topeni (42)

/// definice tlacitek
#define sw1 (4)
#define sw2 (5)
#define sw3 (16)
#define sw4 (17)


//prototypy funkci
void init(void);
static inline int IsKeyPressed(int pin);
int potenciometr(void);
void nastav_displej(char text[]);
void nastav_displej2(int r3, int r4);
void ADCInit(void);
uint32_t ADCCalibrate(void);
uint32_t startTime;


static int i = 0;
char buff[32];
char sbuff[32];
int num;

void delay(void) {
	uint32_t n;
	for ( n=0; n<1000000; n++) {
		;
	}
}


// deklarace pomocnych
int mod=0;
int pozadovanaTep=0;
int pom=0;



int main(void)
{
	short teplota;
	/* Inicializace ovladace snimace teploty */
	smt160_init();
	/*inicializace gpio*/
	GPIO_Initialize();
	// Inicializace A/D prevodniku
		ADCInit();
		// Kalibrace a nova inicializace
		// Pro dosazeni udavane presnosti musi byt prevodnik kalibrovan po
		// kazdem resetu.
		// Nova inicializace je potreba protoze pri kalibraci
		// je prevodnik prenastaven.
		ADCCalibrate();
		ADCInit();
		// Nastaveni pinu, kde je pripojen potenciometr,
		// do rezimu vstupu pro A/D prevodnik: pin PTC2, funkce ALT0
		// 1. Povolime hodinovy signal pro port C
		SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
		// 2. NAstavime funkci pinu na ALT0 = vstup A/D prevodniku
		PORTC->PCR[2] = PORT_PCR_MUX(0);

	/* Inicializace pinu pouziteho pro ovladani topeni jako vystup */
	/* Enable Port C clock (pin used for output ) */
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	GPIOE_PDDR |= (1 << 31);
	PORTE_PCR31 = PORT_PCR_MUX(1); /* E31 set to function Alt1 (GPIO)*/

	LCD_initialize();
	LCD_clear();
	init();
	LCD_clear();
	nastav_displej("sw1=start");

	while (1) {
		if (mod==0){
			if (IsKeyPressed(sw1)){	//start programu - skok do vyberu teploty potakem
			mod = 1;
			pozadovanaTep=potenciometr();
			}
		}


		if(pozadovanaTep>1){

			if(IsKeyPressed(sw3) && mod==2){ //zmacknout 3 pro spusteni programu
				int timer=0;
				while(1){


					teplota = smt160_get_temp(); //cteni teploty

					sprintf(buff, "%02d.%02d", teplota/100, teplota%100); //zobrazuje aktualni teplotu
					LCD_clear();
					LCD_puts(buff);
					nastav_displej2(0,1)
					delay(); //kdy se najde zpusob aby zobrazovani aktualni teploty nepreblikavalo tak se delay odstrani - efektivnejsi //212

					if (timer==0) { //spousteni systicku 
						startTime = SYSTICK_millis();
						timer=1;
					}

					/*
					if ((teplota/100) >=(pozadovanaTep)){
						GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
					}
					else if ((teplota/100)<pozadovanaTep){
						GPIOE_PDOR |= (1 << 31);	// TopOn();
					}
					*/

					int rozdil=pozadovanaTep-(teplota/100); // rozdil kolik zbyva dotopit

					if ((teplota/100) >=(pozadovanaTep)){
						GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
					}

					if(rozdil>=5){
						GPIOE_PDOR |= (1 << 31);	// TopOn();
						if (SYSTICK_millis() - startTime > 2000){
							GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
							timer=0;
							}
					}
					if(rozdil>=1 && rozdil<4){
						GPIOE_PDOR |= (1 << 31);	// TopOn();
						if (SYSTICK_millis() - startTime > 500){
							GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
							timer=0;
							}
					}
					if(rozdil>=0.5 && rozdil<0.9){
						GPIOE_PDOR |= (1 << 31);	// TopOn();
						if (SYSTICK_millis() - startTime > 200){
							GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
							timer=0;
							}
					}
					if(rozdil>=0.1 && rozdil<0.4){
						GPIOE_PDOR |= (1 << 31);	// TopOn();
						if (SYSTICK_millis() - startTime > 100){
							GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
							timer=0;
							}
					}







					if(IsKeyPressed(sw4)){ //nouzove vypnuti sw4
						GPIOE_PDOR &= ~(1 << 31); 	// TopOff();
						pozadovanaTep=0;
						mod=0;
						LCD_clear();
						nastav_displej("sw1=start");
						break;
						}

				}//log.reg

			}
		}


	}
	/* Never leave main */
	return 0;
}



int potenciometr(void){  //potenciometr voli teplotu -- dodelat
	int ctenizpotaku;
	while(1){
		ADC0->SC1[0] = ADC_SC1_ADCH(11);
		// Cekame na dokonceni prevodu
		while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0 )
			;
		// Ulozime vysledek prevodu
		uint16_t vysledek = ADC0->R[0];

		if (vysledek > 150) { //potrange = 0-1023
					ctenizpotaku=450;//30
				}

				if (vysledek > 300) {
					ctenizpotaku=500;//30
				}

				if (vysledek > 450) {
					ctenizpotaku=550;//30
				}
				if (vysledek > 600) {
					ctenizpotaku=600;//40
				}
				if (vysledek > 750) {
					ctenizpotaku=650;//43
				}
				if (vysledek > 800) {//60
					ctenizpotaku=750;
				}
				if (vysledek > 900) {//60
					ctenizpotaku=700;
				}
				if (vysledek > 950) {//60
					ctenizpotaku=800;
				}
				if (vysledek > 1010) {//60
					ctenizpotaku=900;
				}

		sprintf(buff, "teplota je: %02d.%02d", (ctenizpotaku/15), ctenizpotaku%100); //zobrazuje aktualni teplotu
		LCD_clear();
		LCD_puts(buff);
		delay(); //////////////212 - tlacitko sw2 se musi chvili podrzet protoze je delay

		if (ctenizpotaku<400){ //pokud je mensi nez 400 (26stupnuC) tak se nastavi pozadovana na 26C
			ctenizpotaku=401;
		}else{
			pozadovanaTep=ctenizpotaku;
		}

		pozadovanaTep=(pozadovanaTep/15);

		if (IsKeyPressed(sw2)){  //zmacknout 2 pro potvrzeni volby + na displej
			nastav_displej("teplota zvolena");
			nastav_displej2(1,0);
			mod=2;
			return pozadovanaTep; // vraci teplotu v stupnich C
		}

	}

}


/**
 * DEKLARACE FUNKCI
 * */

void nastav_displej(char text[])
{

	static char stary_text[32];
	if (strcmp(stary_text, text) != 0) {
	LCD_clear();
	LCD_set_cursor(2, 1);
	LCD_puts(text);
	}
}

void nastav_displej2(int r3,int r4)
{

	static char stary_text[32];
	if (strcmp(stary_text, text) != 0) {
	LCD_clear();
	if (r3=1){
		LCD_set_cursor(3, 1);
		LCD_puts("sw3=potvrzeni volby teploty");
	}
	if (r4=1){
		LCD_set_cursor(4, 1);
		LCD_puts("sw4=nouzove ukonceni");
		}
	}
}



void init(void){
	SYSTICK_initialize();//inicializace systicku
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTB_MASK |
		SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK);

	//set pin function to gpio
	PORTA->PCR[sw1] = PORT_PCR_MUX(1);
	PORTA->PCR[sw2] = PORT_PCR_MUX(1);
	PORTA->PCR[sw3] = PORT_PCR_MUX(1);
	PORTA->PCR[sw4] = PORT_PCR_MUX(1);

	//smer pinu na vstupni
	PTA->PDDR &= ~(1 << sw1);
	PTA->PDDR &= ~(1 << sw2);
	PTA->PDDR &= ~(1 << sw3);
	PTA->PDDR &= ~(1 << sw4);

}


static inline int IsKeyPressed(int pin)
{
	if ((PTA->PDIR & (1 << pin)) == 0)
		return 1;
	else
		return 0;
}

void ADCInit(void)
{
	// Povolit hodinovy signal pro ADC
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// Zakazeme preruseni, nastavime kanal 31 = A/D prevodnik vypnut, jinak by zapisem
	// doslo ke spusteni prevodu
	// Vybereme single-ended mode
	ADC0->SC1[0] =  ADC_SC1_ADCH(31);

	// Vyber hodinoveho signalu, preddelicky a rozliseni
	// Clock pro ADC nastavime <= 4 MHz, coz je doporuceno pro kalibraci.
	// Pri max. CPU frekvenci 48 MHz je bus clock 24 MHz, pri delicce = 8
	// bude clock pro ADC 3 MHz
	ADC0->CFG1 = ADC_CFG1_ADICLK(0)		/* ADICLK = 0 -> bus clock */
		| ADC_CFG1_ADIV(3)				/* ADIV = 3 -> clock/8 */
		| ADC_CFG1_MODE(2);				/* MODE = 2 -> rozliseni 10-bit */

	// Do ostatnich registru zapiseme vychozi hodnoty:
	// Vybereme sadu kanalu "a", vychozi nejdelsi cas prevodu (24 clocks)
	ADC0->CFG2 = 0;

	// Softwarove spousteni prevodu, vychozi reference
	ADC0->SC2 = 0;

	// Hardwarove prumerovani vypnuto
	ADC0->SC3 = 0;	/* default values, no averaging */

}


uint32_t ADCCalibrate(void)
{
	 unsigned short cal_var;

	  ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;	/* Enable Software Conversion Trigger for Calibration Process */
	  ADC0->SC3 &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK );    /* set single conversion, clear avgs bitfield for next writing */

	  ADC0->SC3 |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(32) ); /* turn averaging ON and set desired value */

	  ADC0->SC3 |= ADC_SC3_CAL_MASK;      /* Start CAL */

	  /* Wait calibration end */
	  while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0 )
		  ;

	  /* Check for Calibration fail error and return */
	  if ( (ADC0->SC3 & ADC_SC3_CALF_MASK) != 0 )
		  return 1;

	  // Calculate plus-side calibration
	  cal_var = 0;
	  cal_var =  ADC0->CLP0;
	  cal_var += ADC0->CLP1;
	  cal_var += ADC0->CLP2;
	  cal_var += ADC0->CLP3;
	  cal_var += ADC0->CLP4;
	  cal_var += ADC0->CLPS;

	  cal_var = cal_var/2;
	  cal_var |= 0x8000; // Set MSB
	  ADC0->PG = ADC_PG_PG(cal_var);

	  // Calculate minus-side calibration
	  cal_var = 0;
	  cal_var =  ADC0->CLM0;
	  cal_var += ADC0->CLM1;
	  cal_var += ADC0->CLM2;
	  cal_var += ADC0->CLM3;
	  cal_var += ADC0->CLM4;
	  cal_var += ADC0->CLMS;

	  cal_var = cal_var/2;
	  cal_var |= 0x8000; // Set MSB
	  ADC0->MG = ADC_MG_MG(cal_var);

	  ADC0->SC3 &= ~ADC_SC3_CAL_MASK;

	  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
