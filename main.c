#include <p18f4550.h>
#include "pwm.h" //inclui biblioteca que programei (arquivo separado)
#include <delays.h>

// Configuração do microcontrolador para execução de instruções
//#define _XTAL_FREQ 8000000
#pragma config IESO     = OFF   /// INTERNAL/EXTERNAL OSCILATOR DISABLE
#pragma config FOSC 	= INTOSC_HS  // quando for p gravar no pic de verdade trocar para INTOSC_HS
#pragma config PWRT     = OFF   /// DISABLE POWER-UP TIMER
#pragma config BORV     = 3     /// BROWN-OUT RESET MINIMUM
#pragma config WDT      = OFF   /// DISABLE WATCHDOG TIMER
#pragma config WDTPS    = 32768 /// WATCHDOG TIMER 32768s
#pragma config MCLRE    = OFF   /// MASTER CLEAR PIN (RE3) DISBALE
#pragma config LPT1OSC  = OFF   /// TIMER1 LOW POWER OPERATION
#pragma config PBADEN   = OFF   /// PORTB.RB0,1,2,3,4 AS I/O DIGITAL
#pragma config STVREN   = ON    /// STACK FULL/UNDERFLOW CAUSE RESET
#pragma config LVP      = OFF   /// DISABLE LOW VOLTAGE PROGRAM (ICSP DISABLE)
#define saidaPWM PORTCbits.RC2 
#define parkin  PORTDbits.RD0


int vagaocupada=0;
void ISR_alta_prioridade(void);

#pragma code int_alta = 0x08


void int_alta()
{
	_asm
		GOTO ISR_alta_prioridade	// Desvia o programa para a função ISR_alta_prioridade.
	_endasm
}
#pragma code
#pragma interrupt ISR_alta_prioridade

void ISR_alta_prioridade()
{
	if (!INTCONbits.TMR0IF);
	else
	{ 
		INTCONbits.TMR0IF=0;
		TMR0H=0xE0;				
		TMR0L=0x00;
		TRISCbits.RC2=~TRISCbits.RC2;
		saidaPWM = 0;
	}
}

void main (void){
 	OSCCON = 0b11110010;  // configura o oscilador interno para 8MHz
	ADCON1=0B00001111;
	TRISB = 0x00;
	PORTB = 0x00;
	TRISD = 0b11111111;

	Inicializa_PWM();                // Inicializa PWM
    Periodo_PWM(52);         // Configura período do PWM
    DutyCycle_PWM(120);

	INTCON2bits.TMR0IP=1;	// programa timer 0 como alta prioridade
	INTCONbits.TMR0IE=1;	// habilita a interrupcao de timer 0

	T0CON = 0b11000100;
	T0CONbits.TMR0ON=1;	// 	Habilita timer0 a contar
	T0CONbits.T08BIT=0;	// 	timer0 com 16 bits
	T0CONbits.T0CS=0;	//	modo timer fonte interna (8Mhz/4=2MHz)
	T0CONbits.PSA=0;	// 	com prescaler
	RCONbits.IPEN=1;		// HABILITA PRIORIDADE
	INTCONbits.GIE=1;		// habilitacao geral

	TMR0L=0xEE;
	TMR0H=0x85;				//valor de recarga para 1 segundo

 
   while(1) 
	{	
	
					
	if(parkin == saidaPWM ){
				PORTBbits.RB2 = 0;
				PORTBbits.RB3 = 1;
			}
			
			else {
				PORTBbits.RB2 = 1;
				PORTBbits.RB3 = 0;


					}
			
		
		}
 	
	}

