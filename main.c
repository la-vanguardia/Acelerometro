#include <p33fj128GP202.h>
#include <stdlib.h>
#include <stdio.h>
#define FCY 40000000UL//
#include <libpic30.h>

_FICD(ICS_PGD2 & JTAGEN_OFF); // Para hacer debuging por el puerto 2
_FOSCSEL(FNOSC_FRC);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FWDT(FWDTEN_OFF);
#pragma config ALTI2C=ON // uso los pines alternativos de i2c

#include "I2C.h"


#define ACELEROMETRO 0x1C
#define DIRECCION_ID 0x0D


unsigned char contador=0, datosRX[20] = {'\0'}, bandera = 0;
unsigned char datosAcelerometro[50] = {0};


void ConfigIni(void);
void ConfigurarPines(void);
void ConfigurarI2C(void);
void ConfigurarGiroscopio();
void ConfigurarRS232();
void EnviarRS232 ( unsigned char *text);

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(){
    unsigned char datoRX = U1RXREG;
    IFS0bits.U1RXIF = 0;
    if(datoRX == 0x0D){  //0x0D Finaliza el mensaje
        bandera = 1;
        contador = 0;
    }
    else{
        datosRX[contador] = datoRX;
        contador++;
    }
}


int main(void)
{
    ConfigIni();
    ConfigurarI2C();
    ConfigurarPines();
    ConfigurarRS232();
    ConfigurarGiroscopio();
    U1TXREG = 'H';
   
    while(1){
        if(bandera == 1){
            bandera = 0;
            recibirDatos(datosAcelerometro , 2, 0x01, ACELEROMETRO);
          
            EnviarRS232(datosAcelerometro);
                    
             /*U1TXREG = datosAcelerometro[0];
             __delay_ms(20);
             U1TXREG = datosAcelerometro[1];*/
        }
        
        
    }
}

void ConfigIni (void) {
    // Configure Oscillator to operate the device at 40 MHz
    // Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
    // Fosc = 8M * 40/(2 * 2) = 80 MHz for 8M input clock

    PLLFBD = 40; 			    // M = 40
    CLKDIVbits.PLLPOST = 0;		// N2 = 2
    CLKDIVbits.PLLPRE = 0; 		// N1 = 2
    __builtin_write_OSCCONH(0x03);	// Initiate Clock Switch to Primary Oscillator with PLL (NOSC = 0b011)
    __builtin_write_OSCCONL(0x01);
    while(OSCCONbits.COSC != 0b011);// Wait for Clock switch to occur
    while(OSCCONbits.LOCK != 1){};	// Wait for PLL to lock
    

    OSCCON = 0x46;			// Command Sequence- Registro de control del oscilador
    OSCCON = 0x57;
    OSCCONbits.IOLOCK = 0;		// Peripherial pin select is not locked

}

void ConfigurarPines(){
    AD1PCFGL=0xFFFF;
    ADPCFG = 0xFFFF;
}

void ConfigurarI2C(void){ 
    I2C1BRG = 0x5D; // Velocidad para 400KHz y 40MHz (Ecuacion 19-1)   
    I2C1CONbits.I2CEN = 1;    
    IFS1bits.SI2C1IF = 0;  //Bandera de interrupcion I2C esclavo
    I2C1CONbits.DISSLW = 1;   //Desabilitar Slew Rate
    I2C1CONbits.IPMIEN = 0;
}

void ConfigurarGiroscopio(){
    iniciarI2C();
    iniciarComunicacion(ACELEROMETRO, WRITE);
    trasmitirDato(0x2A); //Registro para despertar el acelerometro
    trasmitirDato(0x09); //Active mode
    detenerI2C();
}

void ConfigurarRS232(){
   
    RPINR18bits.U1RXR = 0b01000;
    RPOR3bits.RP7R = 0b00011;

    U1BRG = 129;
    
    U1STAbits.UTXISEL1 = 1;
   
    U1MODEbits.UARTEN = 1; 
    IEC0bits.U1RXIE=1; //Interrupcion por recepcion
    U1STAbits.UTXEN = 1;  
  
    __delay_us(60); 
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;

}

void EnviarRS232 ( unsigned char *text){
    unsigned char i=0;
    
    for (i=0; i<Longitud;i++){
        
        U1TXREG = text[i];
        __delay_ms(20);
          
    }   
}