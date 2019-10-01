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
#include "comunicacion.h"

#define cALL 50
#define ACELEROMETRO 0x1C
#define DIRECCION_ID 0x0D
#define TX 0xFD
#define RX 0xFE


enum Estados{
    ESPERAR,
    CLASIFICAR,
    TRANSMITIR,
    DATOSACELEROMETRO
};

enum Tramas{
    TODOSREGISTROS = 0x41,
    ACELERACIONES,
    GUARDARDATO,
    CONFIGURARFIFO,
    INICIOCOMUNICACIONCONTINUA,
    DETENERTCOMUNIACIONCONTINUA,
    ENCENDERLED,
    APAGARLED,
    TAPDETECTIONON,
    TAPDETECTIONOFF
};


unsigned char contador=0, vdatosRX[20] = {'\0'}, bandera = 0, estado = ESPERAR, numero_datos = 0;
unsigned char vdatosAcelerometro[50] = {0}, vdatos_enviar[100] = {0};


void ConfigIni(void);
void ConfigurarPines(void);
void ConfigurarI2C(void);
void ConfigurarGiroscopio();
void ConfigurarRS232();
unsigned char obtenerTrama();
void eClasificarTrama(unsigned char trama, unsigned char *estado);
void eTransmitirDatos();


void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(){
    unsigned char datoRX = U1RXREG;
    IFS0bits.U1RXIF = 0;
    if(datoRX == 0x0D){
        bandera = 1;
        contador = 0;
    }
    else{
        vdatosRX[contador] = datoRX;
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
    unsigned char ID = 0, trama = 0;
    
   
    while(1){
        if(bandera == 1){
            bandera = 0;
            recibirDatos(vdatosAcelerometro , 2, 0x01, ACELEROMETRO);
             U1TXREG = vdatosAcelerometro[0];
             __delay_ms(20);
             U1TXREG = vdatosAcelerometro[1];
        }
        switch(estado){
            case ESPERAR:
                
                break;
            case CLASIFICAR: //Otra maquina de estados
                trama = obtenerTrama();
                eClasificarTrama();
                break;
            case TRANSMITIR:
                eTransmitirDatos();
                break;
            case DATOSACELEROMETRO:
                
                break;
            default:
                estado = ESPERAR;
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

void ConfigurarI2C(void){ //TABLE 10.1 acelerometro
    //address 7bits
    I2C1BRG = 0x5D; // 0x5D   
    I2C1CONbits.I2CEN = 1;    
    IFS1bits.SI2C1IF = 0;
    I2C1CONbits.DISSLW = 1;   
    I2C1CONbits.IPMIEN = 0;
}

void ConfigurarGiroscopio(){
    iniciarI2C();
    iniciarComunicacion(ACELEROMETRO, WRITE);
    trasmitirDato(0x2A);
    trasmitirDato(0x09);
    detenerI2C();
}

void ConfigurarRS232(){
    
    
    RPINR18bits.U1RXR = 0b01000;
    RPOR3bits.RP7R = 0b00011;

    U1BRG = 129;
    
 
    
    U1STAbits.UTXISEL1 = 1;
   
    U1MODEbits.UARTEN = 1; // ENABLE
    IEC0bits.U1RXIE=1; //interrupcion por recepcion
    U1STAbits.UTXEN = 1;  // HABILITA TRANSMISION
  
    __delay_us(60); 
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;

}

unsigned char obtenerTrama(){
    unsigned char datos[50] = {0};
    obtenerDatos(vdatosRX, datos);
    return datos[0]; //depende de orden
}

void eClasificarTrama(unsigned char trama, unsigned char *estado){
    switch(trama){
        case TODOSREGISTROS:
            recibirDatos(vdatosAcelerometro, cALL, 0x00, ACELEROMETRO);
            numero_datos = cALL;
            estado[0] = TRANSMITIR;
            break;
            
        case ACELERACIONES:
            recibirDatos(vdatosAcelerometro, 6, 0x00, ACELEROMETRO);
            numero_datos = 6;
            estado[0] = TRANSMITIR;
            break;      
            
        case GUARDARDATO:
            
            break;
            
        case CONFIGURARFIFO:
            
            break;
            
        case INICIOCOMUNICACIONCONTINUA:
            
            break;
            
        case DETENERTCOMUNIACIONCONTINUA:
            
            break;
            
        case ENCENDERLED:
            
            break;
            
        case APAGARLED:
            
            break;
            
        case TAPDETECTIONON:
            
            break;
            
        case TAPDETECTIONOFF:
            
            break;
    }
}

