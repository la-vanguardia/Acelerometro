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
#define LED PORTBbits.RB0

enum Estados{
    ESPERAR,
    CLASIFICAR,
    TRANSMITIR,
    DATOSACELEROMETRO,
    COMUNICACIONCONTINUA
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
unsigned char vcomunicacionAcelerometro[4] = {0}, contadorComunicacionContinua = 0;


void ConfigIni(void);
void ConfigurarPines(void);
void ConfigurarI2C(void);
void ConfigurarGiroscopio();
void ConfigurarRS232();
void ConfigurarTMR();
void ConfigurarFIFO();

void eClasificarTrama(unsigned char *vdatos, unsigned char trama);
void eTransmitirDatos();
void eComunicar();
void eEstadoSiguienteComunicacion();

void aComunicacion(unsigned char *datos);
void aTransmitirDatos(unsigned char *datos);
void aCambiarLed(unsigned char value);
void aComunicacionContinua();

void leerRegistros(unsigned char *datos);

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(){
    IFS0bits.T3IF = 0;
    if(vdatosRX[1] == contador){
        estado = CLASIFICAR;
        contador = 0;

    }
    T3CONbits.TON = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(){
    TMR5 = 0x000;
    IFS1bits.T5IF = 0;
    if(contadorComunicacionContinua == 20){
        estado = DATOSACELEROMETRO;
        contadorComunicacionContinua = 0;
    }
    else{
        contadorComunicacionContinua++;
    }
}



void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(){
    unsigned char datoRX = U1RXREG;
    IFS0bits.U1RXIF = 0;
    TMR3 = 0x0000;
    T3CONbits.TON = 1;

    if(datoRX == TX){
        contador = 1;
        vdatosRX[0] = TX;
    }
    else{
        vdatosRX[contador] = datoRX;
        contador++;
    }
    //tengo 1byte cada 420uS
}


int main(void)
{
    ConfigIni();
    ConfigurarI2C();
    ConfigurarPines();
    ConfigurarRS232();
    ConfigurarTMR();
    ConfigurarGiroscopio();
    unsigned char ID = 0, trama = 0;
    unsigned char vdatos[60] = {0}, vdatos_enviar[70] = {0};

    
    while(1){
        switch(estado){
            case ESPERAR:
                
                break;
            case CLASIFICAR: //Otra maquina de estados
                aObtenerDatos(vdatosRX, vdatos, &trama);
                eClasificarTrama(vdatos ,trama);
                break;
            case TRANSMITIR:
                aCrearVectorEnviar(RX,  vcomunicacionAcelerometro[1], trama, vdatos, vdatos_enviar);
                aTransmitirDatos(vdatos_enviar);
                eTransmitirDatos();
                break;
            case DATOSACELEROMETRO:
                aComunicacion(vdatos);
                eComunicar();
                break;
            case COMUNICACIONCONTINUA:
                aComunicacionContinua();
                eEstadoSiguienteComunicacion();
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
    TRISBbits.TRISB0=0; // LED
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

void ConfigurarFIFO(){
    trasmirDatos(0x80, 1, 0x09, ACELEROMETRO);
    trasmirDatos(0x40, 1, 0x2D, ACELEROMETRO);
}

void ConfigurarTMR(){
    //TIMER VERIFICACION
    PR3 = 0x41A0;
    TMR3 = 0x0000;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    
    //TIMER COMUNICACION CONTINUA
    PR5 = 0x9C40;
    TMR5 = 0x000;
    IFS1bits.T5IF = 0;
    IEC1bits.T5IE = 1;
}

void eClasificarTrama(unsigned char *vdatos, unsigned char trama){
    estado = DATOSACELEROMETRO;
    switch(trama){
        case TODOSREGISTROS:
            vcomunicacionAcelerometro[0] = READ;
            vcomunicacionAcelerometro[1] = cALL;
            vcomunicacionAcelerometro[2] = 0x00;
            break;
            
        case ACELERACIONES:
            vcomunicacionAcelerometro[0] = READ;
            vcomunicacionAcelerometro[1] = 6;
            vcomunicacionAcelerometro[2] = 0x01;
            break;      
            
        case GUARDARDATO:  //FALTA TERMINAR
            vcomunicacionAcelerometro[0] = WRITE;
            vcomunicacionAcelerometro[1] = 1;
            vcomunicacionAcelerometro[2] = vdatos[1];           
            break;
            
        case CONFIGURARFIFO: //FALTA TERMINAR
            ConfigurarFIFO();
            estado = ESPERAR;
            break;
            
        case INICIOCOMUNICACIONCONTINUA:
            estado = COMUNICACIONCONTINUA;
            break;
            
        case DETENERTCOMUNIACIONCONTINUA:
            estado = COMUNICACIONCONTINUA;
            break;
            
        case ENCENDERLED: 
            aCambiarLed(1);
            estado = ESPERAR;
            break;
            
        case APAGARLED:
            aCambiarLed(0);
            estado = ESPERAR;
            break;
            
        case TAPDETECTIONON: //FALTA TERMINAR
            vcomunicacionAcelerometro[0] = WRITE;
            vcomunicacionAcelerometro[1] = 0x00;
            vcomunicacionAcelerometro[2] = 0x00;
            break;
            
        case TAPDETECTIONOFF: //FALTA TERMINAR
            vcomunicacionAcelerometro[0] = WRITE;
            vcomunicacionAcelerometro[1] = 0x00;
            vcomunicacionAcelerometro[2] = 0x00;
            break;
    }
}

void eTransmitirDatos(){
    estado = ESPERAR;
}

void eComunicar(){
    switch(vcomunicacionAcelerometro[0]){
        case READ:
            estado = TRANSMITIR;
            break;
        case WRITE:
            estado = ESPERAR;
            break;
    }
}

void eEstadoSiguienteComunicacion(){
    estado = ESPERAR;
}

void aComunicacion(unsigned char *datos){
    switch(vcomunicacionAcelerometro[0]){
        case READ:
            leerRegistros(datos);          
            break;
        case WRITE:
            trasmirDatos(datos, vcomunicacionAcelerometro[1], vcomunicacionAcelerometro[2], ACELEROMETRO);
            break;
    }
}

void aTransmitirDatos(unsigned char *datos){
    unsigned char i;
    for(i=0; i<datos[1]; i++){
        U1TXREG = datos[i];
        while(U1STAbits.UTXBF == 1);
    }
}

void aComunicacionContinua(){
    if(T5CONbits.TON == 1){
        T5CONbits.TON = 0;
    }
    else{
        TMR5 = 0x0000;
        contadorComunicacionContinua = 0;
        T5CONbits.TON = 1;
        vcomunicacionAcelerometro[0] = READ;
        vcomunicacionAcelerometro[1] = 6;
        vcomunicacionAcelerometro[2] = 0x01;
    }
}

void leerRegistros(unsigned char *datos){
    if(vcomunicacionAcelerometro[1] == cALL){
        U1TXREG = 0x33;
        unsigned char aux[8] = {0}, aux_2[17] = {0}, aux_3[21] = {0};
        unsigned char i;
        recibirDatos(aux, 7, 0x00, ACELEROMETRO);
        recibirDatos(aux_2, 16, 0x09, ACELEROMETRO);
        recibirDatos(aux_3, 20, 0x1D, ACELEROMETRO);
        for(i = 0; i<7; i++){
            datos[i] = aux[i];
        }
        for(i=0; i<16; i++){
            datos[i+9] = aux_2[i];
        }
        for(i=0; i<20; i++){
            datos[i + 29] = aux_3[i];
        }
    }
    else{
        recibirDatos(datos, vcomunicacionAcelerometro[1], vcomunicacionAcelerometro[2], ACELEROMETRO);

    }
}

void aCambiarLed(unsigned char value){
    LED = value;
}