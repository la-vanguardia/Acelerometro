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

#define cALL 50
#define ACELEROMETRO 0x1C
#define DIRECCION_ID 0x0D
#define TX 0xFD
#define RX 0xFE
#define LED PORTBbits.RB0
#define ACTIVE 0x01
#define SLEEP 0x00


#include "I2C.h"
#include "comunicacion.h"
#include "WIFI.h"



enum Estados{
    ESPERAR,
    CLASIFICAR,
    TRANSMITIR,
    DATOSACELEROMETRO,
    COMUNICACIONCONTINUA,
    TAP
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
    TAPDETECTIONOFF,
    COMANDOWIFI, // 0x4B
    SETEARMODO // 0x4C
};

enum Transmision{
    WIFI,
    NRF,
    USB
};

enum Interrupcion{
    INTFIFO,
    INTTAP,
    INTNINGUNA
};

typedef struct comunicacionAcelerometro{
    unsigned char accion;
    unsigned char dirreccion;
    unsigned char numero_datos;
}comunicacion_t;

unsigned char contador=0, vdatosRX[20] = {'\0'}, bandera = 0, estado = ESPERAR, numero_datos = 0;
unsigned char contadorComunicacionContinua = 0, contadorTAP = 0 ,trama = 0;
unsigned char tiempo = 0;
unsigned char estado_interrupcion = INTNINGUNA, modo_trasmision = USB;
unsigned char vDatosTAP[121] = {0}, head = 0;

unsigned char bandera_dato = 0;
comunicacion_t vInformacionAcelerometro;


unsigned char  contadorU2 = 0, contador_t3, aux = 0;

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
void eTAP();

void aComunicacion(unsigned char *datos);
void aTransmitirDatos(unsigned char *datos);
void aCambiarLed(unsigned char value);
void aComunicacionContinua();
void aCargarTAP();
void aLeerFifo();
void aLeerFifoWIFI();

void leerRegistros(unsigned char *datos);
void ConfigurarTapDetection(unsigned char on);
void EscrituraAcelerometro(unsigned char dato, unsigned char dirreccion);
unsigned char leerID();

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(){
    IFS0bits.T3IF = 0;
    unsigned char i;
    if(vdatosRX[1] == contador){
        estado = CLASIFICAR;
        
        
    }
    contador = 0;
    T3CONbits.TON = 0;
    bandera_dato = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(){
    TMR5 = 0x0000;
    IFS1bits.T5IF = 0;
    if(contadorComunicacionContinua == 20){
        estado = DATOSACELEROMETRO;
        contadorComunicacionContinua = 0;
        vInformacionAcelerometro.accion = READ;
        vInformacionAcelerometro.numero_datos = 6;
        vInformacionAcelerometro.dirreccion = 0x01;
        trama = INICIOCOMUNICACIONCONTINUA;
    }
    else{
        contadorComunicacionContinua++;
    }
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(){
    TMR2 = 0x0000;
    IFS0bits.T2IF = 0;
    if(contadorTAP == 20){
        contadorTAP = 0;
        estado = TAP;
    }
    else{
        contadorTAP++;
    }   
}


void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(){
    unsigned char datoRX = U1RXREG;
    IFS0bits.U1RXIF = 0;
    
    TMR3 = 0x0000;
    if( contador == 0){
        if(datoRX == TX){
           contador = 1;
           vdatosRX[0] = TX; 
        }    

    }
    else{
        vdatosRX[contador] = datoRX;
        contador++;
    }
    //tengo 1byte cada 420uS
    T3CONbits.TON = 1;
    
    
    
}

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(){
    unsigned char datoRX = U2RXREG, i;
    IFS1bits.U2RXIF = 0;
    T3CONbits.TON = 0;
    TMR3 = 0x0000; 
    
    if(bandera_dato == 1){
        vdatosRX[contador] = datoRX;
        contador++;
        T3CONbits.TON = 1;
    }
    
    switch(datoRX){
        case '>':
            bandera_send = 1;
            break;
        case ':':
            bandera_dato = 1;
            contador=0;
            break;
        case 'K':
            bandera_ok = 1;
            break;
    }
}


int main(void)
{
    
    ConfigIni();
    ConfigurarI2C();
    ConfigurarPines();
    ConfigurarRS232();
    ConfigurarTMR();
    ConfigurarWIFI();
    
    unsigned char ID = 0, ID_ans = 0;
    unsigned char vdatos[60] = {0}, vdatos_enviar[70] = {0};
    
    ID = leerID();
    LED = 0;
    if(ID == 0x1A){
        bandera = 0xFF;
        ConfigurarGiroscopio();
    }
    
    while(1){
        switch(estado & bandera){
            case ESPERAR:
                /*iniciarI2C();
                iniciarComunicacion(ACELEROMETRO, WRITE);
                trasmitirDato(0x00);
                resetearI2C();
                iniciarComunicacion(ACELEROMETRO, READ);
                ID = recibirDato(1);
                if(ID == 0xA0){
                    switch(estado_interrupcion){
                        case INTFIFO: //fifo
                            switch(modo_trasmision){
                                case WIFI:
                                    aLeerFifoWIFI();
                                    break;
                                case USB:
                                    aLeerFifo();
                                    break;
                                case NRF:
                                    
                                    break;
                            }
                            
                            EscrituraAcelerometro(SLEEP, 0x2A);
                            EscrituraAcelerometro(0x00, 0x09);
                            EscrituraAcelerometro(ACTIVE, 0x2A);
                            estado_interrupcion = INTNINGUNA;
                            break;
                    }
                }*/
                break;
            case CLASIFICAR: //Otra maquina de estados
                aObtenerDatos(vdatosRX, vdatos, &trama);
                eClasificarTrama(vdatos ,trama);
                break;
            case TRANSMITIR:
                aCrearVectorEnviar(RX,  vInformacionAcelerometro.numero_datos, trama, vdatos, vdatos_enviar);
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
            case TAP:
                aCargarTAP();
                eTAP();
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
    RPINR0bits.INT1R = 4;
    IFS1bits.INT1IF = 0;
    
}

void ConfigurarI2C(void){ 
    I2C1BRG = 0x5D; // Velocidad para 400KHz y 40MHz (Ecuacion 19-1)   
    I2C1CONbits.I2CEN = 1;    
    IFS1bits.SI2C1IF = 0;  //Bandera de interrupcion I2C esclavo
    I2C1CONbits.DISSLW = 1;   //Desabilitar Slew Rate
    I2C1CONbits.IPMIEN = 0;
}

void ConfigurarGiroscopio(){

    EscrituraAcelerometro(SLEEP, 0x2A);
    
    EscrituraAcelerometro(0x00, 0x2D); 
    EscrituraAcelerometro(ACTIVE, 0x2A);
}

void ConfigurarRS232(){
    
    IPC16bits.U1EIP = 0b110;
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

void ConfigurarFIFO(){
    
    EscrituraAcelerometro(SLEEP, 0x2A);
    EscrituraAcelerometro(0x80, 0x09);
    EscrituraAcelerometro(0x00, 0x2D);
    EscrituraAcelerometro(0x00, 0x2E);
    EscrituraAcelerometro(ACTIVE, 0x2A);
}

void ConfigurarTMR(){
    //TIMER VERIFICACION
    //PR3 = 0x41A0;
    PR3 = 0xFFFF;
    TMR3 = 0x0000;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    
    //TIMER COMUNICACION CONTINUA
    PR5 = 0x9C40;
    TMR5 = 0x0000;
    IFS1bits.T5IF = 0;
    IEC1bits.T5IE = 1;
    
    //TIMER TAP (500uS)
    PR2 = 0x4E20;
    TMR2 = 0x0000;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
}

void eClasificarTrama(unsigned char *vdatos, unsigned char trama){
    estado = DATOSACELEROMETRO;
    switch(trama){
        case TODOSREGISTROS:
            vInformacionAcelerometro.accion = READ;
            vInformacionAcelerometro.numero_datos = cALL;
            vInformacionAcelerometro.dirreccion = 0x00;
            break;
            
        case ACELERACIONES:
            vInformacionAcelerometro.accion = READ;
            vInformacionAcelerometro.numero_datos = 6;
            vInformacionAcelerometro.dirreccion = 0x01;
            break;      
            
        case GUARDARDATO:  
            vInformacionAcelerometro.accion = WRITE;
            vInformacionAcelerometro.numero_datos = 1;
            vInformacionAcelerometro.dirreccion = vdatos[0];
            vdatos[0] = vdatos[1];
            break;
            
        case CONFIGURARFIFO: //FALTA TERMINAR
            ConfigurarFIFO();
            estado_interrupcion = INTFIFO;
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
            estado_interrupcion = INTTAP;
            ConfigurarTapDetection(1);
            estado = ESPERAR;
            break;
            
        case TAPDETECTIONOFF: //FALTA TERMINAR
            ConfigurarTapDetection(0);
            estado = ESPERAR;
            break;
        case COMANDOWIFI:
            enviarComando(vdatos); //SE DEBE ENVIAR EL +
            estado = ESPERAR;
            break;
        case SETEARMODO:
            modo_trasmision = vdatos[0];
            estado = ESPERAR;
            break;
    }
}

void eTransmitirDatos(){
    estado = ESPERAR;
}

void eComunicar(){
    switch(vInformacionAcelerometro.accion){
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

void eTAP(){
    estado = ESPERAR;
}

void aComunicacion(unsigned char *datos){
    switch(vInformacionAcelerometro.accion){
        case READ:
            if(T5CONbits.TON == 1){
                T5CONbits.TON = 0;
                leerRegistros(datos);
                datos[6] = tiempo;
                tiempo++;
                vInformacionAcelerometro.numero_datos = 7;
                trama = INICIOCOMUNICACIONCONTINUA;
                
            }
            else{
                leerRegistros(datos);          
            }
            break;
        case WRITE:
            EscrituraAcelerometro(SLEEP, 0x2A);
            trasmirDatos(datos, vInformacionAcelerometro.numero_datos, vInformacionAcelerometro.dirreccion, ACELEROMETRO);
            EscrituraAcelerometro(ACTIVE, 0x2A);
            break;
    }
}

void aTransmitirDatos(unsigned char *datos){
    unsigned char i;
    switch(modo_trasmision){
        case WIFI:
            enviarDato(datos);
            break;
        case NRF:
            
            break;
        default:
            for(i=0; i<datos[1]; i++){
                U1TXREG = datos[i];
                while(U1STAbits.UTXBF == 1);
            }
    }
    
    if( trama == INICIOCOMUNICACIONCONTINUA){
        T5CONbits.TON = 1;
    }
}

void aComunicacionContinua(){
    if(T5CONbits.TON == 1){
        T5CONbits.TON = 0;
    }
    else{
        TMR5 = 0x0000;
        contadorComunicacionContinua = 0;
        tiempo = 0;
        T5CONbits.TON = 1;
        vInformacionAcelerometro.accion = READ;
        vInformacionAcelerometro.numero_datos = 6;
        vInformacionAcelerometro.dirreccion = 0x01;
    }
}

void leerRegistros(unsigned char *datos){
    if(vInformacionAcelerometro.numero_datos == cALL){
        unsigned char aux[8] = {0}, aux_2[17] = {0}, aux_3[21] = {0};
        unsigned char i;
        recibirDatos(aux, 7, 0x00, ACELEROMETRO);
        recibirDatos(aux_2, 16, 0x09, ACELEROMETRO);
        recibirDatos(aux_3, 20, 0x1D, ACELEROMETRO);
        for(i = 0; i<7; i++){
            datos[i] = aux[i];
        }
        for(i=0; i<16; i++){
            datos[i+8] = aux_2[i];
        }
        for(i=0; i<20; i++){
            datos[i + 29] = aux_3[i];
        }
    }
    else{
        recibirDatos(datos, vInformacionAcelerometro.numero_datos, vInformacionAcelerometro.dirreccion, ACELEROMETRO);

    }
}

void ConfigurarTapDetection(unsigned char on){
    if(on == 1){
                
        EscrituraAcelerometro(SLEEP, 0x2A);
        EscrituraAcelerometro(0x15, 0x21);
     
        EscrituraAcelerometro(0x0C, 0x23);
        EscrituraAcelerometro(0x0C, 0x24);
        EscrituraAcelerometro(0x0C, 0x25);
        
        EscrituraAcelerometro(0x50, 0x26);
        EscrituraAcelerometro(0xF0, 0x27);
        
        EscrituraAcelerometro(0x08, 0x2D);
        EscrituraAcelerometro(0x08, 0x2E);
        
        EscrituraAcelerometro(ACTIVE, 0x2A);
    }
    else{
        EscrituraAcelerometro(SLEEP, 0x2A);
        EscrituraAcelerometro(0x00, 0x2D);
        EscrituraAcelerometro(ACTIVE, 0x2A);
    }
}

void aCambiarLed(unsigned char value){
    LED = value;
}

void aLeerFifo(){
    unsigned char i;
    unsigned char aux = 0;
    U1TXREG = RX;
    while(U1STAbits.UTXBF == 1);
    U1TXREG= 195;
    while(U1STAbits.UTXBF == 1);
    U1TXREG = CONFIGURARFIFO;
    vInformacionAcelerometro.dirreccion = 0x01;
    iniciarI2C();
    iniciarComunicacion(ACELEROMETRO, WRITE);
    trasmitirDato(vInformacionAcelerometro.dirreccion);
    
    resetearI2C();
    iniciarComunicacion(ACELEROMETRO, READ);
    for(i=0; i<192; i++){
       
        aux = recibirDato(0);
        if(i != (192 - 1) ){
            I2C1CONbits.ACKDT = 0; // ACK
            I2C1CONbits.ACKEN = 1; // habilitador ACK
            while(I2C1CONbits.ACKEN == 1);
        }
        
        while(U1STAbits.UTXBF == 1);
        U1TXREG = aux;
        
    }
    detenerI2C();
}

void aLeerFifoWIFI(){
    enviarComando("+CIPSEND=195");
    unsigned char i;
    unsigned char aux = 0;
    while(bandera_send == 0);
    bandera_send = 0;
    U2TXREG = RX;
    while(U2STAbits.UTXBF == 1);
    U2TXREG = 195;
    while(U2STAbits.UTXBF == 1);
    U2TXREG = CONFIGURARFIFO;
    vInformacionAcelerometro.dirreccion = 0x01;
    iniciarI2C();
    iniciarComunicacion(ACELEROMETRO, WRITE);
    trasmitirDato(vInformacionAcelerometro.dirreccion);
    
    resetearI2C();
    iniciarComunicacion(ACELEROMETRO, READ);
    for(i=0; i<192; i++){
       
        aux = recibirDato(0);
        if(i != (192 - 1) ){
            I2C1CONbits.ACKDT = 0; // ACK
            I2C1CONbits.ACKEN = 1; // habilitador ACK
            while(I2C1CONbits.ACKEN == 1);
        }
        
        while(U2STAbits.UTXBF == 1);
        U2TXREG = aux;
        
    }
    while(U2STAbits.UTXBF == 1);
    detenerI2C();
}


void aCargarTAP(){
    unsigned char aux[7] = {0}, i;
    vInformacionAcelerometro.accion = READ;
    vInformacionAcelerometro.numero_datos = 6;
    vInformacionAcelerometro.dirreccion = 0x01;
    leerRegistros(aux);
    for(i=0; i<6; i++){
        if(head == 119){
            head = 0;
        }
        head++;
        vDatosTAP[ head ] = aux[i]; 
    }
}

void EscrituraAcelerometro(unsigned char dato, unsigned char dirreccion){
    iniciarI2C();
    iniciarComunicacion(ACELEROMETRO, WRITE);
    trasmitirDato(dirreccion);
    trasmitirDato(dato);
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN == 1);
}

unsigned char leerID(){
    unsigned char ID;
    recibirDatos(&ID, 1, DIRECCION_ID, ACELEROMETRO);
    return ID;
}