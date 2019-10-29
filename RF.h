#include <xc.h>
#include <libpic30.h>

#define RFWRITE 0x20
#define RFREAD 0x00
#define Basura 0xFF
//#define NOP 0xFF


void ConfigurarSPI(void);
unsigned char TransferenciaSPI (unsigned char datoSPI);
unsigned char TransferenciaRF (unsigned char comandoSPI, unsigned char direccionSPI, unsigned char datoSPI);
void ConfigurarRF (unsigned char direccionSPI, unsigned char datoSPI);
void EnviarRF (unsigned char numero_bytes,unsigned char datoSPI);
<<<<<<< HEAD
unsigned char RecibirRF (unsigned char numero_bytes);

=======
void RecibirRF (unsigned char numero_bytes, unsigned char *datos);
>>>>>>> 968e5b0e987907b824635c4b49dca51193c8b5b7


void ConfigurarSPI(void){
    //Habilito el modulo, modo maestro, fase
    SPI1STATBITS.SPIEN=1;
    SPI1CON1BITS.MSTEN=1;
    SPI1CON1BITS.CKE=1;
    //Configurado a ~~~ 250KHz
    SPI1CON1BITS.SPRE=0b110;
    SPI1CON1BITS.PPRE=0b00;
    
    //Definir SSEN y los TRIS - Segun la placa
    
    
    //Limpio bandera y deshabilito interrupcion
    IFS0BITS.SPI1IF = 0;
    IEC0BITS.SPI1IE = 0;
    
}

unsigned char TransferenciaSPI(unsigned char datoSPI){
    //Habilitar CS
    
    SPI1BUF = datoSPI;
    while(IFS0BITS.SPI1IF == 0); //Espero la bandera, se puede probar tambien con los buffer Tx o Rx 
    IFS0BITS.SPI1IF=0;
    return SPI1BUF;
    
    //Apagar CS
}

unsigned char TransferenciaRF(unsigned char comandoSPI, unsigned char direccionSPI, unsigned char datoSPI){
    unsigned char RFStatus=0;
    unsigned char RFdato_leido=0;
    unsigned char BasuraRF=0;
    
    
    //FALTA Habilitar CS
    
    switch(comandoSPI){
       
        case RFREAD:
            
                RFStatus=TransferenciaSPI(comandoSPI | direccionSPI);
                RFdato_leido = TransferenciaSPI(Basura);
                //FALTA apagar CS
                return (RFdato_leido);
            
            
                break;    
            
        case RFWRITE:
                RFStatus=TransferenciaSPI(comandoSPI | direccionSPI);
                BasuraRF = TransferenciaSPI(datoSPI);
                //FALTA apagar CS
                break;
       
            
        default:
                break;
       
    }
    
}
            

void ConfigurarRF(unsigned char direccionSPI, unsigned char datoSPI){
    
    TransferenciaRF(RFWRITE,0x00,0x80); //Apago (VER)
    TransferenciaRF(RFWRITE,direccionSPI,datoSPI); //Configuro
    TransferenciaRF(RFWRITE,0x00,0x02); //Prendo

    
}

void EnviarRF(unsigned char numero_bytes, unsigned char datoSPI){
    //for con "numero_byes" para recibir varios ¿?   
    ConfigurarRF(0x00,0x00);
    TransferenciaSPI(0xA0);
    TransferenciaSPI(datoSPI);
    __delay_ms(1);
    
    
    RecibirRF(numero_bytes); //VER
 
}

void RecibirRF (unsigned char numero_bytes, unsigned char *datos){
    unsigned char status=0, i = 0;
    unsigned char RFdato_leido=0;

    ConfigurarRF(0x00,0x01);
    TransferenciaRF(RFWRITE,0x11,numero_bytes); //Configurar cantidad de bytes a recibir
    
    
    status = TransferenciaRF(RFREAD,0x07,0);
    while((status && 0x10)== 1){  //Bandera de recepcion
        //Habilitar CS
        TransferenciaSPI(0x61); //Comando para leer (VER)
        datos[i] = TransferenciaSPI(Basura);
        status = TransferenciaRF(RFREAD,0x07,0);
        i++;
    }

    
    TransferenciaRF(RFWRITE,0x07,0x01);   //Baja la bandera 
    
}

