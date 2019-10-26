#define TX2 U2TXREG

unsigned char U2datos[50] = {'\0'}, bandera_send = 0, bandera_ok = 0;

void ConfigurarWIFI();
void enviarComando(unsigned char* comando);
void enviarDato(unsigned char* mensaje);
unsigned char longitudSTR(unsigned char *A);


void ConfigurarWIFI(){
    //RX RB9 
    //TX RB10
    
    RPINR19bits.U2RXR = 9;
    RPOR5bits.RP10R = 5;
    
    //U2BRG = 129;
    U2BRG = 21;
    U2STAbits.UTXISEL1 = 1;
   
    U2MODEbits.UARTEN = 1; 
    IEC1bits.U2RXIE = 1; //Interrupcion por recepcion
    U2STAbits.UTXEN = 1;  
  
    __delay_us(60); 
    IFS1bits.U2TXIF = 0;
    IFS1bits.U2RXIF = 0;
    
    //CONFIGURAR MODULO WIFI
    
    __delay_ms(1000);
    
    enviarComando("E0");
    
    

   
}

void enviarComando(unsigned char *comando){
    unsigned char length = longitudSTR(comando), i;
    
    //envio el principio del comando
    while(U2STAbits.UTXBF == 1);
    TX2 = 'A';
    while(U2STAbits.UTXBF == 1);
    TX2 = 'T';
    
    for(i=0; i<length; i++){
        while(U2STAbits.UTXBF == 1);
        TX2 = comando[i];
    }
    
    while(U2STAbits.UTXBF == 1);
    TX2 = 0x0D;
    while(U2STAbits.UTXBF == 1);
    TX2 = 0x0A;
   
}

void enviarDato(unsigned char *mensaje){
    unsigned char length = mensaje[1], i;
    unsigned char comando[20] = {'\0'};
     bandera_send = 0;
    
    sprintf(comando, "+CIPSEND=%u",length); 
    enviarComando(comando);
    
    while(bandera_send == 0);
    bandera_send = 0;
    for(i=0; i<length; i++){
        while(U2STAbits.UTXBF == 1);
        TX2 = mensaje[i];
    }
    bandera_ok = 0;
    while(bandera_ok == 0);
    bandera_ok = 0;
}

unsigned char longitudSTR(unsigned char *A){
    unsigned char i=0, length = 0;
    while(A[i] != '\0'){
        length++;
        i++;
    };
    return length;
}

