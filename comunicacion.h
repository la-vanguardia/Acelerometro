#include <xc.h>
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>

void aCrearVectorEnviar(unsigned char codigo, unsigned char numero_datos, unsigned char orden,unsigned char *datos, unsigned char *enviar){ 
    unsigned char numero_bytes = 3 + numero_datos;
    unsigned char i;
    for(i=0; i<numero_bytes  ;i++){
        switch(i){
            case 0:
                enviar[0] = codigo;
                break;
            case 1:
                enviar[1] = numero_bytes;
                break;
            case 2:
                enviar[2] = orden; 
                break;
            default:
                enviar[i] = datos[i - 3];
        }
    }
}

void aObtenerDatos(unsigned char *recibido, unsigned char *datos, unsigned char *trama){
    unsigned char numero_datos = recibido[1] - 3;
    unsigned char i;
    trama[0] = recibido[2];
    for(i=0; i<numero_datos; i++){
        datos[i] = recibido[i + 3];
    }
}


