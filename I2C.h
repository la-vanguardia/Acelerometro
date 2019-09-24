void iniciarComunicacion(unsigned char direccion, unsigned char read); //read=1 r, read=0 w
void trasmitirDato(unsigned char dato);
void trasmitirDatos(unsigned char *datos, unsigned char numero_datos, unsigned char dirreccion);
unsigned char recibirDato();
void recibirDatos(unsigned char *datos, unsigned char numero_datos, unsigned char dirreccion);
