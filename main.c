#include <p33fj128gp202.h>
#define FCY 40000000UL//

_FICD(ICS_PGD2 & JTAGEN_OFF); // Para hacer debuging por el puerto 2
_FOSCSEL(FNOSC_FRC);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FWDT(FWDTEN_OFF);


void ConfigIni(void);

int main(void)
{

    ConfigIni();

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