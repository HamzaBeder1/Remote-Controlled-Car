// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL      // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))

#include "xc.h"
#include <stdbool.h>


bool stopMotion = 0;
volatile uint32_t finalTime = 0;
volatile int overflowtmr = 0;
int distanceThreshold = 1;
void setup(){
    CLKDIVbits.RCDIV = 0;
    TRISBbits.TRISB4 = 1;  //input for echo
    TRISBbits.TRISB5 = 0; //output for trig 
    LATBbits.LATB5 = 0;

    //IC1 setup
    IC1CONbits.ICTMR = 0; //timer 3
    IC1CONbits.ICM = 1;

    //PPS for IC1
    __builtin_write_OSCCONL(OSCCON & 0xbf);// unlock PPS
    RPINR7bits.IC1R = 4;  // RP4 (pin 11)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS

    T3CON = 0;
    TMR3 = 0;
    T3CONbits.TCKPS = 0;
    _T3IF = 0; //a prescaler of 1 for TMR3
    PR3 = 65535;
    T3CONbits.TON = 1;

    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    IPC2bits.T3IP = 3;
    IFS0bits.IC1IF = 0;
    IEC0bits.IC1IE = 1;
    IPC0bits.IC1IP = 3;

    //TIMER1 setup
    T1CON = 0;
    TMR1 = 0;
    T1CONbits.TCKPS = 0;
    _T1IF = 0;
    PR1 = 160; //for a delay of 10uS
    T1CONbits.TON = 0;

    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    IPC0bits.T1IP = 5; //higher priority than the input capture interrupt

}

void __attribute__((interrupt, auto_psv)) _IC1Interrupt(){
    IFS0bits.IC1IF = 0;
    if(PORTBbits.RB4 == 1){
        //reset both TMR3 and overflowtmr
        TMR3 = 0;
        overflowtmr = 0;
    }
    else{
        finalTime = (TMR3)*625 + overflowtmr*625*65536;
        overflowtmr = 0;
        TMR3 = 0;
        uint16_t distance = finalTime/((58)*(10000/10));
        if(distance <= distanceThreshold)
            stopMotion = 1;
        else
            stopMotion = 0;
    }
}

void __attribute__((interrupt, auto_psv)) _T3Interrupt(){
    IFS0bits.T3IF = 0;
    overflowtmr++;
}

void __attribute__((interrupt, auto_psv)) _T1Interrupt(){
    _T1IF = 0;
    T1CONbits.TON = 0;
    TMR1 = 0;
    LATBbits.LATB5 = 0;
}


void sendTrig(){
    LATBbits.LATB5 = 1;
    T1CONbits.TON = 1;
}

void delay_ms(unsigned int ms){
    while(ms-- > 0){
        asm("repeat #15999");
        asm("nop");
    }
}

int main(void) {
    setup();
    while(1){
        sendTrig();
        delay_ms(2000); //delay for 2 seconds before sending another trig signal.
    }
}
