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
#include <string.h>


bool stopMotion = 0;
volatile uint32_t finalTime = 0;
volatile int overflowtmr = 0;
int distanceThreshold = 1;
volatile char buffer[20];
volatile int front;
void setup(){
    CLKDIVbits.RCDIV = 0;
    TRISB |= 0b0000000000010000;
    TRISB &= 0b1111010001011111;
    LATB &= 0b1111010001011111;
    
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

void ICsetup(){
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
}

void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
    /*addTo(U1RXREG);
    char c = buffer[0];*/
    if(U1RXREG == 'W'){
        LATBbits.LATB7 = 1;
        LATBbits.LATB9 = 1;
    }
    else if(U1RXREG == 'S'){
        LATBbits.LATB8 = 1;
        LATBbits.LATB11 = 1;
    }
    delay_ms(1000);
    LATBbits.LATB7 = 0;
    LATBbits.LATB8 = 0;
    LATBbits.LATB9 = 0;
    LATBbits.LATB11 = 0;
    
}

void addTo(int val){
    buffer[front++] = val;
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
        uint16_t distance = TMR3/58;
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
    LATBbits.LATB5 = 0; //to end pulse sent to trig pin
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

void initUART(){
    CLKDIVbits.RCDIV = 0;
    AD1PCFG = 0x9fff;  

    _TRISB6 = 0;  // U1TX output
    _TRISB10 = 1; // U1RX input

    U1MODE = 0;  

    U1MODEbits.BRGH = 0;
    U1BRG = 25; //for 38400 baud
    //U1BRG = 103; uncomment this for 9600 baud.
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
    U1MODEbits.UEN = 1;
    U1MODEbits.PDSEL = 0; //8 bit data and no parity
    U1MODEbits.STSEL = 0; //one stop bit.

    // Peripheral Pin Select 
    __builtin_write_OSCCONL(OSCCON & 0xbf);
    RPOR3bits.RP6R = 3;   
    RPINR18bits.U1RXR= 10;   
    __builtin_write_OSCCONL(OSCCON | 0x40); 

    IFS0bits.U1RXIF = 0;
    IEC0bits.U1RXIE = 1;

    IPC2bits.U1RXIP = 5; //priority of 5
}


void sendData(char data []){
    int i;
    for(i = 0; i<strlen(data); i++){
        U1TXREG = data[i];
        while(U1STAbits.UTXBF == 1);
    }
}

int main(void) {
    setup();
    initUART();
    delay_ms(2000);
    while(1){
        sendTrig();
        delay_ms(1000);
    }
}
