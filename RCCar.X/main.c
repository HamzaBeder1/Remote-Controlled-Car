
#pragma config ICS = PGx1          
#pragma config FWDTEN = OFF        
#pragma config GWRP = OFF          
#pragma config GCP = OFF           
#pragma config JTAGEN = OFF        



#pragma config I2C1SEL = PRI       
#pragma config IOL1WAY = OFF      
#pragma config OSCIOFNC = ON       
#pragma config FCKSM = CSECME     
                                      
#pragma config FNOSC = FRCPLL      

#include "xc.h"
#include <stdbool.h>
#include <string.h>


bool stopMotion = 0;
volatile uint32_t finalTime = 0;
volatile int overflowtmr = 0;
const float distanceThreshold = 2;
volatile char buffer[20];
volatile int front = 0, back = 0;
int trigDone = 0;
int toggleMove;
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
    
    //TIMER3 setup
    T3CON = 0;
    TMR3 = 0;
    T3CONbits.TCKPS = 0;//a prescaler of 1 for TMR3
    _T3IF = 0; 
    PR3 = 65535;
    T3CONbits.TON = 0;

    /*IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    IPC2bits.T3IP = 3;*/
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
    buffer[front] = U1RXREG;
    front = (front+1)%20;
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
    trigDone = 1;
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
    int len = sizeof(data)/sizeof(data[0]);
    for(i = 0; i<len; i++){
        U1TXREG = data[i];
        while(U1STAbits.UTXBF == 1);
    }
}

void sendData2(float data[]){
    int i;
    int len = sizeof(data)/sizeof(data[0]);
    for(int i = 0; i < 1; i++){
        U1TXREG = data[i];
        while(U1STAbits.UTXBF == 1);
    }   
}

char getData(){
    if(front == back) //no new data
        return 'N';
    else if(toggleMove == 0) //collision about to occur, stop movement.
        return 'N';
    char data = buffer[back];
    back = (back+1)%20;
    return data;
}

int main(void) {
    setup();
    initUART();
    delay_ms(2000);
    while(1){
        sendTrig();
        while(!trigDone);
        while(!PORTBbits.RB4);
        T3CONbits.TON = 1;
        trigDone = 0;
        while(PORTBbits.RB4);
        T3CONbits.TON = 0;
        float a = TMR3;
        a = a/(58*16);
        float arr[1] = {a};
        TMR3 = 0;
        sendData2(arr);
        if(a <= distanceThreshold){
            toggleMove = 0;
        }
        else{
            toggleMove = 1;
        }
        char data = getData();
        switch (data)
        {
            case 'W':{
                LATBbits.LATB7 = 1;
                LATBbits.LATB9 = 1;
                break;
            }
            
            case 'S':{
                LATBbits.LATB8 = 1;
                LATBbits.LATB11 = 1;
                break;
            }
            
            default:{
            break;
            }
        }
        delay_ms(100);
        LATB &= 0b1111010001111111; 
    }
}
