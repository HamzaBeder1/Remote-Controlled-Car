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

#define FCY 16000000UL
#include <libpic30.h>
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
    AD1PCFG |= 0b0001111111111111;
    TRISB |= 0b0001000000000100;
    TRISB &= 0b1000011111000111;
    TRISA &= 0b1111111111100101;
    LATB &= 0b1111010001111111; 
    LATBbits.LATB5 = 1; //CS = 1, initially.
    LATA |= 0b0000000000001010;
    
    //enable global interrupts
    __builtin_enable_interrupts();
    
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

void initIC(){
    IC1CONbits.ICTMR = 0; //timer 3
    IC1CONbits.ICM = 1;

    //PPS for IC1
    __builtin_write_OSCCONL(OSCCON & 0xbf);// unlock PPS
    RPINR7bits.IC1R = 12;  // RP12 (pin 23)
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

void initI2C(){
    I2C1CONbits.I2CEN = 0; 
    //I2C1STATbits.R_W = 1; //data is being read from MPU-6050.
    I2C1BRG = 157;
    IFS1bits.MI2C1IF = 0;
   
    I2C1CONbits.I2CEN = 1; //enables I2C
}

void initSPI(){
    SPI1STATbits.SPIEN = 0;
    SPI1STATbits.SPIROV = 0; //no overflow has occurred. 
    SPI1CON1bits.MSTEN = 1; //master mode.
    SPI1STATbits.SISEL = 0b101; //interrupts when last bit shifts out of SPIxSR, so transmit is complete.
    SPI1STATbits.SPIEN = 1;
    SPI1CON2bits.SPIBEN = 0; //enhanced buffer is disabled.
    
    IFS0bits.SPI1IF = 0;
    IEC0bits.SPI1IE = 1;
    IPC2bits.SPI1IP = 4; //interrupt priority 3.
    
    //PPS
    __builtin_write_OSCCONL(OSCCON & 0xbf);// unlock PPS
    RPINR20bits.SDI1R = 2; //pin 6 for SDI1.
    RPOR1bits.RP3R = 7; //pin 7 for SDO1.
    RPOR2bits.RP4R = 8; //pin 11 for SCK1.
    RPOR2bits.RP5R = 9; //pin 14 for SS1.
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
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

void __attribute__((interrupt, no_auto_psv)) _MI2C1IFInterrupt(void){
    IFS1bits.MI2C1IF = 0;
}



void __attribute__((__interrupt__, __auto_psv__)) _SPI1Interrupt(void){
    IFS0bits.SPI1IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
    buffer[front] = U1RXREG;
    front = (front+1)%20;
}

void __attribute__((interrupt, auto_psv)) _IC1Interrupt(){
    IFS0bits.IC1IF = 0;
    if(PORTBbits.RB12 == 1){
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
    LATAbits.LATA4 = 0; //to end pulse sent to trig pin
    trigDone = 1;
}

void delay_ms(unsigned int ms){
    while(ms-- > 0){
        asm("repeat #15999");
        asm("nop");
    }
}

void sendTrig(){
    LATAbits.LATA4 = 1;
    T1CONbits.TON = 1;
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

unsigned char spixchg(unsigned char data){
    LATBbits.LATB5 = 0; //Set CS LOW.
    SPI1BUF = data;
    while(!SPI1STATbits.SPIRBF);
    LATBbits.LATB5 = 1; //Set CS HIGH.
    return SPI1BUF; //dummy read to clear SPI1BUF. Also return received data.
}

void getData_I2C(unsigned char addr){
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.SEN = 1; //initiate start condition.
    while(I2C1CONbits.SEN);
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = addr;
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN);
    while(!IFS1bits.MI2C1IF);
}


void displayData(){
    LATAbits.LATA1 = 0; //A0 = 0 for command mode.
    spixchg(0b00101001); //DISPON command.
    delay_ms(120);  //120ms delay after using DISPON command.
    spixchg(0b00101100); //RAMWR command.
    LATAbits.LATA1 = 1; //A0 = 1 for data mode, now writing to memory.
    
    int i;
    for(i = 0; i <1; i++) //12 bits per pixel -> 2 pixels should become black.
    {
        spixchg(0b00000000); 
    }
    LATAbits.LATA1 = 0; //A0 = 0 for command mode.6
    spixchg(0b00101001); //ending with any command (method 2, section 9.6.2).
    delay_ms(120); //one more 120ms delay.
}

int main(void) {
    setup();
    initSPI();
    initUART();
    initI2C();
    __delay_ms(2000);
    while(1){
        getData_I2C(0b11010001);
        __delay_ms(100);
    }
    /*while(1){
        sendTrig();
        while(!trigDone);
        while(!PORTBbits.RB12);
        T3CONbits.TON = 1;
        trigDone = 0;
        while(PORTBbits.RB12);
        T3CONbits.TON = 0;
        float a = TMR3;
        a = a/(58*16);
        TMR3 = 0;
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
                LATBbits.LATB11 = 1;
                LATBbits.LATB13 = 1;
                break;
            }
            
            case 'S':{
                LATBbits.LATB12 = 1;
                LATBbits.LATB14 = 1;
                break;
            }
            
            default:{
            break;
            }
        }
        delay_ms(1000);
        LATB &= 0b1000011111111111; 
    }*/
}