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

#define MPU6050_ADDR 0x68
#define COLMOD 0x3A
#define FRMCTR1 0xB1
#define FRMCTR2 0xB2
#define FRMCTR3 0xB3
#define VMCTR1 0xC5
#define INVON 0x21
#define SLPOUT 0x11
#define NORON 0x13
#define DISPON 0x29
#define RAMWR 0x2C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define ACCEL_CONFIG  0x1C

bool stopMotion = 0;
volatile uint32_t finalTime = 0;
volatile int overflowtmr = 0;
const float distanceThreshold = 2;
volatile char buffer[20];
volatile int front = 0, back = 0;
int trigDone = 0;
int toggleMove;
float step;

enum accelRange{
    RANGE_2G = 0b00,
    RANGE_4G = 0b01,
    RANGE_8G = 0b10,
    RANGE_16G = 0b11
};

struct PIC24RTC{
    int16_t year;
    int8_t month;
    int8_t day;
    int8_t wday;
    int8_t hour;
    int8_t minute;
    int8_t second;
};

struct PIC24RTC rtcc;

void setRTCWREN(){

    asm volatile ("push w7");
    asm volatile("push w8");
    asm volatile("disi #5");
    asm volatile("mov #0x55, w7");
    asm volatile("mov w7, _NVMKEY");
    asm volatile("mov #0xAA, w8");
    asm volatile("mov w8, _NVMKEY");
    asm volatile("bset _RCFGCAL, #13");
    asm volatile("pop w8");
    asm volatile("pop w7");
    
}

uint8_t bcd2dec(uint8_t bcd){
  return bcd/16*10 + bcd%16;
}

uint8_t dec2bcd(uint8_t dec){
    uint8_t temp = dec/10*16 + dec%10;
    
  return temp;
}

void initRTCC(int16_t year, int8_t month, int8_t day, int8_t wday, int8_t hour, int8_t minute, int8_t second){
    //RCFGCALbits.RTCWREN = 1;//RTCVALH and RTCVALL registers can be written to.  
    setRTCWREN();
    RCFGCALbits.RTCEN = 1; //enable RTCC module.
    RCFGCALbits.RTCPTR = 0b11; //Pointer to RTCC Value registers. 
    
    int8_t temp = (year%10) + ((year/10)%10)*10;
    RTCVAL = dec2bcd(temp);
    RTCVAL = (dec2bcd(month) << 8) | dec2bcd(day);
    RTCVAL = (dec2bcd(wday) << 8) | dec2bcd(hour);
    RTCVAL = (dec2bcd(minute) <<8 | dec2bcd(second));
    RCFGCALbits.RTCWREN = 0;
    int x = 21;
    x+=21111;
}

void getDateTime(){
    RCFGCALbits.RTCPTR = 0b11; //Pointer to RTCC Value registers. 
    
    int16_t year = RTCVAL;
    int16_t monthAndDay = RTCVAL;
    int16_t wDayAndHour = RTCVAL;
    int16_t minuteAndSecond = RTCVAL;
    
    rtcc.year = bcd2dec(year);
    rtcc.month = bcd2dec(monthAndDay >> 8);
    rtcc.day = bcd2dec(monthAndDay & 0x00FF);
    rtcc.wday = bcd2dec(wDayAndHour >> 8);
    rtcc.hour = bcd2dec(wDayAndHour & 0x00FF);
    rtcc.minute = bcd2dec(minuteAndSecond >> 8);
    rtcc.second = bcd2dec(minuteAndSecond & 0x00FF);
    int x = 21;
    x+=21111;
}

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

/*
void initGyro(){
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.SEN = 1; //initiate start condition.
    while(I2C1CONbits.SEN);
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = 0b11010000; //send slave address and do write operation.
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = PWR_MGMT_1; //send address of register
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.RSEN = 1; //initiate repeated start condition.
    while(I2C1CONbits.RSEN);
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = 0b11010000; //send slave address and do write operation.
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = 0x40; //write this to PWR_MGMT_1 register.
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN);
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
}
*/


void sendStartBitI2C(){
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.SEN = 1; //send start bit.
    while(I2C1CONbits.SEN); //wait for SEN to be cleared by hardware.
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
}

void sendStopBitI2C(){
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.PEN = 1; //send stop bit.
    while(I2C1CONbits.PEN); //hardware will automatically clear this bit when done receiving.
}


void sendDataI2C(unsigned char data){
    I2C1TRN = data;
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
}

unsigned char getDataI2C(){
    unsigned char result = I2C1RCV;
    return result;
}

void writeRegisterI2C(unsigned char addr, unsigned char reg, unsigned char data){
    sendDataI2C(addr << 1);
    sendDataI2C(reg);
    sendDataI2C(data);
}

unsigned char readRegisterI2C(unsigned char addr, unsigned char reg){
    sendDataI2C(addr << 1);
    sendDataI2C(reg);
    sendStopBitI2C();
    sendStartBitI2C();
    sendDataI2C((addr <<1)|1);
    I2C1CONbits.RCEN = 1; //enable receive mode and get data from module.
    while(I2C1CONbits.RCEN); //hardware will automatically clear this bit when done receiving.
    unsigned char i2c_data = I2C1RCV;
    return i2c_data;
}

void writeRegisterMPU6050(unsigned char reg, unsigned char data){
    sendStartBitI2C();
    writeRegisterI2C(MPU6050_ADDR, reg, data);
    sendStopBitI2C();
}

unsigned char readRegisterMPU6050(unsigned char reg){
    sendStartBitI2C();
    unsigned char val = readRegisterI2C(MPU6050_ADDR, reg);
    sendStopBitI2C();
    return val;
}

void initMPU6050(unsigned char AFS_SEL){
    switch(AFS_SEL){
        case RANGE_2G:
            step = 0.00059877;
            break;
        case RANGE_4G:
            step = 0.00119;
        case RANGE_8G:
            step = 0.002395;
        case RANGE_16G:
            step = 0.00479;
        default:
            break;
    }
    writeRegisterMPU6050(PWR_MGMT_1, 0x00);
    unsigned char temp = readRegisterMPU6050(ACCEL_CONFIG);
    temp &= 0b11100111;
    temp |= (AFS_SEL << 3);
    writeRegisterMPU6050(ACCEL_CONFIG, temp);
}

void getAccelMPU6050(){
    int16_t accelXH = readRegisterMPU6050(ACCEL_XOUT_H);
    int16_t accelXL = readRegisterMPU6050(ACCEL_XOUT_L);
    int16_t accelYH = readRegisterMPU6050(ACCEL_YOUT_H);
    int16_t accelYL = readRegisterMPU6050(ACCEL_YOUT_L);
    int16_t accelZH = readRegisterMPU6050(ACCEL_ZOUT_H);
    int16_t accelZL = readRegisterMPU6050(ACCEL_ZOUT_L);
    
    int16_t accelX = (accelXH << 8) | (accelXL);
    int16_t accelY = (accelYH << 8) | (accelYL);
    int16_t accelZ = (accelZH << 8) | (accelZL);
    
    float X = (accelX)*step;
    float Y = (accelY)*step;
    float Z = (accelZ)*step;
    
    int x = 51;
    x+=1;
}


void initDisplay(){
    LATAbits.LATA3 = 1; //RESET = 1, it is an active low pin.
    unsigned char params[1] = {0x55}; //Used for COLMOD.
    unsigned char params2[3] = {0x02, 0x2C, 0x2D};
    unsigned char params3[3] = {0x02, 0x2C, 0x2D};
    unsigned char params4[6] = {0x02, 0x2C, 0x2D, 0x02, 0x2C, 0x2D};
    unsigned char params5[2] = {0x51,0x4D};
    sendCommandDisplay(COLMOD, params, 1); //COLMOD: This formats the pictures as 16bits/pixel.
    sendCommandDisplay(FRMCTR1, params2, 3);
    sendCommandDisplay(FRMCTR2, params3, 3);
    sendCommandDisplay(FRMCTR3, params4, 6);
    sendCommandDisplay(VMCTR1, params5, 2);
    sendCommandDisplay(INVON, params, 0);//INVON: Enter display inversion mode.
    sendCommandDisplay(SLPOUT, params, 0); //SLPOUT: This turns off sleep mode.
    sendCommandDisplay(NORON, params, 0);
    sendCommandDisplay(DISPON, params, 0);
    sendCommandDisplay(RAMWR, params, 0); //RAMWR: Memory write command
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

unsigned char sendDataSPI(unsigned char data){
    LATBbits.LATB5 = 0; //Set CS LOW.
    SPI1BUF = data;
    while(!SPI1STATbits.SPIRBF);
    LATBbits.LATB5 = 1; //Set CS HIGH.
    return SPI1BUF; //dummy read to clear SPI1BUF. Also return received data.
}



/*
 char * getDataI2C(unsigned char data){
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.SEN = 1; //initiate start condition.
    while(I2C1CONbits.SEN);
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = 0b11010000; //send address and write bit.
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = data;
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.RSEN = 1; //initiate repeated start condition.
    while(I2C1CONbits.RSEN);
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1TRN = 0b11010001; //send address and read bit.
    while(!IFS1bits.MI2C1IF);
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.RCEN = 1; //enable receive mode and get data from module.
    while(I2C1CONbits.RCEN); //hardware will automatically clear this bit when done receiving.
    char * i2c_data = I2C1RCV;
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN);
    while(!IFS1bits.MI2C1IF);
    return i2c_data;
}
 */


void sendCommandDisplay(unsigned char data, unsigned char * params, size_t param_size){
    LATAbits.LATA1 = 0; //A0 = 0 for command mode.
    sendDataSPI(data); //send command.
    LATAbits.LATA1 = 1; //A0 = 1
    if(param_size > 0){
        for(int i = 0; i < param_size; i++){
            sendDataSPI(params[i]);
        }  
    }
}

void drawDisplay(unsigned char * data){
    sendCommandDisplay(0x2C, NULL, 0); //RAMWR: Memory write command
    int i;
    for(i = 0; i < 342144; i++){ //128x128 display, 2 bytes per pixel.
        sendDataSPI(0x00);
        sendDataSPI(0x00);
    } //128x128 display, 2 bytes per pixel.
    /*for(int i = 0; i < 1; i++){ //one iteration for simplicity when debugging.
        sendDataSPI(0x00);
    }*/
}

int main(void) {
    setup();
    initSPI();
    initUART();
    initI2C();
    initMPU6050(RANGE_2G);
    initRTCC(2024, 8, 3, 7, 12, 10, 30);
    __delay_ms(1000);
    while(1){
        getDateTime();
        //drawDisplay(NULL);
       //char * x = readRegisterI2C(MPU6050_ADDR, PWR_MGMT_1);
        //readRegisterMPU6050(PWR_MGMT_1);
        //__delay_ms(1000);
    }
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

