/* 
 * File:   main.h
 * Author: Hamza Beder
 *
 * Created on July 3, 2024, 11:26 PM
 */

#include <libpic30.h>
#include "xc.h"
#include <stdbool.h>
#include <string.h>

#define FCY 16000000UL
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
#define _XTAL_FREQ 16000000 
#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif
    
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

bool stopMotion = 0;
volatile uint32_t finalTime = 0;
volatile int overflowtmr = 0;
const float distanceThreshold = 2;
volatile char buffer[20];
volatile int front = 0, back = 0;
int trigDone = 0;
int toggleMove;
float step;
struct PIC24RTC rtcc;
    

void setup();
void setRTCWREN();
uint8_t bcd2dec(uint8_t bcd);
uint8_t dec2bcd(uint8_t dec);
void initRTCC(int16_t year, int8_t month, int8_t day, int8_t wday, int8_t hour, int8_t minute, int8_t second);
void initIC();
void initI2C();
void initSPI();
void initUART();
void initMPU6050(unsigned char AFS_SEL);
void setIdleMode();
void disableIdleMode();
void getDateTime();
void sendStartBitI2C();
void sendStopBitI2C();
void sendDataI2C(unsigned char data);
unsigned char getDataI2C();
void writeRegisterI2C(unsigned char addr, unsigned char reg, unsigned char data);
unsigned char readRegisterI2C(unsigned char addr, unsigned char reg);
void writeRegisterMPU6050(unsigned char reg, unsigned char data);
unsigned char readRegisterMPU6050(unsigned char reg);
void getAccelMPU6050();
void initDisplay();
void sendTrig();
void sendData(char data []);
void sendData2(float data[]);
char getData();
unsigned char sendDataSPI(unsigned char data);
unsigned char sendDataSPI2(unsigned char data);
void sendCommandDisplay(unsigned char data, unsigned char * params, size_t param_size);
void drawPixel(int16_t data);
void drawDisplay(int16_t data[2], int size);



#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

