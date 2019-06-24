/*
 * Remote Call-Holding Alert Transmitter Module
 * Copyright (c) 2019 David Rice
 * 
 * Processor: PIC16F18855
 * 
 * The following macros must be defined on the XC8 command line or in project properties:
 * _XTAL_FREQ - CPU speed in Hz (Fosc) - must be 4000000 for this project
 * RF_CHANNEL - the channel to use for the RF module (example: 0x10U)
 * 
 * The following macro may be defined to change default behavior:
 * MAX_SOFT_RETRIES - the number of times to retry transmission in software (default: 20)
 * 
 * Drivers used:
 * DS18B20
 * NRF24L01P
 * 
 * Peripheral usage:
 * MSSP1 - Communication with RF module (SPI Master mode)
 * 
 * Pin assignments:
 * RA0 - Button A
 * RA1 - Button B
 * RA2 - Button C
 * RA3 - Button D
 * RA4 - LED A
 * RA5 - LED B
 * RA6 - LED C
 * RA7 - LED D
 * RB0 - RF module CE
 * RB1 - RF module CSN
 * RB2 - RF module SCK
 * RB3 - RF module MOSI
 * RB4 - RF module MISO
 * RB5 - RF module IRQ
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC16F18855 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "nRF24L01P.h"

#ifndef RF_CHANNEL
#error RF_CHANNEL must be defined
#endif

#ifndef MAX_SOFT_RETRIES
#define MAX_SOFT_RETRIES 20
#endif

#define ADDR_LEN        5
#define PAYLOAD_WIDTH   1

#define LED_A_ON        LATAbits.LATA4
#define LED_B_ON        LATAbits.LATA5
#define LED_C_ON        LATAbits.LATA6
#define LED_D_ON        LATAbits.LATA7

#define BUTTON_A_PRESSED    !PORTAbits.RA0
#define BUTTON_B_PRESSED    !PORTAbits.RA1
#define BUTTON_C_PRESSED    !PORTAbits.RA2
#define BUTTON_D_PRESSED    !PORTAbits.RA3

#define BUTTON_A_PAYLOAD    0xA6
#define BUTTON_B_PAYLOAD    0xB5
#define BUTTON_C_PAYLOAD    0xC0
#define BUTTON_D_PAYLOAD    0xD2

const uint8_t strip1_addr[ADDR_LEN] = { 'T', 'N', 'E', 'T', 0xAA };
const uint8_t strip2_addr[ADDR_LEN] = { 'T', 'N', 'E', 'T', 0xBB };

void init_ports(void) {
    /* Disable all analog features */
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;
    
    /* Pull all outputs low except RB1 (RF_CSN) */
    LATA = 0x00;
    LATB = _LATB_LATB1_MASK;
    LATC = 0x00;
    
    /* Enable weak pull-ups for buttons */
    WPUA = _WPUA_WPUA0_MASK |
           _WPUA_WPUA1_MASK |
           _WPUA_WPUA2_MASK |
           _WPUA_WPUA3_MASK;
    
    /* Initialize PORTA inputs on RA0, RA1, RA2, RA3 for buttons */
    TRISA = _TRISA_TRISA0_MASK |
            _TRISA_TRISA1_MASK |
            _TRISA_TRISA2_MASK |
            _TRISA_TRISA3_MASK;
    
    /* Initialize all of PORTB as output except for RB4 (SDI1) and RB5 (RF_IRQ) */
    TRISB = _TRISB_TRISB4_MASK |
            _TRISB_TRISB5_MASK;
    
    /* Initialize PORTC as output */
    TRISC = 0x00;

    /* Set TTL input level on RB4 (SDI1) and RB5 (RF_IRQ) */
    INLVLBbits.INLVLB4 = 0;
    INLVLBbits.INLVLB5 = 0;
}

void init_osc(void) {
    /* Set frequency of HFINTOSC to 4 MHz */
    OSCFRQbits.HFFRQ = 0b0010;
    
    /* Set clock divider to 1:1 to achieve Fosc of 4 MHz */
    OSCCON1bits.NDIV = 0b0000;
}

void init_spi(void) {
    /* Set MSSP1 to SPI Master mode, clock = Fosc / 4 = 1 MHz at Fosc = 4 MHz */
    SSP1CON1bits.SSPM = 0b0000;
    
    /* Transmit data on active-to-idle transition */
    SSP1STATbits.CKE = 1;
    
    /* Enable MSSP1 */
    SSP1CON1bits.SSPEN = 1;
}

void init_pps(void) {
    bool state;
    
    /* Preserve global interrupt state and disable interrupts */
    state = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    
    /* Unlock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    
    /* SCK1 on RB2 */
    RB2PPS = 0x14;
    SSP1CLKPPS = 0x0A;
    
    /* SDI1 on RB4 */
    SSP1DATPPS = 0x0C;
    
    /* SDO1 on RB3 */
    RB3PPS = 0x15; 
    
    /* Lock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    
    /* Restore global interrupt state */
    INTCONbits.GIE = state;
}

void init_rf(void) {
    /* Allow for maximum possible RF module startup time */
    __delay_ms(100);
    
    /* Set 500 uSec retry interval and 4 maximum retries */
    nrf24_write_register(NRF24_SETUP_RETR, NRF24_ARD_500 | NRF24_ARC_4);
    
    /* Set RF power to 0 dBm and data rate to 1 Mbit/Sec */
    nrf24_write_register(NRF24_RF_SETUP, NRF24_RF_PWR_0DBM);
    
    /* Set 5-byte address width */
    nrf24_write_register(NRF24_SETUP_AW, NRF24_AW_5);
    
    /* Set initial RF channel */
    nrf24_write_register(NRF24_RF_CH, RF_CHANNEL);
    
    /* Mask RX_DR interrupt on RF module, enable CRC, power up RF module in transmit-standby mode */
    nrf24_write_register(NRF24_CONFIG, NRF24_MASK_RX_DR | NRF24_EN_CRC | NRF24_PWR_UP);
    
    /* Clear any pending RF module interrupts */
    nrf24_write_register(NRF24_STATUS, NRF24_TX_DS | NRF24_MAX_RT | NRF24_RX_DR);
}

void led_test(void) {
    uint8_t i;
    
    __delay_ms(250);
    
    LED_B_ON = 1;
    __delay_ms(250);
    LED_B_ON = 0;
    
    LED_C_ON = 1;
    __delay_ms(250);
    LED_C_ON = 0;
    
    LED_D_ON = 1;
    __delay_ms(250);
    LED_D_ON = 0;
    
    LED_A_ON = 1;
    __delay_ms(250);
    LED_A_ON = 0;
    
    __delay_ms(250);
    
    for (i = 0; i < 4; i++) {
        LED_A_ON = 1;
        LED_B_ON = 1;
        LED_C_ON = 1;
        LED_D_ON = 1;
        
        __delay_ms(125);
        
        LED_A_ON = 0;
        LED_B_ON = 0;
        LED_C_ON = 0;
        LED_D_ON = 0;
        
        __delay_ms(125);
    }
    
}

uint8_t transfer_spi(uint8_t data) {
    SSP1BUF = data;
    
    while (!SSP1STATbits.BF);
    
    data = SSP1BUF;
    
    return data;
}

/* 
 * Sets RF module address and sends packet to specified address
 * 
 * Blocks until packet is acknowledged or until max retries are exceeded
 */
bool send_packet(uint8_t *addr, uint8_t *buf, uint8_t len)
{
    uint8_t status;
    
    nrf24_write_register_multi(NRF24_RX_ADDR_P0, addr, ADDR_LEN);
    nrf24_write_register_multi(NRF24_TX_ADDR, addr, ADDR_LEN);
    
    nrf24_flush_tx();
    nrf24_write_payload(buf, len);

    /* Strobe RF CE line to send one packet of data */
    NRF24_CE = 1;
    __delay_us(15);
    NRF24_CE = 0;

    while (NRF24_IRQ);
    
    status = nrf24_read_register(NRF24_STATUS);
    
    nrf24_write_register(NRF24_STATUS, NRF24_TX_DS | NRF24_MAX_RT);
            
    if (status & NRF24_MAX_RT) {
        return false;
    } else {
        return true;
    }
}

void main(void) {
    uint8_t buf;
    uint8_t retries;
    
    bool success1;
    bool success2;
    
    bool success;
    
    /* Initialize peripherals */
    init_ports();
    init_osc();
    init_spi();
    init_pps();
    init_rf();
    
    led_test();

    while (1) {
        /* This is the ugliest debounce code ever and needs to be fixed even if it works perfectly. */
        if (BUTTON_A_PRESSED) {
            __delay_ms(20);
            
            if (BUTTON_A_PRESSED) {
                LED_A_ON = 1;
                success1 = false;
                success2 = false;
                success = false;
                retries = 0;
                
                buf = BUTTON_A_PAYLOAD;
                
                while (!success && (retries < MAX_SOFT_RETRIES)) {
                    if (!success1) {
                        success1 = send_packet(strip1_addr, &buf, 1);
                    }
                    
                    if (!success2) {
                        success2 = send_packet(strip2_addr, &buf, 1);
                    }
                    
                    success = success1 && success2;
                    
                    if (!success) {
                        retries++;
                        
                        __delay_ms(125);
                        LED_A_ON = 0;
                        __delay_ms(125);
                        LED_A_ON = 1;
                    }
                }
                
                if (success) {
                    LED_A_ON = 1;              
                    __delay_ms(5000);
                }
                LED_A_ON = 0;
            }
        }
        
        if (BUTTON_B_PRESSED) {
            __delay_ms(20);
            
            if (BUTTON_B_PRESSED) {
                LED_B_ON = 1;
                success1 = false;
                success2 = false;
                success = false;
                retries = 0;
                
                buf = BUTTON_B_PAYLOAD;
                
                while (!success && (retries < MAX_SOFT_RETRIES)) {
                    if (!success1) {
                        success1 = send_packet(strip1_addr, &buf, 1);
                    }
                    
                    if (!success2) {
                        success2 = send_packet(strip2_addr, &buf, 1);
                    }
                    
                    success = success1 && success2;
                    
                    if (!success) {
                        retries++;
                        
                        __delay_ms(125);
                        LED_B_ON = 0;
                        __delay_ms(125);
                        LED_B_ON = 1;
                    }
                }
                
                if (success) {
                    LED_B_ON = 1;              
                    __delay_ms(5000);
                }
                LED_B_ON = 0;
            }
        }
        
        if (BUTTON_C_PRESSED) {
            __delay_ms(20);
            
            if (BUTTON_C_PRESSED) {
                LED_C_ON = 1;
                success1 = false;
                success2 = false;
                success = false;
                retries = 0;
                
                buf = BUTTON_C_PAYLOAD;
                
                while (!success && (retries < MAX_SOFT_RETRIES)) {
                    if (!success1) {
                        success1 = send_packet(strip1_addr, &buf, 1);
                    }
                    
                    if (!success2) {
                        success2 = send_packet(strip2_addr, &buf, 1);
                    }
                    
                    success = success1 && success2;
                    
                    if (!success) {
                        retries++;
                        
                        __delay_ms(125);
                        LED_C_ON = 0;
                        __delay_ms(125);
                        LED_C_ON = 1;
                    }
                }
                
                if (success) {
                    LED_C_ON = 1;              
                    __delay_ms(5000);
                }
                LED_C_ON = 0;
            }
        }
        
        if (BUTTON_D_PRESSED) {
            __delay_ms(20);
            
            if (BUTTON_D_PRESSED) {
                LED_D_ON = 1;
                success1 = false;
                success2 = false;
                success = false;
                retries = 0;
                
                buf = BUTTON_D_PAYLOAD;
                
                while (!success && (retries < MAX_SOFT_RETRIES)) {
                    if (!success1) {
                        success1 = send_packet(strip1_addr, &buf, 1);
                    }
                    
                    if (!success2) {
                        success2 = send_packet(strip2_addr, &buf, 1);
                    }
                    
                    success = success1 && success2;
                    
                    if (!success) {
                        retries++;
                        
                        __delay_ms(125);
                        LED_D_ON = 0;
                        __delay_ms(125);
                        LED_D_ON = 1;
                    }
                }
                
                if (success) {
                    LED_D_ON = 1;              
                    __delay_ms(5000);
                }
                LED_D_ON = 0;
            }
        }
    }
}
