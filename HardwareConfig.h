/* 
 * File:   HardwareConfig.h
 * Author: ppzit
 *
 * Created on 04 September 2013, 15:05
 */

#ifndef HARDWARECONFIG_H
#define	HARDWARECONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif


// PIC24FJ64GB004 Configuration Bit Settings

#include <p24Fxxxx.h>

 int CONFIG4 __attribute__((space(prog), address(0xABF8))) = 0xFF7F ;
//_CONFIG1 (
//    DSWDTPS_DSWDTPSF &   // DSWDT Postscale Select (1:2,147,483,648 (25.7 days))
//    DSWDTOSC_LPRC &      // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
//    RTCOSC_SOSC &        // RTCC Reference Oscillator Select (RTCC uses Secondary Oscillator (SOSC))
//    DSBOREN_ON &         // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
//    DSWDTEN_OFF          // Deep Sleep Watchdog Timer (DSWDT disabled)
//);
 int CONFIG3 __attribute__((space(prog), address(0xABFA))) = 0xFCFF ;
//_CONFIG2 (
//    WPFP_WPFP63 &        // Write Protection Flash Page Segment Boundary (Highest Page (same as page 42))
//    SOSCSEL_IO &         // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
//    WUTSEL_LEG &         // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
//    WPDIS_WPDIS &        // Segment Write Protection Disable (Segmented code protection disabled)
//    WPCFG_WPCFGDIS &     // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
//    WPEND_WPENDMEM       // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)
//);
 int CONFIG2 __attribute__((space(prog), address(0xABFC))) = 0x89DF ;
//_CONFIG3 (
//    POSCMOD_NONE &       // Primary Oscillator Select (Primary Oscillator disabled)
//    I2C1SEL_PRI &        // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
//    IOL1WAY_ON &         // IOLOCK One-Way Set Enable (Once set, the IOLOCK bit cannot be cleared)
//    OSCIOFNC_ON &        // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
//    FCKSM_CSDCMD &       // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
//    FNOSC_FRCPLL &       // Initial Oscillator Select (Fast RC Oscillator with Postscaler and PLL module (FRCPLL))
//    PLL96MHZ_ON &        // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)
//    PLLDIV_NODIV &       // USB 96 MHz PLL Prescaler Select (Oscillator input used directly (4 MHz input))
//    IESO_ON              // Internal External Switchover (IESO mode (Two-Speed Start-up) enabled)
//);
 int CONFIG1 __attribute__((space(prog), address(0xABFE))) = 0x3D7F ;
//_CONFIG4 (
//    WDTPS_PS32768 &      // Watchdog Timer Postscaler (1:32,768)
//    FWPSA_PR128 &        // WDT Prescaler (Prescaler ratio of 1:128)
//    WINDIS_OFF &         // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
//    FWDTEN_OFF &         // Watchdog Timer (Watchdog Timer is disabled)
//    ICS_PGx3 &           // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC3/PGED3)
//    GWRP_OFF &           // General Segment Write Protect (Writes to program memory are allowed)
//    GCP_OFF &            // General Segment Code Protect (Code protection is disabled)
//    JTAGEN_OFF           // JTAG Port Enable (JTAG port is disabled)
//);



#ifdef	__cplusplus
}
#endif

#endif	/* HARDWARECONFIG_H */

