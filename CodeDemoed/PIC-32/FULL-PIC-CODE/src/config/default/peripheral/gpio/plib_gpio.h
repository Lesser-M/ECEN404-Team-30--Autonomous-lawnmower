/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for GPIO_RC2 pin ***/
#define GPIO_RC2_Set()               (LATCSET = (1<<2))
#define GPIO_RC2_Clear()             (LATCCLR = (1<<2))
#define GPIO_RC2_Toggle()            (LATCINV= (1<<2))
#define GPIO_RC2_OutputEnable()      (TRISCCLR = (1<<2))
#define GPIO_RC2_InputEnable()       (TRISCSET = (1<<2))
#define GPIO_RC2_Get()               ((PORTC >> 2) & 0x1)
#define GPIO_RC2_PIN                  GPIO_PIN_RC2

/*** Macros for GPIO_RC4 pin ***/
#define GPIO_RC4_Set()               (LATCSET = (1<<4))
#define GPIO_RC4_Clear()             (LATCCLR = (1<<4))
#define GPIO_RC4_Toggle()            (LATCINV= (1<<4))
#define GPIO_RC4_OutputEnable()      (TRISCCLR = (1<<4))
#define GPIO_RC4_InputEnable()       (TRISCSET = (1<<4))
#define GPIO_RC4_Get()               ((PORTC >> 4) & 0x1)
#define GPIO_RC4_PIN                  GPIO_PIN_RC4

/*** Macros for GPIO_RB6 pin ***/
#define GPIO_RB6_Set()               (LATBSET = (1<<6))
#define GPIO_RB6_Clear()             (LATBCLR = (1<<6))
#define GPIO_RB6_Toggle()            (LATBINV= (1<<6))
#define GPIO_RB6_OutputEnable()      (TRISBCLR = (1<<6))
#define GPIO_RB6_InputEnable()       (TRISBSET = (1<<6))
#define GPIO_RB6_Get()               ((PORTB >> 6) & 0x1)
#define GPIO_RB6_PIN                  GPIO_PIN_RB6

/*** Macros for GPIO_RB7 pin ***/
#define GPIO_RB7_Set()               (LATBSET = (1<<7))
#define GPIO_RB7_Clear()             (LATBCLR = (1<<7))
#define GPIO_RB7_Toggle()            (LATBINV= (1<<7))
#define GPIO_RB7_OutputEnable()      (TRISBCLR = (1<<7))
#define GPIO_RB7_InputEnable()       (TRISBSET = (1<<7))
#define GPIO_RB7_Get()               ((PORTB >> 7) & 0x1)
#define GPIO_RB7_PIN                  GPIO_PIN_RB7

/*** Macros for GPIO_RB8 pin ***/
#define GPIO_RB8_Set()               (LATBSET = (1<<8))
#define GPIO_RB8_Clear()             (LATBCLR = (1<<8))
#define GPIO_RB8_Toggle()            (LATBINV= (1<<8))
#define GPIO_RB8_OutputEnable()      (TRISBCLR = (1<<8))
#define GPIO_RB8_InputEnable()       (TRISBSET = (1<<8))
#define GPIO_RB8_Get()               ((PORTB >> 8) & 0x1)
#define GPIO_RB8_PIN                  GPIO_PIN_RB8

/*** Macros for GPIO_RB10 pin ***/
#define GPIO_RB10_Set()               (LATBSET = (1<<10))
#define GPIO_RB10_Clear()             (LATBCLR = (1<<10))
#define GPIO_RB10_Toggle()            (LATBINV= (1<<10))
#define GPIO_RB10_OutputEnable()      (TRISBCLR = (1<<10))
#define GPIO_RB10_InputEnable()       (TRISBSET = (1<<10))
#define GPIO_RB10_Get()               ((PORTB >> 10) & 0x1)
#define GPIO_RB10_PIN                  GPIO_PIN_RB10

/*** Macros for GPIO_RB12 pin ***/
#define GPIO_RB12_Set()               (LATBSET = (1<<12))
#define GPIO_RB12_Clear()             (LATBCLR = (1<<12))
#define GPIO_RB12_Toggle()            (LATBINV= (1<<12))
#define GPIO_RB12_OutputEnable()      (TRISBCLR = (1<<12))
#define GPIO_RB12_InputEnable()       (TRISBSET = (1<<12))
#define GPIO_RB12_Get()               ((PORTB >> 12) & 0x1)
#define GPIO_RB12_PIN                  GPIO_PIN_RB12

/*** Macros for GPIO_RB14 pin ***/
#define GPIO_RB14_Set()               (LATBSET = (1<<14))
#define GPIO_RB14_Clear()             (LATBCLR = (1<<14))
#define GPIO_RB14_Toggle()            (LATBINV= (1<<14))
#define GPIO_RB14_OutputEnable()      (TRISBCLR = (1<<14))
#define GPIO_RB14_InputEnable()       (TRISBSET = (1<<14))
#define GPIO_RB14_Get()               ((PORTB >> 14) & 0x1)
#define GPIO_RB14_PIN                  GPIO_PIN_RB14

/*** Macros for GPIO_RF3 pin ***/
#define GPIO_RF3_Set()               (LATFSET = (1<<3))
#define GPIO_RF3_Clear()             (LATFCLR = (1<<3))
#define GPIO_RF3_Toggle()            (LATFINV= (1<<3))
#define GPIO_RF3_OutputEnable()      (TRISFCLR = (1<<3))
#define GPIO_RF3_InputEnable()       (TRISFSET = (1<<3))
#define GPIO_RF3_Get()               ((PORTF >> 3) & 0x1)
#define GPIO_RF3_PIN                  GPIO_PIN_RF3

/*** Macros for GPIO_RF2 pin ***/
#define GPIO_RF2_Set()               (LATFSET = (1<<2))
#define GPIO_RF2_Clear()             (LATFCLR = (1<<2))
#define GPIO_RF2_Toggle()            (LATFINV= (1<<2))
#define GPIO_RF2_OutputEnable()      (TRISFCLR = (1<<2))
#define GPIO_RF2_InputEnable()       (TRISFSET = (1<<2))
#define GPIO_RF2_Get()               ((PORTF >> 2) & 0x1)
#define GPIO_RF2_PIN                  GPIO_PIN_RF2

/*** Macros for GPIO_RH9 pin ***/
#define GPIO_RH9_Set()               (LATHSET = (1<<9))
#define GPIO_RH9_Clear()             (LATHCLR = (1<<9))
#define GPIO_RH9_Toggle()            (LATHINV= (1<<9))
#define GPIO_RH9_OutputEnable()      (TRISHCLR = (1<<9))
#define GPIO_RH9_InputEnable()       (TRISHSET = (1<<9))
#define GPIO_RH9_Get()               ((PORTH >> 9) & 0x1)
#define GPIO_RH9_PIN                  GPIO_PIN_RH9

/*** Macros for GPIO_RH11 pin ***/
#define GPIO_RH11_Set()               (LATHSET = (1<<11))
#define GPIO_RH11_Clear()             (LATHCLR = (1<<11))
#define GPIO_RH11_Toggle()            (LATHINV= (1<<11))
#define GPIO_RH11_OutputEnable()      (TRISHCLR = (1<<11))
#define GPIO_RH11_InputEnable()       (TRISHSET = (1<<11))
#define GPIO_RH11_Get()               ((PORTH >> 11) & 0x1)
#define GPIO_RH11_PIN                  GPIO_PIN_RH11

/*** Macros for GPIO_RD2 pin ***/
#define GPIO_RD2_Set()               (LATDSET = (1<<2))
#define GPIO_RD2_Clear()             (LATDCLR = (1<<2))
#define GPIO_RD2_Toggle()            (LATDINV= (1<<2))
#define GPIO_RD2_OutputEnable()      (TRISDCLR = (1<<2))
#define GPIO_RD2_InputEnable()       (TRISDSET = (1<<2))
#define GPIO_RD2_Get()               ((PORTD >> 2) & 0x1)
#define GPIO_RD2_PIN                  GPIO_PIN_RD2

/*** Macros for GPIO_RD13 pin ***/
#define GPIO_RD13_Set()               (LATDSET = (1<<13))
#define GPIO_RD13_Clear()             (LATDCLR = (1<<13))
#define GPIO_RD13_Toggle()            (LATDINV= (1<<13))
#define GPIO_RD13_OutputEnable()      (TRISDCLR = (1<<13))
#define GPIO_RD13_InputEnable()       (TRISDSET = (1<<13))
#define GPIO_RD13_Get()               ((PORTD >> 13) & 0x1)
#define GPIO_RD13_PIN                  GPIO_PIN_RD13

/*** Macros for GPIO_RJ1 pin ***/
#define GPIO_RJ1_Set()               (LATJSET = (1<<1))
#define GPIO_RJ1_Clear()             (LATJCLR = (1<<1))
#define GPIO_RJ1_Toggle()            (LATJINV= (1<<1))
#define GPIO_RJ1_OutputEnable()      (TRISJCLR = (1<<1))
#define GPIO_RJ1_InputEnable()       (TRISJSET = (1<<1))
#define GPIO_RJ1_Get()               ((PORTJ >> 1) & 0x1)
#define GPIO_RJ1_PIN                  GPIO_PIN_RJ1

/*** Macros for GPIO_RG14 pin ***/
#define GPIO_RG14_Set()               (LATGSET = (1<<14))
#define GPIO_RG14_Clear()             (LATGCLR = (1<<14))
#define GPIO_RG14_Toggle()            (LATGINV= (1<<14))
#define GPIO_RG14_OutputEnable()      (TRISGCLR = (1<<14))
#define GPIO_RG14_InputEnable()       (TRISGSET = (1<<14))
#define GPIO_RG14_Get()               ((PORTG >> 14) & 0x1)
#define GPIO_RG14_PIN                  GPIO_PIN_RG14

/*** Macros for GPIO_RG13 pin ***/
#define GPIO_RG13_Set()               (LATGSET = (1<<13))
#define GPIO_RG13_Clear()             (LATGCLR = (1<<13))
#define GPIO_RG13_Toggle()            (LATGINV= (1<<13))
#define GPIO_RG13_OutputEnable()      (TRISGCLR = (1<<13))
#define GPIO_RG13_InputEnable()       (TRISGSET = (1<<13))
#define GPIO_RG13_Get()               ((PORTG >> 13) & 0x1)
#define GPIO_RG13_PIN                  GPIO_PIN_RG13

/*** Macros for GPIO_RE3 pin ***/
#define GPIO_RE3_Set()               (LATESET = (1<<3))
#define GPIO_RE3_Clear()             (LATECLR = (1<<3))
#define GPIO_RE3_Toggle()            (LATEINV= (1<<3))
#define GPIO_RE3_OutputEnable()      (TRISECLR = (1<<3))
#define GPIO_RE3_InputEnable()       (TRISESET = (1<<3))
#define GPIO_RE3_Get()               ((PORTE >> 3) & 0x1)
#define GPIO_RE3_PIN                  GPIO_PIN_RE3


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/

typedef enum
{
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_F = 5,
    GPIO_PORT_G = 6,
    GPIO_PORT_H = 7,
    GPIO_PORT_J = 8,
    GPIO_PORT_K = 9,
} GPIO_PORT;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/

typedef enum
{
    GPIO_PIN_RA0 = 0,
    GPIO_PIN_RA1 = 1,
    GPIO_PIN_RA2 = 2,
    GPIO_PIN_RA3 = 3,
    GPIO_PIN_RA4 = 4,
    GPIO_PIN_RA5 = 5,
    GPIO_PIN_RA6 = 6,
    GPIO_PIN_RA7 = 7,
    GPIO_PIN_RA9 = 9,
    GPIO_PIN_RA10 = 10,
    GPIO_PIN_RA14 = 14,
    GPIO_PIN_RA15 = 15,
    GPIO_PIN_RB0 = 16,
    GPIO_PIN_RB1 = 17,
    GPIO_PIN_RB2 = 18,
    GPIO_PIN_RB3 = 19,
    GPIO_PIN_RB4 = 20,
    GPIO_PIN_RB5 = 21,
    GPIO_PIN_RB6 = 22,
    GPIO_PIN_RB7 = 23,
    GPIO_PIN_RB8 = 24,
    GPIO_PIN_RB9 = 25,
    GPIO_PIN_RB10 = 26,
    GPIO_PIN_RB11 = 27,
    GPIO_PIN_RB12 = 28,
    GPIO_PIN_RB13 = 29,
    GPIO_PIN_RB14 = 30,
    GPIO_PIN_RB15 = 31,
    GPIO_PIN_RC1 = 33,
    GPIO_PIN_RC2 = 34,
    GPIO_PIN_RC3 = 35,
    GPIO_PIN_RC4 = 36,
    GPIO_PIN_RC12 = 44,
    GPIO_PIN_RC13 = 45,
    GPIO_PIN_RC14 = 46,
    GPIO_PIN_RC15 = 47,
    GPIO_PIN_RD0 = 48,
    GPIO_PIN_RD1 = 49,
    GPIO_PIN_RD2 = 50,
    GPIO_PIN_RD3 = 51,
    GPIO_PIN_RD4 = 52,
    GPIO_PIN_RD5 = 53,
    GPIO_PIN_RD6 = 54,
    GPIO_PIN_RD7 = 55,
    GPIO_PIN_RD9 = 57,
    GPIO_PIN_RD10 = 58,
    GPIO_PIN_RD11 = 59,
    GPIO_PIN_RD12 = 60,
    GPIO_PIN_RD13 = 61,
    GPIO_PIN_RD14 = 62,
    GPIO_PIN_RD15 = 63,
    GPIO_PIN_RE0 = 64,
    GPIO_PIN_RE1 = 65,
    GPIO_PIN_RE2 = 66,
    GPIO_PIN_RE3 = 67,
    GPIO_PIN_RE4 = 68,
    GPIO_PIN_RE5 = 69,
    GPIO_PIN_RE6 = 70,
    GPIO_PIN_RE7 = 71,
    GPIO_PIN_RE8 = 72,
    GPIO_PIN_RE9 = 73,
    GPIO_PIN_RF0 = 80,
    GPIO_PIN_RF1 = 81,
    GPIO_PIN_RF2 = 82,
    GPIO_PIN_RF3 = 83,
    GPIO_PIN_RF4 = 84,
    GPIO_PIN_RF5 = 85,
    GPIO_PIN_RF8 = 88,
    GPIO_PIN_RF12 = 92,
    GPIO_PIN_RF13 = 93,
    GPIO_PIN_RG0 = 96,
    GPIO_PIN_RG1 = 97,
    GPIO_PIN_RG6 = 102,
    GPIO_PIN_RG7 = 103,
    GPIO_PIN_RG8 = 104,
    GPIO_PIN_RG9 = 105,
    GPIO_PIN_RG12 = 108,
    GPIO_PIN_RG13 = 109,
    GPIO_PIN_RG14 = 110,
    GPIO_PIN_RG15 = 111,
    GPIO_PIN_RH0 = 112,
    GPIO_PIN_RH1 = 113,
    GPIO_PIN_RH2 = 114,
    GPIO_PIN_RH3 = 115,
    GPIO_PIN_RH4 = 116,
    GPIO_PIN_RH5 = 117,
    GPIO_PIN_RH6 = 118,
    GPIO_PIN_RH7 = 119,
    GPIO_PIN_RH8 = 120,
    GPIO_PIN_RH9 = 121,
    GPIO_PIN_RH10 = 122,
    GPIO_PIN_RH11 = 123,
    GPIO_PIN_RH12 = 124,
    GPIO_PIN_RH13 = 125,
    GPIO_PIN_RH14 = 126,
    GPIO_PIN_RH15 = 127,
    GPIO_PIN_RJ0 = 128,
    GPIO_PIN_RJ1 = 129,
    GPIO_PIN_RJ2 = 130,
    GPIO_PIN_RJ3 = 131,
    GPIO_PIN_RJ4 = 132,
    GPIO_PIN_RJ5 = 133,
    GPIO_PIN_RJ6 = 134,
    GPIO_PIN_RJ7 = 135,
    GPIO_PIN_RJ8 = 136,
    GPIO_PIN_RJ9 = 137,
    GPIO_PIN_RJ10 = 138,
    GPIO_PIN_RJ11 = 139,
    GPIO_PIN_RJ12 = 140,
    GPIO_PIN_RJ13 = 141,
    GPIO_PIN_RJ14 = 142,
    GPIO_PIN_RJ15 = 143,
    GPIO_PIN_RK0 = 144,
    GPIO_PIN_RK1 = 145,
    GPIO_PIN_RK2 = 146,
    GPIO_PIN_RK3 = 147,
    GPIO_PIN_RK4 = 148,
    GPIO_PIN_RK5 = 149,
    GPIO_PIN_RK6 = 150,
    GPIO_PIN_RK7 = 151,

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
    GPIO_PIN_NONE = -1

} GPIO_PIN;


void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
    GPIO_PortWrite((GPIO_PORT)(pin>>4), (uint32_t)(0x1) << (pin & 0xF), (uint32_t)(value) << (pin & 0xF));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return (bool)(((GPIO_PortRead((GPIO_PORT)(pin>>4))) >> (pin & 0xF)) & 0x1);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (bool)((GPIO_PortLatchRead((GPIO_PORT)(pin>>4)) >> (pin & 0xF)) & 0x1);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
