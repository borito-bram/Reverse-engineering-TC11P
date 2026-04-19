/*
 * STC15F2K60S2 (LQFP-44)
 *   - P3.5 (Output 1) = voltage regulator enable
 *   - P1.2 (Output 2) = relay 1
 *   - P4.7 (Output 3) = relay 2
 *
 * Output 1 HIGH (steady), always
 * Sequence in loop:
 * 1) Output 2 HIGH (steady),     wait 10 s
 * 2) Output 2 LOW  (steady),     wait 10 s
 * 3) Output 3 HIGH (steady),     wait 10 s
 * 4) Output 3 LOW  (steady),     wait 10 s
 */

#include <REG51.H>

#ifdef SDCC
/* SDCC declarations for STC15 extended port and output bits. */
__sbit __at (0xB5) OUTPUT1; /* P3.5, LQFP-44 pin 6  */
__sbit __at (0x92) OUTPUT2; /* P1.2, LQFP-44 pin 7  */
__sbit __at (0xE7) OUTPUT3; /* P4.7, LQFP-44 pin 43 */
#else
/* Keil C51 declarations for output bits. */
sbit OUTPUT1 = P3^5;  /* LQFP-44 pin 6  */
sbit OUTPUT2 = P1^2;  /* LQFP-44 pin 7  */
sbit OUTPUT3 = P4^7;  /* LQFP-44 pin 43 */
#endif

/*
 * Approximate 1 ms delay for ~11.0592 MHz 8051 clock.
 * If your clock differs, adjust the loop constants.
 */
static void delay_ms(unsigned int ms)
{
    unsigned int i;
    unsigned char j;

    while (ms--)
    {
        i = 2;
        j = 199;
        do
        {
            while (--j);
        }
        while (--i);
    }
}

static void delay_10s(void)
{
    unsigned char count;

    for (count = 0; count < 10; count++)
    {
        delay_ms(1000);
    }
}

void main(void)
{
    OUTPUT1 = 1; /* voltage regulator enable: always HIGH */
    OUTPUT2 = 0;
    OUTPUT3 = 0;

    while (1)
    {
        OUTPUT2 = 1;   /* relay 1 ON  */
        delay_10s();

        OUTPUT2 = 0;   /* relay 1 OFF */
        delay_10s();

        OUTPUT3 = 1;   /* relay 2 ON  */
        delay_10s();

        OUTPUT3 = 0;   /* relay 2 OFF */
        delay_10s();
    }
}
