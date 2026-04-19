#include <REG51.H>

/* Keil C51 declarations for STC15 extended port and output bits. */
sfr P4 = 0xC0;
sbit OUTPUT1 = P3^5;  /* P3.5, LQFP-44 pin 6  */
sbit OUTPUT2 = P4^7;  /* P4.7, LQFP-44 pin 7  */

/*
 * STC15F2K60S2 (LQFP-44)
 * Physical package pin mapping used here:
 *   - Pin 6  -> P3.5 (Output 1)
 *   - Pin 7  -> P4.7 (Output 2)
 *
 * Sequence in loop:
 * 1) Output 1 HIGH (steady),     wait 10 s
 * 2) Output 2 HIGH (steady),     wait 10 s
 * 4) Output 1 LOW  (steady),     wait 10 s
 * 5) Output 2 LOW  (steady),     wait 10 s
 */

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
    OUTPUT1 = 0;
    OUTPUT2 = 0;

    while (1)
    {
        OUTPUT1 = 1;
        delay_10s();

        OUTPUT2 = 1;
        delay_10s();

        OUTPUT1 = 0;
        delay_10s();

        OUTPUT2 = 0;
        delay_10s();
    }
}
