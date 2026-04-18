#include <REG51.H>

#ifdef SDCC
/* SDCC declarations for STC15 extended port and output bits. */
__sbit __at (0xB5) OUTPUT1; /* P3.5, LQFP-44 pin 6  */
__sbit __at (0xA7) OUTPUT2; /* P2.7 */
__sbit __at (0xA6) OUTPUT3; /* P2.6, LQFP-44 pin 36 */
#else
/* Keil C51 declarations for output bits. */
sbit OUTPUT1 = P3^5;  /* LQFP-44 pin 6  */
sbit OUTPUT2 = P2^7;  /* P2.7 */
sbit OUTPUT3 = P2^6;  /* LQFP-44 pin 36 */
#endif

/*
 * STC15F2K60S2 (LQFP-44)
 * Physical package pin mapping used here:
 *   - Pin 6  -> P3.5 (Output 1)
 *   - P2.7 (Output 2)
 *   - Pin 36 -> P2.6 (Output 3)
 *
 * Sequence in loop:
 * 1) Output 1 HIGH (steady),     wait 10 s
 * 2) Output 2 PWM 200 Hz,        run 10 s
 * 3) Output 3 HIGH (steady),     wait 10 s
 * 4) Output 1 LOW  (steady),     wait 10 s
 * 5) Output 2 LOW  (steady),     wait 10 s
 * 6) Output 3 LOW  (steady),     wait 10 s
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

/* 200 Hz PWM for 10 seconds using a 5 ms software period. */
static void pwm_output2_10s(void)
{
    unsigned int cycles;

    for (cycles = 0; cycles < 2000; cycles++)
    {
        OUTPUT2 = 1;
        delay_ms(2);
        OUTPUT2 = 0;
        delay_ms(3);
    }
}

void main(void)
{
    OUTPUT1 = 0;
    OUTPUT2 = 0;
    OUTPUT3 = 0;

    while (1)
    {
        OUTPUT1 = 1;
        delay_10s();

        pwm_output2_10s();

        OUTPUT3 = 1;
        delay_10s();

        OUTPUT1 = 0;
        delay_10s();

        OUTPUT2 = 0;
        delay_10s();

        OUTPUT3 = 0;
        delay_10s();
    }
}
