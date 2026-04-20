#include <REG51.H>

/* SFR declarations for STC15 (not in standard REG51.H). */
sfr P4 = 0xC0;
sfr P5 = 0xC8;

sfr P1M1 = 0x91;
sfr P1M0 = 0x92;
sfr P3M1 = 0xB1;
sfr P3M0 = 0xB2;
sfr P4M1 = 0xB3;
sfr P4M0 = 0xB4;
sfr P5M1 = 0xC9;
sfr P5M0 = 0xCA;

sbit OUTPUT1 = P3^5;  /* Voltage regulator enable: P3.5, LQFP-44 pin 6  */
sbit OUTPUT2 = P4^7;  /* Motor up control:        P4.7, LQFP-44 pin 7  */
sbit OUTPUT3 = P1^2;  /* Motor down control:      P1.2 */

sbit INPUT2  = P5^5;  /* Up command input:        P5.5, LQFP-44 pin 15 */
sbit INPUT3  = P4^0;  /* Down command input:      P4.0, LQFP-44 pin 17 */

/*
 * STC15F2K60S2 / STC15F2K48S2 style I/O test
 *
 * Output control follows the reference script:
 * - OUTPUT1 enables the voltage regulator
 * - OUTPUT2 drives motor UP
 * - OUTPUT3 drives motor DOWN
 *
 * Input control used here:
 * - INPUT2 active -> regulator ON + motor UP ON
 * - INPUT3 active -> regulator ON + motor DOWN ON
 * - both active or both inactive -> all outputs OFF
 *
 * This preserves the reference control principle:
 * the regulator is enabled only while one motor direction output is active.
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

static void motor_stop(void)
{
    OUTPUT1 = 0;
    OUTPUT2 = 0;
    OUTPUT3 = 0;
}

static void motor_up(void)
{
    OUTPUT3 = 0;
    OUTPUT1 = 1;
    OUTPUT2 = 1;
}

static void motor_down(void)
{
    OUTPUT2 = 0;
    OUTPUT1 = 1;
    OUTPUT3 = 1;
}

static void io_init(void)
{
    /* Outputs: push-pull for regulator and both motor direction controls. */
    P1M1 &= (unsigned char)~0x04;
    P1M0 |= 0x04;

    P3M1 &= (unsigned char)~0x20;
    P3M0 |= 0x20;

    P4M1 &= (unsigned char)~0x80;
    P4M0 |= 0x80;

    /* Inputs: keep the two button inputs in quasi-bidirectional mode. */
    P5M1 &= (unsigned char)~0x20;
    P5M0 &= (unsigned char)~0x20;

    P4M1 &= (unsigned char)~0x01;
    P4M0 &= (unsigned char)~0x01;

    /* Release input pins high for input sampling. */
    INPUT2 = 1;
    INPUT3 = 1;

    motor_stop();
}

void main(void)
{
    bit up_active;
    bit down_active;

    io_init();

    while (1)
    {
        /* Active-low buttons are typical with quasi-bidirectional STC inputs. */
        up_active = (INPUT2 == 0);
        down_active = (INPUT3 == 0);

        if (up_active && !down_active)
        {
            motor_up();
        }
        else if (down_active && !up_active)
        {
            motor_down();
        }
        else
        {
            motor_stop();
        }

        delay_ms(10);
    }
}