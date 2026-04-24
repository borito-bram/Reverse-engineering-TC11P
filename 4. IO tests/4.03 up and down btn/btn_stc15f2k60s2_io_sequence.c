#include <REG51.H>

/* SFR declarations for STC15 (not in standard REG51.H) */
sfr AUXR = 0x8E;
sfr P4   = 0xC0;
sfr P5   = 0xC8;

sfr P3M1 = 0xB1;   /* PxM1.n, PxM0.n = 00 -> quasi-bidirectional  */
sfr P3M0 = 0xB2;   /*                 = 01 -> push-pull output     */
sfr P4M1 = 0xB3;   /*                 = 10 -> high-impedance input */
sfr P4M0 = 0xB4;   /*                 = 11 -> open-drain           */
sfr P5M1 = 0xC9;
sfr P5M0 = 0xCA;

sbit OUTPUT1 = P3^3;  /* P3.3, LQFP-44 pin 21 - push-pull output          */
sbit INPUT1  = P5^4;  /* P5.4, LQFP-44 pin 13 - quasi-bidirectional input */
sbit INPUT2  = P5^5;  /* P5.5, LQFP-44 pin 15 - high-impedance input      */
sbit INPUT3  = P4^0;  /* P4.0, LQFP-44 pin 17 - open-drain input          */

/*
 * STC15F2K48S2 (LQFP-44)
 * Physical package pin mapping used here:
 *
 *    - Pin 21  -> P3.3 (Output 1)
 *    - Pin 13  -> P5.4 (Input 1) - quasi-bidirectional: internal weak pull-up
 *    - Pin 15  -> P5.5 (Input 2) - high-impedance:      no pull-up, needs external
 *    - Pin 17  -> P4.0 (Input 3) - open-drain:          no pull-up, needs external
 *
 * 1) Output 1 HIGH (steady)
 * Serialprint if Input 1 is HIGH, else print "Input 1 LOW"
 * Serialprint if Input 2 is HIGH, else print "Input 2 LOW"
 * Serialprint if Input 3 is HIGH, else print "Input 3 LOW"
 * Serialprint same as programming port (P3.0 RXD / P3.1 TXD, UART1)
 *
 * Baud rate: 115200 @ 11.0592 MHz, Timer1 1T mode (AUXR.T1x12=1)
 * Formula (from STC reference): 65536 - FOSC/4/baud
 *   = 65536 - 11059200/4/115200 = 65512 = 0xFFE8
 */
#define FOSC        11059200L
#define BAUD        115200L
#define T1_RELOAD   (65536 - FOSC / 4 / BAUD)

/*
 * UART1 init: 115200 baud, mode 1 (8-bit), no parity.
 * Timer1 in STC15 mode 0 (16-bit auto-reload), 1T.
 * UART1 TX = P3.1, RX = P3.0 (same as ISP programming port).
 *
 * Pin modes (PxM1:PxM0):
 *   P3.3  -> 01 push-pull        (OUTPUT1)
 *   P5.4  -> 00 quasi-bidir      (INPUT1, internal weak pull-up)
 *   P5.5  -> 10 high-impedance   (INPUT2, pure input, needs external pull-up/down)
 *   P4.0  -> 11 open-drain       (INPUT3, no drive, needs external pull-up)
 */
static void uart_init(void)
{
    AUXR  = 0xC0;   /* T0x12=1, T1x12=1: both timers in 1T (Fosc) mode */

    /* P3: bit3 = push-pull (M1=0,M0=1); rest quasi-bidir */
    P3M1  = 0x00;
    P3M0  = 0x08;   /* 0000_1000 -> P3.3 push-pull */

    /* P5: bit4 = quasi-bidir (M1=0,M0=0); bit5 = high-Z (M1=1,M0=0) */
    P5M1  = 0x20;   /* 0010_0000 -> P5.5 high-impedance */
    P5M0  = 0x00;   /* P5.4 quasi-bidirectional (bits clear) */

    /* P4: bit0 = open-drain (M1=1,M0=1) */
    P4M1  = 0x01;   /* 0000_0001 -> P4.0 open-drain */
    P4M0  = 0x01;

    SCON  = 0x50;   /* UART1 mode 1, 8-bit, REN=1 */
    TMOD  = 0x00;   /* Timer0/1 mode 0 (16-bit auto-reload) */
    TH1   = (unsigned char)(T1_RELOAD >> 8);
    TL1   = (unsigned char)(T1_RELOAD);
    TR1   = 1;
}

static void uart_send_char(unsigned char c)
{
    SBUF = c;
    while (!TI);
    TI = 0;
}

static void uart_send_str(const char *s)
{
    while (*s)
    {
        uart_send_char((unsigned char)*s);
        s++;
    }
}

/*
 * Approximate 1 ms delay @ 11.0592 MHz (STC15 always runs in 1T CPU mode).
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

void main(void)
{
    uart_init();

    /* Write 1 to input pins to enable reading in quasi-bidir / open-drain modes */
    INPUT1 = 1;
    INPUT2 = 1;
    INPUT3 = 1;

    /* Output 1 steady LOW */
    OUTPUT1 = 0;

    while (1)
    {
        /* P5.4 - quasi-bidirectional (internal weak pull-up) */
        if (INPUT1)
            uart_send_str("Input 1 HIGH [quasi-bidir]\r\n");
        else
            uart_send_str("Input 1 LOW  [quasi-bidir]\r\n");

        /* P5.5 - high-impedance (pure input, no pull-up) */
        if (INPUT2)
            uart_send_str("Input 2 HIGH [high-Z]\r\n");
        else
            uart_send_str("Input 2 LOW  [high-Z]\r\n");

        /* P4.0 - open-drain (no internal pull-up) */
        if (INPUT3)
            uart_send_str("Input 3 HIGH [open-drain]\r\n");
        else
            uart_send_str("Input 3 LOW  [open-drain]\r\n");

        uart_send_str("---\r\n");
        delay_ms(1000);
    }
}