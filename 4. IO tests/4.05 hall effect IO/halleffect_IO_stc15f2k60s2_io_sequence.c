#include <REG51.H>

/* SFR declarations for STC15 (not in standard REG51.H) */
sfr AUXR = 0x8E;

sfr P1M1 = 0x91;   /* PxM1.n, PxM0.n = 00 -> quasi-bidirectional  */
sfr P1M0 = 0x92;   /*                 = 01 -> push-pull output     */
                   /*                 = 10 -> high-impedance input */
                   /*                 = 11 -> open-drain           */

sbit INPUT1  = P1^4;  /* P1.4 - quasi-bidirectional input */
sbit INPUT2  = P1^5;  /* P1.5 - high-impedance input      */

/*
 * STC15F2K48S2 (LQFP-44)
 * Physical package pin mapping used here:
 *
 *    - P1.4  -> Input 1 - quasi-bidirectional: internal weak pull-up
 *    - P1.5  -> Input 2 - high-impedance:      no pull-up, needs external
 *
 * Monitors and prints state of P1.4 and P1.5 every second.
 * Serialprint same as programming port (P3.0 RXD / P3.1 TXD, UART1)
 *
 * Baud rate: 9600 @ 11.0592 MHz, Timer1 1T mode (AUXR.T1x12=1)
 * Formula (from STC reference): 65536 - FOSC/4/baud
 *   = 65536 - 11059200/4/9600 = 65248 = 0xFF20
 */
#define FOSC        11059200L
#define BAUD        9600L
#define T1_RELOAD   (65536 - FOSC / 4 / BAUD)

/*
 * UART1 init: 9600 baud, mode 1 (8-bit), no parity.
 * Timer1 in STC15 mode 0 (16-bit auto-reload), 1T.
 * UART1 TX = P3.1, RX = P3.0 (same as ISP programming port).
 *
 * Pin modes (PxM1:PxM0):
 *   P1.4  -> 00 quasi-bidir      (INPUT1, internal weak pull-up)
 *   P1.5  -> 10 high-impedance   (INPUT2, pure input, needs external pull-up/down)
 */
static void uart_init(void)
{
    AUXR  = 0xC0;   /* T0x12=1, T1x12=1: both timers in 1T (Fosc) mode */

    /* P1: bit4 = quasi-bidir (M1=0,M0=0); bit5 = high-Z (M1=1,M0=0) */
    P1M1  = 0x20;   /* 0010_0000 -> P1.5 high-impedance */
    P1M0  = 0x00;   /* P1.4 quasi-bidirectional (bits clear) */

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

    /* Write 1 to input pins to enable reading in quasi-bidir mode */
    INPUT1 = 1;
    INPUT2 = 1;

    while (1)
    {
        /* P1.4 - quasi-bidirectional (internal weak pull-up) */
        if (INPUT1)
            uart_send_str("P1.4 HIGH [quasi-bidir]\r\n");
        else
            uart_send_str("P1.4 LOW  [quasi-bidir]\r\n");

        /* P1.5 - high-impedance (pure input, no pull-up) */
        if (INPUT2)
            uart_send_str("P1.5 HIGH [high-Z]\r\n");
        else
            uart_send_str("P1.5 LOW  [high-Z]\r\n");

        uart_send_str("---\r\n");
        delay_ms(1000);
    }
}