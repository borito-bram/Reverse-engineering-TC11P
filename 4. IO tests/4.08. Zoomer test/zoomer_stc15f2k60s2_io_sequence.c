#include <REG51.H>

/* STC15 extended SFRs. */
sfr AUXR = 0x8E;
sfr P2M0 = 0x96;
sfr P2M1 = 0x95;

/* Output bit. */
sbit P2_6 = P2^6;

/*
 * STC15F2K60S2 (LQFP-44)
 * UART1: P3.0 RXD / P3.1 TXD, 115200-8-N-1 @ 11.0592 MHz
 *
 * Sequence: P2.6 HIGH for 10 s, then LOW for 10 s, repeat.
 */

#define FOSC      11059200L
#define BAUD      115200L
#define T1_RELOAD (65536 - FOSC / 4 / BAUD)

/* ---------- UART helpers ---------- */

static void uart_init(void)
{
    AUXR  = 0xC0;
    SCON  = 0x50;
    TMOD  = 0x00;
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

/* ---------- Delay ---------- */

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

/* ---------- Main ---------- */

void main(void)
{
    /* Configure P2 all pins as push-pull output. */
    P2M0 = 0xFF;  P2M1 = 0x00;

    uart_init();

    P2_6 = 0;

    uart_send_str("P2.6 test started.\r\n");

    while (1)
    {
        P2_6 = 1;
        uart_send_str("P2.6 ACTIVE\r\n");
        delay_10s();

        P2_6 = 0;
        uart_send_str("P2.6 NOT ACTIVE\r\n");
        delay_10s();
    }
}
