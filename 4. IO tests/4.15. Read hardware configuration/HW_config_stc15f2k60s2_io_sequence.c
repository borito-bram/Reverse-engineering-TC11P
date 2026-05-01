#include <REG51.H>

/* SFR declarations for STC15 (not in standard REG51.H) */
sfr AUXR = 0x8E;
sfr P4   = 0xC0;
sfr P5   = 0xC8;

sfr P0M1  = 0x93;
sfr P0M0  = 0x94;
sfr P1M1  = 0x91;
sfr P1M0  = 0x92;
sfr P2M1  = 0x95;
sfr P2M0  = 0x96;
sfr P3M1  = 0xB1;
sfr P3M0  = 0xB2;
sfr P4M1  = 0xB3;
sfr P4M0  = 0xB4;
sfr P5M1  = 0xC9;
sfr P5M0  = 0xCA;

/* PxM1.n, PxM0.n = 00 -> quasi-bidirectional
 *                 = 01 -> push-pull output
 *                 = 10 -> high-impedance input
 *                 = 11 -> open-drain */
sfr P1ASF = 0x9D;   /* P1 analog function select: 1=ADC/analog, 0=digital */

/*
 * STC15F2K48S2 (LQFP-44) hardware-configuration reader (all GPIO ports).
 *
 * Reads all available GPIO ports P0..P5 and shares raw states on UART1
 * (P3.1 TXD / P3.0 RXD, same serial port used by ISP/programming).
 *
 * Serial commands (115200-8-N-1):
 *   h/?  -> help
 *   p    -> print all possible hardware config states for all pins
 *   r    -> read and print all port states once
 *   c    -> toggle continuous streaming every 500 ms
 *
 * Baud rate formula for STC15 Timer1 1T mode:
 *   reload = 65536 - FOSC/4/baud
 */
#define FOSC        11059200L
#define BAUD        115200L
#define T1_RELOAD   (65536 - FOSC / 4 / BAUD)

static void uart_init(void)
{
    AUXR  = 0xC0;   /* T0x12=1, T1x12=1: both timers in 1T (Fosc) mode */

    /* Full clear - do NOT use read-modify-write; some 2025 silicon batches
     * restore P1ASF from an option byte after reset, so we overwrite it
     * completely and re-read it back to check whether the write took effect. */
    P1ASF = 0x00;

    /* Configure all GPIO as high-Z inputs for passive hardware-state readback.
     * Keep UART pins P3.0/P3.1 in quasi mode so serial remains active. */
    P0M1  = 0xFF; P0M0 = 0x00;
    P1M1  = 0xFF; P1M0 = 0x00;
    P2M1  = 0xFF; P2M0 = 0x00;
    P3M1  = 0xFF; P3M0 = 0x00;
    P4M1  = 0xFF; P4M0 = 0x00;
    P5M1  = 0xFF; P5M0 = 0x00;

    P3M1 &= (unsigned char)~0x03;
    P3M0 &= (unsigned char)~0x03;

    P0 = 0xFF;
    P1 = 0xFF;
    P2 = 0xFF;
    P3 = 0xFF;
    P4 = 0xFF;
    P5 = 0xFF;

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

static bit uart_try_read_char(unsigned char *c)
{
    if (!RI)
        return 0;

    RI = 0;
    *c = SBUF;
    return 1;
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

static void uart_send_hex(unsigned char v)
{
    unsigned char nibble;
    nibble = (v >> 4) & 0x0F;
    uart_send_char((unsigned char)(nibble < 10 ? '0' + nibble : 'A' + nibble - 10));
    nibble = v & 0x0F;
    uart_send_char((unsigned char)(nibble < 10 ? '0' + nibble : 'A' + nibble - 10));
}

static void uart_send_bin8(unsigned char value)
{
    unsigned char mask = 0x80;

    while (mask)
    {
        uart_send_char((value & mask) ? '1' : '0');
        mask >>= 1;
    }
}

static void print_all_possible_configs(void)
{
    uart_send_str("All possible hardware states for all pins:\r\n");
    uart_send_str("  each pin Px.y can be 0 or 1\r\n");
    uart_send_str("  each full port byte Px can be 0x00..0xFF\r\n");
    uart_send_str("  sampled ports in this script: P0, P1, P2, P3, P4, P5\r\n");
    uart_send_str("  theoretical combinations for 48 bits = 2^48\r\n");
}

static void print_port(const char *name, unsigned char value)
{
    uart_send_str(name);
    uart_send_str("=0x");
    uart_send_hex(value);
    uart_send_str(" b");
    uart_send_bin8(value);
    uart_send_str("\r\n");
}

static void print_current_config(void)
{
    print_port("P0", P0);
    print_port("P1", P1);
    print_port("P2", P2);
    print_port("P3", P3);
    print_port("P4", P4);
    print_port("P5", P5);
    uart_send_str("---\r\n");
}

static void print_help(void)
{
    uart_send_str("Commands: h/? help, p all possible states, r read all pins now, c toggle stream\r\n");
}

void main(void)
{
    bit stream_enabled = 1;
    unsigned char c;

    uart_init();

    uart_send_str("HW config reader (all pins) ready\r\n");
    print_help();
    print_all_possible_configs();

    uart_send_str("REG DUMP:\r\n");
    uart_send_str("  P1ASF=0x"); uart_send_hex(P1ASF); uart_send_str("\r\n");
    uart_send_str("  P0M1 =0x"); uart_send_hex(P0M1);  uart_send_str("  P0M0=0x"); uart_send_hex(P0M0); uart_send_str("\r\n");
    uart_send_str("  P1M1 =0x"); uart_send_hex(P1M1);  uart_send_str("  P1M0=0x"); uart_send_hex(P1M0); uart_send_str("\r\n");
    uart_send_str("  P2M1 =0x"); uart_send_hex(P2M1);  uart_send_str("  P2M0=0x"); uart_send_hex(P2M0); uart_send_str("\r\n");
    uart_send_str("  P3M1 =0x"); uart_send_hex(P3M1);  uart_send_str("  P3M0=0x"); uart_send_hex(P3M0); uart_send_str("\r\n");
    uart_send_str("  P4M1 =0x"); uart_send_hex(P4M1);  uart_send_str("  P4M0=0x"); uart_send_hex(P4M0); uart_send_str("\r\n");
    uart_send_str("  P5M1 =0x"); uart_send_hex(P5M1);  uart_send_str("  P5M0=0x"); uart_send_hex(P5M0); uart_send_str("\r\n");
    uart_send_str("---\r\n");

    while (1)
    {
        while (uart_try_read_char(&c))
        {
            if (c == 'h' || c == 'H' || c == '?')
            {
                print_help();
            }
            else if (c == 'p' || c == 'P')
            {
                print_all_possible_configs();
            }
            else if (c == 'r' || c == 'R')
            {
                print_current_config();
            }
            else if (c == 'c' || c == 'C')
            {
                stream_enabled = !stream_enabled;
                uart_send_str("stream=");
                uart_send_str(stream_enabled ? "ON\r\n" : "OFF\r\n");
            }
        }

        if (stream_enabled)
        {
            print_current_config();
        }

        delay_ms(500);
    }
}