#include <REG51.H>

/* SFR declarations for STC15 (not in standard REG51.H) */
sfr AUXR = 0x8E;
sfr P4   = 0xC0;

sfr P1M1 = 0x91;
sfr P1M0 = 0x92;
sfr P2M1 = 0x95;
sfr P2M0 = 0x96;
sfr P3M1 = 0xB1;
sfr P3M0 = 0xB2;
sfr P4M1 = 0xB3;
sfr P4M0 = 0xB4;

sbit OUTPUT1 = P3^5;  /* Voltage regulator enable: P3.5 */
sbit OUTPUT2 = P4^7;  /* Motor up control:         P4.7 */
sbit OUTPUT3 = P1^2;  /* Motor down control:       P1.2 */
sbit PWM_OUT = P2^7;  /* PWM output (push-pull):   P2.7 */

/*
 * STC15F2K48S2 (LQFP-44)
 *
 * Serial protocol on UART1 (P3.0 RXD / P3.1 TXD), 115200-8-N-1:
 *   u / U        -> motor up
 *   d / D        -> motor down
 *   s / S        -> motor stop
 *   0-100 + CR   -> set PWM duty cycle on P2.7 (%)
 *
 * PWM: software PWM on P2.7, ~100 Hz, 1% resolution.
 *      Timer0 fires every 100 us (1T mode @ 11.0592 MHz).
 *
 * Baud rate: 115200 @ 11.0592 MHz, Timer1 1T mode
 */
#define FOSC        11059200L
#define BAUD        115200L
#define T1_RELOAD   (65536 - FOSC / 4 / BAUD)

/* 100 us per PWM tick -> 100 Hz PWM period (100 ticks) */
#define T0_RELOAD   (65536U - 1106U)

#define UART_RX_BUF_LEN  8

/* ---------- PWM state (updated in ISR) ---------- */
static volatile unsigned char pwm_tick = 0;
static volatile unsigned char pwm_duty = 0;   /* 0-100 */

/* ---------- Timer0 ISR: software PWM on P2.7 ---------- */
void timer0_isr(void) interrupt 1
{
    pwm_tick++;
    if (pwm_tick >= 100)
    {
        pwm_tick = 0;
    }
    PWM_OUT = (pwm_tick < pwm_duty) ? 1 : 0;
}

/* ---------- UART ---------- */

static void uart_init(void)
{
    AUXR = 0xC0;   /* T0x12=1, T1x12=1: both timers in 1T mode */

    /* P1.2 push-pull output. */
    P1M1 &= (unsigned char)~0x04;
    P1M0 |= 0x04;

    /* P2.7 push-pull output (PWM). */
    P2M1 &= (unsigned char)~0x80;
    P2M0 |= 0x80;

    /* P3.5 push-pull output. */
    P3M1 &= (unsigned char)~0x20;
    P3M0 |= 0x20;

    /* P4.7 push-pull output. */
    P4M1 &= (unsigned char)~0x80;
    P4M0 |= 0x80;

    /* Timer0: 16-bit auto-reload, PWM tick every 100 us. */
    TMOD = 0x00;
    TH0  = (unsigned char)(T0_RELOAD >> 8);
    TL0  = (unsigned char)(T0_RELOAD);
    ET0  = 1;
    TR0  = 1;

    /* Timer1: baud rate generator for UART1. */
    SCON = 0x50;
    TH1  = (unsigned char)(T1_RELOAD >> 8);
    TL1  = (unsigned char)(T1_RELOAD);
    TR1  = 1;

    EA = 1;
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

static void uart_send_uint(unsigned char value)
{
    unsigned char digits[3];
    unsigned char count = 0;

    if (value == 0)
    {
        uart_send_char('0');
        return;
    }
    while (value != 0)
    {
        digits[count++] = (unsigned char)('0' + (value % 10));
        value /= 10;
    }
    while (count != 0)
    {
        uart_send_char(digits[--count]);
    }
}

static bit uart_try_read_char(unsigned char *c)
{
    if (!RI)
    {
        return 0;
    }
    RI = 0;
    *c = SBUF;
    return 1;
}

/* ---------- Motor control ---------- */

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

/* ---------- Command processing ---------- */

static void process_uart_commands(void)
{
    static char rx_buf[UART_RX_BUF_LEN];
    static unsigned char rx_len = 0;
    unsigned char c;
    unsigned int val;
    unsigned char i;
    bit is_number;

    while (uart_try_read_char(&c))
    {
        /* Single-char motor commands act immediately (no Enter needed). */
        if ((c == 'u') || (c == 'U'))
        {
            motor_up();
            uart_send_str("MOTOR UP\r\n");
            rx_len = 0;
            continue;
        }
        if ((c == 'd') || (c == 'D'))
        {
            motor_down();
            uart_send_str("MOTOR DOWN\r\n");
            rx_len = 0;
            continue;
        }
        if ((c == 's') || (c == 'S'))
        {
            motor_stop();
            uart_send_str("MOTOR STOP\r\n");
            rx_len = 0;
            continue;
        }

        /* Accumulate digits; parse on CR/LF. */
        if ((c == '\r') || (c == '\n'))
        {
            if (rx_len == 0)
            {
                continue;
            }

            is_number = 1;
            val = 0;
            for (i = 0; i < rx_len; i++)
            {
                if ((rx_buf[i] < '0') || (rx_buf[i] > '9'))
                {
                    is_number = 0;
                    break;
                }
                val = val * 10U + (unsigned int)(rx_buf[i] - '0');
            }

            if (is_number)
            {
                if (val > 100U)
                {
                    val = 100U;
                }
                pwm_duty = (unsigned char)val;
                uart_send_str("PWM DUTY ");
                uart_send_uint(pwm_duty);
                uart_send_str("%\r\n");
            }
            else
            {
                uart_send_str("ERR: send 0-100 for PWM, u/d/s for motor\r\n");
            }

            rx_len = 0;
            continue;
        }

        if (rx_len < (UART_RX_BUF_LEN - 1))
        {
            rx_buf[rx_len++] = (char)c;
        }
        else
        {
            rx_len = 0;
            uart_send_str("ERR OVERFLOW\r\n");
        }
    }
}

/* ---------- Main ---------- */

void main(void)
{
    uart_init();
    motor_stop();
    PWM_OUT = 0;

    uart_send_str("Ready. u=motor up  d=motor down  s=motor stop  0-100=PWM duty%\r\n");

    while (1)
    {
        process_uart_commands();
    }
}
