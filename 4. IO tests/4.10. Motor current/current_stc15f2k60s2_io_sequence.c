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

/* --- ADC SFRs (STC15) --- */
sfr P1ASF     = 0x9D; /* P1 analog function enable (1 = analog) */
sfr ADC_CONTR = 0xBC; /* ADC control register                   */
sfr ADC_RES   = 0xBD; /* ADC result high 8 bits (left-adjusted) */
sfr ADC_RESL  = 0xBE; /* ADC result low 2 bits                  */

#define ADC_POWER   0x80 /* power on ADC module       */
#define ADC_SPEEDLL 0x00 /* 540 clocks/conv (slowest) */
#define ADC_FLAG    0x10 /* conversion-done flag      */
#define ADC_START   0x08 /* start conversion          */
#define ADC_CH(n)   ((n) & 0x07)

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

/* ADC report timer: incremented every 100 us by Timer0 ISR */
static volatile unsigned int adc_tick = 0;
#define ADC_REPORT_TICKS 5000U /* 5000 x 100 us = 500 ms */

/* ---------- Timer0 ISR: software PWM on P2.7 + ADC tick ---------- */
void timer0_isr(void) interrupt 1
{
    pwm_tick++;
    if (pwm_tick >= 100)
    {
        pwm_tick = 0;
    }
    PWM_OUT = (pwm_tick < pwm_duty) ? 1 : 0;

    if (adc_tick < 65535U)
    {
        adc_tick++;
    }
}

/* ---------- ADC ---------- */

static void adc_init(void)
{
    /* P1.0 = current sense (ADC ch0), P1.1 = supply voltage (ADC ch1) */
    P1ASF    = 0x03;                        /* mark P1.0 and P1.1 as analog */
    ADC_CONTR = ADC_POWER | ADC_SPEEDLL;    /* power up ADC, slow conversion */
    /* First throwaway conversion in adc_read_stable() handles ADC settling. */
}

/* Returns one 10-bit result (0..1023) for the given channel. */
static unsigned int adc_read_once(unsigned char ch)
{
    ADC_CONTR = ADC_POWER | ADC_SPEEDLL | ADC_START | ADC_CH(ch);
    while ((ADC_CONTR & ADC_FLAG) == 0)
        ;                        /* busy-wait for conversion done */
    ADC_CONTR &= ~ADC_FLAG;      /* clear flag                    */
    return ((unsigned int)ADC_RES << 2) | (ADC_RESL & 0x03);
}

/*
 * Stable ADC read:
 * - discard first conversion after channel select (settling)
 * - oversample 8 readings and average by right-shifting (fast on 8051)
 * - all samples included: zero is a valid ADC value
 */
static unsigned int adc_read_stable(unsigned char ch)
{
    unsigned long sum = 0UL;
    unsigned char i;

    (void)adc_read_once(ch); /* throwaway conversion: flush channel-switch settling */

    for (i = 0; i < 8; i++)
    {
        sum += (unsigned long)adc_read_once(ch);
    }

    return (unsigned int)(sum >> 3); /* divide by 8 */
}

static unsigned int read_motor_current_raw(void)
{
    /* Sync to the start of the PWM on-period so all samples are taken at the
     * same phase of the switching cycle, eliminating PWM ripple bias. */
    if (pwm_duty > 0U)
    {
        while (pwm_tick != 0U)
            ;
    }
    return adc_read_stable(0); /* channel 0 = P1.0, current sense */
}

static unsigned int read_supply_voltage_raw(void)
{
    return adc_read_stable(1); /* channel 1 = P1.1, supply voltage divider */
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

/* Print a 16-bit unsigned integer (0..65535) over UART. */
static void uart_send_uint16(unsigned int value)
{
    unsigned char digits[5];
    unsigned char count = 0;
    if (value == 0)
    {
        uart_send_char('0');
        return;
    }
    while (value != 0)
    {
        digits[count++] = (unsigned char)('0' + (unsigned char)(value % 10U));
        value /= 10U;
    }
    while (count != 0)
    {
        uart_send_char(digits[--count]);
    }
}

/* Print a 32-bit unsigned integer (0..4294967295) over UART. */
static void uart_send_uint32(unsigned long value)
{
    unsigned char digits[10];
    unsigned char count = 0;
    if (value == 0UL)
    {
        uart_send_char('0');
        return;
    }
    while (value != 0UL)
    {
        digits[count++] = (unsigned char)('0' + (unsigned char)(value % 10UL));
        value /= 10UL;
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
    unsigned int i_raw, v_raw;
    unsigned int i_raw_last_nonzero = 0;
    unsigned long i_scaled, v_scaled;

    uart_init();
    adc_init();
    motor_stop();
    PWM_OUT = 0;

    uart_send_str("Ready. u=up  d=down  s=stop  0-100=PWM%  ADC@500ms: I V\r\n");

    while (1)
    {
        process_uart_commands();

        if (adc_tick >= ADC_REPORT_TICKS)
        {
            adc_tick = 0;
            i_raw = read_motor_current_raw();
            v_raw = read_supply_voltage_raw();

            if ((OUTPUT1 != 0) && ((OUTPUT2 != 0) || (OUTPUT3 != 0)))
            {
                if ((i_raw == 0U) && (i_raw_last_nonzero != 0U))
                {
                    i_raw = i_raw_last_nonzero;
                }
            }
            if (i_raw != 0U)
            {
                i_raw_last_nonzero = i_raw;
            }

            i_scaled = (unsigned long)i_raw * 96UL;
            v_scaled = (unsigned long)v_raw * 64UL;
            uart_send_str("I=");
            uart_send_uint32(i_scaled);
            uart_send_str(" V=");
            uart_send_uint32(v_scaled);
            uart_send_str("\r\n");
        }
    }
}
