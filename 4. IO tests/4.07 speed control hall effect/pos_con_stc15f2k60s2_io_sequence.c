#include <REG51.H>

/* SFR declarations for STC15 (not in standard REG51.H) */
sfr AUXR = 0x8E;
sfr P4   = 0xC0;

sfr P1M1 = 0x91;   /* PxM1.n, PxM0.n = 00 -> quasi-bidirectional  */
sfr P1M0 = 0x92;   /*                 = 01 -> push-pull output     */
sfr P3M1 = 0xB1;
sfr P3M0 = 0xB2;
sfr P4M1 = 0xB3;
sfr P4M0 = 0xB4;
                   /*                 = 10 -> high-impedance input */
                   /*                 = 11 -> open-drain           */

sbit OUTPUT1 = P3^5;  /* Voltage regulator enable: P3.5, LQFP-44 pin 6 */
sbit OUTPUT2 = P4^7;  /* Motor up control:        P4.7, LQFP-44 pin 7 */
sbit OUTPUT3 = P1^2;  /* Motor down control:      P1.2 */

sbit HALL1   = P1^4;  /* Hall sensor A: P1.4 */
sbit HALL2   = P1^5;  /* Hall sensor B: P1.5 */

/*
 * STC15F2K48S2 (LQFP-44)
 * Physical package pin mapping used here:
 *
 *    - P3.5  -> Regulator enable output
 *    - P4.7  -> Motor UP output
 *    - P1.2  -> Motor DOWN output
 *    - P1.4  -> Hall sensor A input
 *    - P1.5  -> Hall sensor B input
 *
 * Serial protocol on UART1 (P3.0 RXD / P3.1 TXD), 115200-8-N-1:
 *    - Send a signed decimal integer followed by CR/LF to set target position.
 *    - Send 's' to print current status.
 *    - Send 'z' to zero current position and target.
 *
 * Hall states are decoded as a 2-bit Gray-code sequence:
 *    00 -> 01 -> 11 -> 10 -> 00  = positive position counts
 * Reverse sequence                = negative position counts
 *
 * The motor runs until current_position is within POSITION_TOLERANCE
 * counts of target_position.
 *
 * Baud rate: 115200 @ 11.0592 MHz, Timer1 1T mode (AUXR.T1x12=1)
 * Formula (from STC reference): 65536 - FOSC/4/baud
 *   = 65536 - 11059200/4/115200 = 65512 = 0xFFE8
 */
#define FOSC        11059200L
#define BAUD        115200L
#define T1_RELOAD   (65536 - FOSC / 4 / BAUD)

#define POSITION_TOLERANCE  5L
#define UART_RX_BUF_LEN  16

static long current_position = 0;
static long target_position = 0;
static unsigned char last_hall_rank = 0;
static bit last_hall_valid = 0;

/*
 * UART1 init: 115200 baud, mode 1 (8-bit), no parity.
 * Timer1 in STC15 mode 0 (16-bit auto-reload), 1T.
 * UART1 TX = P3.1, RX = P3.0 (same as ISP programming port).
 *
 * Pin modes (PxM1:PxM0):
 *   P1.2  -> 01 push-pull        (OUTPUT3)
 *   P1.4  -> 00 quasi-bidir      (HALL1, internal weak pull-up)
 *   P1.5  -> 10 high-impedance   (HALL2, pure input, needs external pull-up/down)
 *   P3.5  -> 01 push-pull        (OUTPUT1)
 *   P4.7  -> 01 push-pull        (OUTPUT2)
 */
static void uart_init(void)
{
    AUXR  = 0xC0;   /* T0x12=1, T1x12=1: both timers in 1T (Fosc) mode */

    /* P1.2 push-pull output, P1.4 quasi-bidir input, P1.5 high-Z input. */
    P1M1 &= (unsigned char)~0x34;
    P1M0 &= (unsigned char)~0x34;
    P1M0 |= 0x04;
    P1M1 |= 0x20;

    /* P3.5 push-pull output. */
    P3M1 &= (unsigned char)~0x20;
    P3M0 |= 0x20;

    /* P4.7 push-pull output. */
    P4M1 &= (unsigned char)~0x80;
    P4M0 |= 0x80;

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

static void uart_send_long(long value)
{
    char digits[11];
    unsigned char count = 0;
    unsigned long magnitude;

    if (value < 0)
    {
        uart_send_char('-');
        magnitude = (unsigned long)(-value);
    }
    else
    {
        magnitude = (unsigned long)value;
    }

    if (magnitude == 0)
    {
        uart_send_char('0');
        return;
    }

    while (magnitude != 0)
    {
        digits[count] = (char)('0' + (magnitude % 10UL));
        count++;
        magnitude /= 10UL;
    }

    while (count != 0)
    {
        count--;
        uart_send_char((unsigned char)digits[count]);
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

static unsigned char hall_read_raw(void)
{
    unsigned char raw = 0;

    if (HALL1)
    {
        raw |= 0x01;
    }
    if (HALL2)
    {
        raw |= 0x02;
    }

    return raw;
}

static bit hall_rank_from_raw(unsigned char raw, unsigned char *rank)
{
    switch (raw)
    {
    case 0x00:
        *rank = 0;
        return 1;
    case 0x01:
        *rank = 1;
        return 1;
    case 0x03:
        *rank = 2;
        return 1;
    case 0x02:
        *rank = 3;
        return 1;
    default:
        return 0;
    }
}

static char hall_step_from_ranks(unsigned char previous_rank, unsigned char next_rank)
{
    if ((unsigned char)((previous_rank + 1) & 0x03) == next_rank)
    {
        return 1;
    }
    if ((unsigned char)((previous_rank + 3) & 0x03) == next_rank)
    {
        return -1;
    }
    return 0;
}

static void hall_update_position(void)
{
    unsigned char next_rank;
    char step;

    if (!hall_rank_from_raw(hall_read_raw(), &next_rank))
    {
        return;
    }

    if (!last_hall_valid)
    {
        last_hall_rank = next_rank;
        last_hall_valid = 1;
        return;
    }

    if (next_rank == last_hall_rank)
    {
        return;
    }

    step = hall_step_from_ranks(last_hall_rank, next_rank);
    last_hall_rank = next_rank;

    if (step > 0)
    {
        current_position++;
    }
    else if (step < 0)
    {
        current_position--;
    }
}

static void report_status(void)
{
    uart_send_str("POS ");
    uart_send_long(current_position);
    uart_send_str(" TARGET ");
    uart_send_long(target_position);
    uart_send_str(" HALL ");
    uart_send_char(HALL1 ? '1' : '0');
    uart_send_char(HALL2 ? '1' : '0');
    uart_send_str("\r\n");
}

static bit parse_signed_long(const char *buffer, unsigned char length, long *value)
{
    unsigned char index = 0;
    bit negative = 0;
    long result = 0;

    if (length == 0)
    {
        return 0;
    }

    if (buffer[index] == '-')
    {
        negative = 1;
        index++;
    }
    else if (buffer[index] == '+')
    {
        index++;
    }

    if (index >= length)
    {
        return 0;
    }

    while (index < length)
    {
        if ((buffer[index] < '0') || (buffer[index] > '9'))
        {
            return 0;
        }

        result = (result * 10L) + (long)(buffer[index] - '0');
        index++;
    }

    if (negative)
    {
        result = -result;
    }

    *value = result;
    return 1;
}

static void process_uart_commands(void)
{
    static char rx_buffer[UART_RX_BUF_LEN];
    static unsigned char rx_length = 0;
    unsigned char c;
    long parsed_value;

    while (uart_try_read_char(&c))
    {
        if ((c == '\r') || (c == '\n'))
        {
            if (rx_length == 0)
            {
                continue;
            }

            if ((rx_length == 1) && ((rx_buffer[0] == 's') || (rx_buffer[0] == 'S')))
            {
                report_status();
            }
            else if ((rx_length == 1) && ((rx_buffer[0] == 'z') || (rx_buffer[0] == 'Z')))
            {
                current_position = 0;
                target_position = 0;
                uart_send_str("ZEROED\r\n");
                report_status();
            }
            else if (parse_signed_long(rx_buffer, rx_length, &parsed_value))
            {
                target_position = parsed_value;
                uart_send_str("TARGET SET ");
                uart_send_long(target_position);
                uart_send_str("\r\n");
                report_status();
            }
            else
            {
                uart_send_str("ERR\r\n");
            }

            rx_length = 0;
            continue;
        }

        if ((c == ' ') || (c == '\t'))
        {
            continue;
        }

        if (rx_length < (UART_RX_BUF_LEN - 1))
        {
            rx_buffer[rx_length] = (char)c;
            rx_length++;
        }
        else
        {
            rx_length = 0;
            uart_send_str("ERR OVERFLOW\r\n");
        }
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
    unsigned int report_divider = 0;

    uart_init();

    /* Release hall inputs high for input sampling and stop motor on startup. */
    HALL1 = 1;
    HALL2 = 1;
    motor_stop();
    hall_update_position();

    uart_send_str("Position control ready. Send signed target, s, or z.\r\n");
    report_status();

    while (1)
    {
        process_uart_commands();
        hall_update_position();

        if (current_position < (target_position - POSITION_TOLERANCE))
        {
            motor_up();
        }
        else if (current_position > (target_position + POSITION_TOLERANCE))
        {
            motor_down();
        }
        else
        {
            motor_stop();
        }

        report_divider++;
        if (report_divider >= 100)
        {
            report_divider = 0;
            report_status();
        }

        delay_ms(10);
    }
}