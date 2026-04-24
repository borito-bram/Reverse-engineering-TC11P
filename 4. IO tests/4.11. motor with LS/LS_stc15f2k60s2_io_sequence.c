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

sbit OUTPUT1 = P3^5;  /* Voltage regulator enable: P3.5 */
sbit OUTPUT2 = P4^7;  /* Motor up control:         P4.7 */
sbit OUTPUT3 = P1^2;  /* Motor down control:       P1.2 */

sbit LS_UP   = P1^4;  /* Upper limit switch: P1.4 (quasi-bidir, internal pull-up)
                          NC wiring: LOW=closed/normal, HIGH=pressed/end-stop    */
sbit LS_DOWN = P1^5;  /* Lower limit switch: P1.5 (quasi-bidir, internal pull-up)
                          NC wiring: LOW=closed/normal, HIGH=pressed/end-stop    */

/*
 * STC15F2K48S2 (LQFP-44)
 *
 * Motor control with hardware limit switches.
 * The motor is free to move in either direction unless the corresponding
 * limit switch is pressed, in which case movement in that direction is
 * blocked and the motor is stopped immediately.
 *
 * Wiring assumption: limit switches are normally-closed (NC), wired between
 * pin and GND. The internal quasi-bidirectional pull-up holds the pin HIGH
 * when the switch opens at end-stop; otherwise the closed switch keeps LOW.
 *
 * Pin assignments:
 *   P1.2  -> OUTPUT3  (motor DOWN, push-pull output)
 *   P1.4  -> LS_UP    (upper limit switch, quasi-bidir input)
 *   P1.5  -> LS_DOWN  (lower limit switch, quasi-bidir input)
 *   P3.5  -> OUTPUT1  (voltage regulator enable, push-pull output)
 *   P4.7  -> OUTPUT2  (motor UP, push-pull output)
 *
 * Serial protocol on UART1 (P3.0 RXD / P3.1 TXD), 115200-8-N-1:
 *   u / U  -> motor up   (blocked when LS_UP  is pressed)
 *   d / D  -> motor down (blocked when LS_DOWN is pressed)
 *   s / S  -> motor stop + print status
 *
 * Baud rate: 115200 @ 11.0592 MHz, Timer1 1T mode
 */
#define FOSC        11059200L
#define BAUD        115200L
#define T1_RELOAD   (65536 - FOSC / 4 / BAUD)

#define UART_RX_BUF_LEN  8

/* Motor direction state used by the main loop limit-switch guard */
#define DIR_STOP 0
#define DIR_UP   1
#define DIR_DOWN 2

/* Pin reads are inverted due to hardware wiring; always use these macros */
#define LS_UP_TRIG    (LS_UP)
#define LS_DOWN_TRIG  (LS_DOWN)

static unsigned char motor_dir = DIR_STOP;
static bit last_ls_up = 0;
static bit last_ls_down = 0;

/* ---------- UART ---------- */

static void uart_init(void)
{
    AUXR  = 0xC0;   /* T0x12=1, T1x12=1: both timers in 1T (Fosc) mode */

    /* P1.2  -> push-pull output  (OUTPUT3 / motor-down)
     * P1.4  -> quasi-bidir input (LS_UP,   internal weak pull-up)
     * P1.5  -> quasi-bidir input (LS_DOWN, internal weak pull-up) */
    P1M1 &= (unsigned char)~0x34;   /* clear M1 bits for 2, 4, 5 */
    P1M0 &= (unsigned char)~0x34;   /* clear M0 bits for 2, 4, 5 */
    P1M0 |= 0x04;                   /* P1.2 push-pull (M1=0, M0=1) */
    /* P1.4 and P1.5 remain 00 = quasi-bidir (internal pull-up active) */

    /* P3.5 push-pull output (OUTPUT1 / regulator enable). */
    P3M1 &= (unsigned char)~0x20;
    P3M0 |= 0x20;

    /* P4.7 push-pull output (OUTPUT2 / motor-up). */
    P4M1 &= (unsigned char)~0x80;
    P4M0 |= 0x80;

    SCON  = 0x50;   /* UART1 mode 1, 8-bit, REN=1 */
    TMOD  = 0x00;   /* Timer0/1 mode 0 (16-bit auto-reload) */
    TH1   = (unsigned char)(T1_RELOAD >> 8);
    TL1   = (unsigned char)(T1_RELOAD);
    TR1   = 1;

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
    OUTPUT1   = 0;
    OUTPUT2   = 0;
    OUTPUT3   = 0;
    motor_dir = DIR_STOP;
}

/* Returns 1 if the move was started, 0 if blocked by limit switch. */
static bit motor_up(void)
{
    if (LS_UP_TRIG)   /* triggered = upper end-stop (NC switch, inverted read) */
    {
        motor_stop();
        uart_send_str("LIMIT UP\r\n");
        return 0;
    }
    OUTPUT3   = 0;
    OUTPUT1   = 1;
    OUTPUT2   = 1;
    motor_dir = DIR_UP;
    return 1;
}

static bit motor_down(void)
{
    if (LS_DOWN_TRIG) /* triggered = lower end-stop (NC switch, inverted read) */
    {
        motor_stop();
        uart_send_str("LIMIT DOWN\r\n");
        return 0;
    }
    OUTPUT2   = 0;
    OUTPUT1   = 1;
    OUTPUT3   = 1;
    motor_dir = DIR_DOWN;
    return 1;
}

/* ---------- Status report ---------- */

static void report_status(void)
{
    uart_send_str("LS_UP=");
    uart_send_char(LS_UP_TRIG   ? '1' : '0');
    uart_send_str(" LS_DOWN=");
    uart_send_char(LS_DOWN_TRIG ? '1' : '0');
    uart_send_str(" MOTOR=");
    if (motor_dir == DIR_UP)
        uart_send_str("UP");
    else if (motor_dir == DIR_DOWN)
        uart_send_str("DOWN");
    else
        uart_send_str("STOP");
    uart_send_str("\r\n");
}

static void report_limit_inputs_if_changed(void)
{
    bit cur_up = LS_UP_TRIG;
    bit cur_down = LS_DOWN_TRIG;

    if ((cur_up != last_ls_up) || (cur_down != last_ls_down))
    {
        last_ls_up = cur_up;
        last_ls_down = cur_down;
        report_status();
    }
}

/* ---------- Command processing ---------- */

static void process_uart_commands(void)
{
    unsigned char c;

    while (uart_try_read_char(&c))
    {
        if ((c == 'u') || (c == 'U'))
        {
            if (!motor_up())
            {
                /* motor_up() already printed "LIMIT UP" and stopped */
            }
            else
            {
                uart_send_str("MOTOR UP\r\n");
            }
            report_status();
        }
        else if ((c == 'd') || (c == 'D'))
        {
            if (!motor_down())
            {
                /* motor_down() already printed "LIMIT DOWN" and stopped */
            }
            else
            {
                uart_send_str("MOTOR DOWN\r\n");
            }
            report_status();
        }
        else if ((c == 's') || (c == 'S'))
        {
            motor_stop();
            uart_send_str("MOTOR STOP\r\n");
            report_status();
        }
    }
}

/* ---------- Main ---------- */

void main(void)
{
    uart_init();

    /* Drive limit-switch pins high to activate internal pull-ups. */
    LS_UP   = 1;
    LS_DOWN = 1;

    motor_stop();
    last_ls_up = LS_UP_TRIG;
    last_ls_down = LS_DOWN_TRIG;

    uart_send_str("READY u=up d=down s=stop\r\n");
    report_status();

    while (1)
    {
        process_uart_commands();

        /* Continuous limit-switch guard: if a switch trips while the motor
         * is already running in that direction, stop immediately. */
        if ((motor_dir == DIR_UP)   && LS_UP_TRIG)
        {
            motor_stop();
            uart_send_str("LIMIT UP - STOPPED\r\n");
            report_status();
        }
        else if ((motor_dir == DIR_DOWN) && LS_DOWN_TRIG)
        {
            motor_stop();
            uart_send_str("LIMIT DOWN - STOPPED\r\n");
            report_status();
        }

        report_limit_inputs_if_changed();
    }
}
