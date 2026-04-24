#include <REG51.H>

/* SFR declarations for STC15F2K48S2 (not in standard REG51.H) */
sfr AUXR    = 0x8E;
sfr P4      = 0xC0;
sfr P5      = 0xC8;

sfr P1M1    = 0x91;   /* PxM1.n, PxM0.n = 00 -> quasi-bidirectional  */
sfr P1M0    = 0x92;   /*                 = 01 -> push-pull output     */
sfr P2M1    = 0x95;   /*                 = 10 -> high-impedance input */
sfr P2M0    = 0x96;   /*                 = 11 -> open-drain           */
sfr P3M1    = 0xB1;
sfr P3M0    = 0xB2;
sfr P4M1    = 0xB3;
sfr P4M0    = 0xB4;
sfr P5M1    = 0xC9;
sfr P5M0    = 0xCA;

/*
 * Hardware PWM: PCA / CCP peripheral (STC15F2K48S2).
 *
 * Clock source: Fsys/2 = 11.0592 MHz / 2 = 5.5296 MHz
 * PWM period:  256 counts -> 256 / 5529600 = 46.3 us -> ~21.6 kHz
 * Resolution:  8-bit (256 steps, ~0.4% per step)
 *
 * CCP2 is routed to P2.7 (our motor speed pin) by setting P_SW1[5:4] = 10.
 *
 * CCAP2H = 0x00 -> 100% duty (full speed)
 * CCAP2H = 0xFF -> ~0%  duty (motor off)
 * Formula: CCAP2H = (100 - duty_pct) * 255 / 100
 */
sfr CCON     = 0xD8;   /* PCA control register (bit-addressable) */
sfr CMOD     = 0xD9;   /* PCA mode: CPS[2:0] at bits [3:1]       */
sfr CL       = 0xE9;   /* PCA counter low                        */
sfr CH       = 0xF9;   /* PCA counter high                       */
sfr CCAPM2   = 0xDC;   /* PCA channel 2 mode                     */
sfr CCAP2L   = 0xEC;   /* PCA ch2 compare low  (latch register)  */
sfr CCAP2H   = 0xFC;   /* PCA ch2 compare high (active register) */
sfr PCA_PWM2 = 0xF4;   /* PCA PWM2 ext. bits; 0x00 = 8-bit mode  */
sfr P_SW1    = 0xA2;   /* Peripheral pin switch                  */

sbit CR = 0xDE;        /* PCA run bit: CCON.6 (bit address 0xD8+6=0xDE) */

sbit OUTPUT1 = P3^5;   /* Voltage regulator enable: P3.5 */
sbit OUTPUT2 = P4^7;   /* Motor up control:         P4.7 */
sbit OUTPUT3 = P1^2;   /* Motor down control:       P1.2 */
sbit PWM_OUT = P2^7;   /* PWM speed output:         P2.7 (driven by PCA CCP2) */

sbit BTN_UP   = P5^5;  /* Push button UP:    P5.5 */
sbit BTN_DOWN = P4^0;  /* Push button DOWN:  P4.0 */
sbit LS_UP    = P1^4;  /* Upper limit switch: P1.4 */
sbit LS_DOWN  = P1^5;  /* Lower limit switch: P1.5 */

/*
 * STC15F2K48S2 (LQFP-44)
 *
 * Motor control with hardware-PWM speed control, limit switches,
 * push buttons, and smooth acceleration / deceleration.
 *
 * Pin assignments:
 *   P1.2  -> OUTPUT3  (motor DOWN, push-pull)
 *   P1.4  -> LS_UP    (upper limit switch, quasi-bidir)
 *   P1.5  -> LS_DOWN  (lower limit switch, quasi-bidir)
 *   P2.7  -> PWM_OUT  (PCA CCP2 hardware PWM ~21.6 kHz)
 *   P3.5  -> OUTPUT1  (voltage regulator enable, push-pull)
 *   P4.7  -> OUTPUT2  (motor UP, push-pull)
 *   P5.5  -> BTN_UP   (push button UP, quasi-bidir)
 *   P4.0  -> BTN_DOWN (push button DOWN, quasi-bidir)
 *
 * Smooth motion:
 *   Normal start : ramp duty 0->100% over ~500 ms (100 steps x 5 ms)
 *   Normal stop  : ramp duty 100->0% then cut direction outputs
 *   Limit switch : immediate hard stop, no ramp
 *
 * Serial protocol UART1 (P3.0/P3.1), 115200-8-N-1:
 *   u / U  -> motor up    (smooth start)
 *   d / D  -> motor down  (smooth start)
 *   s / S  -> motor stop  (smooth ramp-down)
 *   x / X  -> motor stop  (hard / immediate)
 */

#define FOSC        11059200L
#define BAUD        115200L
#define T1_RELOAD   (65536 - FOSC / 4 / BAUD)

/* Timer0: 1 ms tick for ramp timing (1T mode @ 11.0592 MHz) */
#define T0_RELOAD   (65536U - 11059U)

/* Ramp: 5 ms per duty step, 100 steps -> ~500 ms full ramp */
#define RAMP_TARGET    100
#define RAMP_STEP_MS   5

/* Input polarity */
#define LS_UP_TRIG       (LS_UP)
#define LS_DOWN_TRIG     (LS_DOWN)
#define BTN_UP_ACTIVE    (!BTN_UP)
#define BTN_DOWN_ACTIVE  (!BTN_DOWN)

/* Motor direction */
#define DIR_STOP  0
#define DIR_UP    1
#define DIR_DOWN  2

/* Ramp state */
#define RAMP_NONE  0
#define RAMP_ACCEL 1
#define RAMP_DECEL 2

/* ---------- State ---------- */
static volatile unsigned int ms_ticks = 0;

static unsigned char motor_dir  = DIR_STOP;
static unsigned char ramp_state = RAMP_NONE;
static unsigned char ramp_duty  = 0;        /* 0-100 logical duty % */
static unsigned int  last_ramp_tick = 0;

static bit last_ls_up    = 0;
static bit last_ls_down  = 0;
static bit last_btn_up   = 0;
static bit last_btn_down = 0;
static bit button_drive_active = 0;

/* ---------- Timer0 ISR: 1 ms ramp tick ---------- */
void timer0_isr(void) interrupt 1
{
    ms_ticks++;
}

/* ---------- Hardware PWM helpers ---------- */

/*
 * Apply logical duty (0-100%) to PCA CCP2.
 * CCAP2H = (100-duty)*255/100:  0%->0xFF, 100%->0x00
 */
static void pwm_set_duty(unsigned char duty)
{
    unsigned char ccap;
    if (duty >= RAMP_TARGET)
    {
        ccap = 0x00;
    }
    else
    {
        ccap = (unsigned char)((unsigned int)(100U - duty) * 255U / 100U);
    }
    CCAP2L = ccap;
    CCAP2H = ccap;
}

/* Enable PCA PWM output on P2.7. */
static void pwm_enable(void)
{
    CCAP2H  = 0xFF;
    CCAP2L  = 0xFF;
    PCA_PWM2 = 0x00;          /* 8-bit mode */
    CCAPM2  = 0x42;           /* ECOM2=1, PWM2=1 */
}

/* Disable PCA PWM and drive P2.7 low. */
static void pwm_disable(void)
{
    CCAPM2  = 0x00;           /* Disable channel */
    PWM_OUT = 0;
}

/* ---------- UART ---------- */

static void uart_init(void)
{
    AUXR = 0xC0;   /* T0x12=1, T1x12=1 -> 1T mode for both timers */

    /* P1.2 push-pull (OUTPUT3 / motor down) */
    P1M1 &= (unsigned char)~0x04;
    P1M0 |= 0x04;

    /* P1.4, P1.5 quasi-bidir inputs (LS_UP, LS_DOWN) */
    P1M1 &= (unsigned char)~0x30;
    P1M0 &= (unsigned char)~0x30;

    /* P2.7 push-pull (PWM_OUT - driven by PCA) */
    P2M1 &= (unsigned char)~0x80;
    P2M0 |= 0x80;

    /* P3.5 push-pull (OUTPUT1 / regulator enable) */
    P3M1 &= (unsigned char)~0x20;
    P3M0 |= 0x20;

    /* P4.7 push-pull (OUTPUT2 / motor up) */
    P4M1 &= (unsigned char)~0x80;
    P4M0 |= 0x80;

    /* P5.5 quasi-bidir (BTN_UP) */
    P5M1 &= (unsigned char)~0x20;
    P5M0 &= (unsigned char)~0x20;

    /* P4.0 quasi-bidir (BTN_DOWN) */
    P4M1 &= (unsigned char)~0x01;
    P4M0 &= (unsigned char)~0x01;

    /* Timer0: 1 ms tick for ramp */
    TMOD = 0x00;   /* Both timers mode 0 (16-bit auto-reload) */
    TH0  = (unsigned char)(T0_RELOAD >> 8);
    TL0  = (unsigned char)(T0_RELOAD);
    ET0  = 1;
    TR0  = 1;

    /* Timer1: baud rate generator */
    SCON = 0x50;
    TH1  = (unsigned char)(T1_RELOAD >> 8);
    TL1  = (unsigned char)(T1_RELOAD);
    TR1  = 1;

    EA = 1;
}

static void pca_init(void)
{
    /* Route CCP pins to P2 group: CCP2 -> P2.7
     * P_SW1[5:4] = 10 */
    P_SW1 = (unsigned char)((P_SW1 & (unsigned char)~0x30) | 0x20);

    /* PCA clock = Fsys/2 (~5.53 MHz) -> PWM ~21.6 kHz
     * CMOD: CIDL=0, CPS[2:0]=001 (Fsys/2), ECF=0 -> 0x02 */
    CMOD = 0x02;

    /* Clear PCA counter */
    CL = 0;
    CH = 0;

    /* Start PCA counter running */
    CCON = 0x00;
    CR   = 1;

    /* PWM channel disabled at startup; enabled when motor starts */
    pwm_disable();
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

static void uart_send_uint(unsigned char v)
{
    unsigned char d[3];
    unsigned char n = 0;
    if (v == 0) { uart_send_char('0'); return; }
    while (v) { d[n++] = (unsigned char)('0' + v % 10); v /= 10; }
    while (n) uart_send_char(d[--n]);
}

static bit uart_try_read_char(unsigned char *c)
{
    if (!RI) return 0;
    RI = 0;
    *c = SBUF;
    return 1;
}

/* ---------- Motor control ---------- */

/* Hard stop: cut PWM and outputs immediately. Use for limit-switch events. */
static void motor_hard_stop(void)
{
    pwm_disable();
    OUTPUT1   = 0;
    OUTPUT2   = 0;
    OUTPUT3   = 0;
    motor_dir  = DIR_STOP;
    ramp_state = RAMP_NONE;
    ramp_duty  = 0;
    button_drive_active = 0;
}

/* Smooth stop: begin PWM ramp-down; outputs cut when duty reaches 0. */
static void motor_smooth_stop(void)
{
    if (motor_dir == DIR_STOP && ramp_state == RAMP_NONE) return;

    if (ramp_state != RAMP_DECEL)
    {
        ramp_state = RAMP_DECEL;
        EA = 0; last_ramp_tick = ms_ticks; EA = 1;
    }
    button_drive_active = 0;
}

/* Start motor up with smooth acceleration. */
static bit motor_start_up(void)
{
    if (LS_UP_TRIG)
    {
        motor_hard_stop();
        uart_send_str("LIMIT UP\r\n");
        return 0;
    }
    ramp_duty = 0;
    pwm_enable();
    pwm_set_duty(0);
    OUTPUT3   = 0;
    OUTPUT1   = 1;
    OUTPUT2   = 1;
    motor_dir  = DIR_UP;
    ramp_state = RAMP_ACCEL;
    EA = 0; last_ramp_tick = ms_ticks; EA = 1;
    return 1;
}

/* Start motor down with smooth acceleration. */
static bit motor_start_down(void)
{
    if (LS_DOWN_TRIG)
    {
        motor_hard_stop();
        uart_send_str("LIMIT DOWN\r\n");
        return 0;
    }
    ramp_duty = 0;
    pwm_enable();
    pwm_set_duty(0);
    OUTPUT2   = 0;
    OUTPUT1   = 1;
    OUTPUT3   = 1;
    motor_dir  = DIR_DOWN;
    ramp_state = RAMP_ACCEL;
    EA = 0; last_ramp_tick = ms_ticks; EA = 1;
    return 1;
}

/* ---------- Ramp engine (call every main loop iteration) ---------- */
static void update_ramp(void)
{
    unsigned int now;

    if (ramp_state == RAMP_NONE) return;

    EA = 0; now = ms_ticks; EA = 1;
    if ((unsigned int)(now - last_ramp_tick) < (unsigned int)RAMP_STEP_MS) return;
    last_ramp_tick = now;

    if (ramp_state == RAMP_ACCEL)
    {
        if (ramp_duty < RAMP_TARGET)
        {
            ramp_duty++;
            pwm_set_duty(ramp_duty);
        }
        if (ramp_duty >= RAMP_TARGET)
            ramp_state = RAMP_NONE;
    }
    else if (ramp_state == RAMP_DECEL)
    {
        if (ramp_duty > 0)
        {
            ramp_duty--;
            pwm_set_duty(ramp_duty);
        }
        if (ramp_duty == 0)
        {
            pwm_disable();
            OUTPUT1   = 0;
            OUTPUT2   = 0;
            OUTPUT3   = 0;
            motor_dir  = DIR_STOP;
            ramp_state = RAMP_NONE;
            uart_send_str("STOPPED\r\n");
        }
    }
}

/* ---------- Status ---------- */

static void report_status(void)
{
    uart_send_str("LS_UP=");
    uart_send_char(LS_UP_TRIG   ? '1' : '0');
    uart_send_str(" LS_DOWN=");
    uart_send_char(LS_DOWN_TRIG ? '1' : '0');
    uart_send_str(" BTN_UP=");
    uart_send_char(BTN_UP_ACTIVE   ? '1' : '0');
    uart_send_str(" BTN_DOWN=");
    uart_send_char(BTN_DOWN_ACTIVE ? '1' : '0');
    uart_send_str(" MOTOR=");
    if (motor_dir == DIR_UP)        uart_send_str("UP");
    else if (motor_dir == DIR_DOWN) uart_send_str("DOWN");
    else                            uart_send_str("STOP");
    uart_send_str(" RAMP=");
    if (ramp_state == RAMP_ACCEL)      uart_send_str("ACCEL");
    else if (ramp_state == RAMP_DECEL) uart_send_str("DECEL");
    else                               uart_send_str("NONE");
    uart_send_str(" DUTY=");
    uart_send_uint(ramp_duty);
    uart_send_str("%\r\n");
}

static void report_inputs_if_changed(void)
{
    bit cur_ls_up    = LS_UP_TRIG;
    bit cur_ls_down  = LS_DOWN_TRIG;
    bit cur_btn_up   = BTN_UP_ACTIVE;
    bit cur_btn_down = BTN_DOWN_ACTIVE;

    if ((cur_ls_up != last_ls_up) || (cur_ls_down != last_ls_down) ||
        (cur_btn_up != last_btn_up) || (cur_btn_down != last_btn_down))
    {
        last_ls_up   = cur_ls_up;
        last_ls_down = cur_ls_down;
        last_btn_up  = cur_btn_up;
        last_btn_down = cur_btn_down;
        report_status();
    }
}

/* ---------- Button input ---------- */

static void process_button_commands(void)
{
    unsigned char desired_dir = DIR_STOP;
    bit up_active   = BTN_UP_ACTIVE;
    bit down_active = BTN_DOWN_ACTIVE;

    /* BTN_UP physically drives DOWN, BTN_DOWN drives UP (wiring inversion). */
    if (up_active && !down_active)
    {
        desired_dir = DIR_DOWN;
    }
    else if (down_active && !up_active)
    {
        desired_dir = DIR_UP;
    }
    else if (up_active && down_active)
    {
        if (motor_dir != DIR_STOP)
        {
            motor_smooth_stop();
            uart_send_str("BTN BOTH->STOP\r\n");
            report_status();
        }
    }
    else
    {
        if (button_drive_active && motor_dir != DIR_STOP &&
            ramp_state != RAMP_DECEL)
        {
            motor_smooth_stop();
            uart_send_str("BTN RELEASE->STOP\r\n");
            report_status();
        }
        return;
    }

    if (desired_dir == DIR_DOWN)
    {
        if ((motor_dir == DIR_STOP) && (ramp_state == RAMP_NONE))
        {
            if (motor_start_down())
            {
                button_drive_active = 1;
                uart_send_str("BTN->DOWN\r\n");
                report_status();
            }
        }
        else if ((motor_dir == DIR_DOWN) && (ramp_state != RAMP_DECEL))
        {
            button_drive_active = 1;
        }
        else
        {
            if (ramp_state != RAMP_DECEL)
                motor_smooth_stop();
        }
    }
    else if (desired_dir == DIR_UP)
    {
        if ((motor_dir == DIR_STOP) && (ramp_state == RAMP_NONE))
        {
            if (motor_start_up())
            {
                button_drive_active = 1;
                uart_send_str("BTN->UP\r\n");
                report_status();
            }
        }
        else if ((motor_dir == DIR_UP) && (ramp_state != RAMP_DECEL))
        {
            button_drive_active = 1;
        }
        else
        {
            if (ramp_state != RAMP_DECEL)
                motor_smooth_stop();
        }
    }
}

/* ---------- UART command processing ---------- */

static void process_uart_commands(void)
{
    unsigned char c;

    while (uart_try_read_char(&c))
    {
        if ((c == 'u') || (c == 'U'))
        {
            button_drive_active = 0;
            if (motor_dir != DIR_STOP) { motor_smooth_stop(); report_status(); }
            else if (motor_start_up()) { uart_send_str("MOTOR UP\r\n"); report_status(); }
        }
        else if ((c == 'd') || (c == 'D'))
        {
            button_drive_active = 0;
            if (motor_dir != DIR_STOP) { motor_smooth_stop(); report_status(); }
            else if (motor_start_down()) { uart_send_str("MOTOR DOWN\r\n"); report_status(); }
        }
        else if ((c == 's') || (c == 'S'))
        {
            button_drive_active = 0;
            motor_smooth_stop();
            uart_send_str("MOTOR SMOOTH STOP\r\n");
            report_status();
        }
        else if ((c == 'x') || (c == 'X'))
        {
            motor_hard_stop();
            uart_send_str("MOTOR HARD STOP\r\n");
            report_status();
        }
    }
}

/* ---------- Main ---------- */

void main(void)
{
    uart_init();
    pca_init();

    /* Release input pins high for quasi-bidir sampling */
    LS_UP    = 1;
    LS_DOWN  = 1;
    BTN_UP   = 1;
    BTN_DOWN = 1;

    motor_hard_stop();

    last_ls_up    = LS_UP_TRIG;
    last_ls_down  = LS_DOWN_TRIG;
    last_btn_up   = BTN_UP_ACTIVE;
    last_btn_down = BTN_DOWN_ACTIVE;

    uart_send_str("READY u=up d=down s=smooth-stop x=hard-stop\r\n");
    uart_send_str("HW PWM CCP2 on P2.7, ~21.6 kHz, 8-bit\r\n");
    report_status();

    while (1)
    {
        process_uart_commands();
        process_button_commands();
        update_ramp();

        /* Limit switch guard: hard stop regardless of ramp state */
        if ((motor_dir == DIR_UP) && LS_UP_TRIG)
        {
            motor_hard_stop();
            uart_send_str("LIMIT UP - HARD STOP\r\n");
            report_status();
        }
        else if ((motor_dir == DIR_DOWN) && LS_DOWN_TRIG)
        {
            motor_hard_stop();
            uart_send_str("LIMIT DOWN - HARD STOP\r\n");
            report_status();
        }

        report_inputs_if_changed();
    }
}
