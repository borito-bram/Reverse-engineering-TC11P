#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * TC11P motor controller rewrite
 *
 * This file is a readable, high-level rewrite of the decompiled firmware in:
 *   2. Convertion/TC11P-2604170408.bin.c
 *
 * It preserves the controller responsibilities that are visible in the binary:
 * - boot-time state and configuration restore
 * - Hall sensor decoding and position counting
 * - timer-driven fast control updates
 * - PWM and 6-step commutation output control
 * - speed and position control
 * - limit checking, stall detection, overcurrent handling and emergency stop
 * - status reporting and non-volatile state save
 *
 * Original firmware areas used as the basis for this rewrite:
 * - FUN_CODE_2e03  : boot/config restore
 * - FUN_CODE_555d  : timer 0 periodic ISR
 * - FUN_CODE_154b  : fast motor control ISR
 * - FUN_CODE_0f9b  : main scheduler / control loop
 * - FUN_CODE_1fa2  : fault handling / state rerouting
 */

#define TC11P_STATUS_FRAME_SIZE 13u

typedef enum Tc11pControlMode {
    TC11P_CONTROL_DISABLED = 0,
    TC11P_CONTROL_OPEN_LOOP = 1,
    TC11P_CONTROL_SPEED = 2,
    TC11P_CONTROL_POSITION = 3
} Tc11pControlMode;

typedef enum Tc11pDirection {
    TC11P_DIRECTION_REVERSE = -1,
    TC11P_DIRECTION_STOP = 0,
    TC11P_DIRECTION_FORWARD = 1
} Tc11pDirection;

typedef enum Tc11pDriveCommand {
    TC11P_DRIVE_BRAKE = 0,
    TC11P_DRIVE_FORWARD = 1,
    TC11P_DRIVE_REVERSE = 2
} Tc11pDriveCommand;

typedef enum Tc11pFaultCode {
    TC11P_FAULT_NONE = 0,
    TC11P_FAULT_INVALID_HALL,
    TC11P_FAULT_HALL_STUCK,
    TC11P_FAULT_LIMIT,
    TC11P_FAULT_OVERCURRENT,
    TC11P_FAULT_STALL,
    TC11P_FAULT_OVERSPEED,
    TC11P_FAULT_EMERGENCY_STOP,
    TC11P_FAULT_OVERTEMP,
    TC11P_FAULT_UNDERVOLTAGE,
    TC11P_FAULT_NV_STORAGE
} Tc11pFaultCode;

typedef enum Tc11pCommandType {
    TC11P_COMMAND_ENABLE = 0,
    TC11P_COMMAND_DISABLE,
    TC11P_COMMAND_BRAKE,
    TC11P_COMMAND_CLEAR_FAULT,
    TC11P_COMMAND_SET_MODE,
    TC11P_COMMAND_SET_TARGET_POSITION,
    TC11P_COMMAND_SET_TARGET_SPEED,
    TC11P_COMMAND_SAVE_STATE,
    TC11P_COMMAND_LOAD_STATE
} Tc11pCommandType;

typedef struct Tc11pCommand {
    Tc11pCommandType type;
    int32_t value;
} Tc11pCommand;

typedef struct Tc11pHallSample {
    uint8_t raw_code;
    int8_t sector;
    bool valid;
    bool edge_detected;
} Tc11pHallSample;

typedef struct Tc11pPersistentImage {
    uint16_t saved_position;
    uint16_t saved_target_position;
    uint16_t saved_min_speed;
    uint16_t saved_max_speed;
    uint8_t mode_byte;
    uint8_t status_byte;
    uint8_t flags_byte;
    uint8_t saved_hall_code;
    uint8_t saved_fault_code;
} Tc11pPersistentImage;

typedef struct Tc11pConfig {
    int32_t lower_position_limit;
    int32_t upper_position_limit;
    int32_t position_deadband_counts;
    int32_t min_speed_counts_per_tick;
    int32_t max_speed_counts_per_tick;
    int32_t overspeed_limit_counts_per_tick;
    uint16_t pwm_min;
    uint16_t pwm_max;
    uint16_t pwm_startup;
    uint16_t pwm_ramp_step;
    uint16_t align_ticks;
    uint16_t soft_start_ticks;
    uint16_t hall_stuck_ticks;
    uint16_t status_period_ticks;
    uint16_t save_period_ticks;
    uint16_t overcurrent_limit_raw;
    uint16_t overtemp_limit_raw;
    uint16_t undervoltage_limit_raw;
    float speed_kp;
    float speed_ki;
    float position_kp;
    float position_ki;
    bool hall_required;
    bool regenerative_brake_enabled;
    bool persist_position;
} Tc11pConfig;

typedef struct Tc11pHardware {
    uint8_t (*read_hall_code)(void *context);
    bool (*read_upper_limit)(void *context);
    bool (*read_lower_limit)(void *context);
    bool (*read_emergency_stop)(void *context);
    uint16_t (*read_current_raw)(void *context);
    uint16_t (*read_temperature_raw)(void *context);
    uint16_t (*read_supply_raw)(void *context);
    void (*set_pwm_duty)(void *context, uint16_t duty_ticks);
    void (*apply_commutation)(void *context, uint8_t sector, Tc11pDirection direction, bool enable);
    void (*set_brake)(void *context, bool enabled);
    void (*set_gate_enable)(void *context, bool enabled);
    void (*uart_send)(void *context, const uint8_t *data, size_t size);
    bool (*nv_load)(void *context, Tc11pPersistentImage *image);
    bool (*nv_save)(void *context, const Tc11pPersistentImage *image);
    void (*watchdog_kick)(void *context);
} Tc11pHardware;

typedef struct Tc11pController {
    Tc11pHardware hw;
    void *hw_context;
    Tc11pConfig config;
    Tc11pPersistentImage nv_image;

    Tc11pControlMode mode;
    Tc11pDirection direction;
    Tc11pDriveCommand drive_command;
    Tc11pFaultCode fault;

    bool enabled;
    bool closed_loop_active;
    bool hall_locked;
    bool acceleration_complete;
    bool upper_limit_enabled;
    bool lower_limit_enabled;
    bool regenerative_brake_enabled;
    bool status_dirty;
    bool save_requested;

    uint32_t timer_ticks;
    uint32_t fast_ticks;
    uint16_t align_ticks_remaining;
    uint16_t soft_start_ticks_remaining;
    uint16_t hall_stuck_counter;
    uint16_t overcurrent_counter;
    uint16_t overspeed_counter;
    uint16_t stall_counter;
    uint16_t status_ticks_remaining;
    uint16_t save_ticks_remaining;

    Tc11pHallSample hall;
    uint8_t previous_sector;
    uint16_t hall_edge_counter;

    int32_t position_counts;
    int32_t target_position_counts;
    int32_t last_position_counts;
    int32_t speed_counts_per_tick;
    int32_t filtered_speed_counts_per_tick;
    int32_t target_speed_counts_per_tick;
    int32_t position_error_counts;

    float speed_integral;
    float position_integral;

    uint16_t pwm_target;
    uint16_t pwm_output;
} Tc11pController;

static int32_t tc11p_abs32(int32_t value)
{
    return value < 0 ? -value : value;
}

static float tc11p_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static int32_t tc11p_clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint16_t tc11p_clamp_u16(uint16_t value, uint16_t minimum, uint16_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t tc11p_sign32(int32_t value)
{
    if (value > 0) {
        return 1;
    }
    if (value < 0) {
        return -1;
    }
    return 0;
}

static void tc11p_write_u16_be(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value >> 8);
    buffer[1] = (uint8_t)value;
}

static int8_t tc11p_decode_hall_sector(uint8_t raw_code)
{
    switch (raw_code & 0x07u) {
    case 0x01u:
        return 0;
    case 0x05u:
        return 1;
    case 0x04u:
        return 2;
    case 0x06u:
        return 3;
    case 0x02u:
        return 4;
    case 0x03u:
        return 5;
    default:
        return -1;
    }
}

static int8_t tc11p_sector_step(uint8_t previous_sector, uint8_t next_sector)
{
    if ((uint8_t)((previous_sector + 1u) % 6u) == next_sector) {
        return 1;
    }
    if ((uint8_t)((previous_sector + 5u) % 6u) == next_sector) {
        return -1;
    }
    return 0;
}

static void tc11p_set_outputs_disabled(Tc11pController *controller)
{
    controller->pwm_target = 0;
    controller->pwm_output = 0;
    controller->drive_command = TC11P_DRIVE_BRAKE;
    controller->direction = TC11P_DIRECTION_STOP;

    if (controller->hw.set_pwm_duty != NULL) {
        controller->hw.set_pwm_duty(controller->hw_context, 0);
    }
    if (controller->hw.set_brake != NULL) {
        controller->hw.set_brake(controller->hw_context, true);
    }
    if (controller->hw.set_gate_enable != NULL) {
        controller->hw.set_gate_enable(controller->hw_context, false);
    }
}

static void tc11p_reset_integrators(Tc11pController *controller)
{
    controller->speed_integral = 0.0f;
    controller->position_integral = 0.0f;
}

static void tc11p_raise_fault(Tc11pController *controller, Tc11pFaultCode fault)
{
    if (controller->fault == fault) {
        return;
    }

    controller->fault = fault;
    controller->enabled = false;
    controller->closed_loop_active = false;
    controller->hall_locked = false;
    controller->save_requested = true;
    controller->status_dirty = true;
    tc11p_reset_integrators(controller);
    tc11p_set_outputs_disabled(controller);
}

static void tc11p_clear_fault(Tc11pController *controller)
{
    controller->fault = TC11P_FAULT_NONE;
    controller->hall_stuck_counter = 0;
    controller->overcurrent_counter = 0;
    controller->overspeed_counter = 0;
    controller->stall_counter = 0;
    controller->status_dirty = true;
    tc11p_reset_integrators(controller);
}

static void tc11p_config_defaults(Tc11pConfig *config)
{
    memset(config, 0, sizeof(*config));
    config->lower_position_limit = -12000;
    config->upper_position_limit = 12000;
    config->position_deadband_counts = 2;
    config->min_speed_counts_per_tick = 1;
    config->max_speed_counts_per_tick = 12;
    config->overspeed_limit_counts_per_tick = 18;
    config->pwm_min = 10;
    config->pwm_max = 255;
    config->pwm_startup = 36;
    config->pwm_ramp_step = 2;
    config->align_ticks = 40;
    config->soft_start_ticks = 80;
    config->hall_stuck_ticks = 60;
    config->status_period_ticks = 25;
    config->save_period_ticks = 200;
    config->overcurrent_limit_raw = 900;
    config->overtemp_limit_raw = 900;
    config->undervoltage_limit_raw = 300;
    config->speed_kp = 6.0f;
    config->speed_ki = 0.08f;
    config->position_kp = 0.35f;
    config->position_ki = 0.01f;
    config->hall_required = true;
    config->regenerative_brake_enabled = true;
    config->persist_position = true;
}

static void tc11p_capture_nv_image(Tc11pController *controller)
{
    uint8_t status = 0;
    uint8_t flags = 0;

    if (controller->enabled) {
        status |= 0x01u;
    }
    if (controller->closed_loop_active) {
        status |= 0x02u;
    }
    if (controller->hall_locked) {
        status |= 0x04u;
    }
    if (controller->acceleration_complete) {
        status |= 0x08u;
    }

    if (controller->upper_limit_enabled) {
        flags |= 0x01u;
    }
    if (controller->lower_limit_enabled) {
        flags |= 0x02u;
    }
    if (controller->regenerative_brake_enabled) {
        flags |= 0x04u;
    }
    if (controller->drive_command == TC11P_DRIVE_FORWARD) {
        flags |= 0x10u;
    }
    if (controller->drive_command == TC11P_DRIVE_REVERSE) {
        flags |= 0x20u;
    }

    controller->nv_image.saved_position = (uint16_t)controller->position_counts;
    controller->nv_image.saved_target_position = (uint16_t)controller->target_position_counts;
    controller->nv_image.saved_min_speed = (uint16_t)controller->config.min_speed_counts_per_tick;
    controller->nv_image.saved_max_speed = (uint16_t)controller->config.max_speed_counts_per_tick;
    controller->nv_image.mode_byte = (uint8_t)controller->mode;
    controller->nv_image.status_byte = status;
    controller->nv_image.flags_byte = flags;
    controller->nv_image.saved_hall_code = controller->hall.raw_code;
    controller->nv_image.saved_fault_code = (uint8_t)controller->fault;
}

static void tc11p_restore_nv_image(Tc11pController *controller)
{
    controller->position_counts = (int32_t)controller->nv_image.saved_position;
    controller->target_position_counts = (int32_t)controller->nv_image.saved_target_position;
    controller->config.min_speed_counts_per_tick = (int32_t)controller->nv_image.saved_min_speed;
    controller->config.max_speed_counts_per_tick = (int32_t)controller->nv_image.saved_max_speed;
    controller->mode = (Tc11pControlMode)controller->nv_image.mode_byte;
    controller->enabled = (controller->nv_image.status_byte & 0x01u) != 0u;
    controller->closed_loop_active = (controller->nv_image.status_byte & 0x02u) != 0u;
    controller->hall_locked = (controller->nv_image.status_byte & 0x04u) != 0u;
    controller->acceleration_complete = (controller->nv_image.status_byte & 0x08u) != 0u;
    controller->upper_limit_enabled = (controller->nv_image.flags_byte & 0x01u) != 0u;
    controller->lower_limit_enabled = (controller->nv_image.flags_byte & 0x02u) != 0u;
    controller->regenerative_brake_enabled = (controller->nv_image.flags_byte & 0x04u) != 0u;
    controller->fault = TC11P_FAULT_NONE;
    controller->hall.raw_code = controller->nv_image.saved_hall_code;
    controller->hall.sector = tc11p_decode_hall_sector(controller->hall.raw_code);
    controller->hall.valid = controller->hall.sector >= 0;

    if ((controller->nv_image.flags_byte & 0x10u) != 0u) {
        controller->drive_command = TC11P_DRIVE_FORWARD;
        controller->direction = TC11P_DIRECTION_FORWARD;
    } else if ((controller->nv_image.flags_byte & 0x20u) != 0u) {
        controller->drive_command = TC11P_DRIVE_REVERSE;
        controller->direction = TC11P_DIRECTION_REVERSE;
    } else {
        controller->drive_command = TC11P_DRIVE_BRAKE;
        controller->direction = TC11P_DIRECTION_STOP;
    }
}

void tc11p_controller_init(
    Tc11pController *controller,
    const Tc11pHardware *hardware,
    void *hardware_context,
    const Tc11pConfig *config)
{
    memset(controller, 0, sizeof(*controller));

    if (hardware != NULL) {
        controller->hw = *hardware;
    }
    controller->hw_context = hardware_context;

    if (config != NULL) {
        controller->config = *config;
    } else {
        tc11p_config_defaults(&controller->config);
    }

    controller->mode = TC11P_CONTROL_DISABLED;
    controller->direction = TC11P_DIRECTION_STOP;
    controller->drive_command = TC11P_DRIVE_BRAKE;
    controller->fault = TC11P_FAULT_NONE;
    controller->upper_limit_enabled = true;
    controller->lower_limit_enabled = true;
    controller->regenerative_brake_enabled = controller->config.regenerative_brake_enabled;
    controller->status_ticks_remaining = controller->config.status_period_ticks;
    controller->save_ticks_remaining = controller->config.save_period_ticks;
    controller->previous_sector = 0xffu;
    tc11p_set_outputs_disabled(controller);
}

/* Original firmware mapping: FUN_CODE_2e03 */
bool tc11p_controller_boot(Tc11pController *controller)
{
    bool restored = false;

    if ((controller->hw.nv_load != NULL) &&
        controller->hw.nv_load(controller->hw_context, &controller->nv_image)) {
        tc11p_restore_nv_image(controller);
        restored = true;
    }

    controller->status_dirty = true;
    controller->save_requested = false;
    controller->timer_ticks = 0;
    controller->fast_ticks = 0;
    controller->hall_edge_counter = 0;
    controller->speed_counts_per_tick = 0;
    controller->filtered_speed_counts_per_tick = 0;
    controller->last_position_counts = controller->position_counts;
    controller->align_ticks_remaining = 0;
    controller->soft_start_ticks_remaining = 0;

    if (!restored) {
        controller->enabled = false;
        controller->closed_loop_active = false;
        controller->hall_locked = false;
        controller->acceleration_complete = false;
        controller->mode = TC11P_CONTROL_DISABLED;
        controller->target_position_counts = 0;
    }

    tc11p_set_outputs_disabled(controller);
    return restored;
}

static void tc11p_begin_alignment(Tc11pController *controller)
{
    controller->align_ticks_remaining = controller->config.align_ticks;
    controller->soft_start_ticks_remaining = controller->config.soft_start_ticks;
    controller->acceleration_complete = false;
    controller->hall_locked = false;
    controller->pwm_target = controller->config.pwm_startup;
    controller->pwm_output = controller->config.pwm_startup;
    controller->drive_command = TC11P_DRIVE_BRAKE;
}

static void tc11p_update_hall_sample(Tc11pController *controller)
{
    uint8_t raw_code;
    int8_t sector;
    int8_t step;

    controller->hall.edge_detected = false;

    if (controller->hw.read_hall_code == NULL) {
        return;
    }

    raw_code = controller->hw.read_hall_code(controller->hw_context) & 0x07u;
    sector = tc11p_decode_hall_sector(raw_code);

    controller->hall.raw_code = raw_code;
    controller->hall.sector = sector;
    controller->hall.valid = sector >= 0;

    if (!controller->hall.valid) {
        controller->hall_stuck_counter++;
        if (controller->config.hall_required && controller->enabled) {
            tc11p_raise_fault(controller, TC11P_FAULT_INVALID_HALL);
        }
        return;
    }

    if (controller->previous_sector == 0xffu) {
        controller->previous_sector = (uint8_t)sector;
        controller->hall_locked = true;
        controller->hall_stuck_counter = 0;
        return;
    }

    if (controller->previous_sector != (uint8_t)sector) {
        step = tc11p_sector_step(controller->previous_sector, (uint8_t)sector);
        controller->hall.edge_detected = true;
        controller->hall_edge_counter++;
        controller->hall_stuck_counter = 0;

        if (step == 0) {
            tc11p_raise_fault(controller, TC11P_FAULT_INVALID_HALL);
            return;
        }

        controller->position_counts += step;
        controller->previous_sector = (uint8_t)sector;
        controller->hall_locked = true;
    } else {
        controller->hall_stuck_counter++;
    }
}

static void tc11p_update_speed_estimate(Tc11pController *controller)
{
    int32_t position_delta;

    position_delta = controller->position_counts - controller->last_position_counts;
    controller->last_position_counts = controller->position_counts;
    controller->speed_counts_per_tick = position_delta;
    controller->filtered_speed_counts_per_tick =
        (controller->filtered_speed_counts_per_tick * 3 + position_delta) / 4;
}

static void tc11p_update_position_mode(Tc11pController *controller)
{
    float command;
    int32_t limited_speed;

    controller->position_error_counts =
        controller->target_position_counts - controller->position_counts;

    if (tc11p_abs32(controller->position_error_counts) <= controller->config.position_deadband_counts) {
        controller->target_speed_counts_per_tick = 0;
        return;
    }

    controller->position_integral +=
        controller->config.position_ki * (float)controller->position_error_counts;

    command =
        controller->config.position_kp * (float)controller->position_error_counts +
        controller->position_integral;

    limited_speed = (int32_t)command;
    limited_speed = tc11p_clamp32(
        limited_speed,
        -controller->config.max_speed_counts_per_tick,
        controller->config.max_speed_counts_per_tick);

    if (limited_speed == 0) {
        limited_speed = tc11p_sign32(controller->position_error_counts) *
                        controller->config.min_speed_counts_per_tick;
    }

    controller->target_speed_counts_per_tick = limited_speed;
}

static void tc11p_update_speed_controller(Tc11pController *controller)
{
    float speed_error;
    float control;
    uint16_t requested_pwm;

    if (!controller->enabled || controller->fault != TC11P_FAULT_NONE) {
        controller->pwm_target = 0;
        return;
    }

    if (controller->target_speed_counts_per_tick == 0) {
        controller->pwm_target = 0;
        controller->drive_command = TC11P_DRIVE_BRAKE;
        controller->direction = TC11P_DIRECTION_STOP;
        return;
    }

    controller->direction = controller->target_speed_counts_per_tick > 0 ?
        TC11P_DIRECTION_FORWARD : TC11P_DIRECTION_REVERSE;
    controller->drive_command = controller->target_speed_counts_per_tick > 0 ?
        TC11P_DRIVE_FORWARD : TC11P_DRIVE_REVERSE;

    if (!controller->closed_loop_active || !controller->hall.valid) {
        requested_pwm = (uint16_t)tc11p_abs32(controller->target_speed_counts_per_tick);
        requested_pwm = tc11p_clamp_u16(
            requested_pwm + controller->config.pwm_startup,
            controller->config.pwm_min,
            controller->config.pwm_max);
        controller->pwm_target = requested_pwm;
        return;
    }

    speed_error =
        (float)controller->target_speed_counts_per_tick -
        (float)controller->filtered_speed_counts_per_tick;

    controller->speed_integral += controller->config.speed_ki * speed_error;
    if (controller->speed_integral > (float)controller->config.pwm_max) {
        controller->speed_integral = (float)controller->config.pwm_max;
    }
    if (controller->speed_integral < -(float)controller->config.pwm_max) {
        controller->speed_integral = -(float)controller->config.pwm_max;
    }

    control = controller->config.speed_kp * speed_error + controller->speed_integral;
    requested_pwm = (uint16_t)tc11p_absf(control);
    requested_pwm = tc11p_clamp_u16(
        requested_pwm + controller->config.pwm_min,
        controller->config.pwm_min,
        controller->config.pwm_max);

    if (controller->soft_start_ticks_remaining > 0) {
        uint16_t startup_limit = (uint16_t)(controller->config.pwm_startup +
            (controller->config.soft_start_ticks - controller->soft_start_ticks_remaining) *
            controller->config.pwm_ramp_step);
        if (requested_pwm > startup_limit) {
            requested_pwm = startup_limit;
        }
        controller->soft_start_ticks_remaining--;
        controller->acceleration_complete = controller->soft_start_ticks_remaining == 0;
    }

    controller->pwm_target = requested_pwm;
}

static void tc11p_ramp_pwm_output(Tc11pController *controller)
{
    if (controller->pwm_output < controller->pwm_target) {
        uint16_t next_pwm = (uint16_t)(controller->pwm_output + controller->config.pwm_ramp_step);
        if (next_pwm < controller->pwm_output || next_pwm > controller->pwm_target) {
            controller->pwm_output = controller->pwm_target;
        } else {
            controller->pwm_output = next_pwm;
        }
        return;
    }

    if (controller->pwm_output > controller->pwm_target) {
        if (controller->pwm_output - controller->pwm_target <= controller->config.pwm_ramp_step) {
            controller->pwm_output = controller->pwm_target;
        } else {
            controller->pwm_output = (uint16_t)(controller->pwm_output - controller->config.pwm_ramp_step);
        }
    }
}

static void tc11p_apply_drive_outputs(Tc11pController *controller)
{
    uint8_t sector;

    if (!controller->enabled || controller->fault != TC11P_FAULT_NONE) {
        tc11p_set_outputs_disabled(controller);
        return;
    }

    if (controller->align_ticks_remaining > 0) {
        controller->align_ticks_remaining--;

        if (controller->hw.set_brake != NULL) {
            controller->hw.set_brake(controller->hw_context, false);
        }
        if (controller->hw.set_gate_enable != NULL) {
            controller->hw.set_gate_enable(controller->hw_context, true);
        }
        if (controller->hw.apply_commutation != NULL) {
            controller->hw.apply_commutation(controller->hw_context, 0, TC11P_DIRECTION_FORWARD, true);
        }
        if (controller->hw.set_pwm_duty != NULL) {
            controller->hw.set_pwm_duty(controller->hw_context, controller->config.pwm_startup);
        }
        return;
    }

    if (controller->drive_command == TC11P_DRIVE_BRAKE || controller->pwm_output == 0) {
        tc11p_set_outputs_disabled(controller);
        return;
    }

    if (controller->config.hall_required && !controller->hall.valid) {
        tc11p_raise_fault(controller, TC11P_FAULT_INVALID_HALL);
        return;
    }

    sector = controller->hall.valid ? (uint8_t)controller->hall.sector : controller->previous_sector;

    if (controller->hw.set_brake != NULL) {
        controller->hw.set_brake(controller->hw_context, false);
    }
    if (controller->hw.set_gate_enable != NULL) {
        controller->hw.set_gate_enable(controller->hw_context, true);
    }
    if (controller->hw.apply_commutation != NULL) {
        controller->hw.apply_commutation(controller->hw_context, sector, controller->direction, true);
    }
    if (controller->hw.set_pwm_duty != NULL) {
        controller->hw.set_pwm_duty(controller->hw_context, controller->pwm_output);
    }
}

static void tc11p_handle_position_limits(Tc11pController *controller)
{
    if (controller->upper_limit_enabled &&
        controller->position_counts >= controller->config.upper_position_limit &&
        controller->direction == TC11P_DIRECTION_FORWARD) {
        tc11p_raise_fault(controller, TC11P_FAULT_LIMIT);
        return;
    }

    if (controller->lower_limit_enabled &&
        controller->position_counts <= controller->config.lower_position_limit &&
        controller->direction == TC11P_DIRECTION_REVERSE) {
        tc11p_raise_fault(controller, TC11P_FAULT_LIMIT);
        return;
    }

    if (controller->hw.read_upper_limit != NULL && controller->hw.read_upper_limit(controller->hw_context) &&
        controller->direction == TC11P_DIRECTION_FORWARD) {
        tc11p_raise_fault(controller, TC11P_FAULT_LIMIT);
        return;
    }

    if (controller->hw.read_lower_limit != NULL && controller->hw.read_lower_limit(controller->hw_context) &&
        controller->direction == TC11P_DIRECTION_REVERSE) {
        tc11p_raise_fault(controller, TC11P_FAULT_LIMIT);
    }
}

static void tc11p_handle_safety(Tc11pController *controller)
{
    uint16_t current_raw;
    uint16_t temperature_raw;
    uint16_t supply_raw;

    if (controller->hw.read_emergency_stop != NULL &&
        controller->hw.read_emergency_stop(controller->hw_context)) {
        tc11p_raise_fault(controller, TC11P_FAULT_EMERGENCY_STOP);
        return;
    }

    if (controller->hw.read_current_raw != NULL) {
        current_raw = controller->hw.read_current_raw(controller->hw_context);
        if (current_raw > controller->config.overcurrent_limit_raw) {
            controller->overcurrent_counter++;
            if (controller->overcurrent_counter >= 3u) {
                tc11p_raise_fault(controller, TC11P_FAULT_OVERCURRENT);
                return;
            }
        } else {
            controller->overcurrent_counter = 0;
        }
    }

    if (controller->hw.read_temperature_raw != NULL) {
        temperature_raw = controller->hw.read_temperature_raw(controller->hw_context);
        if (temperature_raw > controller->config.overtemp_limit_raw) {
            tc11p_raise_fault(controller, TC11P_FAULT_OVERTEMP);
            return;
        }
    }

    if (controller->hw.read_supply_raw != NULL) {
        supply_raw = controller->hw.read_supply_raw(controller->hw_context);
        if (supply_raw < controller->config.undervoltage_limit_raw) {
            tc11p_raise_fault(controller, TC11P_FAULT_UNDERVOLTAGE);
            return;
        }
    }

    if (tc11p_abs32(controller->filtered_speed_counts_per_tick) >
        controller->config.overspeed_limit_counts_per_tick) {
        controller->overspeed_counter++;
        if (controller->overspeed_counter >= 4u) {
            tc11p_raise_fault(controller, TC11P_FAULT_OVERSPEED);
            return;
        }
    } else {
        controller->overspeed_counter = 0;
    }

    if (controller->enabled && controller->config.hall_required &&
        controller->hall_stuck_counter > controller->config.hall_stuck_ticks) {
        tc11p_raise_fault(controller, TC11P_FAULT_HALL_STUCK);
        return;
    }

    if (controller->enabled && controller->pwm_output > controller->config.pwm_startup &&
        controller->target_speed_counts_per_tick != 0 &&
        controller->filtered_speed_counts_per_tick == 0) {
        controller->stall_counter++;
        if (controller->stall_counter >= controller->config.hall_stuck_ticks) {
            tc11p_raise_fault(controller, TC11P_FAULT_STALL);
        }
    } else {
        controller->stall_counter = 0;
    }
}

static void tc11p_maybe_save_state(Tc11pController *controller)
{
    if (!controller->save_requested) {
        return;
    }

    if (controller->hw.nv_save == NULL) {
        return;
    }

    tc11p_capture_nv_image(controller);
    if (!controller->hw.nv_save(controller->hw_context, &controller->nv_image)) {
        tc11p_raise_fault(controller, TC11P_FAULT_NV_STORAGE);
        return;
    }

    controller->save_requested = false;
}

/* Original firmware mapping: FUN_CODE_555d */
void tc11p_controller_timer0_isr(Tc11pController *controller)
{
    controller->timer_ticks++;

    if (controller->hw.watchdog_kick != NULL) {
        controller->hw.watchdog_kick(controller->hw_context);
    }

    tc11p_update_hall_sample(controller);
    tc11p_update_speed_estimate(controller);
    tc11p_handle_safety(controller);

    if (controller->status_ticks_remaining > 0) {
        controller->status_ticks_remaining--;
    }
    if (controller->save_ticks_remaining > 0) {
        controller->save_ticks_remaining--;
    }

    if (controller->save_ticks_remaining == 0) {
        controller->save_requested = true;
        controller->save_ticks_remaining = controller->config.save_period_ticks;
    }
}

/* Original firmware mapping: FUN_CODE_154b */
void tc11p_controller_fast_isr(Tc11pController *controller)
{
    controller->fast_ticks++;

    if (!controller->enabled || controller->fault != TC11P_FAULT_NONE) {
        tc11p_set_outputs_disabled(controller);
        return;
    }

    controller->closed_loop_active = controller->config.hall_required ? controller->hall.valid : false;

    if (controller->mode == TC11P_CONTROL_POSITION) {
        tc11p_update_position_mode(controller);
    }

    tc11p_update_speed_controller(controller);
    tc11p_handle_position_limits(controller);
    tc11p_ramp_pwm_output(controller);
    tc11p_apply_drive_outputs(controller);
}

static void tc11p_send_status_frame(Tc11pController *controller)
{
    uint8_t frame[TC11P_STATUS_FRAME_SIZE];
    uint8_t checksum;
    uint16_t speed_abs;

    if (controller->hw.uart_send == NULL) {
        return;
    }

    frame[0] = 0x66u;
    frame[1] = (uint8_t)controller->mode;
    frame[2] = (uint8_t)controller->fault;
    frame[3] = controller->hall.raw_code;
    tc11p_write_u16_be(&frame[4], (uint16_t)controller->position_counts);
    tc11p_write_u16_be(&frame[6], (uint16_t)controller->target_position_counts);
    speed_abs = (uint16_t)tc11p_abs32(controller->filtered_speed_counts_per_tick);
    tc11p_write_u16_be(&frame[8], speed_abs);
    tc11p_write_u16_be(&frame[10], controller->pwm_output);
    frame[12] = 0;

    checksum = 0;
    for (size_t index = 0; index < TC11P_STATUS_FRAME_SIZE - 1u; ++index) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    frame[12] = checksum;

    controller->hw.uart_send(controller->hw_context, frame, sizeof(frame));
}

/* Original firmware mapping: FUN_CODE_0f9b */
void tc11p_controller_step(Tc11pController *controller)
{
    if (controller->fault != TC11P_FAULT_NONE) {
        controller->enabled = false;
        tc11p_set_outputs_disabled(controller);
    }

    if (controller->enabled && !controller->hall_locked && controller->config.hall_required) {
        if (controller->align_ticks_remaining == 0) {
            tc11p_begin_alignment(controller);
        }
    }

    if (controller->status_ticks_remaining == 0) {
        controller->status_ticks_remaining = controller->config.status_period_ticks;
        controller->status_dirty = false;
        tc11p_send_status_frame(controller);
    }

    if (controller->save_requested) {
        tc11p_maybe_save_state(controller);
    }
}

/* Original firmware mapping: FUN_CODE_1fa2 */
void tc11p_controller_fault_transition(
    Tc11pController *controller,
    Tc11pFaultCode fault,
    bool request_state_save)
{
    tc11p_raise_fault(controller, fault);
    if (request_state_save) {
        controller->save_requested = true;
    }
}

void tc11p_controller_set_target_position(Tc11pController *controller, int32_t target_position_counts)
{
    controller->target_position_counts = tc11p_clamp32(
        target_position_counts,
        controller->config.lower_position_limit,
        controller->config.upper_position_limit);
    controller->mode = TC11P_CONTROL_POSITION;
    controller->status_dirty = true;
}

void tc11p_controller_set_target_speed(Tc11pController *controller, int32_t target_speed_counts_per_tick)
{
    controller->target_speed_counts_per_tick = tc11p_clamp32(
        target_speed_counts_per_tick,
        -controller->config.max_speed_counts_per_tick,
        controller->config.max_speed_counts_per_tick);
    controller->mode = TC11P_CONTROL_SPEED;
    controller->status_dirty = true;
}

void tc11p_controller_enable(Tc11pController *controller)
{
    if (controller->fault != TC11P_FAULT_NONE) {
        return;
    }

    controller->enabled = true;
    controller->closed_loop_active = false;
    controller->save_requested = true;
    controller->status_dirty = true;
    tc11p_begin_alignment(controller);
}

void tc11p_controller_disable(Tc11pController *controller)
{
    controller->enabled = false;
    controller->closed_loop_active = false;
    controller->hall_locked = false;
    controller->save_requested = true;
    controller->status_dirty = true;
    tc11p_reset_integrators(controller);
    tc11p_set_outputs_disabled(controller);
}

void tc11p_controller_apply_command(Tc11pController *controller, Tc11pCommand command)
{
    switch (command.type) {
    case TC11P_COMMAND_ENABLE:
        tc11p_controller_enable(controller);
        break;
    case TC11P_COMMAND_DISABLE:
        tc11p_controller_disable(controller);
        break;
    case TC11P_COMMAND_BRAKE:
        controller->drive_command = TC11P_DRIVE_BRAKE;
        controller->target_speed_counts_per_tick = 0;
        controller->pwm_target = 0;
        controller->status_dirty = true;
        break;
    case TC11P_COMMAND_CLEAR_FAULT:
        tc11p_clear_fault(controller);
        break;
    case TC11P_COMMAND_SET_MODE:
        controller->mode = (Tc11pControlMode)command.value;
        controller->status_dirty = true;
        break;
    case TC11P_COMMAND_SET_TARGET_POSITION:
        tc11p_controller_set_target_position(controller, command.value);
        break;
    case TC11P_COMMAND_SET_TARGET_SPEED:
        tc11p_controller_set_target_speed(controller, command.value);
        break;
    case TC11P_COMMAND_SAVE_STATE:
        controller->save_requested = true;
        break;
    case TC11P_COMMAND_LOAD_STATE:
        if (!tc11p_controller_boot(controller)) {
            controller->status_dirty = true;
        }
        break;
    default:
        break;
    }
}

size_t tc11p_controller_build_status_frame(
    const Tc11pController *controller,
    uint8_t *frame,
    size_t frame_capacity)
{
    uint8_t checksum;

    if (frame_capacity < TC11P_STATUS_FRAME_SIZE) {
        return 0;
    }

    frame[0] = 0x66u;
    frame[1] = (uint8_t)controller->mode;
    frame[2] = (uint8_t)controller->fault;
    frame[3] = controller->hall.raw_code;
    tc11p_write_u16_be(&frame[4], (uint16_t)controller->position_counts);
    tc11p_write_u16_be(&frame[6], (uint16_t)controller->target_position_counts);
    tc11p_write_u16_be(&frame[8], (uint16_t)tc11p_abs32(controller->filtered_speed_counts_per_tick));
    tc11p_write_u16_be(&frame[10], controller->pwm_output);

    checksum = 0;
    for (size_t index = 0; index < TC11P_STATUS_FRAME_SIZE - 1u; ++index) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    frame[12] = checksum;

    return TC11P_STATUS_FRAME_SIZE;
}
