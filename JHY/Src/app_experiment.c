/* JHY: no automatic arm; bounded control ISR and queued UART DMA telemetry. */
#include "app_experiment.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define QUEUE_SIZE 16u
#define FEEDBACK_TIMEOUT_MS 100u
#define COMMAND_TIMEOUT_MS 500u
#define TEMPERATURE_LIMIT_C 70u
#define TELEMETRY_MS 20u
#define RX_QUEUE_SIZE 256u

typedef struct {
    uint32_t ms, fault, armed, drops, can_errors, command_errors;
    float target[4], omega[4];
    int16_t current[4];
    uint8_t temperature[4];
    uint32_t age[4];
} Sample;

static Struct_RM_Motor_M3508 *motors[4];
static Struct_Chassis_Config *geometry;
static Struct_Target_Velocity command;
static volatile uint32_t last_rx[4], seen[4], last_command;
static volatile uint32_t armed, faults, dropped, can_errors, command_errors;
static Sample queue[QUEUE_SIZE];
static volatile uint32_t head, tail;
static char tx[640], line[100];
static unsigned line_length;
static uint8_t line_overflow;
static uint8_t initialized;
static uint8_t rx_byte, rx_queue[RX_QUEUE_SIZE];
static volatile uint32_t rx_head, rx_tail, rx_errors;

static uint32_t lock_interrupts(void) {
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}
static void unlock_interrupts(uint32_t state) { __set_PRIMASK(state); }

static void zero_outputs(void) {
    unsigned i;
    memset(&command, 0, sizeof(command));
    for (i = 0; i < 4; ++i) {
        Struct_RM_Motor_M3508 *m = motors[i];
        m->Target_Omega = 0.0f;
        m->Out = 0.0f;
        m->PID_Omega.I_Error = 0.0f;
        m->PID_Omega.Pre_Error = 0.0f;
        m->PID_Omega.Pre_Now = m->Now_Omega;
        DJI_M3508_Output(m);
    }
}

static uint32_t health(uint32_t now) {
    uint32_t result = 0;
    unsigned i;
    if (!isfinite(geometry->R) || geometry->R <= 0.0f ||
        !isfinite(geometry->a) || !isfinite(geometry->b) ||
        geometry->a <= 0.0f || geometry->b <= 0.0f ||
        !isfinite(geometry->max_omega) || geometry->max_omega <= 0.0f ||
        !isfinite(command.vx) || !isfinite(command.vy) || !isfinite(command.omega))
        result |= EXP_FAULT_INVALID;
    for (i = 0; i < 4; ++i) {
        if (!seen[i] || (uint32_t)(now - last_rx[i]) > FEEDBACK_TIMEOUT_MS)
            result |= EXP_FAULT_OFFLINE;
        if (motors[i]->Now_Temperature >= TEMPERATURE_LIMIT_C)
            result |= EXP_FAULT_TEMPERATURE;
        if (!isfinite(motors[i]->Now_Omega)) result |= EXP_FAULT_INVALID;
        else if (fabsf(motors[i]->Now_Omega) > geometry->max_omega * 1.2f)
            result |= EXP_FAULT_OVERSPEED;
    }
    return result;
}

void Experiment_Init(Struct_RM_Motor_M3508 *lf, Struct_RM_Motor_M3508 *lb,
                     Struct_RM_Motor_M3508 *rb, Struct_RM_Motor_M3508 *rf,
                     Struct_Chassis_Config *config) {
    motors[0] = lf; motors[1] = lb; motors[2] = rb; motors[3] = rf;
    geometry = config;
    zero_outputs();
    initialized = 1;
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK) ++rx_errors;
}

/* Interrupt reception avoids dropping command bytes while main formats CSV. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart != &huart1) return;
    uint32_t next = (rx_head + 1u) % RX_QUEUE_SIZE;
    if (next == rx_tail) ++rx_errors;
    else { rx_queue[rx_head] = rx_byte; __DMB(); rx_head = next; }
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1) != HAL_OK) ++rx_errors;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if (uart != &huart1) return;
    ++rx_errors;
    if (uart->RxState == HAL_UART_STATE_READY)
        (void)HAL_UART_Receive_IT(uart, &rx_byte, 1);
}

void Experiment_Feedback(unsigned wheel) {
    if (wheel < 4) {
        last_rx[wheel] = HAL_GetTick();
        __DMB();
        seen[wheel] = 1;
    }
}

void Experiment_CAN_Tx_Result(HAL_StatusTypeDef result) {
    if (result != HAL_OK) {
        ++can_errors;
        if (armed) { faults |= EXP_FAULT_CAN; armed = 0; zero_outputs(); }
    }
}

void Experiment_Control_1ms(void) {
    uint32_t now, error;
    static uint32_t divider;
    unsigned i;
    Struct_Wheel_Velocity wheel;
    float target[4];
    if (!initialized) return;
    now = HAL_GetTick();
    error = health(now);
    if (armed) {
        if (rx_errors) error |= EXP_FAULT_INVALID;
        if ((uint32_t)(now - last_command) > COMMAND_TIMEOUT_MS)
            error |= EXP_FAULT_COMMAND_TIMEOUT;
        if (error) { faults |= error; armed = 0; }
    }
    if (armed && !faults) {
        Mecanum_Wheel_Inverse_Kinematics(geometry, &command, &wheel);
        /* Keep supplied firmware motor wiring signs; verify each on a raised rig. */
        target[0] = -wheel.LF; target[1] = -wheel.LB;
        target[2] = -wheel.RB; target[3] = -wheel.RF;
        for (i = 0; i < 4; ++i) {
            motors[i]->Target_Omega = target[i];
            DJI_M3508_TIM_PID_PeriodElapsedCallback(motors[i]);
            if (!isfinite(motors[i]->Out)) { faults |= EXP_FAULT_INVALID; armed = 0; }
        }
    }
    if (!armed || faults) zero_outputs();

    if (++divider >= TELEMETRY_MS) {
        uint32_t next = (head + 1u) % QUEUE_SIZE;
        divider = 0;
        if (next == tail) ++dropped;
        else {
            Sample *s = &queue[head];
            s->ms = now; s->fault = faults | (error & ~EXP_FAULT_COMMAND_TIMEOUT);
            s->armed = armed; s->drops = dropped; s->can_errors = can_errors;
            s->command_errors = command_errors;
            for (i = 0; i < 4; ++i) {
                s->target[i] = motors[i]->Target_Omega;
                s->omega[i] = motors[i]->Now_Omega;
                s->current[i] = motors[i]->Rx_Torque;
                s->temperature[i] = motors[i]->Now_Temperature;
                s->age[i] = seen[i] ? now - last_rx[i] : UINT32_MAX;
            }
            __DMB(); head = next;
        }
    }
}

/* Executed in main; atomically publishes command/config changes to the ISR. */
static void process_line(char *text) {
    char op[16], extra;
    float a, b, c;
    int fields = sscanf(text, "%15s %f %f %f %c", op, &a, &b, &c, &extra);
    uint32_t state = lock_interrupts(), now = HAL_GetTick();
    int valid = 1;
    if (fields == 1 && strcmp(text, "STOP") == 0) {
        armed = 0; zero_outputs();
    } else if (fields == 1 && strcmp(text, "ESTOP") == 0) {
        faults |= EXP_FAULT_ESTOP; armed = 0; zero_outputs();
    } else if (fields == 1 && strcmp(text, "RESET") == 0) {
        armed = 0; zero_outputs();
        faults = health(now); /* RESET never arms or clears an active fault. */
    } else if (fields == 1 && strcmp(text, "ARM") == 0) {
        zero_outputs();
        if (!faults && !health(now)) { armed = 1; last_command = now; }
        else valid = 0;
    } else if (fields == 1 && strcmp(text, "KEEP") == 0) {
        last_command = now;
    } else if (fields == 4 && isfinite(a) && isfinite(b) && isfinite(c)) {
        if (strcmp(op, "SET") == 0 && fabsf(a) <= 3.0f && fabsf(b) <= 3.0f && fabsf(c) <= 6.0f) {
            command.vx = a; command.vy = b; command.omega = c; last_command = now;
        } else if (strcmp(op, "PID") == 0 && !armed && a >= 0 && a <= 10000 && b >= 0 && b <= 1000 && c >= 0 && c <= 1000) {
            unsigned i;
            for (i = 0; i < 4; ++i) {
                motors[i]->PID_Omega.K_P = a; motors[i]->PID_Omega.K_I = b;
                motors[i]->PID_Omega.K_D = c;
            }
        } else if (strcmp(op, "GEOM") == 0 && !armed && a >= .01f && a <= 1 && b >= .01f && b <= 1 && c >= .01f && c <= .5f) {
            geometry->a = a; geometry->b = b; geometry->R = c;
        } else valid = 0;
    } else valid = 0;
    if (!valid) {
        ++command_errors;
        /* A malformed moving command fails closed. */
        if (armed) { faults |= EXP_FAULT_INVALID; armed = 0; zero_outputs(); }
    }
    unlock_interrupts(state);
}

void Experiment_Poll(void) {
    static uint8_t header_sent;
    static uint32_t last_header;
    uint8_t ch;
    unsigned budget = 32;
    if (rx_errors) {
        uint32_t state = lock_interrupts();
        command_errors += rx_errors; rx_errors = 0;
        rx_tail = rx_head;
        unlock_interrupts(state);
        /* Discard the rest of this damaged line before interpreting commands. */
        line_length = 0; line_overflow = 1;
        process_line("INVALID");
    }
    while (budget-- && rx_tail != rx_head) {
        ch = rx_queue[rx_tail];
        __DMB(); rx_tail = (rx_tail + 1u) % RX_QUEUE_SIZE;
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (line_overflow) process_line("INVALID");
            else { line[line_length] = '\0'; if (line_length) process_line(line); }
            line_length = 0; line_overflow = 0;
        } else if (line_length + 1 < sizeof(line) && !line_overflow) line[line_length++] = (char)ch;
        else line_overflow = 1;
    }
    if (huart1.gState != HAL_UART_STATE_READY) return;
    if ((uint32_t)(HAL_GetTick() - last_header) >= 5000u) header_sent = 0;
    if (!header_sent) {
        static const char header[] = "ms,armed,fault,dropped,can_errors,command_errors,LF_target,LB_target,RB_target,RF_target,LF_omega,LB_omega,RB_omega,RF_omega,LF_current_raw,LB_current_raw,RB_current_raw,RF_current_raw,LF_temp_C,LB_temp_C,RB_temp_C,RF_temp_C,LF_age_ms,LB_age_ms,RB_age_ms,RF_age_ms\r\n";
        if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)header, sizeof(header)-1) == HAL_OK) { header_sent = 1; last_header = HAL_GetTick(); }
    } else if (tail != head) {
        Sample s = queue[tail];
        int n = snprintf(tx, sizeof(tx),
            "%lu,%lu,%lu,%lu,%lu,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%u,%u,%u,%u,%lu,%lu,%lu,%lu\r\n",
            (unsigned long)s.ms, (unsigned long)s.armed, (unsigned long)s.fault,
            (unsigned long)s.drops, (unsigned long)s.can_errors, (unsigned long)s.command_errors,
            (double)s.target[0], (double)s.target[1], (double)s.target[2], (double)s.target[3],
            (double)s.omega[0], (double)s.omega[1], (double)s.omega[2], (double)s.omega[3],
            s.current[0], s.current[1], s.current[2], s.current[3],
            s.temperature[0], s.temperature[1], s.temperature[2], s.temperature[3],
            (unsigned long)s.age[0], (unsigned long)s.age[1], (unsigned long)s.age[2], (unsigned long)s.age[3]);
        if (n > 0 && (unsigned)n < sizeof(tx) && HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx, (uint16_t)n) == HAL_OK) {
            __DMB(); tail = (tail + 1u) % QUEUE_SIZE;
        }
    }
}
