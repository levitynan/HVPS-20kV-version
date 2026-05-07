



/*
 * @file main.c
 * @brief Dual MCPWM Channel Controller via Serial Monitor
 *
 * This application controls two independent PWM signals (CH0 and CH1) on an
 * ESP32-WROOM-32 using the MCPWM (Motor Control PWM) peripheral driver from
 * ESP-IDF v5/v6 new-style API.
 *
 * Each channel has:
 *   - Independent GPIO output pin
 *   - Independently controllable ON/OFF state
 *   - Independently configurable frequency (Hz)
 *   - Independently configurable duty cycle (%)
 *
 * Serial command protocol (send via Serial Monitor at 115200 baud):
 *   ON  <ch>              - Start PWM output on channel 0 or 1
 *   OFF <ch>              - Stop  PWM output on channel 0 or 1
 *   FREQ <ch> <hz>        - Set frequency in Hz  (1 – 1,000,000)
 *   DUTY <ch> <percent>   - Set duty cycle in %  (0.0 – 100.0)
 *   STATUS                - Print current settings for both channels
 *
 * Examples:
 *   ON 0           -> Start channel 0
 *   FREQ 1 1000    -> Set channel 1 to 1 kHz
 *   DUTY 0 25.5    -> Set channel 0 duty to 25.5 %
 *   OFF 1          -> Stop channel 1
 *
 * Hardware wiring (ESP32-WROOM-32):
 *   Channel 0 PWM output  -> GPIO 18
 *   Channel 1 PWM output  -> GPIO 19
 *
 * Key MCPWM Concepts (ESP-IDF v5+ new API):
 *   Timer      – Counts at resolution_hz ticks/sec, resets at period_ticks.
 *                Frequency = resolution_hz / period_ticks
 *   Operator   – Links a timer to one or more generators.
 *   Comparator – Fires when timer count == compare_value. Used to create edges.
 *   Generator  – Defines what happens at timer events (set/clear the output GPIO).
 *
 * Dependencies (in idf_component.yml or CMakeLists REQUIRES):
 *   esp_driver_mcpwm, driver, esp_timer
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"   /* New MCPWM API (ESP-IDF v5+) */
#include "esp_log.h"

/* ─────────────────────────── Configuration ─────────────────────────────── */

/* Tag used in all ESP_LOG* calls for easy filtering in the monitor. */
static const char *TAG = "MCPWM_CTRL";

/* Number of independent PWM channels managed by this application. */
#define NUM_CHANNELS 2

/*
 * @brief GPIO pins for each PWM output.
 *
 * Change these to any free GPIO on your board.
 * Avoid strapping pins (0, 2, 12, 15) and input-only pins (34-39).
 */
static const int PWM_GPIO[NUM_CHANNELS] = {
    18,   /* Channel 0 */
    19    /* Channel 1 */
};

/*
 * @brief MCPWM hardware group IDs.
 *
 * The ESP32 has two MCPWM units (group 0 and group 1).
 * Each group provides 3 timers, 3 operators, 6 comparators and 6 generators.
 * We place each channel in its own group so frequencies are fully independent.
 */
static const int MCPWM_GROUP[NUM_CHANNELS] = {
    0,    /* Channel 0 uses MCPWM group 0 */
    1     /* Channel 1 uses MCPWM group 1 */
};

/*
 * @brief Timer resolution in ticks-per-second (Hz).
 *
 * Higher values give finer duty-cycle resolution but limit the maximum period
 * (period_ticks must fit in a 24-bit counter, so max period = 2^24 ticks).
 * 1 MHz → period granularity = 1 µs, supports frequencies down to ~0.06 Hz.
 */
#define TIMER_RESOLUTION_HZ  1000000UL   /* 1 MHz tick rate */

/* Default frequency for both channels at startup (Hz). */
#define DEFAULT_FREQ_HZ      20000U
    
/* Default duty cycle for both channels at startup (%). */
#define DEFAULT_DUTY_PCT     50.0f

/* Maximum line length accepted from the serial input buffer. */
#define CMD_BUF_LEN  64

/* ─────────────────────────── Channel State ─────────────────────────────── */

/*
 * @brief All runtime state for a single PWM channel.
 *
 * The MCPWM new API uses opaque handles for each hardware object.
 * We keep them here so the control functions can reference them at any time.
 */
typedef struct {
    /* --- MCPWM hardware handles --- */
    mcpwm_timer_handle_t      timer;       /**< Timer that drives the period */
    mcpwm_oper_handle_t       oper;        /**< Operator linking timer to generator */
    mcpwm_cmpr_handle_t       comparator;  /**< Comparator that fires the duty edge */
    mcpwm_gen_handle_t        generator;   /**< Generator driving the GPIO */

    /* --- Logical state --- */
    bool     running;      /**< true = PWM actively outputting */
    uint32_t freq_hz;      /**< Current frequency in Hz */
    float    duty_pct;     /**< Current duty cycle 0.0–100.0 % */

    /* --- Derived values (recalculated whenever freq or duty changes) --- */
    uint32_t period_ticks; /**< period_ticks = TIMER_RESOLUTION_HZ / freq_hz */
    uint32_t cmp_ticks;    /**< cmp_ticks = period_ticks * (duty_pct / 100) */
} pwm_channel_t;

/* Array of both channels. Indices match CH0/CH1 throughout. */
static pwm_channel_t g_ch[NUM_CHANNELS];

/* ─────────────────────────── Helper: Recalculate Ticks ─────────────────── */

/*
 * @brief Recompute period_ticks and cmp_ticks from freq_hz and duty_pct.
 *
 * Call this after changing either value before updating the hardware.
 *
 * @param ch Pointer to the channel whose ticks should be recalculated.
 */
static void recalc_ticks(pwm_channel_t *ch)
{
    /* period_ticks = timer_resolution / frequency
     * e.g. 1 MHz / 1000 Hz = 1000 ticks → 1 ms period */
    ch->period_ticks = TIMER_RESOLUTION_HZ / ch->freq_hz;

    /* compare_value = fraction of period where the output goes LOW
     * e.g. 50 % duty  → cmp = 500 of 1000 ticks */
    ch->cmp_ticks = (uint32_t)(ch->period_ticks * (ch->duty_pct / 100.0f));
}

/* ─────────────────────────── MCPWM Initialisation ──────────────────────── */

/*
 * @brief Initialise MCPWM hardware for one channel.
 *
 * This sets up the full MCPWM chain:
 *   Timer → Operator → Comparator → Generator → GPIO
 *
 * The generator is configured so that:
 *   - On timer EMPTY (count == 0)  → output goes HIGH
 *   - On comparator match           → output goes LOW
 * This produces an active-high PWM signal with adjustable duty.
 *
 * After init the timer is enabled but NOT started; call mcpwm_timer_start_stop
 * separately when the user sends ON/OFF commands.
 *
 * @param idx  Channel index (0 or 1).
 */
static void mcpwm_channel_init(int idx)
{
    pwm_channel_t *ch = &g_ch[idx];

    /* Compute initial tick values before touching hardware */
    recalc_ticks(ch);

    /* ── Step 1: Create the timer ─────────────────────────────────────────
     * The timer counts from 0 to period_ticks and then resets (UP mode).
     * resolution_hz sets how fast the counter increments (tick clock).
     */
    mcpwm_timer_config_t timer_cfg = {
        .group_id       = MCPWM_GROUP[idx],
        .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT,  /* 160 MHz APB clock */
        .resolution_hz  = TIMER_RESOLUTION_HZ,
        .period_ticks   = ch->period_ticks,
        .count_mode     = MCPWM_TIMER_COUNT_MODE_UP,     /* Count 0 → period, then wrap */
        .intr_priority  = 0,                             /* Default interrupt priority */
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &ch->timer));

    /* ── Step 2: Create the operator ──────────────────────────────────────
     * The operator must reside in the same group as its timer.
     */
    mcpwm_operator_config_t oper_cfg = {
        .group_id = MCPWM_GROUP[idx],
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &ch->oper));

    /* ── Step 3: Connect operator to timer ───────────────────────────────
     * This is mandatory before creating comparators or generators.
     */
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(ch->oper, ch->timer));

    /* ── Step 4: Create the comparator ───────────────────────────────────
     * The comparator fires an event when timer_count == compare_value.
     * We use this event to drive the falling edge of the PWM waveform.
     * update_cmp_on_tez = true → new compare value takes effect at next zero
     * (avoids glitches when changing duty cycle at runtime).
     */
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,   /* Apply new compare on timer zero */
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(ch->oper, &cmp_cfg, &ch->comparator));

    /* Load the initial compare value */
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(ch->comparator, ch->cmp_ticks));

    /* ── Step 5: Create the generator and bind it to the GPIO ────────────
     * The generator translates comparator/timer events into GPIO transitions.
     */
    mcpwm_generator_config_t gen_cfg = {
        .gen_gpio_num = PWM_GPIO[idx],
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(ch->oper, &gen_cfg, &ch->generator));

    /* ── Step 6: Configure generator actions ─────────────────────────────
     *  • Timer EMPTY event (count reaches 0 / wraps)  → GPIO HIGH  (rising edge)
     *  • Comparator event  (count == cmp_ticks)        → GPIO LOW   (falling edge)
     * Together these produce a standard active-high PWM signal.
     */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        ch->generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,   /* Counting upward */
            MCPWM_TIMER_EVENT_EMPTY,    /* Timer reached zero / wrapped */
            MCPWM_GEN_ACTION_HIGH       /* Drive GPIO HIGH */
        )
    ));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        ch->generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,   /* Counting upward */
            ch->comparator,             /* Our comparator */
            MCPWM_GEN_ACTION_LOW        /* Drive GPIO LOW */
        )
    ));

    /* ── Step 7: Enable the timer (moves it from INIT → ENABLE state) ────
     * Must call enable before start_stop commands will work.
     * The timer is NOT yet running at this point.
     */
    ESP_ERROR_CHECK(mcpwm_timer_enable(ch->timer));

    ESP_LOGI(TAG, "CH%d initialised: GPIO=%d  freq=%lu Hz  duty=%.1f%%",
             idx, PWM_GPIO[idx], (unsigned long)ch->freq_hz, ch->duty_pct);
}

/* ─────────────────────────── PWM Control Functions ─────────────────────── */

/*
 * @brief Start (resume) PWM output on the specified channel.
 *
 * Sends MCPWM_TIMER_START_NO_STOP which runs the timer continuously.
 * The generator will immediately begin toggling the GPIO.
 *
 * @param idx  Channel index (0 or 1).
 */
static void pwm_start(int idx)
{
    pwm_channel_t *ch = &g_ch[idx];
    if (ch->running) {
        ESP_LOGW(TAG, "CH%d already running", idx);
        return;
    }
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(ch->timer, MCPWM_TIMER_START_NO_STOP));
    ch->running = true;
    ESP_LOGI(TAG, "CH%d ON  (freq=%lu Hz, duty=%.1f%%)",
             idx, (unsigned long)ch->freq_hz, ch->duty_pct);
}

/*
 * @brief Stop PWM output on the specified channel.
 *
 * MCPWM_TIMER_STOP_EMPTY causes the timer to stop at the next EMPTY event
 * (i.e. after completing the current period), which ensures the GPIO does
 * not freeze mid-pulse and potentially damage connected hardware.
 *
 * @param idx  Channel index (0 or 1).
 */
static void pwm_stop(int idx)
{
    pwm_channel_t *ch = &g_ch[idx];
    if (!ch->running) {
        ESP_LOGW(TAG, "CH%d already stopped", idx);
        return;
    }
    /* Stop at next timer EMPTY so output completes the current cycle cleanly */
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(ch->timer, MCPWM_TIMER_STOP_EMPTY));
    ch->running = false;
    ESP_LOGI(TAG, "CH%d OFF", idx);
}

/*
 * @brief Change the PWM frequency of a channel at runtime.
 *
 * In the new MCPWM API, frequency is changed by updating period_ticks on the
 * timer using mcpwm_timer_set_period(). The compare value must also be updated
 * so the duty cycle percentage is preserved after the period change.
 *
 * @param idx     Channel index (0 or 1).
 * @param freq_hz New frequency in Hz. Clamped to [1, TIMER_RESOLUTION_HZ].
 */
static void pwm_set_freq(int idx, uint32_t freq_hz)
{
    pwm_channel_t *ch = &g_ch[idx];

    /* Clamp to valid range */
    if (freq_hz < 1)                  freq_hz = 1;
    if (freq_hz > TIMER_RESOLUTION_HZ) freq_hz = TIMER_RESOLUTION_HZ;

    ch->freq_hz = freq_hz;
    recalc_ticks(ch);

    /* Update timer period (takes effect at next EMPTY event) */
    ESP_ERROR_CHECK(mcpwm_timer_set_period(ch->timer, ch->period_ticks));

    /* Update comparator to maintain the same duty percentage with new period */
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(ch->comparator, ch->cmp_ticks));

    ESP_LOGI(TAG, "CH%d FREQ -> %lu Hz (period=%lu ticks)",
             idx, (unsigned long)ch->freq_hz, (unsigned long)ch->period_ticks);
}

/*
 * @brief Change the PWM duty cycle of a channel at runtime.
 *
 * Duty cycle is set by updating the comparator value. The comparator fires
 * when the timer count reaches cmp_ticks, pulling the output LOW. Since
 * update_cmp_on_tez=true was set at init, the new value is applied cleanly
 * at the start of the next period (no glitches).
 *
 * @param idx      Channel index (0 or 1).
 * @param duty_pct New duty cycle percentage [0.0, 100.0].
 */
static void pwm_set_duty(int idx, float duty_pct)
{
    pwm_channel_t *ch = &g_ch[idx];

    /* Clamp to valid range */
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    ch->duty_pct = duty_pct;
    recalc_ticks(ch);

    /* Write new compare value; hardware updates at next timer EMPTY */
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(ch->comparator, ch->cmp_ticks));

    ESP_LOGI(TAG, "CH%d DUTY -> %.2f%% (cmp=%lu/%lu ticks)",
             idx, ch->duty_pct,
             (unsigned long)ch->cmp_ticks,
             (unsigned long)ch->period_ticks);
}

/* ─────────────────────────── Status Print ──────────────────────────────── */

/*
 * @brief Print the current state of all channels to the serial monitor.
 *
 * Useful for verifying settings without an oscilloscope.
 */
static void print_status(void)
{
    printf("\n========== PWM Channel Status ==========\n");
    for (int i = 0; i < NUM_CHANNELS; i++) {
        pwm_channel_t *ch = &g_ch[i];
        printf("  CH%d  GPIO=%-3d  State=%-4s  Freq=%6lu Hz  Duty=%6.2f%%\n",
               i,
               PWM_GPIO[i],
               ch->running ? "ON" : "OFF",
               (unsigned long)ch->freq_hz,
               ch->duty_pct);
    }
    printf("=========================================\n\n");
}

/* ─────────────────────────── Command Parser ─────────────────────────────── */

/*
 * @brief Parse and execute one line of serial input.
 *
 * Accepted commands (case-insensitive channel index):
 *   ON  <ch>              – Start channel 0 or 1
 *   OFF <ch>              – Stop  channel 0 or 1
 *   FREQ <ch> <hz>        – Set frequency (Hz, integer)
 *   DUTY <ch> <percent>   – Set duty cycle (float, 0.0–100.0)
 *   STATUS                – Print status of both channels
 *
 * @param line  Null-terminated command string (already trimmed of CR/LF).
 */
static void parse_command(const char *line)
{
    char cmd[16] = {0};
    int  ch_idx  = -1;

    /* ── STATUS (no arguments) ─────────────────────────────────────────── */
    if (strncasecmp(line, "STATUS", 6) == 0) {
        print_status();
        return;
    }

    /* ── Commands with channel index ──────────────────────────────────── */
    if (sscanf(line, "%15s %d", cmd, &ch_idx) < 2) {
        printf("[ERROR] Bad syntax. Try: ON 0 | OFF 1 | FREQ 0 1000 | DUTY 1 75.0 | STATUS\n");
        return;
    }

    /* Validate channel index */
    if (ch_idx < 0 || ch_idx >= NUM_CHANNELS) {
        printf("[ERROR] Channel must be 0 or 1, got %d\n", ch_idx);
        return;
    }

    /* ── ON ────────────────────────────────────────────────────────────── */
    if (strncasecmp(cmd, "ON", 2) == 0) {
        pwm_start(ch_idx);
        return;
    }

    /* ── OFF ───────────────────────────────────────────────────────────── */
    if (strncasecmp(cmd, "OFF", 3) == 0) {
        pwm_stop(ch_idx);
        return;
    }

    /* ── FREQ <ch> <hz> ────────────────────────────────────────────────── */
    if (strncasecmp(cmd, "FREQ", 4) == 0) {
        uint32_t hz = 0;
        /* Re-scan with three fields to capture the Hz value */
        if (sscanf(line, "%*s %*d %lu", (unsigned long *)&hz) != 1 || hz == 0) {
            printf("[ERROR] Usage: FREQ <ch> <hz>  e.g. FREQ 0 5000\n");
            return;
        }
        pwm_set_freq(ch_idx, hz);
        return;
    }

    /* ── DUTY <ch> <percent> ───────────────────────────────────────────── */
    if (strncasecmp(cmd, "DUTY", 4) == 0) {
        float pct = 0.0f;
        /* Re-scan with three fields to capture the duty percentage */
        if (sscanf(line, "%*s %*d %f", &pct) != 1) {
            printf("[ERROR] Usage: DUTY <ch> <percent>  e.g. DUTY 1 33.3\n");
            return;
        }
        pwm_set_duty(ch_idx, pct);
        return;
    }

    /* ── Unknown command ───────────────────────────────────────────────── */
    printf("[ERROR] Unknown command '%s'. Valid: ON OFF FREQ DUTY STATUS\n", cmd);
}

/* ─────────────────────────── Serial Input Task ──────────────────────────── */

/*
 * @brief FreeRTOS task that reads serial input and dispatches commands.
 *
 * Reads characters from stdin one at a time (blocking). Builds a command
 * string until a newline is received, then calls parse_command().
 * The task runs indefinitely and requires ~4 KB stack.
 *
 * @param arg  Unused task argument.
 */
static void serial_task(void *arg)
{
    char buf[CMD_BUF_LEN];
    int  pos = 0;

    /* Print a startup banner with command reference */
    printf("\n");
    printf("╔══════════════════════════════════════════════════╗\n");
    printf("║        ESP32 Dual MCPWM Controller               ║\n");
    printf("║  ESP-IDF v6  |  MCPWM New API  |  115200 baud   ║\n");
    printf("╠══════════════════════════════════════════════════╣\n");
    printf("║  Commands:                                       ║\n");
    printf("║  ON  <ch>            Start PWM (ch = 0 or 1)    ║\n");
    printf("║  OFF <ch>            Stop  PWM                  ║\n");
    printf("║  FREQ <ch> <hz>      Set frequency in Hz        ║\n");
    printf("║  DUTY <ch> <pct>     Set duty cycle 0.0–100.0%% ║\n");
    printf("║  STATUS              Show all channel settings  ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");

    print_status();

    while (1) {
        /* getchar() blocks until a character is available (UART RX interrupt) */
        int c = getchar();

        if (c == EOF || c < 0) {
            /* No data yet; yield to avoid busy-spinning when using UART ISR */
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Echo the character back so the user can see what they typed */
        putchar(c);
        fflush(stdout);

        if (c == '\n' || c == '\r') {
            /* End of line – null-terminate and dispatch if non-empty */
            buf[pos] = '\0';
            if (pos > 0) {
                parse_command(buf);
                printf("> ");   /* Prompt for next command */
                fflush(stdout);
            }
            pos = 0;  /* Reset buffer for next command */
        } else if (pos < CMD_BUF_LEN - 1) {
            /* Accumulate character (ignore overflow) */
            buf[pos++] = (char)c;
        }
    }
}

/* ─────────────────────────── app_main ──────────────────────────────────── */

/*
 * @brief Application entry point.
 *
 * Initialises both MCPWM channels to their default frequency/duty, then
 * launches the serial command task. Channels start in the OFF state;
 * the user must send "ON 0" or "ON 1" to begin PWM output.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Initialising MCPWM dual-channel controller...");

    /* Initialise each channel with default settings */
    for (int i = 0; i < NUM_CHANNELS; i++) {
        g_ch[i].running   = false;
        g_ch[i].freq_hz   = DEFAULT_FREQ_HZ;
        g_ch[i].duty_pct  = DEFAULT_DUTY_PCT;

        mcpwm_channel_init(i);
    }

    ESP_LOGI(TAG, "Both channels initialised. Launching serial task...");

    /*
     * Create the serial input task on core 1 with 4 KB stack.
     * Priority 5 is above the idle task but below time-critical tasks.
     * Pinning to core 1 keeps MCPWM interrupt handling (core 0 default)
     * separate from the blocking getchar() loop.
     */
    xTaskCreatePinnedToCore(
        serial_task,    /* Task function */
        "serial_cmd",   /* Task name (visible in task list) */
        4096,           /* Stack depth in bytes */
        NULL,           /* Task argument */
        5,              /* Priority */
        NULL,           /* Task handle (not needed) */
        1               /* Pinned to core 1 */
    );
}
