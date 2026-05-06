/**
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

/** Tag used in all ESP_LOG* calls for easy filtering in the monitor. */
static const char *TAG = "MCPWM_CTRL";

/** Number of independent PWM channels managed by this application. */
#define NUM_CHANNELS 2

/**
 * @brief GPIO pins for each PWM output.
 *
 * Change these to any free GPIO on your board.
 * Avoid strapping pins (0, 2, 12, 15) and input-only pins (34-39).
 */
static const int PWM_GPIO[NUM_CHANNELS] = {
    18,   /* Channel 0 */
    19    /* Channel 1 */
};

/**
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

/**
 * @brief Timer resolution in ticks-per-second (Hz).
 *
 * Higher values give finer duty-cycle resolution but limit the maximum period
 * (period_ticks must fit in a 24-bit counter, so max period = 2^24 ticks).
 * 1 MHz → period granularity = 1 µs, supports frequencies down to ~0.06 Hz.
 */
#define TIMER_RESOLUTION_HZ  1000000UL   /* 1 MHz tick rate */

/** Default frequency for both channels at startup (Hz). */
#define DEFAULT_FREQ_HZ      1000U

/** Default duty cycle for both channels at startup (%). */
#define DEFAULT_DUTY_PCT     50.0f

/** Maximum line length accepted from the serial input buffer. */
#define CMD_BUF_LEN  64

/* ─────────────────────────── Channel State ─────────────────────────────── */

/**
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

/** Array of both channels. Indices match CH0/CH1 throughout. */
static pwm_channel_t g_ch[NUM_CHANNELS];

/* ─────────────────────────── Helper: Recalculate Ticks ─────────────────── */

/**
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

/**
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

/**
 * @brief Start (resume) PWM output on the specified channel.
 *
 * Sends MCPWM_TIMER_START_NO_STOP which runs the timer continuously.
 * The generator will immediately