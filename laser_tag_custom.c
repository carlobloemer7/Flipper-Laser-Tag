/*
 * Laser Tag Custom - Flipper Zero App
 *
 * Sendet eigene RAW-IR-Signale (Team-Schuesse) aus /ext/infrared/lasertek.ir
 * Empfaengt Treffer wahlweise ueber:
 *   a) externen TSOP38238 an GPIO PA4
 *   b) eingebauten IR-Empfaenger (furi_hal_infrared async rx)
 * Umschaltbar per UP-Taste.
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_infrared.h>
#include <furi_hal_gpio.h>
#include <furi_hal_resources.h>
#include <furi_hal_light.h>
#include <furi_hal_vibro.h>
#include <furi_hal_cortex.h>

#include <gui/gui.h>
#include <gui/view_port.h>
#include <input/input.h>
#include <storage/storage.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define TAG "LaserTagCustom"

#define IR_FILE_PATH "/ext/infrared/lasertek.ir"

#define MAX_TEAMS 8
#define MAX_TIMINGS 128

#define STARTING_LIVES 6
#define MAX_AMMO 6
#define SHOT_COOLDOWN_MS 1000

#define RX_TOLERANCE_PERCENT 10
#define RX_FRAME_TIMEOUT_US 8000
#define RX_TIMER_PERIOD_MS 20
#define RX_INTERNAL_TIMEOUT_US 200000

// Externer TSOP38238 an PA4 (EXTI4 - frei von System-Diensten)
#define IR_RX_GPIO (&gpio_ext_pa4)

/* ---------------------------------------------------------------------- */
/* Datentypen                                                              */
/* ---------------------------------------------------------------------- */

typedef struct {
    char name[32];
    uint32_t timings[MAX_TIMINGS];
    size_t timings_count;
    uint32_t frequency;
    float duty_cycle;
} IrSignal;

typedef enum {
    LaserTagEventTypeInput,
    LaserTagEventTypeHit,
} LaserTagEventType;

typedef enum {
    RxSourceExternalTsop,
    RxSourceInternalIr,
} RxSource;

typedef struct {
    LaserTagEventType type;
    InputEvent input;
} LaserTagEvent;

typedef struct {
    FuriMessageQueue* event_queue;
    FuriMutex* mutex;

    ViewPort* view_port;
    Gui* gui;

    IrSignal signals[MAX_TEAMS];
    size_t signal_count;
    size_t selected_signal;

    int32_t lives;
    int32_t ammo;
    uint32_t last_shot_tick;
    bool game_over;

    float ticks_per_us;

    RxSource rx_source;
    bool rx_active;

    volatile uint32_t rx_edge_times[MAX_TIMINGS];
    volatile size_t rx_edge_count;
    volatile uint32_t rx_last_edge_tick;
} LaserTagApp;

/* ---------------------------------------------------------------------- */
/* Forward-Deklarationen                                                   */
/* ---------------------------------------------------------------------- */

static void rx_start_internal(LaserTagApp* app);
static void rx_stop_internal(void);
static void rx_start_external(LaserTagApp* app);
static void rx_stop_external(void);

/* ---------------------------------------------------------------------- */
/* Hilfsfunktionen                                                         */
/* ---------------------------------------------------------------------- */

static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static bool timings_match(
    const uint32_t* measured,
    size_t measured_count,
    const uint32_t* reference,
    size_t reference_count) {
    if(measured_count != reference_count || reference_count == 0) {
        return false;
    }
    for(size_t i = 0; i < reference_count; i++) {
        uint32_t diff =
            (measured[i] > reference[i]) ? (measured[i] - reference[i]) : (reference[i] - measured[i]);
        uint32_t tolerance = (reference[i] * RX_TOLERANCE_PERCENT) / 100;
        if(tolerance == 0) tolerance = 1;
        if(diff > tolerance) {
            return false;
        }
    }
    return true;
}

/* ---------------------------------------------------------------------- */
/* Datei laden                                                             */
/* ---------------------------------------------------------------------- */

static bool load_signals_from_file(LaserTagApp* app, const char* path) {
    FURI_LOG_I(TAG, "Opening file: %s", path);
    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);
    bool success = false;

    do {
        if(!storage_file_open(file, path, FSAM_READ, FSOM_OPEN_EXISTING)) {
            FURI_LOG_E(TAG, "Cannot open %s", path);
            break;
        }

        uint64_t size = storage_file_size(file);
        if(size == 0 || size > 32768) {
            FURI_LOG_E(TAG, "File size invalid: %lu", (unsigned long)size);
            break;
        }

        char* buffer = malloc((size_t)size + 1);
        if(!buffer) break;

        uint16_t read = storage_file_read(file, buffer, (uint16_t)size);
        buffer[read] = '\0';

        char* line_save = NULL;
        char* line = strtok_r(buffer, "\n", &line_save);
        IrSignal* current = NULL;

        while(line) {
            size_t len = strlen(line);
            if(len > 0 && line[len - 1] == '\r') {
                line[len - 1] = '\0';
            }

            if(strncmp(line, "name:", 5) == 0) {
                if(app->signal_count < MAX_TEAMS) {
                    current = &app->signals[app->signal_count++];
                    memset(current, 0, sizeof(IrSignal));
                    char* value = line + 5;
                    while(*value == ' ') value++;
                    strncpy(current->name, value, sizeof(current->name) - 1);
                    current->frequency = 38000;
                    current->duty_cycle = 0.33f;
                } else {
                    current = NULL;
                }
            } else if(current && strncmp(line, "frequency:", 10) == 0) {
                current->frequency = (uint32_t)strtoul(line + 10, NULL, 10);
            } else if(current && strncmp(line, "duty_cycle:", 11) == 0) {
                current->duty_cycle = strtof(line + 11, NULL);
            } else if(current && strncmp(line, "data:", 5) == 0) {
                char* value = line + 5;
                while(*value == ' ') value++;

                char* tok_save = NULL;
                char* token = strtok_r(value, " \t", &tok_save);
                size_t count = 0;
                while(token && count < MAX_TIMINGS) {
                    current->timings[count++] = (uint32_t)strtoul(token, NULL, 10);
                    token = strtok_r(NULL, " \t", &tok_save);
                }
                current->timings_count = count;
            }

            line = strtok_r(NULL, "\n", &line_save);
        }

        free(buffer);
        success = app->signal_count > 0;
        FURI_LOG_I(TAG, "Loaded %u signals", (unsigned)app->signal_count);
    } while(false);

    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);

    return success;
}

/* ---------------------------------------------------------------------- */
/* IR senden                                                               */
/* ---------------------------------------------------------------------- */

typedef struct {
    const uint32_t* timings;
    size_t count;
    size_t index;
    bool level;
} IrTxContext;

static FuriHalInfraredTxGetDataState ir_tx_fill_data(void* context, uint32_t* duration, bool* level) {
    IrTxContext* ctx = context;

    if(ctx->index >= ctx->count) {
        *duration = 0;
        *level = false;
        return FuriHalInfraredTxGetDataStateDone;
    }

    *duration = ctx->timings[ctx->index];
    *level = ctx->level;
    ctx->level = !ctx->level;
    ctx->index++;

    if(ctx->index >= ctx->count) {
        return FuriHalInfraredTxGetDataStateLastDone;
    }
    return FuriHalInfraredTxGetDataStateOk;
}

static void send_ir_signal(LaserTagApp* app, IrSignal* signal) {
    if(signal->timings_count == 0) return;

    FURI_LOG_I(TAG, "TX: sending %s (%u timings)", signal->name, (unsigned)signal->timings_count);

    bool was_internal_rx = (app->rx_source == RxSourceInternalIr && app->rx_active);
    if(was_internal_rx) {
        rx_stop_internal();
        app->rx_active = false;
    }

    IrTxContext ctx = {
        .timings = signal->timings,
        .count = signal->timings_count,
        .index = 0,
        .level = true,
    };

    // Callback MUSS vor start gesetzt werden
    furi_hal_infrared_async_tx_set_data_isr_callback(ir_tx_fill_data, &ctx);
    furi_hal_infrared_async_tx_start(signal->frequency, signal->duty_cycle);
    furi_hal_infrared_async_tx_wait_termination();

    FURI_LOG_I(TAG, "TX: done");

    if(was_internal_rx) {
        furi_delay_ms(5);
        app->rx_edge_count = 0;
        app->rx_last_edge_tick = DWT->CYCCNT;
        rx_start_internal(app);
        app->rx_active = true;
    }
}

/* ---------------------------------------------------------------------- */
/* IR empfangen                                                            */
/* ---------------------------------------------------------------------- */

static void ir_rx_isr_external(void* context) {
    LaserTagApp* app = context;

    uint32_t now = DWT->CYCCNT;
    uint32_t delta_ticks = now - app->rx_last_edge_tick;
    app->rx_last_edge_tick = now;

    uint32_t delta_us = (uint32_t)((float)delta_ticks / app->ticks_per_us);

    if(app->rx_edge_count < MAX_TIMINGS) {
        app->rx_edge_times[app->rx_edge_count++] = delta_us;
    }
}

static void ir_rx_isr_internal(void* context, bool level, uint32_t duration) {
    UNUSED(level);
    LaserTagApp* app = context;

    app->rx_last_edge_tick = DWT->CYCCNT;

    if(app->rx_edge_count < MAX_TIMINGS) {
        app->rx_edge_times[app->rx_edge_count++] = duration;
    }
}

static void ir_rx_timeout_isr_internal(void* context) {
    UNUSED(context);
}

static void rx_start_external(LaserTagApp* app) {
    FURI_LOG_I(TAG, "Starting external RX on PA4");
    furi_hal_gpio_init(IR_RX_GPIO, GpioModeInterruptRiseFall, GpioPullUp, GpioSpeedVeryHigh);
    furi_hal_gpio_add_int_callback(IR_RX_GPIO, ir_rx_isr_external, app);
    FURI_LOG_I(TAG, "External RX started");
}

static void rx_stop_external(void) {
    furi_hal_gpio_remove_int_callback(IR_RX_GPIO);
    furi_hal_gpio_init(IR_RX_GPIO, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
}

static void rx_start_internal(LaserTagApp* app) {
    FURI_LOG_I(TAG, "Starting internal RX");
    furi_hal_infrared_async_rx_set_capture_isr_callback(ir_rx_isr_internal, app);
    furi_hal_infrared_async_rx_set_timeout_isr_callback(ir_rx_timeout_isr_internal, app);
    furi_hal_infrared_async_rx_set_timeout(RX_INTERNAL_TIMEOUT_US);
    furi_hal_infrared_async_rx_start();
    FURI_LOG_I(TAG, "Internal RX started");
}

static void rx_stop_internal(void) {
    furi_hal_infrared_async_rx_stop();
}

static void rx_stop_current(LaserTagApp* app) {
    if(!app->rx_active) return;
    if(app->rx_source == RxSourceExternalTsop) {
        rx_stop_external();
    } else {
        rx_stop_internal();
    }
    app->rx_active = false;
}

static void rx_start_current(LaserTagApp* app) {
    app->rx_edge_count = 0;
    app->rx_last_edge_tick = DWT->CYCCNT;
    if(app->rx_source == RxSourceExternalTsop) {
        rx_start_external(app);
    } else {
        rx_start_internal(app);
    }
    app->rx_active = true;
}

static void rx_switch_source(LaserTagApp* app, RxSource new_source) {
    if(app->rx_source == new_source) return;
    rx_stop_current(app);
    app->rx_source = new_source;
    rx_start_current(app);
}

static void rx_check_timer_callback(void* context) {
    LaserTagApp* app = context;

    if(!app->rx_active) return;

    uint32_t idle_ticks;
    size_t count;

    {
        FURI_CRITICAL_ENTER();
        uint32_t now = DWT->CYCCNT;
        idle_ticks = now - app->rx_last_edge_tick;
        count = app->rx_edge_count;
        FURI_CRITICAL_EXIT();
    }

    uint32_t idle_us = (uint32_t)((float)idle_ticks / app->ticks_per_us);

    if(count == 0 || idle_us < RX_FRAME_TIMEOUT_US) {
        return;
    }

    uint32_t frame_us[MAX_TIMINGS];
    size_t frame_count;

    {
        FURI_CRITICAL_ENTER();
        frame_count = app->rx_edge_count;
        for(size_t i = 0; i < frame_count; i++) {
            frame_us[i] = app->rx_edge_times[i];
        }
        app->rx_edge_count = 0;
        FURI_CRITICAL_EXIT();
    }

    if(frame_count < 2) return;

    uint32_t* measured = &frame_us[1];
    size_t measured_count = frame_count - 1;

    for(size_t s = 0; s < app->signal_count; s++) {
        if(timings_match(
               measured, measured_count, app->signals[s].timings, app->signals[s].timings_count)) {
            LaserTagEvent event = {.type = LaserTagEventTypeHit};
            furi_message_queue_put(app->event_queue, &event, 0);
            break;
        }
    }
}

/* ---------------------------------------------------------------------- */
/* GUI                                                                     */
/* ---------------------------------------------------------------------- */

static void draw_callback(Canvas* canvas, void* context) {
    LaserTagApp* app = context;
    furi_mutex_acquire(app->mutex, FuriWaitForever);

    canvas_clear(canvas);

    if(app->game_over) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 24, 30, "GAME OVER");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 8, 46, "BACK = neu starten");
    } else {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 2, 12, "Laser Tag");

        canvas_set_font(canvas, FontSecondary);

        char buf[40];
        snprintf(buf, sizeof(buf), "Leben:%ld/%d Muni:%ld/%d",
            (long)app->lives, STARTING_LIVES, (long)app->ammo, MAX_AMMO);
        canvas_draw_str(canvas, 2, 24, buf);

        if(app->signal_count > 0) {
            snprintf(buf, sizeof(buf), "Team: %s",
                app->signals[app->selected_signal].name);
        } else {
            snprintf(buf, sizeof(buf), "Keine .ir Datei!");
        }
        canvas_draw_str(canvas, 2, 34, buf);

        canvas_draw_str(canvas, 2, 44,
            app->rx_source == RxSourceExternalTsop ?
                "Empf: Extern (TSOP PA4)" : "Empf: Intern (eingebaut)");

        canvas_draw_str(canvas, 2, 54, "OK=Schuss </>Team");
        canvas_draw_str(canvas, 2, 62, "DOWN=Reload UP=Empf.");

        int32_t lc = app->lives < 0 ? 0 : app->lives;
        for(int32_t i = 0; i < STARTING_LIVES; i++) {
            int32_t x = 88 + i * 6;
            if(i < lc) {
                canvas_draw_box(canvas, x, 18, 5, 5);
            } else {
                canvas_draw_frame(canvas, x, 18, 5, 5);
            }
        }
    }

    furi_mutex_release(app->mutex);
}

static void input_callback(InputEvent* input_event, void* context) {
    FuriMessageQueue* queue = context;
    LaserTagEvent event = {.type = LaserTagEventTypeInput, .input = *input_event};
    furi_message_queue_put(queue, &event, 0);
}

/* ---------------------------------------------------------------------- */
/* Entry point                                                             */
/* ---------------------------------------------------------------------- */

int32_t laser_tag_custom_app(void* p) {
    UNUSED(p);
    FURI_LOG_I(TAG, "App starting");

    LaserTagApp* app = malloc(sizeof(LaserTagApp));
    memset(app, 0, sizeof(LaserTagApp));
    FURI_LOG_I(TAG, "App struct allocated");

    app->event_queue = furi_message_queue_alloc(8, sizeof(LaserTagEvent));
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);

    app->lives = STARTING_LIVES;
    app->ammo = MAX_AMMO;
    app->game_over = false;
    app->rx_active = false;

    FURI_LOG_I(TAG, "Initializing DWT");
    dwt_init();
    app->ticks_per_us = furi_hal_cortex_instructions_per_microsecond();
    app->rx_last_edge_tick = DWT->CYCCNT;
    FURI_LOG_I(TAG, "DWT initialized, ticks/us=%ld", (long)app->ticks_per_us);

    // Default: interner Empfaenger (kein GPIO, kein EXTI-Konflikt moeglich)
    app->rx_source = RxSourceInternalIr;

    FURI_LOG_I(TAG, "Setting up GUI");
    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, draw_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app->event_queue);

    app->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);
    FURI_LOG_I(TAG, "GUI ready");

    FURI_LOG_I(TAG, "Loading IR signals");
    if(!load_signals_from_file(app, IR_FILE_PATH)) {
        FURI_LOG_W(TAG, "No signals loaded from %s", IR_FILE_PATH);
    }

    // Starte RX NACH vollstaendiger Initialisierung
    FURI_LOG_I(TAG, "Starting RX (internal)");
    rx_start_current(app);
    FURI_LOG_I(TAG, "RX active");

    FuriTimer* rx_timer = furi_timer_alloc(rx_check_timer_callback, FuriTimerTypePeriodic, app);
    furi_timer_start(rx_timer, furi_ms_to_ticks(RX_TIMER_PERIOD_MS));
    FURI_LOG_I(TAG, "Timer started, entering main loop");

    LaserTagEvent event;
    bool running = true;

    while(running) {
        FuriStatus status = furi_message_queue_get(app->event_queue, &event, FuriWaitForever);
        if(status != FuriStatusOk) continue;

        furi_mutex_acquire(app->mutex, FuriWaitForever);

        if(event.type == LaserTagEventTypeInput) {
            if(event.input.type == InputTypeShort || event.input.type == InputTypeRepeat) {
                switch(event.input.key) {
                case InputKeyOk:
                    if(!app->game_over && app->signal_count > 0 && app->ammo > 0) {
                        uint32_t now = furi_get_tick();
                        uint32_t elapsed = now - app->last_shot_tick;
                        if(elapsed >= furi_ms_to_ticks(SHOT_COOLDOWN_MS)) {
                            send_ir_signal(app, &app->signals[app->selected_signal]);
                            app->ammo--;
                            app->last_shot_tick = now;
                        }
                    }
                    break;

                case InputKeyUp:
                    if(!app->game_over) {
                        RxSource next = (app->rx_source == RxSourceExternalTsop) ?
                            RxSourceInternalIr : RxSourceExternalTsop;
                        rx_switch_source(app, next);
                    }
                    break;

                case InputKeyDown:
                    if(!app->game_over) {
                        app->ammo = MAX_AMMO;
                    }
                    break;

                case InputKeyRight:
                    if(app->signal_count > 0) {
                        app->selected_signal = (app->selected_signal + 1) % app->signal_count;
                    }
                    break;

                case InputKeyLeft:
                    if(app->signal_count > 0) {
                        app->selected_signal =
                            (app->selected_signal + app->signal_count - 1) % app->signal_count;
                    }
                    break;

                case InputKeyBack:
                    if(app->game_over) {
                        app->lives = STARTING_LIVES;
                        app->ammo = MAX_AMMO;
                        app->last_shot_tick = 0;
                        app->game_over = false;
                    } else {
                        running = false;
                    }
                    break;

                default:
                    break;
                }
            }
        } else if(event.type == LaserTagEventTypeHit) {
            if(!app->game_over) {
                app->lives--;

                furi_hal_vibro_on(true);
                furi_hal_light_set(LightRed, 255);

                furi_mutex_release(app->mutex);
                furi_delay_ms(150);
                furi_mutex_acquire(app->mutex, FuriWaitForever);

                furi_hal_vibro_on(false);
                furi_hal_light_set(LightRed, 0);

                if(app->lives <= 0) {
                    app->lives = 0;
                    app->game_over = true;
                }
            }
        }

        furi_mutex_release(app->mutex);
        view_port_update(app->view_port);
    }

    FURI_LOG_I(TAG, "Shutting down");
    furi_timer_stop(rx_timer);
    furi_timer_free(rx_timer);

    rx_stop_current(app);

    gui_remove_view_port(app->gui, app->view_port);
    furi_record_close(RECORD_GUI);
    view_port_free(app->view_port);

    furi_message_queue_free(app->event_queue);
    furi_mutex_free(app->mutex);

    free(app);
    FURI_LOG_I(TAG, "App exited cleanly");
    return 0;
}
