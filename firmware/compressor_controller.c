#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/*
 * Marine compressor clutch controller for ATmega328PB-AU.
 *
 * Inputs
 *  - RPM_SIG       : PD2 / INT0   (conditioned flywheel tooth signal)
 *  - POT_RPM_ADJ   : PC0 / ADC0   (10k linear pot for disengage threshold)
 *  - KICKDOWN_IN   : PC1 / ADC1   (12V input reduced to MCU-safe level)
 *
 * Output
 *  - COMPRESSOR_EN : PB1          (MOSFET low-side drive enable)
 *
 * Wheel information
 *  - 66 teeth
 *  - 1 pulse per tooth
 *
 * Notes
 *  - Brown-out protection is configured in device fuses, not in firmware.
 *  - Watchdog is enabled here and must be serviced by the main loop.
 */

#define BAUD_RATE                  115200UL
#define UART_UBRR_VALUE            ((F_CPU / (16UL * BAUD_RATE)) - 1UL)

#define TOOTH_COUNT                66UL
#define TIMER1_PRESCALER           8UL
#define TIMER1_TICK_HZ             (F_CPU / TIMER1_PRESCALER)
#define RPM_TIMEOUT_MS             120UL
#define RPM_FILTER_SHIFT           2U      /* 1/4 IIR update */

#define NORMAL_ENGAGE_RPM          1700U
#define KICKDOWN_ENGAGE_RPM        1200U
#define DISENGAGE_RPM_MIN          2200U
#define DISENGAGE_RPM_MAX          2800U
#define SAFE_MAX_RPM               3000U
#define ENGAGE_HYSTERESIS_RPM      120U
#define DISENGAGE_MARGIN_RPM       60U

#define KICKDOWN_ADC_THRESHOLD     512U
#define ADC_SETTLE_DELAY_CYCLES    8U

#define COMPRESSOR_PORT            PORTB
#define COMPRESSOR_DDR             DDRB
#define COMPRESSOR_PIN             PB1

#define DEBUG_UART                 0
#define DEBUG_PRINT_INTERVAL_MS    250UL

static volatile uint32_t g_timer1_overflows = 0;
static volatile uint32_t g_last_pulse_ticks = 0;
static volatile uint32_t g_latest_period_ticks = 0;
static volatile bool g_new_period_available = false;

static uint16_t g_filtered_rpm = 0;
static bool g_compressor_enabled = false;

static inline void compressor_set(bool enable)
{
    if (enable) {
        COMPRESSOR_PORT |= (1U << COMPRESSOR_PIN);
    } else {
        COMPRESSOR_PORT &= ~(1U << COMPRESSOR_PIN);
    }

    g_compressor_enabled = enable;
}

static void gpio_init(void)
{
    /* Compressor output low-side gate drive. */
    COMPRESSOR_DDR |= (1U << COMPRESSOR_PIN);
    compressor_set(false);

    /* INT0 input for RPM signal. Keep as input; conditioning handled in hardware. */
    DDRD &= ~(1U << DDD2);

    /* ADC pins as inputs. */
    DDRC &= ~((1U << DDC0) | (1U << DDC1));
}

static void timer1_init(void)
{
    TCCR1A = 0;
    TCCR1B = (1U << CS11);   /* prescaler = 8, 2 MHz timer clock */
    TIMSK1 = (1U << TOIE1);
}

static void int0_init(void)
{
    /* Trigger on rising edge of conditioned RPM signal. */
    EICRA = (1U << ISC01) | (1U << ISC00);
    EIMSK = (1U << INT0);
}

static void adc_init(void)
{
    ADMUX = (1U << REFS0);   /* AVcc reference, right adjusted */
    ADCSRA = (1U << ADEN)
           | (1U << ADPS2)
           | (1U << ADPS1)
           | (1U << ADPS0); /* 16 MHz / 128 = 125 kHz ADC clock */
}

static uint16_t adc_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0U) | (channel & 0x0FU);

    for (uint8_t i = 0; i < ADC_SETTLE_DELAY_CYCLES; ++i) {
        __asm__ __volatile__("nop");
    }

    ADCSRA |= (1U << ADSC);
    while (ADCSRA & (1U << ADSC)) {
        /* wait */
    }

    return ADC;
}

static void watchdog_init(void)
{
    wdt_reset();
    wdt_enable(WDTO_250MS);
}

static void uart_init(void)
{
#if DEBUG_UART
    UBRR0H = (uint8_t)(UART_UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UART_UBRR_VALUE & 0xFFU);
    UCSR0A = 0;
    UCSR0B = (1U << TXEN0);
    UCSR0C = (1U << UCSZ01) | (1U << UCSZ00);
#endif
}

static void uart_write_char(char c)
{
#if DEBUG_UART
    while (!(UCSR0A & (1U << UDRE0))) {
        /* wait */
    }
    UDR0 = c;
#else
    (void)c;
#endif
}

static void uart_write_str(const char *text)
{
    while (*text != '\0') {
        uart_write_char(*text++);
    }
}

static void uart_write_u16(uint16_t value)
{
#if DEBUG_UART
    char buffer[6];
    uint8_t index = sizeof(buffer) - 1U;

    buffer[index] = '\0';
    do {
        --index;
        buffer[index] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value > 0U) && (index > 0U));

    uart_write_str(&buffer[index]);
#else
    (void)value;
#endif
}

static uint32_t timer1_now_ticks(void)
{
    uint32_t overflows;
    uint16_t counter;
    uint8_t sreg = SREG;

    cli();
    overflows = g_timer1_overflows;
    counter = TCNT1;

    if ((TIFR1 & (1U << TOV1)) && (counter < 65535U)) {
        overflows++;
        counter = TCNT1;
    }
    SREG = sreg;

    return (overflows << 16) | counter;
}

static uint16_t map_pot_to_disengage_rpm(uint16_t adc_value)
{
    uint32_t span = (uint32_t)(DISENGAGE_RPM_MAX - DISENGAGE_RPM_MIN);
    uint32_t scaled = (uint32_t)adc_value * span;
    return (uint16_t)(DISENGAGE_RPM_MIN + (scaled / 1023UL));
}

static bool kickdown_active_from_adc(uint16_t adc_value)
{
    return (adc_value >= KICKDOWN_ADC_THRESHOLD);
}

static uint16_t calculate_instant_rpm(uint32_t period_ticks)
{
    if (period_ticks == 0UL) {
        return 0U;
    }

    /* RPM = timer_hz * 60 / (ticks_per_tooth * teeth_per_rev) */
    uint32_t numerator = TIMER1_TICK_HZ * 60UL;
    uint32_t rpm = numerator / (period_ticks * TOOTH_COUNT);

    if (rpm > 65535UL) {
        rpm = 65535UL;
    }

    return (uint16_t)rpm;
}

static uint16_t update_filtered_rpm(void)
{
    uint32_t period_ticks = 0UL;
    uint32_t last_pulse_ticks = 0UL;
    bool new_period = false;
    uint8_t sreg = SREG;

    cli();
    if (g_new_period_available) {
        period_ticks = g_latest_period_ticks;
        g_new_period_available = false;
        new_period = true;
    }
    last_pulse_ticks = g_last_pulse_ticks;
    SREG = sreg;

    if (new_period) {
        uint16_t instant_rpm = calculate_instant_rpm(period_ticks);

        if (g_filtered_rpm == 0U) {
            g_filtered_rpm = instant_rpm;
        } else {
            int32_t delta = (int32_t)instant_rpm - (int32_t)g_filtered_rpm;
            g_filtered_rpm = (uint16_t)((int32_t)g_filtered_rpm + (delta >> RPM_FILTER_SHIFT));
        }
    }

    uint32_t now_ticks = timer1_now_ticks();
    uint32_t ticks_since_pulse = now_ticks - last_pulse_ticks;
    uint32_t timeout_ticks = (TIMER1_TICK_HZ / 1000UL) * RPM_TIMEOUT_MS;

    if (ticks_since_pulse > timeout_ticks) {
        g_filtered_rpm = 0U;
    }

    return g_filtered_rpm;
}

static void control_compressor(uint16_t rpm, uint16_t disengage_rpm, bool kickdown_active)
{
    uint16_t engage_rpm = kickdown_active ? KICKDOWN_ENGAGE_RPM : NORMAL_ENGAGE_RPM;
    uint16_t release_below_rpm = (engage_rpm > ENGAGE_HYSTERESIS_RPM)
                               ? (uint16_t)(engage_rpm - ENGAGE_HYSTERESIS_RPM)
                               : 0U;
    uint16_t upper_off_rpm = disengage_rpm;

    if (upper_off_rpm > SAFE_MAX_RPM) {
        upper_off_rpm = SAFE_MAX_RPM;
    }

    if (g_compressor_enabled) {
        if ((rpm >= SAFE_MAX_RPM)
            || (rpm >= upper_off_rpm)
            || (rpm < release_below_rpm)) {
            compressor_set(false);
        }
        return;
    }

    if ((rpm >= engage_rpm)
        && (rpm < SAFE_MAX_RPM)
        && (rpm + DISENGAGE_MARGIN_RPM < upper_off_rpm)) {
        compressor_set(true);
    }
}

ISR(TIMER1_OVF_vect)
{
    g_timer1_overflows++;
}

ISR(INT0_vect)
{
    uint32_t overflows = g_timer1_overflows;
    uint16_t counter = TCNT1;

    if ((TIFR1 & (1U << TOV1)) && (counter < 65535U)) {
        overflows++;
        counter = TCNT1;
    }

    uint32_t timestamp = (overflows << 16) | counter;
    uint32_t period = timestamp - g_last_pulse_ticks;

    g_last_pulse_ticks = timestamp;

    /* Ignore the first edge after reset; it does not have a valid period yet. */
    if (period > 0UL) {
        g_latest_period_ticks = period;
        g_new_period_available = true;
    }
}

int main(void)
{
    uint32_t debug_timer_ms = 0UL;

    gpio_init();
    timer1_init();
    adc_init();
    int0_init();
    uart_init();
    watchdog_init();

    sei();

    while (1) {
        uint16_t pot_raw = adc_read(0U);
        uint16_t kickdown_raw = adc_read(1U);
        bool kickdown_active = kickdown_active_from_adc(kickdown_raw);
        uint16_t disengage_rpm = map_pot_to_disengage_rpm(pot_raw);
        uint16_t rpm = update_filtered_rpm();

        control_compressor(rpm, disengage_rpm, kickdown_active);

#if DEBUG_UART
        uint32_t now_ms = timer1_now_ticks() / (TIMER1_TICK_HZ / 1000UL);
        if ((now_ms - debug_timer_ms) >= DEBUG_PRINT_INTERVAL_MS) {
            debug_timer_ms = now_ms;
            uart_write_str("RPM=");
            uart_write_u16(rpm);
            uart_write_str(" POT=");
            uart_write_u16(pot_raw);
            uart_write_str(" DIS=");
            uart_write_u16(disengage_rpm);
            uart_write_str(" KD=");
            uart_write_u16(kickdown_active ? 1U : 0U);
            uart_write_str(" OUT=");
            uart_write_u16(g_compressor_enabled ? 1U : 0U);
            uart_write_str("\r\n");
        }
#else
        (void)debug_timer_ms;
#endif

        wdt_reset();
    }
}
