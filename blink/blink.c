#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 100
#endif

// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}
///////

// Begin user config section ---------------------------

const int THROTTLE_LOW = 600;               // ADC value corresponding to minimum throttle, 0-4095
const int THROTTLE_HIGH = 2650;             // ADC value corresponding to maximum throttle, 0-4095

const bool CURRENT_CONTROL = false;          // Use current control or duty cycle control
const int PHASE_MAX_CURRENT_MA = 6000;      // If using current control, the maximum phase current allowed
const int BATTERY_MAX_CURRENT_MA = 3000;    // If using current control, the maximum battery current allowed
const int CURRENT_CONTROL_LOOP_GAIN = 200;  // Adjusts the speed of the current control loop

//Comutation Orders

// ABC          
// A LOW, B HIGH 
// A LOW, C HIGH
// B LOW, C HIGH
// B LOW, A HIGH
// C LOW, A HIGH
// C LOW, B HIGH

// End user config section -----------------------------

const uint LED_PIN = 25;
const uint ENCODER_OVERSAMPLE = 6;

const uint AH_PIN = 16;
const uint AL_PIN = 17;
const uint BH_PIN = 18;
const uint BL_PIN = 19;
const uint CH_PIN = 20;
const uint CL_PIN = 21;

// Optical Encoder Pin Definitions
// Encoder 1
const uint OPT_U1_PIN = 10;
const uint OPT_V1_PIN = 11;
const uint OPT_W1_PIN = 12;

// Encoder 2
const uint OPT_U2_PIN = 13;
const uint OPT_V2_PIN = 14;
const uint OPT_W2_PIN = 15;

const uint ISENSE_PIN = 26;
const uint VSENSE_PIN = 27;
const uint THROTTLE_PIN = 28;

const uint A_PWM_SLICE = 0;
const uint B_PWM_SLICE = 1;
const uint C_PWM_SLICE = 2;

const uint F_PWM = 16000;   // Desired PWM frequency
const uint FLAG_PIN = 2;

const int DUTY_CYCLE_MAX = 65535;
const int CURRENT_SCALING = 3.3 / 0.0005 / 20 / 4096 * 1000;
const int VOLTAGE_SCALING = 3.3 / 4096 * (47 + 2.2) / 2.2 * 1000;
const int ADC_BIAS_OVERSAMPLE = 1000;

int adc_isense = 0;
int adc_vsense = 0;
int adc_throttle = 0;

int adc_bias = 0;
int duty_cycle = 0;
int voltage_mv = 0;
int current_ma = 0;
int current_target_ma = 0;
int optical = 0;
uint motorState = 0;
int fifo_level = 0;
uint64_t ticks_since_init = 0;

uint get_optical_state();
void writePWM(uint motorState, uint duty, bool synchronous);
uint8_t read_throttle();

void on_adc_fifo() {
    // This interrupt is where the magic happens. This is fired once the ADC conversions have finished (roughly 6us for 3 conversions)
    // This reads the optical sensors, determines the motor state to switch to, and reads the current sensors and throttle to
    // determine the desired duty cycle. This takes ~7us to complete.

    uint32_t flags = save_and_disable_interrupts(); // Disable interrupts for the time-critical reading ADC section. USB interrupts may interfere

    adc_run(false);             // Stop the ADC from free running
    gpio_put(FLAG_PIN, 1);      // For debugging, toggle the flag pin

    fifo_level = adc_fifo_get_level();
    adc_isense = adc_fifo_get();    // Read the ADC values into the registers
    adc_vsense = adc_fifo_get();

    adc_throttle = 1000; // temp value
    // adc_throttle = adc_fifo_get();

    restore_interrupts(flags);      // Re-enable interrupts

    if(fifo_level != 3) {
        // The RP2040 is a shitty microcontroller. The ADC is unpredictable, and 1% of times
        // will return more or less than the 3 samples it should. If we don't get the expected number, abort
        return;
    }

    motorState = get_optical_state();                 // Read the optical sensors

    int throttle = ((adc_throttle - THROTTLE_LOW) * 256) / (THROTTLE_HIGH - THROTTLE_LOW);  // Scale the throttle value read from the ADC
    throttle = MAX(0, MIN(255, throttle));      // Clamp to 0-255

    current_ma = (adc_isense - adc_bias) * CURRENT_SCALING;     // Since the current sensor is bidirectional, subtract the zero-current value and scale
    voltage_mv = adc_vsense * VOLTAGE_SCALING;  // Calculate the bus voltage

    if(CURRENT_CONTROL) {
        int user_current_target_ma = throttle * PHASE_MAX_CURRENT_MA / 256;  // Calculate the user-demanded phase current
        int battery_current_limit_ma = BATTERY_MAX_CURRENT_MA * DUTY_CYCLE_MAX / duty_cycle;  // Calculate the maximum phase current allowed while respecting the battery current limit
        current_target_ma = MIN(user_current_target_ma, battery_current_limit_ma);

        if (throttle == 0)
        {
            duty_cycle = 0;     // If zero throttle, ignore the current control loop and turn all transistors off
            ticks_since_init = 0;   // Reset the timer since the transistors were turned on
        }
        else
            ticks_since_init++;

        duty_cycle += (current_target_ma - current_ma) / CURRENT_CONTROL_LOOP_GAIN;  // Perform a simple integral controller to adjust the duty cycle
        duty_cycle = MAX(0, MIN(DUTY_CYCLE_MAX, duty_cycle));   // Clamp the duty cycle

        bool do_synchronous = ticks_since_init > 16000;    // Only enable synchronous switching some time after beginning control loop. This allows control loop to stabilize
        writePWM(motorState, (uint)(duty_cycle / 256), do_synchronous);
    }
    else {
        duty_cycle = throttle * 256;    // Set duty cycle based directly on throttle
        bool do_synchronous = true;     // Note, if doing synchronous duty-cycle control, the motor will regen if the throttle decreases. This may regen VERY HARD

        writePWM(motorState, (uint)(duty_cycle / 256), do_synchronous);
    }

    gpio_put(FLAG_PIN, 0);
}

void on_pwm_wrap() {
    // This interrupt is triggered when the A_PWM slice reaches 0 (the middle of the PWM cycle)
    // This allows us to start ADC conversions while the high-side FETs are on, which is required
    // to read current, based on where the current sensor is placed in the schematic.
    // Takes ~1.3 microseconds

    gpio_put(FLAG_PIN, 1);      // Toggle the flag pin high for debugging
    adc_select_input(0);        // Force the ADC to start with input 0
    adc_run(true);              // Start the ADC
    pwm_clear_irq(A_PWM_SLICE); // Clear this interrupt flag
    while(!adc_fifo_is_empty()) // Clear out the ADC fifo, in case it still has samples in it
        adc_fifo_get();

    gpio_put(FLAG_PIN, 0);
}

void writePhases(uint ah, uint bh, uint ch, uint al, uint bl, uint cl)
{
    // Set the timer registers for each PWM slice. The lowside values are inverted,
    // since the PWM slices were already configured to invert the lowside pin.
    // ah: desired high-side duty cycle, range of 0-255
    // al: desired low-side duty cycle, range of 0-255

    pwm_set_both_levels(A_PWM_SLICE, ah, 255 - al);
    pwm_set_both_levels(B_PWM_SLICE, bh, 255 - bl);
    pwm_set_both_levels(C_PWM_SLICE, ch, 255 - cl);
}

void writePWM(uint motorState, uint duty, bool synchronous)
{
    // Switch the transistors given a desired electrical state and duty cycle
    // motorState: desired electrical position, range of 0-5
    // duty: desired duty cycle, range of 0-255
    // synchronous: perfom synchronous (low-side and high-side alternating) or non-synchronous switching (high-side only) 

    if(duty == 0 || duty > 255)     // If zero throttle, turn both low-sides and high-sides off
        motorState = 255;

    // At near 100% duty cycles, the gate driver bootstrap capacitor may become discharged as the high-side gate is repeatedly driven
    // high and low without time for the phase voltage to fall to zero and charge the bootstrap capacitor. Thus, if duty cycle is near
    // 100%, clamp it to 100%.
    if(duty > 245)
        duty = 255;

    uint complement = 0;
    if(synchronous)
    {
        complement = MAX(0, 248 - (int)duty);    // Provide switching deadtime by having duty + complement < 255
    }

    if(motorState == 0)                         // LOW A, HIGH B
        writePhases(0, duty, 0, 255, complement, 0);
    else if(motorState == 1)                    // LOW A, HIGH C
        writePhases(0, 0, duty, 255, 0, complement);
    else if(motorState == 2)                    // LOW B, HIGH C
        writePhases(0, 0, duty, 0, 255, complement);
    else if(motorState == 3)                    // LOW B, HIGH A
        writePhases(duty, 0, 0, complement, 255, 0);
    else if(motorState == 4)                    // LOW C, HIGH A
        writePhases(duty, 0, 0, complement, 0, 255);
    else if(motorState == 5)                    // LOW C, HIGH B
        writePhases(0, duty, 0, 0, complement, 255);
    else                                        // All transistors off
        writePhases(0, 0, 0, 0, 0, 0);
}

void init_hardware() {
    // Initialize all peripherals

    stdio_init_all();

    // Set LED and FLAG pin as outputs
    gpio_init(LED_PIN);     
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(FLAG_PIN);
    gpio_set_dir(FLAG_PIN, GPIO_OUT);

    // Set up optical sensor 1 pins
    gpio_init(OPT_U1_PIN);
    gpio_set_dir(OPT_U1_PIN, GPIO_IN);
    gpio_init(OPT_V1_PIN);
    gpio_set_dir(OPT_V1_PIN, GPIO_IN);
    gpio_init(OPT_W1_PIN);
    gpio_set_dir(OPT_W1_PIN, GPIO_IN);

    // Set up optical sensor 2 pins
    gpio_init(OPT_U2_PIN);  
    gpio_set_dir(OPT_U2_PIN, GPIO_IN);
    gpio_init(OPT_V2_PIN);
    gpio_set_dir(OPT_V2_PIN, GPIO_IN);
    gpio_init(OPT_W2_PIN);
    gpio_set_dir(OPT_W2_PIN, GPIO_IN);

    // Set gate control pins as output
    gpio_set_function(AH_PIN, GPIO_FUNC_PWM);   
    gpio_set_function(AL_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BH_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BL_PIN, GPIO_FUNC_PWM);
    gpio_set_function(CH_PIN, GPIO_FUNC_PWM);
    gpio_set_function(CL_PIN, GPIO_FUNC_PWM);

    adc_init();
    adc_gpio_init(ISENSE_PIN);  // Set up ADC pins
    adc_gpio_init(VSENSE_PIN);
    adc_gpio_init(THROTTLE_PIN);

    sleep_ms(100);
    for(uint i = 0; i < ADC_BIAS_OVERSAMPLE; i++)   // Find the zero-current ADC reading. Reads the ADC multiple times and takes the average
    {
        adc_select_input(0);
        adc_bias += adc_read();
    }
    adc_bias /= ADC_BIAS_OVERSAMPLE;

    adc_set_round_robin(0b111);     // Set ADC to read our three ADC pins one after the other (round robin)
    adc_fifo_setup(true, false, 3, false, false);   // ADC writes into a FIFO buffer, and an interrupt is fired once FIFO reaches 3 samples
    irq_set_exclusive_handler(ADC_IRQ_FIFO, on_adc_fifo);   // Sets ADC interrupt
    irq_set_priority(ADC_IRQ_FIFO, 0);
    adc_irq_set_enabled(true);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    pwm_clear_irq(A_PWM_SLICE);     // Clear interrupt flag, just in case
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);   // Set interrupt to fire when PWM counter wraps
    irq_set_priority(PWM_IRQ_WRAP, 0);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    float pwm_divider = (float)(clock_get_hz(clk_sys)) / (F_PWM * 255 * 2);     // Calculate the desired PWM divisor
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, pwm_divider);
    pwm_config_set_wrap(&config, 255 - 1);      // Set the PWM to wrap at 254. This allows a PWM value of 255 to equal 100% duty cycle
    pwm_config_set_phase_correct(&config, true);    // Set phase correct (counts up then down). This is allows firing the interrupt in the middle of the PWM cycle
    pwm_config_set_output_polarity(&config, false, true);   // Invert the lowside PWM such that 0 corresponds to lowside transistors on

    writePhases(0, 0, 0, 0, 0, 0);  // Initialize all the PWMs to be off

    pwm_init(A_PWM_SLICE, &config, false);
    pwm_init(B_PWM_SLICE, &config, false);
    pwm_init(C_PWM_SLICE, &config, false);

    pwm_set_mask_enabled(0x07); // Enable our three PWM timers
}

uint get_optical_state() {
    // Initialize counters for oversampling
    uint opt1Counts[] = {0, 0, 0};
    uint opt2Counts[] = {0, 0, 0};

    // Oversampling loop
    for(uint i = 0; i < ENCODER_OVERSAMPLE; i++) {
        opt1Counts[0] += gpio_get(OPT_U1_PIN);
        opt1Counts[1] += gpio_get(OPT_V1_PIN);
        opt1Counts[2] += gpio_get(OPT_W1_PIN);

        opt2Counts[0] += gpio_get(OPT_U2_PIN);
        opt2Counts[1] += gpio_get(OPT_V2_PIN);
        opt2Counts[2] += gpio_get(OPT_W2_PIN);
    }

    // Compute final state based on majority vote
    uint state_OPT_1 = 0;
    uint state_OPT_2 = 0;

    for (uint i = 0; i < 3; i++) {
        if (opt1Counts[i] > ENCODER_OVERSAMPLE / 2) state_OPT_1 |= (1 << (2 - i)); // Encode as 3-bit value
        if (opt2Counts[i] > ENCODER_OVERSAMPLE / 2) state_OPT_2 |= (1 << (2 - i)); // Encode as 3-bit value
    }

    // Combine state_OPT_1 and state_OPT_2 into a single value
    uint combined_state = (state_OPT_1 << 3) | state_OPT_2;

    switch (combined_state) {
        case 41: return 0; // 0b101 001
        case 33: return 1; // 0b100 001
        case 37: return 2; // 0b100 101
        case 53: return 3; // 0b110 101
        case 52: return 4; // 0b110 100
        case 20: return 5; // 0b010 100

        case 22: return 0; // 0b010 110
        case 30: return 1; // 0b011 110
        case 26: return 2; // 0b011 010
        case 10: return 3; // 0b001 010
        case 11: return 4; // 0b001 011
        case 43: return 5; // 0b101 011

        // Reversed cables
        case 13: return 0; // 0b001 101
        case 12: return 1; // 0b001 100
        case 44: return 2; // 0b101 100
        case 46: return 3; // 0b101 110
        case 38: return 4; // 0b100 110
        case 34: return 5; // 0b100 010

        case 50: return 0; // 0b110 010
        case 51: return 1; // 0b110 011
        case 19: return 2; // 0b010 011
        case 17: return 3; // 0b010 001
        case 25: return 4; // 0b011 001
        case 29: return 5; // 0b011 101

        default: return 0; // Invalid state
    }
}

void commutate_open_loop()
{
    // A useful function to debug electrical problems with the board.
    // This slowly advances the motor commutation without reading optical sensors. The motor should slowly spin
    int state = 0;
    while(true)
    {
        writePWM(state % 6, 25, false);
        sleep_ms(50);
        state++;
    }
}

int main() {
    init_hardware();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    sleep_ms(1000);

    pwm_set_irq_enabled(A_PWM_SLICE, true); // Enables interrupts, starting motor commutation

    while (true) {
        printf("%6d, %6d, %6d, %6d, %2d, %2d\n", current_ma, current_target_ma, duty_cycle, voltage_mv, optical, motorState);
        gpio_put(LED_PIN, !gpio_get(LED_PIN));  // Toggle the LED
        sleep_ms(100);
    }

    return 0;
}