#include <Arduino.h>
#include <pico.h>
#include <hardware/adc.h>
#include <hardware/pwm.h>
#include <hardware/gpio.h>

#include <TempSensor.h>


// Pin definitions 
const int FAN_PIN = D2;
const int IGBT_GATE_PIN = D3;
const int CHARGE_PUMP_PIN = D4; 
const int THERMOCOUPLE_1W_PIN = D5;
const int COIL_MONITOR_PIN = A0;
const int AC_MONITOR_PIN = A1;
                           
// Code for calculating Mean Absolute Deviation of input signal for pan detection
const uint16_t N_SAMPLES = 500;           // 1ms at 500ksps = 500 samples
uint16_t input_buffer[N_SAMPLES];         // Buffer to store input samples
uint32_t input_accumulator = 0;           // Accumulator for calculating mean
uint16_t input_buffer_index = 0;          // Index of the input buffer
bool pan_detection_on = false;            // Start bit

TempSensor thermocouple(THERMOCOUPLE_1W_PIN);

//
// ---- Core 0 ----
//

void computeMAD() {
  // Compute mean
  uint16_t mean = uint16_t(input_accumulator * float(1.0f/N_SAMPLES));

  // Compute mean absolute deviation
  uint32_t acc = 0;
  for (uint i = 0; i < N_SAMPLES; i++) {
    acc += abs(input_buffer[i] - mean);
  }
  float mad = acc * (1.0f/N_SAMPLES);

  // Print result
  Serial.print("Mean: ");
  Serial.print(mean);
  Serial.print(" MAD: ");
  Serial.println(mad);
}

// Initialize PWM for charge pump output pin
// Frequency should be around 1.5kHz
// Duty cycle 50%, but precision is not required for this circuit
void start_charge_pump() {
  const uint freq = 1500;     // PWM frequency
  const uint div = 256;       // Clock divider, 1-255

  uint top = uint((133e6 / div) / freq);
  uint slice = pwm_gpio_to_slice_num(CHARGE_PUMP_PIN);
  uint chan = pwm_gpio_to_channel(CHARGE_PUMP_PIN);
  gpio_set_function(CHARGE_PUMP_PIN, GPIO_FUNC_PWM);
  pwm_set_clkdiv(slice, div);
  pwm_set_wrap(slice, top); 
  pwm_set_chan_level(slice, chan, top/2);
  pwm_set_enabled(slice, true);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello World!");

  start_charge_pump();              // Initialize charge pump PWM to generate negative rail for opamp
  gpio_set_dir(FAN_PIN, GPIO_OUT);  

  adc_init();
  adc_gpio_init(COIL_MONITOR_PIN);
  adc_gpio_init(AC_MONITOR_PIN);
}

void loop() {
  ;
}


//
// ---- Core 1 ----
// 

// ADC FIFO IRQ
// Is triggered when the FIFO contains at least 1 sample
//
void __not_in_flash_func(adc_to_buffer)() {
  // Discard samples to get only most recent one
  while (adc_fifo_get_level() > 1) {
    adc_fifo_get(); // Discard sample
  }
  uint16_t sample = adc_fifo_get(); // Get most recent sample

  // Add sample to input buffer and accumulator if needed
  if (input_buffer_index < N_SAMPLES) {
    input_buffer[input_buffer_index] = sample;
    input_accumulator += sample;
    input_buffer_index++;
  }
  // Check if we have enough samples, then stop interrupt
  if (input_buffer_index == N_SAMPLES) {
    adc_run(false);
    irq_clear(ADC_IRQ_FIFO);
  }
}

// Core 1 samples the analog input A0 as fast as possible
void setup1() {
  // Init and configure ADC
  adc_init();
  adc_gpio_init(A0);                                    // Only init one pin
  adc_select_input(0);                                  // Select mux input
}

void start_fast_adc() {
  // Reset buffer
  input_accumulator = 0;
  input_buffer_index = 0;
  // Set up interrupt
  adc_set_clkdiv(0.0f);                                 // Set ADC clock to 48MHz / (96 + n) cycles per sample = 500kHz
  adc_irq_set_enabled(true);
  adc_fifo_setup(
    true, // Write each completed conversion to the sample FIFO
    false,// Disable DMA data request (DREQ)
    1,    // DREQ/IRQ asserted when at least 1 sample present in FIFO
    false,// We won't see the ERR bit because of 1 sample threshold
    false // Don't shift each sample to 8 bits (useful when pushing to FIFO -> DMA to byte buffers)
  );
  irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_to_buffer);  // Set ADC IRQ handler
  irq_set_enabled(ADC_IRQ_FIFO, true);                  // Enable IRQ on current core
  adc_run(true);                                        // Start ADC free running mode
}

// Core 1 main loop
void loop1() {
  // Wait for start signal, fill buffer with samples inside interrupt
  static bool adc_running = false;
  if (pan_detection_on && !adc_running){
    start_fast_adc();
    adc_running = true;
  }
  // Check for complete
  if (adc_running && input_buffer_index == N_SAMPLES){
    pan_detection_on = false;
    adc_running = false;
  }
}
