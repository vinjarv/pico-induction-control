#include <Arduino.h>
#include <pico.h>
#include <hardware/adc.h>

const int INDICATOR_PIN = LED_BUILTIN;

uint16_t negative_treshold = (2^12 / 2) + 50; // Below this value current should be passing through the body diode of the IGBT
                                              //TODO: Compute live based on AC voltage
                                      
// Code for calculating Mean Absolute Deviation of input signal for pan detection
const uint16_t N_SAMPLES = 500;           // 1ms at 500ksps = 500 samples
uint16_t input_buffer[N_SAMPLES];         // Buffer to store input samples
uint32_t input_accumulator = 0;           // Accumulator for calculating mean
uint16_t input_buffer_index = 0;          // Index of the input buffer
bool pan_detection_on = false;            // Start bit

bool cycle_negative = false;

uint16_t avg_buffer[2];
uint16_t moving_avg = 0;

long last_time = 0;
uint32_t shared_var = 0; // For performance monitoring: incremented once per ADC sample

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


void setup() {
  Serial.begin(115200);
  Serial.println("Hello World!");

  pan_detection_on = true; // Start sampling loop
}

void loop() {
  long current_time = millis();
  if (last_time + 1000 < current_time){
    last_time = current_time;
    // Print how many times loop1() was executed in the last second
    uint32_t temp = shared_var;
    shared_var = 0;
    Serial.print("Samples in the last second: ");
    Serial.println(temp);

    if (!pan_detection_on) {
      // Get MAD and start next sampling
      computeMAD();
      input_buffer_index = 0;
      input_accumulator = 0;
      pan_detection_on = true;
    }
  }
    // Indicate if current voltage is below threshold
    digitalWrite(INDICATOR_PIN, cycle_negative);
}


//
// ---- Core 1 ----
// 

// ADC FIFO IRQ
// Is triggered when the FIFO contains at least 1 sample
//
void my_adc_irq() {
  // Discard samples to get only most recent one
  while (adc_fifo_get_level() > 1) {
    adc_fifo_get(); // Discard sample
  }
  uint16_t sample = adc_fifo_get(); // Get most recent sample

  // Add sample to input buffer and accumulator if needed
  if (pan_detection_on && input_buffer_index < N_SAMPLES) {
    input_buffer[input_buffer_index] = sample;
    input_accumulator += sample;
    input_buffer_index++;
  }
  // Check if we have enough samples
  if (input_buffer_index == N_SAMPLES) {
    pan_detection_on = false;
  }

  // Moving average
  avg_buffer[1] = avg_buffer[0];
  avg_buffer[0] = sample;
  moving_avg = (avg_buffer[0] + avg_buffer[1]) / 2; // Division by powers of two should be fast, as it is just a bit shift

  // Indicate if current voltage is below threshold
  cycle_negative = moving_avg < negative_treshold;
  
  // Increment the shared variable to count how many times this IRQ was executed
  shared_var++;
}

// Core 1 samples the analog input A0 as fast as possible
void setup1() {
  // Init and configure ADC
  adc_init();
  adc_gpio_init(A0);                                    // Only init one pin
  adc_select_input(0);                                  // Select mux input
  adc_set_clkdiv(0.0f);                                 // Set ADC clock to 48MHz / (96 + n) cycles per sample = 500kHz
  // Set up interrupt
  adc_irq_set_enabled(true);
  adc_fifo_setup(
    true, // Write each completed conversion to the sample FIFO
    false,// Disable DMA data request (DREQ)
    1,    // DREQ/IRQ asserted when at least 1 sample present in FIFO
    false,// We won't see the ERR bit because of 1 sample threshold
    false // Don't shift each sample to 8 bits (useful when pushing to FIFO -> DMA to byte buffers)
  );
  irq_set_exclusive_handler(ADC_IRQ_FIFO, my_adc_irq);  // Set ADC IRQ handler
  irq_set_enabled(ADC_IRQ_FIFO, true);                  // Enable IRQ on current core
  adc_run(true);                                        // Start ADC free running mode
}

// Core 1 is busy sampling the ADC, no need to do anything here
// void loop1() {
//   tight_loop_contents();
// }
