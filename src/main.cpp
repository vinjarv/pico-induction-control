#include <Arduino.h>
#include <pico.h>
#include <hardware/adc.h>
#include <hardware/pwm.h>
#include <hardware/gpio.h>

#include <TempSensor.h>
#include <MeanAbsoluteDeviation.h>

// Pin definitions 
const int FAN_PIN = D2;
const int IGBT_GATE_PIN = D3;
const int CHARGE_PUMP_PIN = D4; 
const int THERMOCOUPLE_1W_PIN = D5;
const int COIL_MONITOR_PIN = A0;
const int AC_MONITOR_PIN = A1;

const uint PAN_DETECTION_INTERVAL_MS = 1000;    // How often to run pan detection
const uint PAN_DETECTION_TRESHOLD_UPPER = 100;  // MAD tresholds for pan detection - higher means too much ringing - no pan to dampen it
const uint PAN_DETECTION_TRESHOLD_LOWER = 10;   // MAD tresholds for pan detection - too low is probably an error
const uint PAN_DETECTION_SETTLE_TIME_US = 2000; // Time to wait for coil to deenergize before running pan detection

volatile bool pan_detection_on = false;       // Start pan detection ADC sampling cycle
float temp_req = 0.0f;                        // Temp in C
float power_req = 0.0f;                       // Power in W. Likely quite inaccurate!
String command = "";
float temp_current = 0.0f;

TempSensor thermocouple(THERMOCOUPLE_1W_PIN);
MeanAverageDeviation mad_calc(500);           // 1ms at 500ksps = 500 samples

//
// ---- Core 0 ----
//

uint state = 0;

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

// Write induction power level to hardware
void set_gate_pwm_power(float power) {
  uint slice = pwm_gpio_to_slice_num(IGBT_GATE_PIN);
  uint chan = pwm_gpio_to_channel(IGBT_GATE_PIN);

  // Disable output
  if (power <= 0.0f) {
    pwm_set_enabled(slice, false);
    gpio_set_function(IGBT_GATE_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(IGBT_GATE_PIN, GPIO_OUT);
    gpio_put(IGBT_GATE_PIN, LOW);
    return;
  }

  // Constrain power level for safety - lower than this must be handled by switching off and on output at low frequency
  const float MIN_POWER = 1000.0f;
  const float MAX_POWER = 2000.0f;
  power = constrain(power, MIN_POWER, MAX_POWER);

  // T_on controls the amount of power delivered to the tank circuit
  // T_off should be fixed, as the time to zero crossing depends on the resonant frequency
  // For simplicity, these values are fixed. Monitoring and switching at runtime would be ideal, but complicated due to the high frequency
  // T_on min. and max. obtained from testing with stock controller
  uint p_min = uint(1000.0f * 100.0f);
  uint p_max = uint(2000.0f * 100.0f);
  uint t_on_min = uint(13.7f * 100.0f); // In microseconds
  uint t_on_max = uint(19.4f * 100.0f);
  float t_on = map(power, p_min, p_max, t_on_min, t_on_max) * 0.01f; // All values scaled as map function uses integer math
  const float t_off = 27.0f;
  // Set counter wrap level
  // From datasheet:
  // f_pwm = 1/T = f_sys / (TOP + 1)
  // TOP = f_sys / f_pwm - 1
  uint top = int(133 * (t_on + t_off)) - 1; // MHz * ms cancel out
  // Set counter treshold value for high -> low transition
  uint level = uint(top * t_off / (t_on + t_off));
  // Set PWM registers
  pwm_set_wrap(slice, top);
  pwm_set_chan_level(slice, chan, level);
  gpio_set_function(IGBT_GATE_PIN, GPIO_FUNC_PWM);
  pwm_set_enabled(slice, true);
}

// Set induction output based on (approximate) required power level 
void set_power(float power) {
  // Check if we actually have to do anything
  static float power_prev = 0.0f;
  if (power == power_prev)
    return;
  power_prev = power;

  // If power is low, it's inefficient to use very low on-times. Insted, we use a very low frequency PWM to modulate the output
  const uint SLOW_PWM_PERIOD = 2000;    // Milliseconds time period for slow PWM
  uint on_phase = 1;                    // Determine if output should be enabled 
  if (power < 1000.0f) {
    // On until (time mod period) is over treshold
    uint treshold = power * 1.0f/1000.0f * SLOW_PWM_PERIOD;
    on_phase = treshold > (millis() % SLOW_PWM_PERIOD);
  }

  set_gate_pwm_power(power * on_phase);
}

// Set regulated temperature level
void set_temp(float temp_c) {
  if (temp_c > 25.0f && temp_c < 300.0f){
    temp_req = temp_c;
  } else {
    temp_req = 0.0f;
  }

  if (temp_req <= 0.0f) {
    set_power(0.0f);
  }
  // Actually regulating the temperature happens in the main loop
}

// Handle serial command
// Simple protocol
void handle_command() {
  uint separator = command.indexOf(" ");
  String cmd;
  String val;
  if (separator >= (command.length() - 1) || command.length() == 0){
    cmd = command;
    val = "";
  } else {
    cmd = command.substring(0, separator);
    val = command.substring(separator+1, command.length());
  }

  if (cmd == "T") {
    if (val.length() > 0) {
      temp_req = val.toFloat();
      if (temp_req == 0.0f) {
        power_req = 0.0f;
      }
    }
    Serial.println(temp_current);
    return;
  }

  if (cmd == "P") {
    if (val.length() > 0) {
      // Power control mode, disable temp. regulation
      temp_req = 0.0f;
      power_req = val.toFloat();
    }
    Serial.println(power_req);
    return;
  }

  if (cmd == "detect") {
    // Run pan detection
    if (state == 0) {
      // Disabled, it should be safe to run pan detection
      pan_detection_on = true;
      while (pan_detection_on) {tight_loop_contents();} // Wait for result
      Serial.println(mad_calc.MAD);
      return;
    } else {
      Serial.println("");
      return;
    }
  }

  // If no match, reply should be empty
  Serial.println("");
}

void run_regulator() {
  const float HYSTERESIS_C = 2.0f;
  float e = temp_req - temp_current;
  // float p = e * 0.1f; //TODO: PID?
  power_req = 2000.0f * (temp_current < (temp_req - HYSTERESIS_C)) + 0.0f * (temp_current > (temp_req + HYSTERESIS_C));
}

void setup() {
  Serial.begin(115200);             // USB 
  Serial.println("Hello World!");

  Serial1.setTX(D0);
  Serial1.setRX(D1);
  Serial1.begin(115200);            // Hardware UART

  start_charge_pump();              // Initialize charge pump PWM to generate negative rail for opamp
  gpio_set_dir(FAN_PIN, GPIO_OUT);  

  adc_init();
  adc_gpio_init(COIL_MONITOR_PIN);
  adc_gpio_init(AC_MONITOR_PIN);
}

void loop() {
  // Check and respond to serial port
  while (Serial.available() > 0) {
    command += String((char)Serial.read());
    // Handle command when newline is received
    if (command.charAt(command.length()-1) == '\n') {
      command.trim();
      handle_command();
      command = "";
    } else if (command.length() > 32) {
      command = "";
    }
  }

  // temp_current = thermocouple.read_temp(); //TODO: uncomment
  temp_current = 25.0f;

  // Main state machine
  static uint32_t timer_ms = 0;
  static uint32_t timer_us = 0;
  switch (state)
  {
    case 0:
      // Disabled
      digitalWrite(FAN_PIN, LOW);
      set_power(0.0f);
      if (power_req > 0.0f || temp_req > 0.0f) {
        state = 1;
      }
      break;

    case 1:
      // Start up
      // Ensure pan presence
      pan_detection_on = true;
      state = 2;
      break;

    case 2:
      // Wait for pan detection
      if (!pan_detection_on)
        state = 3;
      break;
    
    case 3:
      // Check if pan is present
      if (mad_calc.MAD > PAN_DETECTION_TRESHOLD_LOWER && mad_calc.MAD < PAN_DETECTION_TRESHOLD_UPPER) {
        state = 10;
        timer_ms = millis();
      } else {
        state = 4;
        timer_ms = millis();
      }
      break;

    case 4:
      // Pan detection false, wait for next attempt
      if (millis() > timer_ms + PAN_DETECTION_INTERVAL_MS) {
        state = 0;
      }
      break;

    case 10:
      // Pan detected, main step
      // If pan detection needed again
      if (millis() > timer_ms + PAN_DETECTION_INTERVAL_MS) {
        state = 11;
        set_power(0.0f);
        timer_us = micros();
        break;
      }

      if (temp_req > 0.0f)
        run_regulator();

      digitalWrite(FAN_PIN, HIGH);
      set_power(power_req);

      // Disable condition
      if (temp_req <= 0.0f && power_req <= 0.0f) {
        state = 0;
      }
      break;

    case 11:
      // Wait for coil to deenergize before running pan detection again
      if (micros() > timer_us + PAN_DETECTION_SETTLE_TIME_US) {
        state = 1;
      }
      break;
  }
}

//
// ---- Core 1 ----
// 

// ADC FIFO interrupt handler for coil monitor ADC channel
// Is triggered when the ADC FIFO contains at least 1 sample
// In RAM, might improve access time
void __not_in_flash_func(coil_adc_to_buffer)() {
  // Discard samples to get only most recent one
  while (adc_fifo_get_level() > 1) {
    adc_fifo_get(); // Discard sample
  }
  // Check if finished, stop generating interrupts if so
  if (mad_calc.buffer_full()) {
    adc_run(false);
    adc_fifo_get(); // Discard sample
    return;
  }
  // Add most recent sample to input buffer
  uint16_t sample = adc_fifo_get();
  mad_calc.append(sample);
}

// Indicates if AC peak is detected, set by interrupt only
volatile bool ac_peak = false;
// ADC FIFO interrupt handler for AC monitor ADC channel
// Is triggered when the ADC FIFO contains at least 1 sample
void ac_adc_detection() {
  // Discard samples to get only most recent one
  while (adc_fifo_get_level() > 1) {
    adc_fifo_get(); // Discard sample
  }
  // Averaging to suppress some noise
  static uint16_t last_measurements[3];
  last_measurements[2] = last_measurements[1];
  last_measurements[1] = last_measurements[0];
  last_measurements[0] = adc_fifo_get();
  uint32_t avg = (last_measurements[0] + last_measurements[1] + last_measurements[2]) / 3;
  // ADC values map 0..3v3 to 0..4095
  // Resistor conversion takes -400V..400V to 0..3v3 (so 0V is 2047)
  // 5.12 counts/v
  // AC peak should be around 325V, let's say a treshold of 90% is the peak
  // 2047 + 292V * 5.12 counts/v = 3542
  ac_peak = avg > 3542;
}

// Stop ADC interrupt and sampling
void stop_interrupts() {
  adc_run(false);                                             // Stop ADC if running
  adc_irq_set_enabled(false);
  irq_set_enabled(ADC_IRQ_FIFO, false);
  irq_remove_handler(ADC_IRQ_FIFO, irq_get_exclusive_handler(ADC_IRQ_FIFO)); // Remove previous handler
}

// Enable ADC interrupt and start sampling 
void begin_interrupts(irq_handler_t handler) {
  adc_fifo_drain();                                          
  adc_fifo_setup(
    true, // Write each completed conversion to the sample FIFO
    false,// Disable DMA data request (DREQ)
    1,    // DREQ/IRQ asserted when at least 1 sample present in FIFO
    false,// We won't see the ERR bit because of 1 sample threshold
    false // Don't shift each sample to 8 bits (useful when pushing to FIFO -> DMA to byte buffers)
  );
  irq_set_exclusive_handler(ADC_IRQ_FIFO, handler);  // Set ADC IRQ handler
  irq_set_enabled(ADC_IRQ_FIFO, true);                        // Enable IRQ on current core
  adc_irq_set_enabled(true);
  adc_run(true);                                              // Start ADC free running mode
}

// Start sampling with ADC interrupt callback
void start_fast_coil_adc() {
  stop_interrupts();
  adc_gpio_init(COIL_MONITOR_PIN);
  adc_select_input(COIL_MONITOR_PIN - 26);                    // Select mux input
  // Set up interrupt
  adc_set_clkdiv(0.0f);                                       // Set ADC clock to 48MHz / (96 + n) cycles per sample = 500kHz
  begin_interrupts(coil_adc_to_buffer);
}

void start_ac_adc() {
  stop_interrupts();
  adc_gpio_init(AC_MONITOR_PIN);
  adc_select_input(AC_MONITOR_PIN - 26);                      // Select mux input
  // Set up interrupt
  adc_set_clkdiv(4704.0f);                                    // Set ADC clock to 48MHz / (96 + n) cycles per sample = 10kHz
  begin_interrupts(ac_adc_detection);
}

// Core 1 handles ADC
void setup1() {
  adc_init();
}

// Core 1 main loop
void loop1() {
  static bool adc_started1 = false;
  if (!adc_started1 && millis() > 10) {
    start_ac_adc();
    adc_started1 = true;
  }

  // If pan detection is requested and close to peak AC voltage is reached, ping coil and run fast sampling on coil ADC channel
  // Calculate MAD, and leave results for core 0 to check
  static uint pan_detection_state = 0;
  static uint32_t pan_detection_timer = 0;
  switch (pan_detection_state) {
    case 0:
      // Wait for start bit
      if (pan_detection_on) {
        pan_detection_state = 1;
        mad_calc.reset_buffer();
        pan_detection_timer = millis();
      }
      break;

    case 1:
      // Wait for AC peak
      // if (ac_peak) {
      //   pan_detection_state = 2;
      //   adc_run(false);                 // Stop ADC to avoid interrupts while energizing coil
      // }
      // // Timeout
      // if (millis() > pan_detection_timer + 100) {
      //   mad_calc.MAD = 0.0f;
      //   pan_detection_on = false;
      //   pan_detection_state = 0;
      //   start_ac_adc();
      // }
      //TODO: enable again. Only commented out for testing 
      pan_detection_state = 2;
      break;
    
    case 2:
      // Ping coil
      // Important: pan_detection_on must not be enabled before coil has had time to deenergize
      // Otherwise reading will likely be wrong. Might also result in damage
      gpio_put(IGBT_GATE_PIN, HIGH);
      delayMicroseconds(1);             // 1us pulse
      gpio_put(IGBT_GATE_PIN, LOW);
      start_fast_coil_adc();
      pan_detection_state = 3;
      break;
    
    case 3:
      // Wait for ADC buffer to fill
      if (mad_calc.buffer_full()) {
        mad_calc.compute_MAD();
        pan_detection_state = 4;
      }
      break;
    
    case 4:
      // Pan detection complete, use start bit to indicate completion
      pan_detection_on = false;
      pan_detection_state = 0;
      start_ac_adc();
  }
}
