#include <Arduino.h>
#include <pico.h>
#include <vector>

class MeanAverageDeviation{
public:
// Code for calculating Mean Absolute Deviation of input signal for pan detection
// Output values
volatile uint16_t mean;
volatile float MAD;

const uint16_t N_SAMPLES;

MeanAverageDeviation(uint samples):
    N_SAMPLES(samples)
{
    buffer.reserve(N_SAMPLES);
}

// Calculate Mean Absolute Deviation of buffered values 
void compute_MAD() {
    if (!buffer_full()){
        mean = 0;
        MAD = 0;
        return;
    }
    // Compute mean
    mean = uint16_t(accumulator * float(1.0f/N_SAMPLES));
    // Compute mean absolute deviation
    uint32_t acc = 0;
    for (uint i = 0; i < N_SAMPLES; i++) {
    acc += abs(buffer[i] - mean);
    }
    MAD = acc * (1.0f/N_SAMPLES);
}

// Save a value to the buffer
inline void append(uint16_t val) {
    if (buffer_full())
        return;
    buffer[buffer_index] = val;
    accumulator += val;
    buffer_index++;
}

// Check if buffer is full
volatile bool buffer_full(){
    return buffer_index >= N_SAMPLES;
}

// Reset
void reset_buffer(){
    accumulator = 0;
    buffer_index = 0;
}

// private:
// Storage variables
std::vector<uint16_t> buffer;         // Buffer to store input samples
uint32_t accumulator = 0;           // Accumulator for calculating mean
uint16_t buffer_index = 0;          // Index of the input buffer
};
