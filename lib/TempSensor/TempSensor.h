// Thermocouple interface
#include <OneWireNg_CurrentPlatform.h>
#include <drivers/MAX31850.h>
#include <utils/Placeholder.h>

class TempSensor{
public:
TempSensor(uint pin) :
    onewire_bus(pin, false),
    sensor(onewire_bus)
{
    // Start first conversion
    sensor.convertTempAll(0); // 0 conversion time, we'll handle the temp conversion delay here instead
    last_conversion_ms = millis();
    temp_internal = -1000;
}

// Returns current temperature value from MAX31850 in degrees C
// If conversion is currently in progress, return previous value
float read_temp() 
{
    if (millis() >= (last_conversion_ms + conv_delay_ms)) {
        sensor.readScratchpadSingle(scratchpad);        // Read sensor over bus
        temp_internal = scratchpad->getTemp() * 1e-3;   // Scale from fixed to floating point
        sensor.convertTempAll(0);                       // Start next conversion
        last_conversion_ms = millis();                  // Store timestamp 
        // Check for (likely) wrong values
        if (temp_internal > 400 || temp_internal < -100)
            temp_internal = -1000;
        return temp_internal;
    } else {
        return temp_internal;
    }
}

const uint conv_delay_ms = 100;

private:
OneWireNg_CurrentPlatform onewire_bus;
MAX31850 sensor;
Placeholder<MAX31850::Scratchpad> scratchpad;
float temp_internal;
uint32_t last_conversion_ms;
};
