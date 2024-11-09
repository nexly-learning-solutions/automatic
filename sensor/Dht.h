#ifndef DHT_H
#define DHT_H

#include "Arduino.h"

/// <summary>
/// Defines the debug printer (Serial by default).
/// </summary>
#define DEBUG_PRINTER                                                          \
  Serial

#ifdef DHT_DEBUG
/// <summary>
/// Macro to print debug messages.
/// </summary>
/// <param name="...">The message to print.</param>
#define DEBUG_PRINT(...)                                                       \
  { DEBUG_PRINTER.print(__VA_ARGS__); }
/// <summary>
/// Macro to print debug messages with a newline.
/// </summary>
/// <param name="...">The message to print.</param>
#define DEBUG_PRINTLN(...)                                                     \
  { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
/// <summary>
/// Empty macro for debug printing when debug is disabled.
/// </summary>
#define DEBUG_PRINT(...)                                                       \
  {}
/// <summary>
/// Empty macro for debug printing with newline when debug is disabled.
/// </summary>
#define DEBUG_PRINTLN(...)                                                     \
  {}
#endif

/// <summary>
/// DHT11 sensor type.
/// </summary>
#define DHT11 11
/// <summary>
/// DHT12 sensor type.
/// </summary>
#define DHT12 12
/// <summary>
/// DHT22 sensor type.
/// </summary>
#define DHT22 22
/// <summary>
/// DHT21 sensor type.
/// </summary>
#define DHT21 21
/// <summary>
/// AM2301 sensor type, same as DHT21.
/// </summary>
#define AM2301 21

#if (TARGET_NAME == ARDUINO_NANO33BLE)
#ifndef microsecondsToClockCycles
/// <summary>
/// Defines the conversion from microseconds to clock cycles for the ARDUINO_NANO33BLE platform.
/// </summary>
/// <param name="a">The number of microseconds.</param>
/// <returns>The equivalent clock cycles.</returns>
#define microsecondsToClockCycles(a) ((a) * (SystemCoreClock / 1000000L))
#endif
#endif

/// <summary>
/// Class for handling DHT (Digital Humidity and Temperature) sensor.
/// </summary>
class DHT {
public:
  /// <summary>
  /// Constructor for the DHT class, initializes the sensor with the given pin and type.
  /// </summary>
  /// <param name="pin">The pin connected to the sensor.</param>
  /// <param name="type">The type of the sensor (e.g., DHT11, DHT22).</param>
  /// <param name="count">An unused parameter for compatibility (default is 6).</param>
  DHT(uint8_t pin, uint8_t type, uint8_t count = 6);

  /// <summary>
  /// Initializes the DHT sensor.
  /// </summary>
  /// <param name="usec">The pulse width used for reading sensor data (default is 55).</param>
  void begin(uint8_t usec = 55);

  /// <summary>
  /// Reads the temperature from the sensor in Celsius or Fahrenheit.
  /// </summary>
  /// <param name="S">If true, returns the temperature in Fahrenheit, otherwise in Celsius (default is false).</param>
  /// <param name="force">If true, forces a new reading, otherwise uses the previous one (default is false).</param>
  /// <returns>The temperature value.</returns>
  float readTemperature(bool S = false, bool force = false);

  /// <summary>
  /// Converts Celsius to Fahrenheit.
  /// </summary>
  /// <param name="c">Temperature in Celsius.</param>
  /// <returns>Temperature in Fahrenheit.</returns>
  float convertCtoF(float c);

  /// <summary>
  /// Converts Fahrenheit to Celsius.
  /// </summary>
  /// <param name="f">Temperature in Fahrenheit.</param>
  /// <returns>Temperature in Celsius.</returns>
  float convertFtoC(float f);

  /// <summary>
  /// Computes the heat index based on the current temperature and humidity.
  /// </summary>
  /// <param name="isFahrenheit">If true, computes the heat index in Fahrenheit, otherwise in Celsius (default is true).</param>
  /// <returns>The heat index value.</returns>
  float computeHeatIndex(bool isFahrenheit = true);

  /// <summary>
  /// Computes the heat index based on the given temperature and humidity.
  /// </summary>
  /// <param name="temperature">Temperature value in Celsius or Fahrenheit.</param>
  /// <param name="percentHumidity">Humidity percentage.</param>
  /// <param name="isFahrenheit">If true, assumes the temperature is in Fahrenheit.</param>
  /// <returns>The computed heat index value in the appropriate temperature unit.</returns>
  float computeHeatIndex(float temperature, float percentHumidity,
                         bool isFahrenheit = true);

  /// <summary>
  /// Reads the humidity from the sensor.
  /// </summary>
  /// <param name="force">If true, forces a new reading, otherwise uses the previous one (default is false).</param>
  /// <returns>The humidity value.</returns>
  float readHumidity(bool force = false);

  /// <summary>
  /// Reads the sensor data, ensuring that the reading is valid.
  /// </summary>
  /// <param name="force">If true, forces a new reading, otherwise uses the previous one (default is false).</param>
  /// <returns>True if the reading is successful, false otherwise.</returns>
  bool read(bool force = false);

private:
  uint8_t data[5];                        // Data buffer to store sensor readings.
  uint8_t _pin, _type;                    // Pin and sensor type.
#ifdef __AVR
  uint8_t _bit, _port;                    // Bit and port for AVR platform.
#endif
  uint32_t _lastreadtime, _maxcycles;     // Last read time and max cycle duration.
  bool _lastresult;                       // Stores the last reading result.
  uint8_t pullTime;                       // Pulse time for reading sensor data.

  /// <summary>
  /// Waits for the expected pulse level and returns the pulse duration.
  /// </summary>
  /// <param name="level">The expected pulse level (HIGH or LOW).</param>
  /// <returns>The duration of the pulse in clock cycles or TIMEOUT if it exceeds max cycles.</returns>
  uint32_t expectPulse(bool level);
};

/// <summary>
/// Interrupt lock class for disabling and enabling interrupts.
/// </summary>
class InterruptLock {
public:
  /// <summary>
  /// Constructor disables interrupts.
  /// </summary>
  InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)
    noInterrupts();
#endif
  }

  /// <summary>
  /// Destructor enables interrupts.
  /// </summary>
  ~InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)
    interrupts();
#endif
  }
};

#endif