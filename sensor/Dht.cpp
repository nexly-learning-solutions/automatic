#include "DHT.h"
#include <cstdint>
#include <cmath>

#define MIN_INTERVAL 2000
#define TIMEOUT                                                                \
  UINT32_MAX

/// <summary>
/// Constructor for the DHT class.
/// </summary>
/// <param name="pin">The pin connected to the sensor.</param>
/// <param name="type">The type of sensor (e.g., DHT11, DHT22).</param>
/// <param name="count">Unused parameter, retained for compatibility.</param>
DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  (void)count;
  _pin = pin;
  _type = type;
#ifdef __AVR
  _bit = digitalPinToBitMask(pin);
  _port = digitalPinToPort(pin);
#endif
  _maxcycles =
      microsecondsToClockCycles(1000);
}

/// <summary>
/// Initializes the DHT sensor.
/// </summary>
/// <param name="usec">The pulse width used for reading sensor data.</param>
void DHT::begin(uint8_t usec) {
  pinMode(_pin, INPUT_PULLUP);
  _lastreadtime = millis() - MIN_INTERVAL;
  DEBUG_PRINT("DHT max clock cycles: ");
  DEBUG_PRINTLN(_maxcycles, DEC);
  pullTime = usec;
}

/// <summary>
/// Reads the temperature from the sensor in Celsius or Fahrenheit.
/// </summary>
/// <param name="S">If true, returns the temperature in Fahrenheit. Otherwise, in Celsius.</param>
/// <param name="force">If true, forces a new reading, otherwise uses the previous one.</param>
/// <returns>The temperature value.</returns>
float DHT::readTemperature(bool S, bool force) {
  float f = NAN;

  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if (data[3] & 0x80) {
        f = -1 - f;
      }
      f += (data[3] & 0x0f) * 0.1;
      if (S) {
        f = convertCtoF(f);
      }
      break;
    case DHT12:
      f = data[2];
      f += (data[3] & 0x0f) * 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      if (S) {
        f = convertCtoF(f);
      }
      break;
    case DHT22:
    case DHT21:
      f = ((word)(data[2] & 0x7F)) << 8 | data[3];
      f *= 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      if (S) {
        f = convertCtoF(f);
      }
      break;
    }
  }
  return f;
}

/// <summary>
/// Converts Celsius to Fahrenheit.
/// </summary>
/// <param name="c">Temperature in Celsius.</param>
/// <returns>Temperature in Fahrenheit.</returns>
float DHT::convertCtoF(float c) { return c * 1.8 + 32; }

/// <summary>
/// Converts Fahrenheit to Celsius.
/// </summary>
/// <param name="f">Temperature in Fahrenheit.</param>
/// <returns>Temperature in Celsius.</returns>
float DHT::convertFtoC(float f) { return (f - 32) * 0.55555; }

/// <summary>
/// Reads the humidity from the sensor.
/// </summary>
/// <param name="force">If true, forces a new reading, otherwise uses the previous one.</param>
/// <returns>The humidity value.</returns>
float DHT::readHumidity(bool force) {
  float f = NAN;
  if (read(force)) {
    switch (_type) {
    case DHT11:
    case DHT12:
      f = data[0] + data[1] * 0.1;
      break;
    case DHT22:
    case DHT21:
      f = ((word)data[0]) << 8 | data[1];
      f *= 0.1;
      break;
    }
  }
  return f;
}

/// <summary>
/// Computes the heat index based on the current temperature and humidity.
/// </summary>
/// <param name="isFahrenheit">If true, computes in Fahrenheit, otherwise in Celsius.</param>
/// <returns>The heat index value.</returns>
float DHT::computeHeatIndex(bool isFahrenheit) {
  float hi = computeHeatIndex(readTemperature(isFahrenheit), readHumidity(),
                              isFahrenheit);
  return hi;
}

/// <summary>
/// Computes the heat index from given temperature and humidity values.
/// </summary>
/// <param name="temperature">Temperature value in Celsius or Fahrenheit.</param>
/// <param name="percentHumidity">Humidity percentage.</param>
/// <param name="isFahrenheit">If true, assumes the temperature is in Fahrenheit.</param>
/// <returns>The computed heat index value in the appropriate temperature unit.</returns>
float DHT::computeHeatIndex(float temperature, float percentHumidity,
                            bool isFahrenheit) {
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) +
              (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 + 2.04901523 * temperature + 10.14333127 * percentHumidity +
         -0.22475541 * temperature * percentHumidity +
         -0.00683783 * pow(temperature, 2) +
         -0.05481717 * pow(percentHumidity, 2) +
         0.00122874 * pow(temperature, 2) * percentHumidity +
         0.00085282 * temperature * pow(percentHumidity, 2) +
         -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if ((percentHumidity < 13) && (temperature >= 80.0) &&
        (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) *
            sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if ((percentHumidity > 85.0) && (temperature >= 80.0) &&
             (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}

/// <summary>
/// Reads the sensor data, ensuring that the reading is valid.
/// </summary>
/// <param name="force">If true, forces a new reading, otherwise uses the previous one.</param>
/// <returns>True if the reading is successful, false otherwise.</returns>
bool DHT::read(bool force) {
  uint32_t currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < MIN_INTERVAL)) {
    return _lastresult;
  }
  _lastreadtime = currenttime;

  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

#if defined(ESP8266)
  yield();
#endif

  pinMode(_pin, INPUT_PULLUP);
  delay(1);

  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  switch (_type) {
  case DHT22:
  case DHT21:
    delayMicroseconds(1100);
    break;
  case DHT11:
  default:
    delay(20);
    break;
  }

  uint32_t cycles[80];
  {
    pinMode(_pin, INPUT_PULLUP);

    delayMicroseconds(pullTime);

    InterruptLock lock;

    if (expectPulse(LOW) == TIMEOUT) {
      DEBUG_PRINTLN(F("DHT timeout waiting for start signal low pulse."));
      _lastresult = false;
      return _lastresult;
    }
    if (expectPulse(HIGH) == TIMEOUT) {
      DEBUG_PRINTLN(F("DHT timeout waiting for start signal high pulse."));
      _lastresult = false;
      return _lastresult;
    }

    for (int i = 0; i < 80; i += 2) {
      cycles[i] = expectPulse(LOW);
      cycles[i + 1] = expectPulse(HIGH);
    }
  }

  for (int i = 0; i < 40; ++i) {
    uint32_t lowCycles = cycles[2 * i];
    uint32_t highCycles = cycles[2 * i + 1];
    if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) {
      DEBUG_PRINTLN(F("DHT timeout waiting for pulse."));
      _lastresult = false;
      return _lastresult;
    }
    data[i / 8] <<= 1;
    if (highCycles > lowCycles) {
      data[i / 8] |= 1;
    }
  }

  DEBUG_PRINTLN(F("Received from DHT:"));
  DEBUG_PRINT(data[0], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[1], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[2], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[3], HEX);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[4], HEX);
  DEBUG_PRINT(F("!"));

  if ((data[0] + data[1] + data[2] + data[3]) != data[4]) {
    DEBUG_PRINTLN(F("DHT checksum error"));
    _lastresult = false;
  } else {
    _lastresult = true;
  }

  return _lastresult;
}

/// <summary>
/// Waits for the expected pulse duration.
/// </summary>
/// <param name="level">The expected pulse level (HIGH or LOW).</param>
/// <returns>The duration of the pulse in clock cycles or TIMEOUT if it exceeds max cycles.</returns>
uint32_t DHT::expectPulse(uint8_t level) {
  uint32_t count = 0;
  while (digitalRead(_pin) == level) {
    if (count++ > _maxcycles) {
      return TIMEOUT;
    }
  }
  return count;
}