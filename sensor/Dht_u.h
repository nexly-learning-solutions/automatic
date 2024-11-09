#ifndef DHT_U_H
#define DHT_U_H

#include <Adafruit_Sensor.h>
#include <DHT.h>

#define DHT_SENSOR_VERSION 1

/// <summary>
/// A unified class for interfacing with DHT sensors.
/// This class provides access to temperature and humidity readings from a DHT sensor.
/// </summary>
class DHT_Unified {
public:
  /// <summary>
  /// Constructor for initializing a DHT_Unified object.
  /// </summary>
  /// <param name="pin">The pin connected to the DHT sensor.</param>
  /// <param name="type">The type of the DHT sensor (DHT11, DHT22, etc.).</param>
  /// <param name="count">The number of readings to average (default is 6).</param>
  /// <param name="tempSensorId">The ID for the temperature sensor (default is -1).</param>
  /// <param name="humiditySensorId">The ID for the humidity sensor (default is -1).</param>
  DHT_Unified(uint8_t pin, uint8_t type, uint8_t count = 6,
              int32_t tempSensorId = -1, int32_t humiditySensorId = -1);

  /// <summary>
  /// Initializes the sensor for use.
  /// </summary>
  void begin();

  /// <summary>
  /// Class for reading temperature values from the DHT sensor.
  /// Inherits from Adafruit_Sensor to provide sensor event and information.
  /// </summary>
  class Temperature : public Adafruit_Sensor {
  public:
    /// <summary>
    /// Constructor for initializing the Temperature sensor.
    /// </summary>
    /// <param name="parent">The parent DHT_Unified object.</param>
    /// <param name="id">The unique ID for the temperature sensor.</param>
    Temperature(DHT_Unified *parent, int32_t id);

    /// <summary>
    /// Retrieves a temperature event from the sensor.
    /// </summary>
    /// <param name="event">A pointer to the event structure where the data will be stored.</param>
    /// <returns>true if the event was successfully retrieved, false otherwise.</returns>
    bool getEvent(sensors_event_t *event);

    /// <summary>
    /// Retrieves information about the temperature sensor.
    /// </summary>
    /// <param name="sensor">A pointer to the sensor structure where the sensor details will be stored.</param>
    void getSensor(sensor_t *sensor);

  private:
    DHT_Unified *_parent;     // The parent DHT_Unified object.
    int32_t _id;              // The unique ID of the temperature sensor.
  };

  /// <summary>
  /// Class for reading humidity values from the DHT sensor.
  /// Inherits from Adafruit_Sensor to provide sensor event and information.
  /// </summary>
  class Humidity : public Adafruit_Sensor {
  public:
    /// <summary>
    /// Constructor for initializing the Humidity sensor.
    /// </summary>
    /// <param name="parent">The parent DHT_Unified object.</param>
    /// <param name="id">The unique ID for the humidity sensor.</param>
    Humidity(DHT_Unified *parent, int32_t id);

    /// <summary>
    /// Retrieves a humidity event from the sensor.
    /// </summary>
    /// <param name="event">A pointer to the event structure where the data will be stored.</param>
    /// <returns>true if the event was successfully retrieved, false otherwise.</returns>
    bool getEvent(sensors_event_t *event);

    /// <summary>
    /// Retrieves information about the humidity sensor.
    /// </summary>
    /// <param name="sensor">A pointer to the sensor structure where the sensor details will be stored.</param>
    void getSensor(sensor_t *sensor);

  private:
    DHT_Unified *_parent;       // The parent DHT_Unified object.
    int32_t _id;                // The unique ID of the humidity sensor.
  };

  /// <summary>
  /// Returns the Temperature object.
  /// </summary>
  /// <returns>The Temperature object for accessing temperature readings.</returns>
  Temperature temperature() { return _temp; }

  /// <summary>
  /// Returns the Humidity object.
  /// </summary>
  /// <returns>The Humidity object for accessing humidity readings.</returns>
  Humidity humidity() { return _humidity; }

private:
  DHT _dht;                 // The DHT sensor object.
  uint8_t _type;            // The type of DHT sensor.
  Temperature _temp;        // The Temperature object.
  Humidity _humidity;       // The Humidity object.

  /// <summary>
  /// Sets the name of the sensor.
  /// </summary>
  /// <param name="sensor">A pointer to the sensor structure to be named.</param>
  void setName(sensor_t *sensor);

  /// <summary>
  /// Sets the minimum delay between sensor readings.
  /// </summary>
  /// <param name="sensor">A pointer to the sensor structure to be configured.</param>
  void setMinDelay(sensor_t *sensor);
};

#endif