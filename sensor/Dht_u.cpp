#include "DHT_U.h"
#include "Dht.h"
#include <cstdint>

DHT_Unified::DHT_Unified(uint8_t pin, uint8_t type, uint8_t count,
                         int32_t tempSensorId, int32_t humiditySensorId)
    : _dht(pin, type, count), _type(type), _temp(this, tempSensorId),
      _humidity(this, humiditySensorId) {}

/// <summary>
/// Initializes the DHT sensor by calling the <see cref="DHT::begin()"/> function.
/// </summary>
void DHT_Unified::begin() { _dht.begin(); }

/// <summary>
/// Sets the name of the sensor based on its type.
/// </summary>
/// <param name="sensor">Pointer to the sensor object where the name will be set.</param>
void DHT_Unified::setName(sensor_t *sensor) {
  switch (_type) {
  case DHT11:
    strncpy(sensor->name, "DHT11", sizeof(sensor->name) - 1);
    break;
  case DHT12:
    strncpy(sensor->name, "DHT12", sizeof(sensor->name) - 1);
    break;
  case DHT21:
    strncpy(sensor->name, "DHT21", sizeof(sensor->name) - 1);
    break;
  case DHT22:
    strncpy(sensor->name, "DHT22", sizeof(sensor->name) - 1);
    break;
  default:
    strncpy(sensor->name, "DHT?", sizeof(sensor->name) - 1);
    break;
  }
  sensor->name[sizeof(sensor->name) - 1] = 0;
}

/// <summary>
/// Sets the minimum delay between readings for the sensor based on its type.
/// </summary>
/// <param name="sensor">Pointer to the sensor object where the delay will be set.</param>
void DHT_Unified::setMinDelay(sensor_t *sensor) {
  switch (_type) {
  case DHT11:
    sensor->min_delay = 1000000L;
    break;
  case DHT12:
    sensor->min_delay = 2000000L;
    break;
  case DHT21:
    sensor->min_delay = 2000000L;
    break;
  case DHT22:
    sensor->min_delay = 2000000L;
    break;
  default:
    sensor->min_delay = 2000000L;
    break;
  }
}

/// <summary>
/// Constructor for initializing the Temperature sensor object.
/// </summary>
/// <param name="parent">Pointer to the parent DHT_Unified object.</param>
/// <param name="id">ID for the temperature sensor.</param>
DHT_Unified::Temperature::Temperature(DHT_Unified *parent, int32_t id)
    : _parent(parent), _id(id) {}

/// <summary>
/// Gets a temperature reading event from the sensor.
/// </summary>
/// <param name="event">Pointer to the sensor event where the temperature data will be stored.</param>
/// <returns>true if the event was successfully retrieved; otherwise, false.</returns>
bool DHT_Unified::Temperature::getEvent(sensors_event_t *event) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _id;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = millis();
  event->temperature = _parent->_dht.readTemperature();

  return true;
}

/// <summary>
/// Retrieves sensor metadata for the temperature sensor.
/// </summary>
/// <param name="sensor">Pointer to the sensor object to store the metadata.</param>
void DHT_Unified::Temperature::getSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  _parent->setName(sensor);
  sensor->version = DHT_SENSOR_VERSION;
  sensor->sensor_id = _id;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  _parent->setMinDelay(sensor);
  switch (_parent->_type) {
  case DHT11:
    sensor->max_value = 50.0F;
    sensor->min_value = 0.0F;
    sensor->resolution = 2.0F;
    break;
  case DHT12:
    sensor->max_value = 60.0F;
    sensor->min_value = -20.0F;
    sensor->resolution = 0.5F;
    break;
  case DHT21:
    sensor->max_value = 80.0F;
    sensor->min_value = -40.0F;
    sensor->resolution = 0.1F;
    break;
  case DHT22:
    sensor->max_value = 125.0F;
    sensor->min_value = -40.0F;
    sensor->resolution = 0.1F;
    break;
  default:
    sensor->max_value = 0.0F;
    sensor->min_value = 0.0F;
    sensor->resolution = 0.0F;
    break;
  }
}

/// <summary>
/// Constructor for initializing the Humidity sensor object.
/// </summary>
/// <param name="parent">Pointer to the parent DHT_Unified object.</param>
/// <param name="id">ID for the humidity sensor.</param>
DHT_Unified::Humidity::Humidity(DHT_Unified *parent, int32_t id)
    : _parent(parent), _id(id) {}

/// <summary>
/// Gets a humidity reading event from the sensor.
/// </summary>
/// <param name="event">Pointer to the sensor event where the humidity data will be stored.</param>
/// <returns>true if the event was successfully retrieved; otherwise, false.</returns>
bool DHT_Unified::Humidity::getEvent(sensors_event_t *event) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _id;
  event->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  event->timestamp = millis();
  event->relative_humidity = _parent->_dht.readHumidity();

  return true;
}

/// <summary>
/// Retrieves sensor metadata for the humidity sensor.
/// </summary>
/// <param name="sensor">Pointer to the sensor object to store the metadata.</param>
void DHT_Unified::Humidity::getSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  _parent->setName(sensor);
  sensor->version = DHT_SENSOR_VERSION;
  sensor->sensor_id = _id;
  sensor->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  _parent->setMinDelay(sensor);
  switch (_parent->_type) {
  case DHT11:
    sensor->max_value = 80.0F;
    sensor->min_value = 20.0F;
    sensor->resolution = 5.0F;
    break;
  case DHT12:
    sensor->max_value = 95.0F;
    sensor->min_value = 20.0F;
    sensor->resolution = 5.0F;
    break;
  case DHT21:
    sensor->max_value = 100.0F;
    sensor->min_value = 0.0F;
    sensor->resolution = 0.1F;
    break;
  case DHT22:
    sensor->max_value = 100.0F;
    sensor->min_value = 0.0F;
    sensor->resolution = 0.1F;
    break;
  default:
    sensor->max_value = 0.0F;
    sensor->min_value = 0.0F;
    sensor->resolution = 0.0F;
    break;
  }
}