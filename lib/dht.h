
#ifndef dht_h
#define dht_h

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#define DHT_LIB_VERSION "0.1.14"

#define DHTLIB_OK                0
#define DHTLIB_ERROR_CHECKSUM   -1
#define DHTLIB_ERROR_TIMEOUT    -2
#define DHTLIB_INVALID_VALUE    -999

#define DHTLIB_DHT11_WAKEUP     18
#define DHTLIB_DHT_WAKEUP       1

#define DHTLIB_TIMEOUT (F_CPU/40000)

class dht
{
public:
    int read11(uint8_t pin);
    int read(uint8_t pin);

    inline int read21(uint8_t pin) { return read(pin); };
    inline int read22(uint8_t pin) { return read(pin); };
    inline int read33(uint8_t pin) { return read(pin); };
    inline int read44(uint8_t pin) { return read(pin); };

    double humidity;
    double temperature;

private:
    uint8_t bits[5];
    int _readSensor(uint8_t pin, uint8_t wakeupDelay);
};
#endif
