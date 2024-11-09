
#include "dht.h"
#include <cstdint>


int dht::read11(uint8_t pin)
{
    int rv = _readSensor(pin, DHTLIB_DHT11_WAKEUP);
    if (rv != DHTLIB_OK)
    {
        humidity    = DHTLIB_INVALID_VALUE;
        temperature = DHTLIB_INVALID_VALUE;
        return rv;
    }

    humidity    = bits[0];
    temperature = bits[2];

    uint8_t sum = bits[0] + bits[2];
    if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;

    return DHTLIB_OK;
}


int dht::read(uint8_t pin)
{
    int rv = _readSensor(pin, DHTLIB_DHT_WAKEUP);
    if (rv != DHTLIB_OK)
    {
        humidity    = DHTLIB_INVALID_VALUE;
        temperature = DHTLIB_INVALID_VALUE;
        return rv;
    }

    humidity = word(bits[0], bits[1]) * 0.1;
    temperature = word(bits[2] & 0x7F, bits[3]) * 0.1;
    if (bits[2] & 0x80)
    {
        temperature = -temperature;
    }

    uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
    if (bits[4] != sum)
    {
        return DHTLIB_ERROR_CHECKSUM;
    }
    return DHTLIB_OK;
}


int dht::_readSensor(uint8_t pin, uint8_t wakeupDelay)
{
    uint8_t mask = 128;
    uint8_t idx = 0;

	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *PIR = portInputRegister(port);

    for (uint8_t i = 0; i < 5; i++) bits[i] = 0;

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delay(wakeupDelay);
    digitalWrite(pin, HIGH);
    delayMicroseconds(40);
    pinMode(pin, INPUT);

    uint16_t loopCntLOW = DHTLIB_TIMEOUT;
    while ((*PIR & bit) == LOW )
    {
        if (--loopCntLOW == 0) return DHTLIB_ERROR_TIMEOUT;
    }

    uint16_t loopCntHIGH = DHTLIB_TIMEOUT;
    while ((*PIR & bit) != LOW )
    {
        if (--loopCntHIGH == 0) return DHTLIB_ERROR_TIMEOUT;
    }

    for (uint8_t i = 40; i != 0; i--)
    {
        loopCntLOW = DHTLIB_TIMEOUT;
        while ((*PIR & bit) == LOW )
        {
            if (--loopCntLOW == 0) return DHTLIB_ERROR_TIMEOUT;
        }

        uint32_t t = micros();

        loopCntHIGH = DHTLIB_TIMEOUT;
        while ((*PIR & bit) != LOW )
        {
            if (--loopCntHIGH == 0) return DHTLIB_ERROR_TIMEOUT;
        }

        if ((micros() - t) > 40)
        { 
            bits[idx] |= mask;
        }
        mask >>= 1;
        if (mask == 0)
        {
            mask = 128;
            idx++;
        }
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);

    return DHTLIB_OK;
}
