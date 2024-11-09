
#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>
#include <HardwareSerial.h>


#define _SS_MAX_RX_BUFF 64
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1052__) || defined(__IMXRT1062__)

class SoftwareSerial : public Stream
{
public:
	SoftwareSerial(uint8_t rxPin, uint8_t txPin, bool inverse_logic = false);
	~SoftwareSerial() { end(); }
	void begin(unsigned long speed);
	void end();
	bool listen() { return true; }
	bool isListening() { return true; }
	bool overflow() { bool ret = buffer_overflow; buffer_overflow = false; return ret; }
	virtual int available();
	virtual int read();
	int peek();
	virtual void flush();
	virtual size_t write(uint8_t byte);
	using Print::write;
private:
	HardwareSerial *port;
	uint32_t cycles_per_bit;
	#if defined(__IMXRT1052__) || defined(__IMXRT1062__)
	volatile uint32_t *txreg;
	volatile uint32_t *rxreg;
	#else
	volatile uint8_t *txreg;
	volatile uint8_t *rxreg;
	#endif
	bool buffer_overflow;
	uint8_t txpin;
	uint8_t rxpin;
};

#else
class SoftwareSerial : public Stream
{
private:
  uint8_t _receivePin;
  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;

  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;

  uint16_t _buffer_overflow:1;
  uint16_t _inverse_logic:1;

  static char _receive_buffer[_SS_MAX_RX_BUFF]; 
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftwareSerial *active_object;

  void recv();
  uint8_t rx_pin_read();
  void tx_pin_write(uint8_t pin_state);
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);

  static inline void tunedDelay(uint16_t delay);


public:
  SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
  ~SoftwareSerial();
  void begin(long speed);
  bool listen();
  void end();
  bool isListening() { return this == active_object; }
  bool overflow() { bool ret = _buffer_overflow; _buffer_overflow = false; return ret; }
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();
  
  using Print::write;

  static inline void handle_interrupt();
};

#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif

#endif
