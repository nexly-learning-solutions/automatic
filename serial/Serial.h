#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>
#include <HardwareSerial.h>

/// <summary>
/// Maximum buffer size for the receive buffer.
/// </summary>
#define _SS_MAX_RX_BUFF 64

#ifndef GCC_VERSION
/// <summary>
/// Defines the GCC version.
/// </summary>
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__IMXRT1052__) || defined(__IMXRT1062__)

/// <summary>
/// SoftwareSerial class for platforms that support optimized hardware features.
/// </summary>
class SoftwareSerial : public Stream
{
public:
  /// <summary>
  /// Constructor for SoftwareSerial class, initializes the RX and TX pins.
  /// </summary>
  /// <param name="rxPin">The pin used for receiving data.</param>
  /// <param name="txPin">The pin used for transmitting data.</param>
  /// <param name="inverse_logic">If true, inverts logic level (default is false).</param>
  SoftwareSerial(uint8_t rxPin, uint8_t txPin, bool inverse_logic = false);

  /// <summary>
  /// Destructor for SoftwareSerial class.
  /// </summary>
  ~SoftwareSerial() { end(); }

  /// <summary>
  /// Initializes the serial communication at the given speed.
  /// </summary>
  /// <param name="speed">The communication speed in baud rate.</param>
  void begin(unsigned long speed);

  /// <summary>
  /// Ends the serial communication.
  /// </summary>
  void end();

  /// <summary>
  /// Begins listening to the software serial.
  /// </summary>
  /// <returns>Always returns true.</returns>
  bool listen() { return true; }

  /// <summary>
  /// Checks if the software serial is currently listening.
  /// </summary>
  /// <returns>Always returns true.</returns>
  bool isListening() { return true; }

  /// <summary>
  /// Checks if a buffer overflow occurred.
  /// </summary>
  /// <returns>True if overflow occurred, false otherwise.</returns>
  bool overflow() { bool ret = buffer_overflow; buffer_overflow = false; return ret; }

  /// <summary>
  /// Returns the number of bytes available for reading.
  /// </summary>
  /// <returns>The number of available bytes.</returns>
  virtual int available();

  /// <summary>
  /// Reads a byte from the serial buffer.
  /// </summary>
  /// <returns>The byte read from the serial buffer.</returns>
  virtual int read();

  /// <summary>
  /// Returns the next byte without removing it from the buffer.
  /// </summary>
  /// <returns>The next byte to read.</returns>
  int peek();

  /// <summary>
  /// Clears the transmit buffer.
  /// </summary>
  virtual void flush();

  /// <summary>
  /// Writes a byte to the serial port.
  /// </summary>
  /// <param name="byte">The byte to write.</param>
  /// <returns>The number of bytes written.</returns>
  virtual size_t write(uint8_t byte);

  using Print::write;

private:
  HardwareSerial *port;         // The hardware serial port used.
  uint32_t cycles_per_bit;      // The number of cycles per bit for timing.
  
  /// <summary>
  /// Pointer to the TX register for sending data.
  /// </summary>
  #if defined(__IMXRT1052__) || defined(__IMXRT1062__)
  volatile uint32_t *txreg;
  volatile uint32_t *rxreg;
  #else
  volatile uint8_t *txreg;
  volatile uint8_t *rxreg;
  #endif

  bool buffer_overflow;     // Indicates if a buffer overflow occurred.
  uint8_t txpin;            // The pin used for transmission.
  uint8_t rxpin;            // The pin used for reception.
};

#else
/// <summary>
/// SoftwareSerial class for platforms without optimized hardware features.
/// </summary>
class SoftwareSerial : public Stream
{
private:
  uint8_t _receivePin;                              // Pin for receiving data.
  uint8_t _receiveBitMask;                          // Bitmask for the receive pin.
  volatile uint8_t *_receivePortRegister;           // Register for the receive pin.
  uint8_t _transmitBitMask;                         // Bitmask for the transmit pin.
  volatile uint8_t *_transmitPortRegister;          // Register for the transmit pin.
  uint16_t _rx_delay_centering;                     // Delay for centering the RX signal.
  uint16_t _rx_delay_intrabit;                      // Delay for intra-bit timing.
  uint16_t _rx_delay_stopbit;                       // Delay for the stop bit.
  uint16_t _tx_delay;                               // Delay for the transmit signal.
  uint16_t _buffer_overflow:1;                      // Flag for buffer overflow.
  uint16_t _inverse_logic:1;                        // Flag for inverse logic.
  static char _receive_buffer[_SS_MAX_RX_BUFF];     // Static receive buffer.
  static volatile uint8_t _receive_buffer_tail;     // Tail index of the receive buffer.
  static volatile uint8_t _receive_buffer_head;     // Head index of the receive buffer.
  static SoftwareSerial *active_object;             // Active object pointer.

  /// <summary>
  /// Receives incoming data and stores it in the buffer.
  /// </summary>
  void recv();

  /// <summary>
  /// Reads the RX pin.
  /// </summary>
  /// <returns>The state of the RX pin.</returns>
  uint8_t rx_pin_read();

  /// <summary>
  /// Writes to the TX pin.
  /// </summary>
  /// <param name="pin_state">The state to write to the TX pin.</param>
  void tx_pin_write(uint8_t pin_state);

  /// <summary>
  /// Sets the TX pin.
  /// </summary>
  /// <param name="transmitPin">The pin number for transmission.</param>
  void setTX(uint8_t transmitPin);

  /// <summary>
  /// Sets the RX pin.
  /// </summary>
  /// <param name="receivePin">The pin number for reception.</param>
  void setRX(uint8_t receivePin);

  /// <summary>
  /// Delays for a specified time with precision.
  /// </summary>
  /// <param name="delay">The delay in microseconds.</param>
  static inline void tunedDelay(uint16_t delay);

public:
  /// <summary>
  /// Constructor for SoftwareSerial class, initializes the RX and TX pins.
  /// </summary>
  /// <param name="receivePin">The pin used for receiving data.</param>
  /// <param name="transmitPin">The pin used for transmitting data.</param>
  /// <param name="inverse_logic">If true, inverts logic level (default is false).</param>
  SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);

  /// <summary>
  /// Destructor for SoftwareSerial class.
  /// </summary>
  ~SoftwareSerial();

  /// <summary>
  /// Initializes the serial communication at the given speed.
  /// </summary>
  /// <param name="speed">The communication speed in baud rate.</param>
  void begin(long speed);

  /// <summary>
  /// Begins listening to the software serial.
  /// </summary>
  /// <returns>True if listening, false otherwise.</returns>
  bool listen();

  /// <summary>
  /// Ends the serial communication.
  /// </summary>
  void end();

  /// <summary>
  /// Checks if the software serial is currently listening.
  /// </summary>
  /// <returns>True if listening, false otherwise.</returns>
  bool isListening() { return this == active_object; }

  /// <summary>
  /// Checks if a buffer overflow occurred.
  /// </summary>
  /// <returns>True if overflow occurred, false otherwise.</returns>
  bool overflow() { bool ret = _buffer_overflow; _buffer_overflow = false; return ret; }

  /// <summary>
  /// Returns the next byte without removing it from the buffer.
  /// </summary>
  /// <returns>The next byte to read.</returns>
  int peek();

  /// <summary>
  /// Writes a byte to the serial port.
  /// </summary>
  /// <param name="byte">The byte to write.</param>
  /// <returns>The number of bytes written.</returns>
  virtual size_t write(uint8_t byte);

  /// <summary>
  /// Reads a byte from the serial buffer.
  /// </summary>
  /// <returns>The byte read from the buffer.</returns>
  virtual int read();

  /// <summary>
  /// Returns the number of bytes available to read.
  /// </summary>
  /// <returns>The number of available bytes.</returns>
  virtual int available();

  /// <summary>
  /// Clears the transmit buffer.
  /// </summary>
  virtual void flush();

  using Print::write;

  /// <summary>
  /// Handles interrupts for receiving data.
  /// </summary>
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