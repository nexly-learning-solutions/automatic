#include "Timer.h"

TimerOne Timer1;

unsigned short TimerOne::pwmPeriod = 0;
unsigned char TimerOne::clockSelectBits = 0;
void (*TimerOne::isrCallback)() = TimerOne::isrDefaultUnused;

#if defined (__AVR_ATtiny85__)
ISR(TIMER1_COMPA_vect)
{
  Timer1.isrCallback();
}
#elif defined(__AVR__)
ISR(TIMER1_OVF_vect)
{
  Timer1.isrCallback();
}

#elif defined(__arm__) && defined(CORE_TEENSY)
void ftm1_isr(void)
{
  uint32_t sc = FTM1_SC;
  #ifdef KINETISL
  if (sc & 0x80) FTM1_SC = sc;
  #else
  if (sc & 0x80) FTM1_SC = sc & 0x7F;
  #endif
  Timer1.isrCallback();
}

#endif

void TimerOne::isrDefaultUnused()
{
}
