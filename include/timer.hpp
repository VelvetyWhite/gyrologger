#ifndef __timer__
#define __timer__

#include "pico/time.h"

class Timer {
 public:
  Timer();

  void start();
  void stop();
  void pause();
  void unpause();

  absolute_time_t getTicks() const;

  bool isStarted();
  bool isPaused();

 private:
  absolute_time_t m_startTicks;
  absolute_time_t m_pausedTicks;

  bool m_paused;
  bool m_started;
};

#endif