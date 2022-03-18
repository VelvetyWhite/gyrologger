#include "timer.hpp"

Timer::Timer() {
  m_startTicks = 0;
  m_pausedTicks = 0;
  m_started = false;
  m_paused = false;
}
void Timer::start() {
  m_started = true;
  m_paused = false;

  m_startTicks = get_absolute_time();
  m_pausedTicks = 0;
}
void Timer::stop() {
  m_started = false;
  m_paused = false;
  m_startTicks = 0;
  m_pausedTicks = 0;
}
void Timer::pause() {
  if (m_started && !m_paused) {
    m_paused = true;
    m_pausedTicks = get_absolute_time() - m_startTicks;
    m_startTicks = 0;
  }
}
void Timer::unpause() {
  if (m_started && m_paused) {
    m_paused = false;
    m_startTicks = get_absolute_time() - m_pausedTicks;
    m_pausedTicks = 0;
  }
}
uint64_t Timer::getTicks() const {
  absolute_time_t time = 0;
  if (m_started) {
    if (m_paused)
      time = m_pausedTicks;
    else
      time = get_absolute_time() - m_startTicks;
  }
  return time;
}
bool Timer::isStarted() { return m_started; }
bool Timer::isPaused() { return m_paused && m_started; }