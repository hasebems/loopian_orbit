/* ========================================
 *
 *  TouchMIDI Common Platform for AVR
 *  global_timer.h
 *    description: Global Timer Functions 
 *
 *  Copyright(c)2019- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#ifndef GLOBAL_TIMER_H
#define GLOBAL_TIMER_H
 
#include <Arduino.h>

constexpr int MINIMUM_RESOLUTION = 2; // 2msec

class GlobalTimer {

public:
  GlobalTimer( void ) : _globalTime(0), _gtOld(0), _timer100msec(0), _timer100msec_sabun(0),
                        _timer1sec(0), _timer1sec_sabun(0), _timer10msec_event(false),
                        _timer100msec_event(false), _timer1sec_event(false) {}

  void      setGlobalTime( uint16_t tm ){ _globalTime = tm;}
  void      incGlobalTime( void ){ _globalTime++;}
  uint16_t  globalTime( void ) const { return _globalTime;}
  void      setGtOld( uint16_t tm ){ _gtOld = tm;}
  uint16_t  gtOld( void ) const { return _gtOld;}
  void      setTimer100ms( uint16_t tm ){ _timer100msec = tm;}

  uint32_t  timer10ms( void ) const { return _timer10msec;}
  uint32_t  timer100ms( void ) const { return _timer100msec;}
  uint32_t  timer1s( void ) const { return _timer1sec;}

  void      clearAllTimerEvent( void ){ _timer10msec_event = _timer100msec_event = _timer1sec_event = false;}
  bool      timer10msecEvent( void ) const { return _timer10msec_event;}
  bool      timer100msecEvent( void ) const { return _timer100msec_event;}
  bool      timer1secEvent( void ) const { return _timer1sec_event;}
  
  void      updateTimer( long diff )
  {
    _timer10msec_sabun += (uint16_t)diff;
    while ( _timer10msec_sabun >= 10/MINIMUM_RESOLUTION ){
      _timer10msec++;
      _timer10msec_event = true;
      _timer10msec_sabun -= 10/MINIMUM_RESOLUTION;
    }

    _timer100msec_sabun += (uint16_t)diff;
    while ( _timer100msec_sabun >= 100/MINIMUM_RESOLUTION ){
      _timer100msec++;
      _timer100msec_event = true;
      _timer100msec_sabun -= 100/MINIMUM_RESOLUTION;
    }
    _timer1sec_sabun += (uint16_t)diff;
    while ( _timer1sec_sabun >= 1000/MINIMUM_RESOLUTION ){
      _timer1sec++;
      _timer1sec_event = true;
      _timer1sec_sabun -= 1000/MINIMUM_RESOLUTION;
    }
  }

  bool      _timer10msec_event;
  bool      _timer100msec_event;
  bool      _timer1sec_event;

private:

  volatile uint16_t  _globalTime;
  uint16_t  _gtOld;
  uint32_t  _timer10msec;   // 最大 4294967295 * 10msec: 497.1日
  uint16_t  _timer10msec_sabun;
  uint32_t  _timer100msec;  // 最大 4294967295 * 100msec: 
  uint16_t  _timer100msec_sabun;
  uint32_t  _timer1sec;     // 最大 4294967295 * 1sec:
  uint16_t  _timer1sec_sabun;
};
#endif