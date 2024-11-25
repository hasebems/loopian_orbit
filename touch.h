/* ========================================
 *
 *  loopian::ORBIT touch.h
 *    description: TouchEvent
 *    for Raspberry Pi pico
 *
 *  Copyright(c)2024 Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#ifndef TOUCH_H
#define TOUCH_H

#include  "constants.h"

constexpr int NOTHING = -1;
constexpr int COLLATED = -2;  // 照合済

enum NOTE_TYP {
  TYP_NOTE_ON,
  TYP_SLIDE,
  TYP_NOTE_OFF,
  TYP_MAX,
};
/*----------------------------------------------------------------------------*/
//     Struct
/*----------------------------------------------------------------------------*/
constexpr int OFF = 0;
constexpr int NO_TOUCH = -1;
constexpr int HOLD_ERROR = -2;
constexpr int CHATTERING_TIME = 50; //msec
class EachSwitch {
public:
  int _sw;
  int _timeWhenOn;
  EachSwitch(void): _sw(OFF), _timeWhenOn(NO_TOUCH) {}
};
class SwitchEvent {
  EachSwitch _eachSw[MAX_EACH_SENS];
  //int      _sw[MAX_EACH_SENS];
  //int      _timeWhenOn[MAX_EACH_SENS];
  //uint16_t _oldSwEvent[MAX_EACH_SENS];
  uint16_t _oldSwEvent;
public:
  SwitchEvent(void): _eachSw(), _oldSwEvent(0) {}
  int sw(size_t ele) {return _eachSw[ele]._sw;}
  bool sw_on(size_t ele) {return _eachSw[ele]._timeWhenOn >= 0;}
  void clear_event(int time, size_t ele);
  bool update_sw_event(uint8_t sw[2], int time);
};
/*----------------------------------------------------------------------------*/
struct TouchEvent {
  int _locate_current;  // -1, 0 - 9599 (16*6*100 - 1)
  int _locate_target;   // -1, 0 - 9599
  int _mintch_locate;   // -1, 0 - 47 (8*6 - 1)
  int _maxtch_locate;   // -1, 0 - 47
  int _last_midi;       // 0 - 95 (locate/100)
  int _time;
  TouchEvent(void): 
    _locate_current(NOTHING),
    _locate_target(NOTHING),
    _mintch_locate(NOTHING),
    _maxtch_locate(NOTHING),
    _last_midi(NOTHING),
    _time(NOTHING) {}
  TouchEvent& operator=(const TouchEvent& te){
    _locate_current = te._locate_current;
    _locate_target = te._locate_target;
    _mintch_locate = te._mintch_locate;
    _maxtch_locate = te._maxtch_locate;
    _last_midi = te._last_midi;
    _time = te._time;
    return *this;
  }
};
/*----------------------------------------------------------------------------*/
void extract_finger(TouchEvent (&new_ev)[MAX_TOUCH_EV]);
int update_touch_target(SwitchEvent (&se)[MAX_KAMABOKO_NUM]);
void interporate_location(long difftm);
void generate_midi(NOTE_TYP type, int locate, int last_locate);
#endif