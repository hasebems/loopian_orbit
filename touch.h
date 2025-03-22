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
enum TOUCH_STATE {
  ST_NO_TOUCH,
  ST_TOUCH,
  ST_ERROR,
  ST_MAX,
};
/*----------------------------------------------------------------------------*/
//     Struct
/*----------------------------------------------------------------------------*/
constexpr int OFF = 0;
constexpr int NO_TOUCH = -1;
constexpr int HOLD_ERROR = -2;
constexpr int CHATTERING_TIME = 50; //msec
/*----------------------------------------------------------------------------*/
class EachSwitch {
public:
  bool _sw; // false:off, true:on
  //bool _oldSw; // false:off, true:on
  uint32_t _timeWhenOn; // [msec] 0:off, -1:no touch, -2:error
                        // 最大　0xffffffff :  49.7日
  EachSwitch(void): _sw(false), /*_oldSw(false),*/ _timeWhenOn(NO_TOUCH) {}
};
/*----------------------------------------------------------------------------*/
class SwitchEvent {
  EachSwitch _eachSw[MAX_KAMABOKO_NUM*MAX_EACH_SENS];
public:
  SwitchEvent(void): _eachSw() {}
  bool update_allsw_event(uint32_t time, bool (&available_dev)[MAX_KAMABOKO_NUM]);

  TOUCH_STATE sw(size_t num) const {
    if (_eachSw[num]._timeWhenOn == HOLD_ERROR) {
      return ST_ERROR;
    } else if (_eachSw[num]._sw) {
      return ST_TOUCH;
    } else {
      return ST_NO_TOUCH;
    }
  }
  int duration(size_t num, uint32_t time) const {
    int diff;
    if (_eachSw[num]._timeWhenOn < 0) {
      diff = 0;
    } else {
      diff = time - _eachSw[num]._timeWhenOn;
    }
    return diff;
  }
private:
  bool update_kama_event(uint8_t sw[2], int kama, uint32_t time);
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
bool extract_finger(TouchEvent (&new_ev)[MAX_TOUCH_EV], SwitchEvent &se);
int update_touch_target(SwitchEvent &se, bool &sensor_error);
void interporate_location(long difftm);
void generate_midi(NOTE_TYP type, int locate, int last_locate);
#endif