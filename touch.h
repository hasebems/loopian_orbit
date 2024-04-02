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
constexpr size_t MAX_ELECTRODE_PER_DEV = 8;

/*----------------------------------------------------------------------------*/
//     Struct
/*----------------------------------------------------------------------------*/
constexpr int OFF = 0;
constexpr int CHATTERING_TIME = 50; //msec
class SwitchEvent {
  int _sw[MAX_ELECTRODE_PER_DEV];
public:
  SwitchEvent(void): _sw{OFF} {}
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
int update_touch_target(void);
void interporate_location(long difftm);
void generate_midi(int type, int locate, int last_locate);
#endif