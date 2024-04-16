/* ========================================
 *
 *  loopian: white led
 *    description: white led
 *    for Arduino Leonardo / Sparkfun pro micro
 *
 *  Copyright(c)2023- Masahiko Hasebe at Kigakudoh
 *  This software is released under the MIT License, see LICENSE.txt
 *
 * ========================================
 */
#ifndef WHITE_LED_H
#define WHITE_LED_H
 
#include <Arduino.h>
#include "constants.h"

constexpr int FADE_RATE = 2; // *MINIMUM_RESOLUTION[msec] で 3/4

class WhiteLed {

  long  _total_time;   // *2msec
  int   _update_counter;
  int   _fade_counter;
  int   _tch_tgt[MAX_TOUCH_EV];
  int   _tch_crnt[MAX_TOUCH_EV];
  int   _light_lvl[MAX_LIGHT];  // 0-198
  int   _light_lvl_itp[MAX_LIGHT];

public:
  WhiteLed(void): _total_time(0), _update_counter(0), _fade_counter(0), _light_lvl(), _light_lvl_itp() {}
  void clear_all(void);
  int gen_lighting_in_loop(long difftm, int (&tchev)[MAX_TOUCH_EV], int (&extkbd)[128]);
  void one_kamaboco(int kamanum, uint16_t time);
  void light_led_each(const int num, const int dev_num, uint16_t strength);
};
#endif