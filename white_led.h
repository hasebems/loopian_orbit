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
#include <new>
#include "constants.h"

class Kamaboco {
  int   _mynum;
  long  _kama_fade_counter;
  int   _kama_crnt_led;
  int   _light_lvl[MAX_EACH_LIGHT];  // 0-198
  int   _light_lvl_itp[MAX_EACH_LIGHT];

public:
  Kamaboco():
    _mynum(0), _kama_fade_counter(), _kama_crnt_led(), _light_lvl(), _light_lvl_itp()
    {}
  void init(int num) {_mynum = num;}
  void clr_light(void);
  void add_light_lvl(int lednum, int lvl);
  void set_light_lvl(int lednum, int lvl);
  void one_kamaboco(long total_time);
  void light_led_each(const int num, uint16_t strength);
};

class WhiteLed {
  long  _total_time;   // *2msec
  int   _crnt_kama;
  int   _tch_tgt[MAX_TOUCH_EV];
  int   _tch_crnt[MAX_TOUCH_EV];
  Kamaboco _kama[MAX_KAMABOKO_NUM];

public:
  WhiteLed(void): _total_time(0), _crnt_kama(0), _tch_tgt(), _tch_crnt(), _kama()  {
    for (int i=0; i<MAX_KAMABOKO_NUM; ++i){_kama[i].init(i);}
  }
  void set_led(int kama, int lednum, uint16_t lvl);
  void clear_all(void);
  int gen_lighting_in_loop(long difftm, int (&tchev)[MAX_TOUCH_EV], int (&extkbd)[128]);
  void lighten_led(void);
};
#endif